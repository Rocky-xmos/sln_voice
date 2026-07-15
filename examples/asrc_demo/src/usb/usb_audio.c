// Copyright 2022-2026 XMOS LIMITED.
// This Software is subject to the terms of the XMOS Public Licence: Version 1.
/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2020 Reinhard Panhuber
 * Copyright (c) 2021 XMOS LIMITED
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */
#define DEBUG_UNIT USB_AUDIO
#ifndef DEBUG_PRINT_ENABLE_USB_AUDIO
#define DEBUG_PRINT_ENABLE_USB_AUDIO 0
#endif

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <rtos_printf.h>

#include "FreeRTOS.h"
#include "stream_buffer.h"

#include <xcore/assert.h>
#include <xcore/port.h>

#include "usb_descriptors.h"
#include "tusb.h"

#include "rtos_intertile.h"

#include "app_conf.h"

#include "usb_audio.h"
#include "dbcalc.h"

// Audio controls
// Current states

uint32_t sampFreq;
uint8_t clkValid;

// Range states
audio_control_range_4_n_t(1) sampleFreqRng;                                     // Sample frequency range state

static volatile bool mic_interface_open = false;
static volatile bool spkr_interface_open = false;

static StreamBufferHandle_t samples_to_host_stream_buf;
static StreamBufferHandle_t samples_from_host_stream_buf;
static StreamBufferHandle_t rx_buffer;
static TaskHandle_t usb_audio_out_task_handle;
static uint32_t samples_to_host_stream_buf_size_bytes = 0;
static uint32_t usb_audio_spkr_set_itf_count = 0;
static uint32_t usb_audio_rx_frame_count = 0;
static uint32_t usb_audio_out_task_wake_count = 0;

// GPO related code for setting host active GPO pin
#define USER_ACTIVE_GPO_PIN (4)
static port_t host_active_gpo_port = PORT_GPO;

static inline void SetUserHostActive()
{
    uint32_t port_val = port_peek(host_active_gpo_port);
    int bit = USER_ACTIVE_GPO_PIN;

    port_val |= ((unsigned)1 << bit);

    port_out(host_active_gpo_port, port_val);
    return;
}

static inline void ClearUserHostActive()
{
    uint32_t port_val = port_peek(host_active_gpo_port);
    int bit = USER_ACTIVE_GPO_PIN;

    port_val &= ~((unsigned)1 << bit);

    port_out(host_active_gpo_port, port_val);
    return;
}

static inline void UserHostActive_GPO_Init()
{
    // Inititalise host active GPO port
    port_enable(host_active_gpo_port);
    ClearUserHostActive(); // Turn host active GPO pin off by default
}


void XUD_UserSuspend(void) __attribute__ ((weak));
void XUD_UserSuspend(void)
{
    ClearUserHostActive();
}

void XUD_UserResume(void) __attribute__ ((weak));
void XUD_UserResume(void)
{
    SetUserHostActive();
}

//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted
void tud_mount_cb(void)
{
    SetUserHostActive();
    rtos_printf("USB mounted\n");
}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
    ClearUserHostActive();
    rtos_printf("USB unmounted\n");
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
    (void)remote_wakeup_en;
    xassert(false);
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
}

//--------------------------------------------------------------------+
// Volume control
//--------------------------------------------------------------------+
// These are used by the dbtomult and fixed point volume scaling calcs
#define USB_AUDIO_VOL_MUL_FRAC_BITS     29
#define USB_AUDIO_VOLUME_FRAC_BITS      8

// Volume feature unit range in decibels
// These are stored in 8.8 values in decibels. Max volume is 0dB which is 1.0 gain because we only attenuate.
#define USB_AUDIO_MIN_VOLUME_DB     ((int16_t)-60  << USB_AUDIO_VOLUME_FRAC_BITS)
#define USB_AUDIO_MAX_VOLUME_DB     ((int16_t)0  << USB_AUDIO_VOLUME_FRAC_BITS)
#define USB_AUDIO_VOLUME_STEP_DB    ((int16_t)1  << USB_AUDIO_VOLUME_FRAC_BITS)

static bool mute_d2h[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX + 1] = {0};                         // +1 for master channel 0
static bool mute_h2d[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX + 1] = {0};                         // +1 for master channel 0
static int16_t volume_d2h[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX + 1] = {0};                    // +1 for master channel 0. These are dB val in 8.8
static int16_t volume_h2d[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX + 1] = {0};                    // +1 for master channel 0
static uint32_t vol_mul_d2h[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX] = {0};                      // No +1 because master channel is included already. These are the volume scaling vals
static uint32_t vol_mul_h2d[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX] = {0};                      // No +1 because master channel is included already


static void update_vol_mul(const unsigned chan, const unsigned num_audio_chan, const int16_t volumes[], const bool mutes[], uint32_t vol_muls[])
{
    // Add dB values to master (which means cascade multipliers using log rules)
    if(chan > 0)
    {
        // Update individuals
        int32_t db_val_frac = volumes[chan];     // Sign extend to 32b
        db_val_frac += volumes[0];               // cacade master gain
        uint32_t vol_mul = db_to_mult(db_val_frac, USB_AUDIO_VOLUME_FRAC_BITS, USB_AUDIO_VOL_MUL_FRAC_BITS);
        if(mutes[chan] || mutes[0]) // mute if individual or master
        {
            vol_muls[chan - 1] = 0;
        }
        else
        {
            vol_muls[chan - 1] = vol_mul;
        }
    }
    else
    {
        // Update both with new master settings
        for(int i = 0; i < num_audio_chan; i++)
        {
            int32_t db_val_frac = volumes[0];    // Sign extend master to 32b
            db_val_frac += volumes[i + 1];       // cacade idividual gains
            uint32_t vol_mul = db_to_mult(db_val_frac, USB_AUDIO_VOLUME_FRAC_BITS, USB_AUDIO_VOL_MUL_FRAC_BITS);
            bool mute = mutes[i + 1] || mutes[0];  // mute if individual or master
            vol_muls[i] = mute ? 0 : vol_mul;
        }
    }
}

// Initialise volume multipliers
static void init_volume_multipliers(void)
{
    for(int chan=0; chan<CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX + 1; chan++)
    {
        update_vol_mul(chan, CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX, volume_d2h, mute_d2h, vol_mul_d2h);
    }
    for(int chan=0; chan<CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX + 1; chan++)
    {
        update_vol_mul(chan, CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX, volume_h2d, mute_h2d, vol_mul_h2d);
    }
}

static inline int32_t volume_scale(const uint32_t mul, const int32_t samp){
    int64_t result = (int64_t)samp * (int64_t)mul;
    return (int32_t)(result >> USB_AUDIO_VOL_MUL_FRAC_BITS);
}

//--------------------------------------------------------------------+
// AUDIO Task
//--------------------------------------------------------------------+

#if CFG_TUD_AUDIO_FUNC_1_N_BYTES_PER_SAMPLE_TX == 2
typedef int16_t samp_t;
#elif CFG_TUD_AUDIO_FUNC_1_N_BYTES_PER_SAMPLE_TX == 4
typedef int32_t samp_t;
#else
#error CFG_TUD_AUDIO_FUNC_1_N_BYTES_PER_SAMPLE_TX must be either 2 or 4
#endif


void usb_audio_send(int32_t *frame_buffer_ptr, // buffer containing interleaved samples [samps][ch] format
                    size_t frame_count,
                    size_t num_chans)
{
    samp_t usb_audio_in_frame[appconfAUDIO_PIPELINE_FRAME_ADVANCE][CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX];
#if CFG_TUD_AUDIO_FUNC_1_N_BYTES_PER_SAMPLE_TX == 2
    const int src_32_shift = 16;
#elif CFG_TUD_AUDIO_FUNC_1_N_BYTES_PER_SAMPLE_TX == 4
    const int src_32_shift = 0;
#endif

    memset(usb_audio_in_frame, 0, sizeof(usb_audio_in_frame));
    for (int i = 0; i < frame_count; i++)
    {
        for (int ch = 0; ch < num_chans; ch++)
        {
            usb_audio_in_frame[i][ch] = (volume_scale(vol_mul_d2h[ch % CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX], frame_buffer_ptr[i * num_chans + ch])) >> src_32_shift;
        }
    }
    size_t usb_audio_in_size_bytes = frame_count * num_chans * sizeof(samp_t);
    if (xStreamBufferSpacesAvailable(samples_to_host_stream_buf) >= usb_audio_in_size_bytes)
    {
        xStreamBufferSend(samples_to_host_stream_buf, usb_audio_in_frame, usb_audio_in_size_bytes, 0);
    }
    else
    {
        rtos_printf("lost I2S output samples\n");
    }
}

//
/**
 * @brief Task that forwards USB OUT audio directly to the I2S tile.
 */
void usb_audio_out_task(void *arg)
{
    (void)arg;
#if appconfUSB_AUDIO_CLASS == appconfUSB_AUDIO_CLASS_1
    int32_t usb_audio_out_frame[AUDIO_FRAMES_PER_USB_FRAME][CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX];
#else
    samp_t usb_audio_out_frame[AUDIO_FRAMES_PER_USB_FRAME][CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX];
#endif

    for (;;)
    {
        (void)ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
        usb_audio_out_task_wake_count++;

        size_t bytes_received = 0;
        while (bytes_received < sizeof(usb_audio_out_frame))
        {
            bytes_received += xStreamBufferReceive(samples_from_host_stream_buf,
                                                    &usb_audio_out_frame[0][0] + (bytes_received / sizeof(usb_audio_out_frame[0][0])),
                                                    sizeof(usb_audio_out_frame) - bytes_received,
                                                    0);
        }

            rtos_intertile_tx(
                intertile_i2s_audio_ctx,
                appconfI2S_OUTPUT_SLAVE_PORT,
                usb_audio_out_frame,
                bytes_received);

        if ((usb_audio_out_task_wake_count % 100) == 0)
        {
            rtos_printf("[USB tile %d] usb_audio_out_task wake=%u bytes=%u queued_to_i2s=%u\n",
                        THIS_XCORE_TILE,
                        usb_audio_out_task_wake_count,
                        (unsigned)bytes_received,
                        (unsigned)xStreamBufferBytesAvailable(samples_from_host_stream_buf));
        }
    }
}

//--------------------------------------------------------------------+
// Application Callback API Implementations
//--------------------------------------------------------------------+

// Invoked when audio class specific set request received for an EP
bool tud_audio_set_req_ep_cb(uint8_t rhport,
                             tusb_control_request_t const *p_request,
                             uint8_t *pBuff)
{
    (void)rhport;
    (void)pBuff;

#if appconfUSB_AUDIO_CLASS == appconfUSB_AUDIO_CLASS_1
    uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);

    rtos_printf("UAC1 SET EP req=0x%02x wValue=0x%04x wIndex=0x%04x len=%u\n",
                p_request->bRequest, p_request->wValue, p_request->wIndex, p_request->wLength);

    if ((ctrlSel == AUDIO10_EP_CTRL_SAMPLING_FREQ) &&
        (p_request->bRequest == AUDIO10_CS_REQ_SET_CUR)) {
        TU_VERIFY(p_request->wLength == 3);
        sampFreq = (uint32_t)pBuff[0] | ((uint32_t)pBuff[1] << 8) | ((uint32_t)pBuff[2] << 16);
        return sampFreq == appconfUSB_AUDIO_SAMPLE_RATE;
    }

    return false;
#else
    // We do not support any set range requests here, only current value requests
    TU_VERIFY(p_request->bRequest == AUDIO_CS_REQ_CUR);

    // Page 91 in UAC2 specification
    uint8_t channelNum = TU_U16_LOW(p_request->wValue);
    uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);
    uint8_t ep = TU_U16_LOW(p_request->wIndex);

    (void)channelNum;
    (void)ctrlSel;
    (void)ep;

    return false; // Yet not implemented
#endif
}

// Invoked when audio class specific set request received for an interface
bool tud_audio_set_req_itf_cb(uint8_t rhport,
                              tusb_control_request_t const *p_request,
                              uint8_t *pBuff)
{
    (void)rhport;
    (void)pBuff;

    // We do not support any set range requests here, only current value requests
    TU_VERIFY(p_request->bRequest == AUDIO_CS_REQ_CUR);

    // Page 91 in UAC2 specification
    uint8_t channelNum = TU_U16_LOW(p_request->wValue);
    uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);
    uint8_t itf = TU_U16_LOW(p_request->wIndex);

    (void)channelNum;
    (void)ctrlSel;
    (void)itf;

    return false; // Yet not implemented
}

// Invoked when audio class specific set request received for an entity
bool tud_audio_set_req_entity_cb(uint8_t rhport,
                                 tusb_control_request_t const *p_request,
                                 uint8_t *pBuff)
{
    (void)rhport;

#if appconfUSB_AUDIO_CLASS == appconfUSB_AUDIO_CLASS_1
    uint8_t channelNum = TU_U16_LOW(p_request->wValue);
    uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);
    uint8_t entityID = TU_U16_HIGH(p_request->wIndex);

    rtos_printf("UAC1 SET ENTITY req=0x%02x entity=%u ch=%u ctrl=%u len=%u\n",
                p_request->bRequest, entityID, channelNum, ctrlSel, p_request->wLength);

    if (entityID == UAC1_ENTITY_SPK_FEATURE_UNIT) {
        switch (ctrlSel) {
        case AUDIO10_FU_CTRL_MUTE:
            TU_VERIFY(p_request->bRequest == AUDIO10_CS_REQ_SET_CUR);
            TU_VERIFY(p_request->wLength == 1);
            mute_h2d[channelNum] = pBuff[0] != 0;
            update_vol_mul(channelNum, CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX, volume_h2d, mute_h2d, vol_mul_h2d);
            return true;

        case AUDIO10_FU_CTRL_VOLUME:
            // UAC1 Volume is not advertised in the Feature Unit descriptor.
            (void)pBuff;
            return false;

        default:
            TU_BREAKPOINT();
            return false;
        }
    }

    return false;
#else
    // Page 91 in UAC2 specification
    uint8_t channelNum = TU_U16_LOW(p_request->wValue);
    uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);
    uint8_t itf = TU_U16_LOW(p_request->wIndex);
    uint8_t entityID = TU_U16_HIGH(p_request->wIndex);

    (void)itf;

    // We do not support any set range requests here, only current value requests
    TU_VERIFY(p_request->bRequest == AUDIO_CS_REQ_CUR);

    // If request is for our feature unit
    if (entityID == UAC2_ENTITY_MIC_FEATURE_UNIT)
    {
        switch (ctrlSel)
        {
        case AUDIO_FU_CTRL_MUTE:
            // Request uses format layout 1
            TU_VERIFY(p_request->wLength == sizeof(audio_control_cur_1_t));

            mute_d2h[channelNum] = ((audio_control_cur_1_t *)pBuff)->bCur;
            update_vol_mul(channelNum, CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX, volume_d2h, mute_d2h, vol_mul_d2h);

            TU_LOG2("    Set Mute: %d of channel: %u\r\n", mute[channelNum], channelNum);

            return true;

        case AUDIO_FU_CTRL_VOLUME:
            // Request uses format layout 2
            TU_VERIFY(p_request->wLength == sizeof(audio_control_cur_2_t));

            volume_d2h[channelNum] = ((audio_control_cur_2_t *)pBuff)->bCur;
            update_vol_mul(channelNum, CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX, volume_d2h, mute_d2h, vol_mul_d2h);

            TU_LOG2("    Set Volume: %d dB of channel: %u\r\n", volume[channelNum], channelNum);

            return true;

            // Unknown/Unsupported control
        default:
            TU_BREAKPOINT();
            return false;
        }
    }


    if (entityID == UAC2_ENTITY_SPK_FEATURE_UNIT) {
        switch (ctrlSel) {
        case AUDIO_FU_CTRL_MUTE:
            // Request uses format layout 1
            TU_VERIFY(p_request->wLength == sizeof(audio_control_cur_1_t));

            mute_h2d[channelNum] = ((audio_control_cur_1_t*) pBuff)->bCur;
            update_vol_mul(channelNum, CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX, volume_h2d, mute_h2d, vol_mul_h2d);

            TU_LOG2("    Set Mute: %d of channel: %u\n", mute_h2d[channelNum], channelNum);

            return true;

        case AUDIO_FU_CTRL_VOLUME:
            // Request uses format layout 2
            TU_VERIFY(p_request->wLength == sizeof(audio_control_cur_2_t));

            volume_h2d[channelNum] = ((audio_control_cur_2_t*) pBuff)->bCur;
            update_vol_mul(channelNum, CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX, volume_h2d, mute_h2d, vol_mul_h2d);

            TU_LOG2("    Set Volume: %d dB of channel: %u\n", volume_h2d[channelNum], channelNum);

            return true;

            // Unknown/Unsupported control
        default:
            TU_BREAKPOINT();
            return false;
        }
    }

    return false; // Yet not implemented
#endif
}

// Invoked when audio class specific get request received for an EP
bool tud_audio_get_req_ep_cb(uint8_t rhport,
                             tusb_control_request_t const *p_request)
{
#if appconfUSB_AUDIO_CLASS == appconfUSB_AUDIO_CLASS_1
    uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);

    rtos_printf("UAC1 GET EP req=0x%02x wValue=0x%04x wIndex=0x%04x len=%u\n",
                p_request->bRequest, p_request->wValue, p_request->wIndex, p_request->wLength);

    if ((ctrlSel == AUDIO10_EP_CTRL_SAMPLING_FREQ) &&
        (p_request->bRequest == AUDIO10_CS_REQ_GET_CUR)) {
        uint8_t freq[3] = {
                (uint8_t)(sampFreq & 0xff),
                (uint8_t)((sampFreq >> 8) & 0xff),
                (uint8_t)((sampFreq >> 16) & 0xff),
        };
        return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, freq, sizeof(freq));
    }

    return false;
#else
    (void)rhport;

    // Page 91 in UAC2 specification
    uint8_t channelNum = TU_U16_LOW(p_request->wValue);
    uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);
    uint8_t ep = TU_U16_LOW(p_request->wIndex);

    (void)channelNum;
    (void)ctrlSel;
    (void)ep;

    //	return tud_control_xfer(rhport, p_request, &tmp, 1);

    return false; // Yet not implemented
#endif
}

// Invoked when audio class specific get request received for an interface
bool tud_audio_get_req_itf_cb(uint8_t rhport,
                              tusb_control_request_t const *p_request)
{
    (void)rhport;

    // Page 91 in UAC2 specification
    uint8_t channelNum = TU_U16_LOW(p_request->wValue);
    uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);
    uint8_t itf = TU_U16_LOW(p_request->wIndex);

    (void)channelNum;
    (void)ctrlSel;
    (void)itf;

    return false; // Yet not implemented
}

// Invoked when audio class specific get request received for an entity
bool tud_audio_get_req_entity_cb(uint8_t rhport,
                                 tusb_control_request_t const *p_request)
{
#if appconfUSB_AUDIO_CLASS == appconfUSB_AUDIO_CLASS_1
    uint8_t channelNum = TU_U16_LOW(p_request->wValue);
    uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);
    uint8_t entityID = TU_U16_HIGH(p_request->wIndex);

    rtos_printf("UAC1 GET ENTITY req=0x%02x entity=%u ch=%u ctrl=%u len=%u\n",
                p_request->bRequest, entityID, channelNum, ctrlSel, p_request->wLength);

    if (entityID == UAC1_ENTITY_SPK_FEATURE_UNIT) {
        switch (ctrlSel) {
        case AUDIO10_FU_CTRL_MUTE:
            TU_VERIFY(p_request->bRequest == AUDIO10_CS_REQ_GET_CUR);
            return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &mute_h2d[channelNum], 1);

        case AUDIO10_FU_CTRL_VOLUME:
            // UAC1 Volume is not advertised in the Feature Unit descriptor.
            return false;

        default:
            TU_BREAKPOINT();
            return false;
        }
    }

    return false;
#else
    (void)rhport;

    // Page 91 in UAC2 specification
    uint8_t channelNum = TU_U16_LOW(p_request->wValue);
    uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);
    // uint8_t itf = TU_U16_LOW(p_request->wIndex); 			// Since we have only one audio function implemented, we do not need the itf value
    uint8_t entityID = TU_U16_HIGH(p_request->wIndex);

    // Input terminal (Microphone input)
    if (entityID == UAC2_ENTITY_MIC_INPUT_TERMINAL)
    {
        switch (ctrlSel)
        {
        case AUDIO_TE_CTRL_CONNECTOR:;
            // The terminal connector control only has a get request with only the CUR attribute.

            audio_desc_channel_cluster_t ret;

            // Those are dummy values for now
            ret.bNrChannels = CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX;
            ret.bmChannelConfig = 0;
            ret.iChannelNames = 0;

            TU_LOG2("    Get terminal connector\r\n");
            rtos_printf("Get terminal connector\r\n");

            return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, (void *)&ret, sizeof(ret));

            // Unknown/Unsupported control selector
        default:
            TU_BREAKPOINT();
            return false;
        }
    }

    // Feature unit
    if (entityID == UAC2_ENTITY_MIC_FEATURE_UNIT)
    {
        switch (ctrlSel)
        {
        case AUDIO_FU_CTRL_MUTE:
            // TODO Mute control not yet implemented
            // Audio control mute cur parameter block consists of only one byte - we thus can send it right away
            // There does not exist a range parameter block for mute
            TU_LOG2("    Get Mute of channel: %u\r\n", channelNum);
            return tud_control_xfer(rhport, p_request, &mute_d2h[channelNum], 1);

        case AUDIO_FU_CTRL_VOLUME:

            switch (p_request->bRequest)
            {
            case AUDIO_CS_REQ_CUR:
                TU_LOG2("    Get Volume of channel: %u\r\n", channelNum);
                return tud_control_xfer(rhport, p_request, &volume_d2h[channelNum], sizeof(volume_d2h[channelNum]));
            case AUDIO_CS_REQ_RANGE:
                TU_LOG2("    Get Volume range of channel: %u\r\n", channelNum);
                // TODO Volume control not yet implemented
                audio_control_range_2_n_t(1) ret;

                ret.wNumSubRanges = 1;
                ret.subrange[0].bMin = USB_AUDIO_MIN_VOLUME_DB;
                ret.subrange[0].bMax = USB_AUDIO_MAX_VOLUME_DB;
                ret.subrange[0].bRes = USB_AUDIO_VOLUME_STEP_DB;

                return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, (void *)&ret, sizeof(ret));

                // Unknown/Unsupported control
            default:
                TU_BREAKPOINT();
                return false;
            }

            // Unknown/Unsupported control
        default:
            TU_BREAKPOINT();
            return false;
        }
    }

    if (entityID == UAC2_ENTITY_SPK_FEATURE_UNIT) {
        switch (ctrlSel) {
        case AUDIO_FU_CTRL_MUTE:
            // Audio control mute cur parameter block consists of only one byte - we thus can send it right away
            // There does not exist a range parameter block for mute
            TU_LOG2("    Get Mute of channel: %u\r\n", channelNum);
            return tud_control_xfer(rhport, p_request, &mute_h2d[channelNum], 1);

        case AUDIO_FU_CTRL_VOLUME:

            switch (p_request->bRequest) {
            case AUDIO_CS_REQ_CUR:
                TU_LOG2("    Get Volume of channel: %u\r\n", channelNum);
                return tud_control_xfer(rhport, p_request, &volume_h2d[channelNum], sizeof(volume_h2d[channelNum]));
            case AUDIO_CS_REQ_RANGE:
                TU_LOG2("    Get Volume range of channel: %u\r\n", channelNum);

                audio_control_range_2_n_t(1) ret;

                ret.wNumSubRanges = 1;
                ret.subrange[0].bMin = USB_AUDIO_MIN_VOLUME_DB;
                ret.subrange[0].bMax = USB_AUDIO_MAX_VOLUME_DB;
                ret.subrange[0].bRes = USB_AUDIO_VOLUME_STEP_DB;

                return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, (void*) &ret, sizeof(ret));

                // Unknown/Unsupported control
            default:
                TU_BREAKPOINT();
                return false;
            }

            // Unknown/Unsupported control
        default:
            TU_BREAKPOINT();
            return false;
        }
    }

    // Clock Source unit
    if (entityID == UAC2_ENTITY_CLOCK)
    {
        switch (ctrlSel)
        {
        case AUDIO_CS_CTRL_SAM_FREQ:

            // channelNum is always zero in this case

            switch (p_request->bRequest)
            {
            case AUDIO_CS_REQ_CUR:
                TU_LOG2("    Get Sample Freq.\r\n");
                return tud_control_xfer(rhport, p_request, &sampFreq, sizeof(sampFreq));
            case AUDIO_CS_REQ_RANGE:
                TU_LOG2("    Get Sample Freq. range\r\n");
                //((tusb_control_request_t *)p_request)->wLength = 14;
                return tud_control_xfer(rhport, p_request, &sampleFreqRng, sizeof(sampleFreqRng));

                // Unknown/Unsupported control
            default:
                TU_BREAKPOINT();
                return false;
            }

        case AUDIO_CS_CTRL_CLK_VALID:
            // Only cur attribute exists for this request
            TU_LOG2("    Get Sample Freq. valid\r\n");
            return tud_control_xfer(rhport, p_request, &clkValid, sizeof(clkValid));

            // Unknown/Unsupported control
        default:
            TU_BREAKPOINT();
            return false;
        }
    }

    TU_LOG2("  Unsupported entity: %d\r\n", entityID);
    return false; // Yet not implemented
#endif
}

bool tud_audio_rx_done_pre_read_cb(uint8_t rhport,
                                   uint16_t n_bytes_received,
                                   uint8_t func_id,
                                   uint8_t ep_out,
                                   uint8_t cur_alt_setting)
{
    (void)rhport;
    (void)func_id;
    (void)n_bytes_received;
    (void)ep_out;
    (void)cur_alt_setting;

    return true;
}

bool tud_audio_rx_done_post_read_cb(uint8_t rhport,
                                    uint16_t n_bytes_received,
                                    uint8_t func_id,
                                    uint8_t ep_out,
                                    uint8_t cur_alt_setting)
{
    (void)rhport;

    uint8_t rx_data[CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ];
#if appconfUSB_AUDIO_CLASS == appconfUSB_AUDIO_CLASS_1
    int16_t usb_audio_frames_usb[AUDIO_FRAMES_PER_USB_FRAME][CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX];
    int32_t usb_audio_frames_i2s[AUDIO_FRAMES_PER_USB_FRAME][CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX];
    const size_t usb_audio_frame_bytes = sizeof(usb_audio_frames_usb);
    const size_t stream_buffer_send_byte_count = sizeof(usb_audio_frames_i2s);
    uint8_t *usb_audio_frames_bytes = (uint8_t *)usb_audio_frames_usb;
#else
    samp_t usb_audio_frames[AUDIO_FRAMES_PER_USB_FRAME][CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX];
    const size_t usb_audio_frame_bytes = sizeof(usb_audio_frames);
    const size_t stream_buffer_send_byte_count = sizeof(usb_audio_frames);
    uint8_t *usb_audio_frames_bytes = (uint8_t *)usb_audio_frames;
#endif

    if (!spkr_interface_open)
    {
        spkr_interface_open = true;
    }

    /*
     * rx_data is a holding space to recieve the latest USB transaction.
     * If it fits, we then push into rx_buffer. This could be a nominal-size transaction, or it could not be.
     * We then only push nominal size transactions into the stream buffer.
     * Hopefully this doesn't cause timing issues in the pipeline.
     */

    if (sizeof(rx_data) >= n_bytes_received)
    {
        tud_audio_read(rx_data, n_bytes_received);
    }
    else
    {
        /*
         * I don't believe we ever get here because I think the EP FIFO will complain before this,
         * but better safe than sorry
         */
        rtos_printf("Rx'd too much USB data in one transaction, cannot read\n");
        return false;
    }

    if (xStreamBufferSpacesAvailable(rx_buffer) >= n_bytes_received)
    {
        xStreamBufferSend(rx_buffer, rx_data, n_bytes_received, 0);
    }
    else
    {
        rtos_printf("Rx'd too much total USB data, cannot buffer\n");
        return false;
    }

    if (xStreamBufferBytesAvailable(rx_buffer) >= usb_audio_frame_bytes)
    {
        size_t num_rx_total = 0;
        while (num_rx_total < usb_audio_frame_bytes)
        {
            size_t num_rx = xStreamBufferReceive(
                    rx_buffer,
                    usb_audio_frames_bytes + num_rx_total,
                    usb_audio_frame_bytes - num_rx_total,
                    0);
            num_rx_total += num_rx;
        }
    }
    else
    {
        rtos_printf("Not enough data to send to stream buffer, cycling again\n");
        return true;
    }

    // Apply the speaker feature-unit mute/volume state before forwarding to I2S.
#if appconfUSB_AUDIO_CLASS == appconfUSB_AUDIO_CLASS_1
    for (size_t frame = 0; frame < AUDIO_FRAMES_PER_USB_FRAME; frame++)
    {
        for (size_t ch = 0; ch < CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX; ch++)
        {
            int32_t sample_32 = ((int32_t)usb_audio_frames_usb[frame][ch]) << 16;
            usb_audio_frames_i2s[frame][ch] = volume_scale(vol_mul_h2d[ch], sample_32);
        }
    }

    if (xStreamBufferSpacesAvailable(samples_from_host_stream_buf) >= stream_buffer_send_byte_count)
    {
        xStreamBufferSend(samples_from_host_stream_buf, usb_audio_frames_i2s, stream_buffer_send_byte_count, 0);
        if (xStreamBufferBytesAvailable(samples_from_host_stream_buf) >= stream_buffer_send_byte_count)
        {
            xTaskNotifyGive(usb_audio_out_task_handle);
        }

        usb_audio_rx_frame_count++;
        if ((usb_audio_rx_frame_count % 100) == 0)
        {
            rtos_printf("[USB tile %d] speaker rx frame=%u n_bytes=%u alt_open=%d stream_buf=%u\n",
                        THIS_XCORE_TILE,
                        usb_audio_rx_frame_count,
                        (unsigned)n_bytes_received,
                        (int)spkr_interface_open,
                        (unsigned)xStreamBufferBytesAvailable(samples_from_host_stream_buf));
        }
    }
    else
    {
        rtos_printf("lost USB output samples. Space available %d, send_byte_count %d\n", xStreamBufferSpacesAvailable(samples_from_host_stream_buf), stream_buffer_send_byte_count);
    }
#else
    for (size_t frame = 0; frame < AUDIO_FRAMES_PER_USB_FRAME; frame++)
    {
        for (size_t ch = 0; ch < CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX; ch++)
        {
            usb_audio_frames[frame][ch] = volume_scale(vol_mul_h2d[ch], usb_audio_frames[frame][ch]);
        }
    }

    if (xStreamBufferSpacesAvailable(samples_from_host_stream_buf) >= stream_buffer_send_byte_count)
    {
        xStreamBufferSend(samples_from_host_stream_buf, usb_audio_frames, stream_buffer_send_byte_count, 0);
        if (xStreamBufferBytesAvailable(samples_from_host_stream_buf) >= stream_buffer_send_byte_count)
        {
            xTaskNotifyGive(usb_audio_out_task_handle);
        }

        usb_audio_rx_frame_count++;
        if ((usb_audio_rx_frame_count % 100) == 0)
        {
            rtos_printf("[USB tile %d] speaker rx frame=%u n_bytes=%u alt_open=%d stream_buf=%u\n",
                        THIS_XCORE_TILE,
                        usb_audio_rx_frame_count,
                        (unsigned)n_bytes_received,
                        (int)spkr_interface_open,
                        (unsigned)xStreamBufferBytesAvailable(samples_from_host_stream_buf));
        }
    }
    else
    {
        rtos_printf("lost USB output samples. Space available %d, send_byte_count %d\n", xStreamBufferSpacesAvailable(samples_from_host_stream_buf), stream_buffer_send_byte_count);
    }
#endif

    return true;
}

// TinyUSB calls the ISR entry point directly after each OUT packet. Bridge it
// to the existing pre/post processing used by this application.
bool tud_audio_rx_done_isr(uint8_t rhport,
                           uint16_t n_bytes_received,
                           uint8_t func_id,
                           uint8_t ep_out,
                           uint8_t cur_alt_setting)
{
    if (!tud_audio_rx_done_pre_read_cb(rhport,
                                       n_bytes_received,
                                       func_id,
                                       ep_out,
                                       cur_alt_setting))
    {
        return false;
    }

    return tud_audio_rx_done_post_read_cb(rhport,
                                          n_bytes_received,
                                          func_id,
                                          ep_out,
                                          cur_alt_setting);
}

bool tud_audio_tx_done_pre_load_cb(uint8_t rhport,
                                   uint8_t itf,
                                   uint8_t ep_in,
                                   uint8_t cur_alt_setting)
{
    (void)rhport;
    (void)itf;
    (void)ep_in;
    (void)cur_alt_setting;

    size_t tx_size_bytes;

    samp_t stream_buffer_audio_frames[AUDIO_FRAMES_PER_USB_FRAME][CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX];
    samp_t usb_audio_frames[AUDIO_FRAMES_PER_USB_FRAME][CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX];

    tx_size_bytes = sizeof(samp_t) * AUDIO_FRAMES_PER_USB_FRAME * CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX;
    if (!mic_interface_open)
    {
        memset(usb_audio_frames, 0, tx_size_bytes);
        tud_audio_write(usb_audio_frames, tx_size_bytes);
        return true;
    }

    size_t bytes_available = xStreamBufferBytesAvailable(samples_to_host_stream_buf);
    if (bytes_available == 0)
    {
        memset(usb_audio_frames, 0, tx_size_bytes);
        tud_audio_write(usb_audio_frames, tx_size_bytes);
        return true;
    }

    size_t ready_data_bytes = (bytes_available >= tx_size_bytes) ? tx_size_bytes : bytes_available;
    if (ready_data_bytes < tx_size_bytes)
    {
        memset(stream_buffer_audio_frames, 0, tx_size_bytes);
    }

    size_t num_rx_total = 0;
    while (num_rx_total < ready_data_bytes)
    {
        size_t num_rx = xStreamBufferReceive(
                samples_to_host_stream_buf,
                ((uint8_t *)stream_buffer_audio_frames) + num_rx_total,
                ready_data_bytes - num_rx_total,
                0);
        num_rx_total += num_rx;
    }

    tud_audio_write(stream_buffer_audio_frames, tx_size_bytes);

    return true;
}

// TinyUSB calls the ISR entry point after each IN packet. Bridge it to the
// existing callback which fills the next microphone packet.
bool tud_audio_tx_done_isr(uint8_t rhport,
                           uint16_t n_bytes_sent,
                           uint8_t func_id,
                           uint8_t ep_in,
                           uint8_t cur_alt_setting)
{
    (void)n_bytes_sent;

    return tud_audio_tx_done_pre_load_cb(rhport,
                                         func_id,
                                         ep_in,
                                         cur_alt_setting);
}

bool tud_audio_tx_done_post_load_cb(uint8_t rhport,
                                    uint16_t n_bytes_copied,
                                    uint8_t itf,
                                    uint8_t ep_in,
                                    uint8_t cur_alt_setting)
{
    (void)rhport;
    (void)n_bytes_copied;
    (void)itf;
    (void)ep_in;
    (void)cur_alt_setting;

    return true;
}

bool tud_audio_set_itf_cb(uint8_t rhport,
                          tusb_control_request_t const *p_request)
{
    (void)rhport;
    uint8_t const itf = tu_u16_low(tu_le16toh(p_request->wIndex));
    uint8_t const alt = tu_u16_low(tu_le16toh(p_request->wValue));

#if AUDIO_OUTPUT_ENABLED
    if (itf == ITF_NUM_AUDIO_STREAMING_SPK)
    {
        /* In case the interface is reset without
         * closing it first */
        spkr_interface_open = (alt != 0);
        xStreamBufferReset(samples_from_host_stream_buf);
        xStreamBufferReset(rx_buffer);
        usb_audio_spkr_set_itf_count++;
        rtos_printf("[USB tile %d] speaker interface alt=%u open=%d count=%u\n",
                    THIS_XCORE_TILE,
                    alt,
                    (int)spkr_interface_open,
                    usb_audio_spkr_set_itf_count);
    }
#endif
#if AUDIO_INPUT_ENABLED
    if (itf == ITF_NUM_AUDIO_STREAMING_MIC)
    {
        /* In case the interface is reset without
         * closing it first */
        mic_interface_open = (alt != 0);
        xStreamBufferReset(samples_to_host_stream_buf);
    }
#endif

    rtos_printf("Set audio interface %d alt %d\n", itf, alt);

    return true;
}

bool tud_audio_set_itf_close_EP_cb(uint8_t rhport,
                                   tusb_control_request_t const *p_request)
{
    (void)rhport;
    uint8_t const itf = tu_u16_low(tu_le16toh(p_request->wIndex));

#if AUDIO_OUTPUT_ENABLED
    if (itf == ITF_NUM_AUDIO_STREAMING_SPK)
    {
        spkr_interface_open = false;
    }
#endif
#if AUDIO_INPUT_ENABLED
    if (itf == ITF_NUM_AUDIO_STREAMING_MIC)
    {
        mic_interface_open = false;
    }
#endif

    rtos_printf("Close audio interface %d\n", itf);

    return true;
}


// I2S recv -> |to other tile| -> i2s_to_usb_intertile -> usb_audio_send
// This task receives the I2S samples from the I2S tile into the USB tile.
static void i2s_to_usb_intertile(void *args)
{
    (void) args;
    int32_t i2s_to_usb_samps_interleaved[appconfAUDIO_PIPELINE_FRAME_ADVANCE][NUM_I2S_CHANS];


    for(;;)
    {
        size_t bytes_received;
        // Get I2S audio data from the I2S tile to the USB tile
        bytes_received = rtos_intertile_rx_len(
                intertile_i2s_audio_ctx,
                appconfUSB_AUDIO_PORT,
                portMAX_DELAY);

        if (bytes_received > 0) {
            xassert(bytes_received <= sizeof(i2s_to_usb_samps_interleaved));

            rtos_intertile_rx_data(
                    intertile_i2s_audio_ctx,
                    i2s_to_usb_samps_interleaved,
                    bytes_received);

            usb_audio_send(&i2s_to_usb_samps_interleaved[0][0], (bytes_received >> 3), 2);
        }

    }
}

void usb_audio_init(rtos_intertile_t *intertile_ctx,
                    unsigned priority)
{
    // Init values
    sampFreq = appconfUSB_AUDIO_SAMPLE_RATE;
    clkValid = 1;

    sampleFreqRng.wNumSubRanges = 1;
    sampleFreqRng.subrange[0].bMin = appconfUSB_AUDIO_SAMPLE_RATE;
    sampleFreqRng.subrange[0].bMax = appconfUSB_AUDIO_SAMPLE_RATE;
    sampleFreqRng.subrange[0].bRes = 0;

    init_volume_multipliers();

    UserHostActive_GPO_Init(); // Initialise host active GPO port

    rx_buffer = xStreamBufferCreate(2 * CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ, 0);

#if appconfUSB_AUDIO_CLASS == appconfUSB_AUDIO_CLASS_1
    samples_from_host_stream_buf = xStreamBufferCreate(2 * sizeof(int32_t) * appconfAUDIO_PIPELINE_FRAME_ADVANCE * CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX,
                                                       0);
#else
    samples_from_host_stream_buf = xStreamBufferCreate(2 * sizeof(samp_t) * appconfAUDIO_PIPELINE_FRAME_ADVANCE * CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX,
                                                       0);
#endif

    samples_to_host_stream_buf_size_bytes = 4 * sizeof(samp_t) * appconfAUDIO_PIPELINE_FRAME_ADVANCE * CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX;

    samples_to_host_stream_buf = xStreamBufferCreate(samples_to_host_stream_buf_size_bytes, 0);

    rtos_printf("[USB tile %d] usb_audio_init sample_rate=%u frame=%u chans_rx=%u chans_tx=%u\n",
                THIS_XCORE_TILE,
                (unsigned)appconfUSB_AUDIO_SAMPLE_RATE,
                (unsigned)AUDIO_FRAMES_PER_USB_FRAME,
                (unsigned)CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX,
                (unsigned)CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX);
    rtos_printf("[USB tile %d] stream buffers rx=%u tx=%u\n",
                THIS_XCORE_TILE,
                (unsigned)(2 * CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ),
                (unsigned)samples_to_host_stream_buf_size_bytes);

    xTaskCreate((TaskFunction_t)usb_audio_out_task, "usb_audio_out_task", portTASK_STACK_DEPTH(usb_audio_out_task), intertile_ctx, priority, &usb_audio_out_task_handle);


    // Task for receiving audio from the i2s to usb tile
    xTaskCreate((TaskFunction_t) i2s_to_usb_intertile,
            "i2s_to_usb_intertile",
            RTOS_THREAD_STACK_SIZE(i2s_to_usb_intertile),
            NULL,
            appconfAUDIO_PIPELINE_TASK_PRIORITY,
            NULL);
}
