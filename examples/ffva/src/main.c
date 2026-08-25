// Copyright 2020-2026 XMOS LIMITED.
// This Software is subject to the terms of the XMOS Public Licence: Version 1.

#include <platform.h>
#include <xs1.h>
#include <xcore/channel.h>

/* FreeRTOS headers */
#include "FreeRTOS.h"
#include "task.h"
#include "stream_buffer.h"
#include "queue.h"

/* Library headers */
#include "rtos_printf.h"
#include "src.h"

/* App headers */
#include "app_conf.h"
#include "platform/platform_init.h"
#include "platform/driver_instances.h"
#include "platform/platform_conf.h"
#include "usb_support.h"
#include "usb_audio.h"
#include "audio_pipeline.h"
#include "dfu_servicer.h"

/* Headers used for the WW intent engine */
#if appconfINTENT_ENABLED
#include "intent_engine.h"
#include "intent_handler.h"
#include "fs_support.h"
#include "gpi_ctrl.h"
#include "leds.h"
#endif
#include "gpio_test/gpio_test.h"

/* Config headers for sw_pll */
#include "sw_pll.h"

volatile int mic_from_usb = appconfMIC_SRC_DEFAULT;
volatile int aec_ref_source = appconfAEC_REF_DEFAULT;

/* The audio pipeline uses signed Q1.31 samples. With I2S_DATA_WIDTH=16, the
 * I2S driver takes the transmitted sample from the low 16 bits of its API
 * word. */
static inline int32_t q31_to_i2s16(int32_t sample)
{
    return (int32_t)(int16_t)((uint32_t)sample >> 16);
}

static inline int32_t i2s16_to_q31(int32_t sample)
{
    return (int32_t)sample << 16;
}

#if appconfI2S_ENABLED && (appconfI2S_MODE == appconfI2S_MODE_SLAVE)
void i2s_slave_intertile(void *args) {
    (void) args;
    int32_t tmp[appconfAUDIO_PIPELINE_FRAME_ADVANCE][appconfAUDIO_PIPELINE_CHANNELS];

    while(1) {
        memset(tmp, 0x00, sizeof(tmp));

        size_t bytes_received = 0;
        bytes_received = rtos_intertile_rx_len(
                intertile_ctx,
                appconfI2S_OUTPUT_SLAVE_PORT,
                portMAX_DELAY);

        xassert(bytes_received == sizeof(tmp));

        rtos_intertile_rx_data(
                intertile_ctx,
                tmp,
                bytes_received);

        rtos_i2s_tx(i2s_ctx,
                    (int32_t*) tmp,
                    appconfAUDIO_PIPELINE_FRAME_ADVANCE,
                    portMAX_DELAY);


#if ON_TILE(I2S_TILE_NO) && appconfRECOVER_MCLK_I2S_APP_PLL
    sw_pll_ctx_t* i2s_callback_args = (sw_pll_ctx_t*) args;
    port_clear_buffer(i2s_callback_args->p_bclk_count);
    port_in(i2s_callback_args->p_bclk_count);                                  // Block until BCLK transition to synchronise. Will consume up to 1/64 of a LRCLK cycle
    uint16_t mclk_pt = port_get_trigger_time(i2s_callback_args->p_mclk_count); // Immediately sample mclk_count
    uint16_t bclk_pt = port_get_trigger_time(i2s_callback_args->p_bclk_count); // Now grab bclk_count (which won't have changed)

    sw_pll_lut_do_control(i2s_callback_args->sw_pll, mclk_pt, bclk_pt);
#endif

    }
}
#endif

void audio_pipeline_input(void *input_app_data,
                        int32_t **input_audio_frames,
                        size_t ch_count,
                        size_t frame_count)
{
    (void) input_app_data;
    int32_t **mic_ptr = (int32_t **)(input_audio_frames + (2 * frame_count));

    static int flushed;
    while (!flushed) {
        size_t received;
        received = rtos_mic_array_rx(mic_array_ctx,
                                     mic_ptr,
                                     frame_count,
                                     0);
        if (received == 0) {
            rtos_mic_array_rx(mic_array_ctx,
                              mic_ptr,
                              frame_count,
                              portMAX_DELAY);
            flushed = 1;
        }
    }

    /*
     * NOTE: ALWAYS receive the next frame from the PDM mics,
     * even if USB is the current mic source. The controls the
     * timing since usb_audio_recv() does not block and will
     * receive all zeros if no frame is available yet.
     */
    rtos_mic_array_rx(mic_array_ctx,
                      mic_ptr,
                      frame_count,
                      portMAX_DELAY);

#if appconfDEBUG_RAW_MIC_SAMPLES
    /* mic_ptr addresses channel-sample-format data: frame_count samples of
     * mic 0 followed by frame_count samples of mic 1. Print one snapshot per
     * 100 frames rather than every sample to preserve real-time operation. */
    static unsigned raw_mic_debug_frame;
    int32_t *raw_mic_samples = (int32_t *)mic_ptr;

    if ((raw_mic_debug_frame++ % 100U) == 0U) {
        rtos_printf("raw mic0: %08x %08x %08x %08x | mic1: %08x %08x %08x %08x\n",
                    (unsigned)raw_mic_samples[0],
                    (unsigned)raw_mic_samples[1],
                    (unsigned)raw_mic_samples[2],
                    (unsigned)raw_mic_samples[3],
                    (unsigned)raw_mic_samples[frame_count],
                    (unsigned)raw_mic_samples[frame_count + 1],
                    (unsigned)raw_mic_samples[frame_count + 2],
                    (unsigned)raw_mic_samples[frame_count + 3]);
    }
#endif

#if appconfUSB_ENABLED
    int32_t **usb_mic_audio_frame = NULL;
    size_t ch_cnt = 2;  /* ref frames */

    if (aec_ref_source == appconfAEC_REF_USB) {
        usb_mic_audio_frame = input_audio_frames;
    }

    if (mic_from_usb) {
        ch_cnt += 2;  /* mic frames */
    }

    /*
     * As noted above, this does not block.
     * and expects ref L, ref R, mic 0, mic 1
     */
    usb_audio_recv(intertile_usb_audio_ctx,
                   frame_count,
                   usb_mic_audio_frame,
                   ch_cnt);
#endif

#if appconfI2S_ENABLED
    if (!appconfUSB_ENABLED || aec_ref_source == appconfAEC_REF_I2S) {
        /* This shouldn't need to block given it shares a clock with the PDM mics */

        xassert(frame_count == appconfAUDIO_PIPELINE_FRAME_ADVANCE);
        /* I2S provides sample channel format */
        int32_t tmp[appconfAUDIO_PIPELINE_FRAME_ADVANCE][appconfAUDIO_PIPELINE_CHANNELS];
        int32_t *tmpptr = (int32_t *)input_audio_frames;

        size_t rx_count =
        rtos_i2s_rx(i2s_ctx,
                    (int32_t*) tmp,
                    frame_count,
                    portMAX_DELAY);
        xassert(rx_count == frame_count);

        for (int i=0; i<frame_count; i++) {
            /* ref is first */
#if appconfI2S_AUDIO_SAMPLE_RATE == (3 * appconfAUDIO_PIPELINE_SAMPLE_RATE)
            /* The 3:1 receive SRC converts the PCM16 input to Q1.31 before
             * filtering, so its output is already in the pipeline format. */
            *(tmpptr + i) = tmp[i][0];
            *(tmpptr + i + frame_count) = 0;
#else
            *(tmpptr + i) = i2s16_to_q31(tmp[i][0]);
            *(tmpptr + i + frame_count) = 0;
#endif
        }
        // printf("0x%x,",tmp[0][0]);
    }
#endif

}

int audio_pipeline_output(void *output_app_data,
                        int32_t **output_audio_frames,
                        size_t ch_count,
                        size_t frame_count)
{
    (void) output_app_data;
#if appconfI2S_ENABLED
#if appconfI2S_MODE == appconfI2S_MODE_MASTER
#if !appconfI2S_TDM_ENABLED
    xassert(frame_count == appconfAUDIO_PIPELINE_FRAME_ADVANCE);
    /* I2S expects sample channel format */
    int32_t tmp[appconfAUDIO_PIPELINE_FRAME_ADVANCE][appconfAUDIO_PIPELINE_CHANNELS];
    int32_t *tmpptr = (int32_t *)output_audio_frames;
    for (int j=0; j<frame_count; j++) {
#if appconfI2S_LOOPBACK_DEBUG
        /* output_audio_frames format is processed, reference, raw mic.
         * The I2S input is stored in the reference pair by
         * audio_pipeline_input(). */
        tmp[j][0] = q31_to_i2s16(*(tmpptr + j + (2 * frame_count)));
        tmp[j][1] = q31_to_i2s16(*(tmpptr + j + (3 * frame_count)));
#else
        /* Output the processed microphone signal to the DAC.
         * The processed pipeline output is mono, so duplicate it to both
         * DAC channels. The I2S reference remains input-only for the AEC.
         */
        int32_t processed_sample = q31_to_i2s16(*(tmpptr + j));
        tmp[j][0] = processed_sample;
        // tmp[j][1] = processed_sample;
        // tmp[j][0] = q31_to_i2s16(*(tmpptr + j + (4 * frame_count)))<<5;
        tmp[j][1] = q31_to_i2s16(*(tmpptr + j + (2 * frame_count)));
#endif
        // tmp[j][0] = q31_to_i2s16(*(tmpptr + j + (4 * frame_count)));
        // tmp[j][1] = (*(tmpptr + j + (2 * frame_count)));
    }
    // printf("0x%x,",tmp[0][0]);

    rtos_i2s_tx(i2s_ctx,
                (int32_t*) tmp,
                frame_count,
                portMAX_DELAY);
#else
    int32_t *tmpptr = (int32_t *)output_audio_frames;
    for (int i = 0; i < frame_count; i++) {
        /* output_audio_frames format is
         *   processed_audio_frame
         *   reference_audio_frame
         *   raw_mic_audio_frame
         */
        int32_t tdm_output[6];

        tdm_output[0] = *(tmpptr + i + (4 * frame_count)) & ~0x1;   // mic 0
        tdm_output[1] = *(tmpptr + i + (5 * frame_count)) & ~0x1;   // mic 1
        tdm_output[2] = *(tmpptr + i + (2 * frame_count)) & ~0x1;   // ref 0
        tdm_output[3] = *(tmpptr + i + (3 * frame_count)) & ~0x1;   // ref 1
        tdm_output[4] = *(tmpptr + i) | 0x1;                        // proc 0
        tdm_output[5] = *(tmpptr + i + frame_count) | 0x1;          // proc 1

        rtos_i2s_tx(i2s_ctx,
                    tdm_output,
                    appconfI2S_AUDIO_SAMPLE_RATE / appconfAUDIO_PIPELINE_SAMPLE_RATE,
                    portMAX_DELAY);
    }
#endif

#elif appconfI2S_MODE == appconfI2S_MODE_SLAVE
    /* I2S expects sample channel format */
    int32_t tmp[appconfAUDIO_PIPELINE_FRAME_ADVANCE][appconfAUDIO_PIPELINE_CHANNELS];
    int32_t *tmpptr = (int32_t *)output_audio_frames;
    for (int j=0; j<appconfAUDIO_PIPELINE_FRAME_ADVANCE; j++) {
        /* ASR output is first */
        tmp[j][0] = *(tmpptr+j);
        tmp[j][1] = *(tmpptr+j+appconfAUDIO_PIPELINE_FRAME_ADVANCE);
    }

    rtos_intertile_tx(intertile_ctx,
                      appconfI2S_OUTPUT_SLAVE_PORT,
                      tmp,
                      sizeof(tmp));
#else
    #error "Invalid I2S mode"
#endif
#endif

#if appconfUSB_ENABLED
    usb_audio_send(intertile_usb_audio_ctx,
                frame_count,
                output_audio_frames,
                6);
#endif
#if appconfINTENT_ENABLED

    int32_t ww_samples[appconfAUDIO_PIPELINE_FRAME_ADVANCE];
    for (int j=0; j<appconfAUDIO_PIPELINE_FRAME_ADVANCE; j++) {
        /* ASR output is first */
        ww_samples[j] = (uint32_t) *(output_audio_frames+j);
    }

    intent_engine_sample_push(ww_samples,
                              frame_count);
#endif

    return AUDIO_PIPELINE_FREE_FRAME;
}

RTOS_I2S_APP_SEND_FILTER_CALLBACK_ATTR
size_t i2s_send_upsample_cb(rtos_i2s_t *ctx, void *app_data, int32_t *i2s_frame, size_t i2s_frame_size, int32_t *send_buf, size_t samples_available)
{
    static int i;
    static int32_t src_data[2][SRC_FF3V_FIR_TAPS_PER_PHASE] __attribute__((aligned(8)));

    xassert(i2s_frame_size == 2);

    switch (i) {
    case 0:
        i = 1;
        if (samples_available >= 2) {
            i2s_frame[0] = src_us3_voice_input_sample(src_data[0], src_ff3v_fir_coefs[2], send_buf[0]);
            i2s_frame[1] = src_us3_voice_input_sample(src_data[1], src_ff3v_fir_coefs[2], send_buf[1]);
            return 2;
        } else {
            i2s_frame[0] = src_us3_voice_input_sample(src_data[0], src_ff3v_fir_coefs[2], 0);
            i2s_frame[1] = src_us3_voice_input_sample(src_data[1], src_ff3v_fir_coefs[2], 0);
            return 0;
        }
    case 1:
        i = 2;
        i2s_frame[0] = src_us3_voice_get_next_sample(src_data[0], src_ff3v_fir_coefs[1]);
        i2s_frame[1] = src_us3_voice_get_next_sample(src_data[1], src_ff3v_fir_coefs[1]);
        return 0;
    case 2:
        i = 0;
        i2s_frame[0] = src_us3_voice_get_next_sample(src_data[0], src_ff3v_fir_coefs[0]);
        i2s_frame[1] = src_us3_voice_get_next_sample(src_data[1], src_ff3v_fir_coefs[0]);
        return 0;
    default:
        xassert(0);
        return 0;
    }
}

RTOS_I2S_APP_RECEIVE_FILTER_CALLBACK_ATTR
size_t i2s_send_downsample_cb(rtos_i2s_t *ctx, void *app_data, int32_t *i2s_frame, size_t i2s_frame_size, int32_t *receive_buf, size_t sample_spaces_free)
{
    static int i;
    static int64_t sum[2];
    static int32_t src_data[2][SRC_FF3V_FIR_NUM_PHASES][SRC_FF3V_FIR_TAPS_PER_PHASE] __attribute__((aligned (8)));

    xassert(i2s_frame_size == 2);

    /* With I2S_DATA_WIDTH=16 lib_i2s returns the sample in the low 16 bits
     * without sign extension. Convert it to signed Q1.31 before the FIR;
     * otherwise every negative PCM16 sample is interpreted as a large
     * positive value and the decimated reference becomes broadband noise. */
    const int32_t input_q31[2] = {
        i2s16_to_q31(i2s_frame[0]),
        i2s16_to_q31(i2s_frame[1])
    };

    switch (i) {
    case 0:
        i = 1;
        sum[0] = src_ds3_voice_add_sample(0, src_data[0][0], src_ff3v_fir_coefs[0], input_q31[0]);
        sum[1] = src_ds3_voice_add_sample(0, src_data[1][0], src_ff3v_fir_coefs[0], input_q31[1]);
        return 0;
    case 1:
        i = 2;
        sum[0] = src_ds3_voice_add_sample(sum[0], src_data[0][1], src_ff3v_fir_coefs[1], input_q31[0]);
        sum[1] = src_ds3_voice_add_sample(sum[1], src_data[1][1], src_ff3v_fir_coefs[1], input_q31[1]);
        return 0;
    case 2:
        i = 0;
        if (sample_spaces_free >= 2) {
            receive_buf[0] = src_ds3_voice_add_final_sample(sum[0], src_data[0][2], src_ff3v_fir_coefs[2], input_q31[0]);
            receive_buf[1] = src_ds3_voice_add_final_sample(sum[1], src_data[1][2], src_ff3v_fir_coefs[2], input_q31[1]);
            return 2;
        } else {
            (void) src_ds3_voice_add_final_sample(sum[0], src_data[0][2], src_ff3v_fir_coefs[2], input_q31[0]);
            (void) src_ds3_voice_add_final_sample(sum[1], src_data[1][2], src_ff3v_fir_coefs[2], input_q31[1]);
            return 0;
        }
    default:
        xassert(0);
        return 0;
    }
}

void i2s_rate_conversion_enable(void)
{
#if !appconfI2S_TDM_ENABLED
    rtos_i2s_send_filter_cb_set(i2s_ctx, i2s_send_upsample_cb, NULL);
#endif
    rtos_i2s_receive_filter_cb_set(i2s_ctx, i2s_send_downsample_cb, NULL);
}

void vApplicationMallocFailedHook(void)
{
    rtos_printf("Malloc Failed on tile %d!\n", THIS_XCORE_TILE);
    xassert(0);
    for(;;);
}

static void mem_analysis(void)
{
	for (;;) {
		// rtos_printf("Tile[%d]:\n\tMinimum heap free: %d\n\tCurrent heap free: %d\n", THIS_XCORE_TILE, xPortGetMinimumEverFreeHeapSize(), xPortGetFreeHeapSize());
		vTaskDelay(pdMS_TO_TICKS(5000));
	}
}

void startup_task(void *arg)
{
    rtos_printf("Startup task running from tile %d on core %d\n", THIS_XCORE_TILE, portGET_CORE_ID());
    platform_start();

#if ON_TILE(I2S_TILE_NO) && appconfI2S_ENABLED && (appconfI2S_MODE == appconfI2S_MODE_SLAVE)

// Use sw_pll_ctx only if the MCLK recovery is enabled
#if appconfRECOVER_MCLK_I2S_APP_PLL
    xTaskCreate((TaskFunction_t) i2s_slave_intertile,
                "i2s_slave_intertile",
                RTOS_THREAD_STACK_SIZE(i2s_slave_intertile),
                sw_pll_ctx,
                appconfAUDIO_PIPELINE_TASK_PRIORITY,
                NULL);
#else
    xTaskCreate((TaskFunction_t) i2s_slave_intertile,
                "i2s_slave_intertile",
                RTOS_THREAD_STACK_SIZE(i2s_slave_intertile),
                NULL,
                appconfAUDIO_PIPELINE_TASK_PRIORITY,
                NULL);
#endif
#endif
#if ON_TILE(1)
    gpio_test(gpio_ctx_t0);
#endif

#if appconfI2C_DFU_ENABLED && ON_TILE(I2C_CTRL_TILE_NO)
    // Initialise control related things
    servicer_t servicer_dfu;
    dfu_servicer_init(&servicer_dfu);

    xTaskCreate(
        dfu_servicer,
        "DFU servicer",
        RTOS_THREAD_STACK_SIZE(dfu_servicer),
        &servicer_dfu,
        appconfDEVICE_CONTROL_I2C_PRIORITY,
        NULL
    );
#endif

#if appconfINTENT_ENABLED && ON_TILE(0)
    led_task_create(appconfLED_TASK_PRIORITY, NULL);
#endif

#if appconfINTENT_ENABLED && ON_TILE(1)
    gpio_gpi_init(gpio_ctx_t0);
#endif

#if appconfINTENT_ENABLED && ON_TILE(FS_TILE_NO)
    rtos_fatfs_init(qspi_flash_ctx);
    // Setup flash low-level mode
    //   NOTE: must call rtos_qspi_flash_fast_read_shutdown_ll to use non low-level mode calls
    rtos_qspi_flash_fast_read_setup_ll(qspi_flash_ctx);
#endif

#if appconfINTENT_ENABLED && ON_TILE(ASR_TILE_NO)
    QueueHandle_t q_intent = xQueueCreate(appconfINTENT_QUEUE_LEN, sizeof(int32_t));
    intent_handler_create(appconfINTENT_MODEL_RUNNER_TASK_PRIORITY, q_intent);
    intent_engine_create(appconfINTENT_MODEL_RUNNER_TASK_PRIORITY, q_intent);
#endif

#if appconfINTENT_ENABLED && !ON_TILE(ASR_TILE_NO)
    // Wait until the intent engine is initialized before starting the
    // audio pipeline.
    intent_engine_ready_sync();
#endif

    audio_pipeline_init(NULL, NULL);

    mem_analysis();
}

void vApplicationMinimalIdleHook(void)
{
    rtos_printf("idle hook on tile %d core %d\n", THIS_XCORE_TILE, rtos_core_id_get());
    asm volatile("waiteu");
}

static void tile_common_init(chanend_t c)
{
    platform_init(c);
    chanend_free(c);

#if appconfUSB_ENABLED && ON_TILE(USB_TILE_NO)
    usb_audio_init(intertile_usb_audio_ctx, appconfUSB_AUDIO_TASK_PRIORITY);
#endif

    xTaskCreate((TaskFunction_t) startup_task,
                "startup_task",
                RTOS_THREAD_STACK_SIZE(startup_task),
                NULL,
                appconfSTARTUP_TASK_PRIORITY,
                NULL);

    rtos_printf("start scheduler on tile %d\n", THIS_XCORE_TILE);
    vTaskStartScheduler();
}

#if ON_TILE(0)
void main_tile0(chanend_t c0, chanend_t c1, chanend_t c2, chanend_t c3)
{
    (void) c0;
    (void) c2;
    (void) c3;

    tile_common_init(c1);
}
#endif

#if ON_TILE(1)
void main_tile1(chanend_t c0, chanend_t c1, chanend_t c2, chanend_t c3)
{
    (void) c1;
    (void) c2;
    (void) c3;

    tile_common_init(c0);
}
#endif
