// Copyright 2022-2026 XMOS LIMITED.
// This Software is subject to the terms of the XMOS Public Licence: Version 1.

#define DEBUG_UNIT I2S_AUDIO
#define DEBUG_PRINT_ENABLE_I2S_AUDIO 0

/* STD headers */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "rtos_printf.h"
#include <xcore/hwtimer.h>

/* FreeRTOS headers */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "stream_buffer.h"

/* App headers */
#include "app_conf.h"
#include "platform/driver_instances.h"
#include "i2s_audio.h"
#include "tusb_config.h"

static void recv_frame_from_i2s(int32_t *i2s_rx_data, size_t frame_count)
{
    size_t rx_count =
    rtos_i2s_rx(i2s_ctx,
                (int32_t*) i2s_rx_data,
                frame_count,
                portMAX_DELAY);

    xassert(rx_count == frame_count);

}

static void i2s_audio_recv_task(void *args)
{
    (void)args;
    int32_t input_data[appconfAUDIO_PIPELINE_FRAME_ADVANCE][NUM_I2S_CHANS];
    uint32_t recv_count = 0;
    for(;;)
    {
        recv_frame_from_i2s(&input_data[0][0], appconfAUDIO_PIPELINE_FRAME_ADVANCE);
        recv_count++;
        if ((recv_count % 100) == 0)
        {
            rtos_printf("[I2S tile %d] i2s_rx frames=%u forwarding_to_usb=%u bytes\n",
                        THIS_XCORE_TILE,
                        recv_count,
                        (unsigned)sizeof(input_data));
        }
        rtos_intertile_tx(
            intertile_usb_audio_ctx,
            appconfUSB_AUDIO_PORT,
            input_data,
            sizeof(input_data));
    }
}

// USB audio recv -> |to other tile| -> usb_to_i2s_intertile -> I2S send
static void usb_to_i2s_intertile(void *args) {
    (void) args;
    int32_t usb_to_i2s_samps[appconfAUDIO_PIPELINE_FRAME_ADVANCE][NUM_I2S_CHANS];
    uint32_t tx_count_total = 0;

    for(;;)
    {
        size_t bytes_received = rtos_intertile_rx_len(
            intertile_i2s_audio_ctx,
            appconfI2S_OUTPUT_SLAVE_PORT,
            portMAX_DELAY);

        xassert(bytes_received == sizeof(usb_to_i2s_samps));

        rtos_intertile_rx_data(
            intertile_i2s_audio_ctx,
            usb_to_i2s_samps,
            bytes_received);

        size_t tx_count = rtos_i2s_tx(
            i2s_ctx,
            (int32_t *) usb_to_i2s_samps,
            appconfAUDIO_PIPELINE_FRAME_ADVANCE,
            portMAX_DELAY);

        xassert(tx_count == appconfAUDIO_PIPELINE_FRAME_ADVANCE);
        tx_count_total++;
        if ((tx_count_total % 100) == 0)
        {
            rtos_printf("[I2S tile %d] usb_to_i2s frames=%u bytes=%u tx_count=%u\n",
                        THIS_XCORE_TILE,
                        tx_count_total,
                        (unsigned)bytes_received,
                        (unsigned)tx_count);
        }
    }
}

void i2s_audio_init()
{
    rtos_printf("[I2S tile %d] i2s_audio_init frame_advance=%u chans=%u\n",
                THIS_XCORE_TILE,
                (unsigned)appconfAUDIO_PIPELINE_FRAME_ADVANCE,
                (unsigned)NUM_I2S_CHANS);

    // I2S audio receive task (I2S -> USB)
    (void) rtos_osal_thread_create(
        NULL,
         "i2s_audio_recv",
        i2s_audio_recv_task,
        NULL,
        RTOS_THREAD_STACK_SIZE(i2s_audio_recv_task),
        appconfAUDIO_PIPELINE_TASK_PRIORITY);

    // Task for receiving USB audio frame from the USB tile to I2S
    (void) rtos_osal_thread_create(
        NULL,
        "usb_to_i2s_intertile",
        usb_to_i2s_intertile,
        NULL,
        RTOS_THREAD_STACK_SIZE(usb_to_i2s_intertile),
        appconfAUDIO_PIPELINE_TASK_PRIORITY);
}
