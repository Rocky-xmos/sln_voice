// Copyright 2022-2026 XMOS LIMITED.
// This Software is subject to the terms of the XMOS Public Licence: Version 1.

/* System headers */
#include <platform.h>
#include <rtos_printf.h>

/* FreeRTOS headers */
#include "FreeRTOS.h"

/* Library headers */
#include "fs_support.h"

/* App headers */
#include "platform_conf.h"
#include "platform/driver_instances.h"
#include "dac3101.h"
#include "usb_support.h"

extern void configure_io_expander(void);

static void flash_start(void)
{
#if ON_TILE(FLASH_TILE_NO)
    uint32_t flash_core_map = ~(1 << appconfUSB_INTERRUPT_CORE) & ~(1 << appconfI2C_INTERRUPT_CORE);
    rtos_qspi_flash_start(qspi_flash_ctx, appconfQSPI_FLASH_TASK_PRIORITY);
    rtos_qspi_flash_op_core_affinity_set(qspi_flash_ctx, flash_core_map);
#endif
}

static void i2c_master_start(void)
{
#if ON_TILE(I2C_TILE_NO)
    rtos_i2c_master_start(i2c_master_ctx);
#endif
}

static void audio_codec_start(void)
{
#if appconfI2S_ENABLED && ON_TILE(I2C_TILE_NO)
    int ret = 0;
    rtos_printf("[startup tile %d] starting codec init at %u Hz\n",
                THIS_XCORE_TILE,
                (unsigned)appconfI2S_AUDIO_SAMPLE_RATE);
    if (dac3101_init(appconfI2S_AUDIO_SAMPLE_RATE) != 0) {
        rtos_printf("DAC initialization failed\n");
    } else {
        rtos_printf("[startup tile %d] codec init done\n", THIS_XCORE_TILE);
    }
    rtos_intertile_tx(intertile_ctx, 0, &ret, sizeof(ret));
#elif appconfI2S_ENABLED
    int ret = 0;
    rtos_intertile_rx_len(intertile_ctx, 0, RTOS_OSAL_WAIT_FOREVER);
    rtos_intertile_rx_data(intertile_ctx, &ret, sizeof(ret));
#endif
}

static void enable_level_shifters(void)
{
#if appconfI2S_ENABLED
    int ret = 0;
#if ON_TILE(I2C_TILE_NO)
    rtos_printf("[startup tile %d] configuring IO expander for audio path\n", THIS_XCORE_TILE);
    configure_io_expander();
    rtos_intertile_tx(intertile_ctx, 0, &ret, sizeof(ret));
#else
    rtos_intertile_rx_len(intertile_ctx, 0, RTOS_OSAL_WAIT_FOREVER);
    rtos_intertile_rx_data(intertile_ctx, &ret, sizeof(ret));
#endif
#endif
}

static void i2s_start(void)
{
#if appconfI2S_ENABLED
#if ON_TILE(I2S_TILE_NO)
    rtos_printf("[startup tile %d] starting I2S mode=%d mclk=%u bclk_ratio=%u sample_rate=%u\n",
                THIS_XCORE_TILE,
                appconfI2S_MODE,
                (unsigned)MIC_ARRAY_CONFIG_MCLK_FREQ,
                (unsigned)rtos_i2s_mclk_bclk_ratio(MIC_ARRAY_CONFIG_MCLK_FREQ, appconfI2S_AUDIO_SAMPLE_RATE),
                (unsigned)appconfI2S_AUDIO_SAMPLE_RATE);
    rtos_i2s_start(
            i2s_ctx,
            rtos_i2s_mclk_bclk_ratio(MIC_ARRAY_CONFIG_MCLK_FREQ, appconfI2S_AUDIO_SAMPLE_RATE),
            I2S_MODE_I2S,
            2.2 * appconfAUDIO_PIPELINE_FRAME_ADVANCE,
            1.2 * appconfAUDIO_PIPELINE_FRAME_ADVANCE,
            appconfI2S_INTERRUPT_CORE);
#endif
#endif
}

static void usb_start(void)
{
#if appconfUSB_ENABLED && ON_TILE(USB_TILE_NO)
    rtos_printf("[startup tile %d] starting USB manager\n", THIS_XCORE_TILE);
    usb_manager_start(appconfUSB_MGR_TASK_PRIORITY);
    rtos_printf("[startup tile %d] USB manager started\n", THIS_XCORE_TILE);
#endif
}

void platform_start(void)
{
    rtos_printf("[startup tile %d] platform_start begin\n", THIS_XCORE_TILE);
    rtos_intertile_start(intertile_ctx);
    rtos_intertile_start(intertile_usb_audio_ctx);
    rtos_intertile_start(intertile_i2s_audio_ctx);
    flash_start();
    i2c_master_start();
    enable_level_shifters();
    audio_codec_start();
    i2s_start();
    usb_start();
    rtos_printf("[startup tile %d] platform_start end\n", THIS_XCORE_TILE);
}
