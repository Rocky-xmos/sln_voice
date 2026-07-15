// Copyright 2022-2026 XMOS LIMITED.
// This Software is subject to the terms of the XMOS Public Licence: Version 1.

/* System headers */
#include <platform.h>
#include <rtos_printf.h>

/* App headers */
#include "platform_conf.h"
#include "platform/app_pll_ctrl.h"
#include "platform/driver_instances.h"
#include "platform/platform_init.h"
#include "usb_support.h"

static void mclk_init(chanend_t other_tile_c)
{
#if ON_TILE(I2S_TILE_NO)
    rtos_printf("[startup tile %d] app_pll_init for MCLK\n", THIS_XCORE_TILE);
    app_pll_init();
    rtos_printf("[startup tile %d] app_pll_init done\n", THIS_XCORE_TILE);
#endif
    (void) other_tile_c;
}

static void flash_init(void)
{
#if ON_TILE(FLASH_TILE_NO)
    fl_QuadDeviceSpec qspi_spec = BOARD_QSPI_SPEC;
    fl_QSPIPorts qspi_ports = {
        .qspiCS = PORT_SQI_CS,
        .qspiSCLK = PORT_SQI_SCLK,
        .qspiSIO = PORT_SQI_SIO,
        .qspiClkblk = FLASH_CLKBLK,
    };

    rtos_dfu_image_init(
            dfu_image_ctx,
            &qspi_ports,
            &qspi_spec,
            1);

    rtos_qspi_flash_init(
            qspi_flash_ctx,
            FLASH_CLKBLK,
            PORT_SQI_CS,
            PORT_SQI_SCLK,
            PORT_SQI_SIO,
            NULL);
#endif
}

static void i2c_init(void)
{
#if ON_TILE(I2C_TILE_NO)
    rtos_i2c_master_init(
            i2c_master_ctx,
            PORT_I2C_SCL, 0, 0,
            PORT_I2C_SDA, 0, 0,
            0,
            100);
#endif
}

static void i2s_init(void)
{
#if appconfI2S_ENABLED
#if ON_TILE(I2S_TILE_NO)
    port_t p_i2s_dout[1] = {
            PORT_I2S_DAC_DATA
    };
    port_t p_i2s_din[1] = {
            PORT_I2S_ADC_DATA
    };

    rtos_i2s_master_init(
            i2s_ctx,
            (1 << appconfI2S_IO_CORE),
            p_i2s_dout,
            1,
            p_i2s_din,
            1,
            PORT_I2S_BCLK,
            PORT_I2S_LRCLK,
            PORT_MCLK,
            I2S_CLKBLK);
#endif
#endif
}

static void usb_init(void)
{
#if appconfUSB_ENABLED && ON_TILE(USB_TILE_NO)
    usb_manager_init();
#endif
}

void platform_init(chanend_t other_tile_c)
{
    rtos_intertile_init(intertile_ctx, other_tile_c);

    rtos_intertile_init(intertile_usb_audio_ctx, other_tile_c);
    rtos_intertile_init(intertile_i2s_audio_ctx, other_tile_c);

    mclk_init(other_tile_c);
    flash_init();
    i2c_init();
    i2s_init();
    usb_init();
}
