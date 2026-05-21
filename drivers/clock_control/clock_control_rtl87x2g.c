/*
 * Copyright(c) 2020, Realtek Semiconductor Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT realtek_rtl87x2g_cctl

#include <stdint.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control.h>

#include <rtl_rcc.h>

#include <trace.h>
#define DBG_DIRECT_SHOW 0

struct clock_control_rtl87x2g_config
{
    uint32_t reg;
};

typedef struct
{
    uint32_t apbperiph;
    uint32_t apbperiph_clk;
} apb_cfg;

static const apb_cfg rtl87x2g_apb_table[] = {
	{APBPeriph_SPIC0, APBPeriph_SPIC0_CLOCK},
	{APBPeriph_SPIC1, APBPeriph_SPIC1_CLOCK},
	{APBPeriph_SPIC2, APBPeriph_SPIC2_CLOCK},
	{APBPeriph_GDMA, APBPeriph_GDMA_CLOCK},
	{APBPeriph_SPI0_SLAVE, APBPeriph_SPI0_SLAVE_CLOCK},
	{APBPeriph_SPI1, APBPeriph_SPI1_CLOCK},
	{APBPeriph_SPI0, APBPeriph_SPI0_CLOCK},
	{APBPeriph_I2C3, APBPeriph_I2C3_CLOCK},
	{APBPeriph_I2C2, APBPeriph_I2C2_CLOCK},
	{APBPeriph_I2C1, APBPeriph_I2C1_CLOCK},
	{APBPeriph_I2C0, APBPeriph_I2C0_CLOCK},
	{APBPeriph_UART3, APBPeriph_UART3_CLOCK},
	{APBPeriph_UART2, APBPeriph_UART2_CLOCK},
	{APBPeriph_UART1, APBPeriph_UART1_CLOCK},
	{APBPeriph_UART0, APBPeriph_UART0_CLOCK},
	{APBPeriph_ACCXTAL, APBPeriph_ACCXTAL_CLOCK},
	{APBPeriph_PDCK, APBPeriph_PDCK_CLOCK},
	{APBPeriph_ZBMAC, APBPeriph_ZBMAC_CLOCK},
	{APBPeriph_BTPHY, APBPeriph_BTPHY_CLOCK},
	{APBPeriph_BTMAC, APBPeriph_BTMAC_CLOCK},
	{APBPeriph_SEGCOM, APBPeriph_SEGCOM_CLOCK},
	{APBPeriph_SPI3W, APBPeriph_SPI3W_CLOCK},
	{APBPeriph_ETH, APBPeriph_ETH_CLOCK},
	{APBPeriph_PPE, APBPeriph_PPE_CLOCK},
	{APBPeriph_KEYSCAN, APBPeriph_KEYSCAN_CLOCK},
	{APBPeriph_HRADC, APBPeriph_HRADC_CLOCK},
	{APBPeriph_ADC, APBPeriph_ADC_CLOCK},
	{APBPeriph_A2C, APBPeriph_A2C_CLOCK},
	{APBPeriph_IR, APBPeriph_IR_CLOCK},
	{APBPeriph_ISO7816, APBPeriph_ISO7816_CLOCK},
	{APBPeriph_GPIOB, APBPeriph_GPIOB_CLOCK},
	{APBPeriph_GPIOA, APBPeriph_GPIOA_CLOCK},
	{APBPeriph_DISP, APBPeriph_DISP_CLOCK},
	{APBPeriph_IMDC, APBPeriph_IMDC_CLOCK},
	{APBPeriph_TIMER, APBPeriph_TIMER_CLOCK},
	{APBPeriph_ENHTIMER, APBPeriph_ENHTIMER_CLOCK},
	{APBPeriph_ENHTIMER_PWM1, APBPeriph_ENHTIMER_PWM1_CLOCK},
	{APBPeriph_ENHTIMER_PWM0, APBPeriph_ENHTIMER_PWM0_CLOCK},
	{APBPeriph_ENHTIMER_PWM3, APBPeriph_ENHTIMER_PWM3_CLOCK},
	{APBPeriph_ENHTIMER_PWM2, APBPeriph_ENHTIMER_PWM2_CLOCK},
	{APBPeriph_SDHC, APBPeriph_SDHC_CLOCK},
	{APBPeriph_UART5, APBPeriph_UART5_CLOCK},
	{APBPeriph_UART4, APBPeriph_UART4_CLOCK},
	{APBPeriph_CODEC, APBPeriph_CODEC_CLOCK},
	{APBPeriph_I2S1, APBPeriph_I2S1_CLOCK},
	{APBPeriph_I2S0, APBPeriph_I2S0_CLOCK},
};

static int clock_control_rtl87x2g_on(const struct device *dev,
                                     clock_control_subsys_t sys)
{
    uint16_t id = *(uint16_t *)sys;
    RCC_PeriphClockCmd(rtl87x2g_apb_table[id].apbperiph, \
                       rtl87x2g_apb_table[id].apbperiph_clk, \
                       ENABLE);
#if DBG_DIRECT_SHOW
    DBG_DIRECT("[clock_control_rtl87x2g_on] sys=%d, apbperiph=0x%x, apbperiph_clk=0x%x", \
               id, rtl87x2g_apb_table[id].apbperiph, \
               rtl87x2g_apb_table[id].apbperiph_clk);
#endif
    return 0;
}

static int clock_control_rtl87x2g_off(const struct device *dev,
                                      clock_control_subsys_t sys)
{
    uint16_t id = *(uint16_t *)sys;

    RCC_PeriphClockCmd(rtl87x2g_apb_table[id].apbperiph, \
                       rtl87x2g_apb_table[id].apbperiph_clk, \
                       DISABLE);

#if DBG_DIRECT_SHOW
    DBG_DIRECT("[clock_control_rtl87x2g_off] sys=%d, apbperiph=%d, apbperiph_clk=%d", \
               sys, rtl87x2g_apb_table[id].apbperiph, \
               rtl87x2g_apb_table[id].apbperiph_clk);
#endif
    return 0;
}

static enum clock_control_status
clock_control_rtl87x2g_get_status(const struct device *dev,
                                  clock_control_subsys_t sys)
{
    const struct clock_control_rtl87x2g_config *config = dev->config;
    uint16_t id = *(uint16_t *)sys;

    uint32_t apbRegOff = (rtl87x2g_apb_table[id].apbperiph & (0XFF));
    uint32_t clk_func = rtl87x2g_apb_table[id].apbperiph_clk;

    if (rtl87x2g_apb_table[id].apbperiph == APBPeriph_CODEC)
    {
        if (sys_test_bit(PERIBLKCTRL_AUDIO_REG_BASE + apbRegOff, clk_func) != 0)
        {
            return CLOCK_CONTROL_STATUS_ON;
        }
    }
    else
    {
        if (sys_test_bit(config->reg + apbRegOff, clk_func) != 0)
        {
            DBG_DIRECT("[clock_control_rtl87x2g_get_status] sys=%d, status=on", \
                       sys);
            return CLOCK_CONTROL_STATUS_ON;
        }
    }

#if DBG_DIRECT_SHOW
    DBG_DIRECT("[clock_control_rtl87x2g_get_status] sys=%d, status=off", \
               sys);
#endif
    return CLOCK_CONTROL_STATUS_OFF;
}

static struct clock_control_driver_api clock_control_rtl87x2g_api =
{
    .on = clock_control_rtl87x2g_on,
    .off = clock_control_rtl87x2g_off,
//  .get_rate = clock_control_rtl87x2g_get_rate,
    .get_status = clock_control_rtl87x2g_get_status,
};

static const struct clock_control_rtl87x2g_config config =
{
    .reg = DT_REG_ADDR(DT_INST_PARENT(0)),
};

DEVICE_DT_INST_DEFINE(0, NULL, NULL, NULL, &config, PRE_KERNEL_1,
                      CONFIG_CLOCK_CONTROL_INIT_PRIORITY,
                      &clock_control_rtl87x2g_api);
