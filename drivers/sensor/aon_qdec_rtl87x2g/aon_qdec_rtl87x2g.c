/*
 * Copyright(c) 2020, Realtek Semiconductor Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT realtek_rtl87x2g_aon_qdec

#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/pinctrl.h>
#include <soc.h>

#include <rtl_aon_qdec.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/policy.h>

#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(aon_qdec_rtl87x2g, CONFIG_SENSOR_LOG_LEVEL);
#include "trace.h"
#define FULL_ANGLE 360
#define MAX_ACC_CNT_BITS 16

//add by cwl
#define QDEC_X_PHA_PIN P9_0
#define QDEC_X_PHB_PIN P9_1
#include "rtl_gpio.h"
#include <rtl_pinmux.h>

void qdec_io_read(void)
{
    AON_QDEC_CounterPauseCmd(AON_QDEC, AON_QDEC_AXIS_X, ENABLE);
    // AON_QDEC_Cmd(AON_QDEC, AON_QDEC_AXIS_X, DISABLE);
    
    //note:p6_2/p6_3,p9_0/p9_1 share the same pin!
    Pinmux_Deinit(P6_2);
    Pinmux_Deinit(P6_3);

    Pinmux_Config(QDEC_X_PHA_PIN, DWGPIO);
    Pinmux_Config(QDEC_X_PHB_PIN, DWGPIO);
    Pad_Config(QDEC_X_PHA_PIN, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_NONE, PAD_OUT_DISABLE,
               PAD_OUT_LOW);
    Pad_Config(QDEC_X_PHB_PIN, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_NONE, PAD_OUT_DISABLE,
               PAD_OUT_LOW);
    

    int pha_wakeup_level = GPIO_ReadInputDataBit(GPIO_GetPort(QDEC_X_PHA_PIN),
                                                            GPIO_GetPin(QDEC_X_PHA_PIN));
    int phb_wakeup_level = GPIO_ReadInputDataBit(GPIO_GetPort(QDEC_X_PHB_PIN),
                                                            GPIO_GetPin(QDEC_X_PHB_PIN));

    Pinmux_AON_Config(QDPH0_IN_P9_0_P9_1);

    Pinmux_Config(P6_2, DWGPIO);
    Pinmux_Config(P6_3, DWGPIO);
    Pad_Config(P6_2, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_DOWN, PAD_OUT_DISABLE,
               PAD_OUT_LOW);
    Pad_Config(P6_3, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_DOWN, PAD_OUT_DISABLE,
               PAD_OUT_LOW);
    AON_QDEC_CounterPauseCmd(AON_QDEC, AON_QDEC_AXIS_X, DISABLE);  
    // AON_QDEC_Cmd(AON_QDEC, AON_QDEC_AXIS_X, ENABLE);
    // LOG_WRN("a,b=[%d,%d],gpiob:%x",pha_wakeup_level,phb_wakeup_level,GPIOB->GPIO_PAD_STATE);
  
    if (pha_wakeup_level)
    {
        System_WakeUpPinEnable(QDEC_X_PHA_PIN, PAD_WAKEUP_POL_LOW, PAD_WAKEUP_DEB_DISABLE);
    }
    else
    {
        System_WakeUpPinEnable(QDEC_X_PHA_PIN, PAD_WAKEUP_POL_HIGH, PAD_WAKEUP_DEB_DISABLE);
    }

    if (phb_wakeup_level)
    {
        System_WakeUpPinEnable(QDEC_X_PHB_PIN, PAD_WAKEUP_POL_LOW, PAD_WAKEUP_DEB_DISABLE);
    }
    else
    {
        System_WakeUpPinEnable(QDEC_X_PHB_PIN, PAD_WAKEUP_POL_HIGH, PAD_WAKEUP_DEB_DISABLE);
    }
}
struct aon_qdec_rtl87x2g_data
{
    int32_t acc;
    int16_t round;
    uint8_t counts_per_revolution;
    uint32_t debounce_time_ms;
    sensor_trigger_handler_t data_ready_handler;
    const struct sensor_trigger *data_ready_trigger;
};

struct aon_qdec_rtl87x2g_config
{
    uint32_t reg;
    const struct pinctrl_dev_config *pcfg;
    void (*irq_connect)(void);
};

static int aon_qdec_rtl87x2g_sample_fetch(const struct device *dev,
                                          enum sensor_channel chan)
{
    const struct aon_qdec_rtl87x2g_config *config = dev->config;
    AON_QDEC_TypeDef *aon_qdec = (AON_QDEC_TypeDef *)config->reg;
    struct aon_qdec_rtl87x2g_data *data = dev->data;
    uint16_t acc_cnt;
    unsigned int key;

    if ((chan != SENSOR_CHAN_ALL) && (chan != SENSOR_CHAN_ROTATION))
    {
        return -ENOTSUP;
    }

    key = irq_lock();
    acc_cnt = AON_QDEC_GetAxisCount(aon_qdec, AON_QDEC_AXIS_X);
    irq_unlock(key);

    data->acc = data->round * 65536 + acc_cnt;
    LOG_DBG("acc_cnt=%d, data->round=%d, data->acc=%d\n", \
            acc_cnt, data->round, data->acc);

    return 0;
}

static int aon_qdec_rtl87x2g_channel_get(const struct device *dev,
                                         enum sensor_channel chan,
                                         struct sensor_value *val)
{
    struct aon_qdec_rtl87x2g_data *data = dev->data;
    int32_t acc;
    //change: report delta by cwl@2024/07/05
    static int32_t last_acc=0;  //add
    // acc = (int32_t)data->acc;
    acc = (int32_t)(data->acc-last_acc); //add

    switch (chan)
    {
    case SENSOR_CHAN_ROTATION:
        // val->val1 = FULL_ANGLE / data->counts_per_revolution * acc;
        val->val1 = FULL_ANGLE / (data->counts_per_revolution*10) * acc; //add
        val->val2 = 0;
        break;
    default:
        return -ENOTSUP;
    }

    LOG_DBG("acc=%d, counts_per_revolution=%d, val1=%d\n", \
            acc, data->counts_per_revolution, val->val1);
    last_acc = data->acc; //add
    return 0;
}

static int aon_qdec_rtl87x2g_trigger_set(const struct device *dev,
                                         const struct sensor_trigger *trig,
                                         sensor_trigger_handler_t handler)
{
    struct aon_qdec_rtl87x2g_data *data = dev->data;
    unsigned int key;

    if (trig->type != SENSOR_TRIG_DATA_READY)
    {
        return -ENOTSUP;
    }

    if ((trig->chan != SENSOR_CHAN_ALL) &&
        (trig->chan != SENSOR_CHAN_ROTATION))
    {
        return -ENOTSUP;
    }

    if (handler)
    {
        key = irq_lock();
        data->data_ready_handler = handler;
        data->data_ready_trigger = trig;
        irq_unlock(key);
    }

    return 0;
}

static void aon_qdec_rtl87x2g_isr(const struct device *dev)
{
    struct aon_qdec_rtl87x2g_data *data = dev->data;
    const struct aon_qdec_rtl87x2g_config *config = dev->config;
    AON_QDEC_TypeDef *aon_qdec = (AON_QDEC_TypeDef *)config->reg;
    sensor_trigger_handler_t handler;
    const struct sensor_trigger *trig;
    if (AON_QDEC_GetFlagState(aon_qdec, AON_QDEC_FLAG_ILLEGAL_STATUS_X))
    {
        AON_QDEC_ClearINTPendingBit(aon_qdec, AON_QDEC_CLR_ILLEGAL_INT_X);
        AON_QDEC_ClearINTPendingBit(aon_qdec, AON_QDEC_CLR_ILLEGAL_CT_X);
        LOG_ERR("aon qdec illegal status\n");
    }
    else if (AON_QDEC_GetFlagState(aon_qdec, AON_QDEC_FLAG_NEW_CT_STATUS_X))
    {
        if (AON_QDEC_GetFlagState(aon_qdec, AON_QDEC_FLAG_OVERFLOW_X))
        {
            data->round += 1;
            AON_QDEC_ClearINTPendingBit(aon_qdec, AON_QDEC_CLR_OVERFLOW_X);
        }
        if (AON_QDEC_GetFlagState(aon_qdec, AON_QDEC_FLAG_UNDERFLOW_X))
        {
            data->round -= 1;
            AON_QDEC_ClearINTPendingBit(aon_qdec, AON_QDEC_CLR_UNDERFLOW_X);
        }

        AON_QDEC_ClearINTPendingBit(aon_qdec, AON_QDEC_CLR_NEW_CT_X);
        handler = data->data_ready_handler;
        trig = data->data_ready_trigger;
        LOG_DBG("qdec trig");
        if (handler)
        {
            handler(dev, trig);
        }
    }
}

static const struct sensor_driver_api aon_qdec_rtl87x2g_driver_api =
{
    .sample_fetch = aon_qdec_rtl87x2g_sample_fetch,
    .channel_get  = aon_qdec_rtl87x2g_channel_get,
    .trigger_set  = aon_qdec_rtl87x2g_trigger_set,
};

static int aon_qdec_rtl87x2g_init(const struct device *dev)
{
    DBG_DIRECT("aon_qdec_rtl87x2g_init");
    struct aon_qdec_rtl87x2g_data *data = dev->data;
    const struct aon_qdec_rtl87x2g_config *config = dev->config;
    AON_QDEC_TypeDef *aon_qdec = (AON_QDEC_TypeDef *)config->reg;
    int ret = 0;

    ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);

    if (ret < 0)
    {
        return ret;
    }

    AON_QDEC_InitTypeDef qdec_init_struct;
    AON_QDEC_StructInit(&qdec_init_struct);
    if (data->counts_per_revolution == 2)
    {
        qdec_init_struct.counterScaleX = CounterScale_2_Phase;
    }
    else if (data->counts_per_revolution == 4)
    {
        qdec_init_struct.counterScaleX = CounterScale_1_Phase;
    }
    else
    {
        LOG_ERR("Unspported counts_per_revolution: %d", data->counts_per_revolution);
        return -ENOTSUP;
    }
    qdec_init_struct.axisConfigX = ENABLE;
    qdec_init_struct.debounceTimeX = 32 * data->debounce_time_ms; /*!< 5ms */
    qdec_init_struct.debounceEnableX = ENABLE;
    qdec_init_struct.initPhaseX = phaseMode0;
    qdec_init_struct.manualLoadInitPhase = ENABLE;
    AON_QDEC_Init(aon_qdec, &qdec_init_struct);

    AON_QDEC_INTMask(aon_qdec, AON_QDEC_X_INT_MASK, DISABLE);
    AON_QDEC_INTMask(aon_qdec, AON_QDEC_X_CT_INT_MASK, DISABLE);
    AON_QDEC_INTMask(aon_qdec, AON_QDEC_X_ILLEAGE_INT_MASK, DISABLE);

    AON_QDEC_INTConfig(aon_qdec, AON_QDEC_X_INT_NEW_DATA, ENABLE);
    AON_QDEC_INTConfig(aon_qdec, AON_QDEC_X_INT_ILLEAGE, ENABLE);

    AON_QDEC_Cmd(aon_qdec, AON_QDEC_AXIS_X, ENABLE);
    config->irq_connect();

    return ret;
}
#ifdef CONFIG_PM_DEVICE
static int aon_qdec_rtl87x2g_pm_action(const struct device *dev,
                                  enum pm_device_action action)
{
    const struct aon_qdec_rtl87x2g_config *config = dev->config;

    int err;


    switch (action)
    {
    case PM_DEVICE_ACTION_SUSPEND:

        qdec_io_read();
        /* Move pins to sleep state */
        err = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_SLEEP);
        if ((err < 0) && (err != -ENOENT))
        {
            return err;
        }
        break;
    case PM_DEVICE_ACTION_RESUME:
        /* Set pins to active state */
        err = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
        if (err < 0)
        {
            return err;
        }



        break;
    default:
        return -ENOTSUP;
    }

    return 0;
}
#endif /* CONFIG_PM_DEVICE */


#define AON_QDEC_RTL87X2G_INIT(index)                       \
    PINCTRL_DT_INST_DEFINE(index);                  \
    static void aon_qdec_rtl87x2g_irq_connect_##index(void) \
    {                                   \
        IRQ_CONNECT(DT_INST_IRQN(index),                \
                    DT_INST_IRQ(index, priority),               \
                    aon_qdec_rtl87x2g_isr, DEVICE_DT_INST_GET(index),       \
                    0);                         \
        irq_enable(DT_INST_IRQN(index));                \
    }        \
    static const struct aon_qdec_rtl87x2g_config aon_qdec##index##_rtl87x2g_config = {  \
        .reg = DT_INST_REG_ADDR(index),         \
               .pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(index),      \
                       .irq_connect = aon_qdec_rtl87x2g_irq_connect_##index,                        \
    };                              \
    static struct aon_qdec_rtl87x2g_data aon_qdec##index##_rtl87x2g_data = {  \
        .acc = 0,    \
               .round = 0,    \
                        .debounce_time_ms = DT_INST_PROP(index, debounce_time_ms),    \
                                            .counts_per_revolution = DT_INST_PROP(index, counts_per_revolution),        \
    };      \
    PM_DEVICE_DT_INST_DEFINE(index, aon_qdec_rtl87x2g_pm_action);    \
    SENSOR_DEVICE_DT_INST_DEFINE(index, aon_qdec_rtl87x2g_init, PM_DEVICE_DT_INST_GET(index),   \
                                 &aon_qdec##index##_rtl87x2g_data, &aon_qdec##index##_rtl87x2g_config, \
                                 POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,   \
                                 &aon_qdec_rtl87x2g_driver_api);

DT_INST_FOREACH_STATUS_OKAY(AON_QDEC_RTL87X2G_INIT)
