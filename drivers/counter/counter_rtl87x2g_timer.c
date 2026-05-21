/*
 * Copyright(c) 2020, Realtek Semiconductor Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT realtek_rtl87x2g_timer

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/rtl87x2g_clock_control.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <soc.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/policy.h>

#include <rtl_tim.h>
#include <rtl_enh_tim.h>
#include <rtl_rcc.h>
LOG_MODULE_REGISTER(counter_rtl87x2g_timer, CONFIG_COUNTER_LOG_LEVEL);

struct counter_rtl87x2g_ch_data
{
    counter_alarm_callback_t callback;
    void *user_data;
};

struct counter_rtl87x2g_data
{
    counter_top_callback_t top_cb;
    void *top_user_data;
    uint32_t guard_period;
    uint32_t freq;
 #ifdef CONFIG_PM_DEVICE
    union
    {
        TIMStoreReg_Typedef tim_store_buf;
        ENHTIMStoreReg_Typedef enhtim_store_buf;
    } store_buf;
#endif
    struct counter_rtl87x2g_ch_data alarm[];
};

struct counter_rtl87x2g_config
{
    struct counter_config_info counter_info;
    uint32_t reg;
    uint16_t clkid;
    bool enhanced;
    uint8_t prescaler;
    void (*irq_config)(const struct device *dev);
    uint32_t (*get_irq_pending)(void);
};

static int counter_rtl87x2g_timer_start(const struct device *dev)
{
    LOG_DBG("counter start\n");
    const struct counter_rtl87x2g_config *cfg = dev->config;

    if (cfg->enhanced)
    {
        LOG_DBG("enhed %s, ctrl=%x, current=%x, line%d\n", \
                dev->name, ((ENHTIM_TypeDef *)cfg->reg)->ENHTIM_MAX_CNT, \
                ENHTIM_GetCurrentCount((ENHTIM_TypeDef *)cfg->reg), __LINE__);
        ENHTIM_Cmd((ENHTIM_TypeDef *)cfg->reg, ENABLE);
    }
    else
    {
        LOG_DBG("not enhed %s, ctrl=%x, current=%x, line%d\n", \
                dev->name, ((TIM_TypeDef *)cfg->reg)->TIMER_CONTROLREG, \
                TIM_GetCurrentValue((TIM_TypeDef *)cfg->reg), __LINE__);
        TIM_Cmd((TIM_TypeDef *)cfg->reg, ENABLE);
    }

    return 0;
}

static int counter_rtl87x2g_timer_stop(const struct device *dev)
{
    LOG_DBG("counter stop\n");
    const struct counter_rtl87x2g_config *cfg = dev->config;

    if (cfg->enhanced)
    {
        ENHTIM_Cmd((ENHTIM_TypeDef *)cfg->reg, DISABLE);
    }
    else
    {
        TIM_Cmd((TIM_TypeDef *)cfg->reg, DISABLE);
    }

    return 0;
}

static int counter_rtl87x2g_timer_get_value(const struct device *dev,
                                            uint32_t *ticks)
{
    const struct counter_rtl87x2g_config *cfg = dev->config;

    if (cfg->enhanced)
    {
        *ticks = ENHTIM_GetCurrentCount((ENHTIM_TypeDef *)cfg->reg);
    }
    else
    {
        *ticks = TIM_GetCurrentValue((TIM_TypeDef *)cfg->reg);
    }

    LOG_DBG("ticks=%x\n", *ticks);
    return 0;
}

static uint32_t counter_rtl87x2g_timer_get_top_value(const struct device *dev)
{
    const struct counter_rtl87x2g_config *cfg = dev->config;
    if (cfg->enhanced)
    {
        LOG_DBG("enhed, topvalue=%x\n",
                ((ENHTIM_TypeDef *)cfg->reg)->ENHTIM_MAX_CNT);
        return ((ENHTIM_TypeDef *)cfg->reg)->ENHTIM_MAX_CNT;
    }
    else
    {
        LOG_DBG("not enhed, topvalue=%x\n",
                ((TIM_TypeDef *)cfg->reg)->TIMER_LOADCOUNT);
        return ((TIM_TypeDef *)cfg->reg)->TIMER_LOADCOUNT;
    }
}

static int
counter_rtl87x2g_timer_set_alarm(const struct device *dev, uint8_t chan,
                                 const struct counter_alarm_cfg *alarm_cfg)
{
    struct counter_rtl87x2g_data *data = dev->data;
    struct counter_rtl87x2g_ch_data *chdata = &data->alarm[chan];

    if (alarm_cfg->ticks > counter_rtl87x2g_timer_get_top_value(dev))
    {
        return -EINVAL;
    }

    if (chdata->callback)
    {
        return -EBUSY;
    }

    chdata->callback = alarm_cfg->callback;
    chdata->user_data = alarm_cfg->user_data;

    LOG_DBG("chan=%d\n", chan);
    LOG_ERR("Unspported alarm");
    return -ENOTSUP;
}

static int counter_rtl87x2g_timer_cancel_alarm(const struct device *dev,
                                               uint8_t chan)
{
    struct counter_rtl87x2g_data *data = dev->data;

    data->alarm[chan].callback = NULL;

    LOG_DBG("chan=%d\n", chan);
    LOG_ERR("Unspported alarm");
    return -ENOTSUP;
}

static int counter_rtl87x2g_timer_set_top_value(const struct device *dev,
                                                const struct counter_top_cfg *top_cfg)
{
    const struct counter_rtl87x2g_config *cfg = dev->config;
    struct counter_rtl87x2g_data *data = dev->data;
    int err = 0;

    for (uint32_t i = 0; i < cfg->counter_info.channels; i++)
    {
        /* Overflow can be changed only when all alarms are
         * disables.
         */
        if (data->alarm[i].callback)
        {
            return -EBUSY;
        }
    }
    LOG_DBG("ticks=%x, line%d\n", \
            top_cfg->ticks, __LINE__);

    if (cfg->enhanced)
    {
        ENHTIM_INTConfig((ENHTIM_TypeDef *)cfg->reg, ENHTIM_INT_TIM, DISABLE);
        ENHTIM_SetMaxCount((ENHTIM_TypeDef *)cfg->reg, top_cfg->ticks);
        ENHTIM_ClearINTPendingBit((ENHTIM_TypeDef *)cfg->reg, ENHTIM_INT_TIM);
        data->top_cb = top_cfg->callback;
        data->top_user_data = top_cfg->user_data;
        if (!(top_cfg->flags & COUNTER_TOP_CFG_DONT_RESET))
        {
            ENHTIM_Cmd((ENHTIM_TypeDef *)cfg->reg, DISABLE);
            ENHTIM_Cmd((ENHTIM_TypeDef *)cfg->reg, ENABLE);
        }
        else if (top_cfg->flags & COUNTER_TOP_CFG_RESET_WHEN_LATE)
        {
            ENHTIM_Cmd((ENHTIM_TypeDef *)cfg->reg, DISABLE);
            ENHTIM_Cmd((ENHTIM_TypeDef *)cfg->reg, ENABLE);
        }

        if (top_cfg->callback)
        {
            ENHTIM_INTConfig((ENHTIM_TypeDef *)cfg->reg, ENHTIM_INT_TIM, ENABLE);
        }
    }
    else
    {
        TIM_INTConfig((TIM_TypeDef *)cfg->reg, DISABLE);
        TIM_ChangePeriod((TIM_TypeDef *)cfg->reg, top_cfg->ticks);
        TIM_ClearINT((TIM_TypeDef *)cfg->reg);
        data->top_cb = top_cfg->callback;
        data->top_user_data = top_cfg->user_data;
        if (!(top_cfg->flags & COUNTER_TOP_CFG_DONT_RESET))
        {
            TIM_Cmd((TIM_TypeDef *)cfg->reg, DISABLE);
            TIM_Cmd((TIM_TypeDef *)cfg->reg, ENABLE);
        }
        else if (top_cfg->flags & COUNTER_TOP_CFG_RESET_WHEN_LATE)
        {
            TIM_Cmd((TIM_TypeDef *)cfg->reg, DISABLE);
            TIM_Cmd((TIM_TypeDef *)cfg->reg, ENABLE);
        }

        if (top_cfg->callback)
        {
            TIM_INTConfig((TIM_TypeDef *)cfg->reg, ENABLE);
        }
    }

    return err;
}

static uint32_t counter_rtl87x2g_timer_get_pending_int(const struct device *dev)
{
    LOG_DBG("get pending\n");

    const struct counter_rtl87x2g_config *cfg = dev->config;
    return cfg->get_irq_pending();
}

static uint32_t counter_rtl87x2g_timer_get_freq(const struct device *dev)
{
    struct counter_rtl87x2g_data *data = dev->data;
    LOG_DBG("freq=%d\n", data->freq);

    return data->freq;
}

static uint32_t counter_rtl87x2g_timer_get_guard_period(const struct device *dev,
                                                        uint32_t flags)
{
    LOG_DBG("get guard period\n");

    struct counter_rtl87x2g_data *data = dev->data;

    return data->guard_period;
}

static int counter_rtl87x2g_timer_set_guard_period(const struct device *dev,
                                                   uint32_t guard, uint32_t flags)
{
    LOG_DBG("guard=%d\n", guard);
    struct counter_rtl87x2g_data *data = dev->data;

    __ASSERT_NO_MSG(guard < counter_rtl87x2g_timer_get_top_value(dev));

    data->guard_period = guard;
    return 0;
}

static void top_irq_handle(const struct device *dev)
{
    struct counter_rtl87x2g_data *data = dev->data;
    counter_top_callback_t cb = data->top_cb;
    const struct counter_rtl87x2g_config *cfg = dev->config;
    if (cfg->enhanced)
    {
        if (ENHTIM_GetINTStatus((ENHTIM_TypeDef *)cfg->reg, ENHTIM_INT_TIM))
        {
            ENHTIM_ClearINTPendingBit((ENHTIM_TypeDef *)cfg->reg, ENHTIM_INT_TIM);
            __ASSERT(cb != NULL, "top event enabled - expecting callback");
            cb(dev, data->top_user_data);
        }
    }
    else
    {
        if (TIM_GetINTStatus((TIM_TypeDef *)cfg->reg))
        {
            TIM_ClearINT((TIM_TypeDef *)cfg->reg);
            __ASSERT(cb != NULL, "top event enabled - expecting callback");
            cb(dev, data->top_user_data);
        }
    }
}

static void alarm_irq_handle(const struct device *dev, uint32_t chan)
{
    LOG_DBG("chan=%d\n", chan);
    struct counter_rtl87x2g_data *data = dev->data;
    struct counter_rtl87x2g_ch_data *alarm = &data->alarm[chan];
    counter_alarm_callback_t cb;
    uint32_t ticks;
    counter_rtl87x2g_timer_get_value(dev, &ticks);
    cb = alarm->callback;
    alarm->callback = NULL;

    if (cb)
    {
        cb(dev, chan, ticks, alarm->user_data);
    }
}

#ifdef CONFIG_PM_DEVICE
static int counter_rtl87x2g_timer_pm_action(const struct device *dev,
                                            enum pm_device_action action)
{
    const struct counter_rtl87x2g_config *cfg = dev->config;
    struct counter_rtl87x2g_data *data = dev->data;
    void *timer_base = (void *)cfg->reg;
    // int err;
    extern void ENHTIM_DLPSEnter(void *PeriReg, void *StoreBuf);
    extern void ENHTIM_DLPSExit(void *PeriReg, void *StoreBuf);
    extern void TIM_DLPSEnter(void *PeriReg, void *StoreBuf);
    extern void TIM_DLPSExit(void *PeriReg, void *StoreBuf);

    switch (action)
    {
    case PM_DEVICE_ACTION_SUSPEND:

        if (cfg->enhanced)
        {
            ENHTIM_DLPSEnter(timer_base, &data->store_buf);
        }
        else
        {
            TIM_DLPSEnter(timer_base, &data->store_buf);
        }

    case PM_DEVICE_ACTION_RESUME:
        if (cfg->enhanced)
        {
            ENHTIM_DLPSExit(timer_base, &data->store_buf);
        }
        else
        {
            TIM_DLPSExit(timer_base, &data->store_buf);
        }

        break;
    default:
        return -ENOTSUP;
    }

    return 0;
}
#endif /* CONFIG_PM_DEVICE */

static void irq_handler(const struct device *dev)
{
    const struct counter_rtl87x2g_config *cfg = dev->config;
    top_irq_handle(dev);

    for (uint32_t i = 0; i < cfg->counter_info.channels; i++)
    {
        alarm_irq_handle(dev, i);
    }
}

static int counter_rtl87x2g_timer_init(const struct device *dev)
{
    const struct counter_rtl87x2g_config *cfg = dev->config;
    struct counter_rtl87x2g_data *data = dev->data;
    void *timer_base = (void *)cfg->reg;
    uint8_t clock_div;

    /* use clock_control_get_rate if clock driver is avaliable */
    uint32_t pclk = 40000000;

    (void)clock_control_on(RTL87X2G_CLOCK_CONTROLLER,
                           (clock_control_subsys_t)&cfg->clkid);

    data->freq = pclk / cfg->prescaler;

    cfg->irq_config(dev);

    switch (cfg->prescaler)
    {
    case 1:
        clock_div = CLOCK_DIV_1;
        break;
    case 2:
        clock_div = CLOCK_DIV_2;
        break;
    case 4:
        clock_div = CLOCK_DIV_4;
        break;
    case 8:
        clock_div = CLOCK_DIV_8;
        break;
    case 16:
        clock_div = CLOCK_DIV_16;
        break;
    case 32:
        clock_div = CLOCK_DIV_32;
        break;
    case 40:
        clock_div = CLOCK_DIV_40;
        break;
    case 64:
        clock_div = CLOCK_DIV_64;
        break;
    default:
        clock_div = CLOCK_DIV_1;
        break;
    }

    if (cfg->enhanced)
    {
        ENHTIM_InitTypeDef enh_tim_init_struct;
        ENHTIM_StructInit(&enh_tim_init_struct);
        enh_tim_init_struct.ENHTIM_ClockSource = CK_40M_TIMER;
        enh_tim_init_struct.ENHTIM_ClockDiv_En = ENABLE;
        enh_tim_init_struct.ENHTIM_ClockDiv = clock_div;
        enh_tim_init_struct.ENHTIM_Mode = ENHTIM_MODE_PWM_MANUAL;
        enh_tim_init_struct.ENHTIM_MaxCount = cfg->counter_info.max_top_value;
        ENHTIM_Init((ENHTIM_TypeDef *)timer_base, &enh_tim_init_struct);
        LOG_DBG("enhed %s, ctrl=%x, line%d\n", \
                dev->name, ((ENHTIM_TypeDef *)timer_base)->ENHTIM_CONFIGURE, __LINE__);
    }
    else
    {
        TIM_TimeBaseInitTypeDef timer_init_struct;
        TIM_StructInit(&timer_init_struct);
        timer_init_struct.TIM_ClockSrc = CK_40M_TIMER;
        timer_init_struct.TIM_SOURCE_DIV_En = ENABLE;
        timer_init_struct.TIM_SOURCE_DIV = clock_div;
        timer_init_struct.TIM_Mode = TIM_Mode_UserDefine;
        timer_init_struct.TIM_Period = cfg->counter_info.max_top_value;
        TIM_TimeBaseInit((TIM_TypeDef *)timer_base, &timer_init_struct);
        LOG_DBG("not enhed %s, ctrl=%x, line%d\n", \
                dev->name, ((TIM_TypeDef *)timer_base)->TIMER_CONTROLREG, __LINE__);
    }

    return 0;
}

static const struct counter_driver_api counter_rtl87x2g_timer_driver_api =
{
    .start = counter_rtl87x2g_timer_start,
    .stop = counter_rtl87x2g_timer_stop,
    .get_value = counter_rtl87x2g_timer_get_value,
    .set_alarm = counter_rtl87x2g_timer_set_alarm,
    .cancel_alarm = counter_rtl87x2g_timer_cancel_alarm,
    .set_top_value = counter_rtl87x2g_timer_set_top_value,
    .get_pending_int = counter_rtl87x2g_timer_get_pending_int,
    .get_top_value = counter_rtl87x2g_timer_get_top_value,
    .get_guard_period = counter_rtl87x2g_timer_get_guard_period,
    .set_guard_period = counter_rtl87x2g_timer_set_guard_period,
    .get_freq = counter_rtl87x2g_timer_get_freq,
};

#define TIMER_IRQ_CONFIG(index)                                                    \
    static void irq_config_##index(const struct device *dev)                   \
    {                                                                      \
        IRQ_CONNECT(DT_INST_IRQN(index),                \
                    DT_INST_IRQ(index, priority),               \
                    irq_handler, DEVICE_DT_INST_GET(index),       \
                    0);                         \
        irq_enable(DT_INST_IRQN(index));                \
    }                                                                      \
    static uint32_t get_irq_pending_##index(void)                              \
    {                                                                      \
        return NVIC_GetPendingIRQ(DT_INST_IRQN(index));                  \
    }

#define RTL87X2G_TIMER_INIT(index)                                                     \
    TIMER_IRQ_CONFIG(index);    \
    static struct timer_data_##index { \
        struct counter_rtl87x2g_data data;                                 \
        struct counter_rtl87x2g_ch_data alarm[DT_INST_PROP(index, channels)];  \
    } counter_rtl87x2g_data_##index = {0};    \
    static const struct counter_rtl87x2g_config counter_rtl87x2g_config_##index = {           \
        .counter_info = {.max_top_value = UINT32_MAX,          \
                         .flags = 0,        \
                         .freq = 0,                                          \
                         .channels = DT_INST_PROP(index, channels),                \
                        },                                                        \
                        .reg = DT_INST_REG_ADDR(index),                                    \
                               .clkid = DT_INST_CLOCKS_CELL(index, id),                           \
                                        .enhanced = DT_INST_PROP(index, is_enhanced),                      \
                                                    .prescaler = DT_INST_PROP(index, prescaler),                       \
                                                                 .irq_config = irq_config_##index,                                  \
                                                                                       .get_irq_pending = get_irq_pending_##index,                        \
    };                                                                     \
    \
     PM_DEVICE_DT_INST_DEFINE(index, counter_rtl87x2g_timer_pm_action);    \
     DEVICE_DT_INST_DEFINE(index, counter_rtl87x2g_timer_init, PM_DEVICE_DT_INST_GET(index),                \
                          &counter_rtl87x2g_data_##index, &counter_rtl87x2g_config_##index,              \
                          PRE_KERNEL_1, CONFIG_COUNTER_INIT_PRIORITY,      \
                          &counter_rtl87x2g_timer_driver_api);    \

DT_INST_FOREACH_STATUS_OKAY(RTL87X2G_TIMER_INIT);
