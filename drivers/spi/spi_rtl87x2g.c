/*
 * Copyright(c) 2020, Realtek Semiconductor Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT realtek_rtl87x2g_spi

#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/pinctrl.h>
#include <soc.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/rtl87x2g_clock_control.h>
#ifdef CONFIG_SPI_RTL87X2G_DMA
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/dma/dma_rtl87x2g.h>
#endif
#include <zephyr/irq.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/policy.h>

#include <rtl_spi.h>
#include <rtl_rcc.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(spi_rtl87x2g, CONFIG_SPI_LOG_LEVEL);
#include "spi_context.h"
#include "trace.h"
#ifdef CONFIG_SPI_RTL87X2G_DMA

struct spi_rtl87x2g_dma_data
{
    struct dma_config config;
    struct dma_block_config block;
    uint32_t count;
};

struct spi_dma_stream
{
    const struct device *dma_dev;
    uint32_t dma_channel;
    struct dma_config dma_cfg;
    uint8_t priority;
    uint8_t src_addr_increment;
    uint8_t dst_addr_increment;
    int fifo_threshold;
    struct dma_block_config blk_cfg;
    uint8_t *buffer;
    size_t buffer_length;
    size_t offset;
    volatile size_t counter;
    int32_t timeout;
    struct k_work_delayable timeout_work;
    bool enabled;
};

#endif

struct spi_rtl87x2g_data
{
    struct spi_context ctx;
    const struct device *dev;
    bool   initialized;
    uint32_t datasize;
#ifdef CONFIG_SPI_RTL87X2G_DMA
    struct spi_dma_stream dma_rx;
    struct spi_dma_stream dma_tx;
    struct spi_rtl87x2g_dma_data dma_rx_data;
    struct spi_rtl87x2g_dma_data dma_tx_data;
#endif
#ifdef CONFIG_PM_DEVICE
    SPIStoreReg_Typedef store_buf;
#endif
};

struct spi_rtl87x2g_config
{
    uint32_t reg;
    uint16_t clkid;
    const struct pinctrl_dev_config *pcfg;
#ifdef CONFIG_SPI_RTL87X2G_INTERRUPT
    void (*irq_configure)();
#endif
};

static bool spi_rtl87x2g_transfer_ongoing(struct spi_rtl87x2g_data *data)
{
    return spi_context_tx_on(&data->ctx) ||
           spi_context_rx_on(&data->ctx);
}

static int spi_rtl87x2g_get_err(const struct spi_rtl87x2g_config *cfg)
{
    SPI_TypeDef *spi = (SPI_TypeDef *)cfg->reg;

    if (SPI_GetFlagState(spi, SPI_FLAG_DCOL))
    {
        LOG_ERR("spi%p Data Collision Error status detected", spi);

        return -EIO;
    }

    return 0;
}

static int spi_rtl87x2g_frame_exchange(const struct device *dev)
{
    struct spi_rtl87x2g_data *data = dev->data;
    const struct spi_rtl87x2g_config *dev_config = dev->config;
    SPI_TypeDef *spi = (SPI_TypeDef *)dev_config->reg;
    struct spi_context *ctx = &data->ctx;
    uint32_t datalen = data->datasize;
    int dfs = ((datalen - 1) >> 3) + 1;
    uint16_t tx_frame = 0U, rx_frame = 0U;

    while (SPI_GetFlagState(spi, SPI_FLAG_TFE) == 0)
    {
        /* NOP */
    }

    if (spi_context_tx_buf_on(ctx))
    {
        if (datalen <= 8)
        {
            tx_frame = ctx->tx_buf ? *(uint8_t *)(data->ctx.tx_buf) : 0;
        }
        else if (datalen <= 16)
        {
            tx_frame = ctx->tx_buf ? *(uint16_t *)(data->ctx.tx_buf) : 0;
        }
        else if (datalen <= 32)
        {
            tx_frame = ctx->tx_buf ? *(uint32_t *)(data->ctx.tx_buf) : 0;
        }
    }

    SPI_SendData(spi, tx_frame);

    spi_context_update_tx(ctx, dfs, 1);

    while (SPI_GetFlagState(spi, SPI_FLAG_RFNE) == 0)
    {
        /* NOP */
    }

    rx_frame = SPI_ReceiveData(spi);

    if (spi_context_rx_buf_on(ctx))
    {
        if (datalen <= 8)
        {
            *(uint8_t *)data->ctx.rx_buf = rx_frame;
        }
        else if (datalen <= 16)
        {
            *(uint16_t *)data->ctx.rx_buf = rx_frame;
        }
        else if (datalen <= 32)
        {
            *(uint32_t *)data->ctx.rx_buf = rx_frame;
        }
    }

    spi_context_update_rx(ctx, dfs, 1);

    return spi_rtl87x2g_get_err(dev_config);
}

#ifdef CONFIG_SPI_RTL87X2G_INTERRUPT
static void spi_rtl87x2g_complete(const struct device *dev, int status)
{
    struct spi_rtl87x2g_data *dev_data = dev->data;
    const struct spi_rtl87x2g_config *dev_config = dev->config;
    SPI_TypeDef *spi = (SPI_TypeDef *)dev_config->reg;

    SPI_INTConfig(spi, SPI_INT_TXE | SPI_INT_RXF, DISABLE);

#ifdef CONFIG_SPI_RTL87X2G_DMA
    dma_stop(dev_data->dma_tx.dma_dev, dev_data->dma_tx.dma_channel);
    dma_stop(dev_data->dma_rx.dma_dev, dev_data->dma_rx.dma_channel);
#endif

    spi_context_complete(&dev_data->ctx, dev, status);
}

static void spi_rtl87x2g_isr(struct device *dev)
{
    const struct spi_rtl87x2g_config *cfg = dev->config;
    struct spi_rtl87x2g_data *data = dev->data;
    int err = 0;

    err = spi_rtl87x2g_get_err(cfg);
    if (err)
    {
        spi_rtl87x2g_complete(dev, err);
        return;
    }

    if (spi_rtl87x2g_transfer_ongoing(data))
    {
        err = spi_rtl87x2g_frame_exchange(dev);
    }

    if (err || !spi_rtl87x2g_transfer_ongoing(data))
    {
        spi_rtl87x2g_complete(dev, err);
    }
}

#endif /* CONFIG_SPI_RTL87X2G_INTERRUPT */

#ifdef CONFIG_SPI_RTL87X2G_DMA
static int spi_rtl87x2g_start_dma_transceive(const struct device *dev)
{
    const struct spi_rtl87x2g_config *cfg = dev->config;
    struct spi_rtl87x2g_data *data = dev->data;
    const size_t chunk_len = spi_context_max_continuous_chunk(&data->ctx);
    SPI_TypeDef *spi = (SPI_TypeDef *)cfg->reg;
    struct dma_status status;
    int ret = 0;
    dma_get_status(data->dma_rx.dma_dev, data->dma_rx.dma_channel, &status);
    if (chunk_len != data->dma_rx.counter && !status.busy)
    {
        data->dma_rx.blk_cfg.dest_address = data->ctx.rx_buf == NULL ? \
                                            (uint32_t)data->ctx.tx_buf : (uint32_t)data->ctx.rx_buf;
        data->dma_rx.blk_cfg.block_size = chunk_len;
        ret = dma_config(data->dma_rx.dma_dev, data->dma_rx.dma_channel,
                         &data->dma_rx.dma_cfg);
        if (ret < 0)
        {
            LOG_ERR("dma_config %p failed %d\n", data->dma_rx.dma_dev, ret);
            goto on_error;
        }
        ret = dma_start(data->dma_rx.dma_dev, data->dma_rx.dma_channel);
        if (ret < 0)
        {
            LOG_ERR("dma_start %p failed %d\n", data->dma_rx.dma_dev, ret);
            goto on_error;
        }

    }

    dma_get_status(data->dma_tx.dma_dev, data->dma_tx.dma_channel, &status);
    if (chunk_len != data->dma_tx.counter && !status.busy)
    {
        data->dma_tx.blk_cfg.source_address = (uint32_t)data->ctx.tx_buf;
        data->dma_tx.blk_cfg.block_size = chunk_len;

        ret = dma_config(data->dma_tx.dma_dev, data->dma_tx.dma_channel,
                         &data->dma_tx.dma_cfg);
        if (ret < 0)
        {
            LOG_ERR("dma_config %p failed %d\n", data->dma_tx.dma_dev, ret);
            goto on_error;
        }
        ret = dma_start(data->dma_tx.dma_dev, data->dma_tx.dma_channel);
        if (ret < 0)
        {
            LOG_ERR("dma_start %p failed %d\n", data->dma_tx.dma_dev, ret);
            goto on_error;
        }
    }

    SPI_Cmd(spi, ENABLE);

on_error:
    if (ret < 0)
    {
        dma_stop(data->dma_rx.dma_dev, data->dma_rx.dma_channel);
        dma_stop(data->dma_tx.dma_dev, data->dma_tx.dma_channel);
    }
    return ret;
}

static bool spi_rtl87x2g_chunk_transfer_finished(const struct device *dev)
{
    struct spi_rtl87x2g_data *data = dev->data;
    const size_t chunk_len = spi_context_max_continuous_chunk(&data->ctx);

    return (MIN(data->dma_tx.counter, data->dma_rx.counter) >= chunk_len);
}
void spi_rtl87x2g_dma_rx_cb(const struct device *dev, void *user_data,
                            uint32_t channel, int status)
{

    const struct device *dma_dev = (const struct device *)dev;
    const struct device *spi_dev = (const struct device *)user_data;
    struct spi_rtl87x2g_data *data = spi_dev->data;
    const size_t chunk_len = spi_context_max_continuous_chunk(&data->ctx);
    uint32_t datalen = data->datasize;
    int dfs = ((datalen - 1) >> 3) + 1;
    int err = 0;

    if (status < 0)
    {
        LOG_ERR("dma:%p ch:%d callback gets error: %d", dma_dev, channel,
                status);
        spi_rtl87x2g_complete(spi_dev, status);
        return;
    }

    data->dma_tx.counter += chunk_len;
    data->dma_rx.counter += chunk_len;

    if (spi_rtl87x2g_chunk_transfer_finished(spi_dev))
    {
        spi_context_update_tx(&data->ctx, dfs, chunk_len);
        spi_context_update_rx(&data->ctx, dfs, chunk_len);
        if (spi_rtl87x2g_transfer_ongoing(data))
        {
            /* Next chunk is available, reset the count and
             * continue processing
             */
            data->dma_tx.counter = 0;
            data->dma_rx.counter = 0;
        }
        else
        {
            /* All data is processed, complete the process */
            spi_context_complete(&data->ctx, spi_dev, 0);
            return;
        }
    }

    err = spi_rtl87x2g_start_dma_transceive(spi_dev);
    if (err)
    {
        spi_rtl87x2g_complete(spi_dev, err);
    }
}

void spi_rtl87x2g_dma_tx_cb(const struct device *dev, void *user_data,
                            uint32_t channel, int status)
{
}

#endif /* CONFIG_SPI_RTL87X2G_DMA */

static int spi_rtl87x2g_configure(const struct device *dev,
                                  const struct spi_config *spi_cfg)
{
    struct spi_rtl87x2g_data *data = dev->data;
    const struct spi_rtl87x2g_config *config = dev->config;
    SPI_TypeDef *spi = (SPI_TypeDef *)config->reg;
    struct spi_context *ctx = &data->ctx;
    SPI_InitTypeDef spi_init_struct;
    uint32_t bus_freq;
#ifdef CONFIG_SPI_RTL87X2G_DMA
    int dma_datasize;
#endif
    if (data->initialized && spi_context_configured(ctx, spi_cfg))
    {
        /* Already configured. No need to do it again. */
        return 0;
    }

    if (SPI_OP_MODE_GET(spi_cfg->operation) != SPI_OP_MODE_MASTER)
    {
        LOG_ERR("Slave mode is not supported on %s", dev->name);
        return -EINVAL;
    }

    if (spi_cfg->operation & SPI_MODE_LOOP)
    {
        LOG_ERR("Loopback mode is not supported");
        return -EINVAL;
    }

    if (spi_cfg->operation & SPI_TRANSFER_LSB)
    {
        LOG_ERR("LSB mode is supported");
        return -EINVAL;
    }

    if (IS_ENABLED(CONFIG_SPI_EXTENDED_MODES) &&
        (spi_cfg->operation & SPI_LINES_MASK) != SPI_LINES_SINGLE)
    {
        LOG_ERR("Only single line mode is supported");
        return -EINVAL;
    }

    if (data->initialized)
    {
        SPI_Cmd(spi, DISABLE);
        data->initialized = false;
    }

    SPI_StructInit(&spi_init_struct);
    bus_freq = 40000000;
    spi_init_struct.SPI_BaudRatePrescaler = bus_freq / spi_cfg->frequency;
    spi_init_struct.SPI_DataSize = SPI_WORD_SIZE_GET(spi_cfg->operation) - 1;
    spi_init_struct.SPI_CPOL = spi_cfg->operation & SPI_MODE_CPOL ? SPI_CPOL_High : SPI_CPOL_Low;
    spi_init_struct.SPI_CPHA = spi_cfg->operation & SPI_MODE_CPHA ? SPI_CPHA_2Edge : SPI_CPHA_1Edge;
#ifdef CONFIG_SPI_RTL87X2G_DMA
    if (data->dma_rx.dma_dev != NULL)
    {
        spi_init_struct.SPI_RxDmaEn = ENABLE;
    }

    if (data->dma_tx.dma_dev != NULL)
    {
        spi_init_struct.SPI_TxDmaEn = ENABLE;
    }
#endif
    SPI_Init(spi, &spi_init_struct);

#ifdef CONFIG_SPI_RTL87X2G_DMA
    dma_datasize = ((spi_init_struct.SPI_DataSize >> 3) + 1) >> 1;
    data->dma_rx.dma_cfg.source_data_size = dma_datasize;
    data->dma_rx.dma_cfg.dest_data_size = dma_datasize;
    data->dma_tx.dma_cfg.source_data_size = dma_datasize;
    data->dma_tx.dma_cfg.dest_data_size = dma_datasize;
#endif
    data->datasize = SPI_WORD_SIZE_GET(spi_cfg->operation);
    data->initialized = true;

    ctx->config = spi_cfg;

    return 0;
}

static int spi_rtl87x2g_transceive_impl(const struct device *dev,
                                        const struct spi_config *spi_cfg,
                                        const struct spi_buf_set *tx_bufs,
                                        const struct spi_buf_set *rx_bufs,
                                        bool asynchronous,
                                        spi_callback_t cb,
                                        void *userdata)
{
    struct spi_rtl87x2g_data *data = dev->data;
    const struct spi_rtl87x2g_config *config = dev->config;
    SPI_TypeDef *spi = (SPI_TypeDef *)config->reg;
    int ret;

    spi_context_lock(&data->ctx, asynchronous, cb, userdata, spi_cfg);
    ret = spi_rtl87x2g_configure(dev, spi_cfg);
    if (ret < 0)
    {
        goto error;
    }

    SPI_Cmd(spi, ENABLE);

    spi_context_buffers_setup(&data->ctx, tx_bufs, rx_bufs, \
                              ((data->datasize - 1) >> 3) + 1);

    spi_context_cs_control(&data->ctx, true);

#ifdef CONFIG_SPI_RTL87X2G_INTERRUPT
#ifdef CONFIG_SPI_RTL87X2G_DMA
    if (data->dma_rx.dma_dev && data->dma_tx.dma_dev)
    {
        data->dma_rx.counter = 0;
        data->dma_tx.counter = 0;

        ret = spi_rtl87x2g_start_dma_transceive(dev);
        if (ret < 0)
        {
            goto dma_error;
        }

    }
    else
#endif
    {
        SPI_INTConfig(spi, SPI_INT_TXE | SPI_INT_RXF, ENABLE);
    }

    ret = spi_context_wait_for_completion(&data->ctx);
#else
    do
    {
        ret = spi_rtl87x2g_frame_exchange(dev);
        if (ret < 0)
        {
            break;
        }
    }
    while (spi_rtl87x2g_transfer_ongoing(data));

#ifdef CONFIG_SPI_ASYNC
    spi_context_complete(&data->ctx, dev, ret);
#endif
#endif

    while (!SPI_GetFlagState(spi, SPI_FLAG_TFE) ||
           SPI_GetFlagState(spi, SPI_FLAG_BUSY))
    {
        /* Wait until last frame transfer complete. */
    }

#ifdef CONFIG_SPI_RTL87X2G_DMA
dma_error:
#endif
    spi_context_cs_control(&data->ctx, false);

    SPI_Cmd(spi, DISABLE);

error:
    spi_context_release(&data->ctx, ret);

    return ret;
}

static int spi_rtl87x2g_transceive(const struct device *dev,
                                   const struct spi_config *spi_cfg,
                                   const struct spi_buf_set *tx_bufs,
                                   const struct spi_buf_set *rx_bufs)
{
    return spi_rtl87x2g_transceive_impl(dev, spi_cfg, tx_bufs, rx_bufs, false, NULL, NULL);
}

#ifdef CONFIG_SPI_ASYNC
static int spi_rtl87x2g_transceive_async(const struct device *dev,
                                         const struct spi_config *spi_cfg,
                                         const struct spi_buf_set *tx_bufs,
                                         const struct spi_buf_set *rx_bufs,
                                         spi_callback_t cb,
                                         void *userdata)
{
    return spi_rtl87x2g_transceive_impl(dev, spi_cfg, tx_bufs, rx_bufs, true, cb, userdata);
}
#endif

static int spi_rtl87x2g_release(const struct device *dev,
                                const struct spi_config *config)
{
    struct spi_rtl87x2g_data *data = dev->data;

    spi_context_unlock_unconditionally(&data->ctx);

    return 0;
}

#ifdef CONFIG_PM_DEVICE
static int spi_rtl87x2g_pm_action(const struct device *dev,
                                  enum pm_device_action action)
{
    struct spi_rtl87x2g_data *data = dev->data;
    const struct spi_rtl87x2g_config *config = dev->config;
    SPI_TypeDef *spi = (SPI_TypeDef *)config->reg;
    int err;
    extern void SPI_DLPSEnter(void *PeriReg, void *StoreBuf);
    extern void SPI_DLPSExit(void *PeriReg, void *StoreBuf);

    switch (action)
    {
    case PM_DEVICE_ACTION_SUSPEND:

        SPI_DLPSEnter(spi, &data->store_buf);
        // DBG_DIRECT("spi suspend");
        /* Move pins to sleep state */
        err = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_SLEEP);
        if ((err < 0) && (err != -ENOENT))
        {
            return err;
        }
        break;
    case PM_DEVICE_ACTION_RESUME:
        /* Set pins to active state */
        // DBG_DIRECT("spi resume");
        err = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
        if (err < 0)
        {
            return err;
        }

        SPI_DLPSExit(spi, &data->store_buf);

        break;
    default:
        return -ENOTSUP;
    }

    return 0;
}
#endif /* CONFIG_PM_DEVICE */

static const struct spi_driver_api spi_rtl87x2g_driver_api =
{
    .transceive = spi_rtl87x2g_transceive,
#ifdef CONFIG_SPI_ASYNC
    .transceive_async = spi_rtl87x2g_transceive_async,
#endif
    .release = spi_rtl87x2g_release,
};

#ifdef CONFIG_SPI_RTL87X2G_DMA
static int spi_rtl87x2g_dma_init(const struct device *dev)
{
    struct spi_rtl87x2g_data *data = dev->data;
    const struct spi_rtl87x2g_config *config = dev->config;
    SPI_TypeDef *spi = (SPI_TypeDef *)config->reg;
    int ret = 0;

    if (data->dma_rx.dma_dev != NULL)
    {
        if (!device_is_ready(data->dma_rx.dma_dev))
        {
            return -ENODEV;
        }
    }

    if (data->dma_tx.dma_dev != NULL)
    {
        if (!device_is_ready(data->dma_tx.dma_dev))
        {
            return -ENODEV;
        }
    }

    /* Configure dma rx config */
    memset(&data->dma_rx.blk_cfg, 0, sizeof(data->dma_rx.blk_cfg));

    data->dma_rx.blk_cfg.source_address = (uint32_t)(&(spi->SPI_DR[0]));

    /* dest not ready */
    data->dma_rx.blk_cfg.dest_address = 0;
    data->dma_rx.blk_cfg.source_addr_adj = data->dma_rx.src_addr_increment;
    data->dma_rx.blk_cfg.dest_addr_adj = data->dma_rx.dst_addr_increment;

    data->dma_rx.blk_cfg.source_reload_en  = 0;
    data->dma_rx.blk_cfg.dest_reload_en = 0;

    data->dma_rx.dma_cfg.head_block = &data->dma_rx.blk_cfg;
    data->dma_rx.dma_cfg.user_data = (void *)dev;

    /* Configure dma tx config */
    memset(&data->dma_tx.blk_cfg, 0, sizeof(data->dma_tx.blk_cfg));

    data->dma_tx.blk_cfg.dest_address = (uint32_t)(&(spi->SPI_DR[0]));

    data->dma_tx.blk_cfg.source_address = 0; /* not ready */

    data->dma_tx.blk_cfg.source_addr_adj = data->dma_tx.src_addr_increment;

    data->dma_tx.blk_cfg.dest_addr_adj = data->dma_tx.dst_addr_increment;

    data->dma_tx.dma_cfg.head_block = &data->dma_tx.blk_cfg;
    data->dma_tx.dma_cfg.user_data = (void *)dev;

    ret = dma_config(data->dma_rx.dma_dev, data->dma_rx.dma_channel,
                     &data->dma_rx.dma_cfg);
    if (ret < 0)
    {
        return ret;
    }

    ret = dma_config(data->dma_tx.dma_dev, data->dma_tx.dma_channel,
                     &data->dma_tx.dma_cfg);
    if (ret < 0)
    {
        return ret;
    }

    return ret;
}
#endif

static int spi_rtl87x2g_init(const struct device *dev)
{
    struct spi_rtl87x2g_data *data = dev->data;
    const struct spi_rtl87x2g_config *cfg = dev->config;
    int ret;

    (void)clock_control_on(RTL87X2G_CLOCK_CONTROLLER,
                           (clock_control_subsys_t)&cfg->clkid);

    ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
    if (ret)
    {
        LOG_ERR("Failed to apply pinctrl state");
        return ret;
    }

#ifdef CONFIG_SPI_RTL87X2G_DMA
    if ((data->dma_rx.dma_dev && !data->dma_tx.dma_dev) ||
        (data->dma_tx.dma_dev && !data->dma_rx.dma_dev))
    {
        LOG_ERR("dma must be enabled for both tx and rx channels");
        return -ENODEV;
    }

    if (data->dma_rx.dma_dev && data->dma_tx.dma_dev)
    {
        ret = spi_rtl87x2g_dma_init(dev);
        if (ret < 0)
        {
            LOG_ERR("dma not ready");
        }
    }

#endif

    ret = spi_context_cs_configure_all(&data->ctx);
    if (ret < 0)
    {
        return ret;
    }

#ifdef CONFIG_SPI_RTL87X2G_INTERRUPT
    cfg->irq_configure(dev);
#endif

    spi_context_unlock_unconditionally(&data->ctx);

    return 0;
}


#define SPI_DMA_CHANNEL_INIT(index, dir)            \
    .dma_dev = DEVICE_DT_GET(RTL87X2G_DMA_CTLR(index, dir)),         \
               .dma_channel = DT_INST_DMAS_CELL_BY_NAME(index, dir, channel),          \
                              .dma_cfg = {                                  \
                                                                            .dma_slot = DT_INST_DMAS_CELL_BY_NAME(index, dir, slot),            \
                                                                            .channel_direction = RTL87X2G_DMA_CONFIG_DIRECTION(         \
                                                                                    RTL87X2G_DMA_CHANNEL_CONFIG(index, dir)),           \
                                                                            .channel_priority = RTL87X2G_DMA_CONFIG_PRIORITY(        \
                                                                                    RTL87X2G_DMA_CHANNEL_CONFIG(index, dir)),       \
                                                                            .source_data_size = RTL87X2G_DMA_CONFIG_SOURCE_DATA_SIZE(          \
                                                                                    RTL87X2G_DMA_CHANNEL_CONFIG(index, dir)),        \
                                                                            .dest_data_size = RTL87X2G_DMA_CONFIG_DESTINATION_DATA_SIZE(         \
                                                                                    RTL87X2G_DMA_CHANNEL_CONFIG(index, dir)),         \
                                                                            .source_burst_length = RTL87X2G_DMA_CONFIG_SOURCE_MSIZE(        \
                                                                                    RTL87X2G_DMA_CHANNEL_CONFIG(index, dir)),         \
                                                                            .dest_burst_length = RTL87X2G_DMA_CONFIG_DESTINATION_MSIZE(        \
                                                                                    RTL87X2G_DMA_CHANNEL_CONFIG(index, dir)),         \
                                                                            .block_count = 1,              \
                                                                            .dma_callback = spi_rtl87x2g_dma_##dir##_cb,          \
                                         },                                  \
                                         .src_addr_increment = RTL87X2G_DMA_CONFIG_SOURCE_ADDR_INC(          \
                                                               RTL87X2G_DMA_CHANNEL_CONFIG(index, dir)),          \
                                                               .dst_addr_increment = RTL87X2G_DMA_CONFIG_DESTINATION_ADDR_INC(        \
                                                                       RTL87X2G_DMA_CHANNEL_CONFIG(index, dir)),


#if defined(CONFIG_SPI_RTL87X2G_DMA)
#define SPI_DMA_CHANNEL(index, dir)    \
    .dma_##dir = {                         \
                                           COND_CODE_1(DT_INST_DMAS_HAS_NAME(index, dir),               \
                                                       (SPI_DMA_CHANNEL_INIT(index, dir)),           \
                                                       (NULL))                      \
                 },
#else
#define SPI_DMA_CHANNEL(index, dir)
#endif

#define SPI_RTL87X2G_IRQ_CONFIGURE(index)                          \
    static void spi_rtl87x2g_irq_configure_##index(void)               \
    {                                  \
        IRQ_CONNECT(DT_INST_IRQN(index), DT_INST_IRQ(index, priority), \
                    spi_rtl87x2g_isr,                  \
                    DEVICE_DT_INST_GET(index), 0);         \
        irq_enable(DT_INST_IRQN(index));                   \
    }

#define RTL87X2G_SPI_INIT(index)                           \
    PINCTRL_DT_INST_DEFINE(index);                         \
    IF_ENABLED(CONFIG_SPI_RTL87X2G_INTERRUPT, (SPI_RTL87X2G_IRQ_CONFIGURE(index)));      \
    static struct spi_rtl87x2g_data spi_rtl87x2g_data_##index = {              \
        SPI_CONTEXT_INIT_LOCK(spi_rtl87x2g_data_##index, ctx),         \
        SPI_CONTEXT_INIT_SYNC(spi_rtl87x2g_data_##index, ctx),         \
        SPI_CONTEXT_CS_GPIOS_INITIALIZE(DT_DRV_INST(index), ctx)       \
        .dev  = DEVICE_DT_INST_GET(index),                 \
                .initialized = false,                            \
                               SPI_DMA_CHANNEL(index, rx)      \
                               SPI_DMA_CHANNEL(index, tx)      \
    };                               \
    static struct spi_rtl87x2g_config spi_rtl87x2g_config_##index = {              \
        .reg = DT_INST_REG_ADDR(index),                    \
               .clkid = DT_INST_CLOCKS_CELL(index, id),                   \
                        .pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(index),             \
                                IF_ENABLED(CONFIG_SPI_RTL87X2G_INTERRUPT,                  \
                                           (.irq_configure = spi_rtl87x2g_irq_configure_##index)) }; \
     PM_DEVICE_DT_INST_DEFINE(index, spi_rtl87x2g_pm_action);    \
     DEVICE_DT_INST_DEFINE(index, &spi_rtl87x2g_init, PM_DEVICE_DT_INST_GET(index),             \
                          &spi_rtl87x2g_data_##index, &spi_rtl87x2g_config_##index,    \
                          POST_KERNEL, CONFIG_SPI_INIT_PRIORITY,           \
                          &spi_rtl87x2g_driver_api);

DT_INST_FOREACH_STATUS_OKAY(RTL87X2G_SPI_INIT)
