/*
 * Copyright(c) 2020, Realtek Semiconductor Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT realtek_rtl87x2g_usb

#include <string.h>
#include <stdio.h>

#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/irq.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/usb/usb_dc.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/rtl87x2g_clock_control.h>

#include "usb_dw_registers.h"

#include "rtl876x.h"
#include <soc.h>

#include "usb_dc_rtl87x2g.h"

#ifdef CONFIG_PM
#include "power_manager_unit_platform.h"
#endif

#include "trace.h"

#include <zephyr/logging/log.h>
#define LOG_LEVEL CONFIG_USB_DRIVER_LOG_LEVEL
LOG_MODULE_REGISTER(usb_rtl87x2g, 0);

#define DBG_DIRECT_SHOW 0

/* FIXME: The actual number of endpoints should be obtained from GHWCFG4. */
enum usb_dw_in_ep_idx {
	USB_DW_IN_EP_0 = 0,
	USB_DW_IN_EP_1,
	USB_DW_IN_EP_2,
	USB_DW_IN_EP_3,
	USB_DW_IN_EP_4,
	USB_DW_IN_EP_5,
	USB_DW_IN_EP_NUM
};

/* FIXME: The actual number of endpoints should be obtained from GHWCFG2. */
enum usb_dw_out_ep_idx {
	USB_DW_OUT_EP_0 = 0,
	USB_DW_OUT_EP_1,
	USB_DW_OUT_EP_2,
	USB_DW_OUT_EP_3,
	USB_DW_OUT_EP_4,
	USB_DW_OUT_EP_5,
	USB_DW_OUT_EP_NUM
};

#if CONFIG_USB_DC_RTL87X2G_DMA
K_HEAP_DEFINE(ep_heap, CONFIG_USB_EP_HEAP_SIZE);

#endif

#define USB_DW_CORE_RST_TIMEOUT_US 10000

/* FIXME: The actual MPS depends on endpoint type and bus speed. */
#define DW_USB_MAX_PACKET_SIZE 64

/* Number of SETUP back-to-back packets */
#define USB_DW_SUP_CNT 1

/* Get Data FIFO access register */
#define USB_DW_EP_FIFO(base, idx) (*(uint32_t *)(POINTER_TO_UINT(base) + 0x1000 * (idx + 1)))

#define USB_DW_EP_TX_FIFO(base, idx) USB_DW_EP_FIFO(base, idx)

volatile static bool usb_power_is_on = false;
struct usb_dw_config {
	struct usb_dw_reg *const base;
	void (*irq_enable_func)(const struct device *dev);
	int (*pwr_on_func)(const struct device *dev);
};

/*
 * USB endpoint private structure.
 */
struct usb_ep_ctrl_prv {
	uint8_t ep_ena;
	uint8_t fifo_num;
	uint32_t fifo_size;
	uint16_t mps;          /* Max ep pkt size */
	usb_dc_ep_callback cb; /* Endpoint callback function */
	uint32_t data_len;
	uint32_t rsvd_tx_len;
	const uint8_t *rsvd_tx_buf;
};

static void usb_dw_isr_handler(const void *unused);

/*
 * USB controller private structure.
 */
struct usb_dw_ctrl_prv {
#if CONFIG_USB_DC_RTL87X2G_DMA
	OutDmaDesc_t *volatile aOutDmaDesc[USB_DW_OUT_EP_NUM];
	InDmaDesc_t *volatile aInDmaDesc[USB_DW_IN_EP_NUM];
	inxfer_t *aInXfer[USB_DW_IN_EP_NUM];
	outxfer_t *aOutXfer[USB_DW_OUT_EP_NUM];
	uint8_t *ep_rx_buf[USB_DW_OUT_EP_NUM];
#endif
	usb_dc_status_callback status_cb;
	enum usb_dc_status_code current_status;
	struct usb_ep_ctrl_prv in_ep_ctrl[USB_DW_IN_EP_NUM];
	struct usb_ep_ctrl_prv out_ep_ctrl[USB_DW_OUT_EP_NUM];
	uint8_t attached;
};

static const uint16_t kaTxFifoBytesTable[] = {
	CONFIG_USB_DC_RTL87X2G_EP0_TX_FIFO_SIZE, CONFIG_USB_DC_RTL87X2G_EP1_TX_FIFO_SIZE,
	CONFIG_USB_DC_RTL87X2G_EP2_TX_FIFO_SIZE, CONFIG_USB_DC_RTL87X2G_EP3_TX_FIFO_SIZE,
	CONFIG_USB_DC_RTL87X2G_EP4_TX_FIFO_SIZE, CONFIG_USB_DC_RTL87X2G_EP5_TX_FIFO_SIZE};

static uint32_t kaTxFifoAddrTable[6];

#define TX_FIFO_SIZE(epidx) (kaTxFifoBytesTable[epidx] / 4)

static void usb_dw_isr_handler(const void *unused);
static void usb_dw_resume_isr_handler(const void *unused);
static void usb_isr_suspend_enable(void);
extern int hal_usb_phy_power_on(void);
extern int32_t usb_rtk_resume_sequence(void);
extern void usb_rtk_disable_power_seq(void);

#define USB_DW_DEVICE_DEFINE(n)                                                                    \
                                                                                                   \
	static void usb_dw_irq_enable_func_##n(const struct device *dev)                           \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQ_BY_IDX(0, 0, irq), DT_INST_IRQ_BY_IDX(0, 0, priority),     \
			    usb_dw_isr_handler, 0, 0);                                             \
		IRQ_CONNECT(DT_INST_IRQ_BY_IDX(0, 1, irq), DT_INST_IRQ_BY_IDX(0, 1, priority),     \
			    usb_dw_resume_isr_handler, 0, 0);                                      \
		irq_enable(DT_INST_IRQ_BY_IDX(0, 0, irq));                                         \
		irq_enable(DT_INST_IRQ_BY_IDX(0, 1, irq));                                         \
		usb_isr_suspend_enable();                                                          \
	}                                                                                          \
	static int usb_rtl87x2g_pwr_on_func##n(const struct device *dev)                           \
	{                                                                                          \
		return hal_usb_phy_power_on();                                                     \
	}                                                                                          \
	static const struct usb_dw_config usb_dw_cfg_##n = {                                       \
		.base = (struct usb_dw_reg *)DT_INST_REG_ADDR(n),                                  \
		.irq_enable_func = usb_dw_irq_enable_func_##n,                                     \
		.pwr_on_func = usb_rtl87x2g_pwr_on_func##n,                                        \
	};                                                                                         \
                                                                                                   \
	static struct usb_dw_ctrl_prv usb_dw_ctrl_##n;

USB_DW_DEVICE_DEFINE(0)

#define usb_dw_ctrl usb_dw_ctrl_0
#define usb_dw_cfg  usb_dw_cfg_0

typedef union grstctl_data {
	uint32_t d32;
	struct {
		unsigned csftrst: 1;
		unsigned hsftrst: 1;
		unsigned hstfrm: 1;
		unsigned intknqflsh: 1;
		unsigned rxfflsh: 1;
		unsigned txfflsh: 1;
		unsigned txfnum: 5;
		unsigned reserved11_28: 18;
		unsigned CSftRstDone: 1;
		unsigned dmareq: 1;
		unsigned ahbidle: 1;
	} b;
} grstctl_t;

static uint8_t usb_dw_ep_is_valid(uint8_t ep)
{
	uint8_t ep_idx = USB_EP_GET_IDX(ep);

	/* Check if ep enabled */
	if ((USB_EP_DIR_IS_OUT(ep)) && ep_idx < USB_DW_OUT_EP_NUM) {
		return 1;
	} else if ((USB_EP_DIR_IS_IN(ep)) && ep_idx < USB_DW_IN_EP_NUM) {
		return 1;
	}

	return 0;
}

static uint8_t usb_dw_ep_is_enabled(uint8_t ep)
{
	uint8_t ep_idx = USB_EP_GET_IDX(ep);

	/* Check if ep enabled */
	if ((USB_EP_DIR_IS_OUT(ep)) && usb_dw_ctrl.out_ep_ctrl[ep_idx].ep_ena) {
		return 1;
	} else if ((USB_EP_DIR_IS_IN(ep)) && usb_dw_ctrl.in_ep_ctrl[ep_idx].ep_ena) {
		return 1;
	}

	return 0;
}

static inline void usb_dw_udelay(uint32_t us)
{
	k_busy_wait(us);
}

static int usb_dw_reset(void)
{
#if DBG_DIRECT_SHOW
	DBG_DIRECT("[%s]", __func__);
#endif
	struct usb_dw_reg *const base = usb_dw_cfg.base;
	uint32_t cnt = 0U;

	/* Wait for AHB master idle susb_dw_resettate. */
	while (!(base->grstctl & USB_DW_GRSTCTL_AHB_IDLE)) {
		usb_dw_udelay(1);

		if (++cnt > USB_DW_CORE_RST_TIMEOUT_US) {
			LOG_ERR("USB reset HANG! AHB Idle GRSTCTL=0x%08x", base->grstctl);
			return -EIO;
		}
	}

	/* Core Soft Reset */
	cnt = 0U;
	base->grstctl |= USB_DW_GRSTCTL_C_SFT_RST;

	do {
		if (++cnt > USB_DW_CORE_RST_TIMEOUT_US) {
			LOG_DBG("USB reset HANG! Soft Reset GRSTCTL=0x%08x", base->grstctl);
			return -EIO;
		}
		usb_dw_udelay(1);
	} while (!(base->grstctl & (1 << 29)));
	base->grstctl |= 1 << 29;
	base->grstctl &= ~(1 << 0);
	/* Wait for 3 PHY Clocks */
	usb_dw_udelay(3000);

	return 0;
}

static int usb_dw_num_dev_eps(void)
{
	struct usb_dw_reg *const base = usb_dw_cfg.base;

	return (base->ghwcfg2 >> 10) & 0xf;
}

static void usb_dw_flush_tx_fifo(int ep)
{
	struct usb_dw_reg *const base = usb_dw_cfg.base;
	int fnum = usb_dw_ctrl.in_ep_ctrl[ep].fifo_num;

	base->grstctl = (fnum << 6) | (1 << 5);
	while (base->grstctl & (1 << 5)) {
	}
}

int usb_dw_tx_fifo_avail(int ep)
{
	struct usb_dw_reg *const base = usb_dw_cfg.base;

	return base->in_ep_reg[ep].dtxfsts & USB_DW_DTXFSTS_TXF_SPC_AVAIL_MASK;
}

/* Choose a FIFO number for an IN endpoint */
static int usb_dw_set_fifo(uint8_t ep)
{
	struct usb_dw_reg *const base = usb_dw_cfg.base;
	int ep_idx = USB_EP_GET_IDX(ep);
	volatile uint32_t *reg = &base->in_ep_reg[ep_idx].diepctl;
	uint32_t val;
	int fifo = 0;
	int ded_fifo = !!(base->ghwcfg4 & USB_DW_GHWCFG4_DEDFIFOMODE);

	if (!ded_fifo) {
		/* No support for shared-FIFO mode yet, existing
		 * Zephyr hardware doesn't use it
		 */
		return -ENOTSUP;
	}

	/* In dedicated-FIFO mode, all IN endpoints must have a unique
	 * FIFO number associated with them in the TXFNUM field of
	 * DIEPCTLx, with EP0 always being assigned to FIFO zero (the
	 * reset default, so we don't touch it).
	 *
	 * FIXME: would be better (c.f. the dwc2 driver in Linux) to
	 * choose a FIFO based on the hardware depth: we want the
	 * smallest one that fits our configured maximum packet size
	 * for the endpoint.  This just picks the next available one.
	 */
	if (ep_idx != 0) {
		fifo = ep_idx;
		if (fifo >= usb_dw_num_dev_eps()) {
			return -EINVAL;
		}

		reg = &base->in_ep_reg[ep_idx].diepctl;
		val = *reg & ~USB_DW_DEPCTL_TXFNUM_MASK;
		val |= fifo << USB_DW_DEPCTL_TXFNUM_OFFSET;
		*reg = val;
	}

	usb_dw_ctrl.in_ep_ctrl[ep_idx].fifo_num = fifo;

	usb_dw_flush_tx_fifo(ep_idx);

	val = usb_dw_tx_fifo_avail(ep_idx);
	usb_dw_ctrl.in_ep_ctrl[ep_idx].fifo_size = val;

	return 0;
}

static int usb_dw_ep_set(uint8_t ep, uint32_t ep_mps, enum usb_dc_ep_transfer_type ep_type)
{
	struct usb_dw_reg *const base = usb_dw_cfg.base;
	volatile uint32_t *p_depctl;
	uint8_t ep_idx = USB_EP_GET_IDX(ep);

	LOG_DBG("%s ep %x, mps %d, type %d", __func__, ep, ep_mps, ep_type);

	if (USB_EP_DIR_IS_OUT(ep)) {
		p_depctl = &base->out_ep_reg[ep_idx].doepctl;
		usb_dw_ctrl.out_ep_ctrl[ep_idx].mps = ep_mps;
	} else {
		p_depctl = &base->in_ep_reg[ep_idx].diepctl;
		usb_dw_ctrl.in_ep_ctrl[ep_idx].mps = ep_mps;
	}

	if (!ep_idx) {
		/* Set max packet size for EP0 */
		*p_depctl &= ~USB_DW_DEPCTL0_MSP_MASK;

		switch (ep_mps) {
		case 8:
			*p_depctl |= USB_DW_DEPCTL0_MSP_8 << USB_DW_DEPCTL_MSP_OFFSET;
			break;
		case 16:
			*p_depctl |= USB_DW_DEPCTL0_MSP_16 << USB_DW_DEPCTL_MSP_OFFSET;
			break;
		case 32:
			*p_depctl |= USB_DW_DEPCTL0_MSP_32 << USB_DW_DEPCTL_MSP_OFFSET;
			break;
		case 64:
			*p_depctl |= USB_DW_DEPCTL0_MSP_64 << USB_DW_DEPCTL_MSP_OFFSET;
			break;
		default:
			return -EINVAL;
		}
		/* No need to set EP0 type */
	} else {
		/* Set max packet size for EP */
		if (ep_mps > (USB_DW_DEPCTLn_MSP_MASK >> USB_DW_DEPCTL_MSP_OFFSET)) {
			return -EINVAL;
		}

		*p_depctl &= ~USB_DW_DEPCTLn_MSP_MASK;
		*p_depctl |= ep_mps << USB_DW_DEPCTL_MSP_OFFSET;

		/* Set endpoint type */
		*p_depctl &= ~USB_DW_DEPCTL_EP_TYPE_MASK;

		switch (ep_type) {
		case USB_DC_EP_CONTROL:
			*p_depctl |= USB_DW_DEPCTL_EP_TYPE_CONTROL << USB_DW_DEPCTL_EP_TYPE_OFFSET;
			break;
		case USB_DC_EP_BULK:
			*p_depctl |= USB_DW_DEPCTL_EP_TYPE_BULK << USB_DW_DEPCTL_EP_TYPE_OFFSET;
			break;
		case USB_DC_EP_INTERRUPT:
			*p_depctl |= USB_DW_DEPCTL_EP_TYPE_INTERRUPT
				     << USB_DW_DEPCTL_EP_TYPE_OFFSET;
			break;
		default:
			return -EINVAL;
		}

		/* sets the Endpoint Data PID to DATA0 */
		*p_depctl |= USB_DW_DEPCTL_SETDOPID;
	}

	if (USB_EP_DIR_IS_IN(ep)) {
		int ret = usb_dw_set_fifo(ep);

		if (ret) {
			return ret;
		}
	}

	return 0;
}

static void usb_dw_flush_all_fifo(void)
{
	struct usb_dw_reg *const base = usb_dw_cfg.base;
	grstctl_t grstctl = {.b = {.txfnum = 0x10, .txfflsh = 1, .rxfflsh = 1}};

	base->grstctl = grstctl.d32;
	do {
		grstctl.d32 = base->grstctl;
	} while (grstctl.b.txfflsh || grstctl.b.rxfflsh);
}

static void usb_dw_resize_fifo(void)
{
	struct usb_dw_reg *const base = usb_dw_cfg.base;

	/* 1. Get Tx FIFO size, and caculate Rx FIFO size. */
	uint32_t total_fifo_size = base->ghwcfg3 >> 16;
	uint32_t total_tx_fifo_size = 0;

	for (uint8_t epidx = 0; epidx < 6; ++epidx) {
		total_tx_fifo_size += TX_FIFO_SIZE(epidx);
	}

	uint32_t rx_fifo_size = total_fifo_size - total_tx_fifo_size;

	/* 2. Write back. */
	base->grxfsiz = rx_fifo_size;

	typedef union fifosize_data {
		/* raw register data */
		uint32_t d32;
		/* register bits */
		struct {
			unsigned startaddr: 16;
			unsigned depth: 16;
		} b;
	} fifosize_data_t;
	fifosize_data_t curr_fifo, last_fifo;

	curr_fifo.d32 = base->gnptxfsiz;
	curr_fifo.b.startaddr = rx_fifo_size;
	curr_fifo.b.depth = TX_FIFO_SIZE(0);
	base->gnptxfsiz = curr_fifo.d32;

	last_fifo = curr_fifo;

	for (uint8_t epidx = 1; epidx < 6; ++epidx) {
		kaTxFifoAddrTable[epidx] = curr_fifo.b.depth + kaTxFifoAddrTable[epidx - 1];
		curr_fifo.d32 = (&base->dieptxf1)[epidx - 1];
		curr_fifo.b.startaddr = last_fifo.b.startaddr + last_fifo.b.depth;
		curr_fifo.b.depth = TX_FIFO_SIZE(epidx);
		(&base->dieptxf1)[epidx - 1] = curr_fifo.d32;
		last_fifo = curr_fifo;
	}

	usb_dw_flush_all_fifo();

	/* 4. Link FIFO to IN ep. */
	for (uint8_t epidx = 0; epidx < 6; ++epidx) {
		uint32_t diepctl = base->in_ep_reg[epidx].diepctl;

		diepctl &= (~((BIT4 - 1) << 22));
		diepctl |= (epidx << 22);
		base->in_ep_reg[epidx].diepctl = diepctl;
	}
}

static int usb_rtk_init(void)
{
#if CONFIG_USB_DC_RTL87X2G_DMA
	extern void hal_rtk_usb_init_dma(void);
	hal_rtk_usb_init_dma();
#else
	extern void hal_rtk_usb_init(void);
	hal_rtk_usb_init();
#endif
	return 0;
}

static void usb_dw_handle_reset(void)
{
#if CONFIG_USB_DC_RTL87X2G_DMA
	struct usb_dw_reg *const base = usb_dw_cfg.base;

	usb_dw_ctrl.current_status = USB_DC_RESET;
	/* Inform upper layers */
	if (usb_dw_ctrl.status_cb) {
		usb_dw_ctrl.status_cb(USB_DC_RESET, NULL);
	}

	usb_dw_reset();
	usb_rtk_init();
	usb_dw_resize_fifo();

#else
	struct usb_dw_reg *const base = usb_dw_cfg.base;

	usb_dw_ctrl.current_status = USB_DC_RESET;
	/* Inform upper layers */
	if (usb_dw_ctrl.status_cb) {
		usb_dw_ctrl.status_cb(USB_DC_RESET, NULL);
	}

	usb_dw_resize_fifo();

	/* Clear device address during reset. */
	base->dcfg &= ~USB_DW_DCFG_DEV_ADDR_MASK;

	/* enable global EP interrupts */
	base->doepmsk = 0U;
	base->gintmsk |= USB_DW_GINTSTS_RX_FLVL;
	base->diepmsk |= USB_DW_DIEPINT_XFER_COMPL;
#endif
}

#if CONFIG_USB_DC_RTL87X2G_DMA
static uint32_t usb_get_in_xfer_max_bytes(uint8_t ep_idx)
{
	uint16_t ep_mps = usb_dw_ctrl.in_ep_ctrl[ep_idx].mps;

	return 0xffff / ep_mps * ep_mps;
}

static int usb_ep_dma_send(uint8_t ep_idx)
{
	struct usb_dw_reg *const base = usb_dw_cfg.base;

	usb_dw_ctrl.aInXfer[ep_idx]->BytesSent += usb_dw_ctrl.aInXfer[ep_idx]->BytesSending;
	usb_dw_ctrl.aInXfer[ep_idx]->BytesSending = 0;
	if (usb_dw_ctrl.aInXfer[ep_idx]->BytesSent < usb_dw_ctrl.aInXfer[ep_idx]->BytesTotal) {
		const uint16_t ep_mps = usb_dw_ctrl.in_ep_ctrl[ep_idx].mps;
		uint32_t bytes_residual = usb_dw_ctrl.aInXfer[ep_idx]->BytesTotal -
					  usb_dw_ctrl.aInXfer[ep_idx]->BytesSent;
		uint32_t xfer_bytes = usb_get_in_xfer_max_bytes(ep_idx) < bytes_residual
					      ? usb_get_in_xfer_max_bytes(ep_idx)
					      : bytes_residual;
		volatile InDmaDesc_t *pDmaDesc = usb_dw_ctrl.aInDmaDesc[ep_idx];

		pDmaDesc->Quad.NonIso.TxBytes = xfer_bytes;
		pDmaDesc->Quad.NonIso.Reserved0 = 0;
		pDmaDesc->Quad.NonIso.InterruptOnComplete = 1;
		pDmaDesc->Quad.NonIso.ShortPacket = (xfer_bytes % ep_mps != 0);
		pDmaDesc->Quad.NonIso.Last = 1;
		pDmaDesc->Quad.NonIso.TransmitState = QUAD_TXSTS_SUCCESS;
		pDmaDesc->Quad.NonIso.BufferState = QUAD_BS_HOST_READY;

		pDmaDesc->BufAddr = (uint32_t)usb_dw_ctrl.aInXfer[ep_idx]->pBuf +
				    usb_dw_ctrl.aInXfer[ep_idx]->BytesSent;

		base->in_ep_reg[ep_idx].diepdma = (uint32_t)pDmaDesc;
		base->in_ep_reg[ep_idx].diepctl |= USB_DW_DEPCTL_EP_ENA | BIT26;

		usb_dw_ctrl.aInXfer[ep_idx]->BytesSending = xfer_bytes;
		return xfer_bytes;
	} else if (usb_dw_ctrl.aInXfer[ep_idx]->ToSendZlp) {
		/* All data has been sent, and only zlp is waiting to send. Send zlp. */
		volatile InDmaDesc_t *pDmaDesc = usb_dw_ctrl.aInDmaDesc[ep_idx];

		pDmaDesc->Quad.NonIso.TxBytes = 0;
		pDmaDesc->Quad.NonIso.Reserved0 = 0;
		pDmaDesc->Quad.NonIso.InterruptOnComplete = 1;
		pDmaDesc->Quad.NonIso.ShortPacket = 1;
		pDmaDesc->Quad.NonIso.Last = 1;
		pDmaDesc->Quad.NonIso.TransmitState = QUAD_TXSTS_SUCCESS;
		pDmaDesc->Quad.NonIso.BufferState = QUAD_BS_HOST_READY;

		pDmaDesc->BufAddr = (uint32_t)usb_dw_ctrl.aInXfer[ep_idx]->pBuf +
				    usb_dw_ctrl.aInXfer[ep_idx]->BytesSent;

		base->in_ep_reg[ep_idx].diepdma = (uint32_t)pDmaDesc;
		base->in_ep_reg[ep_idx].diepctl |= USB_DW_DEPCTL_EP_ENA | BIT26;

		usb_dw_ctrl.aInXfer[ep_idx]->ToSendZlp = false;
	}

	return 0;
}

static int usb_dw_tx_dma(uint8_t ep, const uint8_t *const data, uint32_t data_len)
{
	enum usb_dw_in_ep_idx ep_idx = USB_EP_GET_IDX(ep);

	usb_dw_ctrl.aInXfer[ep_idx]->pBuf = data;
	usb_dw_ctrl.aInXfer[ep_idx]->BytesTotal = data_len;
	usb_dw_ctrl.aInXfer[ep_idx]->BytesSent = 0;
	usb_dw_ctrl.aInXfer[ep_idx]->BytesSending = 0;
	usb_dw_ctrl.aInXfer[ep_idx]->ToSendZlp = data_len == 0;

	return usb_ep_dma_send(ep_idx);
}

static uint32_t usb_get_out_xfer_max_bytes(uint8_t ep_idx)
{
	uint16_t ep_mps = usb_dw_ctrl.out_ep_ctrl[ep_idx].mps;

	return (ep_idx == 0) ? ep_mps : (0xffff / ep_mps * ep_mps);
}

static int usb_ep_dma_receive(uint8_t ep_idx)
{
	struct usb_dw_reg *const base = usb_dw_cfg.base;
	const uint16_t ep_mps = usb_dw_ctrl.out_ep_ctrl[ep_idx].mps;
	volatile OutDmaDesc_t *pDmaDesc = usb_dw_ctrl.aOutDmaDesc[ep_idx];
	bool MoreDataToRcv;

	if (usb_dw_ctrl.aOutXfer[ep_idx]->BytesRcving == 0) {
		MoreDataToRcv = true;
	} else {
		uint32_t BytesRcvedThisCycle =
			usb_dw_ctrl.aOutXfer[ep_idx]->BytesRcving - pDmaDesc->Quad.NonIso.RxBytes;

		usb_dw_ctrl.aOutXfer[ep_idx]->BytesRcved += BytesRcvedThisCycle;
		usb_dw_ctrl.aOutXfer[ep_idx]->BytesRcving = 0;
		if (usb_dw_ctrl.aOutXfer[ep_idx]->BytesRcved ==
			    usb_dw_ctrl.aOutXfer[ep_idx]->BytesTotal ||
		    pDmaDesc->Quad.NonIso.ShortPacket) {
			/* All bytes were received, or short packet was received. Out xfer complete.
			 */
			MoreDataToRcv = false;
		} else {
			MoreDataToRcv = true;
		}
	}

	if (MoreDataToRcv) {
		uint32_t bytes_residual = usb_dw_ctrl.aOutXfer[ep_idx]->BytesTotal -
					  usb_dw_ctrl.aOutXfer[ep_idx]->BytesRcved;
		uint32_t xfer_bytes = usb_get_out_xfer_max_bytes(ep_idx) < bytes_residual
					      ? usb_get_out_xfer_max_bytes(ep_idx)
					      : bytes_residual;

		usb_dw_ctrl.aOutXfer[ep_idx]->BytesRcving =
			(xfer_bytes + ep_mps - 1) / ep_mps * ep_mps;

		pDmaDesc->Quad.NonIso.RxBytes = usb_dw_ctrl.aOutXfer[ep_idx]->BytesRcving;
		pDmaDesc->Quad.NonIso.Reserved0 = 0;
		pDmaDesc->Quad.NonIso.MTRF = 0;
		pDmaDesc->Quad.NonIso.SR = 0;
		pDmaDesc->Quad.NonIso.InterruptOnComplete = 1;
		pDmaDesc->Quad.NonIso.ShortPacket = 0;
		pDmaDesc->Quad.NonIso.Last = 1;
		pDmaDesc->Quad.NonIso.ReceiveState = QUAD_RXSTS_SUCCESS;
		pDmaDesc->Quad.NonIso.BufferState = QUAD_BS_HOST_READY;

		pDmaDesc->BufAddr = (uint32_t)usb_dw_ctrl.aOutXfer[ep_idx]->pBuf +
				    usb_dw_ctrl.aOutXfer[ep_idx]->BytesRcved;

		base->out_ep_reg[ep_idx].doepdma = (uint32_t)pDmaDesc;

		base->out_ep_reg[ep_idx].doepctl |= USB_DW_DEPCTL_EP_ENA | BIT26;
	}

	return 0;
}

static int usb_dw_rx_dma(const uint8_t ep, uint8_t *data, const uint32_t read_bytes)
{
	enum usb_dw_in_ep_idx ep_idx = USB_EP_GET_IDX(ep);

	usb_dw_ctrl.aOutXfer[ep_idx]->pBuf = data;
	usb_dw_ctrl.aOutXfer[ep_idx]->BytesTotal = read_bytes;
	usb_dw_ctrl.aOutXfer[ep_idx]->BytesRcving = 0;
	usb_dw_ctrl.aOutXfer[ep_idx]->BytesRcved = 0;

	return usb_ep_dma_receive(ep_idx);
}

#else
static int usb_dw_tx(uint8_t ep, const uint8_t *const data, uint32_t data_len)
{
	struct usb_dw_reg *const base = usb_dw_cfg.base;
	enum usb_dw_in_ep_idx ep_idx = USB_EP_GET_IDX(ep);
	uint32_t max_xfer_size, max_pkt_cnt, pkt_cnt, avail_space;
	uint32_t ep_mps = usb_dw_ctrl.in_ep_ctrl[ep_idx].mps;
	unsigned int key;
	uint32_t i;
	uint32_t tx_data_len = data_len;

#if DBG_DIRECT_SHOW
	DBG_DIRECT("[%s] base->in_ep_reg[%d].diepctl=0x%x", __func__, ep_idx,
		   base->in_ep_reg[ep_idx].diepctl);
#endif

	/* Wait for FIFO space available */
	do {
		avail_space = usb_dw_tx_fifo_avail(ep_idx);
		if (avail_space == usb_dw_ctrl.in_ep_ctrl[ep_idx].fifo_size) {
			break;
		}
		/* Make sure we don't hog the CPU */
		k_yield();
	} while (1);
	key = irq_lock();

	avail_space *= 4U;
	if (!avail_space) {
		LOG_ERR("USB IN EP%d no space available, DTXFSTS %x", ep_idx,
			base->in_ep_reg[ep_idx].dtxfsts);
		irq_unlock(key);
		return -EAGAIN;
	}

	/* For now tx-fifo sizes are not configured (cf usb_dw_set_fifo). Here
	 * we force available fifo size to be a multiple of ep mps in order to
	 * prevent splitting data incorrectly.
	 */
	avail_space -= avail_space % ep_mps;
	if (data_len > avail_space) {
		data_len = avail_space;
	}

	if (data_len != 0U) {
		/* Get max packet size and packet count for ep */
		if (ep_idx == USB_DW_IN_EP_0) {
			max_pkt_cnt = 1;
			max_xfer_size = ep_mps;
		} else {
			max_pkt_cnt =
				USB_DW_DIEPTSIZn_PKT_CNT_MASK >> USB_DW_DEPTSIZ_PKT_CNT_OFFSET;
			max_xfer_size =
				USB_DW_DEPTSIZn_XFER_SIZE_MASK >> USB_DW_DEPTSIZ_XFER_SIZE_OFFSET;
		}

		/* Check if transfer len is too big */
		if (data_len > max_xfer_size) {
			LOG_WRN("USB IN EP%d len too big (%d->%d)", ep_idx, data_len,
				max_xfer_size);
			data_len = max_xfer_size;
		}

		/*
		 * Program the transfer size and packet count as follows:
		 *
		 * transfer size = N * ep_maxpacket + short_packet
		 * pktcnt = N + (short_packet exist ? 1 : 0)
		 */

		pkt_cnt = DIV_ROUND_UP(data_len, ep_mps);
		if (pkt_cnt > max_pkt_cnt) {
			LOG_WRN("USB IN EP%d pkt count too big (%d->%d)", ep_idx, pkt_cnt, pkt_cnt);
			pkt_cnt = max_pkt_cnt;
			data_len = pkt_cnt * ep_mps;
		}
	} else {
		/* Zero length packet */
		pkt_cnt = 1U;
	}

	if (tx_data_len > data_len) {
		usb_dw_ctrl.in_ep_ctrl[ep_idx].rsvd_tx_len = tx_data_len - data_len;
		usb_dw_ctrl.in_ep_ctrl[ep_idx].rsvd_tx_buf = data + data_len;
	} else {
		usb_dw_ctrl.in_ep_ctrl[ep_idx].rsvd_tx_len = 0;
		usb_dw_ctrl.in_ep_ctrl[ep_idx].rsvd_tx_buf = NULL;
	}

	/* Set number of packets and transfer size */
	base->in_ep_reg[ep_idx].dieptsiz = (pkt_cnt << USB_DW_DEPTSIZ_PKT_CNT_OFFSET) | data_len;

	/* Clear NAK and enable ep */
	base->in_ep_reg[ep_idx].diepctl |= (USB_DW_DEPCTL_EP_ENA | USB_DW_DEPCTL_CNAK);

	/*
	 * Write data to FIFO, make sure that we are protected against
	 * other USB register accesses.  According to "DesignWare Cores
	 * USB 1.1/2.0 Device Subsystem-AHB/VCI Databook": "During FIFO
	 * access, the application must not access the UDC/Subsystem
	 * registers or vendor registers (for ULPI mode). After starting
	 * to access a FIFO, the application must complete the transaction
	 * before accessing the register."
	 */
	for (i = 0U; i < data_len; i += 4U) {
		uint32_t val = data[i];

		if (i + 1 < data_len) {
			val |= ((uint32_t)data[i + 1]) << 8;
		}
		if (i + 2 < data_len) {
			val |= ((uint32_t)data[i + 2]) << 16;
		}
		if (i + 3 < data_len) {
			val |= ((uint32_t)data[i + 3]) << 24;
		}
#if DBG_DIRECT_SHOW
		DBG_DIRECT("val=%x", val);
#endif
		USB_DW_EP_TX_FIFO(base, ep_idx) = val;
	}

	irq_unlock(key);

#if DBG_DIRECT_SHOW
	DBG_DIRECT("USB IN EP%d write %d bytes", ep_idx, data_len);
#endif

	return data_len;
}

#endif /* CONFIG_USB_DC_RTL87X2G_DMA */

static void usb_dw_prep_rx(const uint8_t ep, uint8_t setup)
{
#if CONFIG_USB_DC_RTL87X2G_DMA
	enum usb_dw_out_ep_idx ep_idx = USB_EP_GET_IDX(ep);
	uint32_t ep_mps = usb_dw_ctrl.out_ep_ctrl[ep_idx].mps;

	usb_dw_rx_dma(ep, usb_dw_ctrl.ep_rx_buf[ep_idx], ep_mps);

	LOG_DBG("USB OUT EP%d armed", ep_idx);

#else
	struct usb_dw_reg *const base = usb_dw_cfg.base;
	enum usb_dw_out_ep_idx ep_idx = USB_EP_GET_IDX(ep);
	uint32_t ep_mps = usb_dw_ctrl.out_ep_ctrl[ep_idx].mps;

	/* Set max RX size to EP mps so we get an interrupt
	 * each time a packet is received
	 */

	base->out_ep_reg[ep_idx].doeptsiz = (USB_DW_SUP_CNT << USB_DW_DOEPTSIZ_SUP_CNT_OFFSET) |
					    (1 << USB_DW_DEPTSIZ_PKT_CNT_OFFSET) | ep_mps;

	/* Clear NAK and enable ep */
	if (!setup) {
		base->out_ep_reg[ep_idx].doepctl |= USB_DW_DEPCTL_CNAK;
	}

	base->out_ep_reg[ep_idx].doepctl |= USB_DW_DEPCTL_EP_ENA;

	LOG_DBG("USB OUT EP%d armed", ep_idx);
#endif
}

void usb_dw_handle_enum_done(void)
{
#if CONFIG_USB_DC_RTL87X2G_DMA
	struct usb_dw_reg *const base = usb_dw_cfg.base;
	uint8_t speed;
	uint32_t ep_mps;

	speed = (base->dsts & ~USB_DW_DSTS_ENUM_SPD_MASK) >> USB_DW_DSTS_ENUM_SPD_OFFSET;

	usb_dw_ctrl.current_status = USB_DC_CONNECTED;
	/* Inform upper layers */
	if (usb_dw_ctrl.status_cb) {
		usb_dw_ctrl.status_cb(USB_DC_CONNECTED, &speed);
	}

	base->in_ep_reg[0].diepctl = (base->in_ep_reg[0].diepctl & ~USB_DW_DIEPCTL_MPS_MSAK) |
				     USB_DW_DIEPCTL_MPS_64BYTES;
	base->out_ep_reg[0].doepctl = (base->out_ep_reg[0].doepctl & ~USB_DW_DOEPCTL_MPS_MSAK) |
				      USB_DW_DOEPCTL_MPS_64BYTES;
	ep_mps = usb_dw_ctrl.out_ep_ctrl[0].mps;
	usb_dw_rx_dma(0, usb_dw_ctrl.ep_rx_buf[0], ep_mps);

#else
	struct usb_dw_reg *const base = usb_dw_cfg.base;
	uint8_t speed;

	speed = (base->dsts & USB_DW_DSTS_ENUM_SPD_MASK) >> USB_DW_DSTS_ENUM_SPD_OFFSET;

	LOG_DBG("USB ENUM DONE event, %s speed detected", speed == 0 ? "High" : "Full");
#if DBG_DIRECT_SHOW
	DBG_DIRECT("USB ENUM DONE event, %s speed detected", speed == 0 ? "High" : "Full");
#endif

	usb_dw_ctrl.current_status = USB_DC_CONNECTED;
	/* Inform upper layers */
	if (usb_dw_ctrl.status_cb) {
		usb_dw_ctrl.status_cb(USB_DC_CONNECTED, &speed);
	}
#endif
}

/* USB ISR handler */
static inline void usb_dw_int_rx_flvl_handler(void)
{
	struct usb_dw_reg *const base = usb_dw_cfg.base;
	uint32_t grxstsp = base->grxstsp;
	uint32_t status, xfer_size;
	uint8_t ep_idx;
	usb_dc_ep_callback ep_cb;

	/* Packet in RX FIFO */

	ep_idx = grxstsp & USB_DW_GRXSTSR_EP_NUM_MASK;
	status = (grxstsp & USB_DW_GRXSTSR_PKT_STS_MASK) >> USB_DW_GRXSTSR_PKT_STS_OFFSET;
	xfer_size = (grxstsp & USB_DW_GRXSTSR_PKT_CNT_MASK) >> USB_DW_GRXSTSR_PKT_CNT_OFFSET;

	LOG_DBG("USB OUT EP%u: RX_FLVL status %u, size %u", ep_idx, status, xfer_size);
#if DBG_DIRECT_SHOW
	DBG_DIRECT("USB OUT EP%d: grxstsp status %d, size %d", ep_idx, status, xfer_size);
#endif

	usb_dw_ctrl.out_ep_ctrl[ep_idx].data_len = xfer_size;
	ep_cb = usb_dw_ctrl.out_ep_ctrl[ep_idx].cb;

	switch (status) {
#if CONFIG_USB_DC_RTL87X2G_DMA
	case 12:
#endif
	case USB_DW_GRXSTSR_PKT_STS_SETUP:
		LOG_DBG("USB_DW_GRXSTSR_PKT_STS_SETUP");
#if DBG_DIRECT_SHOW
		DBG_DIRECT("USB_DW_GRXSTSR_PKT_STS_SETUP");
#endif
		/* Call the registered callback if any */
		if (ep_cb) {
			ep_cb(USB_EP_GET_ADDR(ep_idx, USB_EP_DIR_OUT), USB_DC_EP_SETUP);
		}

		break;
	case USB_DW_GRXSTSR_PKT_STS_OUT_DATA:
		LOG_DBG("USB_DW_GRXSTSR_PKT_STS_OUT_DATA");
#if DBG_DIRECT_SHOW
		DBG_DIRECT("USB_DW_GRXSTSR_PKT_STS_OUT_DATA");
#endif
		if (ep_cb) {
			ep_cb(USB_EP_GET_ADDR(ep_idx, USB_EP_DIR_OUT), USB_DC_EP_DATA_OUT);
		}

		break;
	case USB_DW_GRXSTSR_PKT_STS_OUT_DATA_DONE:
		LOG_DBG("USB_DW_GRXSTSR_PKT_STS_OUT_DATA_DONE");
#if DBG_DIRECT_SHOW
		DBG_DIRECT("USB_DW_GRXSTSR_PKT_STS_OUT_DATA_DONE");
#endif
		break;
	case USB_DW_GRXSTSR_PKT_STS_SETUP_DONE:
		LOG_DBG("USB_DW_GRXSTSR_PKT_STS_SETUP_DONE");
#if DBG_DIRECT_SHOW
		DBG_DIRECT("USB_DW_GRXSTSR_PKT_STS_SETUP_DONE");
#endif
		break;
	default:
		break;
	}
}

static inline void usb_dw_int_iep_handler(void)
{
#if CONFIG_USB_DC_RTL87X2G_DMA
	struct usb_dw_reg *const base = usb_dw_cfg.base;
	uint32_t ep_int_status;
	uint32_t ActDaint = base->daint & base->daintmsk;
	uint8_t ep_idx = 0;
	usb_dc_ep_callback ep_cb;
	uint32_t ep_mps;

	if (ActDaint & USB_DW_DAINT_IN_EP_INT(ep_idx)) {
		ep_int_status = base->in_ep_reg[ep_idx].diepint & base->diepmsk;
#if DBG_DIRECT_SHOW
		DBG_DIRECT("[%s] Ep0 diepint: 0x%x", __func__, ep_int_status);
#endif
		base->in_ep_reg[ep_idx].diepint = ep_int_status;
		ep_mps = usb_dw_ctrl.out_ep_ctrl[ep_idx].mps;
		usb_dw_rx_dma(0, usb_dw_ctrl.ep_rx_buf[ep_idx], ep_mps);

		ep_cb = usb_dw_ctrl.in_ep_ctrl[ep_idx].cb;
		if (ep_cb && (ep_int_status & USB_DW_DIEPINT_XFER_COMPL)) {
#if DBG_DIRECT_SHOW
			DBG_DIRECT("[%s] USB_DW_DIEPINT_XFER_COMPL", __func__);
#endif
			ep_cb(USB_EP_GET_ADDR(ep_idx, USB_EP_DIR_IN), USB_DC_EP_DATA_IN);
		}
	}
	for (ep_idx = 1U; ep_idx < USB_DW_IN_EP_NUM; ep_idx++) {
		if (base->daint & USB_DW_DAINT_IN_EP_INT(ep_idx)) {
			/* Read IN EP interrupt status */
			ep_int_status = base->in_ep_reg[ep_idx].diepint & base->diepmsk;
#if DBG_DIRECT_SHOW
			DBG_DIRECT("[%s] Ep%d diepint: 0x%x", __func__, ep_idx, ep_int_status);
#endif
			base->in_ep_reg[ep_idx].diepint = ep_int_status;
			ep_cb = usb_dw_ctrl.in_ep_ctrl[ep_idx].cb;
			if (ep_cb && (ep_int_status & USB_DW_DIEPINT_XFER_COMPL)) {

				/* Call the registered callback */
				ep_cb(USB_EP_GET_ADDR(ep_idx, USB_EP_DIR_IN), USB_DC_EP_DATA_IN);
			}
		}
	}
#else
	struct usb_dw_reg *const base = usb_dw_cfg.base;
	uint32_t ep_int_status;
	uint8_t ep_idx;
	usb_dc_ep_callback ep_cb;

	for (ep_idx = 0U; ep_idx < USB_DW_IN_EP_NUM; ep_idx++) {
		if (base->daint & USB_DW_DAINT_IN_EP_INT(ep_idx)) {
			/* Read IN EP interrupt status */
			ep_int_status = base->in_ep_reg[ep_idx].diepint & base->diepmsk;

			/* Clear IN EP interrupts */
			base->in_ep_reg[ep_idx].diepint = ep_int_status;

			LOG_DBG("USB IN EP%u interrupt status: 0x%x", ep_idx, ep_int_status);

			ep_cb = usb_dw_ctrl.in_ep_ctrl[ep_idx].cb;
			if (ep_cb && (ep_int_status & USB_DW_DIEPINT_XFER_COMPL)) {
				/* Call the registered callback */
				ep_cb(USB_EP_GET_ADDR(ep_idx, USB_EP_DIR_IN), USB_DC_EP_DATA_IN);
			}
		}
	}

	/* Clear interrupt. */
	base->gintsts = USB_DW_GINTSTS_IEP_INT;
#endif
}

#if CONFIG_USB_DC_RTL87X2G_DMA
static void Ep0ParseSetupPkt(void)
{
	struct usb_dw_reg *const base = usb_dw_cfg.base;
	uint8_t ep_idx;
	usb_dc_ep_callback ep_cb;
	uint32_t grxstsp = base->grxstsp;

	ep_idx = grxstsp & USB_DW_GRXSTSR_EP_NUM_MASK;
	ep_cb = usb_dw_ctrl.out_ep_ctrl[ep_idx].cb;
	ep_cb(USB_EP_GET_ADDR(ep_idx, USB_EP_DIR_OUT), USB_DC_EP_SETUP);
}
#endif

static inline void usb_dw_int_oep_handler(void)
{
	struct usb_dw_reg *const base = usb_dw_cfg.base;
	uint32_t ep_int_status;
	uint8_t ep_idx = 0;

#if CONFIG_USB_DC_RTL87X2G_DMA
	if (base->daint & USB_DW_DAINT_OUT_EP_INT(ep_idx)) {
		ep_int_status = base->out_ep_reg[ep_idx].doepint & base->doepmsk;
		base->out_ep_reg[ep_idx].doepint = ep_int_status;
#if DBG_DIRECT_SHOW
		DBG_DIRECT("[%s] Ep0 doepint: 0x%x", __func__, ep_int_status);
#endif

		if (!(ep_int_status & USB_DW_DOEPINT_SET_UP) &&
		    (ep_int_status & USB_DW_DOEPINT_XFER_COMPL)) {
#if DBG_DIRECT_SHOW
			DBG_DIRECT("[%s] EP0EVENT_OEPINT_XFERCMPL", __func__);
#endif
		} else if ((ep_int_status & USB_DW_DOEPINT_SET_UP) &&
			   !(ep_int_status & USB_DW_DOEPINT_XFER_COMPL)) {
#if DBG_DIRECT_SHOW
			DBG_DIRECT("[%s] EP0EVENT_OEPINT_SETUPDONE", __func__);
#endif
		} else if ((ep_int_status & USB_DW_DOEPINT_SET_UP) &&
			   (ep_int_status & USB_DW_DOEPINT_XFER_COMPL)) {
#if DBG_DIRECT_SHOW
			DBG_DIRECT("[%s] EP0EVENT_OEPINT_XFERCMPL_AND_SETUPDONE", __func__);
#endif
			Ep0ParseSetupPkt();

		} else if ((ep_int_status & USB_DW_DOEPINT_STSPHSERCVD) &&
			   !(ep_int_status & USB_DW_DOEPINT_XFER_COMPL)) {
			base->out_ep_reg[ep_idx].doepctl |= USB_DW_DEPCTL_CNAK;
		}

		if (ep_int_status & USB_DW_DOEPINT_AHBERR || ep_int_status & USB_DW_DOEPINT_BNA) {
#if DBG_DIRECT_SHOW
			DBG_DIRECT("[%s] USB_DW_DOEPINT_BNA", __func__);
#endif
		}
	}
#endif
#if CONFIG_USB_DC_RTL87X2G_DMA
	for (ep_idx = 1U; ep_idx < USB_DW_OUT_EP_NUM; ep_idx++)
#else
	for (ep_idx = 0U; ep_idx < USB_DW_OUT_EP_NUM; ep_idx++)
#endif
	{
		if (base->daint & USB_DW_DAINT_OUT_EP_INT(ep_idx)) {
			/* Read OUT EP interrupt status */
			ep_int_status = base->out_ep_reg[ep_idx].doepint & base->doepmsk;

			/* Clear OUT EP interrupts */
			base->out_ep_reg[ep_idx].doepint = ep_int_status;

			LOG_DBG("USB OUT EP%u interrupt status: 0x%x\n", ep_idx, ep_int_status);
		}
	}

	/* Clear interrupt. */
	base->gintsts = USB_DW_GINTSTS_OEP_INT;
}

static void usb_dw_isr_handler(const void *unused)
{
	struct usb_dw_reg *const base = usb_dw_cfg.base;
	uint32_t int_status;

#if DBG_DIRECT_SHOW
	DBG_DIRECT("[%s]", __func__);
#endif
	ARG_UNUSED(unused);
	/*  Read interrupt status */
	while ((int_status = (base->gintsts & base->gintmsk))) {
		LOG_WRN("current addr:%d",
			(base->dcfg & USB_DW_DCFG_DEV_ADDR_MASK) >> USB_DW_DCFG_DEV_ADDR_OFFSET);

		if (int_status & USB_DW_GINTSTS_USB_RST) {
#if DBG_DIRECT_SHOW
			DBG_DIRECT("-------------------USB_DW_GINTSTS_USB_RST-------------------");
#endif
			LOG_WRN("USB_DW_GINTSTS_USB_RST");
			/* Clear interrupt. */
			base->gintsts = USB_DW_GINTSTS_USB_RST;

			/* Reset detected */
			usb_dw_handle_reset();
		}

		if (int_status & USB_DW_GINTSTS_ENUM_DONE) {
#if DBG_DIRECT_SHOW
			DBG_DIRECT(
				"-------------------USB_DW_GINTSTS_ENUM_DONE-------------------");
#endif
			LOG_WRN("USB_DW_GINTSTS_ENUM_DONE");
			/* Clear interrupt. */
			base->gintsts = USB_DW_GINTSTS_ENUM_DONE;

			/* Enumeration done detected */
			usb_dw_handle_enum_done();
		}

		if (int_status & USB_DW_GINTSTS_USB_SUSP) {
#if DBG_DIRECT_SHOW
			DBG_DIRECT("-------------------USB_DW_GINTSTS_USB_SUSP-------------------");
#endif
			LOG_WRN("USB_DW_GINTSTS_USB_SUSP");
			/* Clear interrupt. */
			base->gintsts = USB_DW_GINTSTS_USB_SUSP;
			extern int hal_usb_suspend_enter(void);

			hal_usb_suspend_enter();

			usb_power_is_on = false;

			usb_dw_ctrl.current_status = USB_DC_SUSPEND;
			if (usb_dw_ctrl.status_cb) {
				usb_dw_ctrl.status_cb(USB_DC_SUSPEND, NULL);
			}
			return;
		}

		if (int_status & USB_DW_GINTSTS_WK_UP_INT) {
#if DBG_DIRECT_SHOW
			DBG_DIRECT(
				"-------------------USB_DW_GINTSTS_WK_UP_INT-------------------");
#endif
			LOG_WRN("USB_DW_GINTSTS_WK_UP_INT");
			/* Clear interrupt. */
			base->gintsts = USB_DW_GINTSTS_WK_UP_INT;

			usb_dw_ctrl.current_status = USB_DC_RESUME;
			if (usb_dw_ctrl.status_cb) {
				usb_dw_ctrl.status_cb(USB_DC_RESUME, NULL);
			}
		}

		if (int_status & USB_DW_GINTSTS_RX_FLVL) {
#if DBG_DIRECT_SHOW
			DBG_DIRECT("-------------------USB_DW_GINTSTS_RX_FLVL-------------------");
#endif
			LOG_WRN("USB_DW_GINTSTS_RX_FLVL");
			/* Packet in RX FIFO */
			usb_dw_int_rx_flvl_handler();
		}

		if (int_status & USB_DW_GINTSTS_IEP_INT) {
#if DBG_DIRECT_SHOW
			DBG_DIRECT("-------------------USB_DW_GINTSTS_IEP_INT-------------------");
#endif
			LOG_WRN("USB_DW_GINTSTS_IEP_INT");
			/* IN EP interrupt */
			usb_dw_int_iep_handler();
		}

		if (int_status & USB_DW_GINTSTS_OEP_INT) {
#if DBG_DIRECT_SHOW
			DBG_DIRECT("-------------------USB_DW_GINTSTS_OEP_INT-------------------");
#endif
			LOG_WRN("USB_DW_GINTSTS_OEP_INT");
			/* No OUT interrupt expected in FIFO mode,
			 * just clear interrupt
			 */
			usb_dw_int_oep_handler();
		}
	}
}

static void usb_dw_resume_isr_handler(const void *unused)
{
#if DBG_DIRECT_SHOW
	DBG_DIRECT("-------------------USB_DW_RESUME_INT-------------------");
#endif
	SoC_VENDOR->u_004.REG_LOW_PRI_INT_STATUS |= BIT31;
	/* prevent false alarm */
	usb_rtk_resume_sequence();

	usb_power_is_on = true;

	usb_dw_ctrl.current_status = USB_DC_RESUME;
	if (usb_dw_ctrl.status_cb) {
		usb_dw_ctrl.status_cb(USB_DC_RESUME, NULL);
	}
	return;
}

#if CONFIG_PM
static void usb_register_dlps_cb(void);
#endif

int usb_dc_attach(void)
{
#if DBG_DIRECT_SHOW
	DBG_DIRECT("[%s]", __func__);
#endif
	int ret;

	if (usb_dw_ctrl.attached) {
		return 0;
	}

	if (usb_dw_cfg.pwr_on_func != NULL) {
		ret = usb_dw_cfg.pwr_on_func(NULL);
		if (ret) {
			LOG_ERR("usb phy power on fials");
			return ret;
		}

		usb_power_is_on = true;
	}

	ret = usb_rtk_init();

	usb_dw_resize_fifo();

	if (ret) {
		LOG_ERR("usb dw init fials");
		return ret;
	}

#if CONFIG_PM
	usb_register_dlps_cb();
#endif

	usb_dw_cfg.irq_enable_func(NULL);

	usb_dw_ctrl.attached = 1U;

	return 0;
}

int usb_dc_detach(void)
{
	struct usb_dw_reg *const base = usb_dw_cfg.base;

	if (!usb_dw_ctrl.attached) {
		return 0;
	}

	irq_disable(DT_INST_IRQ_BY_IDX(0, 0, irq));
	irq_disable(DT_INST_IRQ_BY_IDX(0, 1, irq));

	if (usb_power_is_on) {
		/* Enable soft disconnect */
		base->dctl |= USB_DW_DCTL_SFT_DISCON;
	}

	usb_rtk_disable_power_seq();
	usb_power_is_on = false;

	usb_dw_ctrl.attached = 0U;

	usb_dw_ctrl.current_status = USB_DC_DISCONNECTED;
	if (usb_dw_ctrl.status_cb) {
		usb_dw_ctrl.status_cb(USB_DC_DISCONNECTED, NULL);
	}

	return 0;
}

int usb_dc_reset(void)
{
	int ret;

	ret = usb_dw_reset();

	/* Clear private data */
	(void)memset(&usb_dw_ctrl, 0, sizeof(usb_dw_ctrl));

	return ret;
}

int usb_dc_set_address(const uint8_t addr)
{
	struct usb_dw_reg *const base = usb_dw_cfg.base;

#if DBG_DIRECT_SHOW
	DBG_DIRECT("[%s] %d", __func__, addr);
#endif
	LOG_WRN("usb_dc_set_address %d", addr);

	if (addr > (USB_DW_DCFG_DEV_ADDR_MASK >> USB_DW_DCFG_DEV_ADDR_OFFSET)) {
		return -EINVAL;
	}

	base->dcfg &= ~USB_DW_DCFG_DEV_ADDR_MASK;
	base->dcfg |= addr << USB_DW_DCFG_DEV_ADDR_OFFSET;

	return 0;
}

int usb_dc_ep_check_cap(const struct usb_dc_ep_cfg_data *const cfg)
{
	uint8_t ep_idx = USB_EP_GET_IDX(cfg->ep_addr);

	LOG_DBG("ep %x, mps %d, type %d", cfg->ep_addr, cfg->ep_mps, cfg->ep_type);

	if ((cfg->ep_type == USB_DC_EP_CONTROL) && ep_idx) {
		LOG_ERR("invalid endpoint configuration");
		return -1;
	}

	if (cfg->ep_mps > DW_USB_MAX_PACKET_SIZE) {
		LOG_WRN("unsupported packet size");
		return -1;
	}

	if (USB_EP_DIR_IS_OUT(cfg->ep_addr) && ep_idx >= USB_DW_OUT_EP_NUM) {
		LOG_WRN("OUT endpoint address out of range");
		return -1;
	}

	if (USB_EP_DIR_IS_IN(cfg->ep_addr) && ep_idx >= USB_DW_IN_EP_NUM) {
		LOG_WRN("IN endpoint address out of range");
		return -1;
	}

	return 0;
}

int usb_dc_ep_configure(const struct usb_dc_ep_cfg_data *const ep_cfg)
{
	uint8_t ep;

	if (!ep_cfg) {
		return -EINVAL;
	}

	ep = ep_cfg->ep_addr;
	LOG_DBG(" ep = 0x%x", ep);
	if (!usb_dw_ctrl.attached || !usb_dw_ep_is_valid(ep)) {
		LOG_ERR("Not attached / Invalid endpoint: EP 0x%x", ep);
		return -EINVAL;
	}

	usb_dw_ep_set(ep, ep_cfg->ep_mps, ep_cfg->ep_type);

	return 0;
}

int usb_dc_ep_set_stall(const uint8_t ep)
{
	struct usb_dw_reg *const base = usb_dw_cfg.base;
	uint8_t ep_idx = USB_EP_GET_IDX(ep);

	if (!usb_dw_ctrl.attached || !usb_dw_ep_is_valid(ep)) {
		LOG_ERR("Not attached / Invalid endpoint: EP 0x%x", ep);
		return -EINVAL;
	}

	if (USB_EP_DIR_IS_OUT(ep)) {
		base->out_ep_reg[ep_idx].doepctl |= USB_DW_DEPCTL_STALL;
	} else {
		base->in_ep_reg[ep_idx].diepctl |= USB_DW_DEPCTL_STALL;
	}

	return 0;
}

int usb_dc_ep_clear_stall(const uint8_t ep)
{
	struct usb_dw_reg *const base = usb_dw_cfg.base;
	uint8_t ep_idx = USB_EP_GET_IDX(ep);

	if (!usb_dw_ctrl.attached || !usb_dw_ep_is_valid(ep)) {
		LOG_ERR("Not attached / Invalid endpoint: EP 0x%x", ep);
		return -EINVAL;
	}

	if (!ep_idx) {
		/* Not possible to clear stall for EP0 */
		return -EINVAL;
	}

	if (USB_EP_DIR_IS_OUT(ep)) {
		base->out_ep_reg[ep_idx].doepctl &= ~USB_DW_DEPCTL_STALL;
	} else {
		base->in_ep_reg[ep_idx].diepctl &= ~USB_DW_DEPCTL_STALL;
	}

	return 0;
}

int usb_dc_ep_halt(const uint8_t ep)
{
	struct usb_dw_reg *const base = usb_dw_cfg.base;
	uint8_t ep_idx = USB_EP_GET_IDX(ep);
	volatile uint32_t *p_depctl;

	if (!usb_dw_ctrl.attached || !usb_dw_ep_is_valid(ep)) {
		LOG_ERR("Not attached / Invalid endpoint: EP 0x%x", ep);
		return -EINVAL;
	}

	if (!ep_idx) {
		/* Cannot disable EP0, just set stall */
		usb_dc_ep_set_stall(ep);
	} else {
		if (USB_EP_DIR_IS_OUT(ep)) {
			p_depctl = &base->out_ep_reg[ep_idx].doepctl;
		} else {
			p_depctl = &base->in_ep_reg[ep_idx].diepctl;
		}

		/* Set STALL and disable endpoint if enabled */
		if (*p_depctl & USB_DW_DEPCTL_EP_ENA) {
			*p_depctl |= USB_DW_DEPCTL_EP_DIS | USB_DW_DEPCTL_STALL;
		} else {
			*p_depctl |= USB_DW_DEPCTL_STALL;
		}
	}

	return 0;
}

int usb_dc_ep_is_stalled(const uint8_t ep, uint8_t *const stalled)
{
	struct usb_dw_reg *const base = usb_dw_cfg.base;
	uint8_t ep_idx = USB_EP_GET_IDX(ep);

	if (!usb_dw_ctrl.attached || !usb_dw_ep_is_valid(ep)) {
		LOG_ERR("Not attached / Invalid endpoint: EP 0x%x", ep);
		return -EINVAL;
	}

	if (!stalled) {
		return -EINVAL;
	}

	*stalled = 0U;
	if (USB_EP_DIR_IS_OUT(ep)) {
		if (base->out_ep_reg[ep_idx].doepctl & USB_DW_DEPCTL_STALL) {
			*stalled = 1U;
		}
	} else {
		if (base->in_ep_reg[ep_idx].diepctl & USB_DW_DEPCTL_STALL) {
			*stalled = 1U;
		}
	}

	return 0;
}

int usb_dc_ep_enable(const uint8_t ep)
{
	struct usb_dw_reg *const base = usb_dw_cfg.base;
	uint8_t ep_idx = USB_EP_GET_IDX(ep);

	LOG_DBG("ep %x", ep);
	if (!usb_dw_ctrl.attached || !usb_dw_ep_is_valid(ep)) {
		LOG_ERR("Not attached / Invalid endpoint: EP 0x%x", ep);
		return -EINVAL;
	}

	/* enable EP interrupts */
	if (USB_EP_DIR_IS_OUT(ep)) {
		base->daintmsk |= USB_DW_DAINT_OUT_EP_INT(ep_idx);
	} else {
		base->daintmsk |= USB_DW_DAINT_IN_EP_INT(ep_idx);
	}

	/* Activate Ep */
	if (USB_EP_DIR_IS_OUT(ep)) {
		base->out_ep_reg[ep_idx].doepctl |= USB_DW_DEPCTL_USB_ACT_EP;
		usb_dw_ctrl.out_ep_ctrl[ep_idx].ep_ena = 1U;
	} else {
		base->in_ep_reg[ep_idx].diepctl |= USB_DW_DEPCTL_USB_ACT_EP;
		usb_dw_ctrl.in_ep_ctrl[ep_idx].ep_ena = 1U;
	}

#if CONFIG_USB_DC_RTL87X2G_DMA
	if (USB_EP_DIR_IS_OUT(ep)) {
		usb_dw_ctrl.aOutXfer[ep_idx] = k_heap_alloc(&ep_heap, sizeof(outxfer_t), K_NO_WAIT);
		if (usb_dw_ctrl.aOutXfer[ep_idx] == NULL) {
			LOG_ERR("Failed to allocate memory");
			return -ENOMEM;
		}

		usb_dw_ctrl.aOutDmaDesc[ep_idx] =
			k_heap_aligned_alloc(&ep_heap, 4, sizeof(OutDmaDesc_t), K_NO_WAIT);
		if (usb_dw_ctrl.aOutDmaDesc[ep_idx] == NULL) {
			LOG_ERR("Failed to allocate memory");
			k_heap_free(&ep_heap, usb_dw_ctrl.aOutXfer[ep_idx]);
			return -ENOMEM;
		}

		usb_dw_ctrl.ep_rx_buf[ep_idx] =
			k_heap_alloc(&ep_heap, usb_dw_ctrl.out_ep_ctrl[ep_idx].mps, K_NO_WAIT);
		if (usb_dw_ctrl.ep_rx_buf[ep_idx] == NULL) {
			LOG_ERR("Failed to allocate memory");
			k_heap_free(&ep_heap, usb_dw_ctrl.aOutXfer[ep_idx]);
			k_heap_free(&ep_heap, usb_dw_ctrl.aOutDmaDesc[ep_idx]);
			return -ENOMEM;
		}

	} else {
		usb_dw_ctrl.aInXfer[ep_idx] = k_heap_alloc(&ep_heap, sizeof(inxfer_t), K_NO_WAIT);
		if (usb_dw_ctrl.aInXfer[ep_idx] == NULL) {
			LOG_ERR("Failed to allocate memory");
			return -ENOMEM;
		}
		usb_dw_ctrl.aInDmaDesc[ep_idx] =
			k_heap_aligned_alloc(&ep_heap, 4, sizeof(InDmaDesc_t), K_NO_WAIT);
		if (usb_dw_ctrl.aInDmaDesc[ep_idx] == NULL) {
			LOG_ERR("Failed to allocate memory");
			k_heap_free(&ep_heap, usb_dw_ctrl.aInXfer[ep_idx]);
			return -ENOMEM;
		}
	}
#endif

	if (USB_EP_DIR_IS_OUT(ep) &&
	    usb_dw_ctrl.out_ep_ctrl[ep_idx].cb != usb_transfer_ep_callback) {
		/* Start reading now, except for transfer managed eps */
		usb_dw_prep_rx(ep, 0);
	}

	return 0;
}

int usb_dc_ep_disable(const uint8_t ep)
{
	if (!usb_power_is_on) {
		return 0;
	}
	LOG_DBG("line%d", __LINE__);
	struct usb_dw_reg *const base = usb_dw_cfg.base;
	uint8_t ep_idx = USB_EP_GET_IDX(ep);

#if DBG_DIRECT_SHOW
	DBG_DIRECT("[%s] ep%x", __func__, ep_idx);
#endif

	if (!usb_dw_ep_is_valid(ep)) {
		LOG_ERR("Not attached / Invalid endpoint: EP 0x%x", ep);
		return -EINVAL;
	}

	/* Disable EP interrupts */
	if (USB_EP_DIR_IS_OUT(ep)) {
		base->daintmsk &= ~USB_DW_DAINT_OUT_EP_INT(ep_idx);
		base->doepmsk &= ~USB_DW_DOEPINT_SET_UP;
	} else {
		base->daintmsk &= ~USB_DW_DAINT_IN_EP_INT(ep_idx);
		base->diepmsk &= ~USB_DW_DIEPINT_XFER_COMPL;
		base->gintmsk &= ~USB_DW_GINTSTS_RX_FLVL;
	}

	/* De-activate, disable and set NAK for Ep */
	if (USB_EP_DIR_IS_OUT(ep)) {
		base->out_ep_reg[ep_idx].doepctl &=
			~(USB_DW_DEPCTL_USB_ACT_EP | USB_DW_DEPCTL_EP_ENA | USB_DW_DEPCTL_SNAK);
		usb_dw_ctrl.out_ep_ctrl[ep_idx].ep_ena = 0U;
	} else {
		base->in_ep_reg[ep_idx].diepctl &=
			~(USB_DW_DEPCTL_USB_ACT_EP | USB_DW_DEPCTL_EP_ENA | USB_DW_DEPCTL_SNAK);
		usb_dw_ctrl.in_ep_ctrl[ep_idx].ep_ena = 0U;
	}

#if CONFIG_USB_DC_RTL87X2G_DMA
	if (USB_EP_DIR_IS_OUT(ep)) {
		k_heap_free(&ep_heap, usb_dw_ctrl.aOutXfer[ep_idx]);
		k_heap_free(&ep_heap, usb_dw_ctrl.aOutDmaDesc[ep_idx]);
		k_heap_free(&ep_heap, usb_dw_ctrl.ep_rx_buf[ep_idx]);
	} else {
		k_heap_free(&ep_heap, usb_dw_ctrl.aInXfer[ep_idx]);
		k_heap_free(&ep_heap, usb_dw_ctrl.aInDmaDesc[ep_idx]);
	}
#endif

	return 0;
}

int usb_dc_ep_flush(const uint8_t ep)
{
	struct usb_dw_reg *const base = usb_dw_cfg.base;
	uint8_t ep_idx = USB_EP_GET_IDX(ep);
	uint32_t cnt;

	if (!usb_dw_ctrl.attached || !usb_dw_ep_is_valid(ep)) {
		LOG_ERR("Not attached / Invalid endpoint: EP 0x%x", ep);
		return -EINVAL;
	}

	if (USB_EP_DIR_IS_OUT(ep)) {
		/* RX FIFO is global and cannot be flushed per EP */
		return -EINVAL;
	}

	/* Each endpoint has dedicated Tx FIFO */
	base->grstctl |= ep_idx << USB_DW_GRSTCTL_TX_FNUM_OFFSET;
	base->grstctl |= USB_DW_GRSTCTL_TX_FFLSH;

	cnt = 0U;

	do {
		if (++cnt > USB_DW_CORE_RST_TIMEOUT_US) {
			LOG_ERR("USB TX FIFO flush HANG!");
			return -EIO;
		}
		usb_dw_udelay(1);
	} while (base->grstctl & USB_DW_GRSTCTL_TX_FFLSH);

	return 0;
}

int usb_dc_ep_write(const uint8_t ep, const uint8_t *const data, const uint32_t data_len,
		    uint32_t *const ret_bytes)
{
#if DBG_DIRECT_SHOW
	DBG_DIRECT("[%s] ep%x data_len=%d", __func__, ep, data_len);
#endif
	int ret;

	if (!usb_dw_ctrl.attached || !usb_dw_ep_is_valid(ep)) {
		LOG_ERR("Not attached / Invalid endpoint: EP 0x%x", ep);
		return -EINVAL;
	}

	/* Check if IN ep */
	if (USB_EP_GET_DIR(ep) != USB_EP_DIR_IN) {
		return -EINVAL;
	}

	/* Check if ep enabled */
	if (!usb_dw_ep_is_enabled(ep)) {
		return -EINVAL;
	}

#if CONFIG_USB_DC_RTL87X2G_DMA
	ret = usb_dw_tx_dma(ep, data, data_len);
#else
	ret = usb_dw_tx(ep, data, data_len);
#endif

	if (ret < 0) {
		return ret;
	}

	if (ret_bytes) {
		*ret_bytes = ret;
	}

	return 0;
}

int usb_dc_ep_read_wait(uint8_t ep, uint8_t *data, uint32_t max_data_len, uint32_t *read_bytes)
{
#if CONFIG_USB_DC_RTL87X2G_DMA
	uint8_t ep_idx = USB_EP_GET_IDX(ep);

	if (data) {
		memcpy(data, usb_dw_ctrl.ep_rx_buf[ep_idx], max_data_len);
		return 0;
	} else {
		return -EINVAL;
	}
#else

	struct usb_dw_reg *const base = usb_dw_cfg.base;
	uint8_t ep_idx = USB_EP_GET_IDX(ep);
	uint32_t i, j, data_len, bytes_to_copy;

	if (!usb_dw_ctrl.attached || !usb_dw_ep_is_valid(ep)) {
		LOG_ERR("Not attached / Invalid endpoint: EP 0x%x", ep);
		return -EINVAL;
	}

	/* Check if OUT ep */
	if (USB_EP_GET_DIR(ep) != USB_EP_DIR_OUT) {
		LOG_ERR("Wrong endpoint direction");
		return -EINVAL;
	}

	/* Allow to read 0 bytes */
	if (!data && max_data_len) {
		LOG_ERR("Wrong arguments");
		return -EINVAL;
	}

	/* Check if ep enabled */
	if (!usb_dw_ep_is_enabled(ep)) {
		LOG_ERR("Not enabled endpoint");
		return -EINVAL;
	}

	data_len = usb_dw_ctrl.out_ep_ctrl[ep_idx].data_len;

	if (!data && !max_data_len) {
		/* When both buffer and max data to read are zero return
		 * the available data in buffer
		 */
		if (read_bytes) {
			*read_bytes = data_len;
		}
		return 0;
	}

	if (data_len > max_data_len) {
		LOG_ERR("Not enough room to copy all the rcvd data!");
		bytes_to_copy = max_data_len;
	} else {
		bytes_to_copy = data_len;
	}

	LOG_DBG("Read EP%d, req %d, read %d bytes", ep, max_data_len, bytes_to_copy);

	/* Data in the FIFOs is always stored per 32-bit words */
	for (i = 0U; i < (bytes_to_copy & ~0x3); i += 4U) {
		*(uint32_t *)(data + i) = USB_DW_EP_FIFO(base, ep_idx);
	}
	if (bytes_to_copy & 0x3) {
		/* Not multiple of 4 */
		uint32_t last_dw = USB_DW_EP_FIFO(base, ep_idx);

		for (j = 0U; j < (bytes_to_copy & 0x3); j++) {
			*(data + i + j) = (sys_cpu_to_le32(last_dw) >> (j * 8U)) & 0xFF;
		}
	}

	usb_dw_ctrl.out_ep_ctrl[ep_idx].data_len -= bytes_to_copy;

	if (read_bytes) {
		*read_bytes = bytes_to_copy;
	}

	return 0;
#endif
}

int usb_dc_ep_read_continue(uint8_t ep)
{
	uint8_t ep_idx = USB_EP_GET_IDX(ep);

	if (!usb_dw_ctrl.attached || !usb_dw_ep_is_valid(ep)) {
		LOG_ERR("Not attached / Invalid endpoint: EP 0x%x", ep);
		return -EINVAL;
	}

	/* Check if OUT ep */
	if (USB_EP_GET_DIR(ep) != USB_EP_DIR_OUT) {
		LOG_ERR("Wrong endpoint direction");
		return -EINVAL;
	}

	if (!usb_dw_ctrl.out_ep_ctrl[ep_idx].data_len) {
		usb_dw_prep_rx(ep_idx, 0);
	}

	return 0;
}

int usb_dc_ep_read(const uint8_t ep, uint8_t *const data, const uint32_t max_data_len,
		   uint32_t *const read_bytes)
{
	if (usb_dc_ep_read_wait(ep, data, max_data_len, read_bytes) != 0) {
		return -EINVAL;
	}

	if (!data && !max_data_len) {
		/* When both buffer and max data to read are zero the above
		 * call would fetch the data len and we simply return.
		 */
		return 0;
	}

	if (usb_dc_ep_read_continue(ep) != 0) {
		return -EINVAL;
	}

	return 0;
}

int usb_dc_ep_set_callback(const uint8_t ep, const usb_dc_ep_callback cb)
{
	uint8_t ep_idx = USB_EP_GET_IDX(ep);

	if (!usb_dw_ctrl.attached || !usb_dw_ep_is_valid(ep)) {
		LOG_ERR("Not attached / Invalid endpoint: EP 0x%x", ep);
		return -EINVAL;
	}

	if (USB_EP_DIR_IS_IN(ep)) {
		usb_dw_ctrl.in_ep_ctrl[ep_idx].cb = cb;
	} else {
		usb_dw_ctrl.out_ep_ctrl[ep_idx].cb = cb;
	}

	return 0;
}

void usb_dc_set_status_callback(const usb_dc_status_callback cb)
{
	usb_dw_ctrl.status_cb = cb;
}

int usb_dc_ep_mps(const uint8_t ep)
{
	enum usb_dw_out_ep_idx ep_idx = USB_EP_GET_IDX(ep);

	if (!usb_dw_ctrl.attached || !usb_dw_ep_is_valid(ep)) {
		LOG_ERR("Not attached / Invalid endpoint: EP 0x%x", ep);
		return -EINVAL;
	}

	if (USB_EP_DIR_IS_OUT(ep)) {
		return usb_dw_ctrl.out_ep_ctrl[ep_idx].mps;
	} else {
		return usb_dw_ctrl.in_ep_ctrl[ep_idx].mps;
	}
}

int usb_dc_wakeup_request(void)
{
	usb_rtk_resume_sequence();
	SoC_VENDOR->u_004.REG_LOW_PRI_INT_STATUS |= BIT31;
	struct usb_dw_reg *const base = usb_dw_cfg.base;

	if (!(base->dsts & USB_DW_DSTS_SUSPSTS_MASK)) {
		LOG_ERR("Remote wakeup while is not in suspend state, or while is not allowed by "
			"host.");
		return -EAGAIN;
	}

	base->dctl |= USB_DW_DCTL_RMTWKUPSIG_MASK;

	k_sleep(K_MSEC(2));

	base->dctl &= ~USB_DW_DCTL_RMTWKUPSIG_MASK;

	LOG_DBG("Remote wakeup from suspend.");

	return 0;
}

static void usb_isr_suspend_enable(void)
{
#ifdef CONFIG_PM
	AON_REG7X_SYS_TYPE reg7x;

	reg7x.d32 = HAL_READ32(SYSTEM_REG_BASE, AON_REG7X_SYS);
	reg7x.usb_wakeup_sel = 1;
	reg7x.USB_WKPOL = 0;
	reg7x.USB_WKEN = 1;
	HAL_WRITE32(SYSTEM_REG_BASE, AON_REG7X_SYS, reg7x.d32);
#endif

	/* edge trigger */
	SoC_VENDOR->u_008.REG_LOW_PRI_INT_MODE |= BIT31;

	/* resing edge trigger */
	SoC_VENDOR->u_018.INTERRUPT_EDGE_OPTION &= ~(BIT31);

	/* Note: must disable at disable flow */
	SoC_VENDOR->u_00C.REG_LOW_PRI_INT_EN |= BIT31;
}

#ifdef CONFIG_PM
extern int hal_usb_wakeup_status_clear(void);
extern int hal_usb_wakeup_status_get(void);
extern void usb_set_pon_domain(void);
extern void usb_dm_start_from_dlps(void);

void usb_start_from_dlps(void)
{
	if (usb_rtk_resume_sequence() != 0) {
		usb_rtk_disable_power_seq();
		return;
	}

	usb_dw_cfg.irq_enable_func(NULL);

	return;
}

static PMCheckResult usb_pm_check(void)
{
	volatile enum usb_dc_status_code usb_state = usb_dw_ctrl.current_status;

	if (usb_state == USB_DC_SUSPEND || usb_state == USB_DC_DISCONNECTED) {
		return PM_CHECK_PASS;
	} else {
		return PM_CHECK_FAIL;
	}
}

static void usb_pm_store(void)
{
#if DBG_DIRECT_SHOW
	DBG_DIRECT("======================usb_pm_store======================");
#endif
	hal_usb_wakeup_status_clear();
	AON_REG8X_SYS_TYPE reg8x;

	reg8x.d32 = AON_REG_READ(AON_REG8X_SYS);
}

static void usb_pm_restore(void)
{
#if DBG_DIRECT_SHOW
	DBG_DIRECT("======================usb_pm_restore======================");
#endif
	if (hal_usb_wakeup_status_get() == 0) {
	} else {
		usb_start_from_dlps();
	}
}

static void usb_register_dlps_cb(void)
{
	usb_set_pon_domain();

	platform_pm_register_callback_func_with_priority((void *)usb_pm_check, PLATFORM_PM_CHECK,
							 1);
	platform_pm_register_callback_func_with_priority((void *)usb_pm_store, PLATFORM_PM_STORE,
							 1);
	platform_pm_register_callback_func_with_priority((void *)usb_pm_restore,
							 PLATFORM_PM_RESTORE, 1);
}
#endif
