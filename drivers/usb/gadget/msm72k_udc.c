/*
 * Driver for HighSpeed USB Client Controller in MSM7K
 *
 * Copyright (C) 2008 Google, Inc.
 * Author: Mike Lockwood <lockwood@android.com>
 *         Brian Swetland <swetland@google.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/list.h>

#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include <linux/platform_device.h>
#include <linux/debugfs.h>
#include <linux/workqueue.h>
#include <linux/clk.h>

#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/io.h>

#include <asm/mach-types.h>

#include <asm/arch/board.h>
#include <asm/arch/msm_hsusb.h>
#include <linux/device.h>
#include <asm/arch/msm_hsusb_hw.h>
#include <asm/arch/vreg.h>

#include <linux/cpufreq.h>

#ifdef CONFIG_USB_GADGET_EVENT
#include <linux/usb/gadget_event.h>
#endif

#undef STOP_DRAW_CURRENT_ON_BUS_SUSPEND

static const char driver_name[] = "msm72k_udc";

static int emsg_count = 0;

#define EMSG_COUNT_MAX		10

/* #define DEBUG */
/* #define VERBOSE */

#define MSM_USB_BASE ((unsigned) ui->addr)

#define	DRIVER_DESC		"MSM 72K USB Peripheral Controller"

#define EPT_FLAG_IN        0x0001

#define SETUP_BUF_SIZE      4096

#define USB_LINK_RESET_TIMEOUT	(msecs_to_jiffies(10))

static const char *const ep_name[] = {
	"ep0out", "ep1out", "ep2out", "ep3out",
	"ep4out", "ep5out", "ep6out", "ep7out",
	"ep8out", "ep9out", "ep10out", "ep11out",
	"ep12out", "ep13out", "ep14out", "ep15out",
	"ep0in", "ep1in", "ep2in", "ep3in",
	"ep4in", "ep5in", "ep6in", "ep7in",
	"ep8in", "ep9in", "ep10in", "ep11in",
	"ep12in", "ep13in", "ep14in", "ep15in"
};

/* current state of VBUS */
static int vbus;

struct msm_request {
	struct usb_request req;

	/* saved copy of req.complete */
	void	(*gadget_complete)(struct usb_ep *ep,
					struct usb_request *req);


	struct usb_info *ui;
	struct msm_request *next;

	unsigned busy:1;
	unsigned live:1;
	unsigned alloced:1;
	unsigned dead:1;

	dma_addr_t dma;
	dma_addr_t item_dma;

	struct ept_queue_item *item;
};

#define to_msm_request(r) container_of(r, struct msm_request, req)
#define to_msm_endpoint(r) container_of(r, struct msm_endpoint, ep)

struct msm_endpoint {
	struct usb_ep ep;
	struct usb_info *ui;
	struct msm_request *req; /* head of pending requests */
	struct msm_request *last;
	unsigned flags;

	/* bit number (0-31) in various status registers
	** as well as the index into the usb_info's array
	** of all endpoints
	*/
	unsigned char bit;
	unsigned char num;

	/* pointers to DMA transfer list area */
	/* these are allocated from the usb_info dma space */
	struct ept_queue_head *head;
};

static void usb_do_work(struct work_struct *w);
static int usb_is_online(struct usb_info *ui);
static int usb_suspend_phy(struct usb_info *ui);
static int usb_wakeup_phy(struct usb_info *ui); 

#define USB_STATE_IDLE    0
#define USB_STATE_ONLINE  1
#define USB_STATE_OFFLINE 2

#define USB_FLAG_START          0x0001
#define USB_FLAG_VBUS_ONLINE    0x0002
#define USB_FLAG_VBUS_OFFLINE   0x0004
#define USB_FLAG_RESET          0x0008

struct usb_info {
	/* lock for register/queue/device state changes */
	spinlock_t lock;

	/* single request used for handling setup transactions */
	struct usb_request *setup_req;

	struct platform_device *pdev;
	int irq;
	void *addr;

	unsigned state;
	unsigned flags;

	unsigned	online:1;
	unsigned	running:1;

	struct dma_pool *pool;

	/* dma page to back the queue heads and items */
	unsigned char *buf;
	dma_addr_t dma;

	struct ept_queue_head *head;

	/* used for allocation */
	unsigned next_item;
	unsigned next_ifc_num;

	/* endpoints are ordered based on their status bits,
	** so they are OUT0, OUT1, ... OUT15, IN0, IN1, ... IN15
	*/
	struct msm_endpoint ept[32];

	struct msm_hsusb_platform_data *pdata;

	struct workqueue_struct *wq;

	struct work_struct work;
	struct work_struct wakeup_phy;
	unsigned phy_status;
	unsigned phy_fail_count;
	struct work_struct vbus_draw_work;
	struct delayed_work bus_suspend_work;
	struct delayed_work bus_resume_work;

	struct usb_gadget		gadget;
	struct usb_gadget_driver	*driver;

#define ep0out ept[0]
#define ep0in  ept[16]

	struct clk *clk;
	struct clk *pclk;
	unsigned int clk_enabled;

	struct vreg *vreg;
	unsigned int vreg_enabled;

	unsigned in_lpm;

	unsigned int ep0_dir;
	u16 test_mode;

	u8 remote_wakeup;

	u8 host_present;

	u16 mA;
	u16 mA_to_resume;

	u8 suspended;
	u8 resume_work_pending;

	u8 reset_detected;

	int cpufreq_flag;
};
static struct usb_info *the_usb_info;

static const struct usb_ep_ops msm72k_ep_ops;

static int msm72k_pullup(struct usb_gadget *_gadget, int is_active);
static int msm72k_vbus_draw(struct usb_gadget *_gadget, unsigned mA);
static int msm72k_set_halt(struct usb_ep *_ep, int value);
static void flush_endpoint(struct msm_endpoint *ept);
static unsigned ulpi_read(struct usb_info *ui, unsigned reg);
static int ulpi_write(struct usb_info *ui, unsigned val, unsigned reg);
static int usb_hw_reset(struct usb_info *ui);
static void usb_reset(struct usb_info *ui);

static int usb_ep_get_stall(struct msm_endpoint *ept)
{
	unsigned int n;
	struct usb_info *ui = ept->ui;

	n = readl(USB_ENDPTCTRL(ept->num));
	if (ept->flags & EPT_FLAG_IN)
		return (CTRL_TXS & n) ? 1 : 0;
	else
		return (CTRL_RXS & n) ? 1 : 0;
}

static void usb_clk_enable(struct usb_info *ui)
{
	if (!ui->clk_enabled) {
		clk_enable(ui->clk);
		clk_enable(ui->pclk);
		ui->clk_enabled = 1;
	}
}

static void usb_clk_disable(struct usb_info *ui)
{
	if (ui->clk_enabled) {
		clk_disable(ui->pclk);
		clk_disable(ui->clk);
		ui->clk_enabled = 0;
	}
}

static void usb_vreg_enable(struct usb_info *ui)
{
	if (!IS_ERR(ui->vreg) && !ui->vreg_enabled) {
		//vreg_enable(ui->vreg);
		ui->vreg_enabled = 1;
	}
}

static void usb_vreg_disable(struct usb_info *ui)
{
	if (!IS_ERR(ui->vreg) && ui->vreg_enabled) {
		//vreg_disable(ui->vreg);
		ui->vreg_enabled = 0;
	}
}

static void usb_lpm_exit(struct usb_info *ui)
{
	if (ui->in_lpm == 0)
		return;

	usb_clk_enable(ui);
	usb_vreg_enable(ui);

	writel(readl(USB_USBCMD) & ~ASYNC_INTR_CTRL, USB_USBCMD);
	writel(readl(USB_USBCMD) & ~ULPI_STP_CTRL, USB_USBCMD);
	/* enable OTGSC interrupts */
	writel(readl(USB_OTGSC) | OTGSC_BSVIE, USB_OTGSC);

	if (readl(USB_PORTSC) & PORTSC_PHCD) {
		disable_irq(ui->irq);
		queue_work(ui->wq, &ui->wakeup_phy);
	}
	else {
		ui->in_lpm = 0;
	}
	pr_info("%s(): USB exited from low power mode\n", __func__);
}

static int usb_lpm_enter(struct usb_info *ui)
{
	unsigned long flags;
	unsigned connected;
	static int n_suspend_failed = 0;

	usb_reset(ui);

	spin_lock_irqsave(&ui->lock, flags);

	if (ui->in_lpm) {
		spin_unlock_irqrestore(&ui->lock, flags);
		pr_debug("already in lpm, nothing to do\n");
		n_suspend_failed = 0;
		return 0;
	}

	if (usb_is_online(ui)) {
		spin_unlock_irqrestore(&ui->lock, flags);
		n_suspend_failed = 0;
		pr_info("%s: lpm procedure aborted\n", __func__);
		return -1;
	}

	ui->in_lpm = 1;
	disable_irq(ui->irq);
	spin_unlock_irqrestore(&ui->lock, flags);

	if (usb_suspend_phy(ui)) {
		ui->in_lpm = 0;
		enable_irq(ui->irq);
		if (++n_suspend_failed > 10)
			panic("can't suspend phy");
		pr_err("%s: phy suspend failed, lpm procedure aborted\n",
				__func__);
		return -1;
	}

	/* enable async interrupt */
	writel(readl(USB_USBCMD) | ASYNC_INTR_CTRL, USB_USBCMD);
	connected = readl(USB_USBCMD) & USBCMD_RS;

	usb_vreg_disable(ui);
	usb_clk_disable(ui);

#if 0
	if (!connected) {
		pr_err("%s: unable to enter lpm mode\n", __func__);
		usb_lpm_exit(ui);
		enable_irq(ui->irq);
		n_suspend_failed = 0;
		return -1;
	}
#endif
	enable_irq(ui->irq);
	n_suspend_failed = 0;
	pr_info("%s: usb in low power mode\n", __func__);
	return 0;
}

static void usb_lpm_wakeup_phy(struct work_struct *w)
{
	struct usb_info *ui = the_usb_info;
	unsigned long flags;

	if (usb_wakeup_phy(ui)) {
		pr_err("fatal error: cannot bring phy out of lpm\n");
		pr_err("%s: resetting controller\n", __func__);

		spin_lock_irqsave(&ui->lock, flags);
		/* clear usb intr register */
		writel(0, USB_USBINTR);
		/* stopping controller by disabling pullup on D+ */
		writel(readl(USB_USBCMD) & ~USBCMD_RS, USB_USBCMD);
		ui->flags = USB_FLAG_RESET;
		queue_work(ui->wq, &ui->work);
		enable_irq(ui->irq);
		spin_unlock_irqrestore(&ui->lock, flags);
		return;
	}

	ui->in_lpm = 0;
	enable_irq(ui->irq);
}

static unsigned ulpi_read(struct usb_info *ui, unsigned reg)
{
	unsigned timeout = 100000;

	/* initiate read operation */
	writel(ULPI_RUN | ULPI_READ | ULPI_ADDR(reg),
	       USB_ULPI_VIEWPORT);

	/* wait for completion */
	while ((readl(USB_ULPI_VIEWPORT) & ULPI_RUN) && (--timeout))
		;

	if (timeout == 0) {
		ERROR("ulpi_read: timeout %08x\n", readl(USB_ULPI_VIEWPORT));
		return 0xffffffff;
	}
	return ULPI_DATA_READ(readl(USB_ULPI_VIEWPORT));
}

static int ulpi_write(struct usb_info *ui, unsigned val, unsigned reg)
{
	unsigned timeout = 10000;

	/* initiate write operation */
	writel(ULPI_RUN | ULPI_WRITE |
	       ULPI_ADDR(reg) | ULPI_DATA(val),
	       USB_ULPI_VIEWPORT);

	/* wait for completion */
	while ((readl(USB_ULPI_VIEWPORT) & ULPI_RUN) && (--timeout))
		;

	if (timeout == 0) {
		ERROR("ulpi_write: timeout\n");
		return -1;
	}

	return 0;
}

static void init_endpoints(struct usb_info *ui)
{
	unsigned n;

	for (n = 0; n < 32; n++) {
		struct msm_endpoint *ept = ui->ept + n;

		ept->ui = ui;
		ept->bit = n;
		ept->num = n & 15;
		ept->ep.name = ep_name[n];
		ept->ep.ops = &msm72k_ep_ops;

		if (ept->bit > 15) {
			/* IN endpoint */
			ept->head = ui->head + (ept->num << 1) + 1;
			ept->flags = EPT_FLAG_IN;
		} else {
			/* OUT endpoint */
			ept->head = ui->head + (ept->num << 1);
			ept->flags = 0;
		}

	}
}

static void config_ept(struct msm_endpoint *ept)
{
	unsigned cfg = CONFIG_MAX_PKT(ept->ep.maxpacket) | CONFIG_ZLT;

	if (ept->bit == 0)
		/* ep0 out needs interrupt-on-setup */
		cfg |= CONFIG_IOS;

	ept->head->config = cfg;
	ept->head->next = TERMINATE;

#if 0
	if (ept->ep.maxpacket)
		INFO("ept #%d %s max:%d head:%p bit:%d\n",
		    ept->num, (ept->flags & EPT_FLAG_IN) ? "in" : "out",
		    ept->ep.maxpacket, ept->head, ept->bit);
#endif
}

static void configure_endpoints(struct usb_info *ui)
{
	unsigned n;

	for (n = 0; n < 32; n++)
		config_ept(ui->ept + n);
}

struct usb_request *usb_ept_alloc_req(struct msm_endpoint *ept,
			unsigned bufsize, gfp_t gfp_flags)
{
	struct usb_info *ui = ept->ui;
	struct msm_request *req;

	req = kzalloc(sizeof(*req), gfp_flags);
	if (!req)
		goto fail1;

	req->item = dma_pool_alloc(ui->pool, gfp_flags, &req->item_dma);
	if (!req->item)
		goto fail2;

	if (bufsize) {
		req->req.buf = kmalloc(bufsize, gfp_flags);
		if (!req->req.buf)
			goto fail3;
		req->alloced = 1;
	}

	return &req->req;

fail3:
	dma_pool_free(ui->pool, req->item, req->item_dma);
fail2:
	kfree(req);
fail1:
	return 0;
}

static void do_free_req(struct usb_info *ui, struct msm_request *req)
{
	if (req->alloced)
		kfree(req->req.buf);

	dma_pool_free(ui->pool, req->item, req->item_dma);
	kfree(req);
}


static void usb_ept_enable(struct msm_endpoint *ept, int yes,
		unsigned char ep_type)
{
	struct usb_info *ui = ept->ui;
	int in = ept->flags & EPT_FLAG_IN;
	unsigned n;

	n = readl(USB_ENDPTCTRL(ept->num));

	if (in) {
		if (yes) {
			n = (n & (~CTRL_TXT_MASK)) |
				(ep_type << CTRL_TXT_EP_TYPE_SHIFT);
			n |= CTRL_TXE | CTRL_TXR;
		} else
			n &= (~CTRL_TXE);
	} else {
		if (yes) {
			n = (n & (~CTRL_RXT_MASK)) |
				(ep_type << CTRL_RXT_EP_TYPE_SHIFT);
			n |= CTRL_RXE | CTRL_RXR;
		} else
			n &= ~(CTRL_RXE);
	}
	writel(n, USB_ENDPTCTRL(ept->num));

#if 1
	INFO("ept %d %s %s\n",
	       ept->num, in ? "in" : "out", yes ? "enabled" : "disabled");
#endif
}

static void usb_ept_start(struct msm_endpoint *ept)
{
	struct usb_info *ui = ept->ui;
	struct msm_request *req = ept->req;

	BUG_ON(req->live);

	/* link the hw queue head to the request's transaction item */
	ept->head->next = req->item_dma;
	ept->head->info = 0;

	/* start the endpoint */
	writel(1 << ept->bit, USB_ENDPTPRIME);

	/* mark this chain of requests as live */
	while (req) {
		req->live = 1;
		req = req->next;
	}
}

int usb_ept_queue_xfer(struct msm_endpoint *ept, struct usb_request *_req)
{
	unsigned long flags;
	struct msm_request *req = to_msm_request(_req);
	struct msm_request *last;
	struct usb_info *ui = ept->ui;
	struct ept_queue_item *item = req->item;
	unsigned length = req->req.length;

	if (length > (0x4000 - ((ptrdiff_t)req->req.buf & (PAGE_SIZE-1)))){
		if (emsg_count++ < EMSG_COUNT_MAX)
			INFO("usb_ept_queue_xfer() FIXME - failing request of size %d\n", length);
		return -EMSGSIZE;
	}

	spin_lock_irqsave(&ui->lock, flags);

	if (req->busy) {
		req->req.status = -EBUSY;
		spin_unlock_irqrestore(&ui->lock, flags);
		if (emsg_count++ < EMSG_COUNT_MAX)
			INFO("usb_ept_queue_xfer() tried to queue busy request\n");
		return -EBUSY;
	}

	if (!ui->online && (ept->num != 0)) {
		req->req.status = -ESHUTDOWN;
		spin_unlock_irqrestore(&ui->lock, flags);
		if (emsg_count++ < EMSG_COUNT_MAX)
			INFO("usb_ept_queue_xfer() called while offline\n");
		return -ESHUTDOWN;
	}

	req->busy = 1;
	req->live = 0;
	req->next = 0;
	req->req.status = -EBUSY;

	req->dma = dma_map_single(NULL, req->req.buf, length,
				  (ept->flags & EPT_FLAG_IN) ?
				  DMA_TO_DEVICE : DMA_FROM_DEVICE);

	/* prepare the transaction descriptor item for the hardware */
	item->next = TERMINATE;
	item->info = INFO_BYTES(length) | INFO_IOC | INFO_ACTIVE;
	item->page0 = req->dma;
	item->page1 = (req->dma + 0x1000) & 0xfffff000;
	item->page2 = (req->dma + 0x2000) & 0xfffff000;
	item->page3 = (req->dma + 0x3000) & 0xfffff000;

	/* Add the new request to the end of the queue */
	last = ept->last;
	if (last) {
		/* Already requests in the queue. add us to the
		 * end, but let the completion interrupt actually
		 * start things going, to avoid hw issues
		 */
		last->next = req;

		/* only modify the hw transaction next pointer if
		 * that request is not live
		 */
		if (!last->live)
			last->item->next = req->item_dma;
	} else {
		/* queue was empty -- kick the hardware */
		ept->req = req;
		usb_ept_start(ept);
	}
	ept->last = req;

	spin_unlock_irqrestore(&ui->lock, flags);
	return 0;
}

/* --- endpoint 0 handling --- */

static void ep0_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct msm_request *r = to_msm_request(req);
	struct msm_endpoint *ept = to_msm_endpoint(ep);
	struct usb_info *ui = ept->ui;

	req->complete = r->gadget_complete;
	r->gadget_complete = 0;
	if	(req->complete)
		req->complete(&ui->ep0in.ep, req);
}

static void ep0_queue_ack_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct msm_endpoint *ept = to_msm_endpoint(ep);

	/* queue up the receive of the ACK response from the host */
	if (req->status == 0) {
		struct usb_info *ui = ept->ui;
		if (req->zero && (ui->ep0_dir == USB_DIR_IN)) {
			/* send a zero-length packet */
			req->zero = 0;
			req->length = 0;
			req->complete = ep0_queue_ack_complete;
			usb_ept_queue_xfer(&ui->ep0in, req);
			return;
		}
		req->length = 0;
		req->complete = ep0_complete;
		if (ui->ep0_dir == USB_DIR_IN)
			usb_ept_queue_xfer(&ui->ep0out, req);
		else
			usb_ept_queue_xfer(&ui->ep0in, req);
	} else
		ep0_complete(ep, req);
}

static void ep0_setup_ack_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct msm_endpoint *ept = to_msm_endpoint(ep);
	struct usb_info *ui = ept->ui;
	unsigned int temp;

	if (!ui->test_mode)
		return;

	switch (ui->test_mode) {
	case J_TEST:
		pr_info("usb electrical test mode: (J)\n");
		temp = readl(USB_PORTSC) & (~PORTSC_PTC);
		writel(temp | PORTSC_PTC_J_STATE, USB_PORTSC);
		break;

	case K_TEST:
		pr_info("usb electrical test mode: (K)\n");
		temp = readl(USB_PORTSC) & (~PORTSC_PTC);
		writel(temp | PORTSC_PTC_K_STATE, USB_PORTSC);
		break;

	case SE0_NAK_TEST:
		pr_info("usb electrical test mode: (SE0-NAK)\n");
		temp = readl(USB_PORTSC) & (~PORTSC_PTC);
		writel(temp | PORTSC_PTC_SE0_NAK, USB_PORTSC);
		break;

	case TST_PKT_TEST:
		pr_info("usb electrical test mode: (TEST_PKT)\n");
		temp = readl(USB_PORTSC) & (~PORTSC_PTC);
		writel(temp | PORTSC_PTC_TST_PKT, USB_PORTSC);
		break;
	}
}

static void ep0_setup_ack(struct usb_info *ui)
{
	struct usb_request *req = ui->setup_req;
	req->length = 0;
	req->complete = ep0_setup_ack_complete;
	usb_ept_queue_xfer(&ui->ep0in, req);
}

static void ep0_setup_stall(struct usb_info *ui)
{
	writel((1<<16) | (1<<0), USB_ENDPTCTRL(0));
}

static void ep0_setup_send(struct usb_info *ui, unsigned length)
{
	struct usb_request *req = ui->setup_req;
	struct msm_request *r = to_msm_request(req);
	struct msm_endpoint *ept = &ui->ep0in;

	if ((length < req->length) && ((req->length & 63) == 0)) {
		/* if the sending data is shorter than requested and
		   ends on a packet boundary, it needs to be followed
		   by a zero length packet.
		*/
		req->zero = 1;
	}
	req->length = length;
	req->complete = ep0_queue_ack_complete;
	r->gadget_complete = 0;
	usb_ept_queue_xfer(ept, req);
}

static void handle_setup(struct usb_info *ui)
{
	struct usb_ctrlrequest ctl;
	struct usb_request *req = ui->setup_req;
	int ret;

	memcpy(&ctl, ui->ep0out.head->setup_data, sizeof(ctl));
	writel(EPT_RX(0), USB_ENDPTSETUPSTAT);

	if (ctl.bRequestType & USB_DIR_IN)
		ui->ep0_dir = USB_DIR_IN;
	else
		ui->ep0_dir = USB_DIR_OUT;

	/* any pending ep0 transactions must be canceled */
	flush_endpoint(&ui->ep0out);
	flush_endpoint(&ui->ep0in);

#if 0
	INFO("setup: type=%02x req=%02x val=%04x idx=%04x len=%04x\n",
	       ctl.bRequestType, ctl.bRequest, ctl.wValue,
	       ctl.wIndex, ctl.wLength);
#endif
	if ((ctl.bRequestType & (USB_DIR_IN | USB_TYPE_MASK)) ==
					(USB_DIR_IN | USB_TYPE_STANDARD)) {
		if (ctl.bRequest == USB_REQ_GET_STATUS) {
			if (ctl.wLength != 2)
				goto stall;
			switch (ctl.bRequestType & USB_RECIP_MASK) {
			case USB_RECIP_ENDPOINT:
			{
				struct msm_endpoint *ept;
				unsigned num =
					ctl.wIndex & USB_ENDPOINT_NUMBER_MASK;
				u16 temp = 0;

				if (num == 0) {
					memset(req->buf, 0, 2);
					break;
				}
				if (ctl.wIndex & USB_ENDPOINT_DIR_MASK)
					num += 16;
				ept = &ui->ep0out + num;
				temp = usb_ep_get_stall(ept);
				temp = temp << USB_ENDPOINT_HALT;
				memcpy(req->buf, &temp, 2);
				break;
			}
			case USB_RECIP_DEVICE:
			{
				u16 temp = 0;

				temp = 1 << USB_DEVICE_SELF_POWERED;
				temp |= (ui->remote_wakeup <<
						USB_DEVICE_REMOTE_WAKEUP);
				memcpy(req->buf, &temp, 2);
				break;
			}
			case USB_RECIP_INTERFACE:
				memset(req->buf, 0, 2);
				break;
			default:
				goto stall;
			}
			ep0_setup_send(ui, 2);
			return;
		}
	}
	if (ctl.bRequestType ==
		    (USB_DIR_OUT | USB_TYPE_STANDARD | USB_RECIP_ENDPOINT)) {
		if ((ctl.bRequest == USB_REQ_CLEAR_FEATURE) ||
				(ctl.bRequest == USB_REQ_SET_FEATURE)) {
			if ((ctl.wValue == 0) && (ctl.wLength == 0)) {
				unsigned num = ctl.wIndex & 0x0f;

				if (num != 0) {
					struct msm_endpoint *ept;

					if (ctl.wIndex & 0x80)
						num += 16;
					ept = &ui->ep0out + num;

					if (ctl.bRequest == USB_REQ_SET_FEATURE)
						msm72k_set_halt(&ept->ep, 1);
					else
						msm72k_set_halt(&ept->ep, 0);
				}
				goto ack;
			}
		}
	}
	if (ctl.bRequestType == (USB_DIR_OUT | USB_TYPE_STANDARD)) {
		if (ctl.bRequest == USB_REQ_SET_CONFIGURATION)
			ui->online = !!ctl.wValue;
		else if (ctl.bRequest == USB_REQ_SET_ADDRESS) {
			/* write address delayed (will take effect
			** after the next IN txn)
			*/
			writel((ctl.wValue << 25) | (1 << 24), USB_DEVICEADDR);
			goto ack;
		} else if (ctl.bRequest == USB_REQ_SET_FEATURE) {
			switch (ctl.wValue) {
			case USB_DEVICE_TEST_MODE:
				switch (ctl.wIndex) {
				case J_TEST:
				case K_TEST:
				case SE0_NAK_TEST:
				case TST_PKT_TEST:
					ui->test_mode = ctl.wIndex;
					goto ack;
				}
				goto stall;
			case USB_DEVICE_REMOTE_WAKEUP:
				ui->remote_wakeup = 1;
				goto ack;
			}
		} else if ((ctl.bRequest == USB_REQ_CLEAR_FEATURE) &&
				(ctl.wValue == USB_DEVICE_REMOTE_WAKEUP)) {
			ui->remote_wakeup = 0;
			goto ack;
		}
	}

	/* delegate if we get here */
	if (ui->driver) {
		ret = ui->driver->setup(&ui->gadget, &ctl);
		if (ret >= 0)
			return;
	}

stall:
	/* stall ep0 on error */
	ep0_setup_stall(ui);
	return;

ack:
	ep0_setup_ack(ui);
}

static void handle_endpoint(struct usb_info *ui, unsigned bit)
{
	struct msm_endpoint *ept = ui->ept + bit;
	struct msm_request *req;
	unsigned long flags;
	unsigned info;

	/*
	INFO("handle_endpoint() %d %s req=%p(%08x)\n",
		ept->num, (ept->flags & EPT_FLAG_IN) ? "in" : "out",
		ept->req, ept->req ? ept->req->item_dma : 0);
	*/

	/* expire all requests that are no longer active */
	spin_lock_irqsave(&ui->lock, flags);
	while ((req = ept->req)) {
		info = req->item->info;

		/* if we've processed all live requests, time to
		 * restart the hardware on the next non-live request
		 */
		if (!req->live) {
			usb_ept_start(ept);
			break;
		}

		/* if the transaction is still in-flight, stop here */
		if (info & INFO_ACTIVE)
			break;

		/* advance ept queue to the next request */
		ept->req = req->next;
		if (ept->req == 0)
			ept->last = 0;

		dma_unmap_single(NULL, req->dma, req->req.length,
				 (ept->flags & EPT_FLAG_IN) ?
				 DMA_TO_DEVICE : DMA_FROM_DEVICE);

		if (info & (INFO_HALTED | INFO_BUFFER_ERROR | INFO_TXN_ERROR)) {
			/* XXX pass on more specific error code */
			req->req.status = -EIO;
			req->req.actual = 0;
			INFO("msm72k_udc: ept %d %s error. info=%08x\n",
			       ept->num,
			       (ept->flags & EPT_FLAG_IN) ? "in" : "out",
			       info);
		} else {
			req->req.status = 0;
			req->req.actual =
				req->req.length - ((info >> 16) & 0x7FFF);
		}
		req->busy = 0;
		req->live = 0;
		if (req->dead)
			do_free_req(ui, req);

		if (req->req.complete) {
			spin_unlock_irqrestore(&ui->lock, flags);
			req->req.complete(&ept->ep, &req->req);
			spin_lock_irqsave(&ui->lock, flags);
		}
	}
	spin_unlock_irqrestore(&ui->lock, flags);
}

static void flush_endpoint_hw(struct usb_info *ui, unsigned bits)
{
	/* flush endpoint, canceling transactions
	** - this can take a "large amount of time" (per databook)
	** - the flush can fail in some cases, thus we check STAT
	**   and repeat if we're still operating
	**   (does the fact that this doesn't use the tripwire matter?!)
	*/
	do {
		writel(bits, USB_ENDPTFLUSH);
		while (readl(USB_ENDPTFLUSH) & bits)
			udelay(100);
	} while (readl(USB_ENDPTSTAT) & bits);
}

static void flush_endpoint_sw(struct msm_endpoint *ept)
{
	struct usb_info *ui = ept->ui;
	struct msm_request *req;
	unsigned long flags;

	/* inactive endpoints have nothing to do here */
	if (ept->ep.maxpacket == 0)
		return;

	/* put the queue head in a sane state */
	ept->head->info = 0;
	ept->head->next = TERMINATE;

	/* cancel any pending requests */
	spin_lock_irqsave(&ui->lock, flags);
	req = ept->req;
	ept->req = 0;
	ept->last = 0;
	while (req != 0) {
		req->busy = 0;
		req->live = 0;
		req->req.status = -ECONNRESET;
		req->req.actual = 0;
		if (req->req.complete) {
			spin_unlock_irqrestore(&ui->lock, flags);
			req->req.complete(&ept->ep, &req->req);
			spin_lock_irqsave(&ui->lock, flags);
		}
		if (req->dead)
			do_free_req(ui, req);
		req = req->next;
	}
	spin_unlock_irqrestore(&ui->lock, flags);
}

static void flush_endpoint(struct msm_endpoint *ept)
{
	flush_endpoint_hw(ept->ui, (1 << ept->bit));
	flush_endpoint_sw(ept);
}

static void flush_all_endpoints(struct usb_info *ui)
{
	unsigned n;

	flush_endpoint_hw(ui, 0xffffffff);

	for (n = 0; n < 32; n++)
		flush_endpoint_sw(ui->ept + n);
}

static void usb_vbus_draw_work(struct work_struct *w)
{
	struct usb_info *ui = container_of(w, struct usb_info, vbus_draw_work);

#ifdef CONFIG_USB_GADGET_EVENT
	gadget_event_power_state_changed(G_EV_SOURCE_BUS, ui->mA);
#endif
}

static void usb_bus_suspend_work(struct work_struct *w)
{
	struct usb_info *ui =
		container_of(w, struct usb_info, bus_suspend_work.work);
	unsigned long flags;

	spin_lock_irqsave(&ui->lock, flags);
	if (ui->host_present == 0) {
		INFO("ignoring bus_suspend due to no host\n");
		spin_unlock_irqrestore(&ui->lock, flags);
		return;
	}
	ui->suspended = 1;
	ui->mA_to_resume = ui->mA;
	spin_unlock_irqrestore(&ui->lock, flags);
#ifdef CONFIG_USB_GADGET_EVENT
	gadget_event_bus_suspended(1);
#endif
#ifdef STOP_DRAW_CURRENT_ON_BUS_SUSPEND
	msm72k_vbus_draw(&ui->gadget, 0); /* draw 0mA */
#endif
}

static void usb_bus_resume_work(struct work_struct *w)
{
	struct usb_info *ui =
		container_of(w, struct usb_info, bus_resume_work.work);
	unsigned long flags;

	spin_lock_irqsave(&ui->lock, flags);
	if (ui->resume_work_pending) {
		ui->resume_work_pending = 0;
#ifdef CONFIG_USB_GADGET_EVENT
		spin_unlock_irqrestore(&ui->lock, flags);
		gadget_event_bus_suspended(0);
		spin_lock_irqsave(&ui->lock, flags);
#endif
	}
	spin_unlock_irqrestore(&ui->lock, flags);
}

static irqreturn_t usb_interrupt(int irq, void *data)
{
	struct usb_info *ui = data;
	unsigned n;
	unsigned m;

	if (ui->in_lpm) {
		usb_lpm_exit(ui);
		/*
		   For an incoming call during LPM, a unknown
		   interrupt occurs without even attaching USB
		   cable. Since this interrupt is not followed by any
		   OTG interrupts, put our internal state machine into
		   a wrong state. We make a RESET request to handle
		   this.
		*/
		INFO("async interrupt: exit LPM\n");
		return IRQ_HANDLED;
	}

	cancel_delayed_work(&ui->bus_suspend_work);

	m = readl(USB_OTGSC);
	writel(m & 0xffff0000, USB_OTGSC);

	n = readl(USB_USBSTS);
	writel(n, USB_USBSTS);

	if (m & (OTG_BSEIS|OTG_BSVIS)) {
		INFO("otg: %s%s%s%s%s%s\n",
		     (m & OTG_BSEIE) ? "BSEIE," : "",
		     (m & OTG_BSVIE) ? "BSVIE," : "",
		     (m & OTG_BSEIS) ? "BSEIS," : "",
		     (m & OTG_BSVIS) ? "BSVIS," : "",
		     (m & OTG_BSE) ? "BSE," : "",
		     (m & OTG_BSV) ? "BSV," : "");
	}

	if (m & OTG_BSEIS) {
		if (m & OTG_BSE) {
			INFO("b-session end\n");
			msm_hsusb_set_vbus_state(0);
			msm72k_vbus_draw(&ui->gadget, 0);
		} else {
			INFO("ignore b-session end interrupt\n");
		}
	}
	if (m & OTG_BSVIS) {
		if (m & OTG_BSV) {
			INFO("b-session valid\n");
			msm_hsusb_set_vbus_state(1);
		} else {
			INFO("ignore b-session valid interrupt\n");
		}
	}

	/* somehow we got an IRQ while in the reset sequence: ignore it */
	if (ui->running == 0 && (n & (STS_PCI|STS_URI|STS_SLI|STS_UI))) {
		INFO("ignore status intrrupt(s)\n");
		return IRQ_HANDLED;
	}

	if (n & STS_PCI) {
		switch (readl(USB_PORTSC) & PORTSC_PSPD_MASK) {
		case PORTSC_PSPD_FS:
			INFO("portchange USB_SPEED_FULL\n");
			ui->gadget.speed = USB_SPEED_FULL;
			break;
		case PORTSC_PSPD_LS:
			INFO("portchange USB_SPEED_LOW\n");
			ui->gadget.speed = USB_SPEED_LOW;
			break;
		case PORTSC_PSPD_HS:
			INFO("portchange USB_SPEED_HIGH\n");
			ui->gadget.speed = USB_SPEED_HIGH;
			break;
		}
		if (ui->suspended) {
			ui->suspended = 0;
			msm72k_vbus_draw(&ui->gadget, ui->mA_to_resume);
			ui->mA_to_resume = 0;
			ui->resume_work_pending = 1;
			queue_delayed_work(ui->wq, &ui->bus_resume_work,
					   msecs_to_jiffies(500));
		}
	}

	if (n & STS_URI) {
		INFO("reset\n");
		ui->reset_detected = 1;

		writel(readl(USB_ENDPTSETUPSTAT), USB_ENDPTSETUPSTAT);
		writel(readl(USB_ENDPTCOMPLETE), USB_ENDPTCOMPLETE);
		writel(0xffffffff, USB_ENDPTFLUSH);
		writel(0, USB_ENDPTCTRL(1));

		if (ui->online) {
			/* marking us offline will cause ept queue attempts
			** to fail
			*/
			ui->online = 0;

			flush_all_endpoints(ui);

			msm72k_vbus_draw(&ui->gadget, 0);

			if (ui->driver) {
				INFO("driver disconnect\n");
				ui->driver->disconnect(&ui->gadget);
			}
		}
		msm72k_vbus_draw(&ui->gadget, 100);
	}

	if (n & STS_SLI) {
		INFO("suspend\n");

		queue_delayed_work(ui->wq, &ui->bus_suspend_work,
				   msecs_to_jiffies(500));
	}

	if (n & STS_UI) {
		n = readl(USB_ENDPTSETUPSTAT);
		if (n & EPT_RX(0))
			handle_setup(ui);

		n = readl(USB_ENDPTCOMPLETE);
		writel(n, USB_ENDPTCOMPLETE);
		while (n) {
			unsigned bit = __ffs(n);
			handle_endpoint(ui, bit);
			n = n & (~(1 << bit));
		}
	}
	return IRQ_HANDLED;
}

static void usb_prepare(struct usb_info *ui)
{
	spin_lock_init(&ui->lock);

	memset(ui->buf, 0, 4096);
	ui->head = (void *) (ui->buf + 0);

	/* only important for reset/reinit */
	memset(ui->ept, 0, sizeof(ui->ept));
	ui->next_item = 0;
	ui->next_ifc_num = 0;

	init_endpoints(ui);

	ui->ep0in.ep.maxpacket = 64;
	ui->ep0out.ep.maxpacket = 64;

	ui->setup_req =
		usb_ept_alloc_req(&ui->ep0in, SETUP_BUF_SIZE, GFP_KERNEL);

	INIT_WORK(&ui->wakeup_phy, usb_lpm_wakeup_phy);
	INIT_WORK(&ui->work, usb_do_work);
	INIT_WORK(&ui->vbus_draw_work, usb_vbus_draw_work);
	INIT_DELAYED_WORK(&ui->bus_suspend_work, usb_bus_suspend_work);
	INIT_DELAYED_WORK(&ui->bus_resume_work, usb_bus_resume_work);
}

static int usb_wakeup_phy(struct usb_info *ui)
{
	int i;

	writel(readl(USB_USBCMD) & ~ULPI_STP_CTRL, USB_USBCMD);

	/* some circuits automatically clear PHCD bit */
	for (i = 0; i < 5 && (readl(USB_PORTSC) & PORTSC_PHCD); i++) {
		writel(readl(USB_PORTSC) & ~PORTSC_PHCD, USB_PORTSC);
		msleep(1);
	}

	if ((readl(USB_PORTSC) & PORTSC_PHCD)) {
		pr_err("%s: cannot clear phcd bit\n", __func__);
		return -1;
	}

	return 0;
}

static int usb_suspend_phy(struct usb_info *ui)
{
	int i;
	unsigned long flags;

	if (usb_is_online(ui))
		return -1;

	/* spec talks about following bits in LPM for external phy.
	 * But they are ignored because
	 * 1. disabling interface protection circuit: by disabling
	 * interface protection curcuit we cannot come out
	 * of lpm as async interrupts would be disabled
	 * 2. setting the suspendM bit: this bit would be set by usb
	 * controller once we set phcd bit.
	 */

	/* clearing latch register, keeping phy comparators ON and
	   turning off PLL are done because of h/w bugs */
	if (ulpi_read(ui, 0x14) == 0xffffffff) /* clear PHY interrupt latch register */
		return -1;
#if 0
	if (ui->vbus_sn_notif &&
			ui->usb_state == USB_STATE_NOTATTACHED)
		/* turn off OTG PHY comparators */
		ulpi_write(ui, 0x00, 0x30);
	else
#endif
	/* turn on the PHY comparators */
	if (ulpi_write(ui, 0x01, 0x30) < 0)
		return -1;

	/* turn off PLL on integrated phy */
	if (ulpi_write(ui, 0x08, 0x09) < 0)
		return -1;

	/* loop for large amount of time */
	for (i = 0; i < 500; i++) {
		spin_lock_irqsave(&ui->lock, flags);
		if (usb_is_online(ui)) {
			spin_unlock_irqrestore(&ui->lock, flags);
			return -1;
		}
		/* set phy to be in lpm */
		writel(readl(USB_PORTSC) | PORTSC_PHCD, USB_PORTSC);
		spin_unlock_irqrestore(&ui->lock, flags);

		msleep(1);
		if (readl(USB_PORTSC) & PORTSC_PHCD)
			goto blk_stp_sig;
	}

	if (!(readl(USB_PORTSC) & PORTSC_PHCD)) {
		pr_err("unable to set phcd of portsc reg\n");
		pr_err("Reset HW link and phy to recover from phcd error\n");
		usb_hw_reset(ui);
		return -1;
	}

	/* we have to set this bit again to work-around h/w bug */
	writel(readl(USB_PORTSC) | PORTSC_PHCD, USB_PORTSC);

blk_stp_sig:
	/* block the stop signal */
	writel(readl(USB_USBCMD) | ULPI_STP_CTRL, USB_USBCMD);

	return 0;
}

static void phy_post_reset(struct usb_info *ui, int *seq)
{
	while (seq[0] >= 0) {
		ulpi_write(ui, seq[0], seq[1]);
		seq += 2;
	}
}

static void phy_config_show(struct usb_info *ui)
{
	int i;
	int reg[4];

	for (i = 0; i < 4; i++)
		reg[i] = ulpi_read(ui, 0x30+i);

	INFO("phyconfig now: %x %x %x %x\n", reg[0], reg[1], reg[2], reg[3]);
}

static int usb_hw_reset(struct usb_info *ui)
{
	unsigned i;
	unsigned long timeout;

	/* reset the phy before resetting link */
	if (readl(USB_PORTSC) & PORTSC_PHCD)
		usb_wakeup_phy(ui);

	/* phy_reset() calls msm_ulpi_init() internally */
	if (ui->pdata->phy_reset)
		ui->pdata->phy_reset();

	msleep(1);

	/* RESET */
	writel(USBCMD_RESET, USB_USBCMD);
	timeout = jiffies + USB_LINK_RESET_TIMEOUT;
	while (readl(USB_USBCMD) & USBCMD_RESET) {
		if (time_after(jiffies, timeout)) {
			dev_err(&ui->pdev->dev, "usb link reset timeout\n");
			break;
		}
		msleep(1);
	}

	/* select DEVICE mode with SDIS active */
	writel((USBMODE_SDIS | USBMODE_DEVICE), USB_USBMODE);
	msleep(1);

	/* select ULPI phy */
	i = (readl(USB_PORTSC) & ~PORTSC_PTS);
	writel(i | PORTSC_PTS_ULPI, USB_PORTSC);

#if 0
	/* If composition contains mass storage only function, decrease USB
	 * interrupt latency to zero to increase usb mass storage performance
	 */
	if (ui->composition->functions == USB_MSC_ONLY_FUNC_MAP)
		writel((readl(USB_USBCMD) & ~USBCMD_ITC_MASK) | USBCMD_ITC(0),
								USB_USBCMD);
#endif
	writel((readl(USB_USBCMD) & ~USBCMD_ITC_MASK) | USBCMD_ITC(1),
								USB_USBCMD);

	if (ui->pdata->phy_post_reset_seq)
		phy_post_reset(ui, ui->pdata->phy_post_reset_seq);

	phy_config_show(ui); /* DEBUG */

#if 0
	/* REVISIT - is using AHB XTOR really more stable? */
	writel(0x0, USB_AHB_BURST); /* If XTOR_BYPASS=0, this should be 0 */
	writel(0x00, USB_AHB_MODE); /* XTOR_BYPASS=0 */
#endif
	writel(readl(USB_OTGSC) | OTG_BSEIE | OTGSC_BSVIE, USB_OTGSC);

	writel(ui->dma, USB_ENDPOINTLISTADDR);

	emsg_count = 0;

	return 0;
}

static void usb_reset(struct usb_info *ui)
{
	unsigned long flags;

	INFO("reset controller...\n");

	spin_lock_irqsave(&ui->lock, flags);
	ui->running = 0;
	spin_unlock_irqrestore(&ui->lock, flags);

#if 0
	/* we should flush and shutdown cleanly if already running */
	writel(0xffffffff, USB_ENDPTFLUSH);
	msleep(2);
#endif

	if (usb_hw_reset(ui)) {
		pr_info("%s: h/w reset failed\n", __func__);
		return;
	}

	configure_endpoints(ui);

	/* marking us offline will cause ept queue attempts to fail */
	ui->online = 0;

	/* terminate any pending transactions */
	flush_all_endpoints(ui);

	spin_lock_irqsave(&ui->lock, flags);
	ui->running = 1;
	spin_unlock_irqrestore(&ui->lock, flags);
}

static void usb_start(struct usb_info *ui)
{
	unsigned long flags;

	spin_lock_irqsave(&ui->lock, flags);
	ui->flags |= USB_FLAG_START;
	queue_work(ui->wq, &ui->work);
	spin_unlock_irqrestore(&ui->lock, flags);
}

static struct usb_info *the_usb_info;

static int usb_free(struct usb_info *ui, int ret)
{
	INFO("usb_free(%d)\n", ret);

	if (ui->irq)
		free_irq(ui->irq, 0);
	if (ui->pool)
		dma_pool_destroy(ui->pool);
	if (ui->dma)
		dma_free_coherent(&ui->pdev->dev, 4096, ui->buf, ui->dma);
	if (ui->addr)
		iounmap(ui->addr);
	if (ui->clk)
		clk_put(ui->clk);
	if (ui->pclk)
		clk_put(ui->pclk);
	if (ui->wq)
		destroy_workqueue(ui->wq);
	kfree(ui);
	return ret;
}

static int usb_is_online(struct usb_info *ui)
{
	/* we're online if we're in test mode, too */
	if (ui->test_mode){
		return -1;
	}

	/* continue lpm if bus is suspended or disconnected or stopped*/
	if (((readl(USB_PORTSC) & PORTSC_SUSP) == PORTSC_SUSP) ||
			((readl(USB_PORTSC) & PORTSC_CCS) == 0) ||
			((readl(USB_USBCMD) & USBCMD_RS) == 0)) {
		//pr_info("%s usb disconnected or stopped.\n", __func__);
		return 0;
	}

	pr_debug("usb is online\n");
	pr_debug("usbcmd:(%08x) usbsts:(%08x) portsc:(%08x)\n",
			readl(USB_USBCMD),
			readl(USB_USBSTS),
			readl(USB_PORTSC));
	return -1;
}
static void usb_vbus_online(struct usb_info *ui)
{
	if (ui->in_lpm) {
		usb_vreg_enable(ui);
		usb_clk_enable(ui);
		usb_wakeup_phy(ui);
		ui->in_lpm = 0;
	}

	usb_reset(ui);
}

static int usb_check_charger(struct usb_info *ui)
{
	int charger_found = 0;
	int host_found = 0;
	int val;
	unsigned long start, expires, now;
	unsigned long flags;

	start = jiffies;
	expires = start + msecs_to_jiffies(500); /* 500ms */

	for (;;) {
		now = jiffies;

		spin_lock_irqsave(&ui->lock, flags);
		if (ui->reset_detected) {
			spin_unlock_irqrestore(&ui->lock, flags);
			INFO("Reset detected (elapsed=%dms)\n",
			     jiffies_to_msecs(now - start));
			host_found = 1;
			break;
		}
		spin_unlock_irqrestore(&ui->lock, flags);

		val = ulpi_read(ui, 0x15); /* DEBUG register */
		if (val < 0) {
			ERROR("Can't read DEBUG register\n");
			break;
		}
#if 0
		if ((val & 0x3) == 0) { /* both D+ and D- are low */
			INFO("Both D+ and D- are low (elapsed=%dms)\n",
			     jiffies_to_msecs(now - start));
			break;
		}
#endif
		if ((val & 0x3) == 0x3) { /* both D+ and D- are high */
			INFO("Charger found (elapsed=%dms)\n",
			     jiffies_to_msecs(now - start));
			charger_found = 1;
			break;
		}
		if (time_after(now, expires)) {
			INFO("Charger not found (elapsed=%ums)\n",
			     jiffies_to_msecs(now - start));
			break;
		}
		spin_lock_irqsave(&ui->lock, flags);
		if (ui->reset_detected) {
			spin_unlock_irqrestore(&ui->lock, flags);
			INFO("Reset detected (elapsed=%dms)\n",
			     jiffies_to_msecs(now - start));
			host_found = 1;
			break;
		}
		spin_unlock_irqrestore(&ui->lock, flags);

		msleep(5);
	}
	if (charger_found) {
#ifdef CONFIG_USB_GADGET_EVENT
		gadget_event_power_state_changed(G_EV_SOURCE_CHARGER, 1000);
#endif
	} else if (host_found) {
		spin_lock_irqsave(&ui->lock, flags);
		ui->host_present = 1;
		spin_unlock_irqrestore(&ui->lock, flags);
#ifdef CONFIG_USB_GADGET_EVENT
		gadget_event_host_connected(1);
#endif
	} else {
		/* handle this case as 500mA charger found */
#ifdef CONFIG_USB_GADGET_EVENT
		gadget_event_power_state_changed(G_EV_SOURCE_CHARGER, 500);
#endif
		charger_found = 1;
	}
	return charger_found;
}

static void usb_do_work(struct work_struct *w)
{
	struct usb_info *ui = container_of(w, struct usb_info, work);
	unsigned long flags;
	unsigned reqs, _vbus;
	int cnt = 0;

	for (;;) {
		spin_lock_irqsave(&ui->lock, flags);
		reqs = ui->flags;
		ui->flags = 0;
		_vbus = vbus;
		spin_unlock_irqrestore(&ui->lock, flags);

		INFO("do_work(%d) state=%d reqs=%x\n", cnt++, ui->state, reqs);

		/* give up if we have nothing to do */
		if (reqs == 0)
			break;

		/* avoid any changes while in test_mode, too */
		if (ui->test_mode)
			break;

		switch (ui->state) {
		case USB_STATE_IDLE:

			if ((reqs & USB_FLAG_START) ||
			    (reqs & USB_FLAG_RESET)) {
				unsigned otgsc;

				CPUFREQ_HOLD_CHECK(&ui->cpufreq_flag);

				usb_clk_enable(ui);
				usb_vreg_enable(ui);
				usb_vbus_online(ui);

				/* adjust vbus */
				otgsc = readl(USB_OTGSC);
				if (otgsc & OTG_BSV) {
					INFO("b-session valid\n");

					vbus = 1;
					ui->reset_detected = 0;
					msm72k_pullup(&ui->gadget, 1);

					if (usb_check_charger(ui)) {
						/* unhold if it is a charger */
						CPUFREQ_UNHOLD_CHECK(&ui->cpufreq_flag);
					}
					ui->state = USB_STATE_ONLINE;
					INFO("IDLE -> ONLINE\n");
				} else {
					if (usb_lpm_enter(ui) < 0) {
						spin_lock_irqsave(&ui->lock, flags);
						ui->state = USB_STATE_IDLE;
						ui->flags |= USB_FLAG_RESET;
						spin_unlock_irqrestore(&ui->lock, flags);
						continue;
					}

					CPUFREQ_UNHOLD_CHECK(&ui->cpufreq_flag);

					ui->state = USB_STATE_OFFLINE;
					INFO("IDLE -> OFFLINE\n");
				}
			}
			break;
		case USB_STATE_ONLINE:

			/* If at any point when we were online, we received
			 * the signal to go offline, we must honor it
			 */
			if (reqs & USB_FLAG_VBUS_OFFLINE) {
				INFO("ONLINE -> OFFLINE\n");

				CPUFREQ_HOLD_CHECK(&ui->cpufreq_flag);

				/* synchronize with irq context */
				spin_lock_irqsave(&ui->lock, flags);
				ui->online = 0;
				msm72k_pullup(&ui->gadget, 0);
				spin_unlock_irqrestore(&ui->lock, flags);

				/* terminate any transactions, etc */
				flush_all_endpoints(ui);

				msm72k_vbus_draw(&ui->gadget, 0);

				if (ui->driver) {
					INFO("driver disconnect\n");
					ui->driver->disconnect(&ui->gadget);
				}

				/* power down phy, clock down usb */
				if (usb_lpm_enter(ui) < 0) {
					spin_lock_irqsave(&ui->lock, flags);
					ui->state = USB_STATE_IDLE;
					ui->flags |= USB_FLAG_RESET;
					spin_unlock_irqrestore(&ui->lock, flags);
					continue;
				}

				spin_lock_irqsave(&ui->lock, flags);
				if (ui->suspended) {
					ui->suspended = 0;
					ui->mA_to_resume = 0;
#ifdef CONFIG_USB_GADGET_EVENT
					spin_unlock_irqrestore(&ui->lock, flags);
					gadget_event_bus_suspended(0);
					spin_lock_irqsave(&ui->lock, flags);
#endif
				}
				if (ui->resume_work_pending) {
					ui->resume_work_pending = 0;
					cancel_delayed_work(&ui->bus_resume_work);
#ifdef CONFIG_USB_GADGET_EVENT
					spin_unlock_irqrestore(&ui->lock, flags);
					gadget_event_bus_suspended(0);
					spin_lock_irqsave(&ui->lock, flags);
#endif
				}
				ui->host_present = 0;
				spin_unlock_irqrestore(&ui->lock, flags);
#ifdef CONFIG_USB_GADGET_EVENT
				gadget_event_host_connected(0);
#endif
				CPUFREQ_UNHOLD_CHECK(&ui->cpufreq_flag);

				ui->state = USB_STATE_OFFLINE;
				break;
			}
			if (reqs & USB_FLAG_RESET) {
				spin_lock_irqsave(&ui->lock, flags);
				ui->flags |= USB_FLAG_RESET;
				ui->state = USB_STATE_IDLE;
				spin_unlock_irqrestore(&ui->lock, flags);
				INFO("ONLINE -> IDLE\n");
				break;
			}
			break;
		case USB_STATE_OFFLINE:
		
			/* If we were signaled to go online and vbus is still
			 * present when we received the signal, go online.
			 */
			if ((reqs & USB_FLAG_VBUS_ONLINE) && _vbus) {
				INFO("OFFLINE -> ONLINE\n");

				CPUFREQ_HOLD_CHECK(&ui->cpufreq_flag);

				if (ui->in_lpm)
					usb_lpm_exit(ui);
				usb_clk_enable(ui);
				usb_reset(ui);
				ui->reset_detected = 0;
				msm72k_pullup(&ui->gadget, 1);
				if (usb_check_charger(ui)) {
					/* unhold if it is a charger */
					CPUFREQ_UNHOLD_CHECK(&ui->cpufreq_flag);
				}
				ui->state = USB_STATE_ONLINE;
				break;
			}
			if (reqs & USB_FLAG_RESET) {
				spin_lock_irqsave(&ui->lock, flags);
				ui->flags |= USB_FLAG_RESET;
				ui->state = USB_STATE_IDLE;
				spin_unlock_irqrestore(&ui->lock, flags);
				INFO("OFFLINE -> IDLE\n");
				break;
			}
			break;
		}
	}
}

/* FIXME - the callers of this function should use a gadget API instead.
 * This is called from htc_battery.c and board-halibut.c
 * WARNING - this can get called before this driver is initialized.
 */
void msm_hsusb_set_vbus_state(int online)
{
	unsigned long flags;
	struct usb_info *ui = the_usb_info;

	if (ui) {
		spin_lock_irqsave(&ui->lock, flags);
		if (vbus != online) {
			vbus = online;
			if (online)
				ui->flags |= USB_FLAG_VBUS_ONLINE;
			else
				ui->flags |= USB_FLAG_VBUS_OFFLINE;
			queue_work(ui->wq, &ui->work);
		}
		spin_unlock_irqrestore(&ui->lock, flags);
	} else {
		printk(KERN_ERR "msm_hsusb_set_vbus_state called before driver initialized\n");
		vbus = online;
	}
}

void msm_ulpi_init(void)
{
	struct usb_info *ui = the_usb_info;
	int *seq = ui->pdata->phy_init_seq;

	if (!ui || !seq)
		return;

	/* select ULPI phy */
	writel(0x80000000, USB_PORTSC);

	while (seq[0] >= 0) {
		ulpi_write(ui, seq[0], seq[1]);
		seq += 2;
	}
}
EXPORT_SYMBOL(msm_ulpi_init);

#if defined(CONFIG_DEBUG_FS)

void usb_function_reenumerate(void)
{
	struct usb_info *ui = the_usb_info;

	/* disable and re-enable the D+ pullup */
	INFO("disable pullup\n");
	msm72k_pullup(&ui->gadget, 0);

	msleep(10);

	INFO("enable pullup\n");
	msm72k_pullup(&ui->gadget, 1);
}

static char debug_buffer[PAGE_SIZE];

static ssize_t debug_read_status(struct file *file, char __user *ubuf,
				 size_t count, loff_t *ppos)
{
	struct usb_info *ui = file->private_data;
	char *buf = debug_buffer;
	unsigned long flags;
	struct msm_endpoint *ept;
	struct msm_request *req;
	int n;
	int i = 0;

	spin_lock_irqsave(&ui->lock, flags);

	i += scnprintf(buf + i, PAGE_SIZE - i,
		   "regs: setup=%08x prime=%08x stat=%08x done=%08x\n",
		   readl(USB_ENDPTSETUPSTAT),
		   readl(USB_ENDPTPRIME),
		   readl(USB_ENDPTSTAT),
		   readl(USB_ENDPTCOMPLETE));
	i += scnprintf(buf + i, PAGE_SIZE - i,
		   "regs:   cmd=%08x   sts=%08x intr=%08x port=%08x\n",
		   readl(USB_USBCMD),
		   readl(USB_USBSTS),
		   readl(USB_USBINTR),
		   readl(USB_PORTSC));
	i += scnprintf(buf + i, PAGE_SIZE - i,
		       "regs:  mode=%08x otgsc=%08x\n\n",
		       readl(USB_USBMODE),
		       readl(USB_OTGSC));


	for (n = 0; n < 32; n++) {
		ept = ui->ept + n;
		if (ept->ep.maxpacket == 0)
			continue;

		i += scnprintf(buf + i, PAGE_SIZE - i,
			"ept%d %s cfg=%08x active=%08x next=%08x info=%08x\n",
			ept->num, (ept->flags & EPT_FLAG_IN) ? "in " : "out",
			ept->head->config, ept->head->active,
			ept->head->next, ept->head->info);

		for (req = ept->req; req; req = req->next)
			i += scnprintf(buf + i, PAGE_SIZE - i,
			"  req @%08x next=%08x info=%08x page0=%08x %c %c\n",
				req->item_dma, req->item->next,
				req->item->info, req->item->page0,
				req->busy ? 'B' : ' ',
				req->live ? 'L' : ' ');
	}
	i += scnprintf(buf + i, PAGE_SIZE - i, "\n");

	i += scnprintf(buf + i, PAGE_SIZE - i, "vbus: %d\n", vbus);
	i += scnprintf(buf + i, PAGE_SIZE - i,
			   "phy status: %d\n", ui->phy_status);
	i += scnprintf(buf + i, PAGE_SIZE - i,
			   "phy failure count: %d\n", ui->phy_fail_count);

	spin_unlock_irqrestore(&ui->lock, flags);

	return simple_read_from_buffer(ubuf, count, ppos, buf, i);
}

static ssize_t debug_write_reset(struct file *file, const char __user *buf,
				 size_t count, loff_t *ppos)
{
	struct usb_info *ui = file->private_data;
	unsigned long flags;

	spin_lock_irqsave(&ui->lock, flags);
	ui->flags |= USB_FLAG_RESET;
	queue_work(ui->wq, &ui->work);
	spin_unlock_irqrestore(&ui->lock, flags);

	return count;
}

static ssize_t debug_write_cycle(struct file *file, const char __user *buf,
				 size_t count, loff_t *ppos)
{
	usb_function_reenumerate();
	return count;
}

static int debug_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

const struct file_operations debug_stat_ops = {
	.open = debug_open,
	.read = debug_read_status,
};

const struct file_operations debug_reset_ops = {
	.open = debug_open,
	.write = debug_write_reset,
};

const struct file_operations debug_cycle_ops = {
	.open = debug_open,
	.write = debug_write_cycle,
};

static void usb_debugfs_init(struct usb_info *ui)
{
	struct dentry *dent;
	dent = debugfs_create_dir("usb", 0);
	if (IS_ERR(dent))
		return;

	debugfs_create_file("status", 0444, dent, ui, &debug_stat_ops);
	debugfs_create_file("reset", 0222, dent, ui, &debug_reset_ops);
	debugfs_create_file("cycle", 0222, dent, ui, &debug_cycle_ops);
}
#else
static void usb_debugfs_init(struct usb_info *ui) {}
#endif

static int
msm72k_enable(struct usb_ep *_ep, const struct usb_endpoint_descriptor *desc)
{
	struct msm_endpoint *ept = to_msm_endpoint(_ep);
	unsigned char ep_type =
			desc->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK;

	_ep->maxpacket = le16_to_cpu(desc->wMaxPacketSize);
	config_ept(ept);
	usb_ept_enable(ept, 1, ep_type);
	return 0;
}

static int msm72k_disable(struct usb_ep *_ep)
{
	struct msm_endpoint *ept = to_msm_endpoint(_ep);

	usb_ept_enable(ept, 0, 0);
	return 0;
}

static struct usb_request *
msm72k_alloc_request(struct usb_ep *_ep, gfp_t gfp_flags)
{
	return usb_ept_alloc_req(to_msm_endpoint(_ep), 0, gfp_flags);
}

static void
msm72k_free_request(struct usb_ep *_ep, struct usb_request *_req)
{
	struct msm_request *req = to_msm_request(_req);
	struct msm_endpoint *ept = to_msm_endpoint(_ep);
	struct usb_info *ui = ept->ui;
	unsigned long flags;
	int dead = 0;

	spin_lock_irqsave(&ui->lock, flags);
	/* defer freeing resources if request is still busy */
	if (req->busy)
		dead = req->dead = 1;
	spin_unlock_irqrestore(&ui->lock, flags);

	/* if req->dead, then we will clean up when the request finishes */
	if (!dead)
		do_free_req(ui, req);
}

static int
msm72k_queue(struct usb_ep *_ep, struct usb_request *req, gfp_t gfp_flags)
{
	struct msm_endpoint *ep = to_msm_endpoint(_ep);
	struct usb_info *ui = ep->ui;

	if (ep == &ui->ep0in) {
		struct msm_request *r = to_msm_request(req);
		if (!req->length)
			goto ep_queue_done;
		r->gadget_complete = req->complete;
		/* ep0_queue_ack_complete queue a receive for ACK before
		** calling req->complete
		*/
		req->complete = ep0_queue_ack_complete;
		if (ui->ep0_dir == USB_DIR_OUT)
			ep = &ui->ep0out;
		else {
			if (req->zero) {
				/* if the sending data is shorter than
				   requested and ends on a packet boundary,
				   it needs to be followed by a zero length
				   packet.
				*/
				req->zero = (req->length & 63) == 0;
			}
		}
	}
ep_queue_done:
	return usb_ept_queue_xfer(ep, req);
}

static int msm72k_dequeue(struct usb_ep *_ep, struct usb_request *_req)
{
	struct msm_endpoint *ep = to_msm_endpoint(_ep);
	struct msm_request *req = to_msm_request(_req);
	struct usb_info *ui = ep->ui;

	struct msm_request *cur, *prev;
	unsigned long flags;

	if (!_ep || !_req)
		return -EINVAL;

	spin_lock_irqsave(&ui->lock, flags);
	cur = ep->req;
	prev = NULL;

	while (cur != 0) {
		if (cur == req) {
			req->busy = 0;
			req->live = 0;
			req->req.status = -ECONNRESET;
			req->req.actual = 0;
			if (req->req.complete) {
				spin_unlock_irqrestore(&ui->lock, flags);
				req->req.complete(&ep->ep, &req->req);
				spin_lock_irqsave(&ui->lock, flags);
			}
			if (req->dead)
				do_free_req(ui, req);
			/* remove from linked list */
			if (prev)
				prev->next = cur->next;
			else
				ep->req = cur->next;
			prev = cur;
			/* break from loop */
			cur = NULL;
		} else
			cur = cur->next;
	}
	spin_unlock_irqrestore(&ui->lock, flags);

	return 0;
}

static int
msm72k_set_halt(struct usb_ep *_ep, int value)
{
	struct msm_endpoint *ept = to_msm_endpoint(_ep);
	struct usb_info *ui = ept->ui;
	unsigned int in = ept->flags & EPT_FLAG_IN;
	unsigned int n;
	unsigned long flags;

	spin_lock_irqsave(&ui->lock, flags);
	n = readl(USB_ENDPTCTRL(ept->num));

	if (in) {
		if (value)
			n |= CTRL_TXS;
		else {
			n &= ~CTRL_TXS;
			n |= CTRL_TXR;
		}
	} else {
		if (value)
			n |= CTRL_RXS;
		else {
			n &= ~CTRL_RXS;
			n |= CTRL_RXR;
		}
	}
	writel(n, USB_ENDPTCTRL(ept->num));
	spin_unlock_irqrestore(&ui->lock, flags);

	return 0;
}

static int
msm72k_fifo_status(struct usb_ep *_ep)
{
	return -EOPNOTSUPP;
}

static void
msm72k_fifo_flush(struct usb_ep *_ep)
{
	flush_endpoint(to_msm_endpoint(_ep));
}

static const struct usb_ep_ops msm72k_ep_ops = {
	.enable		= msm72k_enable,
	.disable	= msm72k_disable,

	.alloc_request	= msm72k_alloc_request,
	.free_request	= msm72k_free_request,

	.queue		= msm72k_queue,
	.dequeue	= msm72k_dequeue,

	.set_halt	= msm72k_set_halt,
	.fifo_status	= msm72k_fifo_status,
	.fifo_flush	= msm72k_fifo_flush,
};

static int msm72k_get_frame(struct usb_gadget *_gadget)
{
	struct usb_info *ui = container_of(_gadget, struct usb_info, gadget);

	/* frame number is in bits 13:3 */
	return (readl(USB_FRINDEX) >> 3) & 0x000007FF;
}

/* VBUS reporting logically comes from a transceiver */
static int msm72k_udc_vbus_session(struct usb_gadget *_gadget, int is_active)
{
	msm_hsusb_set_vbus_state(is_active);
	return 0;
}

static int msm72k_pullup(struct usb_gadget *_gadget, int is_active)
{
	struct usb_info *ui = container_of(_gadget, struct usb_info, gadget);

	pr_info("%s: is_active=%d\n", __func__, is_active);

	disable_irq(ui->irq);
	if (is_active) {
		if (vbus && ui->driver) {
			writel(STS_URI | STS_SLI | STS_UI | STS_PCI, USB_USBINTR);
			writel(readl(USB_USBCMD) | USBCMD_RS, USB_USBCMD);
		} else {
			pr_warning("%s: ignored (vbus=%d ui->driver=%p)\n",
				   __func__, vbus, ui->driver);
		}
	} else {
		writel(readl(USB_USBINTR) & ~(STS_URI | STS_SLI | STS_UI | STS_PCI),
		       USB_USBINTR);
		writel(readl(USB_USBCMD) & ~USBCMD_RS, USB_USBCMD);

		/* Qualcomm's S/W workaround for Spoof Disconnect Failure
		 * Making opmode non-driving and SuspendM set in function
		 */
		ulpi_write(ui, 0x48, 0x04);
	}
	enable_irq(ui->irq);
	return 0;
}

static int msm72k_wakeup(struct usb_gadget *_gadget)
{
	struct usb_info *ui = container_of(_gadget, struct usb_info, gadget);
	unsigned long flags;

	if (!ui->remote_wakeup) {
		pr_err("%s: remote wakeup not supported\n", __func__);
		return -ENOTSUPP;
	}

	if (!ui->online) {
		pr_err("%s: device is not configured\n", __func__);
		return -ENODEV;
	}

	spin_lock_irqsave(&ui->lock, flags);
	if ((readl(USB_PORTSC) & PORTSC_SUSP) == PORTSC_SUSP) {
		pr_info("%s: enabling force resume\n", __func__);
		writel(readl(USB_PORTSC) | PORTSC_FPR, USB_PORTSC);
	}
	spin_unlock_irqrestore(&ui->lock, flags);

	return 0;
}

static int msm72k_vbus_draw(struct usb_gadget *_gadget, unsigned mA)
{
	struct usb_info *ui = container_of(_gadget, struct usb_info, gadget);

	ui->mA = mA;
	queue_work(ui->wq, &ui->vbus_draw_work);
	return 0;
}

static const struct usb_gadget_ops msm72k_ops = {
	.get_frame	= msm72k_get_frame,
	.vbus_session	= msm72k_udc_vbus_session,
	.pullup		= msm72k_pullup,
	.wakeup		= msm72k_wakeup,
	.vbus_draw	= msm72k_vbus_draw,
};

static ssize_t usb_remote_wakeup(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct usb_info *ui = the_usb_info;

	msm72k_wakeup(&ui->gadget);

	return count;
}
static DEVICE_ATTR(wakeup, S_IWUSR, 0, usb_remote_wakeup);

static int msm72k_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct usb_info *ui;
	int irq;
	int ret;

	INFO("msm72k_probe\n");
	ui = kzalloc(sizeof(struct usb_info), GFP_KERNEL);
	if (!ui)
		return -ENOMEM;

	ui->pdev = pdev;

	if (pdev->dev.platform_data)
		ui->pdata = pdev->dev.platform_data;

	irq = platform_get_irq(pdev, 0);
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res || (irq < 0))
		return usb_free(ui, -ENODEV);

	ui->addr = ioremap(res->start, 4096);
	if (!ui->addr)
		return usb_free(ui, -ENOMEM);

	ui->buf = dma_alloc_coherent(&pdev->dev, 4096, &ui->dma, GFP_KERNEL);
	if (!ui->buf)
		return usb_free(ui, -ENOMEM);

	ui->pool = dma_pool_create("msm72k_udc", NULL, 32, 32, 0);
	if (!ui->pool)
		return usb_free(ui, -ENOMEM);

	INFO("msm72k_probe() io=%p, irq=%d, dma=%p(%x)\n",
	       ui->addr, irq, ui->buf, ui->dma);

	ui->clk = clk_get(&pdev->dev, "usb_hs_clk");
	if (IS_ERR(ui->clk))
		return usb_free(ui, PTR_ERR(ui->clk));

	ui->pclk = clk_get(&pdev->dev, "usb_hs_pclk");
	if (IS_ERR(ui->pclk))
		return usb_free(ui, PTR_ERR(ui->pclk));

	ui->vreg = vreg_get(NULL, "usb");
	if (IS_ERR(ui->vreg) || (!ui->vreg)) {
		pr_err("%s: vreg get failed\n", __func__);
		ui->vreg = NULL;
		usb_free(ui, PTR_ERR(ui->pclk));
		return PTR_ERR(ui->pclk);
	}

	// Enable USBPHY3V3 once and always keep it enabled
	vreg_enable(ui->vreg);

	ret = request_irq(irq, usb_interrupt, IRQF_SHARED, pdev->name, ui);
	if (ret)
		return usb_free(ui, ret);
	enable_irq_wake(irq);
	ui->irq = irq;

	ui->gadget.ops = &msm72k_ops;
	ui->gadget.is_dualspeed = 1;
	device_initialize(&ui->gadget.dev);
	strcpy(ui->gadget.dev.bus_id, "gadget");
	ui->gadget.dev.parent = &pdev->dev;
	ui->gadget.dev.dma_mask = pdev->dev.dma_mask;

	ui->wq = create_singlethread_workqueue("msm72k_udc");
	if (ui->wq == NULL)
		return usb_free(ui, -ENOMEM);

	the_usb_info = ui;

	usb_debugfs_init(ui);

	usb_prepare(ui);

	return 0;
}

int usb_gadget_register_driver(struct usb_gadget_driver *driver)
{
	struct usb_info *ui = the_usb_info;
	int			retval, n;

	if (!driver
			|| driver->speed < USB_SPEED_FULL
			|| !driver->bind
			|| !driver->disconnect
			|| !driver->setup)
		return -EINVAL;
	if (!ui)
		return -ENODEV;
	if (ui->driver)
		return -EBUSY;

	/* first hook up the driver ... */
	ui->driver = driver;
	ui->gadget.dev.driver = &driver->driver;
	ui->gadget.name = driver_name;
	INIT_LIST_HEAD(&ui->gadget.ep_list);
	ui->gadget.ep0 = &ui->ep0in.ep;
	INIT_LIST_HEAD(&ui->gadget.ep0->ep_list);
	ui->gadget.speed = USB_SPEED_UNKNOWN;

	for (n = 1; n < 16; n++) {
		struct msm_endpoint *ept = ui->ept + n;
		list_add_tail(&ept->ep.ep_list, &ui->gadget.ep_list);
		ept->ep.maxpacket = 512;
	}
	for (n = 17; n < 32; n++) {
		struct msm_endpoint *ept = ui->ept + n;
		list_add_tail(&ept->ep.ep_list, &ui->gadget.ep_list);
		ept->ep.maxpacket = 512;
	}

	retval = device_add(&ui->gadget.dev);
	if (retval)
		goto fail;

	retval = driver->bind(&ui->gadget);
	if (retval) {
		INFO("bind to driver %s --> error %d\n",
				driver->driver.name, retval);
		device_del(&ui->gadget.dev);
		goto fail;
	}

	/* create sysfs node for remote wakeup */
	retval = device_create_file(&ui->gadget.dev, &dev_attr_wakeup);
	if (retval != 0)
		INFO("failed to create sysfs entry: (wakeup) error: (%d)\n",
					retval);
	INFO("registered gadget driver '%s'\n", driver->driver.name);

	ui->state = USB_STATE_IDLE;
	usb_start(ui);

	return 0;

fail:
	ui->driver = NULL;
	ui->gadget.dev.driver = NULL;
	return retval;
}
EXPORT_SYMBOL(usb_gadget_register_driver);

int usb_gadget_unregister_driver(struct usb_gadget_driver *driver)
{
	struct usb_info *ui = the_usb_info;

	if (!ui)
		return -ENODEV;
	if (!driver || driver != ui->driver || !driver->unbind)
		return -EINVAL;

	msm72k_vbus_draw(&ui->gadget, 0);

	/* Disable interrupts */
	writel(0, USB_USBINTR);
	writel(0, USB_OTGSC);

	INFO("unregister: calling set_vbus_state(0)\n");
	msm_hsusb_set_vbus_state(0);
	INFO("unregister: calling flush workqueue\n");
	flush_workqueue(ui->wq);

	device_remove_file(&ui->gadget.dev, &dev_attr_wakeup);
	driver->unbind(&ui->gadget);
	ui->gadget.dev.driver = NULL;
	ui->driver = NULL;

	device_del(&ui->gadget.dev);

	usb_vreg_disable(ui);
	usb_clk_disable(ui);

	VDEBUG("unregistered gadget driver '%s'\n", driver->driver.name);
	return 0;
}
EXPORT_SYMBOL(usb_gadget_unregister_driver);


static struct platform_driver usb_driver = {
	.probe = msm72k_probe,
	.driver = { .name = "msm_hsusb", },
};

static int __init init(void)
{
	return platform_driver_register(&usb_driver);
}
module_init(init);

static void __exit cleanup(void)
{
	platform_driver_unregister(&usb_driver);
}
module_exit(cleanup);

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR("Mike Lockwood, Brian Swetland");
MODULE_LICENSE("GPL");