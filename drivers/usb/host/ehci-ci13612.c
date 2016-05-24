 /*
  * drivers/usb/host/ehci-ci13612.c
  *
  * USB Host Controller Driver for LSI's ACP
  *
  * Copyright (C) 2010 LSI Inc.
  *
  * This program is free software; you can redistribute it and/or modify
  * it under the terms of the GNU General Public License as published by
  * the Free Software Foundation; either version 2 of the License, or
  * (at your option) any later version.
  *
  * This program is distributed in the hope that it will be useful,
  * but WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  * GNU General Public License for more details.
  *
  * You should have received a copy of the GNU General Public License
  * along with this program.
  */

#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/of_platform.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include "ehci-ci13612.h"


static int ci13612_ehci_halt(struct ehci_hcd *ehci);

#ifdef CONFIG_LSI_USB_SW_WORKAROUND
static void ci13612_usb_setup(struct usb_hcd *hcd)
{
	int USB_TXFIFOTHRES, VUSB_HS_TX_BURST;
	u32 devicemode;
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);
	u32 txfulltuning = 0;

	if ((of_find_compatible_node(NULL, NULL, "lsi,acp3500")
		!= NULL)
		|| (of_find_compatible_node(NULL, NULL, "lsi,axxia35xx")
		!= NULL)) {
		writel(3, USB_SBUSCFG);
		return;
	}

	/* Fix for HW errata 0002832: Settings of VUSB_HS_TX_BURST and
	 * TXFILLTUNING.
	 * TXFIFOTHRES should satisfy
	 * TXFIFOTHRES * VUSB_HS_TX_BURST >= MAXIMUM PACKET SIZE of packet
	 * relationship.
	 */
	VUSB_HS_TX_BURST = readl(USB_HWTXBUF) & 0x0f;
	USB_TXFIFOTHRES = (32 << 16);
	txfulltuning = (txfulltuning  & 0xffc0ffff) | USB_TXFIFOTHRES;
	writel(txfulltuning, (void __iomem *)USB_TXFILLTUNING);

	/* Fix for HW errata 9000556154: When operating in device mode Use
	 * Unspecified Length Bursts by setting SBUSCFG to 0x0, or use stream
	 * disable mode by setting USBMODE.SDIS to 0x1.
	 */
	devicemode = ehci_readl(ehci, hcd->regs + 0x1A8);

	if ((devicemode & 0x3) == 0x2) {
		/* device mode */
		writel(0x0, hcd->regs + 0x90);
	} else if ((devicemode & 0x3) == 0x3) {
		/* host mode */
		writel(0x6, hcd->regs + 0x90);
	}

	pr_err("ehci-ci13612 (ci13612_usb_setup): VUSB_HS_TX_BURST = 0x%x,USB_TXFIFOTHRES = 0x%x\n",
			VUSB_HS_TX_BURST, USB_TXFIFOTHRES);
}
#endif

/* called after powerup, by probe or system-pm "wakeup" */
static int ehci_ci13612_reinit(struct ehci_hcd *ehci)
{

#ifdef CONFIG_LSI_USB_SW_WORKAROUND
	/* S/W workarounds are not needed in AXM55xx */
	ci13612_usb_setup(ehci_to_hcd(ehci));
#endif
	return 0;
}


static int ci13612_ehci_init(struct usb_hcd *hcd)
{
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);
	int retval = 0;
	int len;

	/* EHCI registers start at offset 0x100 */
	ehci->caps = hcd->regs + 0x100;
	ehci->regs = hcd->regs + 0x100
		+ HC_LENGTH(ehci, ehci_readl(ehci, &ehci->caps->hc_capbase));
	len = HC_LENGTH(ehci, ehci_readl(ehci, &ehci->caps->hc_capbase));

	/* configure other settings */
	ehci->hcs_params = ehci_readl(ehci, &ehci->caps->hcs_params);
	hcd->has_tt = 1;

	ehci->sbrn = 0x20;

	/* Reset is only allowed on a stopped controller */
	ci13612_ehci_halt(ehci);

	/* reset controller */
	ehci_reset(ehci);

	/* data structure init */
	retval = ehci_init(hcd);
	if (retval)
		return retval;
	hcd->self.sg_tablesize = 0;

	retval = ehci_ci13612_reinit(ehci);

	return retval;
}

#ifdef CONFIG_LSI_USB_SW_WORKAROUND
/*
 * ci13612_fixup_usbcmd_rs
 *
 * Fix HW errata 0003256: Do not enable USBCMD.RS for some time after the USB
 * reset has been completed (PORTSCx.PR=0). This ensures that the host does not
 * send the SOF until the ULPI post reset processing has been completed. Note:
 * This workaround reduces the likelihood of this problem occuring, but it may
 * not entirely eliminate it.
 */
static int
ci13612_fixup_usbcmd_rs(struct ehci_hcd *ehci)
{
	u32 port_status;
	/* This workaround is not applicable to 3500 */
	if ((of_find_compatible_node(NULL, NULL, "lsi,acp3500")
		!= NULL)
		|| (of_find_compatible_node(NULL, NULL, "lsi,axxia35xx")
		!= NULL)) {
		return 0;
	}

	port_status = ehci_readl(ehci, &ehci->regs->port_status[0]);
	pr_info("ehci-ci13612: port_status = 0x%x\n", port_status);
	if (port_status & 0x100) {
		pr_err("ehci-ci13612: USB port is in reset status, not able to change HC status to run\n");
		return -EFAULT;
	}
	return 0;
}
#else
#define ci13612_fixup_usbcmd_rs(_ehci) (0)
#endif


#ifdef CONFIG_LSI_USB_SW_WORKAROUND
/*
 * ci13612_fixup_txpburst
 *
 * Fix for HW errata 9000373951: You can adjust the burst size and fill the
 * level to minimize under-run possibilities. In the failing case, the transfer
 * size was 96 bytes, the burst size was 16, and the fill threshold level was
 * set to 2. Because of this, the Host core issued the Out token when it
 * requested the second burst of data. If the burst size had been changed to 8,
 * and the fill level set to 3, then the core would have pre-fetched the 96
 * bytes before issuing the OUT token.
 */
static void
ci13612_fixup_txpburst(struct ehci_hcd *ehci)
{
	unsigned burst_size;

	/* This workaround is not applicable to 3500 */
	if ((of_find_compatible_node(NULL, NULL, "lsi,acp3500")
		!= NULL)
		|| (of_find_compatible_node(NULL, NULL, "lsi,axxia35xx")
		!= NULL)) {
		return;
	}

	burst_size = ehci_readl(ehci, &ehci->regs->reserved1[1]);
	burst_size = (burst_size & 0xffff00ff) | 0x4000;	/* TXPBURST */
	ehci_writel(ehci, burst_size, &ehci->regs->reserved1[1]);
}
#else
#define ci13612_fixup_txpburst(ehci) do { (void)ehci; } while (0)
#endif

static int ci13612_ehci_run(struct usb_hcd *hcd)
{
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);
	int retval;
	u32 tmp;

	retval = ci13612_fixup_usbcmd_rs(ehci);
	if (retval)
		return retval;


#ifndef CONFIG_LSI_USB_SW_WORKAROUND
	/* Setup AMBA interface to force INCR16 busts when possible */
	writel(3, USB_SBUSCFG);
#endif

	retval = ehci_run(hcd);
	if (retval)
		return retval;

	ci13612_fixup_txpburst(ehci);

#ifndef CONFIG_LSI_USB_SW_WORKAROUND
	/* Set ITC (bits [23:16]) to zero for interrupt on every micro-frame */
	tmp = ehci_readl(ehci, &ehci->regs->command);
	tmp &= 0xFFFF;
	ehci_writel(ehci, tmp & 0xFFFF, &ehci->regs->command);
#endif

	return retval;
}

static const struct hc_driver ci13612_hc_driver = {
	.description		= "ci13612_hcd",
	.product_desc		= "CI13612A EHCI USB Host Controller",
	.hcd_priv_size		= sizeof(struct ehci_hcd),
	.irq			= ehci_irq,
	.flags			= HCD_MEMORY | HCD_USB2 | HCD_BH,
	.reset			= ci13612_ehci_init,
	.start			= ci13612_ehci_run,
	.stop			= ehci_stop,
	.shutdown		= ehci_shutdown,
	.urb_enqueue		= ehci_urb_enqueue,
	.urb_dequeue		= ehci_urb_dequeue,
	.endpoint_disable	= ehci_endpoint_disable,
	.get_frame_number	= ehci_get_frame,
	.hub_status_data	= ehci_hub_status_data,
	.hub_control		= ehci_hub_control,
#if defined(CONFIG_PM)
	.bus_suspend		= ehci_bus_suspend,
	.bus_resume		= ehci_bus_resume,
#endif
	.relinquish_port	= ehci_relinquish_port,
	.port_handed_over	= ehci_port_handed_over,
};

static int ci13612_ehci_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct usb_hcd *hcd;
	void __iomem *gpreg_base;
	int irq;
	int retval;
	struct resource *res;

	if (usb_disabled())
		return -ENODEV;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_dbg(&pdev->dev, "error getting irq number\n");
		retval = irq;
		goto fail_create_hcd;
	}

	if (0 != irq_set_irq_type(irq, IRQ_TYPE_LEVEL_HIGH)) {
		dev_dbg(&pdev->dev, "set_irq_type() failed\n");
		retval = -EBUSY;
		goto fail_create_hcd;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "Error: resource addr %s setup!\n",
			dev_name(&pdev->dev));
		return -ENODEV;
	}


#ifndef CONFIG_LSI_USB_SW_WORKAROUND
	/* Device using 32-bit addressing */
	pdev->dev.coherent_dma_mask = DMA_BIT_MASK(32);
	pdev->dev.dma_mask = &pdev->dev.coherent_dma_mask;
#endif

	hcd = usb_create_hcd(&ci13612_hc_driver,
			&pdev->dev, dev_name(&pdev->dev));
	if (!hcd) {
		retval = -ENOMEM;
		goto fail_create_hcd;
	}

	hcd->rsrc_start = res->start;
	hcd->rsrc_len = resource_size(res);

	hcd->regs = of_iomap(np, 0);
	if (!hcd->regs) {
		dev_err(&pdev->dev, "of_iomap error\n");
		retval = -ENOMEM;
		goto fail_put_hcd;
	}

	gpreg_base = of_iomap(np, 1);
	if (!gpreg_base) {
		dev_warn(&pdev->dev, "of_iomap error can't map region 1\n");
		retval = -ENOMEM;
		goto fail_put_hcd;
	} else {
		/* Set address bits [39:32] to zero */
		writel(0x0, gpreg_base + 0x8);
#ifndef CONFIG_LSI_USB_SW_WORKAROUND
		/* hprot cachable and bufferable */
		writel(0xc, gpreg_base + 0x74);
#endif
		iounmap(gpreg_base);
	}

	retval = usb_add_hcd(hcd, irq, 0);
	if (retval == 0) {
		platform_set_drvdata(pdev, hcd);
		return retval;
	}

fail_put_hcd:
	usb_put_hcd(hcd);
fail_create_hcd:
	dev_err(&pdev->dev, "init %s fail, %d\n", dev_name(&pdev->dev), retval);
	return retval;
}

static int ci13612_ehci_remove(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);

	usb_remove_hcd(hcd);
	iounmap(hcd->regs);
	usb_put_hcd(hcd);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

static int ci13612_ehci_halt(struct ehci_hcd *ehci)
{
	u32     temp;

	temp = ehci_readl(ehci, &ehci->regs->command);
	temp &= ~CMD_RUN;
	ehci_writel(ehci, temp, &ehci->regs->command);

	return ehci_handshake(ehci, &ehci->regs->status,
		STS_HALT, STS_HALT, 16 * 125);
}

MODULE_ALIAS("platform:ci13612-ehci");

static struct of_device_id ci13612_match[] = {
	{
		.type	= "usb",
		.compatible = "lsi,acp-usb",
	},
	{
		.type	= "usb",
		.compatible = "acp-usb",
	},
	{},
};

static struct platform_driver ci13612_ehci_driver = {
	.probe = ci13612_ehci_probe,
	.remove = ci13612_ehci_remove,
	.driver = {
		.name = "ci13612-ehci",
		.of_match_table = ci13612_match,
	},

};
