/*
 * Driver for mailbox in OcteonTx2
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2018 Marvell
 *
 */
#include <linux/pci.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mailbox_controller.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>

#define DRV_NAME "mbox-thunderx"
#define BAR0 0

#define XCPX_DEVY_XCP_MBOX_LINT_OFFSET 0x000E1C00
#define XCP_TO_DEV_XCP_MBOX_LINT(xcp_core, device_id)  \
	(XCPX_DEVY_XCP_MBOX_LINT_OFFSET | \
	((uint64_t) xcp_core << 36) | \
	((uint64_t)device_id << 4))

#define AP0_TO_SCP_MBOX_LINT    XCP_TO_DEV_XCP_MBOX_LINT(SCP_INDEX, DEV_AP0)

#define SCP_INDEX    0x0
#define DEV_AP0      0x2

/*
 * Doorbell-Register: XCP(0..1)_DEV(0..7)_XCP_MBOX
 * Communication data from devices to XCP. When written, sets
 * XCP(0..1)_DEV(0..7)_XCP_MBOX.
 * PS: it doesn't matter what is written into this register,
 * Attempting to writing 'anything' would cause an interrupt
 * to the target!
 */

#define DONT_CARE_DATA               0xFF
#define XCPX_DEVY_XCP_MBOX_OFFSET    0x000E1000
#define XCP_TO_DEV_XCP_MBOX(xcp_core, device_id) \
	(XCPX_DEVY_XCP_MBOX_OFFSET | \
	((uint64_t) xcp_core << 36) | \
	((uint64_t)device_id << 4))

/* AP0-to-SCP doorbell */
#define AP0_TO_SCP_MBOX         XCP_TO_DEV_XCP_MBOX(SCP_INDEX, DEV_AP0)

#define INTR_STAT_OFS	0x0
#define INTR_SET_OFS	0x8
#define INTR_CLR_OFS	0x10

#define MHU_LP_OFFSET	0x0
#define MHU_HP_OFFSET	0x20
#define MHU_SEC_OFFSET	0x200
#define TX_REG_OFFSET	0x100

#define MHU_NUM_PCHANS	3	/* Secure, Non-Secure High and Low Priority */
#define MHU_CHAN_MAX	20	/* Max channels to save on unused RAM */

struct mvl_mhu_link {
	unsigned int irq;
	void __iomem *tx_reg;
	void __iomem *rx_reg;
};

struct mvl_mhu {
	void __iomem *base;
	struct mvl_mhu_link mlink[MHU_NUM_PCHANS];
	struct mbox_controller mbox;
	struct device *dev;
	const char *name;
};

/**
 * MVL MHU Mailbox platform specific configuration
 *
 * @num_pchans: Maximum number of physical channels
 * @num_doorbells: Maximum number of doorbells per physical channel
 */
struct mvl_mhu_mbox_pdata {
	unsigned int num_pchans;
	unsigned int num_doorbells;
	bool support_doorbells;
};

/**
 * MVL MHU Mailbox allocated channel information
 *
 * @mhu: Pointer to parent mailbox device
 * @pchan: Physical channel within which this doorbell resides in
 * @doorbell: doorbell number pertaining to this channel
 */
struct mvl_mhu_channel {
	struct mvl_mhu *mhu;
	unsigned int pchan;
	unsigned int doorbell;
};

static inline struct mbox_chan *
mvl_mhu_mbox_to_channel(struct mbox_controller *mbox,
		    unsigned int pchan, unsigned int doorbell)
{
	int i;
	struct mvl_mhu_channel *chan_info;

	for (i = 0; i < mbox->num_chans; i++) {
		chan_info = mbox->chans[i].con_priv;
		if (chan_info && chan_info->pchan == pchan &&
		    chan_info->doorbell == doorbell)
			return &mbox->chans[i];
	}

	dev_err(mbox->dev,
		"Channel not registered: physical channel: %d doorbell: %d\n",
		pchan, doorbell);

	return NULL;
}

static void mvl_mhu_mbox_clear_irq(struct mbox_chan *chan)
{
	struct mvl_mhu_channel *chan_info = chan->con_priv;
	void __iomem *base = chan_info->mhu->mlink[chan_info->pchan].rx_reg;

	writel_relaxed(BIT(chan_info->doorbell), base + INTR_CLR_OFS);
}

static unsigned int mvl_mhu_mbox_irq_to_pchan_num(struct mvl_mhu *mhu, int irq)
{
	unsigned int pchan;
	struct mvl_mhu_mbox_pdata *pdata = dev_get_platdata(mhu->dev);

	for (pchan = 0; pchan < pdata->num_pchans; pchan++)
		if (mhu->mlink[pchan].irq == irq)
			break;
	return pchan;
}

static struct mbox_chan *mvl_mhu_mbox_irq_to_channel(struct mvl_mhu *mhu,
						 unsigned int pchan)
{
	unsigned long bits;
	unsigned int doorbell;
	struct mbox_chan *chan = NULL;
	struct mbox_controller *mbox = &mhu->mbox;
	void __iomem *base = mhu->mlink[pchan].rx_reg;

	bits = readl_relaxed(base + INTR_STAT_OFS);
	if (!bits)
		/* No IRQs fired in specified physical channel */
		return NULL;

	/* An IRQ has fired, find the associated channel */
	for (doorbell = 0; bits; doorbell++) {
		if (!test_and_clear_bit(doorbell, &bits))
			continue;

		chan = mvl_mhu_mbox_to_channel(mbox, pchan, doorbell);
		if (chan)
			break;
	}

	return chan;
}

static irqreturn_t mvl_mhu_mbox_thread_handler(int irq, void *data)
{
	struct mbox_chan *chan;
	struct mvl_mhu *mhu = data;
	unsigned int pchan = mvl_mhu_mbox_irq_to_pchan_num(mhu, irq);

	while (NULL != (chan = mvl_mhu_mbox_irq_to_channel(mhu, pchan))) {
		mbox_chan_received_data(chan, NULL);
		mvl_mhu_mbox_clear_irq(chan);
	}

	return IRQ_HANDLED;
}

static bool mvl_mhu_doorbell_last_tx_done(struct mbox_chan *chan)
{
	struct mvl_mhu_channel *chan_info = chan->con_priv;
	void __iomem *base = chan_info->mhu->mlink[chan_info->pchan].tx_reg;

	if (readl_relaxed(base + INTR_STAT_OFS) & BIT(chan_info->doorbell))
		return false;

	return true;
}

static int mvl_mhu_doorbell_send_data(struct mbox_chan *chan, void *data)
{
	struct mvl_mhu_channel *chan_info = chan->con_priv;
	void __iomem *base = chan_info->mhu->mlink[chan_info->pchan].tx_reg;

	/* Send event to co-processor */
	writel_relaxed(BIT(chan_info->doorbell), base + INTR_SET_OFS);

	return 0;
}

static int mvl_mhu_doorbell_startup(struct mbox_chan *chan)
{
	mvl_mhu_mbox_clear_irq(chan);
	return 0;
}

static void mvl_mhu_doorbell_shutdown(struct mbox_chan *chan)
{
	struct mvl_mhu_channel *chan_info = chan->con_priv;
	struct mbox_controller *mbox = &chan_info->mhu->mbox;
	int i;

	for (i = 0; i < mbox->num_chans; i++)
		if (chan == &mbox->chans[i])
			break;

	if (mbox->num_chans == i) {
		dev_warn(mbox->dev, "Request to free non-existent channel\n");
		return;
	}

	/* Reset channel */
	mvl_mhu_mbox_clear_irq(chan);
	chan->con_priv = NULL;
}

static struct mbox_chan *mvl_mhu_mbox_xlate(struct mbox_controller *mbox,
					const struct of_phandle_args *spec)
{
	struct mvl_mhu *mhu = dev_get_drvdata(mbox->dev);
	struct mvl_mhu_mbox_pdata *pdata = dev_get_platdata(mhu->dev);
	struct mvl_mhu_channel *chan_info;
	struct mbox_chan *chan = NULL;
	unsigned int pchan = spec->args[0];
	unsigned int doorbell = pdata->support_doorbells ? spec->args[1] : 0;
	int i;

	/* Bounds checking */
	if (pchan >= pdata->num_pchans || doorbell >= pdata->num_doorbells) {
		dev_err(mbox->dev,
			"Invalid channel requested pchan: %d doorbell: %d\n",
			pchan, doorbell);
		return ERR_PTR(-EINVAL);
	}

	for (i = 0; i < mbox->num_chans; i++) {
		chan_info = mbox->chans[i].con_priv;

		/* Is requested channel free? */
		if (chan_info &&
		    mbox->dev == chan_info->mhu->dev &&
		    pchan == chan_info->pchan &&
		    doorbell == chan_info->doorbell) {
			dev_err(mbox->dev, "Channel in use\n");
			return ERR_PTR(-EBUSY);
		}

		/*
		 * Find the first free slot, then continue checking
		 * to see if requested channel is in use
		 */
		if (!chan && !chan_info)
			chan = &mbox->chans[i];
	}

	if (!chan) {
		dev_err(mbox->dev, "No free channels left\n");
		return ERR_PTR(-EBUSY);
	}

	chan_info = devm_kzalloc(mbox->dev, sizeof(*chan_info), GFP_KERNEL);
	if (!chan_info)
		return ERR_PTR(-ENOMEM);

	chan_info->mhu = mhu;
	chan_info->pchan = pchan;
	chan_info->doorbell = doorbell;

	chan->con_priv = chan_info;

	dev_dbg(mbox->dev, "mbox: %s, created channel phys: %d doorbell: %d\n",
		mhu->name, pchan, doorbell);

	return chan;
}

#ifdef MVL_MHU_INTERRUPT_MODE
static irqreturn_t mvl_mhu_rx_interrupt(int irq, void *p)
{
	struct mvl_mhu *mhu = p;
	unsigned int pchan = mvl_mhu_mbox_irq_to_pchan_num(mhu, irq);
	struct mbox_chan *chan = mvl_mhu_mbox_to_channel(&mhu->mbox, pchan, 0);
	void __iomem *base = mhu->mlink[pchan].rx_reg;
	u32 val;

	val = readl_relaxed(base + INTR_STAT_OFS);
	if (!val)
		return IRQ_NONE;

	mbox_chan_received_data(chan, (void *)&val);

	writel_relaxed(val, base + INTR_CLR_OFS);

	return IRQ_HANDLED;
}
#endif

static bool mvl_mhu_last_tx_done(struct mbox_chan *chan)
{
	struct mvl_mhu_channel *chan_info = chan->con_priv;
	void __iomem *base = chan_info->mhu->mlink[chan_info->pchan].tx_reg;
	u32 val = readl_relaxed(base + INTR_STAT_OFS);

	return (val == 0);
}

static int mvl_mhu_send_data(struct mbox_chan *chan, void *data)
{
	struct mvl_mhu_channel *chan_info = chan->con_priv;
	void __iomem *base = chan_info->mhu->mlink[chan_info->pchan].tx_reg;

	writel_relaxed(DONT_CARE_DATA, base + AP0_TO_SCP_MBOX);
	return 0;
}

static int mvl_mhu_startup(struct mbox_chan *chan)
{
#if 0
	/* need to take care later */
	struct mvl_mhu_channel *chan_info = chan->con_priv;
	void __iomem *base = chan_info->mhu->mlink[chan_info->pchan].tx_reg;

	/* Clear the interrupt to SCP */
	writel(INT_AP_TO_SCP, base + AP0_TO_SCP_MBOX_LINT_ENA_W1C);

	/* Enable the interrupt from AP to SC */
	writel(INT_AP_TO_SCP, base + AP0_TO_SCP_MBOX_LINT_ENA_W1S);
#endif
	return 0;
}

static const struct mbox_chan_ops mvl_mhu_ops = {
	.send_data = mvl_mhu_send_data,
	.startup = mvl_mhu_startup,
	.last_tx_done = mvl_mhu_last_tx_done,
};

static const struct mbox_chan_ops mvl_mhu_doorbell_ops = {
	.send_data = mvl_mhu_doorbell_send_data,
	.startup = mvl_mhu_doorbell_startup,
	.shutdown = mvl_mhu_doorbell_shutdown,
	.last_tx_done = mvl_mhu_doorbell_last_tx_done,
};

static const struct mvl_mhu_mbox_pdata mvl_mhu_pdata = {
	.num_pchans = 3,
	.num_doorbells = 1,
	.support_doorbells = false,
};

static const struct mvl_mhu_mbox_pdata mvl_mhu_doorbell_pdata = {
	.num_pchans = 2,	/* Secure can't be used */
	.num_doorbells = 32,
	.support_doorbells = true,
};


static int mvl_mhu_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	u32 cell_count;
	int i, err, max_chans, ret;
	irq_handler_t handler;
	struct mvl_mhu *mhu;
	struct mbox_chan *chans;
	struct mvl_mhu_mbox_pdata *pdata;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
#ifdef MVL_MHU_INTERRUPT_MODE
	int mvl_mhu_reg[MHU_NUM_PCHANS] = {
		MHU_LP_OFFSET, MHU_HP_OFFSET, MHU_SEC_OFFSET,
	};
#endif
	if (!np)
		return -ENODEV;
	err = of_property_read_u32(np, "#mbox-cells", &cell_count);
	if (err) {
		dev_err(dev, "failed to read #mbox-cells\n");
		return err;
	}

	if (cell_count == 1) {
		max_chans = MHU_NUM_PCHANS;
		pdata = (struct mvl_mhu_mbox_pdata *)&mvl_mhu_pdata;
	} else if (cell_count == 2) {
		max_chans = MHU_CHAN_MAX;
		pdata = (struct mvl_mhu_mbox_pdata *)&mvl_mhu_doorbell_pdata;
	} else {
		dev_err(dev, "incorrect value of #mbox-cells in %s\n",
			np->full_name);
		return -EINVAL;
	}

	if (pdata->num_pchans > MHU_NUM_PCHANS) {
		dev_err(dev, "Number of physical channel can't exceed %d\n",
			MHU_NUM_PCHANS);
		return -EINVAL;
	}

	mhu = devm_kzalloc(dev, sizeof(*mhu), GFP_KERNEL);
	if (!mhu)
		return -ENOMEM;

#ifdef MVL_MHU_INTERRUPT_MODE
	err = pci_enable_device(adev);
	if (err)
		dev_err(dev, "Failed to enable PCI device: err %d\n", err);
#endif
	ret = pci_request_region(pdev, BAR0, DRV_NAME);
	if (ret)
		return ret;

	mhu->base = pcim_iomap(pdev, BAR0, pci_resource_len(pdev, BAR0));
	if (!mhu->base)
		return -EINVAL;

	err = of_property_read_string(np, "mbox-name", &mhu->name);
	if (err)
		mhu->name = np->full_name;

	chans = devm_kcalloc(dev, max_chans, sizeof(*chans), GFP_KERNEL);
	if (!chans)
		return -ENOMEM;

	dev->platform_data = pdata;

	mhu->dev = dev;
	mhu->mbox.dev = dev;
	mhu->mbox.chans = chans;
	mhu->mbox.num_chans = max_chans;
	mhu->mbox.txdone_irq = false;
	mhu->mbox.txdone_poll = true;
	mhu->mbox.txpoll_period = 1;

	mhu->mbox.of_xlate = mvl_mhu_mbox_xlate;
	pci_set_drvdata(pdev, mhu);

	if (pdata->support_doorbells) {
		mhu->mbox.ops = &mvl_mhu_doorbell_ops;
		handler = mvl_mhu_mbox_thread_handler;
	} else {
		mhu->mbox.ops = &mvl_mhu_ops;
#ifdef MVL_MHU_INTERRUPT_MODE
		handler = mvl_mhu_rx_interrupt;
#else
		handler = mvl_mhu_mbox_thread_handler;
#endif
	}

	err = mbox_controller_register(&mhu->mbox);
	if (err) {
		dev_err(dev, "Failed to register mailboxes %d\n", err);
		return err;
	}

#ifdef MVL_MHU_INTERRUPT_MODE
	for (i = 0; i < pdata->num_pchans; i++) {
		int irq = mhu->mlink[i].irq = adev->irq[i];

		if (irq <= 0) {
			dev_dbg(dev, "No IRQ found for Channel %d\n", i);
			continue;
		}

		mhu->mlink[i].rx_reg = mhu->base + mhu_reg[i];
		mhu->mlink[i].tx_reg = mhu->mlink[i].rx_reg + TX_REG_OFFSET;

		err = devm_request_threaded_irq(dev, irq, NULL, handler,
						IRQF_ONESHOT, "mhu_link", mhu);
		if (err) {
			dev_err(dev, "Can't claim IRQ %d\n", irq);
			mbox_controller_unregister(&mhu->mbox);
			return err;
		}
	}
#endif

	for (i = 0; i < pdata->num_pchans; i++)
		mhu->mlink[i].tx_reg = mhu->base;

#ifdef MVL_MHU_INTERRUPT_MODE
	mhu->mlink[i].tx_reg = mhu->base;
	ret = pci_alloc_irq_vectors(adev, 1, 1, PCI_IRQ_MSIX);
	if (ret < 0)
		dev_err(dev, "FAILED pci_alloc_irq_vectors: ret: %d\n", ret);

	ret = devm_request_irq(dev, pci_irq_vector(pdev, 0), handler, 0,
			       DRV_NAME, mhu);
	if (ret)
		dev_err(dev, "devm_request_irq FAILED ret : %d\n", ret);
#endif
	return 0;
}

static void mvl_mhu_remove(struct pci_dev *pdev)
{
	struct mvl_mhu *mhu = pci_get_drvdata(pdev);

	mbox_controller_unregister(&mhu->mbox);
	pcim_iounmap(pdev, mhu->base);
	pci_release_region(pdev, BAR0);
}

static const struct pci_device_id mvl_mhu_ids[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_CAVIUM, 0xA067) },
	{ 0, }	/* end of table */
};

static struct pci_driver mvl_mhu_driver = {
	.name	= "mhu",

	.id_table	= mvl_mhu_ids,
	.probe		= mvl_mhu_probe,
	.remove		= mvl_mhu_remove,
};
module_pci_driver(mvl_mhu_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("MVL MHU Driver");
MODULE_AUTHOR("Sujeet Baranwal <sbaranwal@marvell.com>");
