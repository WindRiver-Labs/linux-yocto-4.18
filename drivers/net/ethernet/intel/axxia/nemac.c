/*
 * Axxia NEMAC Gigabit Ethernet driver
 *
 * Copyright (C) 2015 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#define pr_fmt(_fmt) "[nemac] " _fmt

#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_net.h>
#include <linux/of_mdio.h>
#include <linux/crc32.h>
#include <linux/phy.h>
#include <linux/phy_fixed.h>
#include <linux/regmap.h>
#include <net/ip.h>
#include <net/ipv6.h>

#include "nemac-regs.h"

#define DESCRIPTOR_GRANULARITY   64

static int rx_num_desc = 128; /* Must be multiple of DESCRIPTOR_GRANULARITY */
module_param(rx_num_desc, int, S_IRUSR);
MODULE_PARM_DESC(rx_num_desc, "Number of receive descriptors");

static int tx_num_desc = 128; /* Must be multiple of DESCRIPTOR_GRANULARITY */
module_param(tx_num_desc, int, S_IRUSR);
MODULE_PARM_DESC(tx_num_desc, "Number of transmit descriptors");

/**
 * struct dma_desc - NEMAC DMA descriptor
 */
struct dma_desc {
	u32 ctrl;
#define D_SOP		(1 << 3)
#define D_EOP		(1 << 4)
#define D_INTR		(1 << 5)
#define D_RX_ERR	(1 << 6)
#define D_TX_NEGCRC	(1 << 6)
#define D_SWAP		(1 << 7)
#define D_TX_CRC	(1 << 8)
	u16 xfer_len;
	u16 pdu_len;
	u64 bufptr;
};

#define desc_idx(_desc) (int)(((((u64)(_desc)) >> 4) & 0xff))

/**
 * struct queue_ptr - Holds the state of the RX or TX queue
 *
 * @ring: Pointer to DMA descriptor ring.
 *
 * @phys_addr: DMA address of descriptor ring.
 *
 * @skb: List of skb pointers, one for each entry in the descriptor ring.
 *
 * @hw_tail_reg: Address to hardware tail pointer (updated by hardware). Points
 * to the next descriptor to be used for reception or transmission.
 *
 * @tail: Driver tail pointer. Follows hw_tail and points to next descriptor to
 * be completed.
 *
 * @head: Head pointer where the drivers puts the new buffers queued for
 * transmission, and the where fresh RX buffers are added.
 *
 * @size: Size in bytes of the descriptor ring.
 */
struct queue_ptr {
	struct dma_desc	*ring;
	dma_addr_t	phys_addr;
	struct sk_buff  **skb;
	void __iomem	*hw_tail_reg;
	u32		tail;
	u32		head;
	u32		size;
};

/* Driver private structure */
struct nemac_priv {
	struct device		*dev;
	struct net_device	*netdev;
	int			mac_irq;
	int			dma_irq;
	size_t			rxbuf_sz;

	struct napi_struct	napi ____cacheline_aligned;
	void __iomem		*reg;
	struct queue_ptr	rxq;
	struct queue_ptr	txq;
	spinlock_t		txlock; /* for TX queue */

	/* PHY device */
	struct device_node	*phy_dn;
	struct phy_device	*phy_dev;
	int			link_state;

	/* Stats */
	struct completion	stats_rdy;
};

#define DMA_POINTER_GEN		0x100000
#define DMA_POINTER_MASK	0x0fffff
#define dmaptr_idx(_val) (((_val) & DMA_POINTER_MASK)/sizeof(struct dma_desc))
#define dmaptr_gen(_val) (!!((_val) & DMA_POINTER_GEN))

static void
nemac_set(const struct nemac_priv *priv, u32 offset, u32 bits)
{
	u32 tmp;

	tmp = readl(priv->reg + offset);
	writel(tmp | bits, priv->reg + offset);
}

static void
nemac_clr(const struct nemac_priv *priv, u32 offset, u32 bits)
{
	u32 tmp;

	tmp = readl(priv->reg + offset);
	writel(tmp & ~bits, priv->reg + offset);
}

/* RX/TX-ring
 *
 * tail - Oldest descriptor, i.e. the next descriptor to be processed by RX/TX
 * interrupt. This pointer is only used by the driver (no corresponding
 * hardware register). The interrupt handler will process descriptors from tail
 * to hw_tail.
 *
 * hw_tail - Next descriptor to be processed by hardware. The memory location
 * is updated by the hardware when it switches descriptor (via DMA). A copy of
 * this value is also available in the DMAREG_[RX|TX]_TAIL register.
 *
 * head - Newest descriptor. This is where the driver adds new descriptors
 * (either fresh rx buffers or tx buffers queued for transmission) and the
 * pointer is updated in hardware via the DMAREG_[RX|TX]_HEAD register. The
 * hardware will process descriptors from hw_tail to head. When hw_tail ==
 * head, the ring is empty.
 *
 *		tail	hw_tail		head
 *		|	|		|
 *		V	V		V
 *      +-----+ +-----+ +-----+ +-----+ +-----+ +-----+
 *      |     | |     | |     | |     | |     | |     |
 *      +-----+ +-----+ +-----+ +-----+ +-----+ +-----+
 *
 */

/**
 * queue_get_head - Return next DMA descriptor from head of queue.
 */
static inline struct dma_desc *
queue_get_head(const struct queue_ptr *q)
{
	if ((q->head ^ q->tail) == DMA_POINTER_GEN)
		return NULL;
	return &q->ring[dmaptr_idx(q->head)];
}

/**
 * queue_get_tail - Return next DMA descriptor from tail of queue.
 */
static inline struct dma_desc *
queue_get_tail(const struct queue_ptr *q)
{
	if (q->tail == readl_relaxed(q->hw_tail_reg))
		return NULL;
	return &q->ring[dmaptr_idx(q->tail)];
}

/**
 * queue_set_skb - Assign skb to dma descriptor
 */
static inline void
queue_set_skb(const struct queue_ptr *q, const struct dma_desc *desc,
	      struct sk_buff *skb)
{
	q->skb[desc_idx(desc)] = skb;
}

/**
 * queue_get_skb - Return the skb associated with the given descriptor
 */
static inline struct sk_buff *
queue_get_skb(const struct queue_ptr *q, const struct dma_desc *desc)
{
	return q->skb[desc_idx(desc)];
}

/**
 * inc_pointer - Helper function to increment a DMA pointer. The counter is in
 * the lower bits and is incremented modulo the size of the ring. The bit
 * DMA_POINTER_GEN is toggled when the counter wraps.
 */
static inline u32
inc_pointer(u32 ptr, u32 size)
{
	u32 newptr = (ptr & DMA_POINTER_MASK) + sizeof(struct dma_desc);

	/* When counter wraps (on size), reset and toggle generation bit.
	 * Otherwise preserve generation bit
	 */
	if (newptr >= size)
		newptr = (ptr & DMA_POINTER_GEN) ^ DMA_POINTER_GEN;
	else
		newptr |= ptr & DMA_POINTER_GEN;

	return newptr;
}

static inline u32
queue_inc_head(struct queue_ptr *q)
{
	q->head = inc_pointer(q->head, q->size);
	return q->head;
}

static inline u32
queue_inc_tail(struct queue_ptr *q)
{
	q->tail = inc_pointer(q->tail, q->size);
	return q->tail;
}

static inline void
pr_queue(const char *tag, const struct queue_ptr *q)
{
	u32 hw_tail = readl_relaxed(q->hw_tail_reg);

	pr_debug("%s tail=%d.%lu hw_tail=%d.%lu head=%d.%lu\n",
		 tag,
		 dmaptr_gen(q->tail), dmaptr_idx(q->tail),
		 dmaptr_gen(hw_tail), dmaptr_idx(hw_tail),
		 dmaptr_gen(q->head), dmaptr_idx(q->head));
}

static inline void
pr_desc(const char *tag, const struct dma_desc *desc)
{
	pr_debug("%s #%-3u flags=%#x len=%u/%u buf=%#llx\n",
		 tag, desc_idx(desc),
		 le32_to_cpu(desc->ctrl),
		 le16_to_cpu(desc->xfer_len),
		 le16_to_cpu(desc->pdu_len),
		 le64_to_cpu(desc->bufptr));
}

static int
nemac_alloc_rx_buf(struct nemac_priv *priv, struct queue_ptr *q, int idx)
{
	struct device *dev = priv->dev;
	struct dma_desc *desc = &priv->rxq.ring[idx];
	struct sk_buff *skb;
	dma_addr_t p;

	BUG_ON(desc_idx(desc) != idx);

	skb = netdev_alloc_skb(priv->netdev, priv->rxbuf_sz);
	if (!skb) {
		pr_err("SKB alloc failed (%zu)\n", priv->rxbuf_sz);
		return -ENOMEM;
	}

	p = dma_map_single(dev, skb->data, priv->rxbuf_sz, DMA_FROM_DEVICE);
	if (dma_mapping_error(dev, p) != 0) {
		pr_err("SKB map failed\n");
		dev_kfree_skb_any(skb);
		return -ENOMEM;
	}
	desc->ctrl = cpu_to_le32(D_INTR | D_SWAP);
	desc->xfer_len = cpu_to_le16(priv->rxbuf_sz);
	desc->pdu_len = cpu_to_le16(priv->rxbuf_sz);
	desc->bufptr = cpu_to_le64(p);
	queue_set_skb(q, desc, skb);

	return 0;
}

static u64
nemac_rx_stat_counter(struct nemac_priv *priv, int counter)
{
	u64 result = readq(priv->reg + NEM_STATS_RX_R(counter));

	return (result << 32) | (result >> 32);
}

static u64
nemac_tx_stat_counter(struct nemac_priv *priv, int counter)
{
	u64 result = readq(priv->reg + NEM_STATS_TX_R(counter));

	return (result << 32) | (result >> 32);
}

#ifdef NEMAC_DUMP_STAT

static const char * const tx_stat_names[] = {
	"TX_OCTETS", "TX_BAD_OCTETS", "TX_FRM", "TX_BAD_FRM", "TX_MCAST",
	"TX_BCAST", "TX_PAUSE", "TX_FCS", "TX_DEFER", "TX_UNUSED1", "TX_1COL",
	"TX_MCOL", "TX_LCOL", "TX_XSCOL", "TX_FRAG", "TX_USIZE", "TX_64",
	"TX_65", "TX_128", "TX_256", "TX_512", "TX_1024", "TX_1519",
	"TX_OSIZE", "TX_JAB", "TX_URUN", "TX_STAT_OVFL",
};

static const char * const rx_stat_names[] = {
	"RX_OCTETS", "RX_BAD_OCTETS", "RX_FRM", "RX_BAD_FRM", "RX_MCAST",
	"RX_BCAST", "RX_CTL", "RX_PAUSE", "RX_UNK_OP", "RX_FCS", "RX_ALIGN",
	"RX_LENGTH", "RX_CODE", "RX_FRAG", "RX_USIZE", "RX_64", "RX_65",
	"RX_128", "RX_256", "RX_512", "RX_1024", "RX_1519", "RX_OSIZE",
	"RX_JAB", "RX_DROP", "RX_CARRIER", "RX_ORUN", "RX_STAT_OVFL",
};

static void
dump_stats(struct nemac_priv *priv)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(rx_stat_names); ++i) {
		u64 x = nemac_rx_stat_counter(priv, i);

		if (x)
			pr_info("  %13s = %12llx\n", rx_stat_names[i], x);
	}
	for (i = 0; i < ARRAY_SIZE(tx_stat_names); ++i) {
		u64 x = nemac_tx_stat_counter(priv, i);

		if (x)
			pr_info("  %13s = %12llx\n", tx_stat_names[i], x);
	}
}

#endif

static void
nemac_enable_dma_int(struct nemac_priv *priv)
{
	writel(INTE_RX_DONE | INTE_TX_DONE | INTE_ERR_MASK,
	       priv->reg + NEM_DMA_INTE_MASK);
}

static void
nemac_disable_dma_int(struct nemac_priv *priv)
{
	writel(0, priv->reg + NEM_DMA_INTE_MASK);
}

static void
nemac_link_up(struct nemac_priv *priv)
{
	struct phy_device *phy_dev = priv->phy_dev;

	u32 rgmii_clk, gmii_ctrl;

	switch (phy_dev->speed) {
	case 1000:
		rgmii_clk = MISCCTL_RGMII_1000;
		gmii_ctrl = SPEED_MODE_FORCE(2) | GOOD_LINK_FORCE;
		break;
	case 100:
		rgmii_clk = MISCCTL_RGMII_100;
		gmii_ctrl = SPEED_MODE_FORCE(1) | GOOD_LINK_FORCE;
		break;
	case 10:
	default:
		rgmii_clk = MISCCTL_RGMII_10;
		gmii_ctrl = SPEED_MODE_FORCE(0) | GOOD_LINK_FORCE;
		break;
	}

	if (phy_dev->duplex == DUPLEX_FULL)
		gmii_ctrl |= FULL_DUPLEX_FORCE;

	writel(gmii_ctrl, priv->reg + NEM_GMAC_ANEG_CTRL_R);
	writel(rgmii_clk, priv->reg + NEM_DMA_MISC_CTL);

	if (phy_dev->pause) {
		/* Enable GMAC and DMA to act on and send PAUSE frames */
		nemac_set(priv, NEM_GMAC_ENABLE_R,
			  GMAC_RX_PAUSE_EN | GMAC_TX_PAUSE_EN);
		nemac_set(priv, NEM_DMA_CTL, DMACTL_ALLOW_TX_PAUSE);
		pr_debug("Enable PAUSE\n");
	} else {
		/* Disable use of PAUSE frames */
		nemac_clr(priv, NEM_GMAC_ENABLE_R,
			  GMAC_RX_PAUSE_EN | GMAC_TX_PAUSE_EN);
		nemac_clr(priv, NEM_DMA_CTL, DMACTL_ALLOW_TX_PAUSE);
		pr_debug("Disable PAUSE\n");
	}

	/* Enable RX */
	nemac_set(priv, NEM_GMAC_ENABLE_R, GMAC_RX_EN);
	/* Enable DMA and interrupts */
	nemac_set(priv, NEM_DMA_ENABLE, RXDMA_EN | TXDMA_EN);
	nemac_enable_dma_int(priv);

	netif_start_queue(priv->netdev);
}

static void
nemac_link_down(struct nemac_priv *priv)
{
	netif_stop_queue(priv->netdev);

	/* Disable MAC RX */
	nemac_clr(priv, NEM_GMAC_ENABLE_R, GMAC_RX_EN);
	/* Disable DMA and interrupts */
	nemac_clr(priv, NEM_DMA_ENABLE, RXDMA_EN | TXDMA_EN);
	nemac_disable_dma_int(priv);
}

static void
nemac_adjust_link(struct net_device *ndev)
{
	struct nemac_priv *priv = netdev_priv(ndev);
	struct phy_device *phy_dev = priv->phy_dev;

	/* Link on or off change */
	if (phy_dev->link != priv->link_state) {
		priv->link_state = phy_dev->link;
		if (phy_dev->link)
			nemac_link_up(priv);
		else
			nemac_link_down(priv);

		phy_print_status(phy_dev);
	}
}

static int nemac_open(struct net_device *ndev)
{
	struct nemac_priv *priv = netdev_priv(ndev);

	priv->phy_dev = of_phy_connect(priv->netdev, priv->phy_dn,
				       nemac_adjust_link, 0,
				       PHY_INTERFACE_MODE_RGMII);
	if (IS_ERR_OR_NULL(priv->phy_dev)) {
		netdev_err(ndev, "Could not attach to PHY\n");
		return -ENODEV;
	}

	pr_debug("[%s] (phy %s)\n",
		 priv->phy_dev->drv->name, dev_name(&priv->phy_dev->dev));

	nemac_set(priv, NEM_DMA_CTL, DMACTL_EN | DMACTL_FORCE_RX_ORDER);
	napi_enable(&priv->napi);
	phy_start(priv->phy_dev);

	return 0;
}

static netdev_tx_t nemac_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct nemac_priv *priv = netdev_priv(ndev);
	struct dma_desc *desc;
	dma_addr_t addr;
	unsigned long flags;

	/* Pad undersized frames to minimum size */
	if (skb->len < ETH_ZLEN) {
		int padlen = ETH_ZLEN - skb->len;

		if (skb_pad(skb, padlen) != 0) {
			dev_warn_ratelimited(&ndev->dev, "Padding failed\n");
			dev_kfree_skb(skb);
			return NETDEV_TX_OK;
		}
		skb_put(skb, padlen);
	}

	spin_lock_irqsave(&priv->txlock, flags);
	desc = queue_get_head(&priv->txq);
	if (!desc) {
		spin_unlock_irqrestore(&priv->txlock, flags);
		netif_stop_queue(ndev);
		dev_warn_ratelimited(&ndev->dev, "No TX descriptors!\n");
		return NETDEV_TX_BUSY;
	}
	queue_set_skb(&priv->txq, desc, skb);

	addr = dma_map_single(priv->dev, skb->data, skb->len, DMA_TO_DEVICE);
	desc->ctrl = cpu_to_le32(D_INTR | D_SOP | D_EOP | D_SWAP | D_TX_CRC);
	desc->xfer_len = cpu_to_le16(skb->len);
	desc->pdu_len = desc->xfer_len;
	desc->bufptr = cpu_to_le64(addr);
	pr_desc("TX", desc);
	writel(queue_inc_head(&priv->txq), priv->reg + NEM_DMA_TXHEAD_PTR);
	spin_unlock_irqrestore(&priv->txlock, flags);
	ndev->trans_start = jiffies;
	pr_queue("XMIT", &priv->txq);

	return NETDEV_TX_OK;
}

static void nemac_tx_timeout(struct net_device *ndev)
{
}

static int nemac_stop(struct net_device *ndev)
{
	struct nemac_priv *priv = netdev_priv(ndev);

	pr_info("stop\n");
	writel(0, priv->reg + NEM_DMA_INTE_MASK);
	writel(0, priv->reg + NEM_DMA_ENABLE);
	netif_stop_queue(ndev);
	phy_disconnect(priv->phy_dev);
	napi_disable(&priv->napi);
	return 0;
}

/**
 * nemac_get_sta_address - Read the STA address from NEMAC registers
 */
static void
nemac_get_sta_address(struct nemac_priv *priv, u8 *addr)
{
	u32 upper, lower;

	upper = readl(priv->reg + NEM_GMAC_STA_ADDR_UPPER_R);
	lower = readl(priv->reg + NEM_GMAC_STA_ADDR_R);
	addr[0] = (upper >>  8) & 0xff;
	addr[1] = (upper >>  0) & 0xff;
	addr[2] = (lower >> 24) & 0xff;
	addr[3] = (lower >> 16) & 0xff;
	addr[4] = (lower >>  8) & 0xff;
	addr[5] = (lower >>  0) & 0xff;
}

/**
 * nemac_set_mac_address - Write Ethernet address to NEMAC registers (specified
 * via offsets to both lower and upper part)
 */
static void
nemac_set_mac_address(struct nemac_priv *priv, const u8 *addr,
		      int reg_lower, int reg_upper)
{
	u32 upper, lower;

	upper = (addr[0] << 8) | addr[1];
	lower = (addr[2] << 24) | (addr[3] << 16) | (addr[4] << 8) | addr[5];
	writel(upper, priv->reg + reg_upper);
	writel(lower, priv->reg + reg_lower);
}

/**
 * nemac_update_multicast - Update the 128-bit multicast hash filter with the
 * list of multicast addresses from the interface. The hash (index into
 * bitmask) is bits 30..24 of the CRC of each ethernet address.
 */
static void
nemac_update_multicast(struct nemac_priv *priv)
{
	struct netdev_hw_addr *ha;

	/* Iterate the multicast list and program hash bits */
	netdev_for_each_mc_addr(ha, priv->netdev) {
		u32 crc = ether_crc(ETH_ALEN, ha->addr);
		u32 hash = (crc > 24) & 0x7f;
		u32 reg = (hash >> 5) & 0x3;
		u32 bit = BIT(hash & 0x1f);
		u32 tmp = readl(priv->reg + NEM_GMAC_MULTICAST_R(reg));

		writel(tmp | bit, priv->reg + NEM_GMAC_MULTICAST_R(reg));
	}
}

/**
 * nemac_set_rx_mode - Update MAC filter flags according to the interface
 * settings.
 */
static void nemac_set_rx_mode(struct net_device *dev)
{
	struct nemac_priv *priv = netdev_priv(dev);
	u32 filter = (ENABLE_RUNT_FILTERING |
		      ENABLE_PAUSE_FILTERING |
		      ENABLE_UNICAST_FILTERING);

	if (!(dev->flags & IFF_PROMISC))
		filter |= PACKET_FILTERING_ENABLE;

	if (!(dev->flags & IFF_ALLMULTI)) {
		filter |= ENABLE_MULTICAST_FILTERING;
	} else {
		int i;

		for (i = 0; i < 4; ++i)
			writel(0, priv->reg + NEM_GMAC_MULTICAST_R(i));

		if (!netdev_mc_empty(dev))
			nemac_update_multicast(priv);
	}
	writel(filter, priv->reg + NEM_GMAC_FILTER_CTRL_R);
}

/**
 * nemac_set_features - No special features supported.
 */
static int nemac_set_features(struct net_device *dev,
			      netdev_features_t features)
{
	return 0;
}

/**
 * nemac_tx_cleanup - Release all transmitted skb from the tail of the TX
 * queue.
 */
static int
nemac_tx_cleanup(struct nemac_priv *priv)
{
	struct dma_desc *desc;
	unsigned long flags;
	int complete = 0;

	spin_lock_irqsave(&priv->txlock, flags);
	while ((desc = queue_get_tail(&priv->txq)) != NULL) {
		struct sk_buff *skb = queue_get_skb(&priv->txq, desc);

		dma_unmap_single(priv->dev, le64_to_cpu(desc->bufptr),
				 le16_to_cpu(desc->xfer_len), DMA_TO_DEVICE);
		dev_kfree_skb_any(skb);
		queue_inc_tail(&priv->txq);
		pr_queue("TX-DONE", &priv->txq);
		++complete;
	}
	spin_unlock_irqrestore(&priv->txlock, flags);

	return complete;
}

/**
 * nemac_rx_packet - Get next packet from the RX queue and re-initialize the
 * descriptor entry with a new buffer.
 */
static int
nemac_rx_packet(struct nemac_priv *priv)
{
	struct dma_desc *desc;
	struct sk_buff *skb;
	u32 ctrl;

	pr_queue("RXQ", &priv->rxq);

	desc = queue_get_tail(&priv->rxq);
	if (!desc)
		return -1;
	queue_inc_tail(&priv->rxq);

	dma_unmap_single(priv->dev, le64_to_cpu(desc->bufptr),
			 priv->rxbuf_sz, DMA_FROM_DEVICE);
	skb = queue_get_skb(&priv->rxq, desc);
	ctrl = cpu_to_le32(desc->ctrl);
	if (ctrl & D_RX_ERR) {
		pr_desc("RX-ERR", desc);
		dev_kfree_skb_any(skb);
	} else if ((ctrl & (D_EOP | D_SOP)) != (D_EOP | D_SOP)) {
		pr_desc("RX-BAD", desc);
		dev_kfree_skb_any(skb);
	} else {
		/* No error, pass sk_buff to upper layer */
		pr_desc("RX", desc);
		skb_put(skb, le32_to_cpu(desc->pdu_len));
		skb->protocol = eth_type_trans(skb, priv->netdev);
		netif_receive_skb(skb);
	}

	/* Add new RX buffers */
	desc = queue_get_head(&priv->rxq);
	if (nemac_alloc_rx_buf(priv, &priv->rxq, desc_idx(desc))) {
		pr_warn_ratelimited("No buffer\n");
		return -1;
	}
	/* Advance RX-HEAD pointer */
	writel(queue_inc_head(&priv->rxq), priv->reg + NEM_DMA_RXHEAD_PTR);
	return 0;
}

/**
 * nemac_poll - NAPI poll function.
 */
static int nemac_poll(struct napi_struct *napi, int budget)
{
	struct nemac_priv *priv = container_of(napi, struct nemac_priv, napi);
	int num_rx = 0;

	writel(INTE_RX_DONE | INTE_TX_DONE, priv->reg + NEM_DMA_INTE_STATUS);

	if (nemac_tx_cleanup(priv) > 0)
		netif_wake_queue(priv->netdev);

	while (num_rx < budget) {
		if (nemac_rx_packet(priv) != 0)
			break;
		++num_rx;
	}

	if (num_rx < budget) {
		/* Exit polling mode and re-enable interrupts */
		napi_complete(napi);
		nemac_enable_dma_int(priv);
	}

	return num_rx;
}

/**
 * nemac_get_stats64 - Retrieve the MAC counters.
 */
static struct rtnl_link_stats64*
nemac_get_stats64(struct net_device *dev, struct rtnl_link_stats64 *s)
{
	struct nemac_priv *priv = netdev_priv(dev);
	const unsigned long tmo = msecs_to_jiffies(20);
	int ret;

	/* Request a snapshot of the counters and wait for the interrupt
	 * handler to signal completion.
	 */
	reinit_completion(&priv->stats_rdy);
	writel(MAC(0), priv->reg + NEM_STATS_SNAPSHOT_R);
	ret = wait_for_completion_interruptible_timeout(&priv->stats_rdy, tmo);
	if (ret <= 0)
		return NULL;

#ifdef NEMAC_DUMP_STAT
	dump_stats(priv);
#endif

	s->rx_packets = nemac_rx_stat_counter(priv, RX_FRM);
	s->tx_packets = nemac_tx_stat_counter(priv, TX_FRM);
	s->rx_bytes = nemac_rx_stat_counter(priv, RX_OCTETS);
	s->tx_bytes = nemac_tx_stat_counter(priv, TX_OCTETS);
	s->multicast = nemac_rx_stat_counter(priv, RX_MCAST);
	s->collisions = (nemac_tx_stat_counter(priv, TX_1COL) +
			       nemac_tx_stat_counter(priv, TX_MCOL));

	/* detailed rx_errors: */
	s->rx_length_errors = nemac_rx_stat_counter(priv, RX_LENGTH);
	s->rx_over_errors = nemac_rx_stat_counter(priv, RX_OSIZE);
	s->rx_crc_errors = nemac_rx_stat_counter(priv, RX_FCS);
	s->rx_frame_errors = nemac_rx_stat_counter(priv, RX_FRM);
	s->rx_fifo_errors = nemac_rx_stat_counter(priv, RX_ORUN);

	/* detailed tx_errors */
	s->tx_aborted_errors = nemac_tx_stat_counter(priv, TX_XSCOL);
	s->tx_fifo_errors = nemac_tx_stat_counter(priv, TX_URUN);

	return s;
}

static const struct net_device_ops nemac_netdev_ops = {
	.ndo_start_xmit   = nemac_xmit,
	.ndo_tx_timeout   = nemac_tx_timeout,
	.ndo_open         = nemac_open,
	.ndo_stop         = nemac_stop,
	.ndo_set_features = nemac_set_features,
	.ndo_set_rx_mode  = nemac_set_rx_mode,
	.ndo_get_stats64  = nemac_get_stats64,
};

static int
nemac_hw_init(const struct nemac_priv *priv)
{
	u32 tmp;

	writel(~0, priv->reg + NEM_SWRESET_R);
	writel(0, priv->reg + NEM_SWRESET_R);

	tmp = readl(priv->reg + NEM_VERSION_R);
	dev_info(priv->dev, "NEMAC HW rev %u.%u\n",
		 MAJOR_REV(tmp), MINOR_REV(tmp));

	writel(ACTIVE_GMAC0, priv->reg + NEM_ACTIVE_R);

	/* Disable and clear interrupts on top level */
	writel(0, priv->reg + NEM_INT_ENABLE_R);
	writel(~0, priv->reg + NEM_INT_STATUS_R);

	/* Initialize GMAC */
	tmp = readl(priv->reg + NEM_GMAC_ENABLE_R);
	tmp |= PORT_ENABLE | GMII_BYPASS_MODE | RGMII_MODE;
	writel(tmp, priv->reg + NEM_GMAC_ENABLE_R);

	/* Initialize STATS */
	writel(STATS_INT_SNAPSHOT_RDY, priv->reg + NEM_STATS_INT_ENABLE_R);
	writel(STATS_MAC_RAW | MAX_LENGTH(1518),
	       priv->reg + NEM_STATS_CONFIG_R);

	return 0;
}

static int
nemac_alloc_dma_ring(struct device *dev, struct queue_ptr *q, u32 num_descr,
		     void __iomem *hw_tail)
{
	q->hw_tail_reg = hw_tail;
	q->head = readl(hw_tail);
	q->head = q->tail;
	q->size = num_descr * sizeof(struct dma_desc);
	q->ring = dma_alloc_coherent(dev, q->size, &q->phys_addr, GFP_KERNEL);
	q->skb = kmalloc_array(num_descr, sizeof(void *), GFP_KERNEL);
	return q->ring == NULL || q->skb == NULL ? -ENOMEM : 0;
}

static void
nemac_free_dma_ring(struct device *dev, struct queue_ptr *q)
{
	dma_free_coherent(dev, q->size, q->ring, q->phys_addr);
}

static int
nemac_setup_descriptors(struct nemac_priv *priv)
{
	struct device *dev = priv->dev;
	int i;
	int ret;

	/* The number of rx and tx descriptors must be an even multiple of
	 * DESCRIPTOR_GRANULARITY.
	 */
	rx_num_desc = ALIGN(rx_num_desc, DESCRIPTOR_GRANULARITY);
	tx_num_desc = ALIGN(tx_num_desc, DESCRIPTOR_GRANULARITY);

	writel(0, priv->reg + NEM_DMA_ENABLE);

	/* Initialize RX ring */
	ret = nemac_alloc_dma_ring(dev, &priv->rxq, rx_num_desc,
				   priv->reg + NEM_DMA_RXTAIL_PTR);
	if (ret == 0) {
		writeq(priv->rxq.phys_addr, priv->reg + NEM_DMA_RXQ_ADDR);
		writel(priv->rxq.size / 1024, priv->reg + NEM_DMA_RXQ_SIZE);
		writel(priv->rxq.head, priv->reg + NEM_DMA_RXHEAD_PTR);
	}

	/* Initialize TX ring */
	ret = nemac_alloc_dma_ring(dev, &priv->txq, tx_num_desc,
				   priv->reg + NEM_DMA_TXTAIL_PTR);
	if (ret == 0) {
		writeq(priv->txq.phys_addr, priv->reg + NEM_DMA_TXQ_ADDR);
		writel(priv->txq.size / 1024, priv->reg + NEM_DMA_TXQ_SIZE);
		writel(priv->txq.head, priv->reg + NEM_DMA_TXHEAD_PTR);
	}

	priv->rxbuf_sz = ALIGN(priv->netdev->mtu + ETH_HLEN + ETH_FCS_LEN, 64);
	pr_debug("rxbuf_sz = %zu\n", priv->rxbuf_sz);

	/* Initialize the descriptors */
	for (i = 0; i < rx_num_desc - 1; ++i) {
		nemac_alloc_rx_buf(priv, &priv->rxq, i);
		writel(queue_inc_head(&priv->rxq),
		       priv->reg + NEM_DMA_RXHEAD_PTR);
	}

	return ret;
}

static void
nemac_stats_interrupt(struct nemac_priv *priv)
{
	u32 status = readl(priv->reg + NEM_STATS_INT_STATUS_R);

	if (status & (1<<0)) {
		pr_debug("stats snapshot complete\n");
		complete(&priv->stats_rdy);
	}
	writel(status, priv->reg + NEM_STATS_INT_STATUS_R);
}

static void
nemac_gmac_interrupt(struct nemac_priv *priv)
{
	u32 status_c37 = readl(priv->reg + NEM_GMAC_ANEG_STATUS_R);

	pr_info("status_c37 = %#x\n", status_c37);
}

static irqreturn_t
nemac_dma_interrupt(int irq, void *_dev)
{
	struct nemac_priv *priv = _dev;
	u32 status = readl(priv->reg + NEM_DMA_INTE_STATUS);

	if (status & (INTE_RX_DONE | INTE_TX_DONE)) {
		if (napi_schedule_prep(&priv->napi)) {
			/* Disable interrupts */
			nemac_disable_dma_int(priv);
			__napi_schedule(&priv->napi);
		} else {
			/* Clear interrupt status */
			writel(INTE_RX_DONE | INTE_TX_DONE,
			       priv->reg + NEM_DMA_INTE_STATUS);
		}
	} else if (status & INTE_ERR_MASK) {
		pr_err("DMA error %#x\n", status);
		writel(status & INTE_ERR_MASK,
		       priv->reg + NEM_DMA_INTE_STATUS);
	} else {
		return IRQ_NONE;
	}

	return IRQ_HANDLED;
}

static irqreturn_t
nemac_mac_interrupt(int irq, void *_dev)
{
	struct nemac_priv *priv = _dev;
	u32 status;

	status = readl(priv->reg + NEM_INT_STATUS_R);
	if (status == 0)
		return IRQ_NONE;
	if (status & INT_GROUP_STATS)
		nemac_stats_interrupt(priv);
	if (status & INT_GROUP_GMAC0)
		nemac_gmac_interrupt(priv);
	writel(status, priv->reg + NEM_INT_STATUS_R);

	return IRQ_HANDLED;
}

static int
nemac_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node * const dn = dev->of_node;
	struct nemac_priv *priv;
	struct net_device *ndev;
	const void *macaddr_dt;
	struct resource *res;
	int ret;

	if (dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(64)) != 0) {
		dev_err(&pdev->dev, "Not 64-bit DMA capable");
		return -EIO;
	}

	ndev = alloc_etherdev(sizeof(*priv));
	if (!ndev)
		return -ENOMEM;

	/* Initialize private members */
	priv = netdev_priv(ndev);
	priv->netdev = ndev;
	priv->dev = &pdev->dev;
	init_completion(&priv->stats_rdy);
	spin_lock_init(&priv->txlock);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	priv->reg = devm_ioremap_resource(dev, res);
	if (IS_ERR(priv->reg)) {
		ret = PTR_ERR(priv->reg);
		goto err;
	}
	pr_debug("regs @ %p\n", priv->reg);

	priv->mac_irq = platform_get_irq(pdev, 0);
	priv->dma_irq = platform_get_irq(pdev, 1);
	if (priv->mac_irq <= 0 || priv->dma_irq <= 0) {
		ret = -EINVAL;
		goto err;
	}

	/* In the case of a fixed PHY, the DT node associated
	 * to the PHY is the Ethernet MAC DT node.
	 */
	if (of_phy_is_fixed_link(dn)) {
		ret = of_phy_register_fixed_link(dn);
		if (ret) {
			dev_err(dev, "failed to register fixed PHY\n");
			goto err;
		}
		priv->phy_dn = of_node_get(dn);
	} else {
		priv->phy_dn = of_parse_phandle(dn, "phy-handle", 0);

		if (!priv->phy_dn) {
			dev_err(dev, "no phy-handle\n");
			ret = -ENODEV;
			goto err;
		}
	}

	/* Initialize Ethernet MAC address
	 *
	 * Read current MAC address (STA) from HW register and use this unless
	 * overridden by device-tree. If neither of these are a valid, use a
	 * randomized address
	 */

	macaddr_dt = of_get_mac_address(dn);
	if (macaddr_dt && is_valid_ether_addr(macaddr_dt)) {
		ether_addr_copy(ndev->dev_addr, macaddr_dt);
		dev_info(dev, "DT address %pM\n", ndev->dev_addr);
	} else {
		nemac_get_sta_address(priv, ndev->dev_addr);
		if (is_valid_ether_addr(ndev->dev_addr)) {
			dev_info(dev, "Current address %pM\n", ndev->dev_addr);
		} else {
			random_ether_addr(ndev->dev_addr);
			dev_info(dev, "Random address %pM\n", ndev->dev_addr);
		}
	}

	ret = nemac_hw_init(priv);
	if (ret)
		goto err;

	ret = nemac_setup_descriptors(priv);
	if (ret)
		goto err;

	SET_NETDEV_DEV(ndev, dev);
	nemac_set_mac_address(priv, priv->netdev->dev_addr,
			      NEM_GMAC_UNICAST1_R, NEM_GMAC_UNICAST12_UPPER_R);
	nemac_set_mac_address(priv, priv->netdev->dev_addr,
			      NEM_GMAC_STA_ADDR_R, NEM_GMAC_STA_ADDR_UPPER_R);
	dev_set_drvdata(dev, ndev);
	ndev->netdev_ops = &nemac_netdev_ops;
	netif_napi_add(ndev, &priv->napi, nemac_poll, 64);

	/* HW supported features, none enabled by default */
	ndev->hw_features |= (NETIF_F_RXCSUM | NETIF_F_HIGHDMA);

	/* libphy will adjust the link state accordingly */
	netif_carrier_off(ndev);

	ret = devm_request_irq(dev, priv->mac_irq, nemac_mac_interrupt,
			       IRQF_SHARED, "nemac-mac", priv);
	if (ret < 0) {
		dev_err(dev, "failed to register irq%d\n", priv->mac_irq);
		goto err;
	}

	ret = devm_request_irq(dev, priv->dma_irq, nemac_dma_interrupt,
			       IRQF_SHARED, "nemac-dma", priv);
	if (ret < 0) {
		dev_err(dev, "failed to register irq%d\n", priv->dma_irq);
		goto err;
	}

	writel(INT_GROUP_GMAC0 | INT_GROUP_STATS, priv->reg + NEM_INT_ENABLE_R);

	ret = register_netdev(ndev);
	if (ret) {
		dev_err(dev, "failed to register net_device\n");
		goto err;
	}

	return 0;
err:
	free_netdev(ndev);
	return ret;
}

static int
nemac_remove(struct platform_device *pdev)
{
	struct net_device *ndev = dev_get_drvdata(&pdev->dev);
	struct nemac_priv *priv = netdev_priv(ndev);

	/* Disable and clear interrupts */
	writel(0, priv->reg + NEM_INT_ENABLE_R);
	writel(~0, priv->reg + NEM_INT_STATUS_R);

	if (priv->rxq.ring)
		nemac_free_dma_ring(&ndev->dev, &priv->rxq);
	if (priv->txq.ring)
		nemac_free_dma_ring(&ndev->dev, &priv->txq);
	unregister_netdev(ndev);
	free_netdev(ndev);
	dev_set_drvdata(&pdev->dev, NULL);

	return 0;
}

static const struct of_device_id nemac_of_match[] = {
	{ .compatible = "intel,nemac" },
	{ }
};

static struct platform_driver nemac_driver = {
	.probe	= nemac_probe,
	.remove	= nemac_remove,
	.driver =  {
		.name = "axxia-nemac",
		.of_match_table = nemac_of_match,
	},
};
module_platform_driver(nemac_driver);

MODULE_AUTHOR("Intel Corporation");
MODULE_DESCRIPTION("Axxia NEMAC Ethernet driver");
MODULE_LICENSE("GPL");
