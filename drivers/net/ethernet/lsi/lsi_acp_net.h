/*
 * drivers/net/ethernet/lsi/lsi_acp_net.h
 *
 * Copyright (C) 2013 LSI Corporation.
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
 */

#ifndef _LSI_ACP_NET_H
#define _LSI_ACP_NET_H

#ifdef CONFIG_MTD_NAND_EP501X_UBOOTENV
extern int ubootenv_get(const char *, char *);
#endif

extern int acp_mdio_read(unsigned long, unsigned long, unsigned short *, int);
extern int acp_mdio_write(unsigned long, unsigned long, unsigned short, int);

/* This is the maximum number of packets to be received every
 * NAPI poll
 */
#define LSINET_NAPI_WEIGHT	64

/* This is the maximum number of bytes that serve to hold
 * incoming Rx data.
 */
#define LSINET_MAX_MTU		(ETH_DATA_LEN + 100) /* MTU + padding */

/* Device Data Structures */

struct appnic_dma_descriptor {
#ifdef __LITTLE_ENDIAN
	/* Word 0 */
	/* 00=Fill|01=Block|10=Scatter */
	unsigned long transfer_type:2;
	unsigned long write:1;
	unsigned long start_of_packet:1;
	unsigned long end_of_packet:1;
	unsigned long interrupt_on_completion:1;
	unsigned long error:1;
	/* big endian to little endian */
	unsigned long byte_swapping_on:1;
	unsigned long unused:24;

	/* Word 1 */
	unsigned long data_transfer_length:16;
	unsigned long pdu_length:16;

	/* Word 2 */
	unsigned long target_memory_address;

	/* Word 3 */
	unsigned long host_data_memory_pointer;
#else
	/* Word 0 */
	unsigned long unused:24;
	/* big endian to little endian */
	unsigned long byte_swapping_on:1;
	unsigned long error:1;
	unsigned long interrupt_on_completion:1;
	unsigned long end_of_packet:1;
	unsigned long start_of_packet:1;
	unsigned long write:1;
	/* 00=Fill|01=Block|10=Scatter */
	unsigned long transfer_type:2;

	/* Word 1 */
	unsigned long pdu_length:16;
	unsigned long data_transfer_length:16;

	/* Word 2 */
	unsigned long target_memory_address;

	/* Word 3 */
	unsigned long host_data_memory_pointer;
#endif

} __packed;

union appnic_queue_pointer {
	unsigned long raw;

	struct {
#ifdef __LITTLE_ENDIAN
		unsigned long offset:20;
		unsigned long generation_bit:1;
		unsigned long unused:11;
#else
		unsigned long unused:11;
		unsigned long generation_bit:1;
		unsigned long offset:20;
#endif
	} __packed bits;

} __packed;

/* The appnic Device Structure */

struct appnic_device {
	/* net_device */
	struct net_device *device;

	/* Addresses, Interrupt, and PHY stuff. */
	void __iomem *rx_base;
	void __iomem *tx_base;
	void __iomem *dma_base;
#ifdef CONFIG_ARM
	void __iomem *gpreg_base;
#endif
	unsigned long tx_interrupt;
	unsigned long rx_interrupt;
	unsigned long dma_interrupt;
	unsigned long mdio_clock;
	unsigned long phy_address;
	unsigned long ad_value;
	unsigned long phy_link_auto;
	unsigned long phy_link_speed;
	unsigned long phy_link_duplex;
	unsigned char mac_addr[6];

	/* NAPI */
	struct napi_struct napi;

	/* Statistics */
	struct net_device_stats stats;
	unsigned long dropped_by_stack;
	unsigned long out_of_tx_descriptors;
	unsigned long transmit_interrupts;
	unsigned long receive_interrupts;

	/* DMA-able memory */
	int dma_alloc_size;
	int dma_alloc_size_rx;
	int dma_alloc_size_tx;
	void *dma_alloc;
	void *dma_alloc_rx;
	void *dma_alloc_tx;
	dma_addr_t dma_alloc_dma;
	dma_addr_t dma_alloc_dma_rx;
	dma_addr_t dma_alloc_dma_tx;
	int dma_alloc_offset;
	int dma_alloc_offset_rx;
	int dma_alloc_offset_tx;

	/* tail pointers */
	volatile union appnic_queue_pointer *rx_tail;
	dma_addr_t rx_tail_dma;
	volatile union appnic_queue_pointer *tx_tail;
	dma_addr_t tx_tail_dma;

	/* descriptors */
	struct appnic_dma_descriptor *rx_desc;
	dma_addr_t rx_desc_dma;
	unsigned rx_num_desc;
	struct appnic_dma_descriptor *tx_desc;
	dma_addr_t tx_desc_dma;
	unsigned tx_num_desc;

	/* buffers */
	unsigned rx_buf_sz;
	unsigned rx_buf_per_desc;
	void *rx_buf;
	dma_addr_t rx_buf_dma;
	unsigned tx_buf_sz;
	unsigned tx_buf_per_desc;
	void *tx_buf;
	dma_addr_t tx_buf_dma;

	/* The local pointers */
	union appnic_queue_pointer rx_tail_copy;
	union appnic_queue_pointer rx_head;
	union appnic_queue_pointer tx_tail_copy;
	union appnic_queue_pointer tx_head;

	/* Spin Lock */
	spinlock_t dev_lock;

	/* PHY */
	struct mii_bus *mii_bus;
	struct phy_device *phy_dev;
	int phy_irq[PHY_MAX_ADDR];
	unsigned int link;
	unsigned int speed;
	unsigned int duplex;
};

/* GPREG FEMAC HPROT Register --------------------------------------- */

#define GPREG_HPROT_FEMAC	((unsigned long)pdata->gpreg_base + 0x78)

/* Overview
 * --------
 *
 * Register offset decoding is as follows:
 *
 * Bit(s) Description
 *
 * 16:15  define the Channel.  There is only one; therefore, 00.
 * 14:12  define the MAC within the channel.  Only one so 000.
 * 11:10  define the register "space" as follows:
 * 00 = fast ethernet MACmw.l 06000000 ffffffff 3200000
 * 10 = global
 * 11 = interrupt
 * 9: 2  register
 * 1: 0  always 00, 32 bit registers only.
 *
 * Receive registers start at the base address.  Transmit registers start
 * at 0x20000 above the base address.  DMA start at a completely different
 * base address (in this case 0x8000000 above the base).
 *
*/

/* Receive Configuration -------------------------------------------- */

#define APPNIC_RX_CONF		((unsigned long)pdata->rx_base + 0x004c)
#define APPNIC_RX_CONF_ENABLE   0x0001
/* Pass Any Packet */
#define APPNIC_RX_CONF_PAP	0x0002
#define APPNIC_RX_CONF_JUMBO9K  0x0008
#define APPNIC_RX_CONF_STRIPCRC 0x0010
/* Accept All MAC Types */
#define APPNIC_RX_CONF_AMT	0x0020
/* Accept Flow Control */
#define APPNIC_RX_CONF_AFC	0x0040
/* Enable VLAN */
#define APPNIC_RX_CONF_VLAN	0x0200
/* RX MAC Speed, 1=100MBS */
#define APPNIC_RX_CONF_SPEED    0x0800
/* 1=Duplex Mode */
#define APPNIC_RX_CONF_DUPLEX   0x1000
/* 1=Enable */
#define APPNIC_RX_CONF_LINK	0x2000
/* Determines the action taken when the FE MAC
 * receives an Flow Control packet in FD mode.
 */
#define APPNIC_RX_CONF_RXFCE    0x4000

/* Controls the insertion of Flow Control packets
 * by the MAC transmitter.
 */
#define APPNIC_RX_CONF_TXFCE    0x8000

/* Receive Stat Overflow -------------------------------------------- */

#define APPNIC_RX_STAT_OVERFLOW ((unsigned long)pdata->rx_base + 0x278)

/* Receive Stat Undersize ------------------------------------------- */

#define APPNIC_RX_STAT_UNDERSIZE ((unsigned long)pdata->rx_base + 0x280)

/* Receive Stat Oversize -------------------------------------------- */

#define APPNIC_RX_STAT_OVERSIZE ((unsigned long)pdata->rx_base + 0x2b8)

/* Receive Stat Multicast ------------------------------------------- */

#define APPNIC_RX_STAT_MULTICAST ((unsigned long)pdata->rx_base + 0x2d0)

/* Receive Stat Packet OK ------------------------------------------- */

#define APPNIC_RX_STAT_PACKET_OK ((unsigned long)pdata->rx_base + 0x2c0)

/* Receive Stat CRC Error ------------------------------------------- */

#define APPNIC_RX_STAT_CRC_ERROR ((unsigned long)pdata->rx_base + 0x2c8)

/* Receive Stat Align Error ----------------------------------------- */

#define APPNIC_RX_STAT_ALIGN_ERROR ((unsigned long)pdata->rx_base + 0x2e8)

/* Receive Ethernet Mode -------------------------------------------- */

#define APPNIC_RX_MODE ((unsigned long)pdata->rx_base + 0x0800)
#define APPNIC_RX_MODE_ETHERNET_MODE_ENABLE 0x00001

/* Receive Soft Reset ----------------------------------------------- */

#define APPNIC_RX_SOFT_RESET ((unsigned long)pdata->rx_base + 0x0808)
#define APPNIC_RX_SOFT_RESET_MAC_0 0x00001

/* Receive Internal Interrupt Control ------------------------------- */

#define APPNIC_RX_INTERNAL_INTERRUPT_CONTROL \
	((unsigned long)pdata->rx_base + 0xc00)
#define APPNIC_RX_INTERNAL_INTERRUPT_CONTROL_MAC_0 0x1

/* Receive External Interrupt Control ------------------------------- */

#define APPNIC_RX_EXTERNAL_INTERRUPT_CONTROL \
	((unsigned long)pdata->rx_base + 0xc04)
#define APPNIC_RX_EXTERNAL_INTERRUPT_CONTROL_MAC_0_HIGH_LOW 0x10
#define APPNIC_RX_EXTERNAL_INTERRUPT_CONTROL_MAC_0 0x1

/* Receive Interrupt Status ----------------------------------------- */

#define APPNIC_RX_INTERRUPT_STATUS ((unsigned long)pdata->rx_base + 0xc20)
#define APPNIC_RX_INTERRUPT_EXTERNAL_STATUS_MAC_0 0x10
#define APPNIC_RX_INTERRUPT_INTERNAL_STATUS_MAC_0 0x1

/* Transmit Watermark ----------------------------------------------- */

#define APPNIC_TX_WATERMARK ((unsigned long)pdata->tx_base + 0x18)
#define APPNIC_TX_WATERMARK_TXCONFIG_DTPA_ASSERT 0x8000
#define APPNIC_TX_WATERMARK_TXCONFIG_DTPA_DISABLE 0x4000
#define APPNIC_TX_WATERMARK_TXCONFIG_DTPA_WATER_MARK_HIGH 0x3f00
#define APPNIC_TX_WATERMARK_TXCONFIG_DTPA_WATER_MARK_LOW 0x3f

/* Swap Source Address Registers ------------------------------------ */

#define APPNIC_SWAP_SOURCE_ADDRESS_2 ((unsigned long)pdata->tx_base + 0x20)
#define APPNIC_SWAP_SOURCE_ADDRESS_1 ((unsigned long)pdata->tx_base + 0x24)
#define APPNIC_SWAP_SOURCE_ADDRESS_0 ((unsigned long)pdata->tx_base + 0x28)

/* Transmit Extended Configuration ---------------------------------- */

#define APPNIC_TX_EXTENDED_CONF ((unsigned long)pdata->tx_base + 0x30)
#define APPNIC_TX_EXTENDED_CONF_TRANSMIT_COLLISION_WATERMARK_LEVEL 0xf000
#define APPNIC_TX_EXTENDED_CONF_EXCESSIVE_DEFFERED_PACKET_DROP 0x200
#define APPNIC_TX_EXTENDED_CONF_JUMBO9K 0x100
#define APPNIC_TX_EXTENDED_CONF_LATE_COLLISION_WINDOW_COUNT 0xff

/* Transmit Half Duplex Configuration ------------------------------- */

#define APPNIC_TX_HALF_DUPLEX_CONF ((unsigned long)pdata->tx_base + 0x34)
#define APPNIC_TX_HALF_DUPLEX_CONF_RANDOM_SEED_VALUE 0xff

/* Transmit Configuration ------------------------------------------- */

#define APPNIC_TX_CONF	((unsigned long)pdata->tx_base + 0x0050)
#define APPNIC_TX_CONF_ENABLE_SWAP_SA	0x8000
#define APPNIC_TX_CONF_LINK		0x2000
#define APPNIC_TX_CONF_DUPLEX		0x1000
#define APPNIC_TX_CONF_SPEED		0x0800
#define APPNIC_TX_CONF_XBK_RST_RX_NTX	0x0600
#define APPNIC_TX_CONF_IFG		0x01f0
#define APPNIC_TX_CONF_APP_CRC_ENABLE	0x0004
#define APPNIC_TX_CONF_PAD_ENABLE	0x0002
#define APPNIC_TX_CONF_ENABLE		0x0001

#define TX_CONF_SET_IFG(tx_configuration, ifg)			\
	do {							\
		(tx_configuration) &= ~APPNIC_TX_CONF_IFG;	\
		(tx_configuration) |= ((ifg & 0x1f) << 4);	\
	} while (0)

/* Transmit Time Value Configuration -------------------------------- */

#define APPNIC_TX_TIME_VALUE_CONF ((unsigned long)pdata->tx_base + 0x5c)
#define APPNIC_TX_TIME_VALUE_CONF_PAUSE_VALUE 0xffff

/* Transmit Stat Underrun ------------------------------------------- */

#define APPNIC_TX_STAT_UNDERRUN ((unsigned long)pdata->tx_base + 0x300)

/* Transmit Stat Packet OK ------------------------------------------ */

#define APPNIC_TX_STAT_PACKET_OK ((unsigned long)pdata->tx_base + 0x318)

/* Transmit Stat Undersize ------------------------------------------ */

#define APPNIC_TX_STAT_UNDERSIZE ((unsigned long)pdata->tx_base + 0x350)

/* Transmit Status Late Collision ----------------------------------- */

#define APPNIC_TX_STATUS_LATE_COLLISION ((unsigned long)pdata->tx_base + 0x368)

/* Transmit Status Excessive Collision ------------------------------ */

#define APPNIC_TX_STATUS_EXCESSIVE_COLLISION \
	((unsigned long)pdata->tx_base + 0x370)

/* Transmit Stat Collision Above Watermark -------------------------- */

#define APPNIC_TX_STAT_COLLISION_ABOVE_WATERMARK \
	((unsigned long)pdata->tx_base + 0x380)

/* Transmit Mode ---------------------------------------------------- */

#define APPNIC_TX_MODE ((unsigned long)pdata->tx_base + 0x800)
#define APPNIC_TX_MODE_ETHERNET_MODE_ENABLE 0x1

/* Transmit Soft Reset ---------------------------------------------- */

#define APPNIC_TX_SOFT_RESET ((unsigned long)pdata->tx_base + 0x808)
#define APPNIC_TX_SOFT_RESET_MAC_0 0x1

/* Transmit Interrupt Control --------------------------------------- */

#define APPNIC_TX_INTERRUPT_CONTROL ((unsigned long)pdata->tx_base + 0xc00)
#define APPNIC_TX_INTERRUPT_CONTROL_MAC_0 0x1

/* Transmit Interrupt Status ---------------------------------------- */

#define APPNIC_TX_INTERRUPT_STATUS ((unsigned long)pdata->tx_base + 0xc20)
#define APPNIC_TX_INTERRUPT_STATUS_MAC_0 0x1

/* */

#define APPNIC_DMA_PCI_CONTROL ((unsigned long)pdata->dma_base + 0x00)

/* */

#define APPNIC_DMA_CONTROL ((unsigned long)pdata->dma_base + 0x08)

/* DMA Interrupt Status --------------------------------------------- */

#define APPNIC_DMA_INTERRUPT_STATUS ((unsigned long)pdata->dma_base + 0x18)
#define APPNIC_DMA_INTERRUPT_STATUS_RX 0x2
#define APPNIC_DMA_INTERRUPT_STATUS_TX 0x1
#define RX_INTERRUPT(dma_interrupt_status) \
	(0 != (dma_interrupt_status & APPNIC_DMA_INTERRUPT_STATUS_RX))
#define TX_INTERRUPT(dma_interrupt_status) \
	(0 != (dma_interrupt_status & APPNIC_DMA_INTERRUPT_STATUS_TX))

/* DMA Interrupt Enable --------------------------------------------- */

#define APPNIC_DMA_INTERRUPT_ENABLE ((unsigned long)pdata->dma_base + 0x1c)
#define APPNIC_DMA_INTERRUPT_ENABLE_RECEIVE 0x2
#define APPNIC_DMA_INTERRUPT_ENABLE_TRANSMIT 0x1

/* DMA Receive Queue Base Address ----------------------------------- */

#define APPNIC_DMA_RX_QUEUE_BASE_ADDRESS ((unsigned long)pdata->dma_base + 0x30)

/* DMA Receive Queue Size ------------------------------------------- */

#define APPNIC_DMA_RX_QUEUE_SIZE ((unsigned long)pdata->dma_base + 0x34)

/* DMA Transmit Queue Base Address ---------------------------------- */

#define APPNIC_DMA_TX_QUEUE_BASE_ADDRESS ((unsigned long)pdata->dma_base + 0x38)

/* DMA Transmit Queue Size ------------------------------------------ */

#define APPNIC_DMA_TX_QUEUE_SIZE ((unsigned long)pdata->dma_base + 0x3c)

/* DMA Recevie Tail Pointer Address --------------------------------- */

#define APPNIC_DMA_RX_TAIL_POINTER_ADDRESS \
	((unsigned long)pdata->dma_base + 0x48)

/* DMA Transmit Tail Pointer Address -------------------------------- */

#define APPNIC_DMA_TX_TAIL_POINTER_ADDRESS \
	((unsigned long)pdata->dma_base + 0x4c)

/* DMA Receive Head Pointer ----------------------------------------- */

#define APPNIC_DMA_RX_HEAD_POINTER	((unsigned long)pdata->dma_base + 0x50)
#define APPNIC_DMA_RX_HEAD_POINTER_GB			0x100000
#define APPNIC_DMA_RX_HEAD_POINTER_POINTER		0x0fffff

/* DMA Receive Tail Pointer Local Copy ------------------------------ */

#define APPNIC_DMA_RX_TAIL_POINTER_LOCAL_COPY	\
	((unsigned long)pdata->dma_base + 0x54)
#define APPNIC_DMA_RX_TAIL_POINTER_LOCAL_COPY_GB	0x100000
#define APPNIC_DMA_RX_TAIL_POINTER_LOCAL_COPY_POINTER	0x0fffff

/* DMA Transmit Head Pointer ---------------------------------------- */

#define APPNIC_DMA_TX_HEAD_POINTER	((unsigned long)pdata->dma_base + 0x58)
#define APPNIC_DMA_TX_HEAD_POINTER_GB			0x100000
#define APPNIC_DMA_TX_HEAD_POINTER_POINTER		0x0fffff

/* DMA Transmit Tail Pointer Local Copy ----------------------------- */

#define APPNIC_DMA_TX_TAIL_POINTER_LOCAL_COPY	\
	((unsigned long)pdata->dma_base + 0x5c)
#define APPNIC_DMA_TX_TAIL_POINTER_LOCAL_COPY_GB	0x100000
#define APPNIC_DMA_TX_TAIL_POINTER_LOCAL_COPY_POINTER	0x0fffff

#ifdef CONFIG_ARM

#define read_mac(address)         readl((void __iomem *)(address))
#define write_mac(value, address) writel((value), (void __iomem *)(address))
#else
#define read_mac(address)         in_le32((u32 *) (address))
#define write_mac(value, address) out_le32((u32 *) (address), (value))
#endif /* CONFIG_ARM */

#ifdef __LITTLE_ENDIAN
static inline void
readdescriptor(unsigned long address, struct appnic_dma_descriptor *descriptor)
{
	memcpy(descriptor, (void *)address,
	       sizeof(struct appnic_dma_descriptor));
}

static inline void
writedescriptor(unsigned long address,
		const struct appnic_dma_descriptor *descriptor)
{
	memcpy((void *)address, descriptor, sizeof(struct appnic_dma_descriptor));
}

static inline union appnic_queue_pointer
_swab_queue_pointer(const union appnic_queue_pointer *old_queue)
{
	return *old_queue;
}

static inline void
femac_uncache(struct appnic_device *pdata)
{
	/* Set FEMAC to uncached */
	writel(0x0, (void __iomem *)GPREG_HPROT_FEMAC);
}

#else

static inline void
readdescriptor(unsigned long address, struct appnic_dma_descriptor *descriptor)
{
	unsigned long *from = (unsigned long *)address;
	unsigned long *to = (unsigned long *)descriptor;

	*to++ = swab32(*from++);
	*to++ = swab32(*from++);
	*to++ = swab32(*from++);
	*to++ = swab32(*from++);
}

static inline void
writedescriptor(unsigned long address,
		const struct appnic_dma_descriptor *descriptor)
{
	unsigned long *to = (unsigned long *)address;
	unsigned long *from = (unsigned long *)descriptor;

	*to++ = swab32(*from++);
	*to++ = swab32(*from++);
	*to++ = swab32(*from++);
	*to++ = swab32(*from++);
}

static inline union appnic_queue_pointer
_swab_queue_pointer(const union appnic_queue_pointer *old_queue)
{
	union appnic_queue_pointer new_queue;

	new_queue.raw = swab32(old_queue->raw);
	return new_queue;
}

static inline void
femac_uncache(struct appnic_device *pdata) {}

#endif /* ifdef __LITTLE_ENDIAN */

static int
femac_irq_setup(struct net_device *dev)
{
	struct appnic_device *pdata = netdev_priv(dev);

#ifdef CONFIG_ARM
	dev->irq = pdata->dma_interrupt;
#else
	dev->irq = irq_create_mapping(NULL, pdata->dma_interrupt);
	if (NO_IRQ == dev->irq)
		return -EINVAL;

	if (0 != irq_set_irq_type(dev->irq, IRQ_TYPE_LEVEL_HIGH))
		return -EINVAL;
#endif
	return 0;
}

static inline int
femac_alloc_mem_buffers(struct net_device *dev)
{
	struct appnic_device *pdata = netdev_priv(dev);
	struct device *device = NULL;
	int rc;

#ifndef CONFIG_ARM
	dev->dev.archdata.dma_ops = &dma_direct_ops;
	device = &dev->dev;
#endif

	pdata->dma_alloc = (void *)dma_alloc_coherent(device,
						      pdata->dma_alloc_size,
						      &pdata->dma_alloc_dma,
						      GFP_KERNEL);
	if (pdata->dma_alloc == (void *)0) {
		rc = -ENOMEM;
		goto err_dma_alloc;
	}

	pdata->dma_alloc_offset = (int)pdata->dma_alloc -
					(int)pdata->dma_alloc_dma;

	pdata->dma_alloc_rx = (void *)dma_alloc_coherent(device,
						      pdata->dma_alloc_size_rx,
						      &pdata->dma_alloc_dma_rx,
						      GFP_KERNEL);
	if (pdata->dma_alloc_rx == (void *)0) {
		rc = -ENOMEM;
		goto err_dma_alloc_rx;
	}

	pdata->dma_alloc_offset_rx = (int)pdata->dma_alloc_rx -
					(int)pdata->dma_alloc_dma_rx;

	pdata->dma_alloc_tx = (void *)dma_alloc_coherent(device,
						      pdata->dma_alloc_size_tx,
						      &pdata->dma_alloc_dma_tx,
						      GFP_KERNEL);

	if (pdata->dma_alloc_tx == (void *)0) {
		rc = -ENOMEM;
		goto err_dma_alloc_tx;
	}

	pdata->dma_alloc_offset_tx = (int)pdata->dma_alloc_tx -
					(int)pdata->dma_alloc_dma_tx;

	return 0;

err_dma_alloc_tx:
	dma_free_coherent(device, pdata->dma_alloc_size_rx,
			  pdata->dma_alloc_rx, pdata->dma_alloc_dma_rx);
err_dma_alloc_rx:
	dma_free_coherent(device, pdata->dma_alloc_size,
			  pdata->dma_alloc, pdata->dma_alloc_dma);
err_dma_alloc:
	return rc;
}

static inline void
femac_free_mem_buffers(struct net_device *dev)
{
	struct appnic_device *pdata = netdev_priv(dev);
	struct device *device = NULL;

#ifndef CONFIG_ARM
	device = &dev->dev;
#endif
	dma_free_coherent(device, pdata->dma_alloc_size,
			  pdata->dma_alloc, pdata->dma_alloc_dma);
	dma_free_coherent(device, pdata->dma_alloc_size_rx,
			  pdata->dma_alloc_rx, pdata->dma_alloc_dma_rx);
	dma_free_coherent(device, pdata->dma_alloc_size_tx,
			  pdata->dma_alloc_tx, pdata->dma_alloc_dma_tx);
}

#define swab_queue_pointer(pointer) \
	_swab_queue_pointer((const union appnic_queue_pointer *) (pointer))

#endif /* _LSI_ACP_NET_H */
