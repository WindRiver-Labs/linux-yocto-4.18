/*
 * drivers/net/ethernet/lsi/lsi_acp_net.c
 *
 * Copyright (C) 2013 LSI Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * NOTES:
 *
 * 1) This driver is used by both ACP (PPC) and AXM (ARM) platforms.
 *
 * 2) This driver parses the DTB for driver specific settings. A few of
 *    them can be overridden by setting environment variables in U-boot:
 *
 *    ethaddr - MAC address of interface, in xx:xx:xx:xx:xx:xx format
 *
 *    phy-addr - Specific address of PHY (0 - 0x20). If not specified,
 *               the driver will scan the bus and will attach to the first
 *               PHY it finds.
 *
 *    ad-value - PHY advertise value. Can be set to one of these or they
 *               be OR'ed together. If not set, the driver sets the
 *               advertised value equal to what the driver supports.
 *
 *               0x101 - 100/Full
 *               0x81  - 100/Half
 *               0x41  - 10/Full
 *               0x21  - 10/Half
 *
 * 3) This driver allows the option to disable auto negotiation and manually
 *    specify the speed and duplex setting, with the use of the device tree
 *    variable "phy-link". Legal values for this variable are:
 *
 *    "auto"  - auto negotiation enabled
 *    "100MF" - auto negotiation disabled, set to 100MB Full Duplex
 *    "10MH"  - auto negotiation disabled, set to 100MB Half Duplex
 *    "10MF"  - auto negotiation disabled, set to 10MB Full Duplex
 *    "10MH"  - auto negotiation disabled, set to 10MB Half Duplex
 *
 *    NOTE: If the phy-link variable is not present in the device tree, or
 *    if an invalid value is used, the driver defaults to auto negotiation
 *    mode.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/string.h>
#include <linux/interrupt.h>
#include <linux/errno.h>
#include <linux/in.h>
#include <linux/slab.h>
#include <linux/ioport.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/bitops.h>
#include <linux/proc_fs.h>
#include <linux/platform_device.h>
#include <linux/netdevice.h>
#include <linux/mii.h>
#include <linux/phy.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/skbuff.h>
#include <linux/init.h>
#include <linux/irq.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_net.h>
#include <linux/dma-mapping.h>
#include <linux/uaccess.h>
#include <linux/io.h>

#include <asm/dma.h>

#include "../../../misc/lsi-ncr.h"

#include "lsi_acp_net.h"

#define LSI_DRV_NAME           "acp-femac"
#define LSI_MDIO_NAME          "acp-femac-mdio"
#define LSI_DRV_VERSION        "2014-01-09"

MODULE_AUTHOR("John Jacques");
MODULE_DESCRIPTION("LSI ACP-FEMAC Ethernet driver");
MODULE_LICENSE("GPL");

/* ----------------------------------------------------------------------
 * appnic_mii_read
 *
 * Returns -EBUSY if unsuccessful, the (short) value otherwise.
 */

static int appnic_mii_read(struct mii_bus *bus, int phy, int reg)
{
	unsigned short value;

	/* Always returns success, so no need to check return status. */
	acp_mdio_read(phy, reg, &value, 0);

	return (int)value;
}

/* ----------------------------------------------------------------------
 * appnic_mii_write
 */

static int appnic_mii_write(struct mii_bus *bus, int phy, int reg, u16 val)
{
	return acp_mdio_write(phy, reg, val, 0);
}

/* ----------------------------------------------------------------------
 * appnic_handle_link_change
 *
 * Called periodically when PHY is in polling mode.
 */

static void appnic_handle_link_change(struct net_device *dev)
{
	struct appnic_device *pdata = netdev_priv(dev);
	struct phy_device *phydev = pdata->phy_dev;
	int status_change = 0;
	unsigned long rx_configuration;
	unsigned long tx_configuration = 0;

	rx_configuration =
#ifdef CONFIG_ARM
		APPNIC_RX_CONF_STRIPCRC;
#else
		(APPNIC_RX_CONF_STRIPCRC |
		 APPNIC_RX_CONF_RXFCE |
		 APPNIC_RX_CONF_TXFCE);
#endif
	tx_configuration =
		(APPNIC_TX_CONF_ENABLE_SWAP_SA |
		 APPNIC_TX_CONF_APP_CRC_ENABLE |
		 APPNIC_TX_CONF_PAD_ENABLE);

	TX_CONF_SET_IFG(tx_configuration, 0xf);

	if (phydev->link) {
		if ((pdata->speed != phydev->speed) ||
		    (pdata->duplex != phydev->duplex)) {
			if (phydev->duplex) {
				rx_configuration |= APPNIC_RX_CONF_DUPLEX;
				tx_configuration |= APPNIC_TX_CONF_DUPLEX;
			}
			if (phydev->speed == SPEED_100) {
				rx_configuration |= APPNIC_RX_CONF_SPEED;
				tx_configuration |= APPNIC_TX_CONF_SPEED;
			}

			rx_configuration |= (APPNIC_RX_CONF_ENABLE |
					     APPNIC_RX_CONF_LINK);
			tx_configuration |= (APPNIC_TX_CONF_LINK |
					     APPNIC_TX_CONF_ENABLE);

			pdata->speed = phydev->speed;
			pdata->duplex = phydev->duplex;
			status_change = 1;
		}
	}
	if (phydev->link != pdata->link) {
		if (!phydev->link) {
			pdata->speed = 0;
			pdata->duplex = -1;
		}
		pdata->link = phydev->link;
		status_change = 1;
	}

	if (status_change) {
		if (phydev->link) {
			netif_carrier_on(dev);
			netdev_info(dev, "link up (%d/%s)\n",
				    phydev->speed,
				    phydev->duplex == DUPLEX_FULL ?
				    "Full" : "Half");
		} else {
			netif_carrier_off(dev);
			netdev_info(dev, "link down\n");
		}

		if (rx_configuration != read_mac(APPNIC_RX_CONF))
			write_mac(rx_configuration, APPNIC_RX_CONF);

		if (tx_configuration != read_mac(APPNIC_TX_CONF))
			write_mac(tx_configuration, APPNIC_TX_CONF);
	}
}

/* ----------------------------------------------------------------------
 * appnic_mii_probe
 */

static int appnic_mii_probe(struct net_device *dev)
{
	struct appnic_device *pdata = netdev_priv(dev);
	struct phy_device *phydev = NULL;
	int ret;

	if (pdata->phy_address && (pdata->phy_address < PHY_MAX_ADDR)) {
		phydev = pdata->mii_bus->phy_map[pdata->phy_address];
		if (phydev)
			goto skip_first;
	}

	/* Find the first phy */
	phydev = phy_find_first(pdata->mii_bus);
	if (!phydev) {
		pr_crit("!!! no PHY found !!!\n");
		netdev_err(dev, " no PHY found\n");
		return -ENODEV;
	}

skip_first:

	/* Allow the option to disable auto negotiation and manually specify
	 * the link speed and duplex setting with the use of a environment
	 * setting.
	 */

	if (0 == pdata->phy_link_auto) {
		phydev->autoneg = AUTONEG_DISABLE;
		phydev->speed =
			0 == pdata->phy_link_speed ? SPEED_10 : SPEED_100;
		phydev->duplex =
			0 == pdata->phy_link_duplex ? DUPLEX_HALF : DUPLEX_FULL;
	} else {
		phydev->autoneg = AUTONEG_ENABLE;
	}

	ret = phy_connect_direct(dev, phydev,
				 &appnic_handle_link_change,
				 PHY_INTERFACE_MODE_MII);

	if (ret) {
		netdev_err(dev, "Could not attach to PHY\n");
		return ret;
	}

	netdev_info(dev,
		    "attached PHY driver [%s] (mii_bus:phy_addr=%s, irq=%d)\n",
		    phydev->drv->name, dev_name(&phydev->dev), phydev->irq);

	/* Mask with MAC supported features */
	phydev->supported &= PHY_BASIC_FEATURES;
	if (pdata->ad_value)
		phydev->advertising = mii_adv_to_ethtool_adv_t(pdata->ad_value);
	else
		phydev->advertising = phydev->supported;

	pdata->link = 0;
	pdata->speed = 0;
	pdata->duplex = -1;
	pdata->phy_dev = phydev;

	pr_info("%s: PHY initialized successfully", LSI_DRV_NAME);
	return 0;
}

/* ----------------------------------------------------------------------
 * appnic_mii_init
 */

static int appnic_mii_init(struct platform_device *pdev,
			   struct net_device *dev)
{
	struct appnic_device *pdata = netdev_priv(dev);
	int i, err = -ENXIO;

	pdata->mii_bus = mdiobus_alloc();
	if (!pdata->mii_bus) {
		err = -ENOMEM;
		goto err_out_1;
	}

	pdata->mii_bus->name = LSI_MDIO_NAME;
	snprintf(pdata->mii_bus->id, MII_BUS_ID_SIZE, "%s-%x",
		 pdev->name, pdev->id);
	pdata->mii_bus->priv = pdata;
	pdata->mii_bus->read = appnic_mii_read;
	pdata->mii_bus->write = appnic_mii_write;
	pdata->mii_bus->irq = pdata->phy_irq;
	for (i = 0; i < PHY_MAX_ADDR; ++i)
		pdata->mii_bus->irq[i] = PHY_POLL;

	if (mdiobus_register(pdata->mii_bus)) {
		pr_warn("%s: Error registering mii bus", LSI_DRV_NAME);
		goto err_out_free_bus_2;
	}

	if (appnic_mii_probe(dev) < 0) {
		pr_warn("%s: Error registering mii bus", LSI_DRV_NAME);
		goto err_out_unregister_bus_3;
	}

	return 0;

err_out_unregister_bus_3:
	mdiobus_unregister(pdata->mii_bus);
err_out_free_bus_2:
	mdiobus_free(pdata->mii_bus);
err_out_1:
	return err;
}

/* ======================================================================
 * NIC Interface
 * ======================================================================
*/

#define DESCRIPTOR_GRANULARITY 64
#define BUFFER_ALIGNMENT 64

#define ALIGN64B(address) (PTR_ALIGN((address), BUFFER_ALIGNMENT))
#define ALIGN64B_OFFSET(address) \
	(ALIGN64B(address) - (unsigned long) (address))

/*  ----- Note On Buffer Space -----
 *
 *  Minimum number of descriptors is 64 for the receiver and 64 for the
 *  transmitter; therefore, 2048 bytes (16 bytes each).
 *  This driver uses the following parameters, all of which may be set on
 *  the command line if this drivers is used as a module.
 *
 *  - rx_num_desc : Number of receive descriptors. This  must be a multiple
 *                  of 64.
 *  - tx_num_desc : Number of transmit descriptors. This must be a multiple
 *                  of 64.
 *
 *  The scheme used will be as follows:
 *
 *  - num_[rt]x_desc will be adjusted to be a multiple of 64 (if necessary).
 *  - An skb (with the data area 64 byte aligned) will be allocated for each rx
 *    descriptor.
 */

/* Receiver */

int rx_num_desc = (CONFIG_LSI_NET_NUM_RX_DESC * DESCRIPTOR_GRANULARITY);
module_param(rx_num_desc, int, 0000);
MODULE_PARM_DESC(rx_num_desc, "appnic : Number of receive descriptors");

int rx_buf_sz = CONFIG_LSI_NET_RX_BUF_SZ;
module_param(rx_buf_sz, int, 0000);
MODULE_PARM_DESC(rx_buf_sz, "appnic : Receive buffer size");

/* Transmitter */

int tx_num_desc = (CONFIG_LSI_NET_NUM_TX_DESC * DESCRIPTOR_GRANULARITY);
module_param(tx_num_desc, int, 0000);
MODULE_PARM_DESC(tx_num_desc, "appnic : Number of receive descriptors");

int tx_buf_sz = CONFIG_LSI_NET_TX_BUF_SZ;
module_param(tx_buf_sz, int, 0000);
MODULE_PARM_DESC(tx_buf_sz, "Appnic : Receive buffer size");

/* ======================================================================
 * Utility Functions
 * ======================================================================
 */

/* ----------------------------------------------------------------------
 * mac_addr_valid
 *
 * If mac address is multicast, broadcast, or matches our mac address,
 * it's a valid address. Otherwise, it's not.
 */

static bool mac_addr_valid(struct net_device *dev, u8 *mac_addr)
{
	bool is_valid = false;

	if (is_multicast_ether_addr(mac_addr))
		is_valid = true;
	else if (is_broadcast_ether_addr(mac_addr))
		is_valid = true;
	else if (ether_addr_equal(mac_addr, &dev->dev_addr[0]))
		is_valid = true;

	return is_valid;
}

/* ----------------------------------------------------------------------
 * clear_statistics
 *
 * NOTE: The hardware clears the statistics registers after a read.
 */

static void clear_statistics(struct appnic_device *pdata)
{
	/* Clear memory. */

	memset((void *)&pdata->stats, 0, sizeof(struct net_device_stats));

	/* Clear counters by reading them. */

	/* stats.rx_packets */
	read_mac(APPNIC_RX_STAT_PACKET_OK);

	/* stats.tx_packets */
	read_mac(APPNIC_TX_STAT_PACKET_OK);

	/* stats.rx_bytes - Updated by this driver.
	 * stats.tx_bytes - Updated by this driver.
	 * stats.rx_errors - The sum of all RX errors available.
	 * stats.tx_errors - The sum of all TX errors available.
	 * stats.rx_dropped (unable to allocate skb) - Updated by the stack.
	 * stats.tx_dropped (unable to allocate skb) - Updated by the stack.
	 */

	/* stats.multicast */
	read_mac(APPNIC_RX_STAT_MULTICAST);

	/* stats.collisions - The sum of the following driver stats. */
	read_mac(APPNIC_TX_STATUS_LATE_COLLISION);
	read_mac(APPNIC_TX_STATUS_EXCESSIVE_COLLISION);
	read_mac(APPNIC_TX_STAT_COLLISION_ABOVE_WATERMARK);

	/* stats.rx_length_errors - The sum of the following driver stats. */
	read_mac(APPNIC_RX_STAT_UNDERSIZE);
	read_mac(APPNIC_RX_STAT_OVERSIZE);

	/* stats.rx_over_errors - Not maintained by this driver. */

	/* stats.rx_crc_errors */
	read_mac(APPNIC_RX_STAT_CRC_ERROR);

	/* stats.rx_frame_errors */
	read_mac(APPNIC_RX_STAT_ALIGN_ERROR);

	/* stats.rx_fifo_errors */
	read_mac(APPNIC_RX_STAT_OVERFLOW);

	/* stats.rx_missed - Not maintained by this driver.
	 * stats.tx_aborted_errors - Not maintained by this driver.
	 * stats.tx_carrier_errors - Not maintained by this driver.
	 */

	/* stats.tx_fifo_errors */
	read_mac(APPNIC_TX_STAT_UNDERRUN);

	/* stats.tx_heartbeat_errors - Not maintained by this driver.
	 * stats.tx_window_errors - Not mainteaned by this driver.
	 * stats.rx_compressed - Not maintained by this driver.
	 * stats.tx_compressed - Not maintained by this driver.
	 */
}

/* ----------------------------------------------------------------------
 * get_hw_statistics
 *
 * NOTE: The hardware clears the statistics registers after a read.
 */

static void get_hw_statistics(struct appnic_device *pdata)
{
	unsigned long flags;

	/* stats.tx_packets */
	pdata->stats.tx_packets += read_mac(APPNIC_TX_STAT_PACKET_OK);

	/* stats.multicast */
	pdata->stats.multicast += read_mac(APPNIC_RX_STAT_MULTICAST);

	/* stats.collision */
	pdata->stats.collisions += read_mac(APPNIC_TX_STATUS_LATE_COLLISION);
	pdata->stats.collisions +=
		read_mac(APPNIC_TX_STATUS_EXCESSIVE_COLLISION);
	pdata->stats.collisions +=
	read_mac(APPNIC_TX_STAT_COLLISION_ABOVE_WATERMARK);

	/* stats.rx_length_errors */
	pdata->stats.rx_length_errors += read_mac(APPNIC_RX_STAT_UNDERSIZE);
	pdata->stats.rx_length_errors += read_mac(APPNIC_RX_STAT_OVERSIZE);

	/* stats.tx_fifo_errors */
	pdata->stats.tx_fifo_errors += read_mac(APPNIC_TX_STAT_UNDERRUN);

	/* Lock this section out so the statistics maintained by the driver
	 * don't get clobbered.
	 */

	spin_lock_irqsave(&pdata->dev_lock, flags);

	pdata->stats.rx_errors +=
		(pdata->stats.rx_length_errors +
		 pdata->stats.rx_crc_errors +
		 pdata->stats.rx_frame_errors +
		 pdata->stats.rx_fifo_errors +
		 pdata->stats.rx_dropped +
		 pdata->stats.rx_over_errors);

	pdata->stats.rx_dropped = 0;
	pdata->stats.rx_over_errors = 0;

	pdata->stats.tx_errors += (pdata->stats.tx_fifo_errors +
				   pdata->stats.tx_aborted_errors);
	pdata->stats.tx_aborted_errors = 0;

	spin_unlock_irqrestore(&pdata->dev_lock, flags);
}

/* ----------------------------------------------------------------------
 * queue_initialized
 *
 * Returns the number of descriptors that are ready to receive packets
 * or are waiting to transmit packets.  (from tail to head).
 */

static int queue_initialized(union appnic_queue_pointer head,
			     union appnic_queue_pointer tail,
			     int size)
{
	int initialized;

	/* Calculate the number of descriptors currently initialized. */
	if (head.bits.generation_bit == tail.bits.generation_bit) {
		/* same generation */
		initialized = (head.bits.offset - tail.bits.offset);
	} else {
		/* different generation */
		initialized = head.bits.offset +
			(size * sizeof(struct appnic_dma_descriptor) -
			 tail.bits.offset);
	}

	/* Number of descriptors is offset / sizeof(a descriptor). */
	initialized /= sizeof(struct appnic_dma_descriptor);

	return initialized;
}

/* ----------------------------------------------------------------------
 * queue_uninitialzed
 *
 * Returns the number of unused/uninitialized descriptors. (from head to tail).
*/

static int queue_uninitialized(union appnic_queue_pointer head,
			       union appnic_queue_pointer tail,
			       int size)
{
	int allocated;

	/* Calculate the number of descriptors currently unused/uninitialized */
	if (head.bits.generation_bit == tail.bits.generation_bit)
		/* Same generation. */
		allocated = ((size * sizeof(struct appnic_dma_descriptor)) -
			 head.bits.offset) + tail.bits.offset;
	else
		/* Different generation. */
		allocated = tail.bits.offset - head.bits.offset;

	/* Number of descriptors is offset / sizeof(a descriptor). */
	allocated /= sizeof(struct appnic_dma_descriptor);

	/* That's all. */
	return allocated;
}

/* ----------------------------------------------------------------------
 * queue_increment
 */

static void queue_increment(union appnic_queue_pointer *queue,
			    int number_of_descriptors)
{
	queue->bits.offset += sizeof(struct appnic_dma_descriptor);

	if ((number_of_descriptors * sizeof(struct appnic_dma_descriptor)) ==
		queue->bits.offset) {
		queue->bits.offset = 0;
		queue->bits.generation_bit =
			(0 == queue->bits.generation_bit) ? 1 : 0;
	}
}

/* ----------------------------------------------------------------------
 * queue_decrement
 */

static void queue_decrement(union appnic_queue_pointer *queue,
			    int number_of_descriptors)
{
	if (0 == queue->bits.offset) {
		queue->bits.offset =
			((number_of_descriptors - 1) *
			 sizeof(struct appnic_dma_descriptor));
		queue->bits.generation_bit =
			(0 == queue->bits.generation_bit) ? 1 : 0;
	} else {
		queue->bits.offset -= sizeof(struct appnic_dma_descriptor);
	}
}

/* ----------------------------------------------------------------------
 * disable_rx_tx
 */

static void disable_rx_tx(struct net_device *dev)
{
	struct appnic_device *pdata = netdev_priv(dev);
	unsigned long tx_configuration;
	unsigned long rx_configuration;

	rx_configuration = read_mac(APPNIC_RX_CONF);
	rx_configuration &= ~APPNIC_RX_CONF_ENABLE;
	write_mac(rx_configuration, APPNIC_RX_CONF);

	tx_configuration = read_mac(APPNIC_TX_CONF);
	tx_configuration &= ~APPNIC_TX_CONF_ENABLE;

	write_mac(tx_configuration, APPNIC_TX_CONF);
}

/* ======================================================================
 *  Linux Network Driver Interface
 *  ======================================================================
*/

/* ----------------------------------------------------------------------
 * handle_transmit_interrupt
 */

static void handle_transmit_interrupt(struct net_device *dev)
{
	struct appnic_device *pdata = netdev_priv(dev);
	union appnic_queue_pointer queue;

	/* The hardware's tail pointer should be one descriptor (or more)
	 * ahead of software's copy.
	 */

	queue = swab_queue_pointer(pdata->tx_tail);
	while (0 < queue_initialized(queue, pdata->tx_tail_copy,
				     pdata->tx_num_desc)) {
		queue_increment(&pdata->tx_tail_copy, pdata->tx_num_desc);
		queue = swab_queue_pointer(pdata->tx_tail);
	}
}

/* ----------------------------------------------------------------------
 * lsinet_rx_packet
 */

static void lsinet_rx_packet(struct net_device *dev)
{
	struct appnic_device *pdata = netdev_priv(dev);
	struct appnic_dma_descriptor descriptor;
	struct sk_buff *sk_buff;
	unsigned bytes_copied = 0;
	unsigned error_num = 0;
	unsigned long ok_stat = 0, overflow_stat = 0;
	unsigned long crc_stat = 0, align_stat = 0;
	union appnic_queue_pointer queue;

	readdescriptor(((unsigned long)pdata->rx_desc +
			pdata->rx_tail_copy.bits.offset), &descriptor);

	sk_buff = netdev_alloc_skb(NULL, LSINET_MAX_MTU);

	if ((struct sk_buff *)0 == sk_buff) {
		pr_info_ratelimited("%s: No buffer, packet dropped.\n",
				    LSI_DRV_NAME);
		pdata->stats.rx_dropped++;
		return;
	} else {
		/* Needs to be reviewed.  This fixed an aligment
		* exception when pinging to the target from a host.
		*/

		/* Align IP on 16 byte boundaries */
		skb_reserve(sk_buff, 2);
	}

	ok_stat = read_mac(APPNIC_RX_STAT_PACKET_OK);
	overflow_stat = read_mac(APPNIC_RX_STAT_OVERFLOW);
	crc_stat = read_mac(APPNIC_RX_STAT_CRC_ERROR);
	align_stat = read_mac(APPNIC_RX_STAT_ALIGN_ERROR);

	/* Copy the received packet into the skb. */

	queue = swab_queue_pointer(pdata->rx_tail);
	while (0 < queue_initialized(queue, pdata->rx_tail_copy,
				     pdata->rx_num_desc)) {
		if (skb_tailroom(sk_buff) >= descriptor.pdu_length) {
			unsigned char *buffer;

			buffer = skb_put(sk_buff, descriptor.pdu_length);
			memcpy((void *)buffer,
			       (void *)(descriptor.host_data_memory_pointer +
				 pdata->dma_alloc_offset_rx),
			       descriptor.pdu_length);
		} else {
			pr_err_ratelimited("%s: PDU overrun (%u/%u, %d)\n",
					   LSI_DRV_NAME,
					   descriptor.pdu_length,
					   bytes_copied,
					   descriptor.error);
		}
		bytes_copied += descriptor.pdu_length;
		descriptor.data_transfer_length = pdata->rx_buf_per_desc;
		writedescriptor(((unsigned long)pdata->rx_desc +
					pdata->rx_tail_copy.bits.offset),
				&descriptor);
		if (0 != descriptor.error)
			error_num = 1;
		queue_increment(&pdata->rx_tail_copy, pdata->rx_num_desc);
		if (0 != descriptor.end_of_packet)
			break;
		readdescriptor(((unsigned long)pdata->rx_desc +
					pdata->rx_tail_copy.bits.offset),
			       &descriptor);
		queue = swab_queue_pointer(pdata->rx_tail);
	}

	if (0 == descriptor.end_of_packet) {
		pr_err("%s: No end of packet! %lu/%lu/%lu/%lu\n",
		       LSI_DRV_NAME, ok_stat, overflow_stat,
		       crc_stat, align_stat);
		BUG();
		dev_kfree_skb(sk_buff);

	} else {
		if (0 == error_num) {
			struct ethhdr *ethhdr = (struct ethhdr *) sk_buff->data;

			if (mac_addr_valid(dev, &ethhdr->h_dest[0])) {
				pdata->stats.rx_bytes += bytes_copied;
				pdata->stats.rx_packets++;
				sk_buff->dev = dev;
				sk_buff->protocol = eth_type_trans(sk_buff,
								   dev);
				if (netif_receive_skb(sk_buff) == NET_RX_DROP)
					pdata->dropped_by_stack++;
			} else {
				dev_kfree_skb(sk_buff);
			}
		} else {
			dev_kfree_skb(sk_buff);

			if (0 != overflow_stat)
				pdata->stats.rx_fifo_errors++;
			else if (0 != crc_stat)
				pdata->stats.rx_crc_errors++;
			else if (0 != align_stat)
				pdata->stats.rx_frame_errors++;
		}
	}

	return;
}

/* ----------------------------------------------------------------------
 * lsinet_rx_packets
 */

static int lsinet_rx_packets(struct net_device *dev, int max)
{
	struct appnic_device *pdata = netdev_priv(dev);
	union appnic_queue_pointer orig_queue, new_queue;
	int updated_head_pointer = 0;
	int packets = 0;

	new_queue.raw = pdata->rx_tail_copy.raw;

	/* Receive Packets */

	orig_queue = swab_queue_pointer(pdata->rx_tail);
	while (0 < queue_initialized(orig_queue, new_queue,
				     pdata->rx_num_desc)) {
		struct appnic_dma_descriptor descriptor;

		readdescriptor(((unsigned long)pdata->rx_desc +
				  new_queue.bits.offset),
				&descriptor);

		if (0 != descriptor.end_of_packet) {
			lsinet_rx_packet(dev);
			packets++;
			new_queue.raw = pdata->rx_tail_copy.raw;

			if ((-1 != max) && (packets == max))
				break;
		} else {
			queue_increment(&new_queue, pdata->rx_num_desc);
		}
		orig_queue = swab_queue_pointer(pdata->rx_tail);
	}

	/* Update the Head Pointer */

	while (1 < queue_uninitialized(pdata->rx_head,
				       pdata->rx_tail_copy,
				       pdata->rx_num_desc)) {
		struct appnic_dma_descriptor descriptor;

		readdescriptor(((unsigned long)pdata->rx_desc +
				  pdata->rx_head.bits.offset), &descriptor);
		descriptor.data_transfer_length = pdata->rx_buf_per_desc;
		descriptor.write = 1;
		descriptor.pdu_length = 0;
		descriptor.start_of_packet = 0;
		descriptor.end_of_packet = 0;
		descriptor.interrupt_on_completion = 1;
		writedescriptor(((unsigned long)pdata->rx_desc +
				   pdata->rx_head.bits.offset),
				 &descriptor);
		queue_increment(&pdata->rx_head, pdata->rx_num_desc);
		updated_head_pointer = 1;
	}

	if (0 != updated_head_pointer)
		write_mac(pdata->rx_head.raw, APPNIC_DMA_RX_HEAD_POINTER);

	return packets;
}

/* ----------------------------------------------------------------------
 * lsinet_poll
 */

static int lsinet_poll(struct napi_struct *napi, int budget)
{
	struct appnic_device *pdata =
		container_of(napi, struct appnic_device, napi);
	struct net_device *dev = pdata->device;

	int work_done = 0;
	unsigned long dma_interrupt_status;

	do {
		/* Acknowledge the RX interrupt. */
		write_mac(~APPNIC_DMA_INTERRUPT_ENABLE_RECEIVE, APPNIC_DMA_INTERRUPT_STATUS);

		/* Get Rx packets. */
		work_done += lsinet_rx_packets(dev, budget - work_done);

		/* We've hit the budget limit. */
		if (work_done == budget)
			break;

		dma_interrupt_status = read_mac(APPNIC_DMA_INTERRUPT_STATUS);

	} while (RX_INTERRUPT(dma_interrupt_status));

	if (work_done < budget) {
		napi_complete(napi);

		/* Re-enable receive interrupts (and preserve
		 * the already enabled TX interrupt).
		 */
		write_mac((APPNIC_DMA_INTERRUPT_ENABLE_RECEIVE |
			   APPNIC_DMA_INTERRUPT_ENABLE_TRANSMIT),
			  APPNIC_DMA_INTERRUPT_ENABLE);
	}

	return work_done;
}

/* ----------------------------------------------------------------------
 * appnic_isr
 */

static irqreturn_t appnic_isr(int irq, void *device_id)
{
	struct net_device *dev = (struct net_device *)device_id;
	struct appnic_device *pdata = netdev_priv(dev);
	unsigned long dma_interrupt_status;
	unsigned long flags;

	/* Acquire the lock. */
	spin_lock_irqsave(&pdata->dev_lock, flags);

	/* Get the status. */
	dma_interrupt_status = read_mac(APPNIC_DMA_INTERRUPT_STATUS);

	/* NAPI - don't ack RX interrupt */
	write_mac(APPNIC_DMA_INTERRUPT_ENABLE_RECEIVE,
		  APPNIC_DMA_INTERRUPT_STATUS);

	/* Handle interrupts. */
	if (TX_INTERRUPT(dma_interrupt_status)) {
		/* transmition complete */
		pdata->transmit_interrupts++;
		handle_transmit_interrupt(dev);
	}

	if (RX_INTERRUPT(dma_interrupt_status)) {
		pdata->receive_interrupts++;
		if (napi_schedule_prep(&pdata->napi)) {
			/* Disable RX interrupts and tell the
			 * system we've got work
			 */
			write_mac(APPNIC_DMA_INTERRUPT_ENABLE_TRANSMIT,
				  APPNIC_DMA_INTERRUPT_ENABLE);
			__napi_schedule(&pdata->napi);
		} else {
			write_mac(APPNIC_DMA_INTERRUPT_ENABLE_TRANSMIT,
				  APPNIC_DMA_INTERRUPT_ENABLE);
		}
	}

	/* Release the lock */
	spin_unlock_irqrestore(&pdata->dev_lock, flags);

	return IRQ_HANDLED;
}

#ifdef CONFIG_NET_POLL_CONTROLLER

/* ----------------------------------------------------------------------
 * appnic_poll_controller
 *
 * Polling receive - used by netconsole and other diagnostic tools
 * to allow network i/o with interrupts disabled.
 */

static void appnic_poll_controller(struct net_device *dev)
{
	disable_irq(dev->irq);
	appnic_isr(dev->irq, dev);
	enable_irq(dev->irq);
}

#endif

/* ----------------------------------------------------------------------
 * appnic_open
 *
 * Opens the interface.  The interface is opened whenever ifconfig
 * activates it.  The open method should register any system resource
 * it needs (I/O ports, IRQ, DMA, etc.) turn on the hardware, and
 * increment the module usage count.
 */

static int appnic_open(struct net_device *dev)
{
	struct appnic_device *pdata = netdev_priv(dev);
	int return_code = 0;

	/* Bring the PHY up. */
	phy_start(pdata->phy_dev);

	/* Enable NAPI. */
	napi_enable(&pdata->napi);

	/* Install the interrupt handlers. */
	return_code = request_irq(dev->irq, appnic_isr, 0x00, LSI_DRV_NAME, dev);
	if (0 != return_code) {
		pr_err("%s: request_irq() failed, returned 0x%x/%d\n",
		       LSI_DRV_NAME, return_code, return_code);
		return return_code;
	}

	/* Enable interrupts. */
	write_mac((APPNIC_DMA_INTERRUPT_ENABLE_RECEIVE |
		   APPNIC_DMA_INTERRUPT_ENABLE_TRANSMIT),
		   APPNIC_DMA_INTERRUPT_ENABLE);

	/* Let the OS know we are ready to send packets. */
	netif_start_queue(dev);

	/* That's all. */
	return 0;
}

/* ----------------------------------------------------------------------
 * appnic_stop
 *
 * Stops the interface.  The interface is stopped when it is brought
 * down; operations performed at open time should be reversed.
 */

static int appnic_stop(struct net_device *dev)
{
	struct appnic_device *pdata = netdev_priv(dev);

	pr_info("%s: Stopping the interface.\n", LSI_DRV_NAME);

	/* Disable all device interrupts. */
	write_mac(0, APPNIC_DMA_INTERRUPT_ENABLE);
	free_irq(dev->irq, dev);

	/* Indicate to the OS that no more packets should be sent.  */
	netif_stop_queue(dev);
	napi_disable(&pdata->napi);

	/* Stop the receiver and transmitter. */
	disable_rx_tx(dev);

	/* Bring the PHY down. */
	if (pdata->phy_dev)
		phy_stop(pdata->phy_dev);

	/* That's all. */
	return 0;
}

/* ----------------------------------------------------------------------
 * appnic_hard_start_xmit
 *
 * The method initiates the transmission of a packet.  The full packet
 * (protocol headers and all) is contained in a socket buffer (sk_buff)
 * structure.
 *
 * ----- NOTES -----
 *
 * 1) This will not get called again by the kernel until it returns.
 */

static int appnic_hard_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct appnic_device *pdata = netdev_priv(dev);
	int length;
	int buf_per_desc;
	union appnic_queue_pointer queue;

	length = skb->len < ETH_ZLEN ? ETH_ZLEN : skb->len;
	buf_per_desc = pdata->tx_buf_sz / pdata->tx_num_desc;

	/* If enough transmit descriptors are available, copy and transmit. */

	queue = swab_queue_pointer(pdata->tx_tail);
	while (((length / buf_per_desc) + 1) >=
		queue_uninitialized(pdata->tx_head,
				    queue,
				    pdata->tx_num_desc)) {
		handle_transmit_interrupt(dev);
		queue = swab_queue_pointer(pdata->tx_tail);
	}

	if (((length / buf_per_desc) + 1) <
		queue_uninitialized(pdata->tx_head, queue,
				    pdata->tx_num_desc)) {
		int bytes_copied = 0;
		struct appnic_dma_descriptor descriptor;

		readdescriptor(((unsigned long)pdata->tx_desc +
				pdata->tx_head.bits.offset), &descriptor);
		descriptor.start_of_packet = 1;

		while (bytes_copied < length) {
			descriptor.write = 1;
			descriptor.pdu_length = length;

			if ((length - bytes_copied) > buf_per_desc) {
				memcpy((void *)
					(descriptor.host_data_memory_pointer +
					 pdata->dma_alloc_offset_tx),
				       (void *) ((unsigned long) skb->data +
					bytes_copied),
					buf_per_desc);
				descriptor.data_transfer_length = buf_per_desc;
				descriptor.end_of_packet = 0;
				descriptor.interrupt_on_completion = 0;
				bytes_copied += buf_per_desc;
			} else {
				memcpy((void *)
					(descriptor.host_data_memory_pointer +
					 pdata->dma_alloc_offset_tx),
				       (void *) ((unsigned long) skb->data +
					bytes_copied),
					(length - bytes_copied));
				descriptor.data_transfer_length =
				 (length - bytes_copied);
				descriptor.end_of_packet = 1;
				/* Leave TX interrupts disabled. We work
				 * the same with or w/o them. Set to "1"
				 * if we ever want to enable them though.
				 */
				descriptor.interrupt_on_completion = 0;
				bytes_copied = length;
			}

			pdata->stats.tx_bytes += bytes_copied;
			writedescriptor(((unsigned long) pdata->tx_desc +
				pdata->tx_head.bits.offset), &descriptor);
			queue_increment(&pdata->tx_head, pdata->tx_num_desc);
			readdescriptor(((unsigned long)pdata->tx_desc +
					 pdata->tx_head.bits.offset),
					&descriptor);
			descriptor.start_of_packet = 0;
		}

		/* Data sync barrier. */
		rmb();

		write_mac(pdata->tx_head.raw, APPNIC_DMA_TX_HEAD_POINTER);
		dev->trans_start = jiffies;

	} else {
		pdata->out_of_tx_descriptors++;
		pr_err("%s: No transmit descriptors available!\n",
		       LSI_DRV_NAME);
		return NETDEV_TX_BUSY;
	}

	/* Free the socket buffer. */
	dev_kfree_skb(skb);

	return NETDEV_TX_OK;
}

/* ----------------------------------------------------------------------
 * appnic_net_device_stats
 *
 * Whenever an application needs to get statistics for the interface,
 * this method is called.  This happens, for example, when ifconfig or
 * nstat -i is run.
 */

static struct net_device_stats *appnic_get_stats(struct net_device *dev)
{
	struct appnic_device *pdata = netdev_priv(dev);

	/* Update the statistics structure. */

	get_hw_statistics(pdata);

	return &pdata->stats;
}

/* ----------------------------------------------------------------------
 * appnic_set_mac_address
 */

static int appnic_set_mac_address(struct net_device *dev, void *data)
{
	struct appnic_device *pdata = netdev_priv(dev);
	struct sockaddr *address = data;
	unsigned long swap_source_address;

	if (netif_running(dev))
		return -EBUSY;

	if (!is_valid_ether_addr(address->sa_data))
		return -EADDRNOTAVAIL;

	memcpy(dev->dev_addr, address->sa_data, ETH_ALEN);
	memcpy(dev->perm_addr, address->sa_data, ETH_ALEN);

	swap_source_address = ((address->sa_data[4]) << 8) |
			       address->sa_data[5];
	write_mac(swap_source_address, APPNIC_SWAP_SOURCE_ADDRESS_2);
	swap_source_address = ((address->sa_data[2]) << 8) |
			address->sa_data[3];
	write_mac(swap_source_address, APPNIC_SWAP_SOURCE_ADDRESS_1);
	swap_source_address = ((address->sa_data[0]) << 8) |
			       address->sa_data[1];
	write_mac(swap_source_address, APPNIC_SWAP_SOURCE_ADDRESS_0);
	memcpy(dev->dev_addr, address->sa_data, dev->addr_len);

	return 0;
}

/* ======================================================================
 * ETHTOOL Operations
 * ======================================================================
 */

enum {NETDEV_STATS, APPNIC_STATS};

struct appnic_stats {
	char stat_string[ETH_GSTRING_LEN];
	int sizeof_stat;
	int stat_offset;
};

#define APPNIC_STAT(str, m) { \
		.stat_string = str, \
		.sizeof_stat = sizeof(((struct appnic_device *)0)->m), \
		.stat_offset = offsetof(struct appnic_device, m) }

static const struct appnic_stats appnic_gstrings_stats[] = {
	APPNIC_STAT("rx_packets", stats.rx_packets),
	APPNIC_STAT("tx_packets", stats.tx_packets),
	APPNIC_STAT("rx_bytes", stats.rx_bytes),
	APPNIC_STAT("tx_bytes", stats.tx_bytes),
	APPNIC_STAT("rx_errors", stats.rx_errors),
	APPNIC_STAT("tx_errors", stats.tx_errors),
	APPNIC_STAT("rx_dropped", stats.rx_dropped),
	APPNIC_STAT("tx_dropped", stats.tx_dropped),
	APPNIC_STAT("multicast", stats.multicast),
	APPNIC_STAT("collisions", stats.collisions),
	APPNIC_STAT("rx_length_errors", stats.rx_length_errors),
	APPNIC_STAT("rx_crc_errors", stats.rx_crc_errors),
	APPNIC_STAT("rx_frame_errors", stats.rx_frame_errors),
	APPNIC_STAT("rx_fifo_errors", stats.rx_fifo_errors),
	APPNIC_STAT("tx_fifo_errors", stats.tx_fifo_errors),

	APPNIC_STAT("dropped_by_stack", dropped_by_stack),
	APPNIC_STAT("out_of_tx_descriptors", out_of_tx_descriptors),
	APPNIC_STAT("transmit_interrupts", transmit_interrupts),
	APPNIC_STAT("receive_interrupts", receive_interrupts),
};

#define APPNIC_GLOBAL_STATS_LEN  ARRAY_SIZE(appnic_gstrings_stats)
#define APPNIC_STATS_LEN (APPNIC_GLOBAL_STATS_LEN)

/* ----------------------------------------------------------------------
 * appnic_get_ethtool_stats
 */

static void appnic_get_ethtool_stats(struct net_device *dev,
				     struct ethtool_stats *stats,
				     u64 *data)
{
	struct appnic_device *pdata = netdev_priv(dev);
	int i;
	char *p = NULL;

	get_hw_statistics(pdata);
	for (i = 0; i < APPNIC_GLOBAL_STATS_LEN; i++) {
		p = (char *) pdata + appnic_gstrings_stats[i].stat_offset;
		data[i] = (appnic_gstrings_stats[i].sizeof_stat ==
			sizeof(u64)) ? *(u64 *)p : *(u32 *)p;
	}
}

/* ----------------------------------------------------------------------
 * appnic_get_strings
 */

static void appnic_get_strings(struct net_device *netdev, u32 stringset,
			       u8 *data)
{
	u8 *p = data;
	int i;

	switch (stringset) {
	case ETH_SS_STATS:
		for (i = 0; i < APPNIC_GLOBAL_STATS_LEN; i++) {
			memcpy(p, appnic_gstrings_stats[i].stat_string,
			       ETH_GSTRING_LEN);
			p += ETH_GSTRING_LEN;
		}
		break;
	}
}

/* ----------------------------------------------------------------------
 * appnic_get_sset_count
 */

static int appnic_get_sset_count(struct net_device *netdev, int sset)
{
	switch (sset) {
	case ETH_SS_STATS:
		return APPNIC_STATS_LEN;
	default:
		return -EOPNOTSUPP;
	}
}

/* ----------------------------------------------------------------------
 * appnic_get_drvinfo
 */

static void appnic_get_drvinfo(struct net_device *dev,
			       struct ethtool_drvinfo *info)
{
	strcpy(info->driver, LSI_DRV_NAME);
	strcpy(info->version, LSI_DRV_VERSION);
	strlcpy(info->bus_info, dev_name(dev->dev.parent),
		sizeof(info->bus_info));
}

/* ----------------------------------------------------------------------
 * appnic_get_settings
 */

static int appnic_get_link_ksettings(struct net_device *dev,
			       struct ethtool_link_ksettings *cmd)
{
	struct appnic_device *pdata = netdev_priv(dev);
	struct phy_device *phydev = pdata->phy_dev;

	if (!phydev)
		return -ENODEV;

	phy_ethtool_ksettings_get(phydev, cmd);

	return 0;
}

/* Fill in the struture...  */

static const struct ethtool_ops appnic_ethtool_ops = {
	.get_drvinfo		= appnic_get_drvinfo,
	.get_link_ksettings		= appnic_get_link_ksettings,
	.get_ethtool_stats	= appnic_get_ethtool_stats,
	.get_strings		= appnic_get_strings,
	.get_sset_count		= appnic_get_sset_count,
};

/* ======================================================================
 * Linux Module Interface.
 * ======================================================================
 */

static const struct net_device_ops appnic_netdev_ops = {
	.ndo_open = appnic_open,
	.ndo_stop = appnic_stop,
	.ndo_get_stats = appnic_get_stats,
	.ndo_set_mac_address = appnic_set_mac_address,
	.ndo_start_xmit = appnic_hard_start_xmit,
#ifdef CONFIG_NET_POLL_CONTROLLER
	.ndo_poll_controller = appnic_poll_controller,
#endif

};

/* ----------------------------------------------------------------------
 * appnic_init
 */

int appnic_init(struct net_device *dev)
{
	struct appnic_device *pdata = netdev_priv(dev);
	void *dma_offset;
	int index;
	unsigned long buf;
	struct appnic_dma_descriptor descriptor;
	struct sockaddr address;
	unsigned long node_cfg;
	int rc = 0;

	/* Set FEMAC to uncached */
	femac_uncache(pdata);

	/* Reset the MAC. */

	write_mac(0x80000000, APPNIC_DMA_PCI_CONTROL);

	/* Allocate memory and initialize the descriptors. */

	/* fixup num_[rt]x_desc. */

	if (0 != (rx_num_desc % DESCRIPTOR_GRANULARITY)) {
		pr_err("%s: rx_num_desc was not a multiple of %d.\n",
		       LSI_DRV_NAME, DESCRIPTOR_GRANULARITY);
		rc = -EINVAL;
		goto err_param;
	}

	pdata->rx_num_desc = rx_num_desc;

	if (0 != (tx_num_desc % DESCRIPTOR_GRANULARITY)) {
		pr_err("%s: tx_num_desc was not a multiple of %d.\n",
		       LSI_DRV_NAME, DESCRIPTOR_GRANULARITY);
		rc = -EINVAL;
		goto err_param;
	}

	pdata->tx_num_desc = tx_num_desc;

	/* up [rt]x_buf_sz. Must be some multiple of 64 bytes
	 * per descriptor.
	 */

	if (0 != (rx_buf_sz % (BUFFER_ALIGNMENT * rx_num_desc))) {
		pr_err("%s: rx_buf_sz was not a multiple of %d.\n",
		       LSI_DRV_NAME, (BUFFER_ALIGNMENT * rx_num_desc));
		rc = -EINVAL;
		goto err_param;
	}

	pdata->rx_buf_sz = rx_buf_sz;

	if (0 != (tx_buf_sz % (BUFFER_ALIGNMENT * tx_num_desc))) {
		pr_err("%s: tx_buf_sz was not a multiple of %d.\n",
		       LSI_DRV_NAME, (BUFFER_ALIGNMENT * tx_num_desc));
		rc = -EINVAL;
		goto err_param;
	}

	pdata->tx_buf_sz = tx_buf_sz;

	/* Allocate dma-able memory. Broken into smaller parts to keep
	 * from allocating a single large chunk of memory, but not too
	 * small since mappings obtained from dma_alloc_coherent() have
	 * a minimum size of one page.
	 */

	pdata->dma_alloc_size =
		/* The tail pointers (rx and tx) */
		(sizeof(union appnic_queue_pointer) * 2) +
		/* The RX descriptor ring (and padding to allow
		 * 64 byte alignment)
		 */
		(sizeof(struct appnic_dma_descriptor) * pdata->rx_num_desc) +
		(DESCRIPTOR_GRANULARITY) +
		/* The TX descriptor ring (and padding...) */
		(sizeof(struct appnic_dma_descriptor) * pdata->tx_num_desc) +
		(DESCRIPTOR_GRANULARITY);

	pdata->dma_alloc_size_rx =
		/* The RX buffer (and padding...) */
		(pdata->rx_buf_sz) + (BUFFER_ALIGNMENT);

	pdata->dma_alloc_size_tx =
		/* The TX buffer (and padding...) */
		(pdata->tx_buf_sz) + (BUFFER_ALIGNMENT);

	/* Allocate the buffers. */

	rc = femac_alloc_mem_buffers(dev);
	if (rc != 0) {
		pr_err("%s: Can't allocate DMA-able memory!\n", LSI_DRV_NAME);
		goto err_mem_buffers;
	}

	/* Initialize the tail pointers. */

	dma_offset = pdata->dma_alloc;

	pdata->rx_tail = (union appnic_queue_pointer *)dma_offset;
	pdata->rx_tail_dma = (int)pdata->rx_tail - (int)pdata->dma_alloc_offset;
	dma_offset += sizeof(union appnic_queue_pointer);
	memset((void *)pdata->rx_tail, 0,
	       sizeof(union appnic_queue_pointer));

	pdata->tx_tail = (union appnic_queue_pointer *)dma_offset;
	pdata->tx_tail_dma = (int)pdata->tx_tail - (int)pdata->dma_alloc_offset;
	dma_offset += sizeof(union appnic_queue_pointer);
	memset((void *)pdata->tx_tail, 0, sizeof(union appnic_queue_pointer));

	/* Initialize the descriptor pointers. */

	pdata->rx_desc = (struct appnic_dma_descriptor *)ALIGN64B(dma_offset);
	pdata->rx_desc_dma = (int)pdata->rx_desc - (int)pdata->dma_alloc_offset;
	dma_offset += (sizeof(struct appnic_dma_descriptor) *
			pdata->rx_num_desc) + (DESCRIPTOR_GRANULARITY);
	memset((void *)pdata->rx_desc, 0,
	       (sizeof(struct appnic_dma_descriptor) * pdata->rx_num_desc));

	pdata->tx_desc = (struct appnic_dma_descriptor *)ALIGN64B(dma_offset);
	pdata->tx_desc_dma = (int)pdata->tx_desc - (int)pdata->dma_alloc_offset;
	dma_offset += (sizeof(struct appnic_dma_descriptor) *
			pdata->tx_num_desc) + (DESCRIPTOR_GRANULARITY);
	memset((void *)pdata->tx_desc, 0,
	       (sizeof(struct appnic_dma_descriptor) * pdata->tx_num_desc));

	/* Initialize the buffer pointers. */

	dma_offset = pdata->dma_alloc_rx;

	pdata->rx_buf = (void *)ALIGN64B(dma_offset);
	pdata->rx_buf_dma = (int)pdata->rx_buf -
				(int)pdata->dma_alloc_offset_rx;
	pdata->rx_buf_per_desc = pdata->rx_buf_sz / pdata->rx_num_desc;

	dma_offset = pdata->dma_alloc_tx;

	pdata->tx_buf = (void *)ALIGN64B(dma_offset);
	pdata->tx_buf_dma = (int)pdata->tx_buf -
				(int)pdata->dma_alloc_offset_tx;
	pdata->tx_buf_per_desc = pdata->tx_buf_sz / pdata->tx_num_desc;

	/* Initialize the descriptors. */

	buf = (unsigned long)pdata->rx_buf_dma;
	for (index = 0; index < pdata->rx_num_desc; ++index) {
		memset((void *) &descriptor, 0,
		       sizeof(struct appnic_dma_descriptor));
		descriptor.write = 1;
		descriptor.interrupt_on_completion = 1;
		descriptor.host_data_memory_pointer = buf;
		descriptor.data_transfer_length = pdata->rx_buf_per_desc;

		writedescriptor(((unsigned long)pdata->rx_desc + (index *
				sizeof(struct appnic_dma_descriptor))),
				&descriptor);

		buf += pdata->rx_buf_per_desc;
	}

	buf = (unsigned long)pdata->tx_buf_dma;

	for (index = 0; index < pdata->tx_num_desc; ++index) {
		memset((void *) &descriptor, 0,
		       sizeof(struct appnic_dma_descriptor));
		descriptor.write = 1;
		descriptor.interrupt_on_completion = 1;
		descriptor.host_data_memory_pointer = buf;

		writedescriptor(((unsigned long)pdata->tx_desc + (index *
				 sizeof(struct appnic_dma_descriptor))),
				&descriptor);

		buf += pdata->tx_buf_per_desc;
	}

	/* Initialize the spinlocks. */

	spin_lock_init(&pdata->dev_lock);

	/* Take MAC out of reset. */

	write_mac(0x0, APPNIC_RX_SOFT_RESET);
	write_mac(0x1, APPNIC_RX_MODE);
	write_mac(0x0, APPNIC_TX_SOFT_RESET);
	write_mac(0x1, APPNIC_TX_MODE);

	/* Set the watermark. */

	ncr_read(NCP_REGION_ID(0x16, 0xff), 0x10, 4, &node_cfg);

	if (0 == (0x80000000 & node_cfg))
		write_mac(0x300a, APPNIC_TX_WATERMARK);
	else
		write_mac(0xc00096, APPNIC_TX_WATERMARK);

	write_mac(0x1, APPNIC_TX_HALF_DUPLEX_CONF);
	write_mac(0xffff, APPNIC_TX_TIME_VALUE_CONF);
	write_mac(0x1, APPNIC_TX_INTERRUPT_CONTROL);
	write_mac(0x5275, APPNIC_TX_EXTENDED_CONF);
	write_mac(0x1, APPNIC_RX_INTERNAL_INTERRUPT_CONTROL);
	write_mac(0x1, APPNIC_RX_EXTERNAL_INTERRUPT_CONTROL);
	write_mac(0x40010000, APPNIC_DMA_PCI_CONTROL);
	write_mac(0x30000, APPNIC_DMA_CONTROL);
#ifdef CONFIG_ARM
	writel(0x280044,
	       (void __iomem *)((unsigned long)pdata->dma_base + 0x60));
	writel(0xc0,
	       (void __iomem *)((unsigned long)pdata->dma_base + 0x64));
#else
	out_le32((unsigned *)pdata->dma_base + 0x60, 0x280044);
	out_le32((unsigned *)pdata->dma_base + 0x64, 0xc0);
#endif

	/* Set the MAC address. */
	pr_info("%s: MAC %pM\n", LSI_DRV_NAME, dev->dev_addr);

	memcpy(&(address.sa_data[0]), dev->dev_addr, ETH_ALEN);
	rc = appnic_set_mac_address(dev, &address);
	if (rc != 0) {
		pr_err("%s: Unable to set MAC address!\n", LSI_DRV_NAME);
		goto err_set_mac_addr;
	}

	/* Initialize the queue pointers. */

	/* Receiver. */

	memset((void *)&pdata->rx_tail_copy, 0,
	       sizeof(union appnic_queue_pointer));
	memset((void *)&pdata->rx_head, 0,
	       sizeof(union appnic_queue_pointer));

	write_mac(pdata->rx_desc_dma, APPNIC_DMA_RX_QUEUE_BASE_ADDRESS);
	write_mac((pdata->rx_num_desc *
		   sizeof(struct appnic_dma_descriptor)) / 1024,
		  APPNIC_DMA_RX_QUEUE_SIZE);

	/* Indicate that all of the receive descriptors
	 * are ready.
	 */

	pdata->rx_head.bits.offset = (pdata->rx_num_desc - 1) *
					sizeof(struct appnic_dma_descriptor);
	write_mac(pdata->rx_tail_dma, APPNIC_DMA_RX_TAIL_POINTER_ADDRESS);

	/* N.B.
	 *
	 * The boot loader may have used the NIC.  If so, the
	 * tail pointer must be read and the head pointer (and
	 * local copy of the tail) based on it.
	 */

	pdata->rx_tail->raw =
		  read_mac(APPNIC_DMA_RX_TAIL_POINTER_LOCAL_COPY);
	pdata->rx_tail_copy.raw = pdata->rx_tail->raw;
	pdata->rx_head.raw = pdata->rx_tail->raw;
	queue_decrement(&pdata->rx_head, pdata->rx_num_desc);
	pdata->rx_head.bits.generation_bit =
		  (0 == pdata->rx_head.bits.generation_bit) ? 1 : 0;
	write_mac(pdata->rx_head.raw, APPNIC_DMA_RX_HEAD_POINTER);

	/* Transmitter. */

	memset((void *) &pdata->tx_tail_copy, 0,
	       sizeof(union appnic_queue_pointer));
	memset((void *) &pdata->tx_head, 0,
	       sizeof(union appnic_queue_pointer));

	write_mac(pdata->tx_desc_dma, APPNIC_DMA_TX_QUEUE_BASE_ADDRESS);
	write_mac((pdata->tx_num_desc *
		   sizeof(struct appnic_dma_descriptor)) / 1024,
		  APPNIC_DMA_TX_QUEUE_SIZE);
	write_mac(pdata->tx_tail_dma, APPNIC_DMA_TX_TAIL_POINTER_ADDRESS);

	/* N.B.
	 *
	 * The boot loader may have used the NIC.  If so, the
	 * tail pointer must be read and the head pointer (and
	 * local copy of the tail) based on it.
	 */

	pdata->tx_tail->raw = read_mac(APPNIC_DMA_TX_TAIL_POINTER_LOCAL_COPY);
	pdata->tx_tail_copy.raw = pdata->tx_tail->raw;
	pdata->tx_head.raw = pdata->tx_tail->raw;
	write_mac(pdata->tx_head.raw, APPNIC_DMA_TX_HEAD_POINTER);

	/* Clear statistics. */

	clear_statistics(pdata);

	/* Fill in the net_device structure. */

	ether_setup(dev);

	/* Setup IRQ. */
	rc = femac_irq_setup(dev);
	if (rc != 0) {
		pr_err("%s: IRQ setup failed!\n", LSI_DRV_NAME);
		goto err_irq_setup;
	}

	dev->netdev_ops = &appnic_netdev_ops;
	dev->ethtool_ops = &appnic_ethtool_ops;

	memset((void *) &pdata->napi, 0, sizeof(struct napi_struct));
	netif_napi_add(dev, &pdata->napi,
		       lsinet_poll, LSINET_NAPI_WEIGHT);
	pdata->device = dev;

	return 0;

err_irq_setup:
err_set_mac_addr:
	femac_free_mem_buffers(dev);
err_mem_buffers:
err_param:
	return rc;
}

/* ----------------------------------------------------------------------
 * appnic_probe_config_dt
 */

#ifdef CONFIG_OF
static int appnic_probe_config_dt(struct net_device *dev,
				  struct device_node *np)
{
	struct appnic_device *pdata = netdev_priv(dev);
	const u32 *field;
	const char *mac;
	const char *macspeed;
#ifdef CONFIG_ARM
	struct device_node *gp_node;
#else
	u64 value64;
	u32 value32;
#endif

	if (!np)
		return -ENODEV;

#ifdef CONFIG_ARM
	gp_node = of_find_compatible_node(NULL, NULL, "lsi,gpreg");
	if (!gp_node) {
		pr_err("%s: DTS is missing mode 'gpreg'\n", LSI_DRV_NAME);
		return -ENODEV;
	}
	pdata->gpreg_base = of_iomap(gp_node, 0);

	pdata->rx_base = of_iomap(np, 0);
	pdata->tx_base = of_iomap(np, 1);
	pdata->dma_base = of_iomap(np, 2);

	pdata->tx_interrupt = irq_of_parse_and_map(np, 0);
	pdata->rx_interrupt = irq_of_parse_and_map(np, 1);
	pdata->dma_interrupt = irq_of_parse_and_map(np, 2);
#else
	field = of_get_property(np, "enabled", NULL);

	if (!field || (field && (0 == *field)))
		goto device_tree_failed;

	field = of_get_property(np, "reg", NULL);

	if (!field)
		goto device_tree_failed;

	value64 = of_translate_address(np, field);
	value32 = field[3];
	field += 2;
	pdata->rx_base = ioremap(value64, value32);
	value64 = of_translate_address(np, field);
	value32 = field[3];
	field += 2;
	pdata->tx_base = ioremap(value64, value32);
	value64 = of_translate_address(np, field);
	value32 = field[3];
	field += 2;
	pdata->dma_base = ioremap(value64, value32);

	field = of_get_property(np, "interrupts", NULL);
	if (!field)
		goto device_tree_failed;
	else
		pdata->dma_interrupt = field[0];
#endif

	field = of_get_property(np, "mdio-clock", NULL);
	if (!field)
		goto device_tree_failed;
	else
		pdata->mdio_clock = ntohl(field[0]);

	field = of_get_property(np, "phy-address", NULL);
	if (!field)
		goto device_tree_failed;
	else
		pdata->phy_address = ntohl(field[0]);

	field = of_get_property(np, "ad-value", NULL);
	if (!field)
		goto device_tree_failed;
	else
		pdata->ad_value = ntohl(field[0]);

	macspeed = of_get_property(np, "phy-link", NULL);

	if (macspeed) {
		if (0 == strncmp(macspeed, "auto", strlen("auto"))) {
			pdata->phy_link_auto = 1;
		} else if (0 == strncmp(macspeed, "100MF", strlen("100MF"))) {
			pdata->phy_link_auto = 0;
			pdata->phy_link_speed = 1;
			pdata->phy_link_duplex = 1;
		} else if (0 == strncmp(macspeed, "100MH", strlen("100MH"))) {
			pdata->phy_link_auto = 0;
			pdata->phy_link_speed = 1;
			pdata->phy_link_duplex = 0;
		} else if (0 == strncmp(macspeed, "10MF", strlen("10MF"))) {
			pdata->phy_link_auto = 0;
			pdata->phy_link_speed = 0;
			pdata->phy_link_duplex = 1;
		} else if (0 == strncmp(macspeed, "10MH", strlen("10MH"))) {
			pdata->phy_link_auto = 0;
			pdata->phy_link_speed = 0;
			pdata->phy_link_duplex = 0;
		} else {
			pr_err("Invalid phy-link value \"%s\" in DTS. Defaulting to \"auto\".\n", macspeed);
			pdata->phy_link_auto = 1;
		}
	} else {
		/* Auto is the default. */
		pdata->phy_link_auto = 1;
	}

	mac = of_get_mac_address(np);
	if (!mac)
		goto device_tree_failed;

	memcpy(&pdata->mac_addr[0], mac, ETH_ALEN);
	memcpy(dev->dev_addr, mac, ETH_ALEN);
	memcpy(dev->perm_addr, mac, ETH_ALEN);

	return 0;

device_tree_failed:
	pr_err("%s: Reading Device Tree Failed\n", LSI_DRV_NAME);
#ifdef CONFIG_ARM
	iounmap(pdata->gpreg_base);
#endif
	iounmap(pdata->rx_base);
	iounmap(pdata->tx_base);
	iounmap(pdata->dma_base);

	return -EINVAL;
}
#else
static inline int appnic_probe_config_dt(struct net_device *dev,
					 struct device_node *np)
{
	return -ENODEV;
}
#endif /* CONFIG_OF */

/* ----------------------------------------------------------------------
 * appnic_drv_probe
 */

static int appnic_drv_probe(struct platform_device *pdev)
{
	int rc = 0;
	struct device_node *np = pdev->dev.of_node;
	struct net_device *dev;
	struct appnic_device *pdata;

	pr_info("%s: LSI(R) 10/100 Network Driver - version %s\n",
		LSI_DRV_NAME, LSI_DRV_VERSION);

	/* Allocate space for the device. */

	dev = alloc_etherdev(sizeof(struct appnic_device));
	if (!dev) {
		pr_err("%s: Couldn't allocate net device.\n", LSI_DRV_NAME);
		rc = -ENOMEM;
		goto err_alloc_etherdev;
	}

	SET_NETDEV_DEV(dev, &pdev->dev);
	platform_set_drvdata(pdev, dev);

	pdata = netdev_priv(dev);

	/* Get the physical addresses, interrupt number, etc. from the
	 * device tree.  If no entry exists (older boot loader...) just
	 * use the pre-devicetree method.
	 */

	rc = appnic_probe_config_dt(dev, np);

	if (rc == -EINVAL) {
		goto err_inval;
	} else if (rc == -EINVAL) {
#ifdef CONFIG_MTD_NAND_EP501X_UBOOTENV

		/* The attempt to get device settings from the DTB failed, so
		 * try to grab the ethernet MAC from the u-boot environment
		 * and use hard-coded values for device base addresses.
		 */

		unsigned char ethaddr_string[20];

		if (0 != ubootenv_get("ethaddr", ethaddr_string)) {
			pr_err("%s: Could not read ethernet address!\n",
			       LSI_DRV_NAME);
			rc = -EINVAL;
			goto err_inval;
		} else {
			u8 mac_address[ETH_ALEN];
			int i = 0;
			char *string = ethaddr_string;

			while ((0 != string) && (ETH_ALEN > i)) {
				char *value;
				unsigned long res;

				value = strsep(&string, ":");
				if (kstrtoul(value, 16, &res))
					return -EBUSY;
				mac_address[i++] = (u8)res;
			}

			memcpy(dev->dev_addr, mac_address, ETH_ALEN);
			memcpy(dev->perm_addr, mac_address, ETH_ALEN);
			dev->addr_len = ETH_ALEN;

			pr_info("%s: Using Static Addresses and Interrupts",
				LSI_DRV_NAME);
			pdata->rx_base = ioremap(0x002000480000ULL, 0x1000);
			pdata->tx_base = ioremap(0x002000481000ULL, 0x1000);
			pdata->dma_base = ioremap(0x002000482000ULL, 0x1000);
			pdata->dma_interrupt = 33;
		}
#else
		/* Neither dtb info nor ubootenv driver found. */
		pr_err("%s: Could not read ethernet address!", LSI_DRV_NAME);
		rc = -EINVAL;
		goto err_inval;
#endif
	}

#ifdef CONFIG_MTD_NAND_EP501X_UBOOTENV

	{
		unsigned char uboot_env_string[20];

		/* Override ad_value with u-boot environment variable if set. */
		if (0 == ubootenv_get("ad_value", uboot_env_string)) {
			/* Assume ad_value is always entered as a hex value,
			 * since u-boot defaults this value as hex.
			 */
			unsigned long res;

			if (kstrtoul(uboot_env_string, 16, &res)) {
				rc = -EINVAL;
				goto err_inval;
			}
			pdata->ad_value = res;
		}
	}

#endif

	/* Initialize the device. */
	rc = appnic_init(dev);
	if (0 != rc) {
		pr_err("%s: appnic_init() failed: %d\n", LSI_DRV_NAME, rc);
		rc = -ENODEV;
		goto err_nodev;
	}

	/* Register the device. */
	rc = register_netdev(dev);
	if (0 != rc) {
		pr_err("%s: register_netdev() failed: %d\n", LSI_DRV_NAME, rc);
		rc = -ENODEV;
		goto err_nodev;
	}

	/* Initialize the PHY. */
	rc = appnic_mii_init(pdev, dev);
	if (rc) {
		pr_warn("%s: Failed to initialize PHY", LSI_DRV_NAME);
		rc = -ENODEV;
		goto err_mii_init;
	}

	return 0;

err_mii_init:
	unregister_netdev(dev);
err_nodev:
err_inval:
	free_netdev(dev);
err_alloc_etherdev:
	return rc;
}

/* ----------------------------------------------------------------------
 * appnic_drv_remove
 */

static int appnic_drv_remove(struct platform_device *pdev)
{
	struct net_device *dev = platform_get_drvdata(pdev);
	struct appnic_device *pdata = NULL;

	pr_info("%s: Stopping driver", LSI_DRV_NAME);

	BUG_ON(!dev);
	pdata = netdev_priv(dev);
	BUG_ON(!pdata);
	BUG_ON(!pdata->phy_dev);
	phy_disconnect(pdata->phy_dev);
	pdata->phy_dev = NULL;
	mdiobus_unregister(pdata->mii_bus);
	mdiobus_free(pdata->mii_bus);
	platform_set_drvdata(pdev, NULL);
	unregister_netdev(dev);
	free_irq(dev->irq, dev);
	femac_free_mem_buffers(dev);
	free_netdev(dev);

	iounmap(pdata->rx_base);
	iounmap(pdata->tx_base);
	iounmap(pdata->dma_base);
#ifdef CONFIG_ARM
	iounmap(pdata->gpreg_base);
#endif

	return 0;
}

static const struct of_device_id appnic_dt_ids[] = {
	{ .compatible = "lsi,acp-femac", },
	{ .compatible = "acp-femac", },
	{ /* end of list */ },
};
MODULE_DEVICE_TABLE(of, appnic_dt_ids);

static struct platform_driver appnic_driver = {
	.probe = appnic_drv_probe,
	.remove = appnic_drv_remove,
	.driver = {
		.name   = LSI_DRV_NAME,
		.owner  = THIS_MODULE,
		.pm     = NULL,
		.of_match_table = appnic_dt_ids,
	},
};

module_platform_driver(appnic_driver);
