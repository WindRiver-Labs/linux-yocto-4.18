// SPDX-License-Identifier: (GPL-2.0 OR MIT)
/* Felix Switch driver
 *
 * Copyright 2018 NXP
 */

#include <linux/module.h>
#include <linux/pci.h>
#include <linux/netdevice.h>
#include <linux/phy_fixed.h>
#include <linux/phy.h>
#include <linux/of_mdio.h>
#include <net/sock.h>

#include "ocelot.h"

static const char felix_driver_string[] = "Felix Switch Driver";
#define DRV_VERSION "0.2"
static const char felix_driver_version[] = DRV_VERSION;

#define FELIX_MAX_NUM_PHY_PORTS	5
#define FELIX_EXT_CPU_PORT_ID	4
#define PORT_RES_START		(DEVCPU_PTP + 1)

#define PCI_DEVICE_ID_FELIX_PF5	0xEEF0

/* Switch register block BAR */
#define FELIX_SWITCH_BAR	4

/* pair PCI device */
char *pair_eth = "\0";
module_param(pair_eth, charp, 0000);

static struct pci_device_id felix_ids[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_FREESCALE, PCI_DEVICE_ID_FELIX_PF5) },
	{ 0, }
};
MODULE_DEVICE_TABLE(pci, felix_ids);

/* Mimic the order of ocelot_target */
static struct resource felix_switch_res[] = {
	{
		/* Nothing here */
	},
	{
		.start = 0x0280000,
		.end = 0x028ffff,
		.name = "ana",
		.flags = IORESOURCE_MEM,
	},
	{
		.start = 0x0080000,
		.end = 0x00800ff,
		.name = "qs",
		.flags = IORESOURCE_MEM,
	},
	{
		.start = 0x0200000,
		.end = 0x021ffff,
		.name = "qsys",
		.flags = IORESOURCE_MEM,
	},
	{
		.start = 0x0030000,
		.end = 0x003ffff,
		.name = "rew",
		.flags = IORESOURCE_MEM,
	},
	{
		.start = 0x0010000,
		.end = 0x001ffff,
		.name = "sys",
		.flags = IORESOURCE_MEM,
	},
	{
		.start = 0x0090000,
		.end = 0x00900ff,
		.name = "devcpu_ptp",
		.flags = IORESOURCE_MEM,
	},
	{
		.start = 0x0100000,
		.end = 0x010ffff,
		.name = "port0",
		.flags = IORESOURCE_MEM,
	},
	{
		.start = 0x0110000,
		.end = 0x011ffff,
		.name = "port1",
		.flags = IORESOURCE_MEM,
	},
	{
		.start = 0x0120000,
		.end = 0x012ffff,
		.name = "port2",
		.flags = IORESOURCE_MEM,
	},
	{
		.start = 0x0130000,
		.end = 0x013ffff,
		.name = "port3",
		.flags = IORESOURCE_MEM,
	},
	{
		.start = 0x0140000,
		.end = 0x014ffff,
		.name = "port4",
		.flags = IORESOURCE_MEM,
	},
	{
		.start = 0x0150000,
		.end = 0x015ffff,
		.name = "port5",
		.flags = IORESOURCE_MEM,
	},
};

static void __iomem *regs;

int felix_chip_init(struct ocelot *ocelot);

static inline void felix_set_xfh_field(u32 *efh, u8 bit, u8 w, u16 v)
{
	u8 wi = bit >> 5;
	u8 bi = 32 - (bit - wi * 32 + w);
	u32 val = *(efh + wi);

	v = v & GENMASK(w - 1, 0);
	*(efh + wi) = val | htonl(v << bi);
}

static inline u32 felix_get_xfh_field(u32 *efh, u8 bit, u8 w)
{
	u8 wi = bit >> 5;
	u8 bi = 32 - (bit - wi * 32 + w);

	return (ntohl(*(efh + wi)) >> bi) & GENMASK(w - 1, 0);
}

#define FELIX_IFH_FIELD(name, bit, w) \
static inline void felix_set_ifh_##name(u32 *ifh, u16 v) \
{ \
	felix_set_xfh_field(ifh, bit, w, v); \
}

#define FELIX_EFH_FIELD(name, bit, w) \
static inline u32 felix_get_efh_##name(u32 *efh) \
{ \
	return felix_get_xfh_field(efh, bit, w); \
}

#define FELIX_XFH_LEN 16

/* Felix 128bit-value frame injection header:
 *
 * bit 127: bypass the analyzer processing
 * bit 56-61: destination port mask
 * bit 28-29: pop_cnt: 3 disables all rewriting of the frame
 * bit 20-27: cpu extraction queue mask
 */
FELIX_IFH_FIELD(bypass, 127, 1)
FELIX_IFH_FIELD(dstp, 56, 6)
FELIX_IFH_FIELD(srcp, 43, 4)
FELIX_IFH_FIELD(popcnt, 28, 2)
FELIX_IFH_FIELD(cpuq, 20, 8)

#define FELIX_IFH_INJ_POP_CNT_DISABLE 3

/* Felix 128bit-value frame extraction header */

/* bit 43-45: source port id */
FELIX_EFH_FIELD(srcp, 43, 4)

static void felix_tx_hdr_set(struct sk_buff *skb, struct ocelot_port *port)
{
	u32 *ifh = skb_push(skb, FELIX_XFH_LEN);
	u16 *hdel = skb_push(skb, 2);
	struct ethhdr *ethh = skb_push(skb, ETH_HLEN);

	/* fill frame injection header */
	ether_addr_copy(ethh->h_dest, port->dev->dev_addr);
	eth_zero_addr(ethh->h_source);
	ethh->h_proto = htons(0x8880);
	*hdel = htons(0x000A);

	memset(ifh, 0x0, FELIX_XFH_LEN);
	felix_set_ifh_bypass(ifh, 1);
	felix_set_ifh_dstp(ifh, BIT(port->chip_port));
	felix_set_ifh_srcp(ifh, FELIX_EXT_CPU_PORT_ID);
	felix_set_ifh_popcnt(ifh, FELIX_IFH_INJ_POP_CNT_DISABLE);
	felix_set_ifh_cpuq(ifh, 0xff);
}

static void felix_rx_hdr_extract(struct sk_buff *skb, struct frame_info *info)
{
	char *efh;

	/* ETH_HLEN bytes were already pulled by receivig driver */
	efh = skb_pull(skb, 2);

	info->port = felix_get_efh_srcp((u32 *)efh);
	/* TODO: set traffic class from header */

	/* pull cpu extraction header */
	skb_pull(skb, FELIX_XFH_LEN);
}

static rx_handler_result_t felix_frm_ext_handler(struct sk_buff **pskb)
{
	struct frame_info info = {0};
	struct sk_buff *skb = *pskb;
	struct ocelot_port *port;
	struct net_device *ndev;
	struct ocelot *ocelot;
	int err;

	ndev = skb->dev;
	felix_rx_hdr_extract(skb, &info);

	/* frame for a CPU port */
	if (info.port >= FELIX_EXT_CPU_PORT_ID)
		return RX_HANDLER_PASS;

	ocelot = rcu_dereference(ndev->rx_handler_data);

	port = ocelot->ports[info.port];
	skb->dev = port->dev;
	err = netif_rx(skb);

	return RX_HANDLER_CONSUMED;
}

static netdev_tx_t felix_cpu_inj_handler(struct sk_buff *skb,
					 struct ocelot_port *port)
{
	struct net_device *ndev = port->cpu_inj_handler_data;

	if (!netif_running(ndev)) {
		return NETDEV_TX_BUSY;
	}

	if (unlikely(skb_headroom(skb) < FELIX_XFH_LEN)) {
		struct sk_buff *skb_orig = skb;
		skb = skb_realloc_headroom(skb, FELIX_XFH_LEN);

		/* TODO: free skb in non irq context */
		if (!skb) {
			dev_kfree_skb_any(skb_orig);
			return NETDEV_TX_OK;
		}

		if (skb_orig->sk)
			skb_set_owner_w(skb, skb_orig->sk);

		skb_copy_queue_mapping(skb, skb_orig);
		skb->priority = skb_orig->priority;
#ifdef CONFIG_NET_SCHED
		skb->tc_index = skb_orig->tc_index;
#endif
		dev_consume_skb_any(skb_orig);
	}

	/* add cpu injection header */
	felix_tx_hdr_set(skb, port);

	skb->dev = ndev;
	dev_queue_xmit(skb);

	return NETDEV_TX_OK;
}

static void felix_register_xmit_handler(struct ocelot_port *port,
				      struct net_device *ndev)
{
	struct ocelot *ocelot = port->ocelot;

	/* set packet injection handler */
	port->cpu_inj_handler = &felix_cpu_inj_handler;
	port->cpu_inj_handler_data = ndev;
	/* register rx handler */
	rtnl_lock();
	if (!netdev_is_rx_handler_busy(ndev))
		netdev_rx_handler_register(ndev, felix_frm_ext_handler, ocelot);
	rtnl_unlock();
}

static struct regmap *felix_io_init(struct ocelot *ocelot, u8 target)
{
	void __iomem *target_regs;
	struct regmap_config felix_regmap_config = {
		.reg_bits	= 32,
		.val_bits	= 32,
		.reg_stride	= 4,
	};

	felix_regmap_config.name = felix_switch_res[target].name;
	target_regs = devm_ioremap_resource(ocelot->dev,
					    &felix_switch_res[target]);
	if (IS_ERR(target_regs))
		return ERR_CAST(target_regs);

	return devm_regmap_init_mmio(ocelot->dev, target_regs,
				     &felix_regmap_config);
}

static void felix_release_ports(struct ocelot_port **ports)
{
	struct phy_device *phydev;
	struct net_device *ndev;
	struct device_node *dn;
	int i;

	if (!ports)
		return;

	for (i = 0; i < FELIX_MAX_NUM_PHY_PORTS; i++) {
		if (!ports[i])
			continue;

		phydev = ports[i]->phy;
		if (!phydev)
			continue;

		pr_info("%s: port:%d %s\n", __func__, i,
			phy_is_pseudo_fixed_link(phydev)
			? "fixed-link" : phydev->drv->name);

		unregister_netdev(ports[i]->dev);
		free_netdev(ports[i]->dev);

		if (phy_is_pseudo_fixed_link(phydev)) {
			dn = phydev->mdio.dev.of_node;
			/* decr refcnt: of_phy_register_fixed_link */
			of_phy_deregister_fixed_link(dn);
		}
		phy_device_free(phydev); /* decr refcnt: of_find_phy_device */
		ports[i]->phy = NULL;
		if (ports[i]->cpu_inj_handler_data) {
			ndev = ports[i]->cpu_inj_handler_data;
			rtnl_lock();
			netdev_rx_handler_unregister(ndev);
			rtnl_unlock();
		}
	}
}

static int felix_ports_init(struct ocelot *ocelot)
{
	struct device_node *np = ocelot->dev->of_node;
	struct device_node *phy_node = NULL;
	struct device_node *portnp = NULL;
	struct phy_device *phydev = NULL;
	struct resource *felix_res;
	void __iomem *port_regs;
	struct net_device *ndev;
	u32 port;
	int err;

	ocelot->num_cpu_ports = 1;
	ocelot->cpu_port_id = FELIX_EXT_CPU_PORT_ID;
	ocelot->num_phys_ports = FELIX_MAX_NUM_PHY_PORTS;
	ocelot->ports = devm_kcalloc(ocelot->dev, ocelot->num_phys_ports,
				     sizeof(struct ocelot_port *), GFP_KERNEL);

	/* alloc netdev for each port */
	err = ocelot_init(ocelot);
	if (err)
		return err;

	ndev = NULL;
	if (pair_eth)
		ndev = dev_get_by_name(&init_net, pair_eth);

	for_each_available_child_of_node(np, portnp) {
		if (!portnp || !portnp->name ||
		    of_node_cmp(portnp->name, "port") ||
		    of_property_read_u32(portnp, "reg", &port))
			continue;
		if (port >= FELIX_MAX_NUM_PHY_PORTS) {
			dev_err(ocelot->dev, "invalid port num: %d\n", port);
			continue;
		}
		if (ocelot->ports[port]) {
			dev_warn(ocelot->dev, "port %d already defined\n",
				 port);
			continue;
		}
		felix_res = &felix_switch_res[PORT_RES_START + port];
		port_regs = devm_ioremap_resource(ocelot->dev, felix_res);
		if (IS_ERR(port_regs)) {
			dev_err(ocelot->dev,
				"failed to map registers for port %d\n", port);
			continue;
		}
		if (phy_node) {
			of_node_put(phy_node);
			phy_node = NULL;
		}
		phy_node = of_parse_phandle(portnp, "phy-handle", 0);
		if (!phy_node) {
			if (!of_phy_is_fixed_link(portnp))
				continue;
			err = of_phy_register_fixed_link(portnp);
			if (err < 0) {
				dev_err(ocelot->dev,
					"can't create fixed link for port:%d\n",
					port);
				continue;
			}
			phydev = of_phy_find_device(portnp);
			/* TODO: check if FL phy require phy_start */
		} else {
			phydev = of_phy_find_device(phy_node);
		}
		if (!phydev)
			continue;

		of_node_put(phy_node);
		phy_node = NULL;

		pr_info("%s: port:%d %s\n", __func__, port,
			phy_is_pseudo_fixed_link(phydev)
			? "fixed-link" : phydev->drv->name);

		/* TODO: probe only if its not CPU port */
		err = ocelot_probe_port(ocelot, port, port_regs, phydev);
		if (err) {
			dev_err(ocelot->dev, "failed to probe ports\n");
			goto release_ports;
		}
		/* register xmit handler for external ports */
		if (ndev && port <= FELIX_EXT_CPU_PORT_ID)
			felix_register_xmit_handler(ocelot->ports[port], ndev);
		/* TODO: check if FL phy require phy_start */
		phy_start(phydev);
	}

	return 0;

release_ports:
	felix_release_ports(ocelot->ports);

	return err;
}

static int felix_pci_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	resource_size_t offset;
	struct ocelot *ocelot;
	size_t len;
	int timeout;
	int i, err;

	err = pci_enable_device(pdev);
	if (err) {
		dev_err(&pdev->dev, "device enable failed\n");
		return err;
	}

	/* set up for high or low dma */
	err = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(64));
	if (err) {
		err = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
		if (err) {
			dev_err(&pdev->dev,
				"DMA configuration failed: 0x%x\n", err);
			goto err_dma;
		}
	}

	offset = pci_resource_start(pdev, FELIX_SWITCH_BAR);

	pci_set_master(pdev);

	ocelot = kzalloc(sizeof(*ocelot), GFP_KERNEL);
	if (!ocelot) {
		err = -ENOMEM;
		goto err_alloc_ocelot;
	}

	pci_set_drvdata(pdev, ocelot);
	ocelot->dev = &pdev->dev;

	len = pci_resource_len(pdev, FELIX_SWITCH_BAR);
	if (len == 0) {
		err = -EINVAL;
		goto err_resource_len;
	}

	regs = pci_iomap(pdev, FELIX_SWITCH_BAR, len);
	if (!regs) {
		err = -ENXIO;
		dev_err(&pdev->dev, "ioremap() failed\n");
		goto err_iomap;
	}

	for (i = 0; i < ARRAY_SIZE(felix_switch_res); i++)
		if (felix_switch_res[i].flags == IORESOURCE_MEM) {
			felix_switch_res[i].start += offset;
			felix_switch_res[i].end += offset;
		}

	for (i = ANA; i <= DEVCPU_PTP; i++) {
		struct regmap *target;

		target = felix_io_init(ocelot, i);
		if (IS_ERR(target))
			return PTR_ERR(target);

		ocelot->targets[i] = target;
	}

	err = felix_chip_init(ocelot);
	if (err)
		goto err_chip_init;

	ocelot_write(ocelot, SYS_RAM_INIT_RAM_INIT, SYS_RAM_INIT);

	timeout = 50000;
	while (ocelot_read(ocelot, SYS_RAM_INIT) && --timeout)
		udelay(1); /* busy wait for memory init */
	if (timeout == 0)
		dev_err(&pdev->dev, "Timeout waiting for memory to initialize\n");

	regmap_field_write(ocelot->regfields[SYS_RESET_CFG_CORE_ENA], 1);

	err = felix_ports_init(ocelot);
	if (err)
		goto err_ports_init;

	/* set port for external CPU frame extraction/injection */
	ocelot_write(ocelot, QSYS_EXT_CPU_CFG_EXT_CPUQ_MSK_M |
		     QSYS_EXT_CPU_CFG_EXT_CPU_PORT(FELIX_EXT_CPU_PORT_ID),
		     QSYS_EXT_CPU_CFG);
	/* long prefix frame tagging */
	ocelot_write_rix(ocelot, SYS_PORT_MODE_INCL_XTR_HDR(3) |
			 SYS_PORT_MODE_INCL_INJ_HDR(3), SYS_PORT_MODE,
			 FELIX_EXT_CPU_PORT_ID);

	register_netdevice_notifier(&ocelot_netdevice_nb);

	dev_info(&pdev->dev, "%s - version %s probed\n", felix_driver_string,
		 felix_driver_version);
	return 0;

err_ports_init:
err_chip_init:
	pci_iounmap(pdev, regs);
err_iomap:
err_resource_len:
	kfree(ocelot);
err_alloc_ocelot:
err_dma:
	pci_disable_device(pdev);

	return err;
}

static void felix_pci_remove(struct pci_dev *pdev)
{
	struct ocelot *ocelot;

	ocelot = pci_get_drvdata(pdev);

	/* stop workqueue thread */
	ocelot_deinit(ocelot);

	unregister_netdevice_notifier(&ocelot_netdevice_nb);

	felix_release_ports(ocelot->ports);

	pci_iounmap(pdev, regs);
	kfree(ocelot);
	pci_disable_device(pdev);
	pr_debug("%s - version %s removed\n", felix_driver_string,
		 felix_driver_version);
}

static struct pci_driver felix_pci_driver = {
	.name = "mscc_felix",
	.id_table = felix_ids,
	.probe = felix_pci_probe,
	.remove = felix_pci_remove,
};

module_pci_driver(felix_pci_driver);

MODULE_DESCRIPTION("Felix switch driver");
MODULE_AUTHOR("Razvan Stefanescu <razvan.stefanescu@nxp.com>");
MODULE_LICENSE("Dual MIT/GPL");
