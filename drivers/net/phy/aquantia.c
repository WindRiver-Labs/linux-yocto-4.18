/*
 * Driver for Aquantia PHY
 *
 * Author: Shaohui Xie <Shaohui.Xie@freescale.com>
 *
 * Copyright 2015 Freescale Semiconductor, Inc.
 * Copyright 2018 NXP
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2.  This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/mii.h>
#include <linux/ethtool.h>
#include <linux/phy.h>
#include <linux/mdio.h>

#define PHY_ID_AQ1202	0x03a1b445
#define PHY_ID_AQ2104	0x03a1b460
#define PHY_ID_AQR105	0x03a1b4a2
#define PHY_ID_AQR106	0x03a1b4d0
#define PHY_ID_AQR107	0x03a1b4e0
#define PHY_ID_AQR405	0x03a1b4b0

#define PHY_AQUANTIA_FEATURES	(SUPPORTED_10000baseT_Full | \
				 SUPPORTED_1000baseT_Full | \
				 SUPPORTED_2500baseX_Full | \
				 SUPPORTED_100baseT_Full | \
				 PHY_DEFAULT_FEATURES)

#define MDIO_PMA_CTRL1_AQ_SPEED10	0
#define MDIO_PMA_CTRL1_AQ_SPEED2500	0x2058
#define MDIO_PMA_CTRL1_AQ_SPEED5000	0x205c
#define MDIO_PMA_CTRL2_AQ_2500BT       0x30
#define MDIO_PMA_CTRL2_AQ_5000BT       0x31
#define MDIO_PMA_CTRL2_AQ_TYPE_MASK    0x3F

#define MDIO_AN_VENDOR_PROV_CTRL       0xc400
#define MDIO_AN_RECV_LP_STATUS         0xe820

static int aquantia_pma_setup_forced(struct phy_device *phydev)
{
	int ctrl1, ctrl2, ret;

	/* Half duplex is not supported */
	if (phydev->duplex != DUPLEX_FULL)
		return -EINVAL;

	ctrl1 = phy_read_mmd(phydev, MDIO_MMD_PMAPMD, MDIO_CTRL1);
	if (ctrl1 < 0)
		return ctrl1;

	ctrl2 = phy_read_mmd(phydev, MDIO_MMD_PMAPMD, MDIO_CTRL2);
	if (ctrl2 < 0)
		return ctrl2;

	ctrl1 &= ~MDIO_CTRL1_SPEEDSEL;
	ctrl2 &= ~(MDIO_PMA_CTRL2_AQ_TYPE_MASK);

	switch (phydev->speed) {
	case SPEED_10:
		ctrl2 |= MDIO_PMA_CTRL2_10BT;
		break;
	case SPEED_100:
		ctrl1 |= MDIO_PMA_CTRL1_SPEED100;
		ctrl2 |= MDIO_PMA_CTRL2_100BTX;
		break;
	case SPEED_1000:
		ctrl1 |= MDIO_PMA_CTRL1_SPEED1000;
		/* Assume 1000base-T */
		ctrl2 |= MDIO_PMA_CTRL2_1000BT;
		break;
	case SPEED_10000:
		ctrl1 |= MDIO_CTRL1_SPEED10G;
		/* Assume 10Gbase-T */
		ctrl2 |= MDIO_PMA_CTRL2_10GBT;
		break;
	case SPEED_2500:
		ctrl1 |= MDIO_PMA_CTRL1_AQ_SPEED2500;
		ctrl2 |= MDIO_PMA_CTRL2_AQ_2500BT;
		break;
	case SPEED_5000:
		ctrl1 |= MDIO_PMA_CTRL1_AQ_SPEED5000;
		ctrl2 |= MDIO_PMA_CTRL2_AQ_5000BT;
		break;
	default:
		return -EINVAL;
	}

	ret = phy_write_mmd(phydev, MDIO_MMD_PMAPMD, MDIO_CTRL1, ctrl1);
	if (ret < 0)
		return ret;

	return phy_write_mmd(phydev, MDIO_MMD_PMAPMD, MDIO_CTRL2, ctrl2);
}

static int aquantia_aneg(struct phy_device *phydev, bool control)
{
	int reg = phy_read_mmd(phydev, MDIO_MMD_AN, MDIO_CTRL1);

	if (reg < 0)
		return reg;

	if (control)
		reg |= MDIO_AN_CTRL1_ENABLE | MDIO_AN_CTRL1_RESTART;
	else
		reg &= ~(MDIO_AN_CTRL1_ENABLE | MDIO_AN_CTRL1_RESTART);

	return phy_write_mmd(phydev, MDIO_MMD_AN, MDIO_CTRL1, reg);
}

static int aquantia_config_advert(struct phy_device *phydev)
{
	u32 advertise;
	int oldadv, adv, oldadv1, adv1;
	int err, changed = 0;

	/* Only allow advertising what this PHY supports */
	phydev->advertising &= phydev->supported;
	advertise = phydev->advertising;

	/* Setup standard advertisement */
	oldadv = phy_read_mmd(phydev, MDIO_MMD_AN,
			      MDIO_AN_10GBT_CTRL);
	if (oldadv < 0)
		return oldadv;

	/* Aquantia vendor specific advertisments */
	oldadv1 = phy_read_mmd(phydev, MDIO_MMD_AN,
			       MDIO_AN_VENDOR_PROV_CTRL);
	if (oldadv1 < 0)
		return oldadv1;

	adv  = 0;
	adv1 = 0;

	/*100BaseT_full is supported by default*/

	if (advertise & ADVERTISED_1000baseT_Full)
		adv1 |= 0x8000;
	if (advertise & ADVERTISED_10000baseT_Full)
		adv |= 0x1000;
	if (advertise &  ADVERTISED_2500baseX_Full)
		adv1 |= 0x400;

	if (adv != oldadv) {
		err = phy_write_mmd(phydev, MDIO_MMD_AN,
				    MDIO_AN_10GBT_CTRL, adv);
		if (err < 0)
			return err;
		changed = 1;
	}
	if (adv1 != oldadv1) {
		err = phy_write_mmd(phydev, MDIO_MMD_AN,
				    MDIO_AN_VENDOR_PROV_CTRL, adv1);
		if (err < 0)
			return err;
		changed = 1;
	}

	return changed;
}

static int aquantia_config_aneg(struct phy_device *phydev)
{
	int ret = 0;

	phydev->supported = PHY_AQUANTIA_FEATURES;
	if (phydev->autoneg == AUTONEG_DISABLE) {
		aquantia_pma_setup_forced(phydev);
		return aquantia_aneg(phydev, false);
	}

	ret = aquantia_config_advert(phydev);
	if (ret > 0)
		/* restart autoneg */
		return aquantia_aneg(phydev, true);

	return ret;
}

static int aquantia_config_intr(struct phy_device *phydev)
{
	int err;

	if (phydev->interrupts == PHY_INTERRUPT_ENABLED) {
		err = phy_write_mmd(phydev, MDIO_MMD_AN, 0xd401, 1);
		if (err < 0)
			return err;

		err = phy_write_mmd(phydev, MDIO_MMD_VEND1, 0xff00, 1);
		if (err < 0)
			return err;

		err = phy_write_mmd(phydev, MDIO_MMD_VEND1, 0xff01, 0x1001);
	} else {
		err = phy_write_mmd(phydev, MDIO_MMD_AN, 0xd401, 0);
		if (err < 0)
			return err;

		err = phy_write_mmd(phydev, MDIO_MMD_VEND1, 0xff00, 0);
		if (err < 0)
			return err;

		err = phy_write_mmd(phydev, MDIO_MMD_VEND1, 0xff01, 0);
	}

	return err;
}

static int aquantia_ack_interrupt(struct phy_device *phydev)
{
	int reg;

	reg = phy_read_mmd(phydev, MDIO_MMD_AN, 0xcc01);
	return (reg < 0) ? reg : 0;
}

static int aquantia_read_advert(struct phy_device *phydev)
{
	int adv, adv1;

	/* Setup standard advertisement */
	adv = phy_read_mmd(phydev, MDIO_MMD_AN,
			   MDIO_AN_10GBT_CTRL);

	/* Aquantia vendor specific advertisments */
	adv1 = phy_read_mmd(phydev, MDIO_MMD_AN,
			    MDIO_AN_VENDOR_PROV_CTRL);

	/*100BaseT_full is supported by default*/
	phydev->advertising |= ADVERTISED_100baseT_Full;

	if (adv & 0x1000)
		phydev->advertising |= ADVERTISED_10000baseT_Full;
	else
		phydev->advertising &= ~ADVERTISED_10000baseT_Full;
	if (adv1 & 0x8000)
		phydev->advertising |= ADVERTISED_1000baseT_Full;
	else
		phydev->advertising &= ~ADVERTISED_1000baseT_Full;
	if (adv1 & 0x400)
		phydev->advertising |= ADVERTISED_2500baseX_Full;
	else
		phydev->advertising &= ~ADVERTISED_2500baseX_Full;
	return 0;
}

static int aquantia_read_lp_advert(struct phy_device *phydev)
{
	int adv, adv1;

	/* Read standard link partner advertisement */
	adv = phy_read_mmd(phydev, MDIO_MMD_AN,
			   MDIO_STAT1);

	if (adv & 0x1)
		phydev->lp_advertising |= ADVERTISED_Autoneg |
					  ADVERTISED_100baseT_Full;
	else
		phydev->lp_advertising &= ~(ADVERTISED_Autoneg |
					    ADVERTISED_100baseT_Full);

	/* Read standard link partner advertisement */
	adv = phy_read_mmd(phydev, MDIO_MMD_AN,
			   MDIO_AN_10GBT_STAT);

	/* Aquantia link partner advertisments */
	adv1 = phy_read_mmd(phydev, MDIO_MMD_AN,
			    MDIO_AN_RECV_LP_STATUS);

	if (adv & 0x800)
		phydev->lp_advertising |= ADVERTISED_10000baseT_Full;
	else
		phydev->lp_advertising &= ~ADVERTISED_10000baseT_Full;
	if (adv1 & 0x8000)
		phydev->lp_advertising |= ADVERTISED_1000baseT_Full;
	else
		phydev->lp_advertising &= ~ADVERTISED_1000baseT_Full;
	if (adv1 & 0x400)
		phydev->lp_advertising |= ADVERTISED_2500baseX_Full;
	else
		phydev->lp_advertising &= ~ADVERTISED_2500baseX_Full;

	return 0;
}

static int aquantia_read_status(struct phy_device *phydev)
{
	int reg;

	reg = phy_read_mmd(phydev, MDIO_MMD_AN, MDIO_STAT1);
	if (reg & MDIO_STAT1_LSTATUS)
		phydev->link = 1;
	else
		phydev->link = 0;

	mdelay(10);
	reg = phy_read_mmd(phydev, MDIO_MMD_PMAPMD, MDIO_CTRL1);

	if ((reg & MDIO_CTRL1_SPEEDSELEXT) == MDIO_CTRL1_SPEEDSELEXT)
		reg &= MDIO_CTRL1_SPEEDSEL;
	else
		reg &= MDIO_CTRL1_SPEEDSELEXT;

	switch (reg) {
	case MDIO_PMA_CTRL1_AQ_SPEED5000:
		phydev->speed = SPEED_5000;
		break;
	case MDIO_PMA_CTRL1_AQ_SPEED2500:
		phydev->speed = SPEED_2500;
		break;
	case MDIO_PMA_CTRL1_AQ_SPEED10:
		phydev->speed = SPEED_10;
		break;
	case MDIO_PMA_CTRL1_SPEED100:
		phydev->speed = SPEED_100;
		break;
	case MDIO_PMA_CTRL1_SPEED1000:
		phydev->speed = SPEED_1000;
		break;
	case MDIO_CTRL1_SPEED10G:
		phydev->speed = SPEED_10000;
		break;
	default:
		phydev->speed = SPEED_UNKNOWN;
		break;
	}

	phydev->duplex = DUPLEX_FULL;

	aquantia_read_advert(phydev);
	aquantia_read_lp_advert(phydev);

	return 0;
}

static struct phy_driver aquantia_driver[] = {
{
	.phy_id		= PHY_ID_AQ1202,
	.phy_id_mask	= 0xfffffff0,
	.name		= "Aquantia AQ1202",
	.features	= PHY_AQUANTIA_FEATURES,
	.flags		= PHY_HAS_INTERRUPT,
	.aneg_done	= genphy_c45_aneg_done,
	.config_aneg    = aquantia_config_aneg,
	.config_intr	= aquantia_config_intr,
	.ack_interrupt	= aquantia_ack_interrupt,
	.read_status	= aquantia_read_status,
},
{
	.phy_id		= PHY_ID_AQ2104,
	.phy_id_mask	= 0xfffffff0,
	.name		= "Aquantia AQ2104",
	.features	= PHY_AQUANTIA_FEATURES,
	.flags		= PHY_HAS_INTERRUPT,
	.aneg_done	= genphy_c45_aneg_done,
	.config_aneg    = aquantia_config_aneg,
	.config_intr	= aquantia_config_intr,
	.ack_interrupt	= aquantia_ack_interrupt,
	.read_status	= aquantia_read_status,
},
{
	.phy_id		= PHY_ID_AQR105,
	.phy_id_mask	= 0xfffffff0,
	.name		= "Aquantia AQR105",
	.features	= PHY_AQUANTIA_FEATURES,
	.flags		= PHY_HAS_INTERRUPT,
	.aneg_done	= genphy_c45_aneg_done,
	.config_aneg    = aquantia_config_aneg,
	.config_intr	= aquantia_config_intr,
	.ack_interrupt	= aquantia_ack_interrupt,
	.read_status	= aquantia_read_status,
},
{
	.phy_id		= PHY_ID_AQR106,
	.phy_id_mask	= 0xfffffff0,
	.name		= "Aquantia AQR106",
	.features	= PHY_AQUANTIA_FEATURES,
	.flags		= PHY_HAS_INTERRUPT,
	.aneg_done	= genphy_c45_aneg_done,
	.config_aneg    = aquantia_config_aneg,
	.config_intr	= aquantia_config_intr,
	.ack_interrupt	= aquantia_ack_interrupt,
	.read_status	= aquantia_read_status,
},
{
	.phy_id		= PHY_ID_AQR107,
	.phy_id_mask	= 0xfffffff0,
	.name		= "Aquantia AQR107",
	.features	= PHY_AQUANTIA_FEATURES,
	.flags		= PHY_HAS_INTERRUPT,
	.aneg_done	= genphy_c45_aneg_done,
	.config_aneg    = aquantia_config_aneg,
	.config_intr	= aquantia_config_intr,
	.ack_interrupt	= aquantia_ack_interrupt,
	.read_status	= aquantia_read_status,
},
{
	.phy_id		= PHY_ID_AQR405,
	.phy_id_mask	= 0xfffffff0,
	.name		= "Aquantia AQR405",
	.features	= PHY_AQUANTIA_FEATURES,
	.flags		= PHY_HAS_INTERRUPT,
	.aneg_done	= genphy_c45_aneg_done,
	.config_aneg    = aquantia_config_aneg,
	.config_intr	= aquantia_config_intr,
	.ack_interrupt	= aquantia_ack_interrupt,
	.read_status	= aquantia_read_status,
},
};

module_phy_driver(aquantia_driver);

static struct mdio_device_id __maybe_unused aquantia_tbl[] = {
	{ PHY_ID_AQ1202, 0xfffffff0 },
	{ PHY_ID_AQ2104, 0xfffffff0 },
	{ PHY_ID_AQR105, 0xfffffff0 },
	{ PHY_ID_AQR106, 0xfffffff0 },
	{ PHY_ID_AQR107, 0xfffffff0 },
	{ PHY_ID_AQR405, 0xfffffff0 },
	{ }
};

MODULE_DEVICE_TABLE(mdio, aquantia_tbl);

MODULE_DESCRIPTION("Aquantia PHY driver");
MODULE_AUTHOR("Shaohui Xie <Shaohui.Xie@freescale.com>");
MODULE_LICENSE("GPL v2");
