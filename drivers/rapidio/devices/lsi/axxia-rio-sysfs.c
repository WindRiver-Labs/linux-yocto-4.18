/*
 *   This program is free software;  you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY;  without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See
 *   the GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.
 */

/* #define DEBUG */
/* #define IO_OPERATIONS */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/rio.h>
#include <linux/rio_drv.h>
#include <linux/slab.h>
#include <linux/platform_device.h>

#include "axxia-rio.h"
#include "axxia-rio-irq.h"

static ssize_t axxia_rio_stat_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	struct rio_mport *mport = dev_get_drvdata(dev);
	struct rio_priv *priv = mport->priv;
	char *str = buf;
	u32 reg_val = 0;

	axxia_rio_port_get_state(mport, 0);
	str += sprintf(str, "Master Port state:\n");
	axxia_local_config_read(priv, RIO_ESCSR(priv->port_ndx), &reg_val);
	str += sprintf(str, "ESCSR (0x158) : 0x%08x\n", reg_val);
	return str - buf;
}
static DEVICE_ATTR(stat, S_IRUGO, axxia_rio_stat_show, NULL);

static ssize_t axxia_rio_misc_stat_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	struct rio_mport *mport = dev_get_drvdata(dev);
	struct rio_priv *priv = mport->priv;
	char *str = buf;

	str += sprintf(str, "RIO PIO Stat:\n");
	str += sprintf(str, "\t Successful Count: %d\n",
					priv->rpio_compl_count);
	str += sprintf(str, "\t Failed Count    : %d\n",
					priv->rpio_compl_count);

	str += sprintf(str, "AXI PIO Stat:\n");
	str += sprintf(str, "\t Successful Count: %d\n",
					priv->apio_compl_count);
	str += sprintf(str, "\t Failed Count    : %d\n",
					priv->apio_compl_count);

	str += sprintf(str, "Port Write Stat:\n");
	str += sprintf(str, "\t Interrupt Count : %d\n", priv->rio_pw_count);
	str += sprintf(str, "\t Message Count   : %d\n",
					priv->rio_pw_msg_count);

	return str - buf;

}
static DEVICE_ATTR(misc_stat, S_IRUGO,
		   axxia_rio_misc_stat_show, NULL);
static ssize_t axxia_rio_ib_dme_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	struct rio_mport *mport = dev_get_drvdata(dev);
	struct rio_priv *priv = mport->priv;
	char *str = buf;
	int e, j;
	struct rio_rx_mbox *mb;
	struct rio_msg_dme *me;

	str += sprintf(str, "Inbound Mailbox (DME) counters:\n");
	for (e = 0; e < RIO_MAX_RX_MBOX; e++) {
		mb = priv->ib_mbox[e];
		if (mb) {
			for (j = 0; j < RIO_MSG_MAX_LETTER; j++) {
				me = mb->me[j];
				str += sprintf(str,
					"Mbox %d Letter %d DME %d\n",
					 mb->mbox_no, j, me->dme_no);
				str += sprintf(str,
					"\tNumber of Desc Done  : %d\n",
					me->desc_done_count);
				str += sprintf(str,
					"\tNumber of Desc Errors: %d\n",
					me->desc_error_count);
				str += sprintf(str,
					"\t\tRIO Error    : %d\n",
					me->desc_rio_err_count);
				str += sprintf(str,
					"\t\tAXI Error    : %d\n",
					me->desc_axi_err_count);
				str += sprintf(str,
					"\t\tTimeout Error: %d\n",
					me->desc_tmo_err_count);
			}
		}
	}
	return str - buf;
}
static DEVICE_ATTR(ib_dme_stat, S_IRUGO,
		   axxia_rio_ib_dme_show, NULL);

static ssize_t axxia_rio_ob_dme_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	struct rio_mport *mport = dev_get_drvdata(dev);
	struct rio_priv *priv = mport->priv;
	char *str = buf;
	int e;
	struct rio_msg_dme *me;
	struct rio_tx_mbox *mb;

	str += sprintf(str, "Outbound Message Engine Counters:\n");
	for (e = 0; e < DME_MAX_OB_ENGINES; e++) {
		me = priv->ob_dme_shared[e].me;
		if (me) {
			str += sprintf(str, "DME %d Enabled\n", e);
			str += sprintf(str, "\tNumber of Desc Done  : %d\n",
					me->desc_done_count);
			str += sprintf(str, "\tNumber of Desc Errors: %d\n",
					me->desc_error_count);
			str += sprintf(str, "\t\tRIO Error    : %d\n",
					me->desc_rio_err_count);
			str += sprintf(str, "\t\tAXI Error    : %d\n",
					me->desc_axi_err_count);
			str += sprintf(str, "\t\tTimeout Error: %d\n",
					me->desc_tmo_err_count);
		} else
			str += sprintf(str, "DME %d Disabled\n", e);
	}
	str += sprintf(str, "*********************************\n");
	str += sprintf(str, "Outbound Mbox stats\n");
	for (e = 0; e < RIO_MAX_TX_MBOX; e++) {
		mb = priv->ob_mbox[e];
		if (!mb)
			continue;
		if ((mb->sent_msg_count) || (mb->compl_msg_count)) {
			if (test_bit(RIO_DME_OPEN, &mb->state))
				str += sprintf(str, "Mailbox %d: DME %d\n",
							e, mb->dme_no);
			else
				str += sprintf(str, "Mailbox %d : Closed\n",
							e);
			str += sprintf(str, "\tMessages sent     : %d\n",
						mb->sent_msg_count);
			str += sprintf(str, "\tMessages Completed: %d\n",
						mb->compl_msg_count);
		}
	}

	return str - buf;
}
static DEVICE_ATTR(ob_dme_stat, S_IRUGO,
		   axxia_rio_ob_dme_show, NULL);

static ssize_t axxia_rio_irq_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct rio_mport *mport = dev_get_drvdata(dev);
	struct rio_priv *priv = mport->priv;
	u32 stat;
	char *str = buf;

	str += sprintf(str, "Interrupt enable bits:\n");
	axxia_local_config_read(priv, RAB_INTR_ENAB_GNRL, &stat);
	str += sprintf(str, "General Interrupt Enable (%p)\t%8.8x\n",
		       (void *)RAB_INTR_ENAB_GNRL, stat);
	axxia_local_config_read(priv, RAB_INTR_ENAB_ODME, &stat);
	str += sprintf(str, "Outbound Message Engine  (%p)\t%8.8x\n",
		       (void *)RAB_INTR_ENAB_ODME, stat);
	axxia_local_config_read(priv, RAB_INTR_ENAB_IDME, &stat);
	str += sprintf(str, "Inbound Message Engine   (%p)\t%8.8x\n",
		       (void *)RAB_INTR_ENAB_IDME, stat);
	axxia_local_config_read(priv, RAB_INTR_ENAB_MISC, &stat);
	str += sprintf(str, "Miscellaneous Events     (%p)\t%8.8x\n",
		       (void *)RAB_INTR_ENAB_MISC, stat);
	axxia_local_config_read(priv, RAB_INTR_ENAB_APIO, &stat);
	str += sprintf(str, "Axxia Bus to RIO Events  (%p)\t%8.8x\n",
		       (void *)RAB_INTR_ENAB_APIO, stat);
	axxia_local_config_read(priv, RAB_INTR_ENAB_RPIO, &stat);
	str += sprintf(str, "RIO to Axxia Bus Events  (%p)\t%8.8x\n",
		       (void *)RAB_INTR_ENAB_RPIO, stat);

	str += sprintf(str, "OBDME : in Timer Mode, Period %9.9d nanosecond\n",
			axxia_hrtimer_delay);
	str += sprintf(str, "IBDME : ");
	if (priv->dme_mode == AXXIA_IBDME_TIMER_MODE)
		str += sprintf(str, "in Timer Mode, Period %9.9d nanosecond\n",
			axxia_hrtimer_delay);
	else
		str += sprintf(str, "in Interrupt Mode\n");
	return str - buf;
}
static DEVICE_ATTR(irq, S_IRUGO, axxia_rio_irq_show, NULL);

static ssize_t axxia_rio_tmo_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct rio_mport *mport = dev_get_drvdata(dev);
	struct rio_priv *priv = mport->priv;
	u32 stat;
	char *str = buf;

	str += sprintf(str, "Port Link Timeout Control Registers:\n");
	axxia_local_config_read(priv, RIO_PLTOCCSR, &stat);
	str += sprintf(str, "PLTOCCSR (%p)\t%8.8x\n",
		       (void *)RIO_PLTOCCSR, stat);
	axxia_local_config_read(priv, RIO_PRTOCCSR, &stat);
	str += sprintf(str, "PRTOCCSR (%p)\t%8.8x\n",
		       (void *)RIO_PRTOCCSR, stat);
	axxia_local_config_read(priv, RAB_STAT, &stat);
	str += sprintf(str, "RAB_STAT (%p)\t%8.8x\n",
		       (void *)RAB_STAT, stat);
	axxia_local_config_read(priv, RAB_APIO_STAT, &stat);
	str += sprintf(str, "RAB_APIO_STAT (%p)\t%8.8x\n",
		       (void *)RAB_APIO_STAT, stat);
	axxia_local_config_read(priv, RIO_ESCSR(priv->port_ndx), &stat);
	str += sprintf(str, "PNESCSR (%p)\t%8.8x\n",
		       (void *)RIO_ESCSR(priv->port_ndx), stat);

	return str - buf;
}
static DEVICE_ATTR(tmo, S_IRUGO, axxia_rio_tmo_show, NULL);

static ssize_t axxia_ib_dme_log_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	struct rio_mport *mport = dev_get_drvdata(dev);
	struct rio_priv *priv = mport->priv;
	u32 stat, log;
	char *str = buf;

	axxia_local_config_read(priv, RAB_INTR_STAT_MISC, &stat);
	log = (stat & UNEXP_MSG_LOG) >> 24;
	str += sprintf(str, "mbox[1:0]   %x\n", (log & 0xc0) >> 6);
	str += sprintf(str, "letter[1:0] %x\n", (log & 0x30) >> 4);
	str += sprintf(str, "xmbox[3:0] %x\n", log & 0x0f);

	return str - buf;
}
static DEVICE_ATTR(dme_log, S_IRUGO, axxia_ib_dme_log_show, NULL);

static struct attribute *rio_attributes[] = {
	&dev_attr_stat.attr,
	&dev_attr_irq.attr,
	&dev_attr_misc_stat.attr,
	&dev_attr_ob_dme_stat.attr,
	&dev_attr_ib_dme_stat.attr,
	&dev_attr_tmo.attr,
	&dev_attr_dme_log.attr,
	NULL
};

static struct attribute_group rio_attribute_group = {
	.name = NULL,
	.attrs = rio_attributes,
};

int axxia_rio_init_sysfs(struct platform_device *dev)
{
	return sysfs_create_group(&dev->dev.kobj, &rio_attribute_group);
}
void axxia_rio_release_sysfs(struct platform_device *dev)
{
	sysfs_remove_group(&dev->dev.kobj, &rio_attribute_group);
}
