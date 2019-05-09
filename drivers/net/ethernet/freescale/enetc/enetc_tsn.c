/*
 * Copyright 2017 NXP
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the names of the above-listed copyright holders nor the
 *       names of any contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") as published by the Free Software
 * Foundation, either version 2 of that License or (at your option) any
 * later version.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifdef CONFIG_ENETC_TSN
#include "enetc.h"

#include <net/tsn.h>
#include <linux/module.h>
#include <linux/irqflags.h>
#include <linux/preempt.h>

void DUMP_CBDR(struct enetc_cbd *cbdr)
{
	int i;
	char *data;

	printk("addrl: %04x", cbdr->addr[0]);
	printk("addrh: %04x\n", cbdr->addr[1]);

	data = (char *)cbdr;

	for (i = 0; i < 8; i++) {
		printk("%02x %02x %02x %02x\n",
				*(data + i*4 + 3), *(data + i*4 + 2), *(data + i*4 + 1), *(data + i*4));
	}
	printk("\n");
}

void DUMP_DATA(char *data, int size)
{
	int i;

	printk("data memory: \n");

	for (i = 0; i < size / 4; i++) {
		printk("%02x %02x %02x %02x\n",
				*(data + i*4 + 3), *(data + i*4 + 2), *(data + i*4 + 1), *(data + i*4));
	}

	printk("\n");
}

static int alloc_cbdr(struct enetc_si *si, struct enetc_cbd **curr_cbd)
{
	struct enetc_cbdr *ring = &si->cbd_ring;
	int i;

	i = ring->next_to_use;
	*curr_cbd = ENETC_CBD(*ring, i);
	printk("cbd: %p\n", *curr_cbd);

	memset(*curr_cbd, 0, sizeof(struct enetc_cbd));
	return i;
}

/* Transmit the BD control ring by writing the ccir register.
 * Update the counters maintained by software.
 */
static int xmit_cbdr(struct enetc_si *si, int i)
{
	struct enetc_cbdr *ring = &si->cbd_ring;
	struct enetc_cbd *dest_cbd;
	int nc, timeout;

	i = (i + 1) % ring->bd_count;

	ring->next_to_use = i;
	/* let H/W know BD ring has been updated */
	enetc_wr_reg(ring->cir, i);

	timeout = ENETC_CBDR_TIMEOUT;

	do {
		if (enetc_rd_reg(ring->cisr) == i)
			break;
		udelay(10);
		timeout -= 10;
	} while (timeout);

	if (!timeout)
		return ENETC_CMD_TIMEOUT;
#if 0
	enetc_clean_cbdr(si);
#endif
	nc = ring->next_to_clean;

	while (enetc_rd_reg(ring->cisr) != nc) {
		dest_cbd = ENETC_CBD(*ring, nc);
		if (dest_cbd->status_flags & ENETC_CBD_STATUS_MASK)
			WARN_ON(1);

		/*memset(dest_cbd, 0, sizeof(*dest_cbd));*/

		nc = (nc + 1) % ring->bd_count;
	}

	ring->next_to_clean = nc;

	return ENETC_CMD_OK;
}

/* Class 10: Flow Meter Instance Statistics Query Descriptor - Long Format */
int enetc_qci_fmi_counters_get(struct net_device *ndev, u32 index,
			struct fmi_query_stat_resp *counters)
{
	struct enetc_cbd *cbdr;
	struct fmi_query_stat_resp *fmi_data;
	dma_addr_t dma;
	u16 data_size, dma_size;
	int curr_cbd;
	struct enetc_ndev_priv *priv = netdev_priv(ndev);

	curr_cbd = alloc_cbdr(priv->si, &cbdr);

	cbdr->index = cpu_to_le16((u16)index);
	cbdr->cmd = 2;
	cbdr->cls = BDCR_CMD_FLOW_METER;
	cbdr->status_flags = 0;

	data_size = sizeof(struct fmi_query_stat_resp);

	fmi_data = (struct fmi_query_stat_resp *)kzalloc(data_size, __GFP_DMA | GFP_KERNEL);
	if (fmi_data == NULL)
		return -ENOMEM;

	dma_size = cpu_to_le16(data_size);
	cbdr->length = dma_size;

	dma = dma_map_single(&priv->si->pdev->dev, fmi_data, data_size, DMA_FROM_DEVICE);
	if (dma_mapping_error(&priv->si->pdev->dev, dma)) {
		netdev_err(priv->si->ndev, "DMA mapping failed!\n");
		kfree(fmi_data);
		return -ENOMEM;
	}
	cbdr->addr[0] = lower_32_bits(dma);
	cbdr->addr[1] = upper_32_bits(dma);

	DUMP_CBDR(cbdr);
	xmit_cbdr(priv->si, curr_cbd);
	DUMP_CBDR(cbdr);
	DUMP_DATA((char *)fmi_data, data_size);

	memcpy(counters, fmi_data, sizeof(struct fmi_query_stat_resp));

	memset(cbdr, 0, sizeof(*cbdr));
	kfree(fmi_data);
	return 0;
}

u16 enetc_get_max_gcl_len(struct enetc_hw *hw)
{
	return (enetc_rd(hw, QBV_PTGCAPR_OFFSET) & QBV_MAX_GCL_LEN_MASK);
}

/*
 * CBD Class 5: Time Gated Scheduling Gate Control List configuration
 * Descriptor - Long Format
 */
int enetc_qbv_set(struct net_device *ndev, struct tsn_qbv_conf *admin_conf)
{
	struct enetc_cbd *cbdr;
	struct tgs_gcl_data *gcl_data;
	struct tgs_gcl_conf *gcl_config;
	struct gce *gce;
	u16 gcl_len;
	u16 data_size;
	int i;
	dma_addr_t dma;
	int curr_cbd;
	struct tsn_qbv_basic *admin_basic = &admin_conf->admin;
	struct enetc_ndev_priv *priv = netdev_priv(ndev);

	u32 temp;

	gcl_len = admin_basic->control_list_length;
	if (gcl_len > enetc_get_max_gcl_len(&priv->si->hw))
		return -EINVAL;

	temp = enetc_rd(&priv->si->hw, QBV_PTGCR_OFFSET);
	if (admin_conf->gate_enabled && !(temp & QBV_TGE)) {
		enetc_wr(&priv->si->hw, QBV_PTGCR_OFFSET, temp & (~QBV_TGE));
		udelay(10);
		enetc_wr(&priv->si->hw, QBV_PTGCR_OFFSET, temp | QBV_TGE | QBV_TGDROP_DISABLE);
	} else if (!admin_conf->gate_enabled) {
		enetc_wr(&priv->si->hw, QBV_PTGCR_OFFSET, temp & (~QBV_TGE));
		return 0;
	}

	/*
	 * Set the maximum frame size for each traffic class index
	 * PTCaMSDUR[MAXSDU]. The maximum frame size cannot exceed
	 * 9,600 bytes (0x2580). Frames that exceed the limit are
	 * discarded.
	 */
	if (admin_conf->maxsdu) {
		enetc_wr(&priv->si->hw, ENETC_PTC0MSDUR, admin_conf->maxsdu);
		enetc_wr(&priv->si->hw, ENETC_PTC1MSDUR, admin_conf->maxsdu);
		enetc_wr(&priv->si->hw, ENETC_PTC2MSDUR, admin_conf->maxsdu);
		enetc_wr(&priv->si->hw, ENETC_PTC3MSDUR, admin_conf->maxsdu);
		enetc_wr(&priv->si->hw, ENETC_PTC4MSDUR, admin_conf->maxsdu);
		enetc_wr(&priv->si->hw, ENETC_PTC5MSDUR, admin_conf->maxsdu);
		enetc_wr(&priv->si->hw, ENETC_PTC6MSDUR, admin_conf->maxsdu);
		enetc_wr(&priv->si->hw, ENETC_PTC7MSDUR, admin_conf->maxsdu);
	}

	/*
	 * Configure the (administrative) gate control list using the
	 * control BD descriptor.
	 */
	curr_cbd = alloc_cbdr(priv->si, &cbdr);

	gcl_config = &cbdr->gcl_conf;

	data_size = sizeof(struct tgs_gcl_data) + gcl_len * sizeof(struct gce);

	gcl_data = (struct tgs_gcl_data *)kzalloc(data_size, __GFP_DMA | GFP_KERNEL);
	if (gcl_data == NULL)
		return -ENOMEM;

	gce = (struct gce *)(gcl_data + 1);

	gcl_config->atc = admin_basic->gate_states;
	gcl_config->acl_len = cpu_to_le16(gcl_len);

	if (!admin_basic->base_time) {
		gcl_data->btl = cpu_to_le32(enetc_rd(&priv->si->hw, ENETC_SICTRL));
		gcl_data->bth = cpu_to_le32(enetc_rd(&priv->si->hw, ENETC_SICTRH));
	} else {
		gcl_data->btl = cpu_to_le32(lower_32_bits(admin_basic->base_time));
		gcl_data->bth = cpu_to_le32(upper_32_bits(admin_basic->base_time));
	}

	gcl_data->ct = cpu_to_le32(admin_basic->cycle_time);
	gcl_data->cte = cpu_to_le32(admin_basic->cycle_time_extension);

	for (i = 0; i < gcl_len; i++) {
		struct gce *temp_gce = gce + i;
		struct tsn_qbv_entry *temp_entry = admin_basic->control_list + i;

		temp_gce->gate = temp_entry->gate_state;
		temp_gce->period = cpu_to_le32(temp_entry->time_interval);
	}

	cbdr->length = cpu_to_le16(data_size);
	cbdr->status_flags = 0; /* long format command no ie */

	dma = dma_map_single(&priv->si->pdev->dev, gcl_data, data_size, DMA_TO_DEVICE);
	if (dma_mapping_error(&priv->si->pdev->dev, dma)) {
		netdev_err(priv->si->ndev, "DMA mapping failed!\n");
		kfree(gcl_data);
		return -ENOMEM;
	}

	cbdr->addr[0] = lower_32_bits(dma);
	cbdr->addr[1] = upper_32_bits(dma);
	cbdr->cmd = 0;
	cbdr->cls = BDCR_CMD_PORT_GCL;

	/*
	 * Updated by ENETC on completion of the configuration
	 * command. A zero value indicates success.
	 */
	cbdr->status_flags = 0;

	DUMP_CBDR(cbdr);
	xmit_cbdr(priv->si, curr_cbd);
	 /* config change time could be read in the, but up layer could not get it
	 * */
	DUMP_CBDR(cbdr);
	DUMP_DATA((char *)gcl_data, data_size);
	memset(cbdr, 0, sizeof(struct enetc_cbd));
	dma_unmap_single(&priv->si->pdev->dev, dma, data_size, DMA_TO_DEVICE);
	kfree(gcl_data);

	return 0;
}

/*
 * CBD Class 5: Time Gated Scheduling Gate Control List query
 * Descriptor - Long Format
 */
int enetc_qbv_get(struct net_device *ndev, struct tsn_qbv_conf *admin_conf)
{
	struct enetc_cbd *cbdr;
	struct tgs_gcl_resp *gcl_data;
	struct tgs_gcl_query *gcl_query;
	struct gce *gce;

	struct tsn_qbv_basic *admin_basic = &admin_conf->admin;
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	dma_addr_t dma;
	int curr_cbd;
	u16 maxlen;
	u16 data_size, dma_size;
	u16 admin_len;
	u16 oper_len;
	u64 temp;
	int i;

	curr_cbd = alloc_cbdr(priv->si, &cbdr);

	gcl_query =  &cbdr->gcl_query;

	maxlen = enetc_get_max_gcl_len(&priv->si->hw);

	data_size = sizeof(struct tgs_gcl_resp) + sizeof(struct gce) * 2 * maxlen;

	gcl_data = (struct tgs_gcl_resp *)kzalloc(data_size, __GFP_DMA | GFP_KERNEL);
	if (gcl_data == NULL)
		return -ENOMEM;

	gce = (struct gce *)(gcl_data + 1);

	gcl_query->acl_len = cpu_to_le16(maxlen);

	dma_size = cpu_to_le16(data_size);
	cbdr->length = dma_size;
	cbdr->status_flags = 0; /* long format command no ie */

	dma = dma_map_single(&priv->si->pdev->dev, gcl_data, data_size, DMA_FROM_DEVICE);
	if (dma_mapping_error(&priv->si->pdev->dev, dma)) {
		netdev_err(priv->si->ndev, "DMA mapping failed!\n");
		kfree(gcl_data);
		return -ENOMEM;
	}

	cbdr->addr[0] = lower_32_bits(dma);
	cbdr->addr[1] = upper_32_bits(dma);
	cbdr->cmd = 1;
	cbdr->cls = BDCR_CMD_PORT_GCL;
	DUMP_CBDR(cbdr);
	xmit_cbdr(priv->si, curr_cbd);
	dma_unmap_single(&priv->si->pdev->dev, dma, data_size, DMA_FROM_DEVICE);
	DUMP_CBDR(cbdr);
	DUMP_DATA((char *)gcl_data, data_size);

	/* since cbdr already passed to free, below could be get wrong */
	admin_len = le16_to_cpu(gcl_query->admin_list_len);
	oper_len = le16_to_cpu(gcl_query->oper_list_len);

	admin_basic->control_list_length = admin_len;

	temp = ((u64)le32_to_cpu(gcl_data->abth)) << 32;
	admin_basic->base_time = le32_to_cpu(gcl_data->abtl) + temp;

	admin_basic->cycle_time = le32_to_cpu(gcl_data->act);
	admin_basic->cycle_time_extension = le32_to_cpu(gcl_data->acte);

	admin_basic->control_list =
		kzalloc(admin_len *	sizeof(*(admin_basic->control_list)), GFP_KERNEL);
	if (admin_basic->control_list == NULL) {
		memset(cbdr, 0, sizeof(*cbdr));
		kfree(gcl_data);
		return -ENOMEM;
	}

	for (i = 0; i < admin_len; i++) {
		struct gce *temp_gce = gce + i;
		struct tsn_qbv_entry *temp_entry = admin_basic->control_list + i;

		temp_entry->gate_state = temp_gce->gate;
		temp_entry->time_interval = le32_to_cpu(temp_gce->period);
	}

	if (enetc_rd(&priv->si->hw, QBV_PTGCR_OFFSET) & QBV_TGE)
		admin_conf->gate_enabled = true;
	else
		admin_conf->gate_enabled = false;

	/* Updated by ENETC on completion of the configuration
	 * command. A zero value indicates success.
	 */
	admin_conf->config_change = true;

	memset(cbdr, 0, sizeof(*cbdr));
	kfree(gcl_data);

	return 0;
}

int enetc_qbv_get_status(struct net_device *ndev,
							struct tsn_qbv_status *status)
{
	struct enetc_cbd *cbdr;
	struct tgs_gcl_resp *gcl_data;
	struct tgs_gcl_query *gcl_query;
	struct gce *gce;
	struct tsn_qbv_basic *oper_basic;
	struct enetc_ndev_priv *priv;
	dma_addr_t dma;
	int curr_cbd;
	u16 maxlen;
	u16 data_size, dma_size;
	u16 admin_len;
	u16 oper_len;
	u64 temp;
	int i;

	if (!ndev)
		return -EINVAL;

	if (!status)
		return -EINVAL;

	oper_basic = &status->oper;
	priv = netdev_priv(ndev);

	curr_cbd = alloc_cbdr(priv->si, &cbdr);

	gcl_query = &cbdr->gcl_query;

	maxlen = enetc_get_max_gcl_len(&priv->si->hw);

	data_size = sizeof(struct tgs_gcl_resp) + sizeof(struct gce) * 2 * maxlen;

	gcl_data = (struct tgs_gcl_resp *)kzalloc(data_size, __GFP_DMA | GFP_KERNEL);
	if (gcl_data == NULL)
		return -ENOMEM;

	gce = (struct gce *)(gcl_data + 1);

	gcl_query->acl_len = cpu_to_le16(maxlen);
	gcl_query->ocl_len = cpu_to_le16(maxlen);

	dma_size = cpu_to_le16(data_size);
	cbdr->length = dma_size;
	cbdr->status_flags = 0; /* long format command no ie */

	dma = dma_map_single(&priv->si->pdev->dev, gcl_data, data_size, DMA_FROM_DEVICE);
	if (dma_mapping_error(&priv->si->pdev->dev, dma)) {
		netdev_err(priv->si->ndev, "DMA mapping failed!\n");
		kfree(gcl_data);
		return -ENOMEM;
	}

	cbdr->addr[0] = lower_32_bits(dma);
	cbdr->addr[1] = upper_32_bits(dma);
	cbdr->cmd = 1;
	cbdr->cls = BDCR_CMD_PORT_GCL;
	DUMP_CBDR(cbdr);
	xmit_cbdr(priv->si, curr_cbd);
	dma_unmap_single(&priv->si->pdev->dev, dma, data_size, DMA_FROM_DEVICE);
	DUMP_CBDR(cbdr);
	DUMP_DATA((char *)gcl_data, data_size);

	/* since cbdr already passed to free, below could be get wrong */
	admin_len = le16_to_cpu(gcl_query->admin_list_len);
	oper_len = le16_to_cpu(gcl_query->oper_list_len);

	gce += admin_len;

	if (enetc_rd(&priv->si->hw, QBV_PTGAGLSR_OFFSET) &
						QBV_CFG_PEND_MASK) {
		status->config_pending = true;
		goto exit;
	}

	/* The Oper and Admin timing fields exist in the response buffer even
	 * if no valid corresponding lists exists. These fields are considered
	 * invalid if the corresponding list does not exist.
	 */
	status->config_pending = false;
	temp = ((u64)le32_to_cpu(gcl_data->ccth)) << 32;
	status->config_change_time = le32_to_cpu(gcl_data->cctl) + temp;

	temp = ((u64)le32_to_cpu(gcl_data->cceh)) << 32;
	status->config_change_error = le32_to_cpu(gcl_data->ccel) + temp;

	/* changed to SITGTGR */
	status->tick_granularity = enetc_rd(&priv->si->hw, ENETC_SITGTGR);

	/* current time */
	temp = ((u64)enetc_rd(&priv->si->hw, ENETC_SICTRH)) << 32;
	status->current_time = enetc_rd(&priv->si->hw, ENETC_SICTRL) + temp;

	status->supported_list_max = maxlen;

	/* status->oper.gate_states , no init oper/admin gate state */
	status->oper.control_list_length = oper_len;
	temp = ((u64)le32_to_cpu(gcl_data->obth)) << 32;
	status->oper.base_time = le32_to_cpu(gcl_data->obtl) + temp;
	status->oper.cycle_time = le32_to_cpu(gcl_data->oct);
	status->oper.cycle_time_extension = le32_to_cpu(gcl_data->octe);


	oper_basic->control_list =
		kzalloc(oper_len * sizeof(*(oper_basic->control_list)), GFP_KERNEL);
	if (oper_basic->control_list == NULL) {
		memset(cbdr, 0, sizeof(*cbdr));
		kfree(gcl_data);
		return -ENOMEM;
	}

	for (i = 0; i < oper_len; i++) {
		struct gce *temp_gce = gce + i;
		struct tsn_qbv_entry *temp_entry = oper_basic->control_list + i;

		temp_entry->gate_state = temp_gce->gate;
		temp_entry->time_interval = le32_to_cpu(temp_gce->period);
	}

exit:
	memset(cbdr, 0, sizeof(*cbdr));
	kfree(gcl_data);
	return 0;
}

/* CBD Class 7: Stream Identity Entry Set Descriptor - Long Format */
int enetc_cb_streamid_set(struct net_device *ndev, u32 index,
				bool en, struct tsn_cb_streamid *streamid)
{
	struct enetc_cbd *cbdr;
	void *si_data;
	struct null_streamid_data *si_data1;
	struct smac_streamid_data *si_data2;
	struct streamid_conf *si_conf;
	struct enetc_ndev_priv *priv;
	dma_addr_t dma;
	u16 data_size, dma_size;
	int curr_cbd;
	u64 zero;

	if (!ndev)
		return -EINVAL;

	priv = netdev_priv(ndev);

	curr_cbd = alloc_cbdr(priv->si, &cbdr);

	cbdr->index = cpu_to_le16((u16)index);
	cbdr->cmd = 0;
	cbdr->cls = BDCR_CMD_STREAM_IDENTIFY;
	cbdr->status_flags = 0;

	if (!en) {
		cbdr->length = cpu_to_le16(8);

		zero = 0x8000000000000000;

		dma = dma_map_single(&priv->si->pdev->dev, &zero, 8, DMA_FROM_DEVICE);
		if (dma_mapping_error(&priv->si->pdev->dev, dma)) {
			netdev_err(priv->si->ndev, "DMA mapping failed!\n");
			kfree(si_data);
			return -ENOMEM;
		}

		cbdr->addr[0] = lower_32_bits(dma);
		cbdr->addr[1] = upper_32_bits(dma);

		si_conf = &cbdr->sid_set;
		/* Only one port supported for one entry, set itself */
		si_conf->iports = 1 << (priv->si->pdev->devfn & 0x7);
		si_conf->id_type = 1;
		si_conf->oui[2] = 0x0;
		si_conf->oui[1] = 0x80;
		si_conf->oui[0] = 0xC2;

		xmit_cbdr(priv->si, curr_cbd);
		DUMP_CBDR(cbdr);
		DUMP_DATA((char *)(&zero), 8);
		memset(cbdr, 0, sizeof(*cbdr));
		return 0;
	}

	si_conf = &cbdr->sid_set;
	si_conf->en = 0x80;
	si_conf->stream_handle = cpu_to_le32(streamid->handle);
	si_conf->iports = 1 << (priv->si->pdev->devfn & 0x7);
	si_conf->id_type = streamid->type;
	si_conf->oui[2] = 0x0;
	si_conf->oui[1] = 0x80;
	si_conf->oui[0] = 0xC2;

	if (si_conf->id_type == 1) {
		data_size = sizeof(struct null_streamid_data);
		si_data = (struct null_streamid_data *)kzalloc(data_size, __GFP_DMA | GFP_KERNEL);
	} else if (si_conf->id_type == 2) {
		data_size = sizeof(struct smac_streamid_data);
		si_data = (struct smac_streamid_data *)kzalloc(data_size, __GFP_DMA | GFP_KERNEL);
	} else
		return -EINVAL;

	if (si_data == NULL)
		return -ENOMEM;

	dma_size = cpu_to_le16(data_size);
	cbdr->length = dma_size;
	cbdr->status_flags = 0; /* long format command no ie */

	dma = dma_map_single(&priv->si->pdev->dev, si_data, data_size, DMA_FROM_DEVICE);
	if (dma_mapping_error(&priv->si->pdev->dev, dma)) {
		netdev_err(priv->si->ndev, "DMA mapping failed!\n");
		memset(cbdr, 0, sizeof(*cbdr));
		kfree(si_data);
		return -ENOMEM;
	}
	cbdr->addr[0] = lower_32_bits(dma);
	cbdr->addr[1] = upper_32_bits(dma);

	/* VIDM default to be 1.
	 * VID Match. If set (b1) then the VID must match, otherwise
	 * any VID is considered a match. VIDM setting is only used
	 * when TG is set to b01. */
	if (si_conf->id_type == 1) {
		si_data1 = (struct null_streamid_data *)si_data;
		si_data1->dmac[0] = streamid->para.nid.dmac & 0xFF;
		si_data1->dmac[1] = (streamid->para.nid.dmac >> 8) & 0xFF;
		si_data1->dmac[2] = (streamid->para.nid.dmac >> 16) & 0xFF;
		si_data1->dmac[3] = (streamid->para.nid.dmac >> 24) & 0xFF;
		si_data1->dmac[4] = (streamid->para.nid.dmac >> 32) & 0xFF;
		si_data1->dmac[5] = (streamid->para.nid.dmac >> 40) & 0xFF;
		si_data1->vid_vidm_tg =
		cpu_to_le16((streamid->para.nid.vid & ENETC_CBDR_SID_VID_MASK) +
			((((u16)(streamid->para.nid.tagged) & 0x3) << 14) | ENETC_CBDR_SID_VIDM));
	} else if (si_conf->id_type == 2) {
		si_data2 = (struct smac_streamid_data *)si_data;
		si_data2->smac[0] = streamid->para.sid.smac & 0xFF;
		si_data2->smac[1] = (streamid->para.sid.smac >> 8) & 0xFF;
		si_data2->smac[2] = (streamid->para.sid.smac >> 16) & 0xFF;
		si_data2->smac[3] = (streamid->para.sid.smac >> 24) & 0xFF;
		si_data2->smac[4] = (streamid->para.sid.smac >> 32) & 0xFF;
		si_data2->smac[5] = (streamid->para.sid.smac >> 40) & 0xFF;
		si_data2->vid_vidm_tg =
		cpu_to_le16((streamid->para.sid.vid & ENETC_CBDR_SID_VID_MASK) +
			((((u16)(streamid->para.sid.tagged) & 0x3) << 14) | ENETC_CBDR_SID_VIDM));
	}

	DUMP_CBDR(cbdr);
	xmit_cbdr(priv->si, curr_cbd);
	DUMP_CBDR(cbdr);
	DUMP_DATA((char *)si_data, data_size);

	memset(cbdr, 0, sizeof(*cbdr));
	kfree(si_data);

	return 0;
}

/* CBD Class 7: Stream Identity Entry Query Descriptor - Long Format */
int enetc_cb_streamid_get(struct net_device *ndev, u32 index,
							struct tsn_cb_streamid *streamid)
{
	struct enetc_cbd *cbdr;
	struct streamid_query_resp *si_data;
	struct enetc_ndev_priv *priv;
	dma_addr_t dma;
	u16 data_size, dma_size;
	int curr_cbd;

	if (!ndev)
		return -EINVAL;

	priv = netdev_priv(ndev);

	curr_cbd = alloc_cbdr(priv->si, &cbdr);

	cbdr->index = cpu_to_le32(index);
	cbdr->cmd = 1;
	cbdr->cls = BDCR_CMD_STREAM_IDENTIFY;
	cbdr->status_flags = 0;

	data_size = sizeof(struct streamid_query_resp);
	si_data = (struct streamid_query_resp *)kzalloc(data_size, __GFP_DMA | GFP_KERNEL);
	if (si_data == NULL)
		return -ENOMEM;

	dma_size = cpu_to_le16(data_size);
	cbdr->length = dma_size;
	cbdr->status_flags = 0; /* long format command no ie */

	dma = dma_map_single(&priv->si->pdev->dev, si_data, data_size, DMA_FROM_DEVICE);
	if (dma_mapping_error(&priv->si->pdev->dev, dma)) {
		netdev_err(priv->si->ndev, "DMA mapping failed!\n");
		kfree(si_data);
		return -ENOMEM;
	}
	cbdr->addr[0] = lower_32_bits(dma);
	cbdr->addr[1] = upper_32_bits(dma);

	DUMP_CBDR(cbdr);
	xmit_cbdr(priv->si, curr_cbd);
	DUMP_CBDR(cbdr);
	DUMP_DATA((char *)si_data, data_size);

	streamid->type = si_data->id_type;

	if (streamid->type == 1) {
		streamid->para.nid.dmac = si_data->mac[0] + ((u64)si_data->mac[1] << 8)
			+ ((u64)si_data->mac[2] << 16) + ((u64)si_data->mac[3] << 24)
			+ ((u64)si_data->mac[4] << 32) + ((u64)si_data->mac[5] << 40);
		/* VID Match. If set (b1) then the VID must match, otherwise
		 * any VID is considered a match.
		*/
		/* if (si_data->vid_vidm_tg & ENETC_CBDR_SID_VIDM) */
		streamid->para.nid.vid =
				le16_to_cpu(si_data->vid_vidm_tg & ENETC_CBDR_SID_VID_MASK);
		streamid->para.nid.tagged =
				le16_to_cpu(si_data->vid_vidm_tg >> 14 & 0x3);
	} else if (streamid->type == 2) {
		streamid->para.sid.smac = si_data->mac[0] + ((u64)si_data->mac[1] << 8)
			+ ((u64)si_data->mac[2] << 16) + ((u64)si_data->mac[3] << 24)
			+ ((u64)si_data->mac[4] << 32) + ((u64)si_data->mac[5] << 40);
		/* VID Match. If set (b1) then the VID must match, otherwise
		 * any VID is considered a match.
		 */
		/* if (si_data->vid_vidm_tg & ENETC_CBDR_SID_VIDM) */
		streamid->para.sid.vid =
				le16_to_cpu(si_data->vid_vidm_tg & ENETC_CBDR_SID_VID_MASK);
		streamid->para.sid.tagged =
				le16_to_cpu(si_data->vid_vidm_tg >> 14 & 0x3);
	}

	streamid->handle = le32_to_cpu(si_data->stream_handle);
	streamid->ifac_iport = le32_to_cpu(si_data->input_ports);

	memset(cbdr, 0, sizeof(*cbdr));
	kfree(si_data);

	return 0;
}

/*  CBD Class 7: Stream Identity Statistics Query Descriptor - Long Format */
int enetc_cb_streamid_counters_get(struct net_device *ndev, u32 index,
				struct tsn_cb_streamid_counters *counters)
{
	return 0;
}

void enetc_qci_enable(struct enetc_hw *hw)
{
	enetc_wr(hw, ENETC_PPSFPMR, enetc_rd(hw, ENETC_PPSFPMR)
					| ENETC_PPSFPMR_PSFPEN | ENETC_PPSFPMR_VS
					| ENETC_PPSFPMR_PVC | ENETC_PPSFPMR_PVZC);
}

void enetc_qci_disable(struct enetc_hw *hw)
{
	enetc_wr(hw, ENETC_PPSFPMR, enetc_rd(hw, ENETC_PPSFPMR)
					& ~ENETC_PPSFPMR_PSFPEN & ~ENETC_PPSFPMR_VS
					& ~ENETC_PPSFPMR_PVC & ~ENETC_PPSFPMR_PVZC);
}

/* CBD Class 8: Stream Filter Instance Set Descriptor - Short Format */
int enetc_qci_sfi_set(struct net_device *ndev, u32 index, bool en,
		struct tsn_qci_psfp_sfi_conf *tsn_qci_sfi)
{
	struct enetc_cbd *cbdr;
	struct sfi_conf *sfi_config;

	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	int curr_cbd;

	curr_cbd = alloc_cbdr(priv->si, &cbdr);

	cbdr->index = cpu_to_le16(index);
	cbdr->cmd = 0;
	cbdr->cls = BDCR_CMD_STREAM_FILTER;
	cbdr->status_flags = 0x80;
	cbdr->length = cpu_to_le16(1);

	sfi_config = &cbdr->sfi_conf;
	if (en)
		sfi_config->en = 0x80;

	if (tsn_qci_sfi->stream_handle_spec >= 0) {
		sfi_config->stream_handle =
			cpu_to_le32(tsn_qci_sfi->stream_handle_spec);
		sfi_config->sthm |= 0x80;
	}

	sfi_config->sg_inst_table_index =
		cpu_to_le16(tsn_qci_sfi->stream_gate_instance_id);
	sfi_config->input_ports = 1 << (priv->si->pdev->devfn & 0x7);

	/* The priority value which may be matched against the
	 * frame’s priority value to determine a match for this entry.
	 */
	if (tsn_qci_sfi->priority_spec >= 0)
		sfi_config->multi |= (tsn_qci_sfi->priority_spec & 0x7) | 0x8;

	/* Filter Type. Identifies the contents of the MSDU/FM_INST_INDEX
	 * field as being either an MSDU value or an index into the Flow
	 * Meter Instance table.
	 */
	if (tsn_qci_sfi->stream_filter.maximum_sdu_size != 0) {
		sfi_config->msdu =
		cpu_to_le16(tsn_qci_sfi->stream_filter.maximum_sdu_size);
		sfi_config->multi |= 0x40;
	}

	if (tsn_qci_sfi->stream_filter.flow_meter_instance_id >= 0) {
		sfi_config->fm_inst_table_index =
		cpu_to_le16(tsn_qci_sfi->stream_filter.flow_meter_instance_id);
		sfi_config->multi |= 0x80;
	}

	/* Stream blocked due to oversized frame enable. TRUE or FALSE */
	if (tsn_qci_sfi->block_oversize_enable)
		sfi_config->multi |= 0x20;

	/* Stream blocked due to oversized frame. TRUE or FALSE */
	if (tsn_qci_sfi->block_oversize)
		sfi_config->multi |= 0x10;

	DUMP_CBDR(cbdr);
	xmit_cbdr(priv->si, curr_cbd);
	DUMP_CBDR(cbdr);

	memset(cbdr, 0, sizeof(*cbdr));
	return 0;
}

/* CBD Class 8: Stream Filter Instance Query Descriptor - Short Format */
int enetc_qci_sfi_get(struct net_device *ndev, u32 index,
						struct tsn_qci_psfp_sfi_conf *tsn_qci_sfi)
{
	struct enetc_cbd *cbdr;
	struct sfi_conf *sfi_config;
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	int curr_cbd;

	curr_cbd = alloc_cbdr(priv->si, &cbdr);

	cbdr->index = cpu_to_le16(index);
	cbdr->cmd = 1;
	cbdr->cls = BDCR_CMD_STREAM_FILTER;
	cbdr->status_flags = 0x80;

	xmit_cbdr(priv->si, curr_cbd);
	DUMP_CBDR(cbdr);

	sfi_config = &cbdr->sfi_conf;
	if (sfi_config->sthm & 0x80)
		tsn_qci_sfi->stream_handle_spec =
			le32_to_cpu(sfi_config->stream_handle);
	else
		tsn_qci_sfi->stream_handle_spec = -1;

	tsn_qci_sfi->stream_gate_instance_id =
		le16_to_cpu(sfi_config->sg_inst_table_index);

	if (sfi_config->multi & 0x8)
		tsn_qci_sfi->priority_spec = le16_to_cpu(sfi_config->multi & 0x7);
	else
		tsn_qci_sfi->priority_spec = -1;

	/* Filter Type. Identifies the contents of the MSDU/FM_INST_INDEX
	 * field as being either an MSDU value or an index into the Flow
	 * Meter Instance table.
	 */
	if (sfi_config->multi & 0x80)
		tsn_qci_sfi->stream_filter.flow_meter_instance_id =
			le16_to_cpu(sfi_config->fm_inst_table_index);
	else
		tsn_qci_sfi->stream_filter.flow_meter_instance_id = -1;

	if (sfi_config->multi & 0x40)
		tsn_qci_sfi->stream_filter.maximum_sdu_size =
			le16_to_cpu(sfi_config->msdu);

	/* Stream blocked due to oversized frame enable. TRUE or FALSE */
	if (sfi_config->multi & 0x20)
		tsn_qci_sfi->block_oversize_enable = true;
	/* Stream blocked due to oversized frame. TRUE or FALSE */
	if (sfi_config->multi & 0x10)
		tsn_qci_sfi->block_oversize = true;

	if (sfi_config->en & 0x80) {
		memset(cbdr, 0, sizeof(*cbdr));
		return 1;
	}

	memset(cbdr, 0, sizeof(*cbdr));
	return 0;
}

/* CBD Class 8: Stream Filter Instance Query Statistics
 * Descriptor - Long Format
 */
int enetc_qci_sfi_counters_get(struct net_device *ndev, u32 index,
							struct tsn_qci_psfp_sfi_counters *counters)
{
	struct enetc_cbd *cbdr;
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	int curr_cbd;
	struct sfi_counter_data *sfi_counter_data;
	dma_addr_t dma;
	u16 data_size, dma_size;

	curr_cbd = alloc_cbdr(priv->si, &cbdr);

	cbdr->index = cpu_to_le16((u16)index);
	cbdr->cmd = 2;
	cbdr->cls = BDCR_CMD_STREAM_FILTER;
	cbdr->status_flags = 0;

	data_size = sizeof(struct sfi_counter_data);
	sfi_counter_data = (struct sfi_counter_data *)kzalloc(data_size, __GFP_DMA | GFP_KERNEL);
	if (sfi_counter_data == NULL)
		return -ENOMEM;

	dma = dma_map_single(&priv->si->pdev->dev, sfi_counter_data, data_size, DMA_FROM_DEVICE);
	if (dma_mapping_error(&priv->si->pdev->dev, dma)) {
		netdev_err(priv->si->ndev, "DMA mapping failed!\n");
		kfree(sfi_counter_data);
		return -ENOMEM;
	}
	cbdr->addr[0] = lower_32_bits(dma);
	cbdr->addr[1] = upper_32_bits(dma);

	dma_size = cpu_to_le16(data_size);
	cbdr->length = dma_size;

	DUMP_CBDR(cbdr);
	xmit_cbdr(priv->si, curr_cbd);
	DUMP_CBDR(cbdr);
	DUMP_DATA((char *)sfi_counter_data, data_size);

	counters->matching_frames_count =
			((u64)le32_to_cpu(sfi_counter_data->matchh) << 32)
			+ sfi_counter_data->matchl;

	counters->not_passing_sdu_count =
			((u64)le32_to_cpu(sfi_counter_data->msdu_droph) << 32)
			+ sfi_counter_data->msdu_dropl;

	counters->passing_sdu_count = counters->matching_frames_count
				- counters->not_passing_sdu_count;

	counters->not_passing_frames_count = ((u64)le32_to_cpu(sfi_counter_data->stream_gate_droph) << 32)
				+ le32_to_cpu(sfi_counter_data->stream_gate_dropl);

	counters->passing_frames_count = counters->matching_frames_count
				- counters->not_passing_sdu_count
				- counters->not_passing_frames_count;

	counters->red_frames_count = ((u64)le32_to_cpu(sfi_counter_data->flow_meter_droph) << 32)
				+ le32_to_cpu(sfi_counter_data->flow_meter_dropl);

	memset(cbdr, 0, sizeof(*cbdr));
	return 0;
}

/*
 * CBD Class 9: Stream Gate Instance Table Entry Set
 * Descriptor - Short Format
 */
int enetc_qci_sgi_set(struct net_device *ndev, u32 index,
				struct tsn_qci_psfp_sgi_conf *tsn_qci_sgi)
{
	struct enetc_cbd *cbdr, *cbdr_sgcl;
	struct sgi_table *sgi_config;
	struct sgcl_conf *sgcl_config;
	struct sgcl_data *sgcl_data;
	struct sgce *sgce;
	struct enetc_ndev_priv *priv = netdev_priv(ndev);

	dma_addr_t dma;
	u16 data_size, dma_size;
	int curr_cbd, i;

	/* disable first */
	curr_cbd = alloc_cbdr(priv->si, &cbdr);
	memset(cbdr, 0, sizeof(*cbdr));

	cbdr->index = cpu_to_le16(index);
	cbdr->cmd = 0;
	cbdr->cls = BDCR_CMD_STREAM_GCL;
	cbdr->status_flags = 0x80;
	DUMP_CBDR(cbdr);
	xmit_cbdr(priv->si, curr_cbd);
	DUMP_CBDR(cbdr);
	if (!tsn_qci_sgi->gate_enabled) {
		memset(cbdr, 0, sizeof(*cbdr));
		return 0;
	}

	/* Re-enable */
	curr_cbd = alloc_cbdr(priv->si, &cbdr);
	memset(cbdr, 0, sizeof(*cbdr));

	cbdr->index = cpu_to_le16(index);
	cbdr->cmd = 0;
	cbdr->cls = BDCR_CMD_STREAM_GCL;
	cbdr->status_flags = 0x80;

	sgi_config = &cbdr->sgi_table;

	sgi_config->ocgtst = tsn_qci_sgi->admin.control_list_length ?
			0x80 : (tsn_qci_sgi->admin.gate_states ? 0x80 : 0x0);

	sgi_config->oipv = tsn_qci_sgi->admin.control_list_length ?
			0x0 : ((tsn_qci_sgi->admin.init_ipv < 0) ?
					0x0 : ((tsn_qci_sgi->admin.init_ipv & 0x7) | 0x8));

	sgi_config->en = 0x80;

	if (tsn_qci_sgi->block_invalid_rx_enable)
		sgi_config->gset |= 0x80;
	if (tsn_qci_sgi->block_invalid_rx)
		sgi_config->gset |= 0x40;
	if (tsn_qci_sgi->block_octets_exceeded)
		sgi_config->gset |= 0x10;
	if (tsn_qci_sgi->block_octets_exceeded_enable)
		sgi_config->gset |= 0x20;

	DUMP_CBDR(cbdr);
	xmit_cbdr(priv->si, curr_cbd);
	DUMP_CBDR(cbdr);

	if (tsn_qci_sgi->admin.control_list_length == 0)
		goto exit;

	curr_cbd = alloc_cbdr(priv->si, &cbdr_sgcl);
	memset(cbdr, 0, sizeof(*cbdr));

	cbdr_sgcl->index = cpu_to_le16(index);
	cbdr_sgcl->cmd = 1;
	cbdr_sgcl->cls = BDCR_CMD_STREAM_GCL;
	cbdr_sgcl->status_flags = 0;

	sgcl_config = &cbdr_sgcl->sgcl_conf;

	/* tsn_qci_sgi->admin.control_list_length is not zero now */
	if (tsn_qci_sgi->admin.control_list_length > 4)
		return -EINVAL;
	else
		sgcl_config->acl_len = (tsn_qci_sgi->admin.control_list_length - 1) & 0x3;

	data_size = sizeof(struct sgcl_data) +
		(sgcl_config->acl_len + 1) * sizeof(struct sgce);

	sgcl_data = (struct sgcl_data *)kzalloc(data_size, __GFP_DMA | GFP_KERNEL);
	if (sgcl_data == NULL)
		return -ENOMEM;

	dma_size = cpu_to_le16(data_size);
	cbdr_sgcl->length = dma_size;

	dma = dma_map_single(&priv->si->pdev->dev, sgcl_data, data_size, DMA_FROM_DEVICE);
	if (dma_mapping_error(&priv->si->pdev->dev, dma)) {
		netdev_err(priv->si->ndev, "DMA mapping failed!\n");
		memset(cbdr, 0, sizeof(*cbdr));
		memset(cbdr_sgcl, 0, sizeof(*cbdr_sgcl));
		kfree(sgcl_data);
		return -ENOMEM;
	}
	cbdr_sgcl->addr[0] = lower_32_bits(dma);
	cbdr_sgcl->addr[1] = upper_32_bits(dma);

	sgce = (struct sgce *)(sgcl_data + 1);

	if (tsn_qci_sgi->admin.gate_states)
		sgcl_config->agtst = 0x80;

	sgcl_data->ct = cpu_to_le32(tsn_qci_sgi->admin.cycle_time);
	sgcl_data->cte = cpu_to_le32(tsn_qci_sgi->admin.cycle_time_extension);

	if (tsn_qci_sgi->admin.init_ipv >= 0)
		sgcl_config->aipv = (tsn_qci_sgi->admin.init_ipv & 0x7) | 0x8;

	for (i = 0; i < tsn_qci_sgi->admin.control_list_length; i++) {
		struct tsn_qci_psfp_gcl *temp_sgcl = tsn_qci_sgi->admin.gcl + i;
		struct sgce *temp_entry = (struct sgce *)(sgce + i);

		if (temp_sgcl->gate_state)
			temp_entry->multi |= 0x10;

		if (temp_sgcl->ipv >= 0)
			temp_entry->multi |= ((temp_sgcl->ipv & 0x7) << 5) | 0x08;

		if (temp_sgcl->octet_max)
			temp_entry->multi |= 0x01;

		temp_entry->interval = cpu_to_le32(temp_sgcl->time_interval);
		temp_entry->msdu[0] = temp_sgcl->octet_max & 0xFF;
		temp_entry->msdu[1] = (temp_sgcl->octet_max >> 8) & 0xFF;
		temp_entry->msdu[2] = (temp_sgcl->octet_max >> 16) & 0xFF;
	}

	if (!tsn_qci_sgi->admin.base_time) {
		sgcl_data->btl = cpu_to_le32(enetc_rd(&priv->si->hw, ENETC_SICTRL));
		sgcl_data->bth = cpu_to_le32(enetc_rd(&priv->si->hw, ENETC_SICTRH));
	} else {
		sgcl_data->bth = cpu_to_le32(upper_32_bits(tsn_qci_sgi->admin.base_time));
		sgcl_data->btl = cpu_to_le32(lower_32_bits(tsn_qci_sgi->admin.base_time));
	}

	DUMP_CBDR(cbdr_sgcl);
	DUMP_DATA((char *)sgcl_data, data_size);
	xmit_cbdr(priv->si, curr_cbd);
	DUMP_CBDR(cbdr_sgcl);
	DUMP_DATA((char *)sgcl_data, data_size);

	memset(cbdr_sgcl, 0, sizeof(*cbdr_sgcl));
	kfree(sgcl_data);

exit:
	memset(cbdr, 0, sizeof(*cbdr));
	return 0;
}

/* CBD Class 9: Stream Gate Instance Table Entry Query
 * Descriptor - Short Format
 */
int enetc_qci_sgi_get(struct net_device *ndev, u32 index,
				struct tsn_qci_psfp_sgi_conf *tsn_qci_sgi)
{
	struct enetc_cbd *cbdr, *cbdr_sgcl;
	struct sgi_table *sgi_config;
	struct sgcl_query *sgcl_query;
	struct sgcl_query_resp *sgcl_data;
	struct sgce *sgce;
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	dma_addr_t dma;
	u16 data_size, dma_size, gcl_data_stat = 0;
	u8 admin_len = 0;
	int curr_cbd, i;

	curr_cbd = alloc_cbdr(priv->si, &cbdr);

	cbdr->index = cpu_to_le16(index);
	cbdr->cmd = 2;
	cbdr->cls = BDCR_CMD_STREAM_GCL;
	cbdr->status_flags = 0x80;

	DUMP_CBDR(cbdr);
	xmit_cbdr(priv->si, curr_cbd);
	DUMP_CBDR(cbdr);

	sgi_config = &cbdr->sgi_table;

	tsn_qci_sgi->admin.gate_states = ((sgi_config->ocgtst & 0x80) ? true : false);
	if (sgi_config->oipv & 0x08)
		tsn_qci_sgi->admin.init_ipv = sgi_config->oipv & 0x7;
	else
		tsn_qci_sgi->admin.init_ipv = -1;

	if (sgi_config->en & 0x80)
		tsn_qci_sgi->gate_enabled = true;
	if (sgi_config->gset & 0x80)
		tsn_qci_sgi->block_invalid_rx_enable = true;
	if (sgi_config->gset & 0x40)
		tsn_qci_sgi->block_invalid_rx = true;
	if (sgi_config->gset & 0x20)
		tsn_qci_sgi->block_octets_exceeded_enable = true;
	if (sgi_config->gset & 0x10)
		tsn_qci_sgi->block_octets_exceeded = true;

	/* Check gate list length is zero? */
	if (!(sgi_config->oacl_len & 0x30)) {
		tsn_qci_sgi->admin.control_list_length = 0;
		goto exit;
	}

	curr_cbd = alloc_cbdr(priv->si, &cbdr_sgcl);

	cbdr_sgcl->index = cpu_to_le16(index);
	cbdr_sgcl->cmd = 3;
	cbdr_sgcl->cls = BDCR_CMD_STREAM_GCL;
	cbdr_sgcl->status_flags = 0;

	data_size = sizeof(struct sgcl_query_resp) + 4 * sizeof(struct sgce); /* Max is 4 */

	sgcl_data = (struct sgcl_query_resp *)kzalloc(data_size, __GFP_DMA | GFP_KERNEL);
	if (sgcl_data == NULL)
		return -ENOMEM;

	dma_size = cpu_to_le16(data_size);
	cbdr_sgcl->length = dma_size;
	cbdr_sgcl->status_flags = 0;

	sgcl_query = &cbdr_sgcl->sgcl_query;

	sgcl_query->oacl_len = 0x10;

	dma = dma_map_single(&priv->si->pdev->dev, sgcl_data, data_size, DMA_FROM_DEVICE);
	if (dma_mapping_error(&priv->si->pdev->dev, dma)) {
		netdev_err(priv->si->ndev, "DMA mapping failed!\n");
		memset(cbdr, 0, sizeof(*cbdr));
		memset(cbdr_sgcl, 0, sizeof(*cbdr_sgcl));
		kfree(sgcl_data);
		return -ENOMEM;
	}
	cbdr_sgcl->addr[0] = lower_32_bits(dma);
	cbdr_sgcl->addr[1] = upper_32_bits(dma);

	DUMP_CBDR(cbdr_sgcl);
	xmit_cbdr(priv->si, curr_cbd);
	DUMP_CBDR(cbdr_sgcl);
	DUMP_DATA((char *)sgcl_data, data_size);

	sgce = (struct sgce *)(sgcl_data + 1);

	gcl_data_stat = le16_to_cpu(sgcl_data->stat);
	if (gcl_data_stat & 0x10)
		tsn_qci_sgi->admin.gate_states = true;

	if (gcl_data_stat & 0x80)
		tsn_qci_sgi->admin.init_ipv = gcl_data_stat & 0x7;
	else
		tsn_qci_sgi->admin.init_ipv = -1;

	/* admin_len can also get from gcl_data_stat bit 5,6 OR sgi_config->oacl_len */
	admin_len = (sgcl_query->oacl_len & 0x3) + 1;
	tsn_qci_sgi->admin.control_list_length = admin_len;
	tsn_qci_sgi->admin.cycle_time = le32_to_cpu(sgcl_data->act);
	tsn_qci_sgi->admin.cycle_time_extension = le32_to_cpu(sgcl_data->acte);
	tsn_qci_sgi->admin.base_time = ((u64)(le32_to_cpu(sgcl_data->abth)) << 32)
						+ le32_to_cpu(sgcl_data->abtl);

	tsn_qci_sgi->admin.gcl =
		kzalloc(admin_len * sizeof(struct tsn_qci_psfp_gcl), GFP_KERNEL);
	if (tsn_qci_sgi->admin.gcl == NULL) {
		kfree(sgcl_data);
		return -ENOMEM;
	}

	for (i = 0; i < admin_len; i++) {
		struct tsn_qci_psfp_gcl *temp_sgcl = tsn_qci_sgi->admin.gcl + i;
		struct sgce *temp_entry = (struct sgce *)(sgce + i);

		if (temp_entry->multi & 0x10)
			temp_sgcl->gate_state = true;

		if (temp_entry->multi & 0x08)
			temp_sgcl->ipv = temp_entry->multi >> 5;
		else
			temp_sgcl->ipv = -1;

		temp_sgcl->time_interval = le32_to_cpu(temp_entry->interval);

		if (temp_entry->multi & 0x01)
			temp_sgcl->octet_max = (temp_entry->msdu[0] & 0xff)
							| (((u32)temp_entry->msdu[1] << 8) & 0xff00)
							| (((u32)temp_entry->msdu[1] << 16) & 0xff0000);
		else
			temp_sgcl->octet_max = 0;
	}

	memset(cbdr_sgcl, 0, sizeof(*cbdr_sgcl));
	kfree(sgcl_data);

exit:
	memset(cbdr, 0, sizeof(*cbdr));
	return 0;
}

/* CBD Class 9: Stream Gate Instance Table Entry Query Descriptor - Short Format */
/* CBD Class 9: Stream Gate Control List Query Descriptor - Long Format */
int enetc_qci_sgi_status_get(struct net_device *ndev, u16 index,
				struct tsn_psfp_sgi_status *status)
{
	struct enetc_cbd *cbdr_sgi, *cbdr_sgcl;
	struct sgi_table *sgi_config;
	struct sgcl_query *sgcl_query;
	struct sgcl_query_resp *sgcl_data;
	struct sgce *sgce;
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	dma_addr_t dma;
	u16 data_size, dma_size, gcl_data_stat = 0;
	u8 oper_len = 0;
	u64 temp;
	int curr_cbd, i;

	curr_cbd = alloc_cbdr(priv->si, &cbdr_sgi);

	cbdr_sgi->index = cpu_to_le16(index);
	cbdr_sgi->cmd = 2;
	cbdr_sgi->cls = BDCR_CMD_STREAM_GCL;
	cbdr_sgi->status_flags = 0x80;

	sgi_config = &cbdr_sgi->sgi_table;

	if (sgi_config->gset & 0x4)
		status->config_pending = true;

	status->oper.gate_states = ((sgi_config->ocgtst & 0x80) ? true : false);

	/* Check gate list length is zero? */
	if (!(sgi_config->oacl_len & 0x30)) {
		status->oper.control_list_length = 0;
		goto cmd2quit;
	}

	DUMP_CBDR(cbdr_sgi);
	xmit_cbdr(priv->si, curr_cbd);
	DUMP_CBDR(cbdr_sgi);

	curr_cbd = alloc_cbdr(priv->si, &cbdr_sgcl);

	cbdr_sgcl->index = cpu_to_le16(index);
	cbdr_sgcl->cmd = 3;
	cbdr_sgcl->cls = BDCR_CMD_STREAM_GCL;
	cbdr_sgcl->status_flags = 0;

	data_size = sizeof(struct sgcl_query_resp) + 4 * sizeof(struct sgce); /* Max is 4 */

	sgcl_data = (struct sgcl_query_resp *)kzalloc(data_size, __GFP_DMA | GFP_KERNEL);
	if (sgcl_data == NULL)
		return -ENOMEM;

	dma_size = cpu_to_le16(data_size);
	cbdr_sgcl->length = dma_size;
	cbdr_sgcl->status_flags = 0;

	sgcl_query = &cbdr_sgcl->sgcl_query;

	sgcl_query->oacl_len = 0x20;

	dma = dma_map_single(&priv->si->pdev->dev, sgcl_data, data_size, DMA_FROM_DEVICE);
	if (dma_mapping_error(&priv->si->pdev->dev, dma)) {
		netdev_err(priv->si->ndev, "DMA mapping failed!\n");
		memset(cbdr_sgi, 0, sizeof(*cbdr_sgi));
		memset(cbdr_sgcl, 0, sizeof(*cbdr_sgcl));
		kfree(sgcl_data);
		return -ENOMEM;
	}
	cbdr_sgcl->addr[0] = lower_32_bits(dma);
	cbdr_sgcl->addr[1] = upper_32_bits(dma);

	DUMP_CBDR(cbdr_sgcl);
	xmit_cbdr(priv->si, curr_cbd);
	DUMP_CBDR(cbdr_sgcl);
	DUMP_DATA((char *)sgcl_data, data_size);

	sgce = (struct sgce *)(sgcl_data + 1);

	/* oper_len can also get from gcl_data_stat bit 5,6 OR sgi_config->oacl_len */
	oper_len = ((sgcl_query->oacl_len & 0x0c) >> 2) + 1;

	/* Get Stream Gate Control List */
	status->oper.cycle_time = le32_to_cpu(sgcl_data->oct);
	status->oper.cycle_time_extension = le32_to_cpu(sgcl_data->octe);
	status->oper.base_time = le32_to_cpu(sgcl_data->obtl) + ((u64)le32_to_cpu(sgcl_data->obth) << 32);
	status->oper.control_list_length = oper_len;

	gcl_data_stat = le16_to_cpu(sgcl_data->stat);
	if (gcl_data_stat & 0x400)
		status->oper.init_ipv = gcl_data_stat & 0x38 >> 7;
	else
		status->oper.init_ipv = -1;

	if (gcl_data_stat & 0x800)
		status->oper.gate_states = true;

	status->oper.gcl =
		kzalloc(oper_len * sizeof(struct tsn_qci_psfp_gcl), GFP_KERNEL);
	if (status->oper.gcl == NULL) {
		memset(cbdr_sgi, 0, sizeof(*cbdr_sgi));
		memset(cbdr_sgcl, 0, sizeof(*cbdr_sgcl));
		kfree(sgcl_data);
		return -ENOMEM;
	}

	for (i = 0; i < oper_len; i++) {
		struct tsn_qci_psfp_gcl *temp_sgcl = status->oper.gcl + i;
		struct sgce *temp_entry = (struct sgce *)(sgce + i);

		if (temp_entry->multi & 0x10)
			temp_sgcl->gate_state = true;

		if (temp_entry->multi & 0x08)
			temp_sgcl->ipv = temp_entry->multi >> 5;
		else
			temp_sgcl->ipv = -1;

		temp_sgcl->time_interval = le32_to_cpu(temp_entry->interval);

		if (temp_entry->multi & 0x01)
			temp_sgcl->octet_max = temp_entry->msdu[0]
					| ((((u32)temp_entry->msdu[1]) << 8) & 0xff00)
					| ((((u32)temp_entry->msdu[2]) << 16) & 0xff0000);
		else
			temp_sgcl->octet_max = 0;
	}

	status->config_change_time = le32_to_cpu(sgcl_data->cctl) + ((u64)le32_to_cpu(sgcl_data->ccth) << 32);

	memset(cbdr_sgcl, 0, sizeof(*cbdr_sgcl));
	kfree(sgcl_data);

cmd2quit:
	/* changed to SITGTGR */
	status->tick_granularity = enetc_rd(&priv->si->hw, ENETC_SITGTGR);

	/* current time */
	temp = ((u64)enetc_rd(&priv->si->hw, ENETC_SICTRH)) << 32;
	status->current_time = enetc_rd(&priv->si->hw, ENETC_SICTRL) + temp;

	memset(cbdr_sgi, 0, sizeof(*cbdr_sgi));

	return 0;
}

/* CBD Class 10: Flow Meter Instance Set Descriptor - Short Format */
int enetc_qci_fmi_set(struct net_device *ndev, u32 index, bool enable,
				struct tsn_qci_psfp_fmi *tsn_qci_fmi)
{
	struct enetc_cbd *cbdr;
	struct fmi_conf *fmi_config;

	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	int curr_cbd;
	u64 temp = 0;

	curr_cbd = alloc_cbdr(priv->si, &cbdr);

	cbdr->index = cpu_to_le16((u16)index);
	cbdr->cmd = 0;
	cbdr->cls = BDCR_CMD_FLOW_METER;
	cbdr->status_flags = 0x80;
	DUMP_CBDR(cbdr);
	xmit_cbdr(priv->si, curr_cbd);
	DUMP_CBDR(cbdr);
	if (!enable) {
		memset(cbdr, 0, sizeof(*cbdr));
		return 0;
	}

	/* Re-enable */
	curr_cbd = alloc_cbdr(priv->si, &cbdr);
	memset(cbdr, 0, sizeof(*cbdr));
	cbdr->index = cpu_to_le16((u16)index);
	cbdr->cmd = 0;
	cbdr->cls = BDCR_CMD_FLOW_METER;
	cbdr->status_flags = 0x80;

	fmi_config = &cbdr->fmi_conf;
	fmi_config->en = 0x80;
	if (tsn_qci_fmi->cir) {
		temp = (u64)1000 * tsn_qci_fmi->cir;
		temp = temp / 3725;
	}
	fmi_config->cir = cpu_to_le32((u32)temp);
	fmi_config->cbs = cpu_to_le32(tsn_qci_fmi->cbs);
	temp = 0;
	if (tsn_qci_fmi->eir) {
		temp = (u64)1000 * tsn_qci_fmi->eir;
		temp = temp / 3725;
	}
	fmi_config->eir = cpu_to_le32((u32)temp);
	fmi_config->ebs = cpu_to_le32(tsn_qci_fmi->ebs);

	if (tsn_qci_fmi->mark_red)
		fmi_config->conf |= 0x1;

	if (tsn_qci_fmi->mark_red_enable)
		fmi_config->conf |= 0x2;

	if (tsn_qci_fmi->drop_on_yellow)
		fmi_config->conf |= 0x4;

	if (tsn_qci_fmi->cm)
		fmi_config->conf |= 0x8;

	if (tsn_qci_fmi->cf)
		fmi_config->conf |= 0x10;

	DUMP_CBDR(cbdr);
	xmit_cbdr(priv->si, curr_cbd);
	DUMP_CBDR(cbdr);

	memset(cbdr, 0, sizeof(*cbdr));
	return 0;
}

/* CBD Class 10: Flow Meter Instance Query Descriptor - Short Format */
int enetc_qci_fmi_get(struct net_device *ndev, u32 index,
						struct tsn_qci_psfp_fmi *tsn_qci_fmi)
{
	struct enetc_cbd *cbdr;
	struct fmi_conf *fmi_config;
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	int curr_cbd;
	u16 data_size, dma_size;
	dma_addr_t dma;
	struct fmi_query_stat_resp *fmi_counter_data;
	u64 temp = 0;

	curr_cbd = alloc_cbdr(priv->si, &cbdr);

	cbdr->index = cpu_to_le16(index);
	cbdr->cmd = 1;
	cbdr->cls = BDCR_CMD_FLOW_METER;
	cbdr->status_flags = 0x80;

	DUMP_CBDR(cbdr);
	xmit_cbdr(priv->si, curr_cbd);
	DUMP_CBDR(cbdr);

	fmi_config = &cbdr->fmi_conf;
	if (fmi_config->cir) {
		temp = (u64)3725 * fmi_config->cir;
		temp = temp / 1000;
	}
	tsn_qci_fmi->cir = le32_to_cpu((u32)temp);
	tsn_qci_fmi->cbs = le32_to_cpu(fmi_config->cbs);
	temp = 0;
	if (fmi_config->eir) {
		temp = (u64)3725 * fmi_config->eir;
		temp = temp / 1000;
	}
	tsn_qci_fmi->eir = le32_to_cpu((u32)temp);
	tsn_qci_fmi->ebs = le32_to_cpu(fmi_config->ebs);

	if (fmi_config->conf & 0x1)
		tsn_qci_fmi->mark_red = true;

	if (fmi_config->conf & 0x2)
		tsn_qci_fmi->mark_red_enable = true;

	if (fmi_config->conf & 0x4)
		tsn_qci_fmi->drop_on_yellow = true;

	if (fmi_config->conf & 0x8)
		tsn_qci_fmi->cm = true;

	if (fmi_config->conf & 0x10)
		tsn_qci_fmi->cf = true;

	memset(cbdr, 0, sizeof(*cbdr));

	/* Get counters */
	curr_cbd = alloc_cbdr(priv->si, &cbdr);

	cbdr->index = cpu_to_le16(index);
	cbdr->cmd = 2;
	cbdr->cls = BDCR_CMD_FLOW_METER;
	cbdr->status_flags = 0x0;

	data_size = sizeof(struct fmi_query_stat_resp);
	fmi_counter_data = (struct fmi_query_stat_resp *)kzalloc(data_size, __GFP_DMA | GFP_KERNEL);
	if (fmi_counter_data == NULL)
		return -ENOMEM;

	dma = dma_map_single(&priv->si->pdev->dev, fmi_counter_data, data_size, DMA_FROM_DEVICE);
	if (dma_mapping_error(&priv->si->pdev->dev, dma)) {
		netdev_err(priv->si->ndev, "DMA mapping failed!\n");
		kfree(fmi_counter_data);
		return -ENOMEM;
	}
	cbdr->addr[0] = lower_32_bits(dma);
	cbdr->addr[1] = upper_32_bits(dma);

	dma_size = cpu_to_le16(data_size);
	cbdr->length = dma_size;

	DUMP_CBDR(cbdr);
	xmit_cbdr(priv->si, curr_cbd);
	DUMP_CBDR(cbdr);
	DUMP_DATA((char *)fmi_counter_data, data_size);

	return 0;
}

int enetc_qbu_set(struct net_device *ndev, u8 ptvector)
{
	u32 temp;
	int i;
	struct enetc_ndev_priv *priv = netdev_priv(ndev);

	temp = enetc_rd(&priv->si->hw, QBV_PTGCR_OFFSET);
	if (temp & QBV_TGE)
		enetc_wr(&priv->si->hw, QBV_PTGCR_OFFSET, temp & (~QBV_TGPE));

	for (i = 0; i < 8; i++) {
		/* 1 Enabled. Traffic is transmitted on the preemptive MAC. */
		temp = enetc_port_rd(&priv->si->hw, ENETC_PTCFPR(i));

		if ((ptvector >> i) & 0x1)
			enetc_port_wr(&priv->si->hw, ENETC_PTCFPR(i), temp | ENETC_FPE);
		else
			enetc_port_wr(&priv->si->hw, ENETC_PTCFPR(i), temp & ~ENETC_FPE);
	}

	return 0;
}

int enetc_qbu_get(struct net_device *ndev,
		  struct tsn_preempt_status *preemptstat)
{
	int i;
	struct enetc_ndev_priv *priv = netdev_priv(ndev);

	if (enetc_port_rd(&priv->si->hw, ENETC_PFPMR) & ENETC_PFPMR_PMACE) {
		preemptstat->preemption_active = true;
		if (enetc_rd(&priv->si->hw, QBV_PTGCR_OFFSET) & QBV_TGE)
			preemptstat->hold_request = 1;
		else
			preemptstat->hold_request = 2;
	} else {
		preemptstat->preemption_active = false;
		return 0;
	}

	for (i = 0; i < 8; i++)
		if (enetc_port_rd(&priv->si->hw, ENETC_PTCFPR(i)) & 0x80000000)
			preemptstat->admin_state |= 1 << i;

	preemptstat->hold_advance =
		enetc_rd(&priv->si->hw, QBV_PTGCR_OFFSET) & 0xFFFF;
	preemptstat->release_advance =
		enetc_rd(&priv->si->hw, QBV_PTGCR_OFFSET) & 0xFFFF;

	return 0;
}

u32 __enetc_tsn_get_cap(struct enetc_si *si)
{
	u32 reg = 0;
	u32 cap = 0;

	reg = enetc_port_rd(&si->hw, ENETC_PCAPR0);

	if (reg & ENETC_PCAPR0_PSFP)
		cap |= TSN_CAP_QCI;
	else if (reg & ENETC_PCAPR0_TSN)
		cap |= TSN_CAP_QBV;
	else if (reg & ENETC_PCAPR0_QBU)
		cap |= TSN_CAP_QBU;

	cap |= TSN_CAP_CBS;
	cap |= TSN_CAP_TBS;

	return cap;
}

u32 enetc_tsn_get_capability(struct net_device *ndev)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);

	return __enetc_tsn_get_cap(priv->si);
}

static int enetc_set_cbs(struct net_device *ndev, u8 tc, u8 bw)
{
	return 0;
}

static int enetc_get_cbs(struct net_device *ndev, u8 tc)
{
	return 0;
}

#define GET_CURRENT_TIME(si) (enetc_rd(&(si)->hw, ENETC_SICTR0) \
							| ((u64)enetc_rd(&(si)->hw, ENETC_SICTR1) << 32))

static int enetc_set_tsd(struct net_device *ndev, struct tsn_tsd *ttsd)
{
	return 0;
}

static int enetc_get_tsd(struct net_device *ndev, struct tsn_tsd_status *tts)
{
	return 0;
}

static struct tsn_ops enetc_tsn_ops = {
	.get_capability = enetc_tsn_get_capability,
	.qbv_set = enetc_qbv_set,
	.qbv_get = enetc_qbv_get,
	.qbv_get_status = enetc_qbv_get_status,
	.cb_streamid_set = enetc_cb_streamid_set,
	.cb_streamid_get = enetc_cb_streamid_get,
	.cb_streamid_counters_get = enetc_cb_streamid_counters_get,
	.qci_sfi_set = enetc_qci_sfi_set,
	.qci_sfi_get = enetc_qci_sfi_get,
	.qci_sfi_counters_get = enetc_qci_sfi_counters_get,
	.qci_sgi_set = enetc_qci_sgi_set,
	.qci_sgi_get = enetc_qci_sgi_get,
	.qci_sgi_status_get = enetc_qci_sgi_status_get,
	.qci_fmi_set = enetc_qci_fmi_set,
	.qci_fmi_get = enetc_qci_fmi_get,
	.qbu_set = enetc_qbu_set,
	.qbu_get = enetc_qbu_get,
	.cbs_set = enetc_set_cbs,
	.cbs_get = enetc_get_cbs,
	.tsd_set = enetc_set_tsd,
	.tsd_get = enetc_get_tsd,
};
/*
static u32 get_ndev_speed(struct net_device *netdev)
{
	struct ethtool_link_ksettings ksettings;
	int rc = -1;
	if (netdev->ethtool_ops->get_link_ksettings) {

		if (netdev->ethtool_ops->begin) {
			if ((rc = netdev->ethtool_ops->begin(netdev) < 0))
				return 0;
		}

		memset(&ksettings, 0, sizeof(ksettings));

		if (!netdev->ethtool_ops->get_link_ksettings)
			return 0;

		rc = netdev->ethtool_ops->get_link_ksettings(netdev, &ksettings);

		if (netdev->ethtool_ops->complete)
			netdev->ethtool_ops->complete(netdev);
	}
	return (rc < 0) ? 0 : ksettings.base.speed;
}
*/
static void enetc_cbs_init(struct enetc_si *si)
{
	return;
}

static void enetc_qbv_init(struct enetc_hw *hw)
{
	/* Set PSPEED to be 1Gbps */
	enetc_port_wr(hw, ENETC_PMR, (enetc_port_rd(hw, ENETC_PMR) & (~0xf00)) | 0x200);
}

void enetc_tsn_init(struct enetc_si *si)
{
	u32 capability = 0;

	si->ndev->tsn_ops = &enetc_tsn_ops;

	capability = __enetc_tsn_get_cap(si);

	if (capability & TSN_CAP_CBS)
		enetc_cbs_init(si);

	if (capability & TSN_CAP_QBV)
		enetc_qbv_init(&si->hw);

	if (capability & TSN_CAP_QCI)
		enetc_qci_enable(&si->hw);

	dev_info(&si->pdev->dev, "%s: setup done\n", __func__);
}
#endif	/* #if IS_ENABLED(CONFIG_ENETC_TSN) */
