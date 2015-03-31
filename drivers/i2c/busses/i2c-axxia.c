/*
 * drivers/i2c/busses/i2c-axxia.c
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/module.h>

#define SCL_WAIT_TIMEOUT_NS 25000000
#define I2C_XFER_TIMEOUT    (msecs_to_jiffies(500))
#define I2C_STOP_TIMEOUT    (msecs_to_jiffies(100))
#define TX_FIFO_SIZE        8
#define RX_FIFO_SIZE        8

struct i2c_regs {
	__le32 global_control;
	__le32 interrupt_status;
	__le32 interrupt_enable;
	__le32 wait_timer_control;
	__le32 ibml_timeout;
	__le32 ibml_low_mext;
	__le32 ibml_low_sext;
	__le32 timer_clock_div;
	__le32 i2c_bus_monitor;
	__le32 soft_reset;
	__le32 mst_command;
#define CMD_MANUAL 0x08
#define CMD_AUTO   0x09
	__le32 mst_rx_xfer;
	__le32 mst_tx_xfer;
	__le32 mst_addr_1;
	__le32 mst_addr_2;
	__le32 mst_data;
	__le32 mst_tx_fifo;
	__le32 mst_rx_fifo;
	__le32 mst_int_enable;
	__le32 mst_int_status;
#define MST_STATUS_RFL (1<<13) /* RX FIFO serivce */
#define MST_STATUS_TFL (1<<12) /* TX FIFO service */
#define MST_STATUS_SNS (1<<11) /* Manual mode done */
#define MST_STATUS_SS  (1<<10) /* Automatic mode done */
#define MST_STATUS_SCC (1<<9)  /* Stop complete */
#define MST_STATUS_IP  (1<<8)  /* Invalid parameter */
#define MST_STATUS_TSS (1<<7)  /* Timeout */
#define MST_STATUS_AL  (1<<6)  /* Arbitration lost */
#define MST_STATUS_NAK (MST_STATUS_NA | MST_STATUS_ND)
#define MST_STATUS_ND  (1<<5)  /* NAK on data phase */
#define MST_STATUS_NA  (1<<4)  /* NAK on address phase */
#define MST_STATUS_ERR (MST_STATUS_NAK | \
			MST_STATUS_AL  | \
			MST_STATUS_IP  | \
			MST_STATUS_TSS)
	__le32 mst_tx_bytes_xfrd;
	__le32 mst_rx_bytes_xfrd;
	__le32 slv_addr_dec_ctl;
	__le32 slv_addr_1;
	__le32 slv_addr_2;
	__le32 slv_rx_ctl;
	__le32 slv_data;
	__le32 slv_rx_fifo;
	__le32 slv_int_enable;
	__le32 slv_int_status;
	__le32 slv_read_dummy;
	__le32 reserved;
	__le32 scl_high_period;
	__le32 scl_low_period;
	__le32 spike_fltr_len;
	__le32 sda_setup_time;
	__le32 sda_hold_time;
	__le32 smb_alert;
	__le32 udid_w7;
	__le32 udid_w6;
	__le32 udid_w5;
	__le32 udid_w4;
	__le32 udid_w3;
	__le32 udid_w2;
	__le32 udid_w1;
	__le32 udid_w0;
	__le32 arppec_cfg_stat;
	__le32 slv_arp_int_enable;
	__le32 slv_arp_int_status;
	__le32 mst_arp_int_enable;
	__le32 mst_arp_int_status;
};


/**
 * I2C device context
 */
struct axxia_i2c_dev {
	/** device reference */
	struct device *dev;
	/** core i2c abstraction */
	struct i2c_adapter adapter;
	/* clock reference for i2c input clock */
	struct clk *i2c_clk;
	/* pointer to registers */
	struct i2c_regs __iomem *regs;
	/* xfer completion object */
	struct completion msg_complete;
	/* pointer to current message */
	struct i2c_msg *msg;
	/* number of bytes transferred in msg */
	size_t msg_xfrd;
	/* error code for completed message */
	int msg_err;
	/* IRQ number (or 0 if not using interrupt) */
	int irq;
	/* current i2c bus clock rate */
	u32 bus_clk_rate;
};

static void
i2c_int_disable(struct axxia_i2c_dev *idev, u32 mask)
{
	u32 int_en;

	int_en = readl(idev->base + MST_INT_ENABLE);
	writel(int_en & ~mask, idev->base + MST_INT_ENABLE);
}

static void
i2c_int_enable(struct axxia_i2c_dev *idev, u32 mask)
{
	u32 int_mask = readl(&idev->regs->mst_int_enable);

	int_mask |= mask;
	writel(int_mask, &idev->regs->mst_int_enable);
}

/**
 * Convert nanoseconds to clock cycles for the given clock frequency.
 */
static u32
ns_to_clk(u64 ns, u32 clk_mhz)
{
	return div_u64(ns*clk_mhz, 1000);
}

static int
axxia_i2c_init(struct axxia_i2c_dev *idev)
{
	u32 divisor = clk_get_rate(idev->i2c_clk) / idev->bus_clk_rate;
	u32 clk_mhz = clk_get_rate(idev->i2c_clk) / 1000000;
	u32 t_setup;
	u32 t_high, t_low;
	u32 tmo_clk;
	u32 prescale;
	unsigned long timeout;

	dev_dbg(idev->dev, "rate=%uHz per_clk=%uMHz -> ratio=1:%u\n",
		idev->bus_clk_rate, clk_mhz, divisor);

	/* Reset controller */
	writel(0x01, &idev->regs->soft_reset);
	timeout = jiffies + msecs_to_jiffies(100);
	while (readl(&idev->regs->soft_reset) & 1) {
		if (time_after(jiffies, timeout)) {
			dev_warn(idev->dev, "Soft reset failed\n");
			break;
		}
	}

	/* Enable Master Mode */
	writel(0x1, &idev->regs->global_control);

	if (idev->bus_clk_rate <= 100000) {
		/* Standard mode SCL 50/50, tSU:DAT = 250 ns */
		t_high  = divisor*1/2;
		t_low   = divisor*1/2;
		t_setup = ns_to_clk(250, clk_mhz);
	} else {
		/* Fast mode SCL 33/66, tSU:DAT = 100 ns */
		t_high  = divisor*1/3;
		t_low   = divisor*2/3;
		t_setup = ns_to_clk(100, clk_mhz);
	}

	/* SCL High Time */
	writel(t_high, &idev->regs->scl_high_period);
	/* SCL Low Time */
	writel(t_low, &idev->regs->scl_low_period);
	/* SDA Setup Time */
	writel(t_setup, &idev->regs->sda_setup_time);
	/* SDA Hold Time, 300ns */
	writel(ns_to_clk(300, clk_mhz), &idev->regs->sda_hold_time);
	/* Filter <50ns spikes */
	writel(ns_to_clk(50, clk_mhz), &idev->regs->spike_fltr_len);

	/* Configure Time-Out Registers */
	tmo_clk = ns_to_clk(SCL_WAIT_TIMEOUT_NS, clk_mhz);

	/*
	   Find the prescaler value that makes tmo_clk fit in 15-bits counter.
	 */
	for (prescale = 0; prescale < 15; ++prescale) {
		if (tmo_clk <= 0x7fff)
			break;
		tmo_clk >>= 1;
	}
	if (tmo_clk > 0x7fff)
		tmo_clk = 0x7fff;

	/* Prescale divider (log2) */
	writel(prescale, &idev->regs->timer_clock_div);
	/* Timeout in divided clocks */
	writel((1<<15) | tmo_clk, &idev->regs->wait_timer_control);

	/* Mask all master interrupt bits */
	i2c_int_disable(idev, ~0);

	/* Interrupt enable */
	writel(0x01, &idev->regs->interrupt_enable);

	dev_dbg(idev->dev, "SDA_SETUP:        %08x\n",
		readl(&idev->regs->sda_setup_time));
	dev_dbg(idev->dev, "SDA_HOLD:         %08x\n",
		readl(&idev->regs->sda_hold_time));
	dev_dbg(idev->dev, "SPIKE_FILTER_LEN: %08x\n",
		readl(&idev->regs->spike_fltr_len));
	dev_dbg(idev->dev, "TIMER_DIV:        %08x\n",
		readl(&idev->regs->timer_clock_div));
	dev_dbg(idev->dev, "WAIT_TIMER:       %08x\n",
		readl(&idev->regs->wait_timer_control));

	return 0;
}

static int
i2c_m_rd(const struct i2c_msg *msg)
{
	return (msg->flags & I2C_M_RD) != 0;
}

static int
i2c_m_ten(const struct i2c_msg *msg)
{
	return (msg->flags & I2C_M_TEN) != 0;
}

static int
i2c_m_recv_len(const struct i2c_msg *msg)
{
	return (msg->flags & I2C_M_RECV_LEN) != 0;
}

static int
axxia_i2c_empty_rx_fifo(struct axxia_i2c_dev *idev)
{
	struct i2c_msg *msg = idev->msg;
	size_t rx_fifo_avail = readl(&idev->regs->mst_rx_fifo);
	int bytes_to_transfer = min(rx_fifo_avail, msg->len - idev->msg_xfrd);

	while (0 < bytes_to_transfer--) {
		int c = readl(&idev->regs->mst_data);

		if (idev->msg_xfrd == 0 && i2c_m_recv_len(msg)) {
			/*
			 * Check length byte for SMBus block read
			 */
			if (c <= 0) {
				idev->msg_err = -EPROTO;
				i2c_int_disable(idev, ~0);
				complete(&idev->msg_complete);
				break;
			} else if (c > I2C_SMBUS_BLOCK_MAX) {
				c = I2C_SMBUS_BLOCK_MAX;
			}
			msg->len = 1 + c;
			writel(msg->len, &idev->regs->mst_rx_xfer);
		}
		msg->buf[idev->msg_xfrd++] = c;
	}

	return 0;
}

static int
axxia_i2c_fill_tx_fifo(struct axxia_i2c_dev *idev)
{
	struct i2c_msg *msg = idev->msg;
	size_t tx_fifo_avail = TX_FIFO_SIZE - readl(&idev->regs->mst_tx_fifo);
	int bytes_to_transfer = min(tx_fifo_avail, msg->len - idev->msg_xfrd);

	while (0 < bytes_to_transfer--)
		writel(msg->buf[idev->msg_xfrd++], &idev->regs->mst_data);

	return 0;
}

static char *
status_str(u32 status)
{
	static char buf[128];

	buf[0] = '\0';

	if (status & MST_STATUS_RFL)
		strcat(buf, "RFL ");
	if (status & MST_STATUS_TFL)
		strcat(buf, "TFL ");
	if (status & MST_STATUS_SNS)
		strcat(buf, "SNS ");
	if (status & MST_STATUS_SS)
		strcat(buf, "SS ");
	if (status & MST_STATUS_SCC)
		strcat(buf, "SCC ");
	if (status & MST_STATUS_TSS)
		strcat(buf, "TSS ");
	if (status & MST_STATUS_AL)
		strcat(buf, "AL ");
	if (status & MST_STATUS_ND)
		strcat(buf, "ND ");
	if (status & MST_STATUS_NA)
		strcat(buf, "NA ");
	return buf;
}

static void
axxia_i2c_service_irq(struct axxia_i2c_dev *idev)
{
	u32 status = readl(&idev->regs->mst_int_status);

	/* RX FIFO needs service? */
	if (i2c_m_rd(idev->msg) && (status & MST_STATUS_RFL))
		axxia_i2c_empty_rx_fifo(idev);

	/* TX FIFO needs service? */
	if (!i2c_m_rd(idev->msg) && (status & MST_STATUS_TFL)) {
		if (idev->msg_xfrd < idev->msg->len)
			axxia_i2c_fill_tx_fifo(idev);
		else
			i2c_int_disable(idev, MST_STATUS_TFL);
	}

	if (status & MST_STATUS_SCC) {
		/* Stop completed? */
		i2c_int_disable(idev, ~0);
		complete(&idev->msg_complete);
	} else if (status & (MST_STATUS_SNS | MST_STATUS_SS)) {
		/* Transfer done? */
		if (i2c_m_rd(idev->msg) && idev->msg_xfrd < idev->msg->len)
			axxia_i2c_empty_rx_fifo(idev);
		i2c_int_disable(idev, ~0);
		complete(&idev->msg_complete);
	} else if (unlikely(status & MST_STATUS_ERR)) {
		/* Transfer error? */
		idev->msg_err = status & MST_STATUS_ERR;
		i2c_int_disable(idev, ~0);
		dev_dbg(idev->dev, "error %s, rx=%u/%u tx=%u/%u\n",
			status_str(status),
			readl(&idev->regs->mst_rx_bytes_xfrd),
			readl(&idev->regs->mst_rx_xfer),
			readl(&idev->regs->mst_tx_bytes_xfrd),
			readl(&idev->regs->mst_tx_xfer));
		complete(&idev->msg_complete);
	}
}

static irqreturn_t
axxia_i2c_isr(int irq, void *_dev)
{
	struct axxia_i2c_dev *idev = _dev;

	if ((readl(&idev->regs->interrupt_status) & 0x1) == 0)
		return IRQ_NONE;

	if (!idev->msg)
		return IRQ_NONE;

	axxia_i2c_service_irq(idev);

	/* Clear interrupt */
	writel(0x01, &idev->regs->interrupt_status);

	return IRQ_HANDLED;
}


static int
axxia_i2c_xfer_msg(struct axxia_i2c_dev *idev, struct i2c_msg *msg)
{
	u32 int_mask = MST_STATUS_ERR | MST_STATUS_SNS;
	u32 addr_1, addr_2;
	int ret;

	if (msg->len == 0 || msg->len > 255)
		return -EINVAL;

	idev->msg      = msg;
	idev->msg_xfrd = 0;
	idev->msg_err =  0;
	reinit_completion(&idev->msg_complete);

	if (i2c_m_rd(msg)) {
		/* TX 0 bytes */
		writel(0, &idev->regs->mst_tx_xfer);
		/* RX # bytes */
		if (i2c_m_recv_len(msg))
			writel(I2C_SMBUS_BLOCK_MAX, &idev->regs->mst_rx_xfer);
		else
			writel(msg->len, &idev->regs->mst_rx_xfer);
	} else {
		/* TX # bytes */
		writel(msg->len, &idev->regs->mst_tx_xfer);
		/* RX 0 bytes */
		writel(0, &idev->regs->mst_rx_xfer);
	}

	if (i2c_m_ten(msg)) {
		/* 10-bit address
		 *   addr_1: 5'b11110 | addr[9:8] | (R/W)
		 *   addr_2: addr[7:0]
		 */
		addr_1 = 0xF0 | ((msg->addr >> 7) & 0x06);
		addr_2 = msg->addr & 0xFF;
	} else {
		/* 7-bit address
		 *   addr_1: addr[6:0] | (R/W)
		 *   addr_2: dont care
		 */
		addr_1 = i2c_8bit_addr_from_msg(msg);
		addr_2 = 0;
	}
	if (i2c_m_rd(msg))
		addr_1 |= 1;
	writel(addr_1, &idev->regs->mst_addr_1);
	writel(addr_2, &idev->regs->mst_addr_2);

	if (i2c_m_rd(msg)) {
		int_mask |= MST_STATUS_RFL;
	} else {
		axxia_i2c_fill_tx_fifo(idev);
		if (idev->msg_xfrd < msg->len)
			int_mask |= MST_STATUS_TFL;
	}

	/* Start manual mode */
	writel(0x8, &idev->regs->mst_command);

	if (idev->irq > 0) {
		i2c_int_enable(idev, int_mask);
		ret = wait_for_completion_timeout(&idev->msg_complete,
						  I2C_XFER_TIMEOUT);
		i2c_int_disable(idev, int_mask);
		WARN_ON(readl(&idev->regs->mst_command) & 0x8);
	} else {
		unsigned long tmo = jiffies + I2C_XFER_TIMEOUT;

		do {
			/* Poll interrupt status */
			axxia_i2c_service_irq(idev);
			ret = try_wait_for_completion(&idev->msg_complete);
		} while (!ret && time_before(jiffies, tmo));
	}

	if (ret == 0) {
		dev_warn(idev->dev, "xfer timeout (%#x)\n", msg->addr);
		axxia_i2c_init(idev);
		return -ETIMEDOUT;
	}

	if (idev->msg_err == -ETIMEDOUT)
		i2c_recover_bus(&idev->adapter);

	if (unlikely(idev->msg_err != 0)) {
		axxia_i2c_init(idev);
		return -EIO;
	}

	return 0;
}

static int
axxia_i2c_stop(struct axxia_i2c_dev *idev)
{
	u32 int_mask = MST_STATUS_ERR | MST_STATUS_SCC;
	int ret;

	reinit_completion(&idev->msg_complete);

	/* Issue stop */
	writel(0xb, &idev->regs->mst_command);
	i2c_int_enable(idev, int_mask);
	ret = wait_for_completion_timeout(&idev->msg_complete,
					  I2C_STOP_TIMEOUT);
	i2c_int_disable(idev, int_mask);
	if (ret == 0)
		return -ETIMEDOUT;

	WARN_ON(readl(&idev->regs->mst_command) & 0x8);

	return 0;
}

static int
axxia_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg msgs[], int num)
{
	struct axxia_i2c_dev *idev = i2c_get_adapdata(adap);
	int i;
	int ret = 0;

	for (i = 0; ret == 0 && i < num; i++)
		ret = axxia_i2c_xfer_msg(idev, &msgs[i]);

	axxia_i2c_stop(idev);

	return ret ? : i;
}

static int axxia_i2c_get_scl(struct i2c_adapter *adap)
{
	struct axxia_i2c_dev *idev = i2c_get_adapdata(adap);

	return !!(readl(idev->base + I2C_BUS_MONITOR) & BM_SCLS);
}

static void axxia_i2c_set_scl(struct i2c_adapter *adap, int val)
{
	struct axxia_i2c_dev *idev = i2c_get_adapdata(adap);
	u32 tmp;

	/* Preserve SDA Control */
	tmp = readl(idev->base + I2C_BUS_MONITOR) & BM_SDAC;
	if (!val)
		tmp |= BM_SCLC;
	writel(tmp, idev->base + I2C_BUS_MONITOR);
}

static int axxia_i2c_get_sda(struct i2c_adapter *adap)
{
	struct axxia_i2c_dev *idev = i2c_get_adapdata(adap);

	return !!(readl(idev->base + I2C_BUS_MONITOR) & BM_SDAS);
}

static struct i2c_bus_recovery_info axxia_i2c_recovery_info = {
	.recover_bus = i2c_generic_scl_recovery,
	.get_scl = axxia_i2c_get_scl,
	.set_scl = axxia_i2c_set_scl,
	.get_sda = axxia_i2c_get_sda,
};

static u32 axxia_i2c_func(struct i2c_adapter *adap)
{
		u32 caps = (I2C_FUNC_I2C |
			    I2C_FUNC_10BIT_ADDR |
			    I2C_FUNC_SMBUS_EMUL |
			    I2C_FUNC_SMBUS_BLOCK_DATA);
	return caps;
}

static const struct i2c_algorithm axxia_i2c_algo = {
	.master_xfer = axxia_i2c_xfer,
	.functionality = axxia_i2c_func,
};

static int axxia_i2c_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct axxia_i2c_dev *idev = NULL;
	struct resource *res;
	void __iomem *base;
	int ret = 0;

	idev = devm_kzalloc(&pdev->dev, sizeof(*idev), GFP_KERNEL);
	if (!idev)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(base))
		return PTR_ERR(base);

	idev->irq = platform_get_irq(pdev, 0);
	if (idev->irq < 0)
		dev_info(&pdev->dev, "No IRQ specified, using polling mode\n");

	idev->i2c_clk = devm_clk_get(&pdev->dev, "i2c");
	if (IS_ERR(idev->i2c_clk)) {
		dev_err(&pdev->dev, "missing I2C bus clock");
		return PTR_ERR(idev->i2c_clk);
	}

	idev->regs = (struct i2c_regs __iomem *) base;
	idev->base = base;
	idev->dev = &pdev->dev;
	init_completion(&idev->msg_complete);

	of_property_read_u32(np, "clock-frequency", &idev->bus_clk_rate);
	if (idev->bus_clk_rate == 0)
		idev->bus_clk_rate = 100000; /* default clock rate */

	ret = clk_prepare_enable(idev->i2c_clk);
	if (ret) {
		dev_err(&pdev->dev, "failed to enable clock\n");
		return ret;
	}

	ret = axxia_i2c_init(idev);
	if (ret) {
		dev_err(&pdev->dev, "Failed to initialize i2c controller");
		goto error_disable_clk;
	}

	if (idev->irq >= 0) {
		ret = devm_request_irq(&pdev->dev, idev->irq, axxia_i2c_isr, 0,
				       pdev->name, idev);
		if (ret) {
			dev_err(&pdev->dev, "can't claim irq %d\n", idev->irq);
			goto error_disable_clk;
		}
	}

	i2c_set_adapdata(&idev->adapter, idev);
	strlcpy(idev->adapter.name, pdev->name, sizeof(idev->adapter.name));
	idev->adapter.owner = THIS_MODULE;
	idev->adapter.class = I2C_CLASS_HWMON;
	idev->adapter.algo = &axxia_i2c_algo;
	idev->adapter.bus_recovery_info = &axxia_i2c_recovery_info;
	idev->adapter.dev.parent = &pdev->dev;
	idev->adapter.dev.of_node = pdev->dev.of_node;

	ret = i2c_add_adapter(&idev->adapter);
	if (ret) {
		dev_err(&pdev->dev, "Failed to add I2C adapter\n");
		goto error_disable_clk;
	}

	platform_set_drvdata(pdev, idev);

	return 0;

error_disable_clk:
	clk_disable_unprepare(idev->i2c_clk);
	return ret;
}

static int
axxia_i2c_remove(struct platform_device *pdev)
{
	struct axxia_i2c_dev *idev = platform_get_drvdata(pdev);

	i2c_del_adapter(&idev->adapter);

	return 0;
}

#ifdef CONFIG_PM
static int axxia_i2c_suspend(struct platform_device *pdev, pm_message_t state)
{
	return -EOPNOTSUPP;
}

static int axxia_i2c_resume(struct platform_device *pdev)
{
	return -EOPNOTSUPP;
}
#else
#define axxia_i2c_suspend NULL
#define axxia_i2c_resume NULL
#endif

/* Match table for of_platform binding */
static const struct of_device_id axxia_i2c_of_match[] = {
	{ .compatible = "lsi,api2c", },
	{},
};

MODULE_DEVICE_TABLE(of, axxia_i2c_of_match);

static struct platform_driver axxia_i2c_driver = {
	.probe   = axxia_i2c_probe,
	.remove  = axxia_i2c_remove,
	.suspend = axxia_i2c_suspend,
	.resume  = axxia_i2c_resume,
	.driver  = {
		.name  = "axxia-i2c",
		.owner = THIS_MODULE,
		.of_match_table = axxia_i2c_of_match,
	},
};

module_platform_driver(axxia_i2c_driver);

MODULE_DESCRIPTION("Axxia I2C Bus driver");
MODULE_AUTHOR("Anders Berg <anders.berg@lsi.com>");
MODULE_LICENSE("GPL v2");
