// SPDX-License-Identifier: GPL-2.0
/*
 *
 * Copyright (C) 2019 Marvell International Ltd.
 *
 */

#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include "tal.h"
#include "tal_dev.h"

#define	TALDEV_NAME	"tal"

static DECLARE_WAIT_QUEUE_HEAD(tal_dev_wait);
static DEFINE_SPINLOCK(tal_dev_lock);
static unsigned char *rx_buff_p, *tx_buff_p;
static size_t rx_buff_size, tx_buff_size;

static void tal_dev_rx_callback(unsigned char *rx_buff, int size)
{
	unsigned long flags;

	/* Save buffer */
	spin_lock_irqsave(&tal_dev_lock, flags);
	rx_buff_p = rx_buff;
	rx_buff_size = size;
	spin_unlock_irqrestore(&tal_dev_lock, flags);

	wake_up_interruptible(&tal_dev_wait);
}

static void tal_dev_tx_callback(unsigned char *tx_buff, int size)
{
	unsigned long flags;

	/* Save buffer */
	spin_lock_irqsave(&tal_dev_lock, flags);
	tx_buff_p = tx_buff;
	tx_buff_size = size;
	spin_unlock_irqrestore(&tal_dev_lock, flags);

	wake_up_interruptible(&tal_dev_wait);
}

static struct tal_params tal_params;
static struct tal_mmp_ops tal_mmp_ops = {
	.tal_mmp_rx_callback	= tal_dev_rx_callback,
	.tal_mmp_tx_callback	= tal_dev_tx_callback,
};

static ssize_t tal_dev_read(struct file *file_p, char __user *buf,
			    size_t size, loff_t *ppos)
{
	unsigned long flags;
	unsigned char *rx_buff;

	/* Check if we have got the buffer */
	spin_lock_irqsave(&tal_dev_lock, flags);
	rx_buff = rx_buff_p;
	rx_buff_p = NULL;
	size = min(rx_buff_size, size);
	spin_unlock_irqrestore(&tal_dev_lock, flags);

	if (!rx_buff)
		return 0;

	/* Copy data to userspace */
	if (copy_to_user(buf, rx_buff, size))
		return -EFAULT;

	return size;
}

static ssize_t tal_dev_write(struct file *file_p, const char __user *buf,
			     size_t size, loff_t *ppos)
{
	unsigned long flags;
	unsigned char *tx_buff;

	/* Check if we have got the buffer */
	spin_lock_irqsave(&tal_dev_lock, flags);
	tx_buff = tx_buff_p;
	tx_buff_p = NULL;
	size = min(tx_buff_size, size);
	spin_unlock_irqrestore(&tal_dev_lock, flags);

	if (!tx_buff)
		return 0;

	/* Copy data from userspace */
	if (copy_from_user(tx_buff, buf, size))
		size = -EFAULT;

	/* Pass the buffer to TAL */
	if (tal_write(tx_buff, size) != TAL_STAT_OK)
		return -EIO;

	return size;
}

static int tal_dev_open(struct inode *inode_p, struct file *file_p)
{
	try_module_get(THIS_MODULE);
	return 0;
}

static int tal_dev_release(struct inode *inode_p, struct file *file_p)
{
	module_put(THIS_MODULE);
	return 0;
}

static unsigned int tal_dev_poll(struct file *file_p, poll_table *poll_table_p)
{
	unsigned long flags;
	int mask = 0;

	poll_wait(file_p, &tal_dev_wait, poll_table_p);

	spin_lock_irqsave(&tal_dev_lock, flags);
	if (rx_buff_p)
		mask |= POLLIN | POLLRDNORM;
	if (tx_buff_p)
		mask |= POLLOUT | POLLWRNORM;
	spin_unlock_irqrestore(&tal_dev_lock, flags);

	return mask;
}

static long tal_dev_ioctl(struct file *file_p, unsigned int cmd,
							unsigned long arg)
{
	struct tal_dev_params tal_dev_params;
	char buffer[16];
	long ret = 0;
	int i;

	/* Argument checking */
	if (_IOC_TYPE(cmd) != TAL_DEV_IOCTL_MAGIC) {
		pr_err("%s: invalid TAL DEV Magic Num %i %i\n",
		       __func__, _IOC_TYPE(cmd), TAL_DEV_IOCTL_MAGIC);
		return -ENOTTY;
	}

	if (_IOC_DIR(cmd) & _IOC_READ)
		ret = !access_ok(VERIFY_WRITE, (void __user *)arg,
				 _IOC_SIZE(cmd));

	if ((_IOC_DIR(cmd) & _IOC_WRITE) && !ret)
		ret = !access_ok(VERIFY_READ, (void __user *)arg,
				 _IOC_SIZE(cmd));

	if (ret) {
		pr_err("%s: invalid TAL DEV access type %i from cmd %i\n",
		       __func__, _IOC_DIR(cmd), cmd);
		return -EFAULT;
	}

	switch (cmd) {
	case TAL_DEV_INIT:
		if (copy_from_user(&tal_dev_params,
		    (void *)arg, sizeof(tal_dev_params)))
			return -EFAULT;

		tal_params.pcm_format = tal_dev_params.pcm_format;
		tal_params.sampling_period = 10; /* ms */
		tal_params.total_lines = tal_dev_params.total_lines;
		for (i = 0; i < TAL_MAX_PHONE_LINES; i++)
			tal_params.pcm_slot[i] =
					    (i + 1) * tal_dev_params.pcm_format;

		if (tal_init(&tal_params, &tal_mmp_ops) != TAL_STAT_OK)
			return -EIO;

		break;

	case TAL_DEV_EXIT:
		tal_exit();
		break;

	case TAL_DEV_PCM_START:
		rx_buff_p = NULL;
		tx_buff_p = NULL;
		tal_pcm_start();
		break;

	case TAL_DEV_PCM_STOP:
		tal_pcm_stop();
		break;

	default:
		/* Pass ioctl to the low-level interface */
		if (_IOC_SIZE(cmd) > sizeof(buffer))
			return -E2BIG;

		if (_IOC_DIR(cmd) & _IOC_WRITE)
			if (copy_from_user(buffer, (void *)arg, _IOC_SIZE(cmd)))
				return -EFAULT;

		ret = tal_control(cmd, buffer);

		if (_IOC_DIR(cmd) & _IOC_READ)
			if (copy_to_user((void *)arg, buffer, _IOC_SIZE(cmd)))
				return -EFAULT;

		break;
	}

	return ret;
}

static const struct file_operations tal_dev_fops = {
	.owner		= THIS_MODULE,
	.read		= tal_dev_read,
	.write		= tal_dev_write,
	.poll		= tal_dev_poll,
	.unlocked_ioctl	= tal_dev_ioctl,
	.open		= tal_dev_open,
	.release	= tal_dev_release,
};

static struct miscdevice tal_dev = {
	.minor	= TALDEV_MINOR,
	.name	= TALDEV_NAME,
	.fops	= &tal_dev_fops,
};

static int __init tal_dev_init(void)
{
	int status;

	status = misc_register(&tal_dev);
	if (status < 0) {
		pr_err("Failed to register TAL device!\n");
		return status;
	}

	return 0;
}

static void __exit tal_dev_exit(void)
{
	misc_deregister(&tal_dev);
}

/* Module stuff */
module_init(tal_dev_init);
module_exit(tal_dev_exit);
MODULE_DESCRIPTION("Marvell TAL Device Interface");
MODULE_AUTHOR("Piotr Ziecik <kosmo@angel.net.pl>");
MODULE_LICENSE("GPL");
