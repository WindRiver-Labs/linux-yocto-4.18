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

static void tal_dev_rx_callback(struct device *dev, unsigned char *rx_buff,
				int size)
{
	struct tal_device *tal_dev = dev->platform_data;
	unsigned long flags;

	/* Save buffer */
	spin_lock_irqsave(&tal_dev->lock, flags);
	tal_dev->rx_buff_p = rx_buff;
	tal_dev->rx_buff_size = size;
	spin_unlock_irqrestore(&tal_dev->lock, flags);

	wake_up_interruptible(&tal_dev->wait);
}

static void tal_dev_tx_callback(struct device *dev, unsigned char *tx_buff,
				int size)
{
	struct tal_device *tal_dev = dev->platform_data;
	unsigned long flags;

	/* Save buffer */
	spin_lock_irqsave(&tal_dev->lock, flags);
	tal_dev->tx_buff_p = tx_buff;
	tal_dev->tx_buff_size = size;
	spin_unlock_irqrestore(&tal_dev->lock, flags);

	wake_up_interruptible(&tal_dev->wait);
}

static struct tal_mmp_ops tal_mmp_ops = {
	.tal_mmp_rx_callback	= tal_dev_rx_callback,
	.tal_mmp_tx_callback	= tal_dev_tx_callback,
};

static ssize_t tal_dev_read(struct file *file_p, char __user *buf,
			    size_t size, loff_t *ppos)
{
	struct tal_device *tal_dev = file_p->private_data;
	unsigned long flags;
	unsigned char *rx_buff;

	/* Check if we have got the buffer */
	spin_lock_irqsave(&tal_dev->lock, flags);
	rx_buff = tal_dev->rx_buff_p;
	tal_dev->rx_buff_p = NULL;
	size = min(tal_dev->rx_buff_size, size);
	spin_unlock_irqrestore(&tal_dev->lock, flags);

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
	struct tal_device *tal_dev = file_p->private_data;
	unsigned long flags;
	unsigned char *tx_buff;

	/* Check if we have got the buffer */
	spin_lock_irqsave(&tal_dev->lock, flags);
	tx_buff = tal_dev->tx_buff_p;
	tal_dev->tx_buff_p = NULL;
	size = min(tal_dev->tx_buff_size, size);
	spin_unlock_irqrestore(&tal_dev->lock, flags);

	if (!tx_buff)
		return 0;

	/* Copy data from userspace */
	if (copy_from_user(tx_buff, buf, size))
		size = -EFAULT;

	/* Pass the buffer to TAL */
	if (tal_write(tal_dev->dev, tx_buff, size) != TAL_STATUS_OK)
		return -EIO;

	return size;
}

static int tal_dev_open(struct inode *inode_p, struct file *file_p)
{
	struct device *dev;

	/* Replace the file's private data with a pointer to 'tal_dev' */
	dev = ((struct miscdevice *)file_p->private_data)->parent;

	file_p->private_data = dev->platform_data;

	return 0;
}

static int tal_dev_release(struct inode *inode_p, struct file *file_p)
{
	return 0;
}

static unsigned int tal_dev_poll(struct file *file_p, poll_table *poll_table_p)
{
	struct tal_device *tal_dev = file_p->private_data;
	unsigned long flags;
	int mask = 0;

	poll_wait(file_p, &tal_dev->wait, poll_table_p);

	spin_lock_irqsave(&tal_dev->lock, flags);
	if (tal_dev->rx_buff_p)
		mask |= POLLIN | POLLRDNORM;
	if (tal_dev->tx_buff_p)
		mask |= POLLOUT | POLLWRNORM;
	spin_unlock_irqrestore(&tal_dev->lock, flags);

	return mask;
}

static long tal_dev_ioctl(struct file *file_p, unsigned int cmd,
							unsigned long arg)
{
	struct tal_device *tal_dev = file_p->private_data;
	struct tal_dev_params tal_dev_params;
	char buffer[16];
	long ret = 0;
	int i;

	/* Argument checking */
	if (_IOC_TYPE(cmd) != TAL_DEV_IOCTL_MAGIC) {
		dev_err(tal_dev->dev, "%s: invalid TAL DEV Magic Num %i %i\n",
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
		dev_err(tal_dev->dev, "%s: wrong TAL access type %i (cmd %i)\n",
		       __func__, _IOC_DIR(cmd), cmd);
		return -EFAULT;
	}

	switch (cmd) {
	case TAL_DEV_INIT:
		if (copy_from_user(&tal_dev_params,
		    (void *)arg, sizeof(tal_dev_params)))
			return -EFAULT;

		tal_dev->params.pcm_format = tal_dev_params.pcm_format;
		tal_dev->params.sampling_period = 10; /* ms */
		tal_dev->params.total_lines = tal_dev_params.total_lines;
		tal_dev->params.enable_internal_loopback =
					tal_dev_params.enable_internal_loopback;
		for (i = 0; i < TAL_MAX_PHONE_LINES; i++)
			tal_dev->params.pcm_slot[i] =
					    (i + 1) * tal_dev_params.pcm_format;

		if (tal_init(tal_dev->dev, &tal_dev->params, &tal_mmp_ops) !=
		    TAL_STATUS_OK)
			return -EIO;

		break;

	case TAL_DEV_EXIT:
		tal_exit(tal_dev->dev);
		break;

	case TAL_DEV_PCM_START:
		tal_dev->rx_buff_p = NULL;
		tal_dev->tx_buff_p = NULL;
		tal_pcm_start(tal_dev->dev);
		break;

	case TAL_DEV_PCM_STOP:
		tal_pcm_stop(tal_dev->dev);
		break;

	default:
		/* Pass ioctl to the low-level interface */
		if (_IOC_SIZE(cmd) > sizeof(buffer))
			return -E2BIG;

		if (_IOC_DIR(cmd) & _IOC_WRITE)
			if (copy_from_user(buffer, (void *)arg, _IOC_SIZE(cmd)))
				return -EFAULT;

		ret = tal_control(cmd, tal_dev->dev);

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

enum tal_status tal_dev_init(struct device *dev, u32 tdm_index,
			     struct miscdevice *miscdev)
{
	struct tal_device *tal_dev;
	char name[10];
	int status;

	/* Initialize TAL device */
	tal_dev = devm_kzalloc(dev, sizeof(*tal_dev), GFP_KERNEL);
	if (!tal_dev)
		return TAL_STATUS_INIT_ERROR;

	tal_dev->dev = dev;
	init_waitqueue_head(&tal_dev->wait);
	spin_lock_init(&tal_dev->lock);
	dev->platform_data = tal_dev;

	/* Configure and register MISC device */
	snprintf(name, sizeof(name), "tal%d", tdm_index);

	miscdev->name = name;
	miscdev->minor = TALDEV_MINOR + tdm_index;
	miscdev->fops = &tal_dev_fops;
	miscdev->parent = dev;

	status = misc_register(miscdev);
	if (status < 0) {
		dev_err(dev, "Failed to register %s device!\n", name);
		return TAL_STATUS_INIT_ERROR;
	}

	dev_info(dev, "register /dev/%s\n", name);

	return TAL_STATUS_OK;
}

void tal_dev_exit(struct miscdevice *miscdev)
{
	misc_deregister(miscdev);
}
