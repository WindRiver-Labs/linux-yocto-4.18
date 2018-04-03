/*
 * GPIO interface for SSP chip select pins.
 *
 * Copyright (C) 2013 INTEL Corporation
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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/of.h>

struct gpio_dev {
	void __iomem    *regs;
	struct gpio_chip gpio_chip;
};

static int
ssp_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	struct gpio_dev *priv = dev_get_drvdata(chip->parent);
	u32 tmp = readl(priv->regs + 0x30);

	return !!(tmp & (1<<offset));
}

static int
ssp_gpio_direction_out(struct gpio_chip *chip, unsigned offset, int value)
{
	/* Pins are only outputs */
	return 0;
}

static void
ssp_gpio_set(struct gpio_chip *chip, unsigned offset, int value)
{
	struct gpio_dev *priv = dev_get_drvdata(chip->parent);
	u32 tmp = readl(priv->regs + 0x30);

	if (value)
		tmp |= (1<<offset);
	else
		tmp &= ~(1<<offset);
	writel(tmp, priv->regs + 0x30);
}

static int
ssp_gpio_probe(struct platform_device *pdev)
{
	struct gpio_dev *priv;
	struct resource *io;
	struct gpio_chip *chip;
	int ret;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	platform_set_drvdata(pdev, priv);

	io = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!io)
		return -EINVAL;

	priv->regs = devm_ioremap_nocache(&pdev->dev,
					   io->start, resource_size(io));
	if (!priv->regs)
		return -ENXIO;

	chip = &priv->gpio_chip;
	chip->parent = &pdev->dev;
#ifdef CONFIG_OF_GPIO
	chip->of_node = pdev->dev.of_node;
#endif
	chip->get = ssp_gpio_get;
	chip->direction_output = ssp_gpio_direction_out;
	chip->set = ssp_gpio_set;
	chip->label = "ssp-gpio";
	chip->owner = THIS_MODULE;
	chip->base = -1;
	chip->ngpio = 5;

	/* Deassert all */
	writel(0x1f, priv->regs + 0x30);

	ret = gpiochip_add(chip);
	if (ret < 0)
		dev_err(&pdev->dev, "could not register gpiochip, %d\n", ret);

	return ret;
}

static int
ssp_gpio_remove(struct platform_device *pdev)
{
	struct gpio_dev *priv = dev_get_drvdata(&pdev->dev);

	gpiochip_remove(&priv->gpio_chip);
	return 0;
}

static const struct of_device_id ssp_gpio_id_table[] = {
	{ .compatible = "axxia,ssp-gpio" },
	{}
};
MODULE_DEVICE_TABLE(platform, ssp_gpio_id_table);

static struct platform_driver ssp_gpio_driver = {
	.driver = {
		.name	= "ssp-gpio",
		.owner	= THIS_MODULE,
		.of_match_table = ssp_gpio_id_table
	},
	.probe		= ssp_gpio_probe,
	.remove		= ssp_gpio_remove,
};

module_platform_driver(ssp_gpio_driver);

MODULE_AUTHOR("INTEL Corporation");
MODULE_DESCRIPTION("GPIO interface for SSP chip selects");
MODULE_LICENSE("GPL");
