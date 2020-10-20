/*
 * Generic driver for redpitaya_converters_12 IP
 *
 * (c) Copyright 2019
 * Gwenhael Goavec-Merou <gwenhael.goavec-merou@trabucayre.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/list.h>
#include <linux/jiffies.h>

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/fs.h>		/* for file  operations */
#include <linux/init.h>
#include <linux/ioport.h>	/* request_mem_region */
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/timer.h>

#include <linux/slab.h>		/* kmalloc */

#include <linux/miscdevice.h>
#include <asm/uaccess.h>	/* copy_to_user function */
#include <asm/io.h>		/* readw() writew() */

#include "redpitaya_converters_12_config.h"

#define DRIVER_NAME "redpitaya_converters_12"

#define USE_32Bits

#define REDPITAYA_CONVERTERS_12_CONF_BIT		(0x1 << 0)
#define REDPITAYA_CONVERTERS_12_ADC_DAC_SEL_BIT		(0x1 << 1)
#define REDPITAYA_CONVERTERS_12_CONF_EN_BIT		(0x1 << 2)
#define REDPITAYA_CONVERTERS_12_PLL_EN_BIT              (0x1 << 3)
#define REDPITAYA_CONVERTERS_12_PLL_OK_BIT              (0x1 << 4)

struct redpitaya_converters_12_dev {
	char *name;		/* name of the instance */
	void *membase;		/* base address for instance  */
	struct resource *mem_res;
	struct device *dev;
	struct miscdevice misc;
	struct list_head list;
};

static LIST_HEAD(redpitaya_converters_12_data_list);

static long redpitaya_converters_12_ioctl(struct file *filp, unsigned int cmd,
			    unsigned long arg)
{
	int retval = 0;
	struct redpitaya_converters_12_dev *redpitaya_converters_12;
	u32 ioc, reg;

	/* guard against device removal before, or while,
	 * we issue this ioctl.
	 */
	redpitaya_converters_12 = filp->private_data;
	if (redpitaya_converters_12 == NULL)
		return -ENODATA;

	reg = (_IOC_NR(cmd) << 2);
	if (_IOC_DIR(cmd) & _IOC_READ) {
		ioc = readl(redpitaya_converters_12->membase + reg);
		printk("read %x\n", ioc);
		if (put_user(ioc, (u32 __user *) arg))
			return -EACCES;
	} else {
		printk("ioc write");
		if (get_user(ioc, (u32 __user *) arg))
			return -EACCES;

		writel(ioc, redpitaya_converters_12->membase + reg);
	}
	return retval;
}

int redpitaya_converters_12_open(struct inode *inode, struct file *filp)
{
	struct redpitaya_converters_12_dev *pos, *data = NULL;
	/* Allocate and fill any data structure to be put in filp->private_data */
	list_for_each_entry(pos, &redpitaya_converters_12_data_list, list) {
		if (pos->misc.minor == iminor(inode)) {
			data = pos;
			break;
		}
	}
	if (data == NULL)
		return -ENODATA;

	filp->private_data = (struct device *)data;

	return 0;
}

int redpitaya_converters_12_release(struct inode *inode, struct file *filp)
{
	printk("redpitaya_converters_12 release\n");
	filp->private_data = NULL;
	return 0;
}

static struct file_operations redpitaya_converters_12_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = redpitaya_converters_12_ioctl,
	.open = redpitaya_converters_12_open,
	.release = redpitaya_converters_12_release,
};

static int redpitaya_converters_12_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	int ret = 0;
	int size_name;
	struct redpitaya_converters_12_dev *sdev;
	struct resource *mem_res;

	if (!np) {
		dev_err(&pdev->dev, "Platform data required !\n");
		return -ENODEV;
	}
	printk("probing %s\n", np->name);

	/* get resources */
	mem_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem_res) {
		dev_err(&pdev->dev, "can't find mem resource\n");
		return -EINVAL;
	}

	mem_res =
	    request_mem_region(mem_res->start, resource_size(mem_res),
			       pdev->name);
	if (!mem_res) {
		dev_err(&pdev->dev, "iomem already in use\n");
		return -EBUSY;
	}

	/* allocate memory for private structure */
	sdev = kzalloc(sizeof(struct redpitaya_converters_12_dev), GFP_KERNEL);
	if (!sdev) {
		ret = -ENOMEM;
		goto out_release_mem;
	}

	sdev->membase = ioremap(mem_res->start, resource_size(mem_res));
	if (!sdev->membase) {
		dev_err(&pdev->dev, "ioremap failed\n");
		ret = -ENOMEM;
		goto out_dev_free;
	}
	sdev->mem_res = mem_res;

	size_name = (1 + strlen(np->name)) * sizeof(char);

	sdev->name = (char *)kmalloc(size_name, GFP_KERNEL);

	if (sdev->name == NULL) {
		dev_err(&pdev->dev, "Kmalloc name space error\n");
		goto out_iounmap;
	}

	if (strncpy(sdev->name, np->name, 1 + strlen(np->name)) < 0) {
		printk("copy error");
		goto out_name_free;
	}
	printk("name: %s %d %d\n", sdev->name, sizeof(sdev->name), strlen(sdev->name));

	sdev->misc.name = sdev->name;
	sdev->misc.minor = MISC_DYNAMIC_MINOR;
	sdev->misc.fops = &redpitaya_converters_12_fops,

	platform_set_drvdata(pdev, sdev);

	ret = misc_register(&sdev->misc);
	if (ret) {
		dev_err(&pdev->dev, KERN_ERR "%s:%u: misc_register failed %d\n",
		       __func__, __LINE__, ret);
		goto out_name_free;
	}

	dev_info(&pdev->dev, "%s: Add the device to the kernel, "
		 "connecting cdev to major/minor number \n", sdev->name);

	list_add(&sdev->list, &redpitaya_converters_12_data_list);

	/* OK driver ready ! */
	dev_info(&pdev->dev, KERN_INFO "%s loaded\n", sdev->name);
	return 0;

 out_name_free:
	kfree(sdev->name);
 out_iounmap:
	iounmap(sdev->membase);
 out_dev_free:
	kfree(sdev);
 out_release_mem:
	release_mem_region(mem_res->start, resource_size(mem_res));

	return ret;
}

static int redpitaya_converters_12_remove(struct platform_device *pdev)
{
	struct redpitaya_converters_12_dev *sdev;
	sdev = (struct redpitaya_converters_12_dev *)platform_get_drvdata(pdev);

	misc_deregister(&sdev->misc);
	printk(KERN_INFO "%s: deleted with success\n", sdev->name);
	kfree(sdev->name);
	iounmap(sdev->membase);
	release_mem_region(sdev->mem_res->start, resource_size(sdev->mem_res));
	kfree(sdev);

	return 0;
}

static struct of_device_id redpitaya_converters_12_of_match[] = {
	{ .compatible = "ggm,redpitaya_converters_12", },
	{}
};

MODULE_DEVICE_TABLE(of, redpitaya_converters_12_of_match);

static struct platform_driver plat_redpitaya_converters_12_driver = {
	.probe = redpitaya_converters_12_probe,
	.remove = redpitaya_converters_12_remove,
	.driver = {
		   .name = "redpitaya_converters_12",
		   .owner = THIS_MODULE,
		   .of_match_table = redpitaya_converters_12_of_match,
   },
};

module_platform_driver(plat_redpitaya_converters_12_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("OscimpDigital");
MODULE_DESCRIPTION("redpitaya_converters_12 IP generic driver");
