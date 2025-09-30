/*
 * Core driver for switch IP
 *
 * (c) Copyright 2015-2018 	OscillatorIMP Digital
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <linux/ioctl.h>
#include <linux/version.h>

#include "switch_config.h"

#define DRIVER_NAME "switch"
#define SELECT_INPUT_REG (0x01 << 2)

struct switch_dev {
	char *name;
	void *membase;
	struct resource *mem_res;
	struct device *dev;
	struct miscdevice misc;
	struct list_head list;
};

static LIST_HEAD(switch_data_list);

static long switch_ioctl(struct file *filp, unsigned int cmd,
			    unsigned long arg)
{
	int retval = 0;
	struct switch_dev *sdev;

	sdev = filp->private_data;
	if (sdev == NULL)
		return -ENODATA;

	if (cmd == SWITCH_SELECT_INPUT) {
		get_user(retval, (int __user *)arg);
		writel(retval, sdev->membase + SELECT_INPUT_REG);
	} else {
		printk("wrong cmd case\r\n");
		retval = -EFAULT;
	}
	return retval;
}

int switch_open(struct inode *inode, struct file *filp)
{
	struct switch_dev *pos, *data = NULL;
	/* Allocate and fill any data structure to be put in filp->private_data */
	list_for_each_entry(pos, &switch_data_list, list) {
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

int switch_release(struct inode *inode, struct file *filp)
{
	printk("%s released\r\n", DRIVER_NAME);

	filp->private_data = NULL;

	return 0;
}

static struct file_operations fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = switch_ioctl,
	.open = switch_open,
	.release = switch_release,
};

static int switch_probe(struct platform_device *pdev)
{
	struct plat_switch_port *pdata = pdev->dev.platform_data;
	struct device_node *np = pdev->dev.of_node;
	int ret = 0;
	struct switch_dev *sdev;
	struct resource *mem_res;

	if (pdata)
		printk("%s probing %d\n", pdata->name, pdata->num);
	else {
		if (!np) {
			return -ENODEV;
		}
		printk("probing %s\n", np->name);
	}

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
	sdev = kzalloc(sizeof(struct switch_dev), GFP_KERNEL);
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

	if (pdata) {
		pdata->sdev = sdev;

		sdev->name = (char *)kmalloc((1 + strlen(pdata->name)) * sizeof(char),
				     GFP_KERNEL);
	} else {
		sdev->name = (char *)kmalloc(sizeof(char)*strlen(np->name) + 1, GFP_KERNEL);
	}
	if (sdev->name == NULL) {
		dev_err(&pdev->dev, "Kmalloc failed for platform device name\r\n");
		goto out_iounmap;
	}

	if (pdata) {
		if (snprintf(sdev->name, 1+sizeof(pdata->name), "%s", pdata->name) < 0) {
			printk(KERN_ERR "copy error\n");
			goto out_iounmap;
		}
	} else {
		if (strncpy(sdev->name, np->name, 1+strlen(np->name)) < 0) {
			printk("copy error");
			goto out_iounmap;
		}
	}
	printk("name: %s %d %d\n", sdev->name, sizeof(sdev->name), strlen(sdev->name));

	sdev->misc.name = sdev->name;
	sdev->misc.minor = MISC_DYNAMIC_MINOR;
	sdev->misc.fops = &fops;

	platform_set_drvdata(pdev, sdev);

	ret = misc_register(&sdev->misc);
	if (ret) {
		dev_err(&pdev->dev, KERN_ERR "%s:%u: misc_register failed %d\n",
		       __func__, __LINE__, ret);
		goto out_name_free;
	}
	dev_info(&pdev->dev, "%s: Add the device to the kernel, "
		 "connecting cdev to major/minor number \n", sdev->name);

	list_add(&sdev->list, &switch_data_list);

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

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 11, 0)
static int switch_remove(struct platform_device *pdev)
#else
static void switch_remove(struct platform_device *pdev)
#endif
{
	struct switch_dev *sdev;

	sdev = (struct switch_dev *)platform_get_drvdata(pdev);

	misc_deregister(&sdev->misc);
	printk(KERN_INFO "%s: deleted with success\n", sdev->name);
	kfree(sdev->name);
	iounmap(sdev->membase);
	release_mem_region(sdev->mem_res->start, resource_size(sdev->mem_res));
	kfree(sdev);

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 11, 0)
	return 0;
#endif
}

static struct of_device_id switch_of_match[] = {
	{ .compatible = "ggm,switch", },
	{}
};

MODULE_DEVICE_TABLE(of, switch_of_match);

static struct platform_driver plat_switch_driver = {
	.probe = switch_probe,
	.remove = switch_remove,
	.driver = {
		   .name = DRIVER_NAME,
		   .owner = THIS_MODULE,
		   .of_match_table = switch_of_match,
   },
};

module_platform_driver(plat_switch_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Gwenhael Goavec-Merou <gwenhael.goavec-merou@trabucayre.com>");
MODULE_DESCRIPTION("switch CTRL DRIVER");
