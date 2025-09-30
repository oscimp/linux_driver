/*
 * Generic driver for nco_counter IP
 *
 * (c) Copyright 2013-2019    OscillatorIMP Digital
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
#include <linux/version.h>

#include <linux/slab.h>		/* kmalloc */

#include <linux/miscdevice.h>
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 12, 0)
#  include <asm/uaccess.h>	/* copy_to_user function */
#else
#  include <linux/uaccess.h>	/* copy_to_user function */
#endif
#include <asm/io.h>		/* readw() writew() */

#include "nco_counter_config.h"

#define DRIVER_NAME "nco_counter"

#define  NCO_COUNTER_REG_ID		(0x00 << 2)
#define  NCO_COUNTER_REG_RESET	(0x01 << 2)

struct nco_counter_dev {
	char *name;			/* name of the instance */
	void *membase;		/* base address for instance  */
	struct resource *mem_res;
	struct device *dev;
	struct miscdevice misc;
	struct list_head list;
};

static LIST_HEAD(nco_counter_data_list);

static long nco_counter_ioctl(struct file *filp, unsigned int cmd,
			    unsigned long arg)
{
	int retval = 0;
	struct nco_counter_dev *nco_counter;
	u64 val;
	u64 ioc;
	u64 t1, t2;
	printk("hello\n");

	/* guard against device removal before, or while,
	 * we issue this ioctl.
	 */
	nco_counter = filp->private_data;
	if (nco_counter == NULL)
		return -ENODATA;

	if (_IOC_NR(cmd) < 0)
		return -EINVAL;
	
	val = _IOC_NR(cmd);
	if (_IOC_DIR(cmd) & _IOC_READ) {
		ioc = 0;
		switch (val) {
		case REG_PINC:
			t1 = readl(nco_counter->membase + (REG_PINC_L << 2));
			t2 = readl(nco_counter->membase + (REG_PINC_H << 2));
			ioc = (t2 << 32) | (0xffffffff&t1);
			break;
		case REG_MAX_ACCUM:
			t1 = readl(nco_counter->membase + (REG_MAX_ACCUM_L << 2));
			t2 = readl(nco_counter->membase + (REG_MAX_ACCUM_H << 2));
			ioc = (t2 << 32) | (0xffffffff&t1);
			break;
		default:
			ioc = readl(nco_counter->membase + (val << 2));
		}
		printk("read %llu\n", ioc);
		if (put_user(ioc, (u64 __user*) arg))
			return -EACCES;
	}else {
		if (get_user(ioc, (u64 __user*) arg)) {
			printk("merdouille\n");
			return -EACCES;
		}

		printk("write %llu\n", ioc);
		switch (val) {
		case REG_PINC:
			writel(ioc & 0xffffffff, nco_counter->membase + (REG_PINC_L << 2));
			writel(ioc >> 32, nco_counter->membase + (REG_PINC_H << 2));
			break;
		case REG_MAX_ACCUM:
			writel(ioc & 0xffffffff, nco_counter->membase + (REG_MAX_ACCUM_L << 2));
			writel(ioc >> 32, nco_counter->membase + (REG_MAX_ACCUM_H << 2));
			break;
		default:
			writel(ioc, nco_counter->membase + (val << 2));
		}
	}
	return retval;
}

int nco_counter_open(struct inode *inode, struct file *filp)
{
	struct nco_counter_dev *pos, *data = NULL;
	/* Allocate and fill any data structure to be put in filp->private_data */
	list_for_each_entry(pos, &nco_counter_data_list, list) {
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

int nco_counter_release(struct inode *inode, struct file *filp)
{
	printk("nco_counter release\n");
	filp->private_data = NULL;
	return 0;
}

static struct file_operations nco_counter_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = nco_counter_ioctl,
	.open = nco_counter_open,
	.release = nco_counter_release,
};

static int nco_counter_probe(struct platform_device *pdev)
{
	struct plat_nco_counter_port *pdata = pdev->dev.platform_data;
	struct device_node *np = pdev->dev.of_node;
	int ret = 0;
	struct nco_counter_dev *sdev;
	struct resource *mem_res;

	if (pdata) {
		printk("%s probing %d\n", pdata->name, pdata->num);
	} else {
		if (!np)
			return -ENODEV;
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
	sdev = kzalloc(sizeof(struct nco_counter_dev), GFP_KERNEL);
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
		sdev->name =
		    (char *)kmalloc((1 + strlen(pdata->name)) * sizeof(char),
				    GFP_KERNEL);
	} else {
		sdev->name =
		    (char *)kmalloc(sizeof(char) * strlen(np->name) + 1,
				    GFP_KERNEL);
	}

	if (sdev->name == NULL) {
		dev_err(&pdev->dev, "Kmalloc name space error\n");
		goto out_iounmap;
	}

	if (pdata) {
		if (snprintf
		    (sdev->name, 1 + sizeof(pdata->name), "%s",
		     pdata->name) < 0) {
			printk("copy error");
			goto out_name_free;
		}
	} else {
		if (strncpy(sdev->name, np->name, 1 + strlen(np->name)) < 0) {
			printk("copy error");
			goto out_name_free;
		}
	}

	printk("name: %s %d %d\n", sdev->name, sizeof(sdev->name),
	       strlen(sdev->name));

	sdev->misc.name = sdev->name;
	sdev->misc.minor = MISC_DYNAMIC_MINOR;
	sdev->misc.fops = &nco_counter_fops,

	platform_set_drvdata(pdev, sdev);

	ret = misc_register(&sdev->misc);
	if (ret) {
		dev_err(&pdev->dev, KERN_ERR "%s:%u: misc_register failed %d\n",
		       __func__, __LINE__, ret);
		goto out_name_free;
	}

	dev_info(&pdev->dev, "%s: Add the device to the kernel, "
		 "connecting cdev to major/minor number \n", sdev->name);

	list_add(&sdev->list, &nco_counter_data_list);

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

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 11, 0)
static int nco_counter_remove(struct platform_device *pdev)
#else
static void nco_counter_remove(struct platform_device *pdev)
#endif
{
	struct nco_counter_dev *sdev;
	sdev = (struct nco_counter_dev *)platform_get_drvdata(pdev);

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

static struct of_device_id nco_counter_of_match[] = {
	{.compatible = "ggm,nco_counter",},
	{}
};

MODULE_DEVICE_TABLE(of, nco_counter_of_match);

static struct platform_driver plat_nco_counter_driver = {
	.probe = nco_counter_probe,
	.remove = nco_counter_remove,
	.driver = {
		   .name = "nco_counter",
		   .owner = THIS_MODULE,
		   .of_match_table = nco_counter_of_match,
   },
};

module_platform_driver(plat_nco_counter_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Gwenhael Goavec-Merou <gwenhael.goavec-merou@trabucayre.com>");
MODULE_DESCRIPTION("nco_counter IP generic driver");
