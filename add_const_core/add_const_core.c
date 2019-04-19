/*
 * Generic driver for Wishbone add_const IP
 *
 * (c) Copyright 2008-2011    The Armadeus Project - ARMadeus Systems
 * Fabien Marteau <fabien.marteau@armadeus.com>
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

#include "add_const_config.h"
//#include <common.h>
typedef unsigned long data_type;
#define writefpga(__value, __addr) writel(__value, __addr);
#define readfpga(__addr) readl(__addr)

#define DRIVER_NAME "add_const"

#ifdef USE_32Bits
#define ADD_CONST_REG_ID			(0x00 << 2)
#define ADD_CONST_REG_OFFSET_L	(0x01 << 2)
#define ADD_CONST_REG_OFFSET_H	(0x02 << 2)
#else
#define ADD_CONST_REG_ID			(0x00 << 1)
#define ADD_CONST_REG_OFFSET_L	(0x01 << 1)
#define ADD_CONST_REG_OFFSET_H	(0x02 << 1)
#endif

struct add_const_dev {
	char *name;			/* name of the instance */
	void *membase;		/* base address for instance  */
	struct resource *mem_res;
	struct device *dev;
	struct miscdevice misc;
	struct list_head list;
};

static LIST_HEAD(add_const_data_list);

static long add_const_ioctl(struct file *filp, unsigned int cmd,
			    unsigned long arg)
{
	int retval = 0;
	struct add_const_dev *add_const;
	s64 ioc;
	s32 offset_l;
	s32 offset_h;

	/* guard against device removal before, or while,
	 * we issue this ioctl.
	 */
	add_const = filp->private_data;
	if (add_const == NULL)
		return -ENODATA;

	if (_IOC_DIR(cmd) & _IOC_READ) {
		ioc = 0;
		offset_l = readfpga(add_const->membase + ADD_CONST_REG_OFFSET_L);
		offset_h = readfpga(add_const->membase + ADD_CONST_REG_OFFSET_H);
		printk("%x %x\n", offset_h, offset_l);
		//ioc = ((((s64)offset_h) << 32)&0xffffffff00000000) || (0x00000000ffffffff & (s64)offset_l);
		ioc = ((s64)offset_h) << 32;
		ioc |= offset_l;
		printk("%Ld\n", ioc);
		if (put_user(ioc, (s64 __user*) arg))
			return -EACCES;
	}else {
		if (get_user(ioc, (s64 __user*) arg))
			return -EACCES;
		offset_l = (s32)ioc & 0x00000000ffffffff;
		offset_h = (s32)(ioc >> 32) & 0x00000000ffffffff;
		printk("write : %x %x\n", offset_h, offset_l);
		writefpga(offset_l, add_const->membase + ADD_CONST_REG_OFFSET_L);
		writefpga(offset_h, add_const->membase + ADD_CONST_REG_OFFSET_H);
	}

	return retval;
}

int add_const_open(struct inode *inode, struct file *filp)
{
	struct add_const_dev *pos, *data = NULL;
	/* Allocate and fill any data structure to be put in filp->private_data */
	list_for_each_entry(pos, &add_const_data_list, list) {
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

int add_const_release(struct inode *inode, struct file *filp)
{
	printk("add_const release\n");
	filp->private_data = NULL;
	return 0;
}

static struct file_operations add_const_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = add_const_ioctl,
	.open = add_const_open,
	.release = add_const_release,
};

static int add_const_probe(struct platform_device *pdev)
{
	struct plat_add_const_port *pdata = pdev->dev.platform_data;
	struct device_node *np = pdev->dev.of_node;
	int ret = 0;
	int size_name;
	u16 ip_id;
	struct add_const_dev *sdev;
	struct resource *mem_res;

	if (pdata)
		printk("%s probing %d\n", pdata->name, pdata->num);
	else {
		if (!np) {
			dev_err(&pdev->dev, "Platform data required !\n");
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
	sdev = kzalloc(sizeof(struct add_const_dev), GFP_KERNEL);
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

	ip_id = readl(sdev->membase + ADD_CONST_REG_ID);
	printk("ip_id : %d\n",ip_id);

	if (pdata) {
		pdata->sdev = sdev;
		size_name = (1 + strlen(pdata->name)) * sizeof(char);
	} else {
		size_name = (1 + strlen(np->name)) * sizeof(char);
	}

	sdev->name = (char *)kmalloc(size_name, GFP_KERNEL);

	if (sdev->name == NULL) {
		dev_err(&pdev->dev, "Kmalloc name space error\n");
		goto out_iounmap;
	}

	if (pdata) {
		if (snprintf(sdev->name, size_name, "%s", pdata->name) < 0) {
			dev_err(&pdev->dev, "copy error");
			goto out_free_name;
		}
	} else {
		if (snprintf(sdev->name, size_name, "%s", np->name) < 0) {
			printk("copy error");
			goto out_free_name;
		}
	}

	sdev->misc.name = sdev->name;
	sdev->misc.minor = MISC_DYNAMIC_MINOR;
	sdev->misc.fops = &add_const_fops,

	platform_set_drvdata(pdev, sdev);

	ret = misc_register(&sdev->misc);
	if (ret) {
		dev_err(&pdev->dev, KERN_ERR "%s:%u: misc_register failed %d\n",
		       __func__, __LINE__, ret);
		goto out_free_name;
	}

	dev_info(&pdev->dev, "%s: Add the device to the kernel, "
		 "connecting cdev to major/minor number \n", sdev->name);

	list_add(&sdev->list, &add_const_data_list);

	/* OK driver ready ! */
	dev_info(&pdev->dev, KERN_INFO "%s loaded\n", sdev->name);
	return 0;

 out_free_name:
	kfree(sdev->name);
 out_iounmap:
	iounmap(sdev->membase);
 out_dev_free:
	kfree(sdev);
 out_release_mem:
	release_mem_region(mem_res->start, resource_size(mem_res));

	return ret;
}

static int add_const_remove(struct platform_device *pdev)
{
	struct add_const_dev *sdev;
	sdev = (struct add_const_dev *)platform_get_drvdata(pdev);

	misc_deregister(&sdev->misc);
	printk(KERN_INFO "%s: deleted with success\n", sdev->name);
	kfree(sdev->name);
	iounmap(sdev->membase);
	release_mem_region(sdev->mem_res->start, resource_size(sdev->mem_res));
	kfree(sdev);

	return 0;
}

static struct of_device_id add_const_of_match[] = {
	{.compatible = "ggm,add_const",},
	{}
};

MODULE_DEVICE_TABLE(of, add_const_of_match);

static struct platform_driver plat_add_const_driver = {
	.probe = add_const_probe,
	.remove = add_const_remove,
	.driver = {
		   .name = "add_const",
		   .owner = THIS_MODULE,
		   .of_match_table = add_const_of_match,
		   },
};

module_platform_driver(plat_add_const_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Gwenhael Goavec-Merou <gwenhael.goavec-merou@trabucayre.com>");
MODULE_DESCRIPTION("add_const IP generic driver");
