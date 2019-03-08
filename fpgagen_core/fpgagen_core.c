/*
 * Core driver for fpgagen IP
 *
 * (c) Copyright 2013-2019 	OscillatorIMP Digital
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

#include "fpgagen_config.h"

#define DRIVER_NAME "fpgagen"
#define FPGAGEN_ID 0x00

struct fpgagen_dev {
	char *name;			/* name of the instance */
	void *membase;		/* base address for instance  */
	struct resource *mem_res;
	struct device *dev;
	struct miscdevice misc;
	struct list_head list;
};

static LIST_HEAD(fpgagen_data_list);

static long fpgagen_ioctl(struct file *filp, unsigned int cmd,
			    unsigned long arg)
{
	int retval = 0;
	struct fpgagen_dev *fpgagen;
	u32 ioc;

	/* guard against device removal before, or while,
	 * we issue this ioctl.
	 */
	fpgagen = filp->private_data;
	if (fpgagen == NULL)
		return -ENODATA;
	
	if (_IOC_DIR(cmd) & _IOC_READ) {
		ioc = readl(fpgagen->membase + _IOC_NR(cmd));
		printk("%d\n",ioc);
		if (put_user(ioc, (u32 __user*) arg))
			return -EACCES;
	}else {
		printk("ioctl ");
		if (get_user(ioc, (u32 __user*) arg))
			return -EACCES;
		printk("%x %x (%d)\n", _IOC_NR(cmd), ioc, ioc);

		writel(ioc, fpgagen->membase + _IOC_NR(cmd));
	}
	return retval;
}

int fpgagen_open(struct inode *inode, struct file *filp)
{
	struct fpgagen_dev *pos, *data = NULL;
	/* Allocate and fill any data structure to be put in filp->private_data */
	list_for_each_entry(pos, &fpgagen_data_list, list) {
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

int fpgagen_release(struct inode *inode, struct file *filp)
{
	printk("fpgagen release\n");

	filp->private_data = NULL;

	return 0;
}

static struct file_operations fpgagen_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = fpgagen_ioctl,
	.open = fpgagen_open,
	.release = fpgagen_release,
};

static int fpgagen_probe(struct platform_device *pdev)
{
	struct plat_fpgagen_port *pdata = pdev->dev.platform_data;
	struct device_node *np = pdev->dev.of_node;
	int ret = 0;
	u16 ip_id;
	struct fpgagen_dev *sdev;
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
	sdev = kzalloc(sizeof(struct fpgagen_dev), GFP_KERNEL);
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

	/* check if ID is correct */
	ip_id = readl(sdev->membase + FPGAGEN_ID);
	printk("id : %d\n", ip_id);
	if (pdata) {
#ifndef USE_32Bits
		if (ip_id != pdata->idnum) {
			ret = -ENODEV;
			dev_err(&pdev->dev, "For %s id:%d doesn't match "
				 "with id read %d,\n is device present ?\n",
				 pdata->name, pdata->idnum, ip_id);
			goto out_iounmap;
		}
#endif
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
	printk("name: %s %d %d\n", sdev->name, sizeof(sdev->name), strlen(sdev->name));

	sdev->misc.name = sdev->name;
	sdev->misc.minor = MISC_DYNAMIC_MINOR;
	sdev->misc.fops = &fpgagen_fops,

	platform_set_drvdata(pdev, sdev);

	ret = misc_register(&sdev->misc);
	if (ret) {
		dev_err(&pdev->dev, KERN_ERR "%s:%u: misc_register failed %d\n",
		       __func__, __LINE__, ret);
		goto out_name_free;
	}
	dev_info(&pdev->dev, "%s: Add the device to the kernel, "
		 "connecting cdev to major/minor number \n", sdev->name);

	list_add(&sdev->list, &fpgagen_data_list);

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

static int fpgagen_remove(struct platform_device *pdev)
{
	struct fpgagen_dev *sdev;
	
	sdev = (struct fpgagen_dev *)platform_get_drvdata(pdev);

	misc_deregister(&sdev->misc);
	printk(KERN_INFO "%s: deleted with success\n", sdev->name);
	kfree(sdev->name);
	iounmap(sdev->membase);
	release_mem_region(sdev->mem_res->start, resource_size(sdev->mem_res));
	kfree(sdev);

	return 0;
}

static struct of_device_id fpgagen_of_match[] = {
	{ .compatible = "ggm,fpgagen", },
	{}
};

MODULE_DEVICE_TABLE(of, fpgagen_of_match);

static struct platform_driver plat_fpgagen_driver = {
	.probe = fpgagen_probe,
	.remove = fpgagen_remove,
	.driver = {
		   .name = "fpgagen",
		   .owner = THIS_MODULE,
		   .of_match_table = fpgagen_of_match,
   },
};

module_platform_driver(plat_fpgagen_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Gwenhael Goavec-Merou <gwenhael.goavec-merou@trabucayre.com>");
MODULE_DESCRIPTION("Wishbone fpgagen IP generic driver");
