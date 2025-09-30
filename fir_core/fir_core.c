/*
 * Generic driver for firReal and firComplex IPs
 *
 * (c) Copyright 2013-2018    OscillatorIMP Digital
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
#include <linux/version.h>
#include <linux/jiffies.h>

#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/fs.h>		/* for file  operations */
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/ioport.h>	/* request_mem_region */
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/timer.h>

#include <linux/slab.h>		/* kmalloc */

#include <linux/miscdevice.h>
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 12, 0)
#  include <asm/uaccess.h>	/* copy_to_user function */
#else
#  include <linux/uaccess.h>	/* copy_to_user function */
#endif
#include <linux/uaccess.h>
#include <asm/io.h>		/* readw() writew() */

#include "fir_config.h"

#define DRIVER_NAME "fir"

#define  FIR_REG_ID		(0x00 << 2)
#define  FIR_REG_COEFF 	(0x01 << 2)

struct fir_dev {
	char *name;		/* name of the instance */
	void *membase;		/* base address for instance  */
	struct resource *mem_res;
	struct device *dev;
	struct miscdevice misc;
	struct list_head list;
};

static LIST_HEAD(fir_data_list);

static ssize_t fir_write(struct file *filp, const char __user * buf,
			      size_t count, loff_t * pos)
{
	struct fir_dev *fir = filp->private_data;
	u32 *data;
	int i;

	data = (u32 *)kmalloc(count, GFP_KERNEL);

	if (data == NULL)
		return -ENOMEM;

	if (copy_from_user(data, buf, count))
		return -EFAULT;

	/* reset write counter */
	writew(data[0], fir->membase + FIR_REG_ID);
	/* send coeffs */
	for (i=0; i< count/sizeof(u32); i++)
		writew(data[i], fir->membase + FIR_REG_COEFF);

	kfree(data);

	return count;
}
static ssize_t fir_read(struct file *filp, char __user * buf,
			      size_t count, loff_t * pos)
{
	int i;
	struct fir_dev *fir = filp->private_data;
	u32 *data = (u32 *)kmalloc(count, GFP_KERNEL);

	if (data == NULL)
		return -ENOMEM;

	writew(data[0], fir->membase + FIR_REG_ID);
	for (i=0; i< count/sizeof(u32); i++)
		data[i] = readw(fir->membase + FIR_REG_COEFF);

	if (copy_to_user(buf, (u32 *)data, count ))
		return -EFAULT;
	kfree(data);
	return count;
}

int fir_open(struct inode *inode, struct file *filp)
{
	struct fir_dev *pos, *data = NULL;
	/* Allocate and fill any data structure to be put in filp->private_data */
	list_for_each_entry(pos, &fir_data_list, list) {
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

int fir_release(struct inode *inode, struct file *filp)
{
	printk("fir release\n");
	filp->private_data = NULL;
	return 0;
}

static struct file_operations fir_fops = {
	.owner = THIS_MODULE,
	.open = fir_open,
	.write= fir_write,
	.read = fir_read,
	.release = fir_release,
};

static int fir_probe(struct platform_device *pdev)
{
	struct plat_fir_port *pdata = pdev->dev.platform_data;
	struct device_node *np = pdev->dev.of_node;
	int ret = 0;
	int size_name;
	u16 ip_id;
	struct fir_dev *sdev;
	struct resource *mem_res;

	if (pdata)
		printk("%s probing %d\n", pdata->name, pdata->num);
	else {
		if (!np) {
			dev_err(&pdev->dev, "Platform data required !\n");
			return -ENODEV;
		}
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
	sdev = kzalloc(sizeof(struct fir_dev), GFP_KERNEL);
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

	ip_id = readl(sdev->membase + FIR_REG_ID);
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
	printk("name: %s %d %d\n",sdev->name, sizeof(sdev->name), strlen(sdev->name));

	sdev->misc.name = sdev->name;
	sdev->misc.minor = MISC_DYNAMIC_MINOR;
	sdev->misc.fops = &fir_fops;

	platform_set_drvdata(pdev, sdev);

	ret = misc_register(&sdev->misc);
	if (ret) {
		printk(KERN_ERR "%s:%u: misc_register failed %d\n",
		       __func__, __LINE__, ret);
		goto out_free_name;
	}

	dev_info(&pdev->dev, "%s: Add the device to the kernel, "
		 "connecting cdev to major/minor number \n", sdev->name);

	list_add(&sdev->list, &fir_data_list);

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

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 11, 0)
static int fir_remove(struct platform_device *pdev)
#else
static void fir_remove(struct platform_device *pdev)
#endif
{
	struct fir_dev *sdev;

	sdev = (struct fir_dev *)platform_get_drvdata(pdev);

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

static struct of_device_id fir_of_match[] = {
	{ .compatible = "ggm,fir", },
	{}
};

MODULE_DEVICE_TABLE(of, fir_of_match);

static struct platform_driver plat_fir_driver = {
	.probe = fir_probe,
	.remove = fir_remove,
	.driver = {
		   .name = "fir",
		   .owner = THIS_MODULE,
		   .of_match_table = fir_of_match,
   },
};

module_platform_driver(plat_fir_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Gwenhael Goavec-Merou <gwenhael.goavec-merou@trabucayre.com>");
MODULE_DESCRIPTION("fir Real/Complex IP generic driver");
