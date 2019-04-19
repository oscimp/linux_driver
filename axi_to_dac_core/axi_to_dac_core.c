/*
 * Generic driver for axi_to_dac IP
 *
 * (c) Copyright 2018
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

#include "axi_to_dac_config.h"
#include "../common.h"

#define DRIVER_NAME "axi_to_dac"

#ifdef USE_32Bits
#define AXI_TO_DAC_REG_ID		(0x00 << 2)
#define AXI_TO_DAC_REG_DACA	(0x01 << 2)
#define AXI_TO_DAC_REG_DACB	(0x02 << 2)
#else
#define AXI_TO_DAC_REG_ID		(0x00 << 1)
#define AXI_TO_DAC_REG_DACA	(0x01 << 1)
#define AXI_TO_DAC_REG_DACB	(0x02 << 1)
#endif

struct axi_to_dac_dev {
	char *name;		/* name of the instance */
	void *membase;		/* base address for instance  */
	struct resource *mem_res;
	struct device *dev;
	struct miscdevice misc;
	struct list_head list;
};

static LIST_HEAD(axi_to_dac_data_list);

static long axi_to_dac_ioctl(struct file *filp, unsigned int cmd,
			    unsigned long arg)
{
	int retval = 0;
	struct axi_to_dac_dev *axi_to_dac;
	data_type ioc;  // defini dans ../common.h

	/* guard against device removal before, or while,
	 * we issue this ioctl.
	 */
	axi_to_dac = filp->private_data;
	if (axi_to_dac == NULL)
		return -ENODATA;

	if (get_user(ioc, (data_type __user*) arg))
		return -EACCES;
	if (_IOC_NR(cmd) == 0){
		printk("daca %lu", ioc);
		writefpga(ioc, axi_to_dac->membase + AXI_TO_DAC_REG_DACA);
		printk("check: %d", readfpga(axi_to_dac->membase + AXI_TO_DAC_REG_DACA));
	} else {
		printk("dacb %lu\n",ioc);
		writefpga(ioc, axi_to_dac->membase + AXI_TO_DAC_REG_DACB);
		printk("check: %d", readfpga(axi_to_dac->membase + AXI_TO_DAC_REG_DACB));
	}
	return retval;
}

int axi_to_dac_open(struct inode *inode, struct file *filp)
{
	struct axi_to_dac_dev *pos, *data = NULL;
	/* Allocate and fill any data structure to be put in filp->private_data */
	list_for_each_entry(pos, &axi_to_dac_data_list, list) {
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

int axi_to_dac_release(struct inode *inode, struct file *filp)
{
	printk("axi_to_dac release\n");
	filp->private_data = NULL;
	return 0;
}

static struct file_operations axi_to_dac_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = axi_to_dac_ioctl,
	.open = axi_to_dac_open,
	.release = axi_to_dac_release,
};

static int axi_to_dac_probe(struct platform_device *pdev)
{
	struct plat_axi_to_dac_port *pdata = pdev->dev.platform_data;
	struct device_node *np = pdev->dev.of_node;
	int ret = 0;
	int size_name;
	u16 ip_id;
	struct axi_to_dac_dev *sdev;
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
	sdev = kzalloc(sizeof(struct axi_to_dac_dev), GFP_KERNEL);
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
	ip_id = readfpga(sdev->membase + AXI_TO_DAC_REG_ID);
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
		pdata->sdev = sdev;
#endif
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
			printk("copy error");
			goto out_name_free;
		}
	} else {
		if (strncpy(sdev->name, np->name, 1+strlen(np->name)) < 0) {
			printk("copy error");
			goto out_name_free;
		}
	}
	printk("name: %s %d %d\n",sdev->name, sizeof(sdev->name), strlen(sdev->name));

	sdev->misc.name = sdev->name;
	sdev->misc.minor = MISC_DYNAMIC_MINOR;
	sdev->misc.fops = &axi_to_dac_fops,

	platform_set_drvdata(pdev, sdev);

	ret = misc_register(&sdev->misc);
	if (ret) {
		dev_err(&pdev->dev, KERN_ERR "%s:%u: misc_register failed %d\n",
		       __func__, __LINE__, ret);
		goto out_name_free;
	}

	dev_info(&pdev->dev, "%s: Add the device to the kernel, "
		 "connecting cdev to major/minor number \n", sdev->name);

	list_add(&sdev->list, &axi_to_dac_data_list);

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

static int axi_to_dac_remove(struct platform_device *pdev)
{
	struct axi_to_dac_dev *sdev;

	sdev = (struct axi_to_dac_dev *)platform_get_drvdata(pdev);

	misc_deregister(&sdev->misc);
	printk(KERN_INFO "%s: deleted with success\n", sdev->name);
	kfree(sdev->name);
	iounmap(sdev->membase);
	release_mem_region(sdev->mem_res->start, resource_size(sdev->mem_res));
	kfree(sdev);

	return 0;
}

static struct of_device_id axi_to_dac_of_match[] = {
	{ .compatible = "ggm,axi_to_dac", },
	{}
};

MODULE_DEVICE_TABLE(of, axi_to_dac_of_match);

static struct platform_driver plat_axi_to_dac_driver = {
	.probe = axi_to_dac_probe,
	.remove = axi_to_dac_remove,
	.driver = {
		   .name = "axi_to_dac",
		   .owner = THIS_MODULE,
		   .of_match_table = axi_to_dac_of_match,
   },
};

module_platform_driver(plat_axi_to_dac_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Gwenhael Goavec-Merou <gwenhael.goavec-merou@armadeus.com>");
MODULE_DESCRIPTION("Wishbone axi_to_dac IP generic driver");
