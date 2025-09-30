/*
 * Generic driver for pidv3_axi IP
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
#include <linux/version.h>

#include <linux/slab.h>		/* kmalloc */

#include <linux/miscdevice.h>
#include <asm/uaccess.h>	/* copy_to_user function */
#include <asm/io.h>		/* readw() writew() */

#include "pidv3_axi_config.h"

#define DRIVER_NAME "pidv3_axi"

#define USE_32Bits

/*#define PIDV3_AXI_REG_KP		(0x00 << 2)
#define PIDV3_AXI_REG_KI		(0x01 << 2)
#define PIDV3_AXI_REG_KD		(0x02 << 2)
#define PIDV3_AXI_REG_SETPOINT	(0x03 << 2)
#define PIDV3_AXI_REG_SIGN		(0x04 << 2)
#define PIDV3_AXI_REG_INT_RST	(0x05 << 2)
#define PIDV3_AXI_REG_INPUT		(0x06 << 2)*/

#define PIDV3_AXI_SETPOINT_BIT	(0x1 << 0)
#define PIDV3_AXI_KP_BIT		(0x1 << 1)
#define PIDV3_AXI_KI_BIT		(0x1 << 2)
#define PIDV3_AXI_KD_BIT		(0x1 << 3)
#define PIDV3_AXI_SIGN_BIT		(0x1 << 4)
#define PIDV3_AXI_INT_RST_BIT	(0x1 << 5)



struct pidv3_axi_dev {
	char *name;		/* name of the instance */
	void *membase;		/* base address for instance  */
	struct resource *mem_res;
	struct device *dev;
	struct miscdevice misc;
	struct list_head list;
};

static LIST_HEAD(pidv3_axi_data_list);

static long pidv3_axi_ioctl(struct file *filp, unsigned int cmd,
			    unsigned long arg)
{
	int i, ii;
	int retval = 0;
	struct pidv3_axi_dev *pidv3_axi;
	u32 ioc, reg, tmp, mask;

	/* guard against device removal before, or while,
	 * we issue this ioctl.
	 */
	pidv3_axi = filp->private_data;
	if (pidv3_axi == NULL)
		return -ENODATA;

	reg = (_IOC_NR(cmd) << 2);

	if (_IOC_DIR(cmd) & _IOC_READ) {
		ioc = readl(pidv3_axi->membase + reg);
		if (put_user(ioc, (u32 __user *) arg))
			return -EACCES;
	} else {
		if (get_user(ioc, (u32 __user*) arg))
			return -EACCES;
		tmp = ioc;
		/* for REG_INPUT we must change only mandatory part */
		if (reg == REG_PIDV3_AXI_INPUT) {
			tmp = readl(pidv3_axi->membase + reg);
			for (i=0, ii=0; i < 6; i++, ii+=2) {
				mask = (ioc >> ii) & 0x3;
				if (mask != 0) {
					if (mask == 1)
						tmp |= (1 << i);
					else
						tmp &= ~(1 << i);
				}
			}
		}
		writel(tmp, pidv3_axi->membase + reg);
	}
	return retval;
}

int pidv3_axi_open(struct inode *inode, struct file *filp)
{
	struct pidv3_axi_dev *pos, *data = NULL;
	/* Allocate and fill any data structure to be put in filp->private_data */
	list_for_each_entry(pos, &pidv3_axi_data_list, list) {
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

int pidv3_axi_release(struct inode *inode, struct file *filp)
{
	printk("pidv3_axi release\n");
	filp->private_data = NULL;
	return 0;
}

static struct file_operations pidv3_axi_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = pidv3_axi_ioctl,
	.open = pidv3_axi_open,
	.release = pidv3_axi_release,
};

static int pidv3_axi_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	int ret = 0;
	int size_name;
	struct pidv3_axi_dev *sdev;
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
	sdev = kzalloc(sizeof(struct pidv3_axi_dev), GFP_KERNEL);
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
	sdev->misc.fops = &pidv3_axi_fops,

	platform_set_drvdata(pdev, sdev);

	ret = misc_register(&sdev->misc);
	if (ret) {
		dev_err(&pdev->dev, KERN_ERR "%s:%u: misc_register failed %d\n",
		       __func__, __LINE__, ret);
		goto out_name_free;
	}

	dev_info(&pdev->dev, "%s: Add the device to the kernel, "
		 "connecting cdev to major/minor number \n", sdev->name);

	list_add(&sdev->list, &pidv3_axi_data_list);

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
static int pidv3_axi_remove(struct platform_device *pdev)
#else
static void pidv3_axi_remove(struct platform_device *pdev)
#endif
{
	struct pidv3_axi_dev *sdev;

	sdev = (struct pidv3_axi_dev *)platform_get_drvdata(pdev);

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

static struct of_device_id pidv3_axi_of_match[] = {
	{ .compatible = "ggm,pidv3_axi", },
	{}
};

MODULE_DEVICE_TABLE(of, pidv3_axi_of_match);

static struct platform_driver plat_pidv3_axi_driver = {
	.probe = pidv3_axi_probe,
	.remove = pidv3_axi_remove,
	.driver = {
		   .name = "pidv3_axi",
		   .owner = THIS_MODULE,
		   .of_match_table = pidv3_axi_of_match,
   },
};

module_platform_driver(plat_pidv3_axi_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Gwenhael Goavec-Merou <gwenhael.goavec-merou@trabucayre.com>");
MODULE_DESCRIPTION("pidv3_axi IP generic driver");
