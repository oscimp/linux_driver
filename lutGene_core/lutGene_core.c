/*
 * Generic driver for lutGeneratorReal and lutGeneratorComplex IPs
 *
 * (c) Copyright 2023    OscillatorIMP Digital
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
#include <asm/io.h>		/* readl() writel() */

#include "lutGene_config.h"

#define DRIVER_NAME "lutGene"

#define LUTGENE_REG_LENGTH    (0x00 << 2)
#define LUTGENE_REG_PRESCALER (0x01 << 2)
#define LUTGENE_REG_ENABLE    (0x02 << 2)
#define LUTGENE_REG_DATA_I    (0x03 << 2)
#define LUTGENE_REG_DATA_Q    (0x04 << 2)
#define LUTGENE_REG_RST_ADDR  (0x05 << 2)
#define LUTEGENE_CMD_TYPE     (0xff     )

struct lutGene_dev {
	char *name;	         /* name of the instance */
	void *membase;       /* base address for instance */
	uint8_t output_type; /* 0: real, 1: complex */
	struct resource *mem_res;
	struct device *dev;
	struct miscdevice misc;
	struct list_head list;
};

static LIST_HEAD(lutGene_data_list);

static long lutGene_ioctl(struct file *filp, unsigned int cmd,
		unsigned long arg)
{
	int retval = 0;
	struct lutGene_dev *sdev;
	u32 ioc, reg;

	/* guard against device removal before, or while,
	 * we issue this ioctl.
	 */
	sdev = filp->private_data;
	if (!sdev)
		return -ENODATA;

	switch (_IOC_NR(cmd)) {
	case LUTGENE_RAM_LENGTH:
		reg = LUTGENE_REG_LENGTH;
		break;
	case LUTGENE_PRESCALER:
		reg = LUTGENE_REG_PRESCALER;
		break;
	case LUTGENE_ENABLE:
		reg = LUTGENE_REG_ENABLE;
		break;
	case LUTGENE_TYPE:
		reg = LUTEGENE_CMD_TYPE;
		break;
	default:
		return -EINVAL;
	}
	if (_IOC_DIR(cmd) & _IOC_READ) {
		if (reg == LUTEGENE_CMD_TYPE)
			ioc = sdev->output_type;
		else
			ioc = readl(sdev->membase + reg);
		if (put_user(ioc, (u32 __user *)arg))
			return -EACCES;
	} else {
		if (get_user(ioc, (u32 __user *)arg))
			return -EACCES;
		if (reg == LUTEGENE_CMD_TYPE)
			sdev->output_type = ioc;
		else
			writel(ioc, sdev->membase + reg);
	}
	return retval;
}

static ssize_t lutGene_write(struct file *filp, const char __user * buf,
			      size_t count, loff_t * pos)
{
	struct lutGene_dev *sdev = filp->private_data;
	u32 *data;
	int i;

	if (!sdev)
		return -ENODATA;

	data = (u32 *)kmalloc(count, GFP_KERNEL);

	if (data == NULL)
		return -ENOMEM;

	if (copy_from_user(data, buf, count))
		return -EFAULT;

	/* reset write counter */
	writel(data[0], sdev->membase + LUTGENE_REG_RST_ADDR);
	/* send coeffs */
	if (sdev->output_type == 0) { // real
		for (i=0; i< count/sizeof(u32); i++)
			writel(data[i], sdev->membase + LUTGENE_REG_DATA_I);
	} else { /* complex */
		printk("complex values %d\n", count / sizeof(u32));
		for (i=0; i< count/sizeof(u32); i+=2) {
			writel(data[i], sdev->membase + LUTGENE_REG_DATA_I);
			writel(data[i+1], sdev->membase + LUTGENE_REG_DATA_Q);
			if (i < 20) {
				printk("%d %d\n", data[i], data[i+1]);
			}
		}
	}

	kfree(data);

	return count;
}

int lutGene_open(struct inode *inode, struct file *filp)
{
	struct lutGene_dev *pos, *data = NULL;
	/* Allocate and fill any data structure to be put in filp->private_data */
	list_for_each_entry(pos, &lutGene_data_list, list) {
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

int lutGene_release(struct inode *inode, struct file *filp)
{
	filp->private_data = NULL;
	return 0;
}

static struct file_operations lutGene_fops = {
	.owner = THIS_MODULE,
	.write = lutGene_write,
	.unlocked_ioctl = lutGene_ioctl,
	.open = lutGene_open,
	.release = lutGene_release,
};

static int lutGene_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	int ret = 0;
	int size_name;
	struct lutGene_dev *sdev;
	struct resource *mem_res;

	if (!np) {
		dev_err(&pdev->dev, "Platform data required !\n");
		return -ENODEV;
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
	sdev = kzalloc(sizeof(struct lutGene_dev), GFP_KERNEL);
	if (!sdev) {
		ret = -ENOMEM;
		goto out_release_mem;
	}

	sdev->output_type = 1; // default complex type

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

	if (snprintf(sdev->name, size_name, "%s", np->name) < 0) {
		printk("copy error");
		goto out_free_name;
	}

	printk("name: %s %d %d\n",sdev->name, sizeof(sdev->name), strlen(sdev->name));

	sdev->misc.name = sdev->name;
	sdev->misc.minor = MISC_DYNAMIC_MINOR;
	sdev->misc.fops = &lutGene_fops;

	platform_set_drvdata(pdev, sdev);

	ret = misc_register(&sdev->misc);
	if (ret) {
		printk(KERN_ERR "%s:%u: misc_register failed %d\n",
		       __func__, __LINE__, ret);
		goto out_free_name;
	}

	dev_info(&pdev->dev, "%s: Add the device to the kernel, "
		 "connecting cdev to major/minor number \n", sdev->name);

	list_add(&sdev->list, &lutGene_data_list);

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

static int lutGene_remove(struct platform_device *pdev)
{
	struct lutGene_dev *sdev;

	sdev = (struct lutGene_dev *)platform_get_drvdata(pdev);

	misc_deregister(&sdev->misc);
	printk(KERN_INFO "%s: deleted with success\n", sdev->name);
	kfree(sdev->name);
	iounmap(sdev->membase);
	release_mem_region(sdev->mem_res->start, resource_size(sdev->mem_res));
	kfree(sdev);

	return 0;
}

static struct of_device_id lutGene_of_match[] = {
	{ .compatible = "ggm,lutGene", },
	{}
};

MODULE_DEVICE_TABLE(of, lutGene_of_match);

static struct platform_driver plat_lutGene_driver = {
	.probe = lutGene_probe,
	.remove = lutGene_remove,
	.driver = {
		   .name = "lutGene",
		   .owner = THIS_MODULE,
		   .of_match_table = lutGene_of_match,
   },
};

module_platform_driver(plat_lutGene_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Gwenhael Goavec-Merou <gwenhael.goavec-merou@trabucayre.com>");
MODULE_DESCRIPTION("lutGene Real/Complex IP generic driver");
