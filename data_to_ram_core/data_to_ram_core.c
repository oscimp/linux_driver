/*
 * Core driver for data_to_ram IP
 *
 * (c) Copyright 2018 	OscillatorIMP Digital
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

#include "data_to_ram_config.h"

#define DRIVER_NAME "data_to_ram"

#define  DCTR_REG_START  	(0x00 << 2)
#define  DCTR_REG_STATUS	(0x01 << 2)
#define  DCTR_REG_DATA		(0x02 << 2)

struct data_to_ram_dev {
	char *name;		/* name of the instance */
	void *membase;		/* base address for instance  */
	struct resource *mem_res;
	struct mutex buf_lock;
	struct device *dev;
	struct miscdevice misc;
	struct list_head list;
	int wait_read;
	/* irq */
	u8 use_irq;
	int irq_res;
	wait_queue_head_t	wq;
	bool				irq_handled;
};

static LIST_HEAD(data_to_ram_data_list);

static irqreturn_t data_to_ram_irq(int irq, void *data)
{

	struct data_to_ram_dev *dev = (struct data_to_ram_dev *)data;
	printk("IT!\n");
	if (dev == NULL) {
		printk("problem de dev null\n");
		return IRQ_HANDLED;
	}
	dev->irq_handled = true;
	wake_up_interruptible(&dev->wq);

	return IRQ_HANDLED;
}

static ssize_t data_to_ram_read(struct file *filp,
				char __user * ubuf, size_t count,
				loff_t * f_pos)
{
	int retval = count;
	int counter = count / sizeof(u32);
	struct data_to_ram_dev *data_dev = filp->private_data;
	u32 *data = NULL;
	int i;
	u32 value;
	int timeout;
	ktime_t timeout_irq = ktime_set(40, 0);
	int ret;

	data = kmalloc(retval, GFP_KERNEL);
	if (data == NULL)
		return -ENOMEM;

	if (data_dev->wait_read == 0) {
		data_dev->irq_handled = false;
		writel(0x01, data_dev->membase + DCTR_REG_START);
	} else {
		data_dev->wait_read=0;
	}

	if (data_dev->use_irq) {
		/* wait until interrupt happened */
		ret = wait_event_interruptible_hrtimeout(data_dev->wq,
				data_dev->irq_handled, timeout_irq);
		if (ret == -ETIME) {
			dev_alert(data_dev->misc.this_device, "  timeout!\n");
			return -ETIME;
		} else if (ret) {
			dev_alert(data_dev->misc.this_device, "  error=%d\n", ret);
			return -ERESTARTSYS;
		}
		data_dev->irq_handled = false;
	} else {

		timeout = 0;
		do {
			value = readl(data_dev->membase + DCTR_REG_STATUS);
			if (timeout == 1000000000) {
				printk("timeout\n");
				retval = -EFAULT;
				goto out_free;
			} else
				timeout++;
		} while ((value & 0x01) != 0);
	}

	for (i = 0; i < counter; i++)
		data[i] = readl(data_dev->membase + DCTR_REG_DATA);

	if (copy_to_user(ubuf, (u32 *) data, retval)) {
		printk(KERN_WARNING "read : copy to user data error\n");
		retval = -EFAULT;
		goto out_free;
	}

 out_free:
	kfree(data);
	return retval;

}

static long data_to_ram_ioctl(struct file *filp, unsigned int cmd,
			      unsigned long arg)
{
	int retval = 0;
	struct data_to_ram_dev *data_dev;
	u32 ioc;

	/* guard against device removal before, or while,
	 * we issue this ioctl.
	 */
	data_dev = filp->private_data;
	if (data_dev == NULL)
		return -ENODATA;

	if (_IOC_DIR(cmd) & _IOC_READ) {
		ioc = readl(data_dev->membase + (_IOC_NR(cmd) << 2));
		printk("%u\n", ioc);
		if (put_user(ioc, (u32 __user *) arg))
			return -EACCES;
	} else {
		printk("ioctl ");
		if (get_user(ioc, (u32 __user *) arg))
			return -EACCES;
		printk("%x %x (%u)\n", _IOC_NR(cmd), ioc, ioc);
		if (_IOC_NR(cmd) == DCTR_START) {
			writel(0x01, data_dev->membase + DCTR_REG_START);
			data_dev->wait_read = 1;
			data_dev->irq_handled = false;
		}
	}
	return retval;
}

int data_to_ram_open(struct inode *inode, struct file *filp)
{
	struct data_to_ram_dev *pos, *data = NULL;
	/* Allocate and fill any data structure to be put in filp->private_data */
	list_for_each_entry(pos, &data_to_ram_data_list, list) {
		if (pos->misc.minor == iminor(inode)) {
			data = pos;
			break;
		}
	}
	if (data == NULL)
		return -ENODATA;

	filp->private_data = (struct device *)data;
	data->wait_read = 0;

	return 0;
}

int data_to_ram_release(struct inode *inode, struct file *filp)
{
	printk("data_to_ram release\n");
	filp->private_data = NULL;
	return 0;
}

static struct file_operations data_to_ram_fops = {
	.owner = THIS_MODULE,
	.read = data_to_ram_read,
	.unlocked_ioctl = data_to_ram_ioctl,
	.open = data_to_ram_open,
	.release = data_to_ram_release,
};

static int data_to_ram_probe(struct platform_device *pdev)
{
	struct plat_data_to_ram_port *pdata = pdev->dev.platform_data;
	struct device_node *np = pdev->dev.of_node;
	int ret = 0;
	struct data_to_ram_dev *sdev;
	struct resource *mem_res;
	size_t size_name;

	if (pdata) {
		printk("%s probing %d\n", pdata->name, pdata->num);
	} else {
		if (!np)
			return -ENODEV;
		printk("probing %s with dts\n", np->name);
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
	sdev = kzalloc(sizeof(struct data_to_ram_dev), GFP_KERNEL);
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
		size_name = sizeof(char) * strlen(pdata->name) + 1;
	} else {
		size_name = sizeof(char) * strlen(np->name) + 1;
	}
	sdev->name = (char *)kmalloc(size_name, GFP_KERNEL);

	if (sdev->name == NULL) {
		dev_err(&pdev->dev, "Kmalloc name space error\n");
		goto out_iounmap;
	}

	if (pdata) {
		if (snprintf(sdev->name, size_name, "%s", pdata->name) < 0) {
			printk("copy error");
			goto out_free_name;
		}
	} else {
		if (strncpy(sdev->name, np->name, size_name) < 0) {
			printk("copy error");
			goto out_free_name;
		}
	}

	printk("name: %s %d %d\n", sdev->name, sizeof(sdev->name),
	       strlen(sdev->name));

	sdev->misc.name = sdev->name;
	sdev->misc.minor = MISC_DYNAMIC_MINOR;
	sdev->misc.fops = &data_to_ram_fops;

	platform_set_drvdata(pdev, sdev);

	ret = misc_register(&sdev->misc);
	if (ret) {
		printk(KERN_ERR "%s:%u: misc_register failed %d\n",
		       __func__, __LINE__, ret);
		goto out_free_name;
	}

	dev_info(&pdev->dev, "%s: Add the device to the kernel, "
		 "connecting cdev to major/minor number \n", sdev->name);

	/* irq */
	/* get IRQ 0 'dmatest' from device tree */
	sdev->irq_res = platform_get_irq(pdev, 0);
	if (sdev->irq_res > 0) {
		sdev->use_irq = 1;
		sdev->irq_handled = false;
		init_waitqueue_head(&sdev->wq);
		printk("irq res %d\n", sdev->irq_res);

		if (devm_request_irq(&pdev->dev, sdev->irq_res, data_to_ram_irq,
					IRQF_TRIGGER_RISING, "complex16ToRam", sdev)) {
			printk(KERN_ERR "Couldn't allocate IRQ (%d)\n", sdev->irq_res);
			goto out_free_name;
		}
		printk("irq ok : %d\n", sdev->irq_res);
	} else {
		sdev->use_irq = 0;
	}

	list_add(&sdev->list, &data_to_ram_data_list);

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

static int data_to_ram_remove(struct platform_device *pdev)
{
	struct data_to_ram_dev *sdev;
	sdev = (struct data_to_ram_dev *)platform_get_drvdata(pdev);

	if (sdev->use_irq)
		devm_free_irq(&pdev->dev, sdev->irq_res, sdev);

	misc_deregister(&sdev->misc);
	printk(KERN_INFO "%s: deleted with success\n", sdev->name);
	kfree(sdev->name);
	iounmap(sdev->membase);
	release_mem_region(sdev->mem_res->start, resource_size(sdev->mem_res));
	kfree(sdev);

	return 0;
}

static struct of_device_id data_to_ram_of_match[] = {
	{.compatible = "ggm,dataToRam",},
	{}
};

MODULE_DEVICE_TABLE(of, data_to_ram_of_match);

static struct platform_driver plat_data_to_ram_driver = {
	.probe = data_to_ram_probe,
	.remove = data_to_ram_remove,
	.driver = {
		   .name = "dataToRam",
		   .owner = THIS_MODULE,
		   .of_match_table = data_to_ram_of_match,
		   },
};

module_platform_driver(plat_data_to_ram_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Gwenhael Goavec-Merou <gwenhael.goavec-merou@trabucayre.com>");
MODULE_DESCRIPTION("data_to_ram IP core driver");
