/*
 * Generic driver for fft IP
 *
 * (c) Copyright 2013-2018    OscillatorIMP Digital
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

#include "fft_config.h"
//#include <common.h>
typedef unsigned long data_type;
#define writefpga(__value, __addr) writel(__value, __addr);
#define readfpga(__addr) readl(__addr)


#define DRIVER_NAME "fft"

#define  FFT_REG_ID		(0x00 << 2)
#define  FFT_REG_RE_COEFF  	(0x01 << 2)
#define  FFT_REG_IM_COEFF  	(0x02 << 2)
#define  FFT_REG_TEST_DATA 	(0x03 << 2)

#define TEST_DATA
struct fft_dev {
	char *name;		/* name of the instance */
	void *membase;		/* base address for instance  */
	struct resource *mem_res;
	struct device *dev;
	struct miscdevice misc;
	struct list_head list;
};

static LIST_HEAD(fft_data_list);

static ssize_t fft_write(struct file *filp, const char __user * buf,
			      size_t count, loff_t * pos)
{
	struct fft_dev *fft = filp->private_data;
	size_t size = count / sizeof(u32);
	int i;
	ssize_t retval;
	u32 *data;

	retval = count;

	data = kmalloc(retval, GFP_KERNEL);

	if (copy_from_user(data, buf, retval)) {
		printk("erreur lors de copie from user\n");
		retval = -EFAULT;
		goto end;
	}

	for (i=0; i < size; i+=2){
		writefpga(data[i], fft->membase + FFT_REG_IM_COEFF);
		writel(data[i+1], fft->membase + FFT_REG_RE_COEFF);
	}
	*pos += retval;

end:
	kfree(data);
	return retval;
}

static ssize_t fft_read(struct file *filp, 
			char __user *ubuf, size_t count, loff_t *f_pos)
{
	struct fft_dev *fft = filp->private_data;
	int retval = count;
	int counter = count/sizeof(u32);
	int i;
	u32 *data = NULL;

	data = kmalloc(retval, GFP_KERNEL);
	if (data == NULL)
		return -ENOMEM;

	readfpga(fft->membase + FFT_REG_ID);
#ifdef TEST_DATA
	for (i=0; i< counter; i++) {
		data[i] = readl(fft->membase + FFT_REG_TEST_DATA); 
	}
#else
	for (i=0; i< counter; i+=2) {
		data[i] = readl(fft->membase + FFT_REG_IM_COEFF); 
		data[i+1] = readl(fft->membase + FFT_REG_RE_COEFF); 
	//	if (i < 10)
	//		printk("%d %d\n", data[i], data[i+1]);
	}
#endif

	if (copy_to_user(ubuf, (u32 *) data, retval)) {
		printk(KERN_WARNING "read : copy to user data error\n");
		retval = -EFAULT;
	}

	kfree(data);
	return retval;
}

int fft_open(struct inode *inode, struct file *filp)
{
	struct fft_dev *pos, *data = NULL;
	/* Allocate and fill any data structure to be put in filp->private_data */
	list_for_each_entry(pos, &fft_data_list, list) {
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

int fft_release(struct inode *inode, struct file *filp)
{
	printk("fft release\n");
	filp->private_data = NULL;
	return 0;
}

static struct file_operations fft_fops = {
	.owner = THIS_MODULE,
	.open = fft_open,
	.write = fft_write,
	.read = fft_read,
	.release = fft_release,
};

static int fft_probe(struct platform_device *pdev)
{
	struct plat_fft_port *pdata = pdev->dev.platform_data;
	struct device_node *np = pdev->dev.of_node;
	int ret = 0;
	int size_name;
	u16 ip_id;
	struct fft_dev *sdev;
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
	sdev = kzalloc(sizeof(struct fft_dev), GFP_KERNEL);
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

	ip_id = readl(sdev->membase + FFT_REG_ID);
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
	sdev->misc.fops = &fft_fops;

	platform_set_drvdata(pdev, sdev);

	ret = misc_register(&sdev->misc);
	if (ret) {
		printk(KERN_ERR "%s:%u: misc_register failed %d\n",
		       __func__, __LINE__, ret);
		goto out_free_name;
	}

	dev_info(&pdev->dev, "%s: Add the device to the kernel, "
		 "connecting cdev to major/minor number \n", sdev->name);

	list_add(&sdev->list, &fft_data_list);

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

static int fft_remove(struct platform_device *pdev)
{
	struct fft_dev *sdev;

	sdev = (struct fft_dev *)platform_get_drvdata(pdev);

	misc_deregister(&sdev->misc);
	printk(KERN_INFO "%s: deleted with success\n", sdev->name);
	kfree(sdev->name);
	iounmap(sdev->membase);
	release_mem_region(sdev->mem_res->start, resource_size(sdev->mem_res));
	kfree(sdev);

	return 0;
}

static struct of_device_id fft_of_match[] = {
	{ .compatible = "ggm,fft", },
	{}
};

MODULE_DEVICE_TABLE(of, fft_of_match);

static struct platform_driver plat_fft_driver = {
	.probe = fft_probe,
	.remove = fft_remove,
	.driver = {
		   .name = "fft",
		   .owner = THIS_MODULE,
		   .of_match_table = fft_of_match,
   },
};

module_platform_driver(plat_fft_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("OscimpDigital");
MODULE_DESCRIPTION("fft IP generic driver");
