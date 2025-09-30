/*
 * Core driver for data_dma_direct IP
 *
 * (c) Copyright 2020 	OscillatorIMP Digital
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
#include <linux/mutex.h>
#include <linux/fs.h>		/* for file  operations */
#include <linux/init.h>
#include <linux/irq.h>
#include <linux/ioport.h>	/* request_mem_region */
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/timer.h>
#include <linux/version.h>
#include <linux/semaphore.h>
#include <linux/slab.h>		/* kmalloc */
#include <linux/spinlock.h>
#include <linux/miscdevice.h>
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 12, 0)
#  include <asm/uaccess.h>	/* copy_to_user function */
#else
#  include <linux/uaccess.h>	/* copy_to_user function */
#endif
#include <asm/io.h>		/* readw() writew() */
#include <linux/dma-mapping.h>
#include <linux/ioctl.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>

#include "data_dma_direct_config.h"

#define DRIVER_NAME "dataDMADirect"

#define IRQ 1
//#undef IRQ

#define VERBOSE 1
#undef VERBOSE

#define REG_DMA_DIRECT_CONTROL (0x30)
#define REG_DMA_DIRECT_STATUS (0x34)
#define REG_DMA_DIRECT_ADDR (0x48)
#define REG_DMA_DIRECT_LENGTH (0x58)

struct dataDmaDirect_dev {
	char *name;		/* name of the instance */
	void *membase_ip;		/* base address for instance  */
	void *membase;		/* base address for instance  */
	struct device_node      *devnode;
	struct resource *mem_res;
	struct resource *mem_res_ip;
	struct platform_device *pdev;
	int irq_res;
	struct device *dev;
	struct miscdevice misc;
	struct list_head list;
	dma_addr_t ram_buff;
	void* virt_addr;
	struct semaphore sema_read;
	uint32_t *data_buff;
	wait_queue_head_t	wq;
	bool				irq_handled;
	int buff_size;
	u32 ip_buffer_byte_size;
};

DEFINE_MUTEX(mutex_complex);

static LIST_HEAD(dataDmaDirect_data_list);

static void printStatus(unsigned long status)
{
    printk("%lx\n", status);
    if (status & 0x01) printk("halted ");
    if (status & (0x01<<1)) printk("idle ");
    if (status & (0x01<<3)) printk("SGInc ");
    if (status & (0x01<<4)) printk("DMAInErr ");
    if (status & (0x01<<5)) printk("DMASlvErr ");
    if (status & (0x01<<6)) printk("DMADecErr ");
    if (status & (0x01<<8)) printk("SGIntErr ");
    if (status & (0x01<<9)) printk("SGSlvErr ");
    if (status & (0x01<<10)) printk("SGDecErr ");
    if (status & (0x01<<12)) printk("IOC_Irq ");
    if (status & (0x01<<13)) printk("Dly_Irq ");
    if (status & (0x01<<14)) printk("Err_Irq\n");
}

static irqreturn_t dataDmaDirect_irq(int irq, void *data)
{

	struct dataDmaDirect_dev *dev = (struct dataDmaDirect_dev *)data;
	printk("IT!\n");
	if (dev == NULL) {
		printk("problem de dev null\n");
		return IRQ_HANDLED;
	}
	dev->irq_handled = true;
	wake_up_interruptible(&dev->wq);

	return IRQ_HANDLED;
}

ssize_t dataDmaDirect_read(struct file * filp, char __user * user,
                  size_t size, loff_t * offp)
{
	int retval = size;
	int ret;
	unsigned long value;
	struct dataDmaDirect_dev *data = filp->private_data;
#ifdef IRQ
	int i;
	/* timeout 3 seconds*/
	ktime_t timeout = ktime_set(20, 0);
#else
	int timeout = 0;
#endif

	/* reset the controler */
	value = readl(data->membase + REG_DMA_DIRECT_CONTROL) | (1<<2);
	writel(value, data->membase + REG_DMA_DIRECT_CONTROL);
	do { /* wait for end of reset */
		value = readl(data->membase + REG_DMA_DIRECT_CONTROL);
	} while((value & (1<<2)) != 0x00);
	udelay(1000);

	printk("status : %x\n", readl(data->membase + REG_DMA_DIRECT_STATUS));
	printk("lancement acquisition\n");
	/* start DMA copy GO | Err_IrqEn | IOC_IrqEn */
	value = (1<<0);
#ifdef IRQ
	value |= (1<<14) | (1<<13) | (1<<12);
#endif

	writel(value, data->membase + REG_DMA_DIRECT_CONTROL);
	/* wait for dma ready */
	do {
		ret = readl(data->membase + REG_DMA_DIRECT_STATUS);
	} while((ret & (1<<0)) != 0x00);

	data->buff_size = size;
	writel((unsigned long)data->ram_buff, data->membase + REG_DMA_DIRECT_ADDR);
	writel(data->buff_size, data->membase + REG_DMA_DIRECT_LENGTH);
	value = readl(data->membase + REG_DMA_DIRECT_STATUS);

	/* start acquisition block */
	writel(0x01, data->membase_ip + DATA_DMA_DIRECT_REG_START);
	printk("%x\n", readl(data->membase_ip + DATA_DMA_DIRECT_REG_START));

#ifdef IRQ
	/* wait until interrupt happened */
	ret = wait_event_interruptible_hrtimeout(data->wq, data->irq_handled, 
				timeout);
	if (ret == -ETIME) {
		value = readl(data->membase + REG_DMA_DIRECT_STATUS);
		printk("%x errirq%x dlyirq%x dmainterr%x\n", (unsigned int)value,
			(unsigned int)(0x01&(value>>14)), 
			(unsigned int)(0x01&(value>>13)), (unsigned int)(value>>4)&0x01);
		for (i=15; i>=0;i--)
			printk("%d => %x\n", i, (unsigned int)(0x01&(value >> i)));

		printStatus(readl(data->membase + REG_DMA_DIRECT_STATUS));
		printk("%d\n", readl(data->membase + REG_DMA_DIRECT_LENGTH));
		dev_alert(data->misc.this_device, "  timeout!\n");
		retval = -ETIME;
	} else if (ret) {
		dev_alert(data->misc.this_device, "  error=%d\n", ret);
		return -ERESTARTSYS;
	}
	printk("%x\n", readl(data->membase_ip + DATA_DMA_DIRECT_REG_START));
	printk("%x %d\n", readl(data->membase + REG_DMA_DIRECT_STATUS),
			readl(data->membase + REG_DMA_DIRECT_LENGTH));
#else
    /* polling */
	printk("polling\n");
    do {
        ret = readl(data->membase + REG_DMA_DIRECT_STATUS);
        if (timeout == 1000000) {
            printk("timeout %x\n", ret);
            printStatus(ret);
            value = readl(data->membase + REG_DMA_DIRECT_LENGTH);
            printk("length %lu %lu\n", value, value/sizeof(long));
            return -ETIME;
		} else
            timeout++;
	} while(((ret & (1<<1)) == 0x00) && ((ret & (1<<12)) == 0x00));
    value = readl(data->membase + REG_DMA_DIRECT_LENGTH);
    printk("length %lu %lu\n", value, value/sizeof(long));
    printk("plop\n");

    value = readl(data->membase + REG_DMA_DIRECT_STATUS);
    printStatus(value);
    printk("status : %lx\n", value);

    /* clear all */
    writel(7<<12, data->membase + REG_DMA_DIRECT_STATUS);

#endif

	/* reset condition variable */
	data->irq_handled = false;

	mutex_lock(&mutex_complex);
	memcpy(data->data_buff, data->virt_addr, size);
	if (copy_to_user(user, (u32 *) data->data_buff, size)) {
		printk(KERN_WARNING "read : copy to user data error\n");
		retval = -EFAULT;
		goto out_free;
	}
	mutex_unlock(&mutex_complex);
out_free:
	return retval;

}

static long dataDmaDirect_ioctl(struct file *filp, unsigned int cmd,
			unsigned long arg)
{
	int retval = -EACCES;
	u32 ioc;
	struct dataDmaDirect_dev *data = filp->private_data;
	if (data == NULL) {
		printk("pas de data\n");
		return -ENODATA;
	}

	if (_IOC_DIR(cmd) & _IOC_READ) {
		if (_IOC_NR(cmd) == DATA_DMA_DIRECT_REG_SIZE) {
			ioc = data->ip_buffer_byte_size;
			if (put_user(data->ip_buffer_byte_size, (u32 __user *) arg))
				return -EACCES;
			retval = 4;
		}
	}
	return retval;
}

int dataDmaDirect_open(struct inode *inode, struct file *filp)
{
	/* Allocate and fill any data structure to be put in filp->private_data */
	struct dataDmaDirect_dev *pos, *data = NULL;
	unsigned long control;

	printk("open\n");
	list_for_each_entry(pos, &dataDmaDirect_data_list, list) {
		if (pos->misc.minor == iminor(inode)) {
			data = pos;
			break;
		}
	}
	if (data == NULL) {
		printk("pas trouve\n");
		return -ENODATA;
	}
	filp->private_data = (struct device *)data;

	data->irq_handled = false;
	sema_init(&data->sema_read, 0);

	/* reset dma IP */
	control = readl(data->membase + REG_DMA_DIRECT_CONTROL) | (1<<2);
	writel(control, data->membase + REG_DMA_DIRECT_CONTROL);
	do { /* wait for end of reset */
		control = readl(data->membase + REG_DMA_DIRECT_CONTROL);
	} while((control & (1<<2)) != 0x00);
	udelay(1000);

	return 0;
}

int dataDmaDirect_release(struct inode *inode, struct file *filp)
{
	u32 status;
	struct dataDmaDirect_dev *sdev = filp->private_data;
	
	/* stop DMA IP */
	status = readl(sdev->membase + REG_DMA_DIRECT_CONTROL);
	status &= ~0x01;
	writel(status, sdev->membase + REG_DMA_DIRECT_CONTROL);
	filp->private_data = NULL;

	return 0;
}

static struct file_operations dataDmaDirect_fops = {
	.owner = THIS_MODULE,
	.read = dataDmaDirect_read,
	.unlocked_ioctl = dataDmaDirect_ioctl,
	.open = dataDmaDirect_open,
	.release = dataDmaDirect_release,
};

static int dataDmaDirect_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	int ret = 0;
	struct dataDmaDirect_dev *sdev;
	struct resource *mem_res;
	size_t size_name;

	struct resource *mem_res_ip;

	if (!np)
		return -ENODEV;
	printk("probing %s with dts\n", np->name);

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

	/* get ressources for specific ip */
	mem_res_ip = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!mem_res_ip) {
		dev_err(&pdev->dev, "can't find mem resource\n");
		ret = -EINVAL;
		goto out_release_mem;
	}
	mem_res_ip = request_mem_region(mem_res_ip->start, 
					resource_size(mem_res_ip), pdev->name);
	if (!mem_res_ip) {
		dev_err(&pdev->dev, "iomem already in use\n");
		ret = -EBUSY;
		goto out_release_mem;
	}

	/* allocate memory for private structure */
	sdev = kzalloc(sizeof(struct dataDmaDirect_dev), GFP_KERNEL);
	if (!sdev) {
		ret = -ENOMEM;
		goto out_release_mem_ip;
	}

	sdev->membase = ioremap(mem_res->start, resource_size(mem_res));
	if (!sdev->membase) {
		dev_err(&pdev->dev, "ioremap failed\n");
		ret = -ENOMEM;
		goto out_dev_free;
	}
	sdev->membase_ip = ioremap(mem_res_ip->start, resource_size(mem_res_ip));
	if (!sdev->membase_ip) {
		dev_err(&pdev->dev, "ioremap failed\n");
		ret = -ENOMEM;
		goto out_iounmap;
	}
	sdev->mem_res = mem_res;
	sdev->mem_res_ip = mem_res_ip;
	printk("4\n");

	sdev->ip_buffer_byte_size = readl(sdev->membase_ip + DATA_DMA_DIRECT_REG_SIZE);
	printk("%d\n", sdev->ip_buffer_byte_size);

	/* get device name */
	size_name = sizeof(char) * strlen(np->name) + 1;
	sdev->name = (char *)kmalloc(size_name, GFP_KERNEL);

	if (sdev->name == NULL) {
		dev_err(&pdev->dev, "Kmalloc name space error\n");
		goto out_iounmap_ip;
	}
	if (strncpy(sdev->name, np->name, size_name) < 0) {
		printk("erreur <1> 0x%x\n", ret);
		goto out_cdev_free;
	}

	/* init waitqueue and lock */
	sdev->irq_handled = false;
	init_waitqueue_head(&sdev->wq);

	/* misc init */
	sdev->misc.name = sdev->name;
	sdev->misc.minor = MISC_DYNAMIC_MINOR;
	sdev->misc.fops = &dataDmaDirect_fops;
	printk("%s\n", sdev->name);
	platform_set_drvdata(pdev, sdev);

	ret = misc_register(&sdev->misc);
	if (ret) {
		printk(KERN_ERR "%s:%u: misc_register failed %d \n",
		       __func__, __LINE__, ret);
		goto out_cdev_free;
	}
	
	/* init buffer */
	sdev->data_buff = kmalloc (sdev->ip_buffer_byte_size, GFP_KERNEL);
	if (!sdev->data_buff) {
		printk("erreur d'alloc du buffer taille %d %ld\n",
			sdev->ip_buffer_byte_size, KMALLOC_MAX_SIZE);
		goto out_malloc_err;
	}

	/* DMA */
	ret = dma_set_coherent_mask(sdev->misc.this_device, DMA_BIT_MASK(32));
	if (ret != 0) {
		goto out_dma_corr_err;
	}

	sdev->virt_addr = dma_alloc_coherent(
		sdev->misc.this_device, (size_t)sdev->ip_buffer_byte_size,
		&sdev->ram_buff, GFP_KERNEL);
	if (!sdev->virt_addr) {
		ret = -ENOMEM;
		goto out_dma_corr_err;
	}

	sema_init(&sdev->sema_read, 0);
	sdev->pdev = pdev;

	/* irq */
	/* get IRQ 0 'dmatest' from device tree */
	sdev->irq_res = platform_get_irq(pdev, 0);
	if (sdev->irq_res <= 0) {
		ret = -ENXIO;
        dev_err(&pdev->dev, "irq number is invalid\n");
		goto out_dma_alloc_err;
	}
	printk("irq res %d\n", sdev->irq_res);

	if (devm_request_irq(&sdev->pdev->dev, sdev->irq_res, dataDmaDirect_irq, 
				IRQF_TRIGGER_RISING, "complex16ToRam", sdev)) {
		printk(KERN_ERR "Couldn't allocate IRQ (%d)\n", sdev->irq_res);
		goto out_dma_alloc_err;
	} 
	printk("irq ok : %d\n", sdev->irq_res);

	list_add(&sdev->list, &dataDmaDirect_data_list);
	/* OK driver ready ! */
	dev_info(&pdev->dev, KERN_INFO "%s loaded\n", sdev->name);
	return 0;
out_dma_alloc_err:
	dma_free_coherent(sdev->misc.this_device, sdev->ip_buffer_byte_size,
		sdev->virt_addr, sdev->ram_buff);
out_dma_corr_err:
	kfree(sdev->data_buff);
out_malloc_err:
	misc_deregister(&sdev->misc);
 out_cdev_free:
 	kfree(sdev->name);
 out_iounmap_ip:
	iounmap(sdev->membase_ip);
 out_iounmap:
	iounmap(sdev->membase);
 out_dev_free:
	kfree(sdev);
 out_release_mem_ip:
	release_mem_region(mem_res_ip->start, resource_size(mem_res_ip));
 out_release_mem:
	release_mem_region(mem_res->start, resource_size(mem_res));

	return ret;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 11, 0)
static int dataDmaDirect_remove(struct platform_device *pdev)
#else
static void dataDmaDirect_remove(struct platform_device *pdev)
#endif
{
	struct dataDmaDirect_dev *sdev;
	sdev = (struct dataDmaDirect_dev *)platform_get_drvdata(pdev);

	printk("kfree de l'ensemble\n");

	devm_free_irq(&sdev->pdev->dev, sdev->irq_res, sdev);
	dma_free_coherent(sdev->misc.this_device, sdev->ip_buffer_byte_size,
		sdev->virt_addr, sdev->ram_buff);
	kfree(sdev->data_buff);

	misc_deregister(&sdev->misc);
	kfree(sdev->name);

	release_mem_region(sdev->mem_res_ip->start, resource_size(sdev->mem_res_ip));
	release_mem_region(sdev->mem_res->start, resource_size(sdev->mem_res));
	iounmap(sdev->membase_ip);
	iounmap(sdev->membase);
	kfree(sdev);
	printk(KERN_INFO "%s: deleted with success\n", sdev->name);

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 11, 0)
	return 0;
#endif
}

static struct of_device_id dmacopy_of_match[] = {
	{.compatible = "ggm,dataDmaDirect",},
	{}
};

MODULE_DEVICE_TABLE(of, dmacopy_of_match);

static struct platform_driver plat_dataDmaDirect_driver = {
	.probe = dataDmaDirect_probe,
	.remove = dataDmaDirect_remove,
	.driver = {
		   .name = "dataDmaDirect",
		   .owner = THIS_MODULE,
		   .of_match_table = dmacopy_of_match,
	},
};

module_platform_driver(plat_dataDmaDirect_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Gwenhael Goavec-Merou <gwenhael.goavec-merou@trabucayre.com>");
MODULE_DESCRIPTION("dataDmaDirect IP generic driver");
