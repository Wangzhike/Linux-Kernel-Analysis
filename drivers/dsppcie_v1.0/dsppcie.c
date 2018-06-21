#include <linux/module.h>
#include <linux/init.h>     /* module_init(), module_exit() */
#include <linux/kernel.h>   /* printk() */
#include <linux/slab.h>     /* kmalloc() */
#include <linux/fs.h>       /* everything..., like struct file */
#include <linux/types.h>    /* size_t, unit8_t, dev_t, etc. */
#include <linux/errno.h>    /* error codes */
#include <linux/fcntl.h>    /* O_ACCMODE */
#include <linux/uaccess.h>  /* copy_*_user() */
#include <linux/cdev.h>     /* character device */
#include <linux/pci.h>      /* pci device: pci_read_config_*() */
#include <linux/ioport.h>   /* I/O memory */
#include <linux/jiffies.h>
#include <linux/delay.h>
#include "dsppcie.h"
#include "pcieLocalReset_6678.h"	/* Local Reset Array header file */
#include "pcieInterrupt_6678.h"		/* Init DDR and set DSP PCIE interrupt Array header file */

MODULE_LICENSE("Dual BSD/GPL");

/* 驱动程序支持的不同类型的PCI设备列表 */
static const struct pci_device_id pci_id_tables[] = {
    { PCI_DEVICE(PCIE_TI_VENDOR, PCIE_TI_DEVICE), },    /* PCI_DEVICE()创建一个和特定厂商及设备ID相匹配的struct pci_device_id */
    { 0, },     /* 数组的最后一个值设置为全部为0的空结构体表示结束 */
};

/* 本PCI驱动程序的主结构体struct pci_driver */
static struct pci_driver dsppcie_driver = {
    .name = DSPPCIE_CDEV_NAME,
    .id_table = pci_id_tables,
    .probe = probe,
    .remove = remove,
    /* suspend, resume are optional */
};

/* 作为字符设备文件操作的结构体 */
static struct file_operations dpe_cdev_fops = {
	.owner = THIS_MODULE,
	.open = dpe_cdev_open,
	.release = dpe_cdev_release,
    .read = dpe_cdev_read,
    .write = dpe_cdev_write,
};

static int dpe_cdev_open(struct inode *inode, struct file *filp)
{
	struct dsppcie_dev *dpe;

	/* pointer to containing data structure of the character device inode */
	printk(KERN_DEBUG "[open] pointer to containing data structure of the character device inode\n");
	dpe = container_of(inode->i_cdev, struct dsppcie_dev, cdev);
	/* create a reference to our device state in the opened file */
	filp->private_data = dpe;
	return 0;
}

static int dpe_cdev_release(struct inode *inode, struct file *filp)
{
    printk(KERN_DEBUG "[release] close character device\n"); 
    return 0;
}

static ssize_t dpe_cdev_read(struct file *filp, char __user *buf, size_t count,loff_t *pos)
{
	int i;
	size_t real_count = count;
    struct dsppcie_dev *dpe = (struct dsppcie_dev *)filp->private_data;
	if (count > dpe->data_buf_size)
		real_count = dpe->data_buf_size;


	printk(KERN_DEBUG "------------- DMA Read Start --------------\n");
	if (!dpe->rcv_data_virt_addr) {
		printk(KERN_DEBUG "[read] Don't allocate corherent DMA buffer for %s character device, so read failed!\n", DSPPCIE_CDEV_NAME);
		goto fail;
	}

	printk(KERN_DEBUG "[read] Read DMA from DSP...\n");
	/* HAL_readDMA是阻塞读，返回就代表读取完毕 */
	if (count > dpe->data_buf_size)
		printk(KERN_DEBUG "[read] Read maximum size limits %d bytes\n", dpe->data_buf_size);
	//HAL_readDMA(dpe, dpe->data_entry_from_dsp, dpe->data_bus_addr, real_count, 1);	/* Move from DSP to GPP */
	HAL_readDMA(dpe, dpe->data_entry_from_dsp, dpe->rcv_data_bus_addr, real_count, 1);	/* Move from DSP to GPP */
	for (i = 0; i < 20; ++i) {
		printk(KERN_DEBUG "[read] %d = %d\n", i, dpe->rcv_data_virt_addr[i]);
	}
	printk(KERN_DEBUG "------------ DMA Read End -----------------\n");
	
	/* Move data to user space */
	printk(KERN_DEBUG "[read] Move data to user space...\n");
	copy_to_user(buf, (void *)dpe->rcv_data_virt_addr, real_count);

	return real_count;

fail:
	printk(KERN_DEBUG "[read] DMA Read Failed!\n");
	return 0;
}

static ssize_t dpe_cdev_write(struct file *filp, const char __user *buf, size_t count, loff_t *pos)
{
	int i;
	size_t real_count = count;
	struct dsppcie_dev *dpe = (struct dsppcie_dev *)filp->private_data;
	if (count > dpe->data_buf_size)
		real_count = dpe->data_buf_size;


	printk(KERN_DEBUG "-------------- DMA Write Start ---------------\n");
	if (!dpe->data_virt_addr) {
		printk(KERN_DEBUG "[wirte] Don't allocate corherent DMA buffer for %s character device, so write failed!\n", DSPPCIE_CDEV_NAME);
		goto fail;
	}
	/* Get data from user space */
	printk(KERN_DEBUG "[write] Get data from user space...\n");
	copy_from_user((void *)dpe->data_virt_addr, buf, real_count);
	for (i = 0; i < 20; ++i) {
		printk(KERN_DEBUG "[write] %d = %d\n", i, dpe->data_virt_addr[i]);
	}
	printk(KERN_DEBUG "[write] Write DMA to DSP...\n");
	/* HAL_writeDMA是阻塞写，返回就代表写入DSP完毕 */
	if (count > dpe->data_buf_size)
		printk(KERN_DEBUG "[write] Read maximum size limits %d bytes\n", dpe->data_buf_size);
	//HAL_writeDMA(dpe, dpe->ddr_bus_addr, dpe->data_entry_from_dsp, real_count, 1);	/* Move from GPP to DSP */
	HAL_writeDMA(dpe, dpe->data_bus_addr, dpe->data_entry_from_dsp, real_count, 1);	/* Move from GPP to DSP */
    printk(KERN_DEBUG "-------------- DMA Write End ------------------\n");

	/* Note: From PCIE specification, legacy interrupts cannot be generated from RC and be passed downstream. 
	   The example below is just making use of the facility that RC can access EP side register to generate 
	   a generic interrupt on local (EP) side using one of the event inputs of Interrupt Controller (INTC). 
	   There is no real interrupt signal sent over the PCIe link */
	printk (KERN_DEBUG "[write] Generating interrupt to DSP ...\n");
	iowrite32(1, dpe->appreg_virt_addr + LEGACY_A_IRQ_STATUS_RAW/4);
	return real_count; 

fail:
	printk(KERN_DEBUG "[write] DMA Read Failed!\n");
	return 0;
}

/* 字符设备初始化，在PCI设备probe中调用 */
static int dpe_cdev_init(struct dsppcie_dev *dpe) {
	int result, err, i;
	uint32_t bootEntryAddr = 0;		/* Store 32-bit boot entry address read from .h file */
	uint32_t buffer[BLOCK_TRANSFER_SIZE/4];		/* Store 32-bit DSP memory readback data */

	/* 分配主设备号 */
	printk(KERN_DEBUG "Allocating major and minor as character device for %s...\n", DSPPCIE_CDEV_NAME);
	if (DSPPCIE_CDEV_MAJOR) {
		dpe->cdevno = MKDEV(DSPPCIE_CDEV_MAJOR, 0);
		result = register_chrdev_region(dpe->cdevno, 1, DSPPCIE_CDEV_NAME);
	} else {
		result = alloc_chrdev_region(&(dpe->cdevno), 0, 1, DSPPCIE_CDEV_NAME);
	}if (result < 0) {
		printk(KERN_DEBUG "Can't get major %d for %s\n", MAJOR(dpe->cdevno), DSPPCIE_CDEV_NAME);
		goto fail_alloc;
	}
	printk(KERN_DEBUG "Allocate major number: %d for %s\n", MAJOR(dpe->cdevno), DSPPCIE_CDEV_NAME);

	printk(KERN_DEBUG "Registering as character device for %s...\n", DSPPCIE_CDEV_NAME);
	cdev_init(&(dpe->cdev), &dpe_cdev_fops);
	dpe->cdev.owner = THIS_MODULE;
	dpe->cdev.ops = &dpe_cdev_fops;		 /* not sure what is this doing here!!, this is OLD STYLE */
	err = cdev_add(&(dpe->cdev), dpe->cdevno, 1);	/* bring character device live */
	/* Fail gracefully if need be */
    if (err) {
		printk(KERN_DEBUG "Error %d adding %s as character device\n", err, DSPPCIE_CDEV_NAME);
		goto fail_add;
	}
    else
        printk(KERN_DEBUG "Add %s as character device major: %d and minor: %d\n", DSPPCIE_CDEV_NAME, MAJOR(dpe->cdevno), 0);
	
	/* 设置字符设备读写的DSP内存起始位置是DDR首地址 */
	dpe->data_entry_from_dsp = DDR_START;
	dpe->data_buf_size = DMA_TRANSFER_SIZE;		/* DMA缓冲区的大小 */
    //dpe->data_buf_size = 100;
	
    /* Allocate DMA buffer */
	printk(KERN_DEBUG "Allocating DMA buffer as character device for %s\n", DSPPCIE_CDEV_NAME);
	dpe->data_virt_addr = (uint8_t *)dma_alloc_coherent(&(dpe->pci_dev->dev), dpe->data_buf_size, &(dpe->data_bus_addr), GFP_KERNEL);
	if (!dpe->data_virt_addr) {
		printk(KERN_DEBUG "Could not allocate Data DMA buffer!\n");
		goto fail_dma_alloc_1;
	}
    dpe->rcv_data_virt_addr = (uint8_t *)dma_alloc_coherent(&(dpe->pci_dev->dev), dpe->data_buf_size, &(dpe->rcv_data_bus_addr), GFP_KERNEL);
	if (!dpe->rcv_data_virt_addr) {
		printk(KERN_DEBUG "Could not allocate Receving DMA buffer!\n");
		goto fail_dma_alloc_2;
	}

    for (i = 0; i < dpe->data_buf_size; i++) {
		dpe->data_virt_addr[i] = i;
        dpe->rcv_data_virt_addr[i] = 0;
    }

	/* 初始化DSP的DDR以及设置中断处理程序 */
	/* Load Interrupt demo code into DSP */
	pushData(dpe, pcieInterrupt, 0, &bootEntryAddr);

	/* Write boot entry address into MAGIC_ADDR */
	writeDSPMemory(dpe, 0, MAGIC_ADDR, &bootEntryAddr, 4);       

	while (1) {
		readDSPMemory(dpe, 0, MAGIC_ADDR, buffer, 4);
		if (buffer[0] == 0)  break;
		for (i = 0; i < 1000; i++) i++;
	} 

	/* Wait 2 second for DDR init */
	mdelay(2000);

    // printk ("Write DMA to DSP ...\n");
    // for (i = 0; i < 20; ++i)
    //     printk(KERN_DEBUG "[cdev_int] %d = %d\n",i, dpe->data_virt_addr[i]);
    // HAL_writeDMA(dpe, dpe->data_bus_addr, dpe->data_entry_from_dsp, dpe->data_buf_size, 1);   /* Move from GPP to DSP */ 

    // /* Note: From PCIE specification, legacy interrupts cannot be generated from RC and be passed downstream. 
    //    The example below is just making use of the facility that RC can access EP side register to generate 
    //    a generic interrupt on local (EP) side using one of the event inputs of Interrupt Controller (INTC). 
    //    There is no real interrupt signal sent over the PCIe link */
    // printk ("Generating interrupt to DSP ...\n");
    // iowrite32(1, dpe->appreg_virt_addr + LEGACY_A_IRQ_STATUS_RAW/4); 

    // /* Waiting DSP ISR handler to process the interrupt from DSP and then generates an interrupt to host
    //    Waiting for host ISR handler to process the interrupt from DSP before program exits */ 
    // mdelay(1000);

	return 0;

fail_dma_alloc_2:
    /* free the DMA buffer */
	if (dpe->data_virt_addr) {
		pci_free_consistent(dpe->pci_dev, dpe->data_buf_size, dpe->data_virt_addr, dpe->data_bus_addr);
	}
fail_dma_alloc_1:
fail_add:
	/* free the dynamically allocated character device node */
	unregister_chrdev_region(dpe->cdevno, 1);
fail_alloc:
	return -1;
}

/* 字符设备注销函数，在PCI设备remove中调用 */
static void dpe_cdev_exit(struct dsppcie_dev *dpe) {

	printk(KERN_DEBUG "Free allocating DMA buffer\n");
	/* free the DMA buffer */
	if (dpe->data_virt_addr) {
		dma_free_coherent(&(dpe->pci_dev->dev), dpe->data_buf_size, dpe->data_virt_addr, dpe->data_bus_addr);
	}
	if (dpe->rcv_data_virt_addr) {
		dma_free_coherent(&(dpe->pci_dev->dev), dpe->data_buf_size, dpe->rcv_data_virt_addr, dpe->rcv_data_bus_addr);
	}
    /* remove the character device */
	printk(KERN_DEBUG "Remove the character device\n");
	cdev_del(&(dpe->cdev));
	/* free the dynamically allocated character devide node */
	printk(KERN_DEBUG "Free the character device node major: %d, minor: %d\n", MAJOR(dpe->cdevno), 0);
	unregister_chrdev_region(dpe->cdevno, 1);
}

/*
 * 当PCI核心认为有一个驱动程序需要控制的struct pci_dev时，就会调用该函数。
 * 返回0表示可以控制该设备。
 */
static int probe(struct pci_dev *dev, const struct pci_device_id *id_tables) {
    int result;
    int ret = 0;
    int i;
    struct dsppcie_dev *dpe = NULL;
    printk(KERN_DEBUG "Probed TI device: vendor=0x%04x, device=0x%04x, irqNo=0x%08x\n", dev->vendor, dev->device, dev->irq);
    
    printk(KERN_DEBUG "Allocate memory for dsppcie_dev\n");
    dpe = kmalloc(sizeof(dpe), GFP_KERNEL);
    if (!dpe) {
        printk(KERN_DEBUG "Could not kmalloc memory for %s\n", DSPPCIE_CDEV_NAME);
        ret = -ENOMEM;
        goto err_dpe;
    }

    /* 给驱动设备自定义结构体struct dsppcie_dev赋值 */
    dpe->pci_dev = dev;
    dev->dev.platform_data = (void *)dpe;

    /* enalbe device */
    printk(KERN_DEBUG "Enable the pci device...\n");
    result = pci_enable_device(dev);
    if (result) {
        printk(KERN_DEBUG "Could not enable %s!\n", DSPPCIE_CDEV_NAME);
        ret = -ENODEV;
        goto err_enable;
    }

    /* 读取BAR寄存器 */
    printk(KERN_DEBUG "Reading the BAR area...\n");
    PCI_readBAR(dev);

    /* 读取中断号 */
    dpe->irqNo = dev->irq;
	dpe->irq_count = 0;


    /* Linux Function: Associates the given data with the given pci_driver structure */
    pci_set_drvdata(dev, dpe->localL2_virt_addr);

    /* enable bus master capability on device */
    printk(KERN_DEBUG "Enable bus master capability on device\n");
    PCI_set_master(dev);

    /* Configure IB_BAR0 to BAR0 for PCIE registers; Configure IB_BAR1 to BAR1 for LL2 for core 0;
       Configure IB_BAR2 to BAR2 for MSMC; Configure IB_BAR3 to BAR3 for DDR */
    printk(KERN_DEBUG "Configure IB_BAR0 to BAR0 for PCIE application registers...\n");
    printk(KERN_DEBUG "Configure IB_BAR1 to BAR1 for Local L2 RAM for core 0...\n");
    printk(KERN_DEBUG "Configure IB_BAR2 to BAR2 for Multicore Shared Memory...\n");
    printk(KERN_DEBUG "Configure IB_BAR3 to BAR3 for DDR3...\n");
	printk(KERN_DEBUG "Access PCIE application register ....\n");
    for (i = 0; i < 4; i++) {
        iowrite32(i, dpe->appreg_virt_addr + IB_BAR(i)/4);     
        iowrite32(dev->resource[i].start, dpe->appreg_virt_addr + IB_START_LO(i)/4);    
        iowrite32(0, dpe->appreg_virt_addr + IB_START_HI(i)/4);
    }     
    iowrite32(PCIE_BASE_ADDRESS, dpe->appreg_virt_addr + IB_OFFSET(0)/4);    
    iowrite32(LL2_START + (1 << 28), dpe->appreg_virt_addr + IB_OFFSET(1)/4);    
    iowrite32(MSMC_START, dpe->appreg_virt_addr + IB_OFFSET(2)/4);  
    iowrite32(DDR_START, dpe->appreg_virt_addr + IB_OFFSET(3)/4); 
    printk(KERN_DEBUG "Configure IB_BAR0 to IB_BAR3 finish\n");

    printk(KERN_DEBUG "Registering the irq %d ...\n", dpe->irqNo);
    request_irq(dpe->irqNo, dsppcie_isr, IRQF_SHARED, DSPPCIE_CDEV_NAME, (void *)dpe);

    printk(KERN_DEBUG "Enable the DSP to generate interrupts to the Host\n");
    HAL_PciEnableDspInterrupt(dpe);
	
    printk(KERN_DEBUG "**************************************************\n");
    printk(KERN_DEBUG "Initialize character devide for %s...\n", DSPPCIE_CDEV_NAME);
	dpe_cdev_init(dpe);
	
    printk(KERN_DEBUG "Probe finished.\n");
    return ret;

err_enable:
    kfree(dpe);
err_dpe:
    return ret;
}

static void remove(struct pci_dev *dev) {
    
    struct dsppcie_dev *dpe;
    printk(KERN_DEBUG "Removing TI device...\n");
    if (dev == NULL || dev->dev.platform_data == NULL) {
        printk(KERN_DEBUG "remove(dev = 0x%p) dev->dev.platform_data = 0x%p\n", dev, dev->dev.platform_data);
        return ;
    }

    dpe = (struct dsppcie_dev *)dev->dev.platform_data;
    printk(KERN_DEBUG "remove(dev = 0x%p) where dev->dev.platform_dat = 0x%p\n", dev, dpe);

    
    printk(KERN_DEBUG "Perform DSP cores and periphrals reset...\n");
    dspLocalReset(dpe);

	printk(KERN_DEBUG "Cleanup character device for %s...\n", DSPPCIE_CDEV_NAME);
	dpe_cdev_exit(dpe);
	printk(KERN_DEBUG "**************************************************\n");

    printk(KERN_DEBUG "Disable the DSP to generate interrupts to the Host.\n");
    HAL_PciDisableDspInterrupt(dpe);
    
    /* Unmap and release Application Register region */
    printk(KERN_DEBUG "Unmap and release Application Register region\n");
    iounmap(dpe->appreg_virt_addr);
    if (pci_resource_flags(dev, 0) & IORESOURCE_MEM) {
        release_mem_region(dpe->appreg_bus_addr, dpe->appreg_len);
    }
    else {
        release_region(dpe->appreg_bus_addr, dpe->appreg_len);
    }

    /* Unmap and release Local L2 RAM of core 0 region */
    printk(KERN_DEBUG "Unmap and release Local L2 RAM of core 0 region\n");
    iounmap(dpe->localL2_virt_addr);
    if (pci_resource_flags(dev, 1) & IORESOURCE_MEM) {
        release_mem_region(dpe->localL2_bus_addr, dpe->localL2_len);
    }
    else {
        release_region(dpe->localL2_bus_addr, dpe->localL2_len);
    }

    /* Unmap and release Multicore Shared Memory region */
    printk(KERN_DEBUG "Unmap and release Multicore Shared Memory region\n");
    iounmap(dpe->msmc_virt_addr);
    if (pci_resource_flags(dev, 2) & IORESOURCE_MEM) {
        release_mem_region(dpe->msmc_bus_addr, dpe->msms_len);
    }
    else {
        release_region(dpe->msmc_bus_addr, dpe->msms_len);
    }

    /* Unmap and release DDR Memory region */
    printk(KERN_DEBUG "Unmap and release DDR Memory region\n");
    iounmap(dpe->ddr_vrit_addr);
    if (pci_resource_flags(dev, 3) & IORESOURCE_MEM) {
        release_mem_region(dpe->ddr_bus_addr, dpe->ddr_len);
    }
    else {
        release_region(dpe->ddr_bus_addr, dpe->ddr_len);
    }

    printk(KERN_DEBUG "Release allocated irq: %d for the device\n", dpe->irqNo);
    free_irq(dpe->irqNo, dpe);

	printk(KERN_DEBUG "Free allocated dpspcie_dev struct\n");
	kfree(dpe);
}

static irqreturn_t dsppcie_isr(int irq, void *dev_id) {
    // uint32_t i, dma_failure_flag = 0, counter = 0;
    struct dsppcie_dev *dpe = (struct dsppcie_dev*)dev_id;
	uint32_t status = HAL_CheckPciInterrupt(dpe);


	if (status) {
		if (!dpe)
			return IRQ_NONE;

        printk("Interrupt %d received from DSP\n", irq);

		// printk("Read DMA from DSP ...\n");
		// HAL_readDMA(dpe, dpe->data_entry_from_dsp, dpe->rcv_data_bus_addr, dpe->data_buf_size, 1);     /* Move from DSP to GPP */

		// for (i = 0; i < DMA_TRANSFER_SIZE; i++) {
		// 	if ((~(dpe->rcv_data_virt_addr[i])&0xFF) != dpe->data_virt_addr[i]) {
		// 		dma_failure_flag = 1;
		// 		counter++;
		// 	}
		// }

		// if (dma_failure_flag) 
		// 	printk("DMA test failed with %d locations !\n", counter);
		// else 
		// 	printk("DMA test passed!\n");

		dpe->irq_count++;
		printk(KERN_DEBUG "Enter interrupt service routine for %d time\n", dpe->irq_count);
		HAL_PciClearDspInterrupt(dpe);
		return IRQ_HANDLED;
	}
	return IRQ_NONE;
    
}

static void PCI_readBAR(struct pci_dev *dev) {
    struct dsppcie_dev *dpe = NULL;
    resource_size_t barStart[4];
    resource_size_t barLen[4];
    resource_size_t barFlags[4];

    dpe = dev->dev.platform_data;

    /* BAR0: 4K for PCIE application registers */
    barStart[0] = pci_resource_start(dev, 0);
    barLen[0] = pci_resource_len(dev, 0);
    barFlags[0] = pci_resource_flags(dev, 0);
    /* BAR1: 512K for core0 local L2 RAM */
    barStart[1] = pci_resource_start(dev, 1);
    barLen[1] = pci_resource_len(dev, 1);
    barFlags[1] = pci_resource_flags(dev, 1);
    /* BAR2: 4M for shared L2 */
    barStart[2] = pci_resource_start(dev, 2);
    barLen[2] = pci_resource_len(dev, 2);
    barFlags[2] = pci_resource_flags(dev, 2);
    /* BAR3: 16M for DDR3 */
    barStart[3] = pci_resource_start(dev, 3);
    barLen[3] = pci_resource_len(dev, 3);
    barFlags[3] = pci_resource_flags(dev, 3);

    /*
    * 使用I/O内存的步骤：
    *   1. 首先分配I/O内存区域
    *      struct resource *request_mem_region(unsigned long start, unsigned long len, char *name);
    *   2. 为I/O内存分配虚拟地址
    *      void *ioremap(unsinged long phys_addr, unsigned long size);
    *   另外，由ioremap返回的地址不应直接引用，而应该使用内核提供的accessor函数：
    *      ioreadn(), iowriten()
    */
    /* Map the application register memory region */
	if (barFlags[0] & IORESOURCE_MEM) {
		dpe->appreg_bus_addr = barStart[0];
		/* Map the memory region. */
		request_mem_region(dpe->appreg_bus_addr, barLen[0], DSPPCIE_CDEV_NAME);
	}
	else {
		/* Map the memory region. */
		request_region(dpe->appreg_bus_addr, barLen[0], DSPPCIE_CDEV_NAME);
	}
	if (dpe->appreg_bus_addr > 0) {
		dpe->appreg_virt_addr = ioremap(barStart[0], barLen[0]);
	}

    /* Map the Local L2 RAM memory region */
    if (barFlags[1] & IORESOURCE_MEM) {
        dpe->localL2_bus_addr = barStart[1];
        request_mem_region(dpe->localL2_bus_addr, barLen[1], DSPPCIE_CDEV_NAME);
    } else {
        request_region(dpe->localL2_bus_addr, barLen[1], DSPPCIE_CDEV_NAME);
    }
    if (dpe->localL2_bus_addr > 0)
        dpe->localL2_virt_addr = ioremap(barStart[1], barLen[1]);
    
    /* Map the MSMC memory region */
    if (barFlags[2] & IORESOURCE_MEM) {
        dpe->msmc_bus_addr = barStart[2];
        request_mem_region(dpe->msmc_bus_addr, barLen[2], DSPPCIE_CDEV_NAME);
    } else {
        request_region(dpe->msmc_bus_addr, barLen[2], DSPPCIE_CDEV_NAME);
    }
    if (dpe->msmc_bus_addr > 0)
        dpe->msmc_virt_addr = ioremap(barStart[2], barLen[2]);
    
    /* Map the DDR memory region */
    if (barFlags[3] & IORESOURCE_MEM) {
        dpe->ddr_bus_addr = barStart[3];
        request_mem_region(dpe->ddr_bus_addr, barLen[3], DSPPCIE_CDEV_NAME);
    } else {
        request_region(dpe->ddr_bus_addr, barLen[3], DSPPCIE_CDEV_NAME);
    }
    if (dpe->ddr_bus_addr > 0)
        dpe->ddr_vrit_addr = ioremap(barStart[3], barLen[3]);

    printk(KERN_DEBUG "Application Register Memory Bus Base Address: 0x%lld\n", dpe->appreg_bus_addr);
    printk(KERN_DEBUG "Local L2 Memory of Core 0 Bus Base Address: 0x%lld\n", dpe->localL2_bus_addr);
    printk(KERN_DEBUG "Multicore Shared Memory Bus Base Address: 0x%lld\n", dpe->msmc_bus_addr);
    printk(KERN_DEBUG "DDR Bus Base Address: 0x%lld\n", dpe->ddr_bus_addr);
    
    dpe->appreg_len = barLen[0];
    dpe->localL2_len = barLen[1];
    dpe->msms_len = barLen[2];
    dpe->ddr_len = barLen[3];
}

static void PCI_set_master(struct pci_dev *dev) {
    int32_t  retVal;
	uint16_t cmdVal;

	/* set the DMA mask */
	if (pci_set_dma_mask(dev, 0xfffffff0ULL)) {
	}

	/* set the desired PCI dev to be master, this internally sets the latency timer */
	pci_set_master(dev);
	pci_write_config_byte(dev, PCI_LATENCY_TIMER, 0x80);

	/* Add support memory write invalidate */
	retVal = pci_set_mwi(dev);

	pci_read_config_word(dev, PCI_COMMAND, (u16 *)&cmdVal);
	/* and set the master bit in command register. */
	cmdVal |= PCI_COMMAND_MEMORY
		| PCI_COMMAND_MASTER
		| PCI_COMMAND_SERR;
	/* and clear the interrupt disable bit in command register. */
	cmdVal &= ~PCI_COMMAND_INTX_DISABLE;
	pci_write_config_word(dev, PCI_COMMAND, cmdVal);
}

/* Allow the DSP to generate interrupts to the Host. */
static void HAL_PciEnableDspInterrupt(struct dsppcie_dev *dpe)
{
	iowrite32(1, dpe->appreg_virt_addr+LEGACY_A_IRQ_ENABLE_SET/4);
}

/* Disable the DSP to generate interrupts to the Host. */
static void HAL_PciDisableDspInterrupt(struct dsppcie_dev *dpe)
{
	iowrite32(1, dpe->appreg_virt_addr+LEGACY_A_IRQ_ENABLE_CLR/4);
}

/* Perform DSP cores and periphrals rest */
static void dspLocalReset(struct dsppcie_dev *dpe) 
{
	uint32_t i, bootEntryAddr = 0;

	/* Local reset of all cores */
	coreLocalReset(dpe, PD8,  LPSC_C0_TIM0, LOC_RST_ASSERT);
	coreLocalReset(dpe, PD9,  LPSC_C1_TIM1, LOC_RST_ASSERT);
	coreLocalReset(dpe, PD10, LPSC_C2_TIM2, LOC_RST_ASSERT);
	coreLocalReset(dpe, PD11, LPSC_C3_TIM3, LOC_RST_ASSERT);
	coreLocalReset(dpe, PD12, LPSC_C4_TIM4, LOC_RST_ASSERT);
	coreLocalReset(dpe, PD13, LPSC_C5_TIM5, LOC_RST_ASSERT);
	coreLocalReset(dpe, PD14, LPSC_C6_TIM6, LOC_RST_ASSERT);
	coreLocalReset(dpe, PD15, LPSC_C7_TIM7, LOC_RST_ASSERT);

	/* Disable all other modules */
	setPscState(dpe, PD0, LPSC_EMIF16_SPI, PSC_SWRSTDISABLE);
	setPscState(dpe, PD0, LPSC_TSIP, PSC_SWRSTDISABLE);
	setPscState(dpe, PD1, LPSC_DEBUG, PSC_SWRSTDISABLE);
	setPscState(dpe, PD1, LPSC_TETB_TRC, PSC_SWRSTDISABLE);
	setPscState(dpe, PD2, LPSC_SA, PSC_SWRSTDISABLE);
	setPscState(dpe, PD2, LPSC_SGMII, PSC_SWRSTDISABLE);
	setPscState(dpe, PD2, LPSC_PA, PSC_SWRSTDISABLE);
	//setPscState(dpe, PD3, LPSC_PCIE, PSC_SWRSTDISABLE);
	setPscState(dpe, PD4, LPSC_SRIO, PSC_SWRSTDISABLE);
	setPscState(dpe, PD5, LPSC_HYPER, PSC_SWRSTDISABLE);
	//setPscState(dpe, PD6, LPSC_RESERV, PSC_SWRSTDISABLE);
	setPscState(dpe, PD7, LPSC_MSMCRAM, PSC_SWRSTDISABLE);

	for (i = 0; i < 8; i++) {
		pushData(dpe, localResetCode, i, &bootEntryAddr);
		if (setBootAddrIpcgr(dpe, i, bootEntryAddr) == 0) {
			printk("Core %d is not ready !!! \n", i);
		}
	}

	/* Enable all other modules */
	setPscState(dpe, PD0, LPSC_EMIF16_SPI, PSC_ENABLE);
	setPscState(dpe, PD0, LPSC_TSIP, PSC_ENABLE);
	setPscState(dpe, PD1, LPSC_DEBUG, PSC_ENABLE);
	setPscState(dpe, PD1, LPSC_TETB_TRC, PSC_ENABLE);
	setPscState(dpe, PD2, LPSC_PA, PSC_ENABLE);
	setPscState(dpe, PD2, LPSC_SGMII, PSC_ENABLE);
	setPscState(dpe, PD2, LPSC_SA, PSC_ENABLE);
	//setPscState(dpe, PD3, LPSC_PCIE, PSC_ENABLE);
	setPscState(dpe, PD4, LPSC_SRIO, PSC_ENABLE);
	setPscState(dpe, PD5, LPSC_HYPER, PSC_ENABLE);
	//setPscState(dpe, PD6, LPSC_RESERV, PSC_ENABLE);
	setPscState(dpe, PD7, LPSC_MSMCRAM, PSC_ENABLE);

	/* Local out of reset of all cores */
	coreLocalReset(dpe, PD8,  LPSC_C0_TIM0, LOC_RST_DEASSERT);
	coreLocalReset(dpe, PD9,  LPSC_C1_TIM1, LOC_RST_DEASSERT);
	coreLocalReset(dpe, PD10, LPSC_C2_TIM2, LOC_RST_DEASSERT);
	coreLocalReset(dpe, PD11, LPSC_C3_TIM3, LOC_RST_DEASSERT);
	coreLocalReset(dpe, PD12, LPSC_C4_TIM4, LOC_RST_DEASSERT);
	coreLocalReset(dpe, PD13, LPSC_C5_TIM5, LOC_RST_DEASSERT);
	coreLocalReset(dpe, PD14, LPSC_C6_TIM6, LOC_RST_DEASSERT);
	coreLocalReset(dpe, PD15, LPSC_C7_TIM7, LOC_RST_DEASSERT);
}

/* Reset a particular CorePac, 6678 Data Manual, section 7.4.4 initiated by LPSC MMRs */
static void coreLocalReset(struct dsppcie_dev *dpe, uint32_t pid, uint32_t mid, uint32_t state) 
{
	uint32_t temp, counter = 0;

	/* Set MST_PRIV bit to access PSC via PCIE */
	iowrite32(((ioread32(dpe->appreg_virt_addr + PRIORITY/4))|0x00010000), dpe->appreg_virt_addr + PRIORITY/4);    

	/* Temporarily re-map IB region 3 from DDR memory to PSC registers */
	iowrite32(PSC_BASE_ADDRESS, dpe->appreg_virt_addr + IB_OFFSET(3)/4);

    /* Now it points to the start of PSC_BASE_ADDRESS */
	temp = myIoread32(dpe->ddr_vrit_addr + MDCTL(mid)/4);
	if (state == 0) {
		/* Reset assert */
		temp = ((temp & ~0x1F) | PSC_ENABLE) & (~0x100);
		printk("Start local reset assert for core (module id): %d ...\n", mid);
	} else 	{
		/* Reset de-assert */
		temp = (temp & ~0x1F) | PSC_ENABLE | (1 << 8);
		printk("Start local reset de-assert for core (module id): %d ...\n", mid);
	}

	myIowrite32(temp, dpe->ddr_vrit_addr + MDCTL(mid)/4);    /* Assert/De-assert local reset */

	/* No previous transition in progress */
	counter = 0;
	while (true) {
		temp = myIoread32(dpe->ddr_vrit_addr + PTSTAT/4);
		if ((temp & (1 << pid)) == 0) break;
		mdelay(1);
		counter ++;
		if (counter > 10) {
			printk("Previous transition in progress pid %d mid %d state: %d\n", pid, mid, state);
			break;
		}
	}

	myIowrite32((1 << pid), dpe->ddr_vrit_addr + PTCMD/4); 

	/* Current transition finished */
	counter = 0;
	while (true) {
		temp = myIoread32(dpe->ddr_vrit_addr + PTSTAT/4);
		if ((temp & (1 << pid)) == 0) break;
		mdelay(1);
		counter ++;
		if (counter > 10) {
			printk("Current transition in progress pid %d mid %d state: %d\n", pid, mid, state);
			break;
		}
	}

	/* Verifying state change */
	counter = 0;
	while (true) {
		temp = myIoread32(dpe->ddr_vrit_addr + MDSTAT(mid)/4);
		if ((temp & 0x1F) == 3) break;
		mdelay(1);
		counter ++;
		if (counter > 10) {
			printk("MD stat for pid %d mid %d state: %d timeout\n", pid, mid, state);
			break;
		}
	}

	/* Restore pointer */
	iowrite32(DDR_START, dpe->appreg_virt_addr + IB_OFFSET(3)/4);   /* Point to PCIE application registers */
}

/*
 * Set a new power state for the specified domain id in a power controler
 * domain. Wait for the power transition to complete.
 */
static void setPscState(struct dsppcie_dev *dpe, uint32_t pid, uint32_t mid, uint32_t state)
{
	uint32_t mdctl, pdctl, temp, counter = 0;

	/* Point to PCIE application registers */
	/* Set MST_PRIV bit to access PSC via PCIE */
	iowrite32(((ioread32(dpe->appreg_virt_addr + PRIORITY/4))|0x00010000), dpe->appreg_virt_addr + PRIORITY/4);    

	/* Temporarily re-map IB region 3 from DDR memory to PSC registers */
	iowrite32(PSC_BASE_ADDRESS, dpe->appreg_virt_addr + IB_OFFSET(3)/4); 

	/* Now it points to the start of PSC_BASE_ADDRESS */
	mdctl = myIoread32(dpe->ddr_vrit_addr + MDCTL(mid)/4);
	pdctl = myIoread32(dpe->ddr_vrit_addr + PDCTL(pid)/4);

	/* No previous transition in progress */
	counter = 0;
	while (true) {
		temp = myIoread32(dpe->ddr_vrit_addr + PTSTAT/4);
		if ((temp & (1 << pid)) == 0) break;
		mdelay(1);
		counter ++;
		if (counter > 10) {
			printk("Previous transition in progress pid %d mid %d state: %d\n", pid, mid, state);
			break;
		}
	}

	/* Set power domain control */
	myIowrite32(pdctl | 0x1, dpe->ddr_vrit_addr + PDCTL(pid)/4);

	/* Set MDCTL NEXT to new state */
	mdctl = ((mdctl) & ~(0x1f)) | state;
	myIowrite32(mdctl, dpe->ddr_vrit_addr + MDCTL(mid)/4);

	/* Start power transition by setting PTCMD GO to 1 */
	temp = myIoread32(dpe->ddr_vrit_addr + PTCMD/4);
	myIowrite32(temp | (0x1<<pid), dpe->ddr_vrit_addr + PTCMD/4);

	/* Current transition finished */
	counter = 0;
	while (true) {
		temp = myIoread32(dpe->ddr_vrit_addr + PTSTAT/4);
		if ((temp & (1 << pid)) == 0) break;
		mdelay(1);
		counter ++;
		if (counter > 10) {
			printk("Current transition in progress pid %d mid %d state: %d\n", pid, mid, state);
			break;
		}
	}

	/* Verifying state change */
	counter = 0;
	while (true) {
		temp = myIoread32(dpe->ddr_vrit_addr + MDSTAT(mid)/4);
		if ((temp & 0x1F) == state) break;
		mdelay(1);
		counter ++;
		if (counter > 10) {
			printk("MD stat for pid %d mid %d state: %d timeout\n", pid, mid, state);
			break;
		}
	}

	/* Restore pointer */
	iowrite32(DDR_START, dpe->appreg_virt_addr + IB_OFFSET(3)/4);   /* Point to PCIE application registers */
}

/* Write boot entry point into DSP_BOOT_ADDR0 and send an IPC */
static uint32_t setBootAddrIpcgr(struct dsppcie_dev *dpe, uint32_t core, uint32_t addr)  
{
	/* Set MST_PRIV bit to access PSC via PCIE */
	iowrite32(((ioread32(dpe->appreg_virt_addr + PRIORITY/4))|0x00010000), dpe->appreg_virt_addr + PRIORITY/4);    

	/* Temporarily re-map IB region 3 from DDR memory to chip level registers */
	iowrite32(CHIP_LEVEL_BASE_ADDRESS, dpe->appreg_virt_addr + IB_OFFSET(3)/4);  

	/* Now it points to the start of CHIP_LEVEL_BASE_ADDRESS */
	/* Unlock KICK0, KICK1 */
	myIowrite32(KICK0_UNLOCK, dpe->ddr_vrit_addr + KICK0/4); 
	myIowrite32(KICK1_UNLOCK, dpe->ddr_vrit_addr + KICK1/4);

	/* Check if the last 10 bits of addr is 0 */
	if ((addr & 0x3f) != 0) {
		printk("The address is not 1K aligned.\n");
		return 0;
	}

	myIowrite32(addr, dpe->ddr_vrit_addr + DSP_BOOT_ADDR(core)/4);
	myIowrite32(1, dpe->ddr_vrit_addr + IPCGR(core)/4);  

	mdelay(1); 

	/* Restore pointer */
	iowrite32(DDR_START, dpe->appreg_virt_addr + IB_OFFSET(3)/4);   /* Point to PCIE application registers */

	return 1;
}

/* Parse function for DSP boot image array */
static void pushData(struct dsppcie_dev *dpe, uint8_t *pDspCode, uint8_t coreNum, uint32_t *bootEntryAddr)
{
	uint32_t i, j, tempArray[BLOCK_TRANSFER_SIZE/4];
	uint32_t size, section = 0, totalSize = 0;
	uint32_t count, remainder, startAddr, temp;
	uint8_t newCoreNum;

	/* Get the boot entry address */
	*bootEntryAddr = byteTo32bits(pDspCode);
	printk("Boot entry address is 0x%8x\n", *bootEntryAddr);
	pDspCode +=4;

	while(1) {

		/* Get the size */
		size = byteTo32bits(pDspCode);
		if(size == 0) break;

		if ((size/4)*4 != size) {
			size = ((size/4)+1)*4;
		}

		totalSize += size;
		section++;
		pDspCode += 4;
		startAddr = byteTo32bits(pDspCode);

		/* In case there are several sections within different memory regions */
		temp = (startAddr & 0xFF000000) >> 24;

		if (temp == 0x00 || ((temp >> 4) == 0x1)) {
			if (coreNum < 8) {
				/* Write address like 0x00800000 to core 1, 2, ... */
				newCoreNum = coreNum;
			} else {
				newCoreNum = 0;
			}
		} else if (temp == 0x0C) {
			newCoreNum = 8;
		} else {
			newCoreNum = 9;
		}

		pDspCode+= 4;
		count = size/BLOCK_TRANSFER_SIZE;
		remainder = size - count * BLOCK_TRANSFER_SIZE;

		for(i = 0; i < count; i++) {
			for (j = 0; j < BLOCK_TRANSFER_SIZE/4; j++) {
				tempArray[j] = byteTo32bits(pDspCode);
				pDspCode += 4;
			}
			/* Transfer boot tables to DSP */
			writeDSPMemory(dpe, newCoreNum, startAddr, tempArray, BLOCK_TRANSFER_SIZE); 
			startAddr += BLOCK_TRANSFER_SIZE;
		}

		for (j = 0; j < remainder/4; j++) {
			tempArray[j] = byteTo32bits(pDspCode);
			pDspCode += 4;
		}
		writeDSPMemory(dpe, newCoreNum, startAddr, tempArray, remainder); 
		// printk("Section %d started at 0x%8x with size 0x%8x bytes written to core %d\n", section, startAddr, size, newCoreNum);
	}
	printk("Total %d sections, 0x%x bytes of data were written\n", section, totalSize);
}


/* Convert 4 bytes to 32 bits long word */
static uint32_t byteTo32bits(uint8_t *pDspCode)
{
	uint32_t i, temp;

	temp = *pDspCode++;
	for(i = 0; i < 3;i++) {
		temp <<= 8;
		temp |= *pDspCode++;
	}
	return(temp);
}

/* Endian swap: 0xA0B1C2D3 to 0xD3C2B1A0 */
static void swap4bytes(uint32_t *pDspCode, uint32_t size)
{
	uint32_t i, temp;

	for(i = 0; i < size; i += 4, pDspCode++) {
		temp = *pDspCode;
		temp = (temp>>24) |
			((temp<<8) & 0x00FF0000) |
			((temp>>8) & 0x0000FF00) |
			(temp<<24);

		*pDspCode = temp;
	}
}

/*
 * Write a bloack of data into DSP memory from Linux host.
 * Note the data buffer is in 32-bit format, unit of length is byte.
 * coreNum: to write data: 0-7 for cores 0-7; 8 for MSMC; 9 for DDR.
 */
static uint32_t writeDSPMemory(struct dsppcie_dev *dpe, uint32_t coreNum, uint32_t DSPMemAddr, uint32_t *buffer, uint32_t length) 
{
	uint32_t i, offset, tempReg = 0;
	uint32_t *ptr;

	if (length == 0) {
		return 0;
	}

	switch (coreNum) {
		case 0:
		case 1:
		case 2:
		case 3:
		case 4:
		case 5:
		case 6:
		case 7: 
			DSPMemAddr &= 0x00FFFFFF;
			tempReg = ioread32(dpe->appreg_virt_addr + IB_OFFSET(1)/4);
			iowrite32(tempReg + coreNum*0x01000000, dpe->appreg_virt_addr + IB_OFFSET(1)/4); /* pointing to a different core */

			if (DSPMemAddr < LL2_START) {
				return 0;
			} else {  
				offset = DSPMemAddr - LL2_START;
				ptr = (uint32_t *)(dpe->localL2_virt_addr) + offset/4;
			}
			break;
		case 8:  /* this is for MSMC */
			if (DSPMemAddr < MSMC_START) {
				return 0;
			} else {
				offset = DSPMemAddr - MSMC_START;
				ptr = (uint32_t *)(dpe->msmc_virt_addr) + offset/4;
			}		
			break;   
		case 9:  /* this is for DDR */
			if (DSPMemAddr < DDR_START) {
				return 0;
			} else {  
				offset = DSPMemAddr - DDR_START;
				ptr = (uint32_t *)(dpe->ddr_vrit_addr) + offset/4;
			}
			break; 
		default:
			printk("Use coreNum 0-7 for core 0-7 of EVMC6678L, or coreNum 0-3 for core 0-3 of EVMC6670L; coreNum 8 for MSMC and coreNum 9 for DDR.\n");
			return 0;
			break;
	}     

	for (i = 0; i < length/4; i++) {
#if BIG_ENDIAN
		swap4bytes(&buffer[i], 4);
#endif
		*ptr = buffer[i];
		ptr++; 
	}

	if ((coreNum >= 0)&&(coreNum <= 7)) {
		iowrite32(tempReg, dpe->appreg_virt_addr + IB_OFFSET(1)/4);  /* Restore IB_OFFSET1 */
	}

	return length;
}

/*
 * Read a block of DSP memory by Linux host.
 * Note the data buffer is in 32-bit format, unit of length is byte.
 * coreNum: to read data: 0-7 for cores 0-7; 8 for MSMC; 9 for DDR.
 */
static uint32_t readDSPMemory(struct dsppcie_dev *dpe, uint32_t coreNum, uint32_t DSPMemAddr, uint32_t *buffer, uint32_t length) 
{
	uint32_t i, offset, tempReg = 0;
	uint32_t *ptr;

	if (length == 0) {
		return 0;
	}  

	switch (coreNum) {
		case 0:
		case 1:
		case 2:
		case 3:
		case 4:
		case 5:
		case 6:
		case 7: 
			DSPMemAddr &= 0x00FFFFFF;
			tempReg = ioread32(dpe->appreg_virt_addr + IB_OFFSET(1)/4);
			iowrite32(tempReg + coreNum*0x1000000, dpe->appreg_virt_addr + IB_OFFSET(1)/4);        /* pointing to a different core */
			if (DSPMemAddr < LL2_START) {
				return 0;
			} else {  
				offset = DSPMemAddr - LL2_START;
				ptr = (uint32_t *)(dpe->localL2_virt_addr) + offset/4;
			}
			break;
		case 8:  /* this is for MSMC */
			if (DSPMemAddr < MSMC_START) {
				return 0;
			} else {                   
				offset = DSPMemAddr - MSMC_START;
				ptr = (uint32_t *)(dpe->msmc_virt_addr) + offset/4;
			}
			break;   
		case 9:  /* this is for DDR */
			if (DSPMemAddr < DDR_START) {
				return 0;
			} else {                   
				offset = DSPMemAddr - DDR_START;
				ptr = (uint32_t *)(dpe->ddr_vrit_addr) + offset/4;
			}
			break; 
		default:
			printk("Use coreNum 0-7 for core 0-7 of EVMC6678L, or coreNum 0-3 for core 0-3 of EVMC6670L; coreNum 8 for MSMC and coreNum 9 for DDR.\n");
			return 0;
			break;
	}     

	for (i = 0; i < length/4; i++) {
		buffer[i] = *ptr;
#if BIG_ENDIAN
		swap4bytes(&buffer[i], 4);
#endif
		ptr++;
	}

	if ((coreNum >= 0)&&(coreNum <= 7)) {
		iowrite32(tempReg, dpe->appreg_virt_addr + IB_OFFSET(1)/4);  /* Restore IB_OFFSET1 */
	}

	return length;
}

/*
 * Move DMAs contents from DSP memory to GPP Memory. For DSP this is outbound write.
 * flag: 0: Move data inside DSP; 1: Move data between GPP and DSP
 */
static void HAL_readDMA(struct dsppcie_dev *dpe, uint32_t srcAddr, uint32_t dstAddr, uint32_t size, uint32_t flag)
{
	uint32_t tmp, pageBase, i, tSize;

	/* Point to PCIE application registers */

	/* Move data between GPP and DSP, need to program PCIE OB registers */
	if (flag) { 
		iowrite32(0x0, dpe->appreg_virt_addr + OB_SIZE/4); /* 1MB outbound translation size */

		if (size <= PCIE_ADLEN_1MB) {
			pageBase = dstAddr & PCIE_1MB_BITMASK;
			iowrite32(pageBase|0x1, dpe->appreg_virt_addr + OB_OFFSET_INDEX(0)/4);
			iowrite32(0x0, dpe->appreg_virt_addr + OB_OFFSET_HI(0)/4);  
		}
		else {
			for (tmp = size, i = 0; tmp > 0; tmp -= PCIE_ADLEN_1MB, i++) {
				pageBase = (dstAddr + (PCIE_ADLEN_1MB * i)) & PCIE_1MB_BITMASK;
				iowrite32(pageBase|0x1, dpe->appreg_virt_addr + OB_OFFSET_INDEX(i)/4);
				iowrite32(0x0, dpe->appreg_virt_addr + OB_OFFSET_HI(i)/4);
			}
		}
	}

	/* Temporarily re-map IB region 3 from DDR memory to EDMA registers */
	iowrite32(EDMA_TPCC0_BASE_ADDRESS, dpe->appreg_virt_addr + IB_OFFSET(3)/4);  

	/* Now it points to the start of EDMA_TPCC0_BASE_ADDRESS */

	while (true) {
		/* Use TC0 for DBS = 128 bytes */
		myIowrite32(0x0, dpe->ddr_vrit_addr + DMAQNUM0/4);                

		/* Set the interrupt enable for 1st Channel (IER). */
		myIowrite32(0x1, dpe->ddr_vrit_addr + IESR/4);

		/* Clear any pending interrupt (IPR). */
		myIowrite32(0x1, dpe->ddr_vrit_addr + ICR/4);

		/* Populate the Param entry. */
		myIowrite32(0x00100004, dpe->ddr_vrit_addr + PARAM_0_OPT/4);    /* Enable SYNCDIM and TCINTEN, TCC = 0 */

		if (flag == 1) {
			/* Calculate the DSP PCI address for the PC address */
			tmp = PCIE_DATA + (dstAddr & ~PCIE_1MB_BITMASK);
			myIowrite32(tmp, dpe->ddr_vrit_addr + PARAM_0_DST/4);
		} else {
			myIowrite32(dstAddr, dpe->ddr_vrit_addr + PARAM_0_DST/4);
		}

		/* Calculate the A & B count */
		if (size > PCIE_TRANSFER_SIZE)  {
			tmp = size/PCIE_TRANSFER_SIZE;
			tSize = tmp*PCIE_TRANSFER_SIZE;
			size -= (tmp*PCIE_TRANSFER_SIZE);
			tmp <<= 16;
			tmp |= PCIE_TRANSFER_SIZE;
		}
		else {
			tmp = 0x10000|size;
			tSize = size;
			size = 0;
		}

		myIowrite32(tmp, dpe->ddr_vrit_addr + PARAM_0_A_B_CNT/4);
		myIowrite32(srcAddr,dpe->ddr_vrit_addr + PARAM_0_SRC/4);

		myIowrite32(((PCIE_TRANSFER_SIZE<<16)|PCIE_TRANSFER_SIZE), dpe->ddr_vrit_addr + PARAM_0_SRC_DST_BIDX/4);
		myIowrite32(0xFFFF, dpe->ddr_vrit_addr + PARAM_0_LINK_BCNTRLD/4);
		myIowrite32(0x0, dpe->ddr_vrit_addr + PARAM_0_SRC_DST_CIDX/4);

		/* C Count is set to 1 since mostly size will not be more than 1.75GB */
		myIowrite32(0x1, dpe->ddr_vrit_addr + PARAM_0_CCNT/4);

		/* Set the Event Enable Set Register. */
		myIowrite32(0x1, dpe->ddr_vrit_addr + EESR/4);

		/* Set the event set register. */
		myIowrite32(0x1, dpe->ddr_vrit_addr + ESR/4);

		/* wait for current DMA to finish. */
		while (true) {
			/* check in steps of 10 usec. */
			udelay(10);
			tmp = myIoread32(dpe->ddr_vrit_addr + IPR/4);
			if ((tmp & 0x1) == 1) {
				break;
			}
		}

		if (size != 0) {
			srcAddr += tSize;
			dstAddr += tSize;
		} else {
			break;
		}
	} 

	/* Clear any pending interrupt. */
	myIowrite32(1, dpe->ddr_vrit_addr + ICR/4);

	/* Restore pointer */
	iowrite32(DDR_START, dpe->appreg_virt_addr + IB_OFFSET(3)/4);	/* Point to PCIE application registers */
}

/*
 * Move DMAs contents from GPP memory to DSP Memory. For DSP this is outbound read.
 * (这里的outbound指的是DMA的目的地址需要执行出站转换，并不是指数据流向；
 *  read是DSP从GPP读数据，指的是数据流向，入站方向。)
 * flag: 0: Move data inside DSP; 1: Move data between GPP and DSP
 */
static void HAL_writeDMA(struct dsppcie_dev *dpe, uint32_t srcAddr, uint32_t dstAddr, uint32_t size, uint32_t flag)
{
	uint32_t tmp, pageBase, i, tSize;
    

	/* Move data between GPP and DSP, need to program PCIE OB registers */
	if (flag) { 
		iowrite32(0x0, dpe->appreg_virt_addr + OB_SIZE/4); /* 1MB outbound translation size */

		if (size <= PCIE_ADLEN_1MB) {
			pageBase = srcAddr & PCIE_1MB_BITMASK;
			iowrite32(pageBase|0x1, dpe->appreg_virt_addr + OB_OFFSET_INDEX(0)/4);
			iowrite32(0x0, dpe->appreg_virt_addr + OB_OFFSET_HI(0)/4);  
		}
		else {
			for (tmp = size, i = 0; tmp > 0; tmp -= PCIE_ADLEN_1MB, i++) {
				pageBase = (srcAddr + (PCIE_ADLEN_1MB * i)) & PCIE_1MB_BITMASK;
				iowrite32(pageBase|0x1, dpe->appreg_virt_addr + OB_OFFSET_INDEX(i)/4);
				iowrite32(0x0, dpe->appreg_virt_addr + OB_OFFSET_HI(i)/4);
			}
		}
	}

	/* Temporarily re-map IB region 3 from DDR memory to EDMA registers */
	iowrite32(EDMA_TPCC0_BASE_ADDRESS, dpe->appreg_virt_addr + IB_OFFSET(3)/4);  

	/* Now it points to the start of EDMA_TPCC0_BASE_ADDRESS */

	while (true) {
		/* Use TC0 for DBS = 128 bytes */
		myIowrite32(0x0, dpe->ddr_vrit_addr + DMAQNUM0/4);                

		/* Set the interrupt enable for 1st Channel (IER). */
		myIowrite32(0x1, dpe->ddr_vrit_addr + IESR/4);

		/* Clear any pending interrupt (IPR). */
		myIowrite32(0x1, dpe->ddr_vrit_addr + ICR/4);

		/* Populate the Param entry. */
		myIowrite32(0x00100004, dpe->ddr_vrit_addr + PARAM_0_OPT/4);    /* Enable SYNCDIM and TCINTEN, TCC = 0 */

		if (flag == 1) {
			/* Calculate the DSP PCI address for the PC address */
			tmp = PCIE_DATA + (srcAddr & ~PCIE_1MB_BITMASK);
			myIowrite32(tmp, dpe->ddr_vrit_addr + PARAM_0_SRC/4);
		} else {
			myIowrite32(srcAddr, dpe->ddr_vrit_addr + PARAM_0_SRC/4);
		}

		/* Calculate the A & B count */
		if (size > PCIE_TRANSFER_SIZE)  {
			tmp = size/PCIE_TRANSFER_SIZE;
			tSize = tmp*PCIE_TRANSFER_SIZE;
			size -= (tmp*PCIE_TRANSFER_SIZE);
			tmp <<= 16;
			tmp |= PCIE_TRANSFER_SIZE;
		}
		else {
			tmp = 0x10000|size;
			tSize = size;
			size = 0;
		}

		myIowrite32(tmp, dpe->ddr_vrit_addr + PARAM_0_A_B_CNT/4);
		myIowrite32(dstAddr, dpe->ddr_vrit_addr + PARAM_0_DST/4);

		myIowrite32(((PCIE_TRANSFER_SIZE<<16)|PCIE_TRANSFER_SIZE), dpe->ddr_vrit_addr + PARAM_0_SRC_DST_BIDX/4);
		myIowrite32(0xFFFF, dpe->ddr_vrit_addr + PARAM_0_LINK_BCNTRLD/4);
		myIowrite32(0x0, dpe->ddr_vrit_addr + PARAM_0_SRC_DST_CIDX/4);

		/* C Count is set to 1 since mostly size will not be more than 1.75GB */
		myIowrite32(0x1, dpe->ddr_vrit_addr + PARAM_0_CCNT/4);

		/* Set the Event Enable Set Register. */
		myIowrite32(0x1, dpe->ddr_vrit_addr + EESR/4);

		/* Set the event set register. */
		myIowrite32(0x1, dpe->ddr_vrit_addr + ESR/4);

		/* wait for current DMA to finish. */
		while (true) {
			/* check in steps of 10 usec. */
			udelay(10);
			tmp = myIoread32(dpe->ddr_vrit_addr + IPR/4);
			if ((tmp & 0x1) == 1) {
				break;
			}
		}

		if (size != 0) {
			srcAddr += tSize;
			dstAddr += tSize;
		} else {
			break;
		}
	} 

	/* Clear any pending interrupt. */
	myIowrite32(1, dpe->ddr_vrit_addr + ICR/4);

	/* Restore pointer */
	iowrite32(DDR_START, dpe->appreg_virt_addr + IB_OFFSET(3)/4);	//Point to PCIE application registers
}


/* This function check whether interrupt is generated by C667x or not. */
static bool HAL_CheckPciInterrupt(struct dsppcie_dev *dpe)
{
	return ioread32(dpe->appreg_virt_addr + EP_IRQ_STATUS/4);
}

/* Clear pending interrupt from DSP to Host. */
static void HAL_PciClearDspInterrupt(struct dsppcie_dev *dpe)
{
	iowrite32(1, dpe->appreg_virt_addr + EP_IRQ_CLR/4);
}

/*
 *  Module initialization, registers pci device.
 */
static int __init dsppcie_init(void) {
    int res = 0;
    printk(KERN_DEBUG "-----------------------------------------------------\n");
    printk(KERN_DEBUG "Init %s module\n", DSPPCIE_CDEV_NAME);
    printk(KERN_DEBUG "Register driver for %s\n", DSPPCIE_CDEV_NAME);
    res = pci_register_driver(&dsppcie_driver);     /* register this driver with the PCI bus driver */
    return res;
}

/*
 *  Module cleanup, unregisters device.
 */
static void __exit dsppcie_exit(void) {
    printk(KERN_DEBUG "Cleanup %s moudle\n", DSPPCIE_CDEV_NAME);
    printk(KERN_DEBUG "Unregister driver for %s\n", DSPPCIE_CDEV_NAME);
    pci_unregister_driver(&dsppcie_driver);
    printk(KERN_DEBUG "------------------------------------------------------\n");
}

module_init(dsppcie_init);
module_exit(dsppcie_exit);
