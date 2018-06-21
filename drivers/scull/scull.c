#include <linux/module.h>
#include <linux/init.h>
#include <linux/moduleparam.h>

#include <linux/kernel.h>   /* printk() */
#include <linux/slab.h>     /* kmalloc() */
#include <linux/fs.h>       /* everything... */
#include <linux/errno.h>    /* error codes */
#include <linux/types.h>    /* size_t */

#include <linux/fcntl.h>    /* O_ACCMODE */
#include <linux/seq_file.h>
#include <linux/cdev.h>

//#include <asm/system.h>     /* cli(), *_flags */
#include <linux/version.h>
#if LINUX_VERSION_CODE > KERNEL_VERSION(3, 3, 0)
    #include <asm/switch_to.h>
#else
    #include <asm/system.h>
#endif
#include <asm/uaccess.h>    /* copy_*_user */

#include "scull.h"          /* local definitions */

/*
 * Our parameters which can be set at load time.
 */
int scull_major =   SCULL_MAJOR;
int scull_minor =   0;
int scull_nr_devs = SCULL_NR_DEVS;  /* number of bare scull devices */
int scull_quantum = SCULL_QUANTUM;
int scull_qset = SCULL_QSET;

module_param(scull_major, int, S_IRUGO);
module_param(scull_minor, int, S_IRUGO);
module_param(scull_nr_devs, int, S_IRUGO);
module_param(scull_quantum, int, S_IRUGO);
module_param(scull_qset, int, S_IRUGO);

MODULE_LICENSE("Dual BSD/GPL");

struct scull_dev *scull_devices;    /* allocated in scull_init_module */

/*
 * Empty out the scull device; must be called with the device
 * semaphore held.
 */
int scull_trim(struct scull_dev *dev)
{
	struct scull_qset *next, *dptr;
	int qset = dev->qset;   /* "dev" is not-null */
	int i;

	for (dptr = dev->data; dptr; dptr = next) { /* all the list items */
		if (dptr->data) {
			for (i = 0; i < qset; i++)
				kfree(dptr->data[i]);
			kfree(dptr->data);
			dptr->data = NULL;
		}
		next = dptr->next;
		kfree(dptr);
	}
	dev->size = 0;
	dev->quantum = scull_quantum;
	dev->qset = scull_qset;
	dev->data = NULL;
    printk(KERN_INFO "sucll: scull_trim for device: %d\n", (int)(dev - scull_devices));
	return 0;
}

/*
 * Open and close
 */
int scull_open(struct inode *inode, struct file *filp)
{
	struct scull_dev *dev; /* device information */
    
    dev = container_of(inode->i_cdev, struct scull_dev, cdev);
    filp->private_data = dev; /* for other methods */
    
	/* now trim to 0 the length of the device if open was write-only */
    if ( (filp->f_flags & O_ACCMODE) == O_WRONLY) {
        if (down_interruptible(&dev->sem))
            return -ERESTARTSYS;
        scull_trim(dev); /* ignore errors */
        up(&dev->sem);
    }
    printk(KERN_INFO "scull: scull_open for device: %d\n", (int)(dev - scull_devices));
    return 0;			/* success */
}

int scull_release(struct inode *inode, struct file *filp)
{
    struct scull_dev *dev = filp->private_data;
    printk(KERN_INFO "scull: scull_release for devices: %d\n", (int)(dev - scull_devices));
    return 0;
}

/*
 * Follow the list
 */
struct scull_qset *scull_follow(struct scull_dev *dev, int n)
{
    struct scull_qset *qs = dev->data;

    printk(KERN_INFO "scull: scull_follow item: %d for device: %d\n", n, (int)(dev - scull_devices));
        /* Allocate first qset explicitly if need be */
    if (! qs) {
        qs = dev->data = kmalloc(sizeof(struct scull_qset), GFP_KERNEL);
        if (qs == NULL)
            return NULL;    /* Never mind */
        memset(qs, 0, sizeof(struct scull_qset));
    }

    /* Then follow the list */
    while (n--) {
        if (!qs->next) {
            qs->next = kmalloc(sizeof(struct scull_qset), GFP_KERNEL);
            if (qs->next == NULL)
                return NULL;    /* Never mind */
            memset(qs->next, 0, sizeof(struct scull_qset));
        }
        qs = qs->next;
        continue;
    }
    return qs;
}

/*
 * Data management: read and write
 */
ssize_t scull_read(struct file *filp, char __user *buf, size_t count,
        loff_t *f_pos)
{
    struct scull_dev *dev = filp->private_data;
    struct scull_qset *dptr;    /* the first listitem */
    int quantum = dev->quantum, qset = dev->qset;
    int itemsize = quantum * qset;  /* how many bytes in the listitem */
    int item, s_pos, q_pos, rest;
    ssize_t retval = 0;

    if (down_interruptible(&dev->sem))
        return -ERESTARTSYS;
    if (*f_pos >= dev->size)
        goto out;
    if (*f_pos + count > dev->size)
        count = dev->size - *f_pos;

    /* 在量子集中寻找链表项、qset索引以及偏移量 */
    item = (long)*f_pos / itemsize;
    rest = (long)*f_pos % itemsize;
    s_pos = rest / quantum; q_pos = rest % quantum;

    /* 沿链表前行，直到正确的位置(在其他地方定义) */
    dptr = scull_follow(dev, item);
    
    if (dptr == NULL || !dptr->data || ! dptr->data[s_pos])
        goto out;   /* don't fill holes */

    /* 读取该量子的数据直到结尾 */
    if (count > quantum - q_pos)
        count = quantum - q_pos;

    if (copy_to_user(buf, dptr->data[s_pos] + q_pos, count)) {
        retval = -EFAULT;
        goto out;
    }
    *f_pos += count;
    retval = count;

out:
    up(&dev->sem);
    printk(KERN_INFO "scull: read %ld bytes from device: %d\n", retval, (int)(dev - scull_devices));
    return retval;
}

ssize_t scull_write(struct file *filp, const char __user *buf, size_t count,
        loff_t *f_pos)
{
    struct scull_dev *dev = filp->private_data;
    struct scull_qset *dptr;
    int quantum = dev->quantum, qset = dev->qset;
    int itemsize = quantum * qset;
    int item, s_pos, q_pos, rest;
    ssize_t retval = -ENOMEM;   /* "goto out" 语句使用的值 */

    if (down_interruptible(&dev->sem))
        return -ERESTARTSYS;

    /* 在量子集中寻找链表项、qset索引以及偏移量 */
    item = (long)*f_pos / itemsize;
    rest = (long)*f_pos % itemsize;
    s_pos = rest / quantum; q_pos = rest % quantum;

    /* 沿该链表前行，直到正确的位置(在其他地方定义) */
    dptr = scull_follow(dev, item);
    if (dptr == NULL)
        goto out;
    if (!dptr->data) {
        dptr->data = kmalloc(qset * sizeof(char *), GFP_KERNEL);
        if (!dptr->data)
            goto out;
        memset(dptr->data, 0, qset * sizeof(char *));
    }
    if (!dptr->data[s_pos]) {
        dptr->data[s_pos] = kmalloc(quantum, GFP_KERNEL);
        if (!dptr->data[s_pos])
            goto out;
    }
    /* 将数据写入该量子，直到结尾 */
    if (count > quantum - q_pos)
        count = quantum - q_pos;

    if (copy_from_user(dptr->data[s_pos]+q_pos, buf, count)) {
        retval = -EFAULT;
        goto out;
    }
    *f_pos += count;
    retval = count;

    /* 更新文件大小 */
    if (dev->size < *f_pos)
        dev->size = *f_pos;

out:
    up(&dev->sem);
    if (retval < 0)
        printk(KERN_WARNING "scull: write error for device: %d\n", (int)(dev - scull_devices));
    else
        printk(KERN_INFO "scull: write %ld bytes to device: %d\n", retval, (int)(dev - scull_devices));
    return retval;
}

/*
 * The "extended" operations -- only seek
 */
loff_t scull_llseek(struct file *filp, loff_t off, int whence)
{
    struct scull_dev *dev = filp->private_data;
    loff_t newpos;

    switch(whence) {
        case 0: /* SEEK_SET */
            newpos = off;
            break;

        case 1: /* SEEK_CUR */
            newpos = filp->f_pos + off;
            break;

        case 2: /* SEEK_END */
            newpos = dev->size + off;
            break;

        default: /* can't happen */
            return -EINVAL;
    }
    printk(KERN_INFO "scull: llseek %lld bytes from %d\n", off, whence);
    if (newpos < 0) return -EINVAL;
    filp->f_pos = newpos;
    return newpos;
}

struct file_operations scull_fops = {
    .owner =    THIS_MODULE,
    .llseek =   scull_llseek,
    .read =     scull_read,
    .write =    scull_write,
    .open =     scull_open,
    .release =  scull_release,
};
/*
 * Finally, the module stuff
 */

/* The cleanup function is used to handle initialization failure as well.
 * Thefore, it must be careful to work correctly even if some of the items
 * have not been initialized.
 */
void scull_cleanup_module(void)
{
    int i;
    dev_t devno = MKDEV(scull_major, scull_minor);

    /* Get rid of our char dev entiries */
    if (scull_devices) {
        for (i = 0; i < scull_nr_devs; ++i) {
            scull_trim(scull_devices + i);
            cdev_del(&scull_devices[i].cdev);
            printk(KERN_INFO "scull: delete %d:%d device\n", scull_major, scull_minor + i);
        }
        kfree(scull_devices);
    }

    /* clean_module is never called if registering failed */
    unregister_chrdev_region(devno, scull_nr_devs);
    printk(KERN_INFO "scull: unregister\n");
}

/*
 * Set up the char_dev structure for this device.
 */
static void scull_setup_cdev(struct scull_dev *dev, int index)
{
    int err, devno = MKDEV(scull_major, scull_minor + index);

    cdev_init(&dev->cdev, &scull_fops);
    dev->cdev.owner = THIS_MODULE;
    dev->cdev.ops = &scull_fops;    /* not sure what is this doing here!!, this is OLD STYLE */
    err = cdev_add(&dev->cdev, devno, 1);
    /* Fail gracefully if need be */
    if (err)
        printk(KERN_NOTICE "Error %d adding scull%d\n", err, index);
    else
        printk(KERN_INFO "scull: add %d:%d device\n", scull_major, scull_minor + index);
}

int scull_init_module(void)
{
    int result, i;
    dev_t dev = 0;

/*
 * Get a range of minor numbers to work with, asking for a dynamic
 * major unless directed otherwise at load time.
 */
    if (scull_major) {
        dev = MKDEV(scull_major, scull_minor);
        result = register_chrdev_region(dev, scull_nr_devs, "scull");
    } else {
        result = alloc_chrdev_region(&dev, scull_minor, scull_nr_devs,
                "scull");
        scull_major = MAJOR(dev);
    }
    if (result < 0) {
        printk(KERN_WARNING "scull: can't get major %d\n", scull_major);
        return result;
    }
    printk(KERN_INFO "scull: allocate major number: %d\n", scull_major);

    /*
     * allocate to devices -- we can't have them static, as the number
     * can be specified at load time.
     */
    scull_devices = kmalloc(scull_nr_devs * sizeof(struct scull_dev), GFP_KERNEL);
    if (!scull_devices) {
        result = -ENOMEM;
        goto fail;  /* Make this more graceful */
    }
    memset(scull_devices, 0, scull_nr_devs * sizeof(struct scull_dev));
    printk(KERN_INFO "scull: allocate %d scull_dev structures for these devices\n", scull_nr_devs);

    /* Initialize  each device */
    for (i = 0; i < scull_nr_devs; ++i) {
        scull_devices[i].quantum = scull_quantum;
        scull_devices[i].qset = scull_qset;
        sema_init(&scull_devices[i].sem, 1);
        scull_setup_cdev(&scull_devices[i], i);
    }

    return 0;   /* succeed */

fail:
    scull_cleanup_module();
    return result;
}

module_init(scull_init_module);
module_exit(scull_cleanup_module);
