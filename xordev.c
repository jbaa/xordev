/**
 * @file
 * @author Jakub Bartodziej <jakub.bartodziej@students.mimuw.edu.pl>
 * @version 1.0
 * @date 2012-05-14
 *
 * @section DESCRIPTION
 *
 * This file contains the code for xor machine driver implemented
 * as a Linux module.
 *
 * @section LICENSE
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#define DEBUG

#include <linux/module.h>
#include <linux/pci.h>
#include <linux/mod_devicetable.h>
#include <linux/stddef.h>
#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/device.h>
#include <linux/kdev_t.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/dma-mapping.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/pci.h>
#include <linux/completion.h>

#include <asm/uaccess.h>

MODULE_AUTHOR("Jakub Bartodziej <jakub.bartodziej@students.mimuw.edu.pl>");
MODULE_DESCRIPTION("xordev driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");

#define VENDOR 0x1af4
#define DEVICE 0x10fd

#define MAX_DEVICES 64
#define BUFFER_SIZE 4096
#define DEVICE_ADDRESS_SIZE 32

#define INTR_EN    0x00
#define SRC1       0x04
#define SRC2       0x08
#define DST        0x0c
#define COUNT      0x10
#define INTR_COUNT 0x14

#define _devt_dbg(devt, fmt, ...) \
    pr_debug("xordev %d %d: " fmt, MAJOR(devt), MINOR(devt), ##__VA_ARGS__)

#define print_ddata(devt) \
    _devt_dbg(devt, "in_use: %u, device_removed: %u, open_counter: %u, buffer_start: %u, \
            device_size: %u, src1.size: %u, src2.size: %u, dest.size: %u\n",  \
            ddata->in_use, ddata->device_removed, ddata->open_counter, ddata->buffer_start, \
            ddata->device_size, ddata->src1.size, ddata->src2.size, ddata->dest.size) \

#define devt_dbg(devt, fmt, ...) \
    do {    \
        _devt_dbg(devt, fmt, ##__VA_ARGS__);    \
        print_ddata(devt);  \
    } while (0)

/* Represents one of the files in /dev/ */
struct device_outlet {
    /* The character device. */
    struct cdev cdev;
    /* DMA buffer for access from host.
     *
     * The dma buffer consists of three regions:
     *   - Device region - Starts at buffer_start and contains device_size consecutive bytes.
     *                     Does not wrap at the end of the physical buffer. This region is
     *                     exclusively for the device to operate on. In source buffers it contains
     *                     data ready for processing and in destination buffer it is ensured
     *                     that the device can safely write there without loss of data.
     *   - Outlet region - Starts at buffer_start + device_size and contains size bytes.
     *                     This region can wrap around the end of the physical buffer.
     *                     In case of source outlet contains data that have been copied from
     *                     user space, but have yet to be passed to the device for processing.
     *                     In case of destination contains data that have already been copied
     *                     to user space, but have yet to be marked as ready to be written
     *                     by the device.
     *   - Free region   - The remainder of the buffer is the free region. In source outlets it
     *                     will be the destination for copies from userspace, in destionation
     *                     outlets it contains the data ready to be copied to userspace.
     *                     It starts after the end of the outlet region and contains bytes in
     *                     order of copying.
     */
    u8 *buffer_driver;
    /* DMA buffer for access from device. */
    dma_addr_t buffer_device;
    /* Used for queueing processes requesting access to the outlet (eg. wanting to write) */
    struct mutex mutex;
    /* Size of the outlet region */
    u32 size;
};

/**
 * Represents a single xor device.
 */
struct device_data {
    /* Means that this slot in device table is taken. */
    bool in_use;
    /* Used to inform processes that the device has been removed and they
     * should terminate as soon as possible. */
    bool device_removed;
    /* Counts the open() calls withiout corresponding release() */
    u32 open_counter;
    /* Used by the remove function to wait for release() functions to be called. */
    struct completion all_files_released;
    /* Pointer to BAR0 region mapped to kernel space. */
    u8 __iomem *bar0;
    /* Three files: xorXs1, xorXs2 and xorXd */
    struct device_outlet src1, src2, dest;
    /* Used to ensure critical sections. Because those sections are short and are not
     * likely to be needed by many processes in the same time, spinlocks are an acceptable
     * solution. */
    spinlock_t spinlock;
    /* Processes sleep here when the buffers are full and they have to read or write
     * something. */
    wait_queue_head_t buffers_available;
    /* Where the region of the buffer containing the data currently processed by the device
     * is. Immediately after new data to read in case of source or free space to write in case
     * of destination.
     */
    u32 buffer_start;
    /* Size of the region containing currently processed data. */
    u32 device_size;
};

/* Device class common for all xordevs. */
static struct class *class;

/* Major number of all xordev character devices. */
static int major;

/* Slots for xordevs. */
static struct device_data ddata_base[MAX_DEVICES];

/**
 * Sleeps until the free region of the respective buffer is nonempty.
 *
 * To be used as part of xor_operation(). When successful returns with
 * spinlock held, releases it on failure.
 * @param ddata - device handled
 * @param outlet - outlet with the buffer
 * @param flags - a place to save irq flags
 * @param total_copied - how many bytes did we copy so far in the current xor_operation
 * @return 0 on success, -errno on error
 */
static int wait_for_buffer_space(struct device_data *ddata,
        struct device_outlet *outlet, unsigned long *flags, size_t total_copied) {
    int retval;

    spin_lock_irqsave(&ddata->spinlock, *flags);
    /* If is OK here, because our process is the only one who can increase the sum
     * below. */
    if (ddata->device_size + outlet->size == BUFFER_SIZE) {
        /* If the device is working, tell it that we want an interrupt immediately
         * after it processes at least one byte */
        if (ddata->device_size > 0) {
            iowrite32(ddata->device_size - 1, ddata->bar0 + INTR_COUNT);
        }
        spin_unlock_irqrestore(&ddata->spinlock, *flags);

        devt_dbg(outlet->cdev.dev, "Buffer full, going to sleep.\n");

        /* Sleep until we're waked by the interrupt handler or the remove function.
         * (Or a signal...) */
        retval = wait_event_interruptible(ddata->buffers_available,
                ddata->device_size + outlet->size < BUFFER_SIZE ||
                ddata->device_removed);
        if (retval) {
            return total_copied == 0 ? -ERESTARTSYS : -EINTR;
        }

        devt_dbg(outlet->cdev.dev,
                "Woke up after waiting for buffer to free.\n");

        spin_lock_irqsave(&ddata->spinlock, *flags);
    }

    if (ddata->device_removed) {
        spin_unlock_irqrestore(&ddata->spinlock, *flags);
        return -ENODEV;
    }

    return 0;
}

/**
 * Copies bytes from (to) the free region of the userspace buffer to (from) the DMA buffer.
 *
 * To be used as part of xor_operation(). Assumes that the process holds the spinlock
 * when entering this method and realeases it on return.
 *
 * Because the free region can be wrapped around the end of the physical buffer, a call
 * to this function peforms up to two calls to copy_from(to)_user.
 *
 * @param ddata - device handled
 * @param outlet - outlet with the buffer
 * @param from_user - do we copy from (or to) the userspace
 * @param total_copied - how many bytes did we copy so far in the current xor_operation
 * @param buffer_user - the user buffer
 * @param count - the count parameter passed to read/write call
 * @param peformed_copy_size - a place to write how many bytes have we actually copied
 * @param flags - a place to save irq flags
 * @return 0 on success, -errno on error
 */
static int perform_copy(struct device_data *ddata, struct device_outlet *outlet,
        bool from_user, size_t total_copied, char __user *buffer_user,
        size_t count, size_t *performed_copy_size, unsigned long *flags) {
    int retval;
    size_t first_copy_size, second_copy_size, free_start_wrapped, free_start;

    free_start = ddata->buffer_start + ddata->device_size + outlet->size;
    free_start_wrapped = free_start % BUFFER_SIZE;

    devt_dbg(outlet->cdev.dev, "Performing a copy.\n");

    /* The beginning of the free region in physical buffer is after the device region. */
    if (free_start < BUFFER_SIZE) {
        /* Copy as much as we can to the free region after the end of the outlet region. */
        first_copy_size =
                min(BUFFER_SIZE - (ddata->device_size + outlet->size), count - total_copied);
        /* Copy as much as we can to the free region before the start of the device region. */
        second_copy_size =
                min(ddata->buffer_start, count - (total_copied + first_copy_size));
    }
    /* The beginning of the free region is before the device region. */
    else {
        /* Copy as much as we can to the free region before the start of the device region. */
        first_copy_size = min(ddata->buffer_start - free_start_wrapped, count - total_copied);
        second_copy_size = 0;
    }

    spin_unlock_irqrestore(&ddata->spinlock, *flags);

    if (first_copy_size > 0) {
        devt_dbg(outlet->cdev.dev, "Performing first copy.\n");

        if (from_user) {
            retval = copy_from_user(outlet->buffer_driver + free_start_wrapped,
                    buffer_user + total_copied, first_copy_size);
        } else {
            retval = copy_to_user(buffer_user + total_copied,
                    outlet->buffer_driver + free_start_wrapped, first_copy_size);
        }

        if (retval > 0) {
            *performed_copy_size = first_copy_size - retval;
            return -EFAULT;
        }
    }

    if (second_copy_size > 0) {
        devt_dbg(outlet->cdev.dev, "Performing second copy.\n");

        if (from_user) {
            retval = copy_from_user(outlet->buffer_driver,
                    buffer_user + total_copied + first_copy_size,
                    second_copy_size);
        } else {
            retval = copy_to_user(buffer_user + total_copied + first_copy_size,
                    outlet->buffer_driver, second_copy_size);
        }

        if (retval > 0) {
            *performed_copy_size = first_copy_size + second_copy_size - retval;
            return -EFAULT;
        }
    }

    devt_dbg(outlet->cdev.dev,
            "Performed a copy, first copy size: %u, second_copy_size: %u\n",
            first_copy_size, second_copy_size);

    *performed_copy_size = first_copy_size + second_copy_size;
    return 0;
}

/**
 * Sets up the private_data pointer so that it points to our device_structure
 * then atomically checks if the device_removed flag has been set and returns with
 * error if so, or increases the open_counter otherwise.
 *
 * @param inode - inode structure
 * @param filp - file structure
 * @return 0 on success, -errno on error
 */
static int open(struct inode *inode, struct file *filp) {
    int retval, slot;
    unsigned long flags;
    struct device_data *ddata;

    slot = MINOR(inode->i_rdev) % MAX_DEVICES;
    ddata = ddata_base + slot;

    devt_dbg(ddata->src1.cdev.dev, "Performing an open.\n");

    filp->private_data = ddata;

    spin_lock_irqsave(&ddata->spinlock, flags);
    if (ddata->device_removed) {
        retval = -ENODEV;
    } else {
        retval = 0;
        ++ddata->open_counter;

        devt_dbg(ddata->src1.cdev.dev,
                "Performed an open and increased the counter to %u.\n",
                ddata->open_counter);
    }
    spin_unlock_irqrestore(&ddata->spinlock, flags);

    return retval;
}

/**
 * Decreases the open_counter and wakes the remove fuction if it has dropped to 0 and
 * the device_removed flag has been set.
 * @param inode - inode structure
 * @param filp - file structure
 * @return 0 on success, -errno on error
 */
static int release(struct inode *inode, struct file *filp) {
    unsigned long flags;
    struct device_data *ddata;
    bool call_complete = false;

    ddata = filp->private_data;

    devt_dbg(ddata->src1.cdev.dev, "Performing a release.\n");

    spin_lock_irqsave(&ddata->spinlock, flags);
    if (--ddata->open_counter == 0 && ddata->device_removed) {
        call_complete = true;
    }

    devt_dbg(ddata->src1.cdev.dev,
            "Performed a release and decreased the counter to %u.\n",
            ddata->open_counter);
    spin_unlock_irqrestore(&ddata->spinlock, flags);

    if (call_complete) {
        devt_dbg(ddata->src1.cdev.dev, "Complete is being called.\n");
        complete(&ddata->all_files_released);
    }

    return 0;
}

/**
 * Adjusts the device region, and registers so that they contain the largest possible
 * number of bytes.
 *
 * Assumes that the proccess holds the spinlock.
 *
 * @param ddata - device data structure
 * @return 0 ons success, -errno on error
 */
static int update_device_region_and_registers(struct device_data * ddata) {
    u32 count, already_processed;
    size_t extension;

    if (ddata->device_removed) {
        return -ENODEV;
    }

    devt_dbg(ddata->src1.cdev.dev, "Updating the device and registers.\n");

    count = ioread32(ddata->bar0 + COUNT);
    already_processed = ddata->device_size - count;

    // Shrink the device region from left.
    ddata->device_size = count;
    ddata->buffer_start += already_processed;
    ddata->buffer_start %= BUFFER_SIZE;

    // Extend the device region from the right.
    extension = min(
            min(ddata->src1.size, ddata->src2.size),
            min(ddata->dest.size,
                    BUFFER_SIZE - (ddata->buffer_start + ddata->device_size)));
    ddata->device_size += extension;
    ddata->src1.size -= extension;
    ddata->src2.size -= extension;
    ddata->dest.size -= extension;

    devt_dbg(ddata->src1.cdev.dev, "New device size is %u.\n",
            ddata->device_size);

    /* We don't want interrupts if the device region is empty */
    iowrite32(ddata->device_size == 0 ? 0 : 1, ddata->bar0 + INTR_EN);
    /* PCI posting */
    ioread32(ddata->bar0 + INTR_EN);

    if (count == 0) {
        devt_dbg(ddata->src1.cdev.dev,
                "The device is currently not running.\n");

        iowrite32(ddata->src1.buffer_device + ddata->buffer_start,
                ddata->bar0 + SRC1);
        iowrite32(ddata->src2.buffer_device + ddata->buffer_start,
                ddata->bar0 + SRC2);
        iowrite32(ddata->dest.buffer_device + ddata->buffer_start,
                ddata->bar0 + DST);
    } else {
        devt_dbg(ddata->src1.cdev.dev, "The device is still running.\n");
    }

    iowrite32(0, ddata->bar0 + INTR_COUNT);
    devt_dbg(ddata->src1.cdev.dev, "We're writing %u to COUNT.\n", extension);
    iowrite32(extension, ddata->bar0 + COUNT);

    devt_dbg(ddata->src1.cdev.dev, "Updated the device and registers.\n");

    return 0;
}

/**
 * Depending on arguments it acts as a xors1_write, xors2_write, or xord_read
 *
 * @param ddata - device data structure
 * @param outlet - outlet with the buffer
 * @param from_user - do we copy from (or to) the userspace
 * @param buff - the user space buffer
 * @param count - the count parameter passed to read/write call
 * @param offp - file offset pointer
 * @return size processed on success, -errno on error
 */
static ssize_t xor_operation(struct device_data *ddata,
        struct device_outlet *outlet, bool from_user, char __user *buff,
        size_t count, loff_t *offp) {
    int retval;
    size_t total_copied, performed_copy_size;
    unsigned long flags;

    spin_lock_irqsave(&ddata->spinlock, flags);
    if (ddata->device_removed) {
        return -ENODEV;
    }
    spin_unlock_irqrestore(&ddata->spinlock, flags);

    /* Processes wait in queue for every outlet and only one can read/write to it at a time. */
    retval = mutex_lock_interruptible(&outlet->mutex);
    if (retval) {
        return -ERESTARTSYS;
    }

    total_copied = 0;
    /* Write doesn't return until it writes count bytes. Read doesn't return until it
     * reads at least one byte. */
    while ((total_copied < count && from_user)
            || (total_copied == 0 && !from_user)) {

        /* Sleep until there is a nonempty free region in the buffer.  */
        retval = wait_for_buffer_space(ddata, outlet, &flags, total_copied);
        if (retval) {
            goto ret;
        }

        /* Copy as much as you can. */
        retval = perform_copy(ddata, outlet, from_user, total_copied,
                (void *) buff, count, &performed_copy_size, &flags);

        /* Increase the counter and pointer. */
        total_copied += performed_copy_size;
        *offp += performed_copy_size;

        spin_lock_irqsave(&ddata->spinlock, flags);
        /* Increase outlet region size. */
        outlet->size += performed_copy_size;
        if (retval) {
            spin_unlock_irqrestore(&ddata->spinlock, flags);
            goto ret;
        }
        /* Update the device. */
        retval = update_device_region_and_registers(ddata);
        spin_unlock_irqrestore(&ddata->spinlock, flags);

        if (retval) {
            goto ret;
        }
    }

    retval = total_copied;

    ret: mutex_unlock(&outlet->mutex);
    return retval;
}

static ssize_t xors1_write(struct file *filp, const char __user *buff,
        size_t count, loff_t *offp) {
    struct device_data *ddata = filp->private_data;
    int retval;

    devt_dbg(ddata->src1.cdev.dev, "Writing to xors1.\n");
    retval = xor_operation(ddata, &ddata->src1, true, (void *) buff, count,
            offp);
    devt_dbg(ddata->src1.cdev.dev, "Wrote to xors1.\n");

    return retval;
}

static ssize_t xors2_write(struct file *filp, const char __user *buff,
        size_t count, loff_t *offp) {
    struct device_data *ddata = filp->private_data;
    int retval;

    devt_dbg(ddata->src2.cdev.dev, "Writing to xors2.\n");
    retval = xor_operation(ddata, &ddata->src2, true, (void *) buff, count,
            offp);
    devt_dbg(ddata->src2.cdev.dev, "Wrote to xors2.\n");

    return retval;
}

static ssize_t xord_read(struct file *filp, char __user *buff, size_t count,
        loff_t *offp) {
    struct device_data *ddata = filp->private_data;
    int retval;

    devt_dbg(ddata->dest.cdev.dev, "Reading from xord.\n");
    retval = xor_operation(ddata, &ddata->dest, false, buff, count, offp);
    devt_dbg(ddata->dest.cdev.dev, "Read from xord.\n");

    return retval;
}

/**
 * Updates device and wakes processes waiting for buffers to become available.
 *
 * @param irq - irq number
 * @param dev - the device
 * @return IRQ_NONE when the device didn't request interrupt, IRQ_HANDLED when it did.
 */
static irqreturn_t irq_handler(int irq, void *dev) {
    struct device_data *ddata = dev;
    u32 count, intr_count, intr_en;
    unsigned long flags;

    spin_lock_irqsave(&ddata->spinlock, flags);
    count = ioread32(ddata->bar0 + COUNT);
    intr_count = ioread32(ddata->bar0 + INTR_COUNT);
    intr_en = ioread32(ddata->bar0 + INTR_EN);

    if (!(count <= intr_count && intr_en)) {
        spin_unlock(&ddata->spinlock);
        return IRQ_NONE;
    }

    devt_dbg(ddata->src1.cdev.dev, "Irq handler started.\n");

    update_device_region_and_registers(ddata);

    spin_unlock_irqrestore(&ddata->spinlock, flags);
    wake_up_interruptible(&ddata->buffers_available);

    devt_dbg(ddata->src1.cdev.dev, "Irq handler finished.\n");

    return IRQ_HANDLED;
}

static struct file_operations xors1_operations = { .owner = THIS_MODULE, .open =
        open, .write = xors1_write, .release = release };

static struct file_operations xors2_operations = { .owner = THIS_MODULE, .open =
        open, .write = xors2_write, .release = release };

static struct file_operations xord_operations = { .owner = THIS_MODULE, .open =
        open, .read = xord_read, .release = release };

/**
 * Find a slot for the device and initialize it.
 *
 * Assumes that the kernel does not perform mutithreaded probing.
 *
 * @param dev -pic_dev structure
 * @param id - pci_device_id structure
 * @return 0 on success, -errno on error
 */
static int probe(struct pci_dev *dev, const struct pci_device_id *id) {
    struct device_data *ddata;
    struct device *device;
    unsigned int slot;
    int retval;

    dev_dbg(&dev->dev, "Probing started.\n");

    /* Reserve a slot in array */

    for (slot = 0; slot < MAX_DEVICES && ddata_base[slot].in_use; ++slot)
        ;

    if (slot == MAX_DEVICES) {
        dev_err(&dev->dev, "too many devices found!\n");
        retval = -EIO;
        goto err0;
    }

    ddata = ddata_base + slot;
    ddata->in_use = true;


    pci_set_drvdata(dev, ddata);

    /* Enable device memory mapping to kernel address space */

    retval = pci_enable_device(dev);
    if (retval)
        goto err1;

    /* Reserve device, so only we are able to access its memory regions */

    retval = pci_request_regions(dev, "xordev");
    if (retval)
        goto err2;

    retval = -ENOMEM;

    /* Map BAR0 to driver address space */

    ddata->bar0 = pci_iomap(dev, 0, 0);
    if (ddata->bar0 == NULL) {
        dev_err(&dev->dev, "can't map BAR0 to driver address space\n");
        goto err3;
    }

    /* Enable DMA. */

    pci_set_master(dev);

    /* Tell the DMA subsystem, that xordev supports 32-bit addresses. */

    retval = pci_set_dma_mask(dev, DMA_BIT_MASK(DEVICE_ADDRESS_SIZE));
    if (retval) {
        dev_err(&dev->dev, "can't set DMA mask");
        goto err4;
    }

    retval = pci_set_consistent_dma_mask(dev,
            DMA_BIT_MASK(DEVICE_ADDRESS_SIZE));
    if (retval) {
        dev_err(&dev->dev, "can't set DMA consistent mask");
        goto err4;
    }

    /* Allocate the DMA buffers. */

    ddata->src1.buffer_driver = dma_alloc_coherent(&dev->dev, BUFFER_SIZE * 3,
            &ddata->src1.buffer_device, GFP_KERNEL);
    if (ddata->src1.buffer_driver == NULL) {
        dev_err(&dev->dev, "can't allocate the DMA buffers");
        goto err4;
    }

    ddata->src2.buffer_driver = ddata->src1.buffer_driver + BUFFER_SIZE;
    ddata->src2.buffer_device = ddata->src1.buffer_device + BUFFER_SIZE;
    ddata->dest.buffer_driver = ddata->src1.buffer_driver + 2 * BUFFER_SIZE;
    ddata->dest.buffer_device = ddata->src1.buffer_device + 2 * BUFFER_SIZE;

    dev_dbg(&dev->dev, "src1.buffer_driver: %p\n", (void *) ddata->src1.buffer_driver);
    dev_dbg(&dev->dev, "src2.buffer_driver: %p\n", (void *) ddata->src2.buffer_driver);
    dev_dbg(&dev->dev, "dest.buffer_driver: %p\n", (void *) ddata->dest.buffer_driver);
    dev_dbg(&dev->dev, "src1.buffer_device: %p\n", (void *) ddata->src1.buffer_device);
    dev_dbg(&dev->dev, "src2.buffer_device: %p\n", (void *) ddata->src2.buffer_device);
    dev_dbg(&dev->dev, "dest.buffer_device: %p\n", (void *) ddata->dest.buffer_device);

    /* Initialize misc. fields */

    ddata->device_removed = false;
    ddata->open_counter = 0;
    init_completion(&ddata->all_files_released);
    spin_lock_init(&ddata->spinlock);
    init_waitqueue_head(&ddata->buffers_available);
    ddata->buffer_start = 0;
    ddata->device_size = 0;
    mutex_init(&ddata->src1.mutex);
    mutex_init(&ddata->src2.mutex);
    mutex_init(&ddata->dest.mutex);
    ddata->src1.size = 0;
    ddata->src2.size = 0;
    ddata->dest.size = BUFFER_SIZE;

    /* Disable device interrupts. */

    iowrite32(0, ddata->bar0 + INTR_EN);
    /* PCI posting */
    ioread32(ddata->bar0 + INTR_EN);

    /* Set up irq handler */
    retval = request_irq(dev->irq, irq_handler, IRQF_SHARED, "xordev", ddata);
    if (retval) {
        dev_err(&dev->dev, "can't register interrupt handler\n");
        goto err5;
    }

    cdev_init(&ddata->src1.cdev, &xors1_operations);
    cdev_init(&ddata->src2.cdev, &xors2_operations);
    cdev_init(&ddata->dest.cdev, &xord_operations);

    ddata->src1.cdev.owner = THIS_MODULE;
    ddata->src2.cdev.owner = THIS_MODULE;
    ddata->dest.cdev.owner = THIS_MODULE;

    retval = cdev_add(&ddata->src1.cdev, MKDEV(major, slot), 1);
    if (retval) {
        dev_err(&dev->dev, "can't add xors1 cdev");
        goto err6;
    }
    retval = cdev_add(&ddata->src2.cdev, MKDEV(major, MAX_DEVICES + slot), 1);
    if (retval) {
        dev_err(&dev->dev, "can't add xors2 cdev");
        goto err7;
    }
    retval = cdev_add(&ddata->dest.cdev, MKDEV(major, 2 * MAX_DEVICES + slot),
            1);
    if (retval) {
        dev_err(&dev->dev, "can't add xord cdev");
        goto err8;
    }

    device = device_create(class, &dev->dev, MKDEV(major, slot), NULL,
            "xor%us1", slot);
    if (IS_ERR(device)) {
        dev_err(&dev->dev, "can't create xors1 device");
        retval = PTR_ERR(device);
        goto err9;
    }
    device = device_create(class, &dev->dev, MKDEV(major, MAX_DEVICES + slot),
            NULL, "xor%us2", slot);
    if (IS_ERR(device)) {
        dev_err(&dev->dev, "can't create xors2 device");
        retval = PTR_ERR(device);
        goto err10;
    }
    device = device_create(class, &dev->dev,
            MKDEV(major, 2 * MAX_DEVICES + slot), NULL, "xor%ud", slot);
    if (IS_ERR(device)) {
        dev_err(&dev->dev, "can't create xord device");
        retval = PTR_ERR(device);
        goto err11;
    }

    dev_dbg(&dev->dev, "Probing finished.\n");

    return 0;

    err11: device_destroy(class, MKDEV(major, MAX_DEVICES + slot));
    err10: device_destroy(class, MKDEV(major, slot));
    err9: cdev_del(&ddata->src1.cdev);
    err8: cdev_del(&ddata->src2.cdev);
    err7: cdev_del(&ddata->dest.cdev);
    err6: free_irq(dev->irq, ddata);
    err5: dma_free_coherent(&dev->dev, BUFFER_SIZE * 3,
            ddata->src1.buffer_driver, ddata->src1.buffer_device);
    err4: pci_iounmap(dev, ddata->bar0);
    err3: pci_release_regions(dev);
    err2: pci_disable_device(dev);
    err1: ddata->in_use = false;
    err0: return retval;
}

/**
 * Clean up device data and wait for processes using the device files to complete.
 *
 * @param dev - pci_dev structure
 */

static void remove(struct pci_dev *dev) {
    struct device_data *ddata = pci_get_drvdata(dev);
    unsigned long slot = ddata_base - ddata, flags;

    dev_dbg(&dev->dev, "Removing started.\n");

    spin_lock_irqsave(&ddata->spinlock, flags);
    ddata->device_removed = true;
    iowrite32(0, ddata->bar0 + INTR_EN);
    /* PCI posting */
    ioread32(ddata->bar0 + INTR_EN);
    spin_unlock_irqrestore(&ddata->spinlock, flags);

    /* If open counter equals 0, it will never grow now that we've set up the device_removed
     * flag, so it's safe to check with a simple if. */
    if (ddata->open_counter > 0) {
        dev_dbg(&dev->dev,
                "Remove: waking up everyone and waiting for all files to close.\n");
        wake_up_interruptible(&ddata->buffers_available);
        wait_for_completion(&ddata->all_files_released);
    }

    device_destroy(class, MKDEV(major, 2 * MAX_DEVICES + slot));
    device_destroy(class, MKDEV(major, MAX_DEVICES + slot));
    device_destroy(class, MKDEV(major, slot));
    cdev_del(&ddata->src1.cdev);
    cdev_del(&ddata->src2.cdev);
    cdev_del(&ddata->dest.cdev);
    free_irq(dev->irq, ddata);
    dma_free_coherent(&dev->dev, BUFFER_SIZE * 3, ddata->src1.buffer_driver,
            ddata->src1.buffer_device);
    pci_iounmap(dev, ddata->bar0);
    pci_release_regions(dev);
    pci_disable_device(dev);
    ddata->in_use = false;

    dev_dbg(&dev->dev, "Removing finished.\n");
}

static struct pci_device_id pci_device_id[] = { { .vendor = VENDOR, .device =
        DEVICE, .subvendor = PCI_ANY_ID, .subdevice = PCI_ANY_ID, .class = 0,
        .class_mask = 0 }, { 0 } };

static struct pci_driver pci_driver = { .name = "xordev", .id_table =
        pci_device_id, .probe = probe, .remove = remove };

static int init(void) {
    int retval;
    dev_t dev;

    pr_info("xordev: driver initialization begun\n");

    class = class_create(THIS_MODULE, "xordev");
    if (IS_ERR(class)) {
        retval = PTR_ERR(class);
        pr_err("xordev: can't register class\n");
        goto err0;
    }

    retval = alloc_chrdev_region(&dev, 0, 3 * MAX_DEVICES, "xordev");
    if (retval) {
        pr_err("xordev: can't register character device\n");
        goto err1;
    }
    major = MAJOR(dev);

    retval = pci_register_driver(&pci_driver);
    if (retval) {
        pr_err("xordev: can't register pci driver\n");
        goto err2;
    }

    pr_info("xordev: driver was successfully initialized\n");

    return 0;
    err2: unregister_chrdev_region(dev, 3 * MAX_DEVICES);
    err1: class_destroy(class);
    err0: return retval;
}

static void exit(void) {
    pr_info("xordev: module removing begun\n");
    pci_unregister_driver(&pci_driver);
    unregister_chrdev_region(MKDEV(major, 0), 3 * MAX_DEVICES);
    class_destroy(class);
    pr_info("xordev: module successfully removed\n");
}

module_init(init);
module_exit(exit);
