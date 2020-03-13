/*
 * 		DHT11UNIMI DRIVER
 */

#include <linux/errno.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/cdev.h>
#include <linux/io.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/poll.h>


#include "gpio_lib.h"
#include "dht11unimi.h"

/*
 * 	device driver data structures to keep data about registration on the platfrom
 * 	and realtime data from the sensor
 */
static dev_t chrdev_devt;
static struct class *chrdev_class;

struct chrdev_device chrdev_dht11;

void dht11_work_sampling(struct work_struct * work);
struct delayed_work my_work;

/*
 *	Scheduled thread work
 */
void dht11_work_sampling(struct work_struct *work) {

	bool ret;

	if (chrdev_dht11.work_status == DHT11_WORKS_SHUTDOWN) {
		chrdev_dht11.work_status = DHT11_WORK_STOPPED;
		return;
	}

	if (!chrdev_dht11.sensor_busy) {
		mutex_lock(&chrdev_dht11.mux);
		read_dht11(chrdev_dht11.gpio_pin);
		mutex_unlock(&chrdev_dht11.mux);
	} else
		printk(KERN_WARNING "MCU <-> dht11 sensor busy, waiting\n");

	ret = schedule_delayed_work(&my_work, msecs_to_jiffies(chrdev_dht11.w_dht11_work_time));
	if (!ret)
		printk(KERN_ERR "scheduling driver work failed. Data sampling is stopped\n");

	return;
}

/*
 * Methods
 */
static ssize_t chrdev_read(struct file *filp, char __user *buf, size_t count, loff_t *ppos) {

	int ret;

	mutex_lock(&chrdev_dht11.mux);

	/*
	 *		i'm a bit out of time, for now i want user space equale or bigger
	 *		than driver buffer
	 */
	if (count < chrdev_dht11.wbytes) {
		mutex_unlock(&chrdev_dht11.mux);
		return -EFBIG;
	}

	/*
	 *		Sensor data are sampled by a scheduled thread working
	 */
	if (chrdev_dht11.sensor_busy) {
		mutex_unlock(&chrdev_dht11.mux);
		return -EBUSY;
	}

	/* Return data to the user space */
	ret = copy_to_user(buf, chrdev_dht11.buf, chrdev_dht11.wbytes);

	if (ret < 0) {
		mutex_unlock(&chrdev_dht11.mux);
		return -EFAULT;
	}

	mutex_unlock(&chrdev_dht11.mux);
	count = chrdev_dht11.wbytes;
	*ppos += count;

	return count;
}

static ssize_t chrdev_write(struct file *filp, const char __user *buf, size_t count, loff_t *ppos) {
	struct chrdev_device *chrdev = filp->private_data;
	int ret;

	dev_info(chrdev->dev, "should write %ld bytes (*ppos=%lld)\n",	count, *ppos);

	if (chrdev->read_only)
		return -EINVAL;

	/* Check for end-of-buffer */
	if (*ppos + count >= BUF_LEN)
		count = BUF_LEN - *ppos;

	/* Get data from the user space */
	ret = copy_from_user(chrdev->buf + *ppos, buf, count);
	if (ret < 0)
		return -EFAULT;

	*ppos += count;
	dev_info(chrdev->dev, "got %ld bytes (*ppos=%lld)\n", count, *ppos);

	return count;
}

static int chrdev_open(struct inode *inode, struct file *filp) {

	struct chrdev_device *chrdev = container_of(inode->i_cdev, struct chrdev_device, cdev);
	filp->private_data = chrdev;
	kobject_get(&chrdev->dev->kobj);

	dev_info(chrdev->dev, "dht11chrdev (id=%d) opened\n", chrdev->id);

	return 0;
}

static int chrdev_release(struct inode *inode, struct file *filp) {
	struct chrdev_device *chrdev = container_of(inode->i_cdev, struct chrdev_device, cdev);
	kobject_put(&chrdev->dev->kobj);
	filp->private_data = NULL;
	dev_info(chrdev->dev, "dht11chrdev (id=%d) released\n", chrdev->id);
	return 0;
}

static __poll_t dht11_poll(struct file *filp, poll_table *wait) {

	struct chrdev_device *chrdev = filp->private_data;
	__poll_t mask = 0;
	poll_wait(filp, &chrdev->queue, wait);

	mutex_lock(&chrdev->mux);
	if (chrdev->wbytes > 0)
		mask |= EPOLLIN | EPOLLRDNORM;
	mutex_unlock(&chrdev->mux);

	return mask;
}

static const struct file_operations chrdev_fops = {
		.owner = THIS_MODULE,
		.read = chrdev_read,
		.write = chrdev_write,
		.open = chrdev_open,
		.release = chrdev_release,
		.poll = dht11_poll
};

/*
 * Exported functions
 */

int chrdev_device_register(const char *label, unsigned int id, unsigned int read_only, struct module *owner, struct device *parent) {

	struct chrdev_device *chrdev;
	dev_t devt;
	int ret;

	printk(KERN_INFO "dht11unimi registering device driver id %d label %s\n", id, label);

	chrdev = &chrdev_dht11;

	/*
	 *		Already used?
	 */
	if (chrdev->busy) {
		pr_err("id %d\n is busy", id);
		return -EBUSY;
	}

	/*
     *		CREATE CDEV
	 */
	cdev_init(&chrdev->cdev, &chrdev_fops);
	chrdev->cdev.owner = owner;
	devt = MKDEV(MAJOR(chrdev_devt), id);
	ret = cdev_add(&chrdev->cdev, devt, 1);
	if (ret) {
		pr_err("failed to add char device %s at %d:%d\n", label, MAJOR(chrdev_devt), id);
		return ret;
	}

	/*
	 *		GPIO PIN
	 */
	pr_info("Retrieving gpio pin\n");
	ret = of_get_gpio(parent->of_node, 0);
	if (ret < 0) {
		dev_err(parent, "Failed to retrieve gpio pin id");
		return ret;
	}
	pr_info("DHT11 pin data bus %d", ret);
	chrdev->gpio_pin = ret;
	chrdev->sensor_busy = 0;
	gpiolib_init();						// map physical memory
	gpiolib_set_pull_resistor(chrdev->gpio_pin, 2);	// pull-up resistor

	/*
	 *		CREATE DEVICE
	 */
	chrdev->dev = device_create(chrdev_class, parent, devt, chrdev, "%s@%d", label, id);
	if (IS_ERR(chrdev->dev)) {
		pr_err("unable to create device %s\n", label);
		ret = PTR_ERR(chrdev->dev);
		goto del_cdev;
	}
	dev_set_drvdata(chrdev->dev, chrdev);

	/*
	 *		INIT DRIVER DATA
	 */
	chrdev->id = id;
	chrdev->read_only = read_only;
	chrdev->busy = 1;
	chrdev->w_dht11_work_time = 3000;	/* sensors read duty cycle */
	strncpy(chrdev->label, label, NAME_LEN);
	memset(chrdev->buf, 0, BUF_LEN);
	mutex_init(&chrdev->mux);
	init_waitqueue_head(&chrdev->queue);

	/* install scheduled work */
	INIT_DELAYED_WORK(&my_work, dht11_work_sampling);
	schedule_delayed_work(&my_work, msecs_to_jiffies(chrdev->w_dht11_work_time));

	dev_info(chrdev->dev, "dht11chrdev %s with id %d added\n", label, id);
	return 0;

del_cdev:
	cdev_del(&chrdev->cdev);
	return ret;
}
EXPORT_SYMBOL(chrdev_device_register);


int chrdev_device_unregister(const char *label, unsigned int id) {

	int ret;
	struct chrdev_device *chrdev;

	chrdev = &chrdev_dht11;

	/* ... then check if device is actualy allocated */
	if (!chrdev->busy || strcmp(chrdev->label, label)) {
		pr_err("id %d is not busy or label %s is not known\n", id, label);
		return -EINVAL;
	}

	/* Deinit the chrdev data */
	do {
		chrdev_dht11.work_status = DHT11_WORKS_SHUTDOWN;
		printk(KERN_INFO "waiting works completition\n");
		msleep(1000);
	} while (chrdev_dht11.work_status != DHT11_WORK_STOPPED);
	printk(KERN_INFO "works done. Removing the driver\n");
	chrdev->id = 0;
	chrdev->busy = 0;
	chrdev->sensor_busy = 0;
	dev_info(chrdev->dev, "dht11chrdev %s with id %d removed\n", label, id);

	/* Dealocate the device */
	device_destroy(chrdev_class, chrdev->dev->devt);

	cdev_del(&chrdev->cdev);

	gpiolib_cleanup_module();

	return 0;
}
EXPORT_SYMBOL(chrdev_device_unregister);

/*
 * Module stuff
 */

static int __init chrdev_init(void) {

	int ret;

	/* Create the new class for the chrdev devices */
	chrdev_class = class_create(THIS_MODULE, "dht11chrdev");
	if (!chrdev_class) {
		pr_err("dht11chrdev: failed to allocate class\n");
		return -ENOMEM;
	}

	/* Allocate a region for character devices */
	ret = alloc_chrdev_region(&chrdev_devt, 0, MAX_DEVICES, "dht11chrdev");
	if (ret < 0) {
		pr_err("failed to allocate char device region\n");
		goto remove_class;
	}

	pr_info("got major %d\n", MAJOR(chrdev_devt));

	return 0;

remove_class:
	class_destroy(chrdev_class);

	return ret;
}

static void __exit chrdev_exit(void) {
	unregister_chrdev_region(chrdev_devt, MAX_DEVICES);
	class_destroy(chrdev_class);
}

module_init(chrdev_init);
module_exit(chrdev_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Nunzio Castelli");
MODULE_DESCRIPTION("dht11 device driver - SSRI Università degli Studi di Milano");
