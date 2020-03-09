/*
 * 		DHT11UNIMI DRIVER REGISTER
 */

#include <linux/errno.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/cdev.h>

#include <linux/kernel.h>
#include <linux/module.h>

#include "dht11unimi.h"


/*
 * 	Kernel platform driver
 */
static int chrdev_req_probe(struct platform_device *pdev) {

	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	struct module *owner = THIS_MODULE;
	int count, ret;

	pr_info("starting probing device %s\n", node->full_name);

	/*
	 * If we are not registering a fixed chrdev device then get
	 * the number of chrdev devices from DTS
	 */
	count = device_get_child_node_count(dev);
	pr_info("child node nr. %d\n", count);
	if (count > 0) {
		dev_err(dev, "Multiple device driver config into device tree not supported yet\n");
		return -ENODEV;
	}

	/*
	 * 	Register the new chr device
	 */
	ret = chrdev_device_register("dht11unimi", 0, true, owner, dev);
	if (ret) {
		dev_err(dev, "unable to register device %s Err %d", dev->init_name, ret);
		return ret;
	}

	pr_info("probing completed\n");

	return 0;
}

static int chrdev_req_remove(struct platform_device *pdev) {

	struct device *dev = &pdev->dev;
	int ret;

	/*
	 * 	Remove driver
	 * 	TODO: remove fixed reference
	 */
	ret = chrdev_device_unregister("dht11unimi", 0);
	if (ret)
		dev_err(dev, "unable to unregister");

	return 0;
}

static const struct of_device_id of_chrdev_req_match[] = {
		{ .compatible =	DHT11UNIMI_COMPATIBLE, },
		{ /* sentinel */}
};
MODULE_DEVICE_TABLE(of, of_chrdev_req_match);

static struct platform_driver chrdev_req_driver = {
		.probe = chrdev_req_probe,
		.remove = chrdev_req_remove,
		.driver = { .name = "dht11unimi-req",
					.of_match_table = of_chrdev_req_match,
				  },
};
module_platform_driver(chrdev_req_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Nunzio Castelli");
MODULE_DESCRIPTION("dht11 device driver loader - SSRI Università degli Studi di Milano");
