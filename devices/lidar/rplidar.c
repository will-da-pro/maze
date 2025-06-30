#include <linux/module.h>
#include <linux/init.h>
#include <linux/usb.h>

#define RPLIDAR_VENDOR_ID 0x10c4
#define RPLIDAR_PRODUCT_ID 0xea60

MODULE_LICENSE("GPL");
MODULE_AUTHOR("William D'Olier");
MODULE_DESCRIPTION("Driver for Slamtec RPLidar sensors");

static int rplidar_usb_probe(struct usb_interface *interface,
		const struct usb_device_id *id) {
	printk("RPLidar Device Detected!\n");
	return 0;
}

static struct usb_device_id rplidar_usb_id_table [] = {
	{ USB_DEVICE(RPLIDAR_VENDOR_ID, RPLIDAR_PRODUCT_ID) },
	{ }
};

static struct usb_driver rplidar_usb_driver = {
	.name        = "rplidar",
	.probe       = rplidar_usb_probe,
	.id_table    = rplidar_usb_id_table,
	.supports_autosuspend = 1,
};

static int __init rplidar_init(void) {
	printk("RPLidar Driver Initializing...\n");

	int result;

	result = usb_register(&rplidar_usb_driver);

	if (result < 0) {
		pr_err("usb_register failed for the %s driver. Error number %d\n",
                       rplidar_usb_driver.name, result);
                return -1;
	}

	return 0;
}

static void __exit rplidar_exit(void) {
	printk("RPLidar Driver Exiting\n");
	usb_deregister(&rplidar_usb_driver);
}

module_init(rplidar_init);
module_exit(rplidar_exit);
