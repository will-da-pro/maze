#include <linux/module.h>
#include <linux/init.h>
#include <linux/usb.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR("William D'Olier");
MODULE_DESCRIPTION("Driver for Slamtec RPLidar sensors");

static int __init rplidar_init(void) {
	printk("RPLidar Driver Initializing...\n");
	return 0;
}

static void __exit rplidar_exit(void) {
	printk("RPLidar Driver Exiting\n");
}

module_init(rplidar_init);
module_exit(rplidar_exit);
