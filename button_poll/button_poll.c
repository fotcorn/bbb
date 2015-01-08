/**
 *-----------------------------------------------------------------------------
 * \brief BBB Driver Exercise
 *
 *   Button Driver for the BBB
 *
 *   This driver allows you to read the state of the buttons S1 through S4 
 *   on the breadboard cape.
 *   Four GPIO pins are assigned to the the four buttons:
 *       button    pin                  GPIO# 
 *    	 S1 (BTN1) P9-23 (GPIO1_17) --> 49  (1*32+17)
 *    	 S2 (BTN2) P9-30 (GPIO3_16) --> 112 (3*32+16)
 *    	 S3 (BTN3) P9-16 (GPIO1_19) --> 51  (1*32+19)
 *    	 S4 (BTN4) P9-42a(GPIO0_7)  --> 7   (0*32+7)
 *
 *       Newer drivers should make use of the GPIO API functions like
 *         gpio_request_one() or gpio_request_array()
 *         gpio_free() or gpio_free_array()
 *         gpio_set_direction()
 *         gpio_get_value(gpio), gpio_set_value()
 *
 *   The driver supports 4 minor devices: minor number 0 through 3.
 *   Minor number 0 represents BTN1 or S1.
 *   Minor number 1 represents BTN2 or S2.
 *   Minor number 2 represents BTN3 or S3.
 *   Minor number 3 represents BTN4 or S4.
 *
 * Driver Test
 *   -- read method: use the shell script button_poll.sh
 *                   it reads the device and displays the result, which is
 *                   either "on" or "off" in the console.
 *
 *   -- ioctl methos: use the program button-ioctl.c
 *      
 *
 *  Sept/2013 myf1 : Driver is based on CARME button driver.
 *
 *  File: button-drvr.c
 *---------------------------------------------------------------------------*/
#include <linux/module.h>
#include <linux/version.h>
#include <linux/fs.h>
#include <linux/ioctl.h>	/* ioctl command macros */
#include <linux/cdev.h>
#include <linux/gpio.h>
#include <linux/device.h>

#include <asm/io.h>		/* MSC1 address, io_p2v() */
#include <asm/uaccess.h>
#include <asm/delay.h>     	/* udelay() */

#include <asm/gpio.h>		/* GPIO configuration */
#include <asm-generic/gpio.h>	/* GPIO configuration */

//#include "../include/button.h"	/* contains info shared with applications */
/*
 * debug control
 */
//#define BTN_OPEN_DEBUG
#undef BTN_OPEN_DEBUG
//#define BTN_CLOSE_DEBUG
//#undef BTN_CLOSE_DEBUG
//#define BTN_READ_DEBUG
#undef BTN_READ_DEBUG
//#define BTN_IOCTL_DEBUG
#undef BTN_IOCTL_DEBUG

/*
 * driver characteristic values
 */
#define BTN_MAJOR	( 62 )	/* major number of /dev/btn*/
#define BTN_DRVR_NAME	"BtnPoll"     /* driver name in /proc fs */
#define BTN_DEV_NAME	"gpioBtn"        /* device name in sysfs */
#define BTN_DRVR_VER	"1.0"   /* driver version number */
#define BTN_MAX_DEV	( 4 )	/*max # of devices (minor # values)*/
/*
 * GPIO assignment to buttons
 */
#define GPIO_BTN_1	(49)
#define GPIO_BTN_2 	(112)
#define GPIO_BTN_3	(51)
#define GPIO_BTN_4 	(7)

/* 
 * file operation declarations
 */
static int      btn_open(struct inode *, struct file *);
static int	btn_close(struct inode *, struct file *);
static ssize_t	btn_read (struct file *, char *, size_t, loff_t*);

/*
 * variables visible in this file
 *
 * file operations implemented by this driver
 */
static struct file_operations btn_ops =
{
	.owner 		= THIS_MODULE,
	.open  		= btn_open,
	.release	= btn_close,
	.read 		= btn_read,
};

/*
 * button device records and button configuration records
 */
static struct gpio	/*from <asm-generic/gpio.h> */
                /* gpio(u)     flags(ul) label(p)*/
    btn_gpio[] = {{GPIO_BTN_1, GPIOF_IN, BTN_DEV_NAME"1" },
		  {GPIO_BTN_2, GPIOF_IN, BTN_DEV_NAME"2" },
		  {GPIO_BTN_3, GPIOF_IN, BTN_DEV_NAME"3" },
		  {GPIO_BTN_4, GPIOF_IN, BTN_DEV_NAME"4" }};
static struct device *btn_dev[BTN_MAX_DEV];

static const char *class_name=BTN_DEV_NAME;    /* gpio class name*/
static struct class *class; 	    /* sub-class in /sys/class*/
static dev_t dev0 = MKDEV(BTN_MAJOR,0);
static struct cdev cdev;    	/* internal device representation*/

/**
 *----------------------------------------------------------------------------
 * init -- register button driver
 *	   (1) register device region
 *	   (2) configure GPIOs
 *         (3) register device using cdev
 *
 * \return zero if registered successfully
 * \return error otherwise
 *--------------------------------------------------------------------------*/
static int __init btn_driver_init(void)
{
	int i_dev, rc = 0;

	printk( KERN_INFO BTN_DRVR_NAME " V%s (compiled: %s %s)\n",
	            BTN_DRVR_VER, __DATE__, __TIME__);
	/*
	 * register device region starting a dev0, BTN_MAXDEV min numbers
	 */
	rc = register_chrdev_region(dev0, BTN_MAX_DEV, BTN_DRVR_NAME);
	if (rc != 0) {
		printk(KERN_WARNING BTN_DRVR_NAME ": dev 0x%x not available\n", dev0);
		return rc;
	}
	printk(KERN_WARNING BTN_DRVR_NAME ": got %d devs from 0x%x on\n", 
               BTN_MAX_DEV, dev0);
	/*
	 * initialize buttons
	 * use GPIO configuration kernel API function
	 * gpio_request_array() to configure all buttons in a single call.  
	 */
	rc = gpio_request_array(btn_gpio, BTN_MAX_DEV);
	if (rc) goto freeGPIOs;  
	/*
	 * registering device, new style using cdev, use two cdev objects:
	 * the first one to register the range of single button deives associated
	 * with minor devices 0..3, and the second one to register the all button
	 * device associated with minor number 4. 
	 */
	kobject_set_name(&cdev.kobj, BTN_DEV_NAME);
	cdev.owner = THIS_MODULE;
	cdev_init(&cdev, &btn_ops);
	rc = cdev_add(&cdev, dev0, BTN_MAX_DEV);
	if (rc != 0) {
		printk(KERN_WARNING BTN_DEV_NAME ": cdev_add(cdev) failed\n");
		goto freeDevRegion;
	}
	/*
	 * create a class and a device object
	 */
	class = class_create(THIS_MODULE, class_name);
	if (IS_ERR(class)) {
		printk(KERN_WARNING BTN_DRVR_NAME ": creating class failed\n");
		goto unregDevice;
	};
	printk(KERN_INFO BTN_DRVR_NAME " class created\n");
	for (i_dev = 0; i_dev < BTN_MAX_DEV; i_dev++) {
		struct gpio *pdev = btn_gpio + i_dev;
		printk(KERN_WARNING BTN_DRVR_NAME ": creating device\n");
		printk(KERN_WARNING BTN_DRVR_NAME ":  dev_t=0x%x label=%s\n", dev0+i_dev, pdev->label);
		btn_dev[i_dev] = device_create(class, NULL, dev0+i_dev, NULL, "%s", pdev->label);
		if (IS_ERR(btn_dev[i_dev])) {
			printk(KERN_WARNING BTN_DRVR_NAME ": creating dev failed\n");
			goto destroyDevs;
		}
	}

	printk(KERN_INFO BTN_DRVR_NAME " loaded. Major#:%d\n", BTN_MAJOR);
	return(0);
	/*
	 * error exits
	 */
	destroyDevs:
		while (i_dev > 0) {
			device_destroy(class, dev0+i_dev);
			i_dev--;
		}
	unregDevice:
		kobject_put(&cdev.kobj);
		cdev_del(&cdev);
	freeDevRegion:
		kobject_put(&cdev.kobj);
		unregister_chrdev_region(dev0, BTN_MAX_DEV);
	freeGPIOs:
		gpio_free_array(btn_gpio, BTN_MAX_DEV);
	return rc;
}

/*-----------------------------------------------------------------------------
 * exit -- release resources, unregister , free GPIOs
 * return -
 *---------------------------------------------------------------------------*/
static void __exit btn_driver_exit(void)
{
	int i;
	/*
	 * release resources in reverse order
	 */
	for (i = 0; i < BTN_MAX_DEV; i++) {
			device_destroy(class, dev0+i);
	}
	device_destroy(class, dev0);
	class_destroy(class);
	kobject_put(&cdev.kobj);
	cdev_del(&cdev);
	unregister_chrdev_region(cdev.dev, BTN_MAX_DEV);
	gpio_free_array(btn_gpio, BTN_MAX_DEV);
	printk( KERN_INFO BTN_DRVR_NAME " unloaded\n");
	return;
}

/*----------------------------------------------------------------------------
 * open device - Link the file object with the device record
 * return value:0 success
 *              -ENODEV wrong minor number
 *--------------------------------------------------------------------------*/
static int btn_open(struct inode *inode, struct file *file)
{
	int minor = MINOR(inode->i_rdev);
	file->private_data = &btn_gpio[minor];
	#ifdef BTN_OPEN_DEBUG
	printk( KERN_INFO BTN_DRVR_NAME " opened. Minor #:%d\n", minor);
	#endif
	return 0;
}

/*----------------------------------------------------------------------------
 * close device - unlink the file object
 * return - always success (0)
 *--------------------------------------------------------------------------*/
static int btn_close(struct inode *inode, struct file *file)
{
	#ifdef BTN_CLOSE_DEBUG
	printk( KERN_INFO BTN_DRVR_NAME " closed.\n");
	#endif
	file->private_data = NULL;
	return 0;
}

/*----------------------------------------------------------------------------
 * btn_read -- get the button state.
 *             Return "on" if the button is pressed and "off" otherwise.
 *             Write the button state to a user space buffer
 * return value: number of character transferred
 *              -EFAULT: error while transferring to user space
 *--------------------------------------------------------------------------*/
static ssize_t btn_read(struct file *file, 
                        char *buf, size_t size, loff_t *ppos )
{
	char *onOff;
	int len, val;
	unsigned long notCopied;
	struct gpio *pgpio;
	#ifdef BTN_READ_DEBUG
	printk(KERN_WARNING BTN_DRVR_NAME 
		": read(buf%p,size:%u,ppos:%p,pos:%llu)\n",
		buf, size, ppos, ppos ? *ppos: 0);
	#endif
	if (*ppos) {
		*ppos = 0; 
		return 0;
	}
	if (file->private_data == NULL)
		return -EFAULT;
	pgpio = (struct gpio *)file->private_data;
	/*
	 * get the button state of the button specified by the minor number
	 * or get the button state of all buttons
	 */
	val = gpio_get_value(pgpio->gpio);
	#ifdef BTN_READ_DEBUG
	printk(KERN_WARNING BTN_DRVR_NAME ": gpio=%d read(0x%08x)\n",
		pgpio->gpio, val);
	#endif

	/*
	 * myf1: bfh cape is low active, thus test inverted
	 */
//	if (val)
	if (!val)
		onOff = "on\n";
	else
		onOff = "off\n";
	len = strlen(onOff);
	notCopied = copy_to_user(buf, onOff, len);
	if (notCopied) 
		return(-1);
	*ppos = len;
	return len;
}

MODULE_LICENSE("GPL");
module_init(btn_driver_init);
module_exit(btn_driver_exit);

