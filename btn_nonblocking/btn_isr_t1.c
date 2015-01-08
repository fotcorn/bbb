/*-------------------------------------------------------------------
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the
 *   Free Software Foundation, Inc.,
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 *---------------------------------------------------------------------
 *
 * Simple GPIO Interrupt Module using button T1 on the BFH cape
 *
 * In the module init function you request two GPIOs for L1 (output,
 * init low) and T1 (input).
 *
 * To determine the IRQ that corresponds to a given GPIO number use the 
 * macro gpio_to_irq() as irq = gpio_to_irq(gpio).
 *
 * Request an irq for T12.
 *
 * When the interrupt for T1 fires toggle led L1 in the interrupt
 * handler as confirmation. 
 *
 * This module installs only an interrupt handler, no slow handling
 * is performed.
 *
 * source file: btn_isr_t1.c
 * 
 * load this module
 *   -- #modprobe btn_isr_t1
 * 
 *   -- set irq type           #modprobe btn_isr_t1 irqtype=[f|F|b|B]
 *                             default is rising; fF: falling; bB: both
 * unload this module
 *	#modprobe -r btn_isr_t1
 *
 * When the module is unloaded, the ISR counter value are displayed.
 *
 * perhaps run "#/sbin/depmod" before you load the module
 *
 *
 *  V1: Franz Meyer, June 2014 - original
 *-----------------------------------------------------------------*/
#include <linux/module.h>
#include <linux/version.h>
#include <linux/interrupt.h>	/* request_/release_irq() */
#include <linux/fs.h>		/* file operations */
#include <linux/gpio.h>		/* GPIOF_IN, etc*/

#include <asm/uaccess.h>
#include <asm/delay.h>		/* udelay() */




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

/*
 * debug macro
 */
#define DEBUG

#ifdef DEBUG
#define DPRINT(fmt,args...) printk(KERN_INFO "%s,%i:" fmt "\n", \
                            __FUNCTION__, __LINE__,##args);
#else
#define DPRINT(fmt,args...)
#endif
/*
 * debug control
 *  ISR and tasklet are running in interrupt context. Thus be careful with
 *  printk() because it may block. No problem with kernel threads.
 */
//#define BTN_T1_HANDLER_DEBUG
#undef BTN_T1_HANDLER_DEBUG

/*
 * module def macros and constants
 */
#define GPIO_T1		( 49 )		/* Pin P9-23, GPIO1_17, 1*32+17=49*/
#define GPIO_L1		( 61 )		/* Pin P8-26, GPIO1_29, 1*32+29=61*/
#define BTN_T1_MOD_NAME	"BtnIsrT1"      /* module name in /proc fs */
#define BTN_T1_MOD_VER	"1.2"           /* module version number */

/*
 * module parameters
 */
static char *irqtype = "r"; /*default is rising edge*/
module_param(irqtype, charp, S_IRUGO);










#define BTN_MAJOR	( 62 )	/* major number of /dev/btn*/
#define BTN_DRVR_NAME	"BtnPoll"     /* driver name in /proc fs */
#define BTN_DEV_NAME	"gpioBtn"        /* device name in sysfs */
#define BTN_DRVR_VER	"1.0"   /* driver version number */
#define BTN_MAX_DEV	( 1 )	/*max # of devices (minor # values)*/


static int btn_open(struct inode *, struct file *);
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

//#static struct gpio	/*from <asm-generic/gpio.h> */
//                /* gpio(u)     flags(ul) label(p)*/
//    btn_gpio[] = {{GPIO_BTN_1, GPIOF_IN, BTN_DEV_NAME"1" }};
static struct device *btn_dev[BTN_MAX_DEV];

static const char *class_name=BTN_DEV_NAME;    /* gpio class name*/
static struct class *class; 	    /* sub-class in /sys/class*/
static dev_t dev0 = MKDEV(BTN_MAJOR,0);
static struct cdev cdev;    	/* internal device representation*/





















/*
 * global variables for interrupt fast handling 
 */

//const char *btnT1devId  ="btnT1id";     /* device id for GPIO T1 irq */
//const char *btnT1devName="btnT1dev";    /* device name for GPIO_T1 irq */
//static int s_btn_T1_irq = 0;     /* IRQ for GPIO T1 ISR */
//static int s_btn_T1_int_ct = 0;  /* number of processed GPIO T1 ISR */


struct gpio_dev {
    const char *dev_name;
    const char *dev_id;
    const char *led_name;
    int button_pin;
    int led_pin;
    int button_irq;
    int button_count;
};

struct gpio_dev gpio_buttons[] = {
    {"btnT1dev", "btnT1dev", "GPIO_L1", 49, 61, 0, 0,},
    {"btnT2dev", "btnT1dev", "GPIO_L2", 112, 44, 0, 0,},
    {"btnT3dev", "btnT1dev", "GPIO_L3", 51, 68, 0, 0,},
    {"btnT4dev", "btnT1dev", "GPIO_L4", 7, 67, 0, 0,},
};


/*--------------------------------------------------------------------
 * Button T1 interrupt handler - count the interrupt, toggle LED1
 *
 *  Note: an interrupt handler MUST NOT block, thus the DPRINT
 *        statement may not work.
 *-------------------------------------------------------------------*/

static char state = 0;

#define BIT_SET(a,b) ((a) |= (1<<(b)))
#define BIT_CLEAR(a,b) ((a) &= ~(1<<(b)))

static irqreturn_t btn_T1_handler(int irq, void *dev_id)
{
    int i = -1;
    struct gpio_dev* gpio_button = NULL;

    if (dev_id == &gpio_buttons[0]) { i = 0; gpio_button = dev_id; }
    if (dev_id == &gpio_buttons[1]) { i = 1; gpio_button = dev_id; }
    if (dev_id == &gpio_buttons[2]) { i = 2; gpio_button = dev_id; }
    if (dev_id == &gpio_buttons[3]) { i = 3; gpio_button = dev_id; }

    //printk(KERN_INFO BTN_T1_MOD_NAME ": interrupt\n");
    if (i != -1) {
        //
        
        
        gpio_button->button_count++;
        int value = !gpio_get_value(gpio_button->led_pin);
        gpio_set_value(gpio_button->led_pin, value);

        if (!value) {
            BIT_SET(state, i);
        } else {
            BIT_CLEAR(state, i);
        }
        BIT_SET(state, i + 4);

        //state = gpio_button->button_count;

        return IRQ_HANDLED;
    }
    return IRQ_NONE; 
}

/*--------------------------------------------------------------------
 * module initialization
 * return value:  zero if successfully initialized
 *                error otherwise
 *-------------------------------------------------------------------*/
static int __init btn_int_t1_init(void)
{
	int irqt = 0;
    int i = 0;
    int rc = 0;
    int i_dev = 0;
    //s_btn_T1_int_ct = 0;

	printk( KERN_INFO BTN_T1_MOD_NAME " V%s (compiled: %s %s)\n",
	            BTN_T1_MOD_VER, __DATE__, __TIME__);
	/*
	 * display the result of some macros
	 */
	printk(KERN_INFO BTN_T1_MOD_NAME ": Some Macros\n");
	printk(KERN_INFO BTN_T1_MOD_NAME ": gpio_to_irq(%d)=%d\n", GPIO_T1, gpio_to_irq(GPIO_T1));




    register_chrdev_region(dev0, BTN_MAX_DEV, BTN_DRVR_NAME);
    kobject_set_name(&cdev.kobj, BTN_DEV_NAME);
	cdev.owner = THIS_MODULE;
	cdev_init(&cdev, &btn_ops);
	rc = cdev_add(&cdev, dev0, BTN_MAX_DEV);
	class = class_create(THIS_MODULE, class_name);
	for (i_dev = 0; i_dev < BTN_MAX_DEV; i_dev++) {
		//struct gpio *pdev = btn_gpio + i_dev;
		//printk(KERN_WARNING BTN_DRVR_NAME ": creating device\n");
		//printk(KERN_WARNING BTN_DRVR_NAME ":  dev_t=0x%x label=%s\n", dev0+i_dev, pdev->label);
		btn_dev[i_dev] = device_create(class, NULL, dev0+i_dev, NULL, "%s", "nonblocking");
		//if (IS_ERR(btn_dev[i_dev])) {
		//	printk(KERN_WARNING BTN_DRVR_NAME ": creating dev failed\n");
		//	goto destroyDevs;
		//}
	}






	/*
	 * initialize interrupt for T1 
	 * - configure GPIO_L1 for output, intially low
	 * - configure GPIO_T1 for input
	 * - set falling/rising edge (of both) for GPIO_T1
	 * - install handler and enable interrupt
	 */
	switch (*irqtype) {
		case 'f':
		case 'F': irqt = IRQF_TRIGGER_FALLING; break;
		case 'b':
		case 'B': irqt = IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING; break;
		default: irqt = IRQF_TRIGGER_RISING; break;
 	}
	/*
	 * configure GPIO (output) to show pin level
	 * requesting GPIO (input) and IRQ
	 */
    for (;i < 4; i++) {
        gpio_request(gpio_buttons[i].led_pin, gpio_buttons[i].led_name);
        gpio_direction_output(gpio_buttons[i].led_pin, 0);
        //gpio_set_value(gpio_buttons[i].led_pin, gpio_get_value(gpio_buttons[i].button_pin));
        gpio_request_one(gpio_buttons[i].button_pin, GPIOF_IN, gpio_buttons[i].dev_name);

        /* init button1 interrupt */
        gpio_buttons[i].button_irq = gpio_to_irq(gpio_buttons[i].button_pin);
        rc = request_irq(gpio_buttons[i].button_irq, &btn_T1_handler, irqt, gpio_buttons[i].dev_name, (void*)&gpio_buttons[i]);

        gpio_buttons[i].button_count = 0;
    }
	return 0;
	/*
	 * error exits
	 */
//releaseGpioT1:
	gpio_free(GPIO_T1);
//releaseGpioL1:
	gpio_free(GPIO_L1);
	return (-1);
}
/*----------------------------------------------------------------------------
 * free irq and gpio, display ISR counter
 * return -
 *--------------------------------------------------------------------------*/
static void __exit btn_int_t1_exit(void)
{
    int i = 0;
    for (;i < 4; i++) {
        free_irq(gpio_buttons[i].button_irq, &gpio_buttons[i]);
        gpio_free(gpio_buttons[i].button_pin);
        gpio_free(gpio_buttons[i].led_pin);
    	printk(KERN_INFO BTN_T1_MOD_NAME " ISR counter: %d\n", gpio_buttons[i].button_count);
    }




    device_destroy(class, dev0);
	class_destroy(class);
	kobject_put(&cdev.kobj);
	cdev_del(&cdev);
	unregister_chrdev_region(cdev.dev, BTN_MAX_DEV);





   	printk(KERN_INFO BTN_T1_MOD_NAME " unloaded\n");
    return;
}




static int btn_open(struct inode *inode, struct file *file)
{
	/*int minor = MINOR(inode->i_rdev);
	file->private_data = &btn_gpio[minor];
	#ifdef BTN_OPEN_DEBUG
	printk( KERN_INFO BTN_DRVR_NAME " opened. Minor #:%d\n", minor);
	#endif*/
	return 0;
}

/*----------------------------------------------------------------------------
 * close device - unlink the file object
 * return - always success (0)
 *--------------------------------------------------------------------------*/
static int btn_close(struct inode *inode, struct file *file)
{/*
	#ifdef BTN_CLOSE_DEBUG
	printk( KERN_INFO BTN_DRVR_NAME " closed.\n");
	#endif
	file->private_data = NULL;*/
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
	/*struct gpio *pgpio;
	#ifdef BTN_READ_DEBUG
	printk(KERN_WARNING BTN_DRVR_NAME 
		": read(buf%p,size:%u,ppos:%p,pos:%llu)\n",
		buf, size, ppos, ppos ? *ppos: 0);
	#endif*/
	if (*ppos) {
		*ppos = 0; 
		return 0;
	}
/*
	if (file->private_data == NULL)
		return -EFAULT;
	pgpio = (struct gpio *)file->private_data;

	val = gpio_get_value(pgpio->gpio);
	#ifdef BTN_READ_DEBUG
	printk(KERN_WARNING BTN_DRVR_NAME ": gpio=%d read(0x%08x)\n",
		pgpio->gpio, val);
	#endif

	
//	if (val)
	if (!val)
		onOff = "on\n";
	else
		onOff = "off\n";*/
    //buf[0] = state;

	//len = strlen(onOff);
	notCopied = copy_to_user(buf, &state, 1);
	if (notCopied) 
		return(-1);
	*ppos = len;
    return 1;
}




MODULE_LICENSE("GPL");
module_init(btn_int_t1_init);
module_exit(btn_int_t1_exit);

