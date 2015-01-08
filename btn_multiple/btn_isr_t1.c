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
static irqreturn_t btn_T1_handler(int irq, void *dev_id)
{
    //printk(KERN_INFO BTN_T1_MOD_NAME ": interrupt\n");
    if (dev_id == &gpio_buttons[0] || dev_id == &gpio_buttons[1] || dev_id == &gpio_buttons[2] || dev_id == &gpio_buttons[3]) {
        struct gpio_dev* gpio_button = dev_id;
        gpio_button->button_count++;
        gpio_set_value(gpio_button->led_pin, !gpio_get_value(gpio_button->led_pin));
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
    //s_btn_T1_int_ct = 0;

	printk( KERN_INFO BTN_T1_MOD_NAME " V%s (compiled: %s %s)\n",
	            BTN_T1_MOD_VER, __DATE__, __TIME__);
	/*
	 * display the result of some macros
	 */
	printk(KERN_INFO BTN_T1_MOD_NAME ": Some Macros\n");
	printk(KERN_INFO BTN_T1_MOD_NAME ": gpio_to_irq(%d)=%d\n", GPIO_T1, gpio_to_irq(GPIO_T1));

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
   	printk(KERN_INFO BTN_T1_MOD_NAME " unloaded\n");
    return;
}

MODULE_LICENSE("GPL");
module_init(btn_int_t1_init);
module_exit(btn_int_t1_exit);

