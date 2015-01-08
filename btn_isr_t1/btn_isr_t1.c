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
const char *btnT1devId  ="btnT1id";     /* device id for GPIO T1 irq */
const char *btnT1devName="btnT1dev";    /* device name for GPIO_T1 irq */
static int s_btn_T1_irq = 0;     /* IRQ for GPIO T1 ISR */
static int s_btn_T1_int_ct = 0;  /* number of processed GPIO T1 ISR */

/*--------------------------------------------------------------------
 * Button T1 interrupt handler - count the interrupt, toggle LED1
 *
 *  Note: an interrupt handler MUST NOT block, thus the DPRINT
 *        statement may not work.
 *-------------------------------------------------------------------*/
static irqreturn_t btn_T1_handler(int irq, void *dev_id)
{
    if (dev_id != btnT1devId) {
        return IRQ_NONE;
    }
    s_btn_T1_int_ct++;
	return IRQ_HANDLED;

/*
s_btn_T1_int_ct++;
#ifdef BTN_T1_HANDLER_DEBUG
DPRINT("count:%d\n", s_btn_T1_int_ct);
#endif
gpio_set_value(GPIO_L1, !gpio_get_value(GPIO_L1));
return IRQ_HANDLED;
*/
}

/*--------------------------------------------------------------------
 * module initialization
 * return value:  zero if successfully initialized
 *                error otherwise
 *-------------------------------------------------------------------*/
static int __init btn_int_t1_init(void)
{
	int rc = 0;
	int irqt = 0;
    int ret = 0;

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
/*
gpio_set_value(GPIO_L1, !gpio_get_value(GPIO_T1));
rc = gpio_request_one(GPIO_T1, GPIOF_IN, devName);
*/


    s_btn_T1_irq = gpio_to_irq(GPIO_T1);


    ret = request_irq(s_btn_T1_irq, &btn_T1_handler, irqt, btnT1devName, (void*)btnT1devId);
    if (ret != 0) {
        printk(KERN_INFO BTN_T1_MOD_NAME ": failed to request_urq %d\n", ret);
        return -1;
    }

	/********** INSERT YOUR CODE HERE *********/

	printk(KERN_INFO BTN_T1_MOD_NAME ": initialize irq %d\n", s_btn_T1_irq);

	return 0;
	/*
	 * error exits
	 */
releaseGpioT1:
	gpio_free(GPIO_T1);
releaseGpioL1:
	gpio_free(GPIO_L1);
	return (-1);
}
/*----------------------------------------------------------------------------
 * free irq and gpio, display ISR counter
 * return -
 *--------------------------------------------------------------------------*/
static void __exit btn_int_t1_exit(void)
{
    free_irq(s_btn_T1_irq, (void*)btnT1devId);
    gpio_free(GPIO_T1);
    //gpio_free(GPIO_L1);

	printk(KERN_INFO BTN_T1_MOD_NAME " ISR counter: %d\n", s_btn_T1_int_ct);
   	printk(KERN_INFO BTN_T1_MOD_NAME " unloaded\n");
    	return;
}

MODULE_LICENSE("GPL");
module_init(btn_int_t1_init);
module_exit(btn_int_t1_exit);

