#!/bin/sh
#
# check of all_leds and button_poll drivers are loaded
#
# poll buttons S1 through S4 until you press Ctrl/C
# show button state on the corresponding LED:
# S1 on L1, S2 on L2, and so on
#
# myf1, Sep 2013
#
# set trap for CTRL/C
trap '{ echo "cleanup"; exit 1; }' INT
echo "type CTRL/C to stop polling"
BTN_MINOR_NUMBERS="1 2 3 4"
BTN_DEV=/dev/gpioBtn
LED_DEV=/dev/gpioLed
BTN_DRIVER=button_poll
LED_DRIVER=leds_wronly

if [ ! -e ${BTN_DEV}1  ]; then
	echo "check if ${BTN_DRIVER} driver is loaded" 
	exit 1
fi
if [ ! -e ${LED_DEV}1  ]; then
	echo "check if ${LED_DRIVER} driver is loaded"
	exit 1 
fi

sleep 1

while true; do
	for i in ${BTN_MINOR_NUMBERS}; do
		echo -n "Button S${i} is:"
		STATE=$(/bin/cat ${BTN_DEV}${i})
		echo $STATE
		echo $STATE > ${LED_DEV}${i}
	done
	sleep 0.2
done


