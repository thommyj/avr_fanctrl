#!/bin/bash

set -e


if [ -f fanctrl ]
then
	echo "last build:"
	mv fanctrl fanctrl_old
	avr-size fanctrl_old
fi

if [ -f fanctrl.hex ]
then
	mv fanctrl.hex fanctrl_old.hex
fi

avr-gcc -mmcu=attiny13 -Os -Wall main.c -o fanctrl
avr-objcopy -R .eeprom -O ihex fanctrl fanctrl.hex
echo "new build"
avr-size fanctrl
