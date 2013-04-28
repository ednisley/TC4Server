#!/bin/sh
#
# Load the Leonardo bootloader back onto a Leonardo

# Location of the Arduino applications "root" directory
ADIR=/Applications/Arduino.app/Contents/Resources/Java

# Paths to avrdude and the avrdude configuration file
AVRDUDE=$ADIR/hardware/tools/avr/bin/avrdude
CONF=$ADIR/hardware/tools/avr/etc/avrdude.conf

PROGRAMMER=usbtiny
PORT=usb
PROCESSOR=m32u4

# Bootloader firmware
BOOTLOADER=$ADIR/hardware/arduino/bootloaders/caterina/Caterina-Leonardo.hex

# Load the bootloader
$AVRDUDE -C$CONF -c$PROGRAMMER -P$PORT -p$PROCESSOR -U flash:w:$BOOTLOADER
