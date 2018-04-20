Moitessier Device Driver
========================

The moitessier device driver is a linux kernel module that enables communication with the Moitessier
navigation HAT.
It is designed for and tested on the Raspberry Pi but it should work (with possible minor changes) 
on any other devices.


Installation
------------

1.  Compile the driver. Pass ARCH and CROSS_COMPILE options to make command. For example:
        host> make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- all KDIR=<PATH_TO_KERNEL_SRC>

3.  Load resulting moitessier.ko module:
        raspi> sudo insmod moitessier.ko


Usage
-----

raspi> echo /dev/moitessier.tty
or
raspi> echo /dev/moitessier.spi
