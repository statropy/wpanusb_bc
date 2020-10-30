.. wpanusb_bc:

wpanusb for Beagle Connect
##############

Overview
********

This application is derived from the Zephyr samples/net/wpanusb sample.
It is intended to be used with the Beagle Connect board. 
This application exports ieee802154 radio over USB to be used in
other OSes such as Linux.  For Linux, the ieee802154 stack would be
implemented using the Linux SoftMAC driver.

Requirements
************

- a Beagle Connect board connected via USB to a Linux host
- wpanusb Linux kernel driver (in the process of being open sourced)
- wpan-tools (available for all Linux distributions)

Building
********

Clone and update the submodule for the BeagleConnect board definition

.. code-block:: console

    cd ~
    git clone https://github.com/statropy/wpanusb_bc
    cd wpanusb_bc
    git submodule init
    git submodule update
    cd <zephyr root>

Build the wpanusb_bc project for 2.4GHz:

.. code-block:: console

    west build -b beagle_connect ~/wpanusb_bc

Or build the wpanusb_bc project for Sub GHz:

.. code-block:: console

    west build -b beagle_connect ~/wpanusb_bc -- -DOVERLAY_CONFIG=overlay-subghz.conf

A build directory can be specified with the -d option

Updating with west flash
************************

To program the BeagleConnect with west flash the MSP430 needs to be loaded with the 
master branch of https://github.com/statropy/msp430F55xx_usb_uart_bridge. See 
https://github.com/statropy/msp430F55xx_usb_uart_bridge/blob/master/README.md for
instructions on building and programming the MSP430.

Then to flash the CC1352 with the wpanusb_bc program:

.. code-block:: console
    west flash

Now update the MSP430 again, this time with the wpan_endpoint branch. Checkout that 
branch and follow the same programming instructions. This allows the wpanusb_bc program 
to communicate with the linux host.

The linux host needs the wpanusb kernel module, clone and follow the instructions at
https://github.com/statropy/wpanusb
