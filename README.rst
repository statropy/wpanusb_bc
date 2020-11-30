.. wpanusb_bc:

wpanusb for BeagleConnect Freedom
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

- BeagleConnect Freedom board connected via USB to a Linux host
- wpanusb Linux kernel driver https://github.com/statropy/wpanusb
- wpan-tools (available for all Linux distributions)

Building
********

Clone and update the submodule for the BeagleConnect Freedom board definition

.. code-block:: console

    cd ~
    git clone https://github.com/statropy/wpanusb_bc
    cd wpanusb_bc
    git submodule init
    git submodule update
    cd <zephyr root>

Build the wpanusb_bc project for 2.4GHz:

.. code-block:: console

    west build -b beagleconnect_freedom ~/wpanusb_bc -- -DDTC_OVERLAY_FILE=beagleconnect_freedom.overlay

Or build the wpanusb_bc project for Sub GHz:

.. code-block:: console

    west build -b beagleconnect_freedom ~/wpanusb_bc -- -DOVERLAY_CONFIG=overlay-subghz.conf -DDTC_OVERLAY_FILE=beagleconnect_freedom.overlay

A build directory can be specified with the -d option

Updating with west flash
************************

Ensure the MSP430 is up-to-date with the latest code.

Then to flash the CC1352 with the wpanusb_bc program:

.. code-block:: console
    west flash

The linux host needs the wpanusb kernel module, clone and follow the instructions at
https://github.com/statropy/wpanusb
