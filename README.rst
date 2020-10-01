.. _wpanusb-sample:

wpanusb sample for Beagle Connect
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

Building and Running
********************

Build the wpanusb sample for a board:

.. zephyr-app-commands::
   :zephyr-app: wpanusb-bc
   :board: beagle-connect
   :goals: build
   :compact:

The following script enables the network interface in Linux
(uses iwpan tool from above):

.. code-block:: console

    #!/bin/sh
    PHY=`iwpan phy | grep wpan_phy | cut -d' ' -f2`
    echo 'Using phy' $PHY
    iwpan dev wpan0 set pan_id 0xabcd
    iwpan dev wpan0 set short_addr 0xbeef
    iwpan phy $PHY set channel 0 26
    ip link add link wpan0 name lowpan0 type lowpan
    ip link set wpan0 up
    ip link set lowpan0 up
