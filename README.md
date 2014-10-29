
USB Host Mass Storage Class Example on Tiva Launchpad (EK-TM4C123GXL)
====================================================================

The code is a port of the usb msc example provided by TI for Connected Launchpad (DK-TM4C129X). When a USB flash drive formatted with a FAT file system is connected to the launchpad via a USB-OTG cable, the usb-serial port on the DEBUG usb port will print the file structure of the flash drive. 

- Place 0 ohm resistors at R25 and R29
- Short Pin's 1 and 2 at Jumper above R14 
- Connect USB-OTG Cable at Device Micro-USB port

