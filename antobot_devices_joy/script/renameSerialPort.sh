#!/bin/bash

sudo rm -rf /etc/udev/rules.d/99-usb.rules

RULE="SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"1a86\", ATTRS{idProduct}==\"7523\", MODE=\"0777\", SYMLINK+=\"anto_joy\", GROUP=\"dialout\""

echo "$RULE" | sudo tee -a /etc/udev/rules.d/99-usb.rules > /dev/null

sudo udevadm trigger
	
ls -l /dev | grep tty

exit 0
