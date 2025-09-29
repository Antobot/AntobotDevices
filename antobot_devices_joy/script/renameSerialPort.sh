#!/bin/bash

Device_Name="/dev/ttyUSB0"
LINK_NAME="anto_gps" # anto_gps / anto_joy
RULE_FILE="/etc/udev/rules.d/99-usb_$LINK_NAME.rules"
# echo $RULE_FILE
sudo rm -rf $RULE_FILE

KPATH=$(udevadm info -q path -n "$Device_Name" | grep -o 'usb[0-9]\+/\([^/]\+\)' | tail -n1 | cut -d/ -f2)
# echo $KPATH

# # CH340： ATTRS{idVendor}=="1a86"； ATTRS{idProduct}=="7523"
RULE="SUBSYSTEM==\"tty\", KERNELS==\"$KPATH\", ATTRS{idVendor}==\"1a86\", ATTRS{idProduct}==\"7523\", MODE=\"0777\", SYMLINK+=\"$LINK_NAME\", GROUP=\"dialout\""

echo "$RULE" | sudo tee $RULE_FILE > /dev/null

sudo udevadm control --reload-rules
sudo udevadm trigger


