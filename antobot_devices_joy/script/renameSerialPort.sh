#!/bin/bash

Device_Name="/dev/ttyUSB0" # ttyCH341USB0 / ttyUSB0
LINK_NAME="anto_gps" # anto_gps / anto_joy
RULE_FILE="/etc/udev/rules.d/99-usb_$LINK_NAME.rules"

sudo rm -rf $RULE_FILE

BUSHNUM=$(udevadm info -a -n "$Device_Name" | grep 'ATTRS{busnum}' | head -n1 | sed -E 's/.*"([^"]+).*/\1/') 
echo "busnum:$BUSHNUM"

DEVPATH=$(udevadm info -a -n "$Device_Name" | grep 'ATTRS{devpath}' | head -n1 | sed -E 's/.*"([^"]+).*/\1/') 
echo "devpath:$DEVPATH"

RULE="SUBSYSTEM==\"tty\", ATTRS{busnum}==\"$BUSHNUM\", ATTRS{devpath}==\"$DEVPATH\", MODE=\"0777\", SYMLINK+=\"$LINK_NAME\", GROUP=\"dialout\""

echo "$RULE" | sudo tee $RULE_FILE > /dev/null

sudo udevadm control --reload-rules
sudo udevadm trigger


