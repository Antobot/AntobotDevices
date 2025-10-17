#!/bin/bash

devices=(ttyUSB0)
for dev in "${devices[@]}"; do
	cd ~/
	script -c "udevadm info --attribute-walk --name=/dev/$dev" serial_info.log

	logfile="$HOME/serial_info.log"
	count=0
	kernel_value=""
	while IFS= read -r line; do
		if [[ $line =~ KERNELS==\"([^\"]+)\" ]];then
			((count++))
			if [ $count -eq 2 ]; then
				kernel_value="${BASH_REMATCH[1]}"
				break
			fi
		fi
	done < "$logfile" 

	RULE="KERNELS==\"$kernel_value\", MODE:=\"0777\", GROUP:=\"dialout\", SYMLINK+=\"anto_joy\""

	echo "$RULE" | sudo tee -a /etc/udev/rules.d/usb.rules > /dev/null

	sudo udevadm trigger
	
	rm "$logfile"
done

exit 0
