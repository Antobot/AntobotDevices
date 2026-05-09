#!/bin/bash

sudo busybox devmem 0x02430068 w 0x004
sudo busybox devmem 0x0c303018 w 0x0000c458
sudo busybox devmem 0x0c303010 w 0x0000c400
sudo modprobe can
sudo modprobe can_raw
sudo modprobe mttcan
sudo ip link set down can0
#sudo ip link set can0 type can bitrate 500000 #dbitrate 2000000 berr-reporting on fd on
#sudo ip link set can0 type can restart-ms 100
sudo ip link set can0 type can bitrate 1000000 loopback off
sudo ip link set can0 txqueuelen 1000
sudo ip link set up can0
#sudo ip link set down can0
#candump -x any &
#cansend can0 123#abcdabcd
#ip -details -statistics link show can0

