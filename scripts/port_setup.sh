#!/usr/bin/env bash

#ls /dev/ttyACM*
#PortNum=$(ls /dev/ttyACM* | sed -e s/[^0-9]//g)
##PortNum=$(ls /home/willard/ttyACM* | sed -e s/[^0-9]//g)
#CanPort="/dev/ttyACM$PortNum"
#
##portNum=$(ls /dev/ttyACM*)
##portNum="${portNum: -1}"
#echo $CanPort

#sudo slcand -o -c -s6 $CanPort can0
#sudo slcand -o -c -s6 /dev/ttyACM12 can0
sudo slcand -o -c -s6 CANable1 can0
sudo ifconfig can0 up
sudo ifconfig can0 txqueuelen 1000

