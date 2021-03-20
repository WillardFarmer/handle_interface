#!/usr/bin/env bash
portNum=$(ls /dev/ttyACM*)
port = 

sudo slcand -o -c -s6 /dev/ttyACM2 can0
sudo ifconfig can0 up
sudo ifconfig can0 txqueuelen 1000

