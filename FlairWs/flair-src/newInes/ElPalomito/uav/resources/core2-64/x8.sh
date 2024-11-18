#! /bin/bash

if [ -f /proc/xenomai/version ];then
	EXEC=./ElPalomito_rt
else
	EXEC=./ElPalomito_nrt
fi

$EXEC -n Drone_0 -a 127.0.0.1 -p 9000 -l ./ -x setup_x8.xml -t x8_simu
