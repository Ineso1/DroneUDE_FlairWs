#! /bin/bash

if [ -f /proc/xenomai/version ];then
	EXEC=./AeroInes_rt
else
	EXEC=./AeroInes_nrt
fi

$EXEC -n Drone_0 -a 172.26.213.62 -p 9000 -l /tmp -x setup_x4_intelaero.xml -t aero
