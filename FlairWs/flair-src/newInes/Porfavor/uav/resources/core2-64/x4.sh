#! /bin/bash

if [ -f /proc/xenomai/version ];then
	EXEC=./Porfavor_rt
else
	EXEC=./Porfavor_nrt
fi

$EXEC -n Drone_0 -a 127.0.0.1 -p 9000 -l /tmp -x setup_x4.xml -t x4_simu 
