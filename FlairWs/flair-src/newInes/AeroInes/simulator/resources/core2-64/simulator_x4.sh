#! /bin/bash
if [ -f /proc/xenomai/version ];then
	EXEC=./AeroInes_simulator_rt
else
	EXEC=./AeroInes_simulator_nrt
fi

echo "Simona la mona pelona"
$EXEC -n Drone_0 -t x4 -p 9000 -a 127.0.0.1 -x simulator_x4.xml -o 10 -m $FLAIR_ROOT/flair-src/models -s $FLAIR_ROOT/flair-src/models/indoor_flight_arena.xml
