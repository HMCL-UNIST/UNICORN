#!/bin/bash

echo "Finish trajectory..."
rosservice call /finish_trajectory 0
if [ "$1" = "" ]
then
	echo "Please enter a map name: "
	read map
	echo "Save map under the name $map.pbstream"
	rosservice call /write_state "{filename: '${HOME}/catkin_ws/src/race_stack/stack_master/maps/$map.pbstream', include_unfinished_submaps: "true"}"
else
	echo "Save map under the name $1.pbstream"
	rosservice call /write_state "{filename: '${HOME}/catkin_ws/src/race_stack/stack_master/maps/$1.pbstream', include_unfinished_submaps: "true"}"
fi
