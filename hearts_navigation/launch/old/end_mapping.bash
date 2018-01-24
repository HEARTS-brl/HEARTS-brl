#!/bin/bash

if [ $# -lt 1 ]
then
    echo "no map name"
else
    echo "map name is $1"
    rosservice call /pal_map_manager/save_map "directory: '$1'"
    sleep 2
    rosservice call /pal_navigation_sm "input: 'LOC'"
    sleep 5
    rosservice call /pal_map_manager/change_map "input: '$1'"
fi

echo "$0 complete"
