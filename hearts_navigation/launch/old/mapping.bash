#!/bin/bash

rosservice call /pal_navigation_sm "input: 'MAP'"
sleep 2
rosrun rviz rviz -d /home/turtlebot/tb_ws/src/deps/tiago_navigation/tiago_2dnav/config/rviz/navigation_public_sim.rviz
