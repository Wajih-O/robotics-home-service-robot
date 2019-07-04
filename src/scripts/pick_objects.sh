#!/bin/sh
xterm -e "  export ROBOT_INITIAL_POSE=\"-x 0 -y 0\" && roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(pwd)/src/map/house_14x12_tall_doors_clear.world" &
sleep 5
xterm  -e " roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$(pwd)/src/map/map.yaml initial_pose_x:=0.0 initial_pose_y:=0.0 initial_pose_a:=-1.57" &
sleep 5
xterm -e " roslaunch  turtlebot_rviz_launchers view_navigation.launch" &
sleep 5
xterm  -e "  roslaunch pick_objects pick_objects.launch" &
sleep 5
