#!/bin/sh
#xterm -e " roslaunch robot_pose_ekf robot_pose_ekf.launch "&
#sleep 3
xterm -e "  export ROBOT_INITIAL_POSE=\"-x 2 -y 2\" && roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(pwd)/src/map/furnished_house_14x12_world.world" &
sleep 5
xterm  -e " roslaunch turtlebot_gazebo gmapping_demo.launch" &
sleep 5
#xterm -e " roslaunch  turtlebot_rviz_launchers view_navigation.launch" &
#sleep 5
xterm  -e "  roslaunch turtlebot_teleop keyboard_teleop.launch" &
sleep 5

#xterm  -e " roslaunch gmapping slam_gmapping_pr2.launch" &