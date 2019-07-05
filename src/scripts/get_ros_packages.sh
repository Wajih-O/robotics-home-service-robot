#!/bin/bash
if [ $# -ne 1 ]; then
    echo "Usage (should be run from project root): ./src/scripts/get_ros_packages.sh [ssh|https]" && exit 1
fi || exit 1

case $1 in
    "ssh")
    GIT="git@github.com:"
    GIT_SUFF=".git"
    ;;
    "https") # default https
    GIT="https://github.com/"
    GIT_SUFF=""
    ;;

esac

PREF="" # swtich between "echo "/"" to test or enable cloning

# TODO test if directories exist (already cloned)

# gmapping
$PREF git clone $GIT"ros-perception/slam_gmapping"$GIT_SUFF  src/slam_mapping
# teleop
$PREF git clone $GIT"turtlebot/turtlebot"$GIT_SUFF src/turtlebot
# turtlebot_rviz_launchers:
$PREF git clone $GIT"turtlebot/turtlebot_interactions"$GIT_SUFF src/turtlebot_interactions
# turtlebot gazebo
$PREF git clone $GIT"turtlebot/turtlebot_simulator"$GIT_SUFF src/turtlebot_simulator
# robot pose ekf
$PREF git clone $GIT"udacity/robot_pose_ekf"$GIT_SUFF src/robot_pose_ekf
# odom to trajectory
$PREF git clone $GIT"udacity/odom_to_trajectory"$GIT_SUFF src/odom_to_trajectory
# for creating/updating maps
$PREF git clone $GIT"udacity/pgm_map_creator"$GIT_SUFF src/pgm_map_creator