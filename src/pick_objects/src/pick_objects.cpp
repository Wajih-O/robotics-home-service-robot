#include <iostream>
#include <tuple>
#include <vector>

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/ros.h>

// Define a client for to send goal requests to the move_base server through a
// SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
    MoveBaseClient;

int main(int argc, char **argv) {
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");

  // tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while (!ac.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  // setup goals as tuple of  position.x, position.y
  std::vector<std::pair<float, float>> goals_to_reach{std::make_pair(-1.0, -6.0),
                                                      std::make_pair(5.0, 1.0)};

  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "base_footprint";
  goal.target_pose.header.stamp = ros::Time::now();
  auto reached_goal_counter = 0;
  for (auto x_y_position : goals_to_reach) {
    // Define a position and orientation for the robot to reach (relative to the current robot position)
    goal.target_pose.pose.position.x = x_y_position.first;
    goal.target_pose.pose.position.y = x_y_position.second;

    // orientation
    goal.target_pose.pose.orientation.w = 1.0;
    goal.target_pose.pose.orientation.x = 0.0;
    goal.target_pose.pose.orientation.y = 0.0;
    goal.target_pose.pose.orientation.z = 0.0;

    // Send the goal position and orientation for the robot to reach
    ROS_INFO("Sending goal");
    ac.sendGoal(goal);

    // Wait an infinite time for the results
    ac.waitForResult();

    // Check if the robot reached its goal
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      reached_goal_counter += 1;
      ROS_INFO("Hooray, robot reached goal %d", reached_goal_counter);
      // wait 5 seconds for pickup
      sleep(5);
    } else {
      ROS_INFO("The base failed to move to the goal");
    }
  }

  return 0;
}