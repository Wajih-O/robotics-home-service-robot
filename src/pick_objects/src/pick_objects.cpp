#include <tuple>
#include <vector>

#include <actionlib/client/simple_action_client.h>
#include <add_markers/pick_object_mission.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/ros.h>

// Define a client for to send goal requests to the move_base server through a
// SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
    MoveBaseClient;

class PickObject {

public:
  PickObject() : move_base_client(new MoveBaseClient("move_base", true)) {
    ROS_INFO(" PickObject (init mission exec) !");
    // Wait 5 sec for move_base action server to come up
    while (!move_base_client->waitForServer(ros::Duration(5.0))) {
      ROS_INFO("Waiting for the move_base action server to come up");
    }
    ROS_INFO(" Init mission subscriber !");
    missions_subscriber = node_handle.subscribe(
        "pick_object_missions", 100, &PickObject::mission_msg_handler, this);
  }

private:
  void mission_msg_handler(const add_markers::pick_object_mission &msg) const {
    ROS_INFO(" mission received pick-up -> x: [%f], y: [%f] drop-off -> x: "
             "[%f], y: [%f]",
             msg.pick_up_x, msg.pick_up_y, msg.drop_off_x, msg.drop_off_y);

    // setup goals pick-up drop-off (to reach in the mission) as pairs of
    // position.x, position.y
    std::vector<std::pair<float, float>> goals_to_reach{
        std::make_pair(msg.pick_up_x, msg.pick_up_y),
        std::make_pair(msg.drop_off_x, msg.drop_off_y)};

    move_base_msgs::MoveBaseGoal goal;

    // set up the frame parameters
    goal.target_pose.header.frame_id = "base_footprint";
    goal.target_pose.header.stamp = ros::Time::now();

    // orientation (keeps the orientation cst for pick-up drop-off goals)
    goal.target_pose.pose.orientation.w = 1.0;
    goal.target_pose.pose.orientation.x = 0.0;
    goal.target_pose.pose.orientation.y = 0.0;
    goal.target_pose.pose.orientation.z = 0.0;

    auto reached_goal_counter = 0;
    for (auto x_y_position : goals_to_reach) {
      // Define a position and orientation for the robot to reach (relative to
      // the current robot position)
      goal.target_pose.pose.position.x = x_y_position.first;
      goal.target_pose.pose.position.y = x_y_position.second;

      // Send the goal position and orientation for the robot to reach
      ROS_INFO("Sending goal");
      move_base_client->sendGoal(goal);

      // Wait an infinite time for the results
      move_base_client->waitForResult();

      // Check if the robot reached its goal
      if (move_base_client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        reached_goal_counter += 1;
        ROS_INFO("Hooray, robot reached goal %d", reached_goal_counter);
        // wait 5 seconds for pickup
        sleep(5);
      } else {
        ROS_INFO("The base failed to move to the goal");
      }
    }
  }

private:
  MoveBaseClient *move_base_client;
  ros::NodeHandle node_handle;
  ros::Subscriber missions_subscriber;
};

int main(int argc, char **argv) {
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");
  PickObject pick_object = PickObject();
  ros::spin();
  return 0;
}