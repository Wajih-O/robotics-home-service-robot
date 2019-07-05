/* @modify date 2019-07-03 04:21:38
 * Copyright (c) 2010, Willow Garage, Inc.
 * All rights reserved.

 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "utils.hpp"
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <add_markers/pick_object_mission.h>

enum class RobotState { START, LOADED, UNLOADED };

/**
 * A helper function defining the pickup/drop off proximity condition
 */

bool is_close_enough(std::pair<float, float> pos_1,
                     std::pair<float, float> pos_2, float epsilon = 1.0) {
  return std::sqrt(std::pow(pos_1.first - pos_2.first, 2) +
          std::pow(pos_1.second - pos_2.second, 2)) < epsilon;
}

/**
 * A helper function to translate position within a 2D plane
 */

std::pair<float, float> translate(std::pair<float, float> pos,
                                  std::pair<float, float> translation) {
  return std::make_pair(pos.first + translation.first,
                        pos.second + translation.second);
}

/**
 * A helper function to check the state of the moving state of the robot given
 * the linear x, y velocity and the angular
 */
bool is_moving(float linear_x, float linear_y, float angular_z,
               float epsilon = 0.001) {
  return std::pow(linear_x, 2) + std::pow(linear_y, 2) +
             std::pow(angular_z, 2) >
         std::pow(epsilon, 2);
}

/**
 * A helper function to pin a marker at position
 * @param marker
 * @param position default origin/robot position
 */

void pin_marker(visualization_msgs::Marker &marker,
                std::pair<float, float> x_y_pos) {
  marker.pose.position.x = x_y_pos.first;
  marker.pose.position.y = x_y_pos.second;
  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo:
  // 3 (DELETEALL)
  marker.action = visualization_msgs::Marker::ADD;
}

/**
 * a service to pick up an object (marker) from a pick_up to a drop off
 * positions. Where the coordinate of the drop off is relative to the pick up
 * location
 */

class PickUpDropOffHomeService {

public:
  PickUpDropOffHomeService(std::pair<float, float> pick_up_position,
                           std::pair<float, float> drop_off_position)
      : pick_up_position(pick_up_position),
        drop_off_position(drop_off_position) {

    robot_state = RobotState::START; // initialize the state of the robot
    marker_pub = node_handle.advertise<visualization_msgs::Marker>(
        "visualization_marker", 5);
    pick_object_mission_pub = node_handle.advertise<add_markers::pick_object_mission>("pick_object_missions", 100);
    odom_subscriber = node_handle.subscribe(
        "odom", 1000, &PickUpDropOffHomeService::pickup_dropoff, this);

    // Create a cube marker
    marker = create_marker(0);
    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1) {
      if (!ros::ok()) {
        return;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    // pin the marker at the pick-up position for 5 seconds
    pin_marker(marker, pick_up_position);
    marker_pub.publish(marker);

    // pin some marker for debugging (to be removed later)
    auto origin_marker =create_marker(1, visualization_msgs::Marker::SPHERE, std::make_pair(0.0, 0.0), .2, std::make_tuple(1.0f, 0.0f, 0.0f, 1.0f));
    auto pick_up_marker = create_marker(2, visualization_msgs::Marker::SPHERE, std::make_pair(0.0, 0.0), .2, std::make_tuple(1.0f, 0.0f, 0.0f, 1.0f));
    auto drop_off_marker = create_marker(3, visualization_msgs::Marker::SPHERE, std::make_pair(0.0, 0.0), .2, std::make_tuple(1.0f, 0.0f, 0.0f, 1.0f));
    pin_marker(origin_marker, std::make_pair(0.0, 0.0));
    pin_marker(pick_up_marker, pick_up_position);
    pin_marker(drop_off_marker, translate(drop_off_position, pick_up_position));
    // publish the markers
    marker_pub.publish(origin_marker);
    marker_pub.publish(pick_up_marker);
    marker_pub.publish(drop_off_marker);

    // create mission message
    while (pick_object_mission_pub.getNumSubscribers() < 1) {
      if (!ros::ok()) {
        return;
      }
      ROS_WARN_ONCE("waiting for pick object missions subscriber");
      sleep(1);
    }
    add_markers::pick_object_mission mission_msg;
    mission_msg.pick_up_x = pick_up_position.first;
    mission_msg.pick_up_y = pick_up_position.second;
    mission_msg.drop_off_x = drop_off_position.first;
    mission_msg.drop_off_y = drop_off_position.second;

    pick_object_mission_pub.publish(mission_msg);

  }

  void pick_up() {
    // Pick up (hide the marker)
    this->marker.action = visualization_msgs::Marker::DELETE;
    this->marker_pub.publish(this->marker);
    robot_state = RobotState::LOADED; // update robot state
    ROS_INFO("loaded !");
  }

  void drop_off() {
    pin_marker(this->marker, std::make_pair(0.6, 0.0));
    this->marker_pub.publish(this->marker);
    robot_state = RobotState::UNLOADED; // update robot state
  }

  void pickup_dropoff(const nav_msgs::Odometry::ConstPtr &msg) {
    switch (robot_state) {
      // if robot is close to the pickup position
    case RobotState::START:
      if (is_close_enough(std::make_pair(msg->pose.pose.position.x,
                                         msg->pose.pose.position.y),
                          pick_up_position) &&
          !is_moving(msg->twist.twist.linear.x, msg->twist.twist.linear.y,
                     msg->twist.twist.angular.z)) {
        ROS_INFO("Reached loading (pick up) Position -> x: [%f], y: [%f] "
                 "loading ...",
                 msg->pose.pose.position.x, msg->pose.pose.position.y);
        ROS_INFO("Loaded robot ! position -> x: [%f], y: [%f] (goal  x: [%f], "
                 "y: [%f]) ",
                 msg->pose.pose.position.x, msg->pose.pose.position.y,
                 pick_up_position.first, pick_up_position.second);
        pick_up();
      }
      break;
    case RobotState::LOADED:
      auto absolute_goal = translate(drop_off_position, pick_up_position);
      ROS_INFO("Loaded robot ! position -> x: [%f], y: [%f] (goal  x: [%f], y: "
               "[%f]) ",
               msg->pose.pose.position.x, msg->pose.pose.position.y,
               absolute_goal.first, absolute_goal.second);
      if (is_close_enough(std::make_pair(msg->pose.pose.position.x,
                                         msg->pose.pose.position.y),
                          absolute_goal) &&
          !is_moving(msg->twist.twist.linear.x, msg->twist.twist.linear.y, 0)) {
        ROS_INFO("Reached drop off Position -> x: [%f], y: [%f]  unloading ...",
                 msg->pose.pose.position.x, msg->pose.pose.position.y);
        drop_off();
      }
      break;
    }
  }

private:
  visualization_msgs::Marker marker;
  // define pick-up drop-off zones
  std::pair<float, float> pick_up_position;
  std::pair<float, float> drop_off_position;
  //  NodeHandle
  ros::NodeHandle node_handle;
  // mission publisher
  ros::Publisher pick_object_mission_pub;
  // marker publisher
  ros::Publisher marker_pub;
  RobotState robot_state;
  ros::Subscriber odom_subscriber;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "add_markers");
  PickUpDropOffHomeService mission(std::make_pair(-1.0, -6.0),
                                   std::make_pair(-3.0, 0.0));
  ros::spin();

  return 0;
}
