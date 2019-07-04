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

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "utils.hpp"



/**
 * A helper function to pin a marker to a given x, y position
 * @param marker
 * @param position
 */

void pin_marker(visualization_msgs::Marker &marker,
                std::pair<float, float> x_y_pos) {
  marker.pose.position.x = x_y_pos.first;
  marker.pose.position.y = x_y_pos.second;
  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: (DELETEALL)
  marker.action = visualization_msgs::Marker::ADD;
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub =
      n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  // define pick-up drop-off zones
  std::pair<float, float> pick_up = std::make_pair(-1.0, -6.0);
  std::pair<float, float> drop_off = std::make_pair(5.0, 1.0);

  // Create a cube marker
  auto marker = create_marker(0);
  // Publish the marker
  while (marker_pub.getNumSubscribers() < 1) {
    if (!ros::ok()) {
      return 0;
    }
    ROS_WARN_ONCE("Please create a subscriber to the marker");
    sleep(1);
  }
  // pin the marker at the pick-up position for 5 seconds
  pin_marker(marker, pick_up);
  marker_pub.publish(marker);
  sleep(5);
  // Hide the marker
  marker.action = visualization_msgs::Marker::DELETE;
  marker_pub.publish(marker);
  // Move the marker to the drop-off position
  pin_marker(marker, std::make_pair(pick_up.first + drop_off.first,
                                    pick_up.second + drop_off.second));
  marker_pub.publish(marker);

  ros::spin();

  return 0;
}
