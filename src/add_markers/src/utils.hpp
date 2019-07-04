#include <string>
#include <visualization_msgs/Marker.h>


/**
 * A helper function to create a marker (and pin it) at a position
 * @param shape marker shape, default CUBE (visualization_msgs::Marker::CUBE)
 * @param x_y_pos  x, y coordinate as std::pair<float, float>
 * @scale marker scale
 */

visualization_msgs::Marker
create_marker(uint32_t shape = visualization_msgs::Marker::CUBE,
              std::pair<float, float> x_y_pos = std::make_pair(0, 0),
              float scale = 0.5, std::string frame_id = "base_footprint", std::string ns="add_markers") {

  // create and initialize marker to be in x_y_pos
  visualization_msgs::Marker marker;
  // Set the frame ID and timestamp.  See the TF tutorials for information on
  // these.
  marker.header.frame_id = frame_id;
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique
  // ID Any marker sent with the same namespace and id will overwrite the old
  // one

  marker.ns = ns;
  marker.id = 0;

  // Set the marker type.  Initially this is CUBE, and cycles between that and
  // SPHERE, ARROW, and CYLINDER

  marker.type = shape;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the
  // frame/time specified in the header

  marker.pose.position.x = x_y_pos.first;
  marker.pose.position.y = x_y_pos.second;

  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = scale * 1.0;
  marker.scale.y = scale * 1.0;
  marker.scale.z = scale * 1.0;

  marker.pose.position.z = marker.scale.z/2; // pose the marker on the  z==0 plane

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  // marker lifetime
  marker.lifetime = ros::Duration();

  return marker;
}