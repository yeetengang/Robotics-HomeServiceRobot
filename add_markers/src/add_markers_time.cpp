#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "nav_msgs/Odometry.h"
#include <cmath>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  uint32_t shape = visualization_msgs::Marker::CUBE;
  bool finish_msg = false;

  enum State {
    PICKUP,  // Going to pick-up zone
    CARRY,   // Carrying to drop-off zone
    DROP,    // Package dropped-off
  } state = PICKUP;

  ROS_INFO("Going to pick-up zone.. ");
  while (ros::ok()){
    visualization_msgs::Marker marker;

    // Set marker parameters
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.id = 0;
    marker.type = shape;

    n.getParam("/pick_up_loc/tz", marker.pose.position.z);
    n.getParam("/pick_up_loc/qx", marker.pose.orientation.x);
    n.getParam("/pick_up_loc/qy", marker.pose.orientation.y);
    n.getParam("/pick_up_loc/qz", marker.pose.orientation.z);
    n.getParam("/pick_up_loc/qw", marker.pose.orientation.w);

    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    while (marker_pub.getNumSubscribers() < 1) {
      if (!ros::ok())
        return 0;
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }

    if (state == PICKUP) {
      marker.action = visualization_msgs::Marker::ADD;
      n.getParam("/pick_up_loc/tx", marker.pose.position.x);
      n.getParam("/pick_up_loc/ty", marker.pose.position.y);
      marker_pub.publish(marker);
      sleep(5); // Wait 5 seconds for virtual object to disappear
      ROS_INFO("Package picked-up. Now carrying to the drop-off zone..");
      state = CARRY;
    }
    else if (state == CARRY) {
      marker.action = visualization_msgs::Marker::DELETE;
      marker_pub.publish(marker);
      sleep(5); // Wait 5 seconds for hidden virtual object to reach drop off point
      ROS_INFO("Package reached drop-off zone and dropped off..");
      state = DROP;
    }
    else /* state == DROP */ {
      n.getParam("/drop_off_loc/tx", marker.pose.position.x);
      n.getParam("/drop_off_loc/ty", marker.pose.position.y);
      marker.action = visualization_msgs::Marker::ADD;
      marker_pub.publish(marker);
      if (!finish_msg) {
        ROS_INFO("Package is at drop-off zone, completed!");
        finish_msg = true;
      }
    }
  }
}