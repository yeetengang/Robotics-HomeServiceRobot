#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "nav_msgs/Odometry.h"
#include <cmath>

double pick_up_position[2];
double drop_off_position[2];
double pose[2] = {0, 0};  // current pose


void get_current_pose(const nav_msgs::Odometry::ConstPtr& msg)
{
  pose[0] = msg->pose.pose.position.x;
  pose[1] = msg->pose.pose.position.y;
}

double get_distance_to(double goal[2])
{
  double dx = goal[0] - pose[0];
  double dy = goal[1] - pose[1];
  return sqrt(dx*dx + dy*dy);
}

bool is_pick_up_zone_reached()
{
  return get_distance_to(pick_up_position) < 0.3;
}

bool is_drop_off_zone_reached()
{
  return get_distance_to(drop_off_position) < 0.3;
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber pose_sub = n.subscribe("/odom", 10, get_current_pose);

  uint32_t shape = visualization_msgs::Marker::CUBE;

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

    n.getParam("/pick_up_loc/tx", pick_up_position[0]);
    n.getParam("/pick_up_loc/ty", pick_up_position[1]);
    n.getParam("/pick_up_loc/tz", marker.pose.position.z);
    n.getParam("/pick_up_loc/qx", marker.pose.orientation.x);
    n.getParam("/pick_up_loc/qy", marker.pose.orientation.y);
    n.getParam("/pick_up_loc/qz", marker.pose.orientation.z);
    n.getParam("/pick_up_loc/qw", marker.pose.orientation.w);

    n.getParam("/drop_off_loc/tx", drop_off_position[0]);
    n.getParam("/drop_off_loc/ty", drop_off_position[1]);

    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();
    
    ros::spinOnce();

    if (state == PICKUP) {
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = pick_up_position[0];
      marker.pose.position.y = pick_up_position[1];
      marker_pub.publish(marker);
      if (is_pick_up_zone_reached()) {
        sleep(5);
        ROS_INFO("Package picked-up. Now carrying to the drop-off zone.. ");
        state = CARRY;
      }
    }
    else if (state == CARRY) {
      marker.action = visualization_msgs::Marker::DELETE;
      marker.pose.position.x = drop_off_position[0];
      marker.pose.position.y = drop_off_position[1];
      marker_pub.publish(marker);
      if (is_drop_off_zone_reached()) {
        ROS_INFO("Package reached drop-off zone and dropped off..");
        state = DROP;
      }
    }
    else /* state == DROP */ {
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = drop_off_position[0];
      marker.pose.position.y = drop_off_position[1];
      marker_pub.publish(marker);
    }
  }
}