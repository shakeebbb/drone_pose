#include <ros/ros.h>
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"

using namespace std;

ros::Publisher pose_pub;
ros::Subscriber odom_sub;

geometry_msgs::PoseStamped poseStamped;

void odom_cb(const nav_msgs::Odometry&);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vision_node");

  ros::NodeHandle n;

  pose_pub = n.advertise<geometry_msgs::PoseStamped>("/mavros/mocap/pose", 1000);
  odom_sub = n.subscribe("/zed/odom", 1000, odom_cb);

  ros::spin();
  return  0;
}

void odom_cb(const nav_msgs::Odometry& odometry)
{
poseStamped.header = odometry.header;
poseStamped.pose.position.x = odometry.pose.pose.position.x;
poseStamped.pose.position.y = odometry.pose.pose.position.y;
poseStamped.pose.position.z = odometry.pose.pose.position.z;
poseStamped.pose.orientation = odometry.pose.pose.orientation;

pose_pub.publish(poseStamped);
}