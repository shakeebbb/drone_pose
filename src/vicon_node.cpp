#include <ros/ros.h>
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PoseStamped.h"

using namespace std;

ros::Publisher pose_pub;
ros::Subscriber transform_sub;

geometry_msgs::PoseStamped poseStamped;

void transform_cb(const geometry_msgs::TransformStamped&);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vicon_node");

  ros::NodeHandle n;
  
  string vicon_topic;
  string remapped_topic;
  
  ros::param::get("vicon_node/vicon_topic", vicon_topic);
  ros::param::get("vicon_node/remapped_topic", remapped_topic);

  transform_sub = n.subscribe(vicon_topic, 1000, transform_cb);
  pose_pub = n.advertise<geometry_msgs::PoseStamped>(remapped_topic, 1000);
  
  ros::spin();
  return  0;
}
void transform_cb(const geometry_msgs::TransformStamped& transformStamped)
{
poseStamped.header = transformStamped.header;
poseStamped.pose.position.x = transformStamped.transform.translation.x;
poseStamped.pose.position.y = transformStamped.transform.translation.y;
poseStamped.pose.position.z = transformStamped.transform.translation.z;
poseStamped.pose.orientation = transformStamped.transform.rotation;

pose_pub.publish(poseStamped);
}