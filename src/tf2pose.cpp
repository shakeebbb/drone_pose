#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>

using namespace std;

int main(int argc, char** argv){
  ros::init(argc, argv, "tf2pose");

  ros::NodeHandle node;

  string tf_target_frame = "odom";
  string tf_source_frame = "base_link";
  string pose_frame_id = "world";

  ros::param::get("tf2pose/tf_target_frame", tf_target_frame);
  ros::param::get("tf2pose/tf_source_frame", tf_source_frame);
  ros::param::get("tf2pose/pose_frame_id", pose_frame_id);

  ros::Publisher posePub = node.advertise<geometry_msgs::PoseStamped>("pose_topic", 10);

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  ros::Rate rate(50.0);
  while (node.ok()){
    geometry_msgs::TransformStamped transformStamped;
    try{
      transformStamped = tfBuffer.lookupTransform(tf_target_frame, tf_source_frame,
                               ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    geometry_msgs::PoseStamped poseMsg;

    poseMsg.header.stamp = transformStamped.header.stamp;
    poseMsg.header.frame_id = pose_frame_id;
    
    poseMsg.pose.position.x = transformStamped.transform.translation.x;
    poseMsg.pose.position.y = transformStamped.transform.translation.y;
    poseMsg.pose.position.z = transformStamped.transform.translation.z;

    poseMsg.pose.orientation.x = transformStamped.transform.rotation.x;
    poseMsg.pose.orientation.y = transformStamped.transform.rotation.y;
    poseMsg.pose.orientation.z = transformStamped.transform.rotation.z;
    poseMsg.pose.orientation.w = transformStamped.transform.rotation.w;

    posePub.publish(poseMsg);

    rate.sleep();
  }
  return 0;
};
