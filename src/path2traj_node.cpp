
#include "ros/ros.h"
#include "drone_pose/trajectoryMsg.h"
#include "drone_pose/flightModeSrv.h"
#include "nav_msgs/Path.h"
#include "tf2/LinearMath/Transform.h"
#include "tf/transform_datatypes.h"

using namespace std;
	
float carrotDistance;

void path_cb(const nav_msgs::Path::ConstPtr&);
float pose_distance(geometry_msgs::Pose, geometry_msgs::Pose, string = "all");

drone_pose::trajectoryMsg trajMsg;
ros::Publisher trajPub;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "path2traj_node");
	ros::NodeHandle nh;
	
	while(!nh.getParam("path2traj_node/carrot_distance", carrotDistance));
	
	trajPub = nh.advertise<drone_pose::trajectoryMsg>("trajectory", 10);
	ros::Subscriber pathSub = nh.subscribe<nav_msgs::Path>("path", 10, path_cb);
	ros::ServiceClient flightModeClient = nh.serviceClient<drone_pose::flightModeSrv>("flight_mode");
	
	drone_pose::flightModeSrv flightModeSrv;
	flightModeSrv.request.setGet = 0;
	flightModeSrv.request.flightMode = 'W';
	
	while(flightModeSrv.response.flightMode != 'W')
	{
	flightModeClient.call(flightModeSrv);
	ROS_WARN("path2traj_node: Could not change flight mode. Trying again ...");
	sleep(1.5);
	}
	
	ROS_INFO("path2traj_node: Change of flight mode successful");
	
	ros::spin();
	return 0;
}
	
void path_cb(const nav_msgs::Path::ConstPtr& msg)
{
	trajMsg.poses.clear();
	trajMsg.appendRefresh = true;
	trajMsg.samplingTime = 0.1;
	
	float pathDistance = 0;
	
	int itr;
	for(itr = 0; itr < (msg->poses.size()-1); itr++)
	{
		pathDistance += pose_distance(msg->poses[itr].pose, msg->poses[itr+1].pose, "position");
		
		if(pathDistance >= carrotDistance)
		break;
	}
	
	trajMsg.poses.push_back(msg->poses[itr].pose);
	
	trajPub.publish(trajMsg);
}	

float pose_distance(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2, string field)
{
	float x_err = pose2.position.x - pose1.position.x;
	float y_err = pose2.position.y - pose1.position.y;
	float z_err = pose2.position.z - pose1.position.z;
	
	tf::Quaternion quadOrientation1(pose1.orientation.x,
  															 pose1.orientation.y,
  															 pose1.orientation.z,
  															 pose1.orientation.w);
  															 
  tf::Quaternion quadOrientation2(pose2.orientation.x,
  															 pose2.orientation.y,
  															 pose2.orientation.z,
  															 pose2.orientation.w);
  															 
  double currentRoll1, currentPitch1, currentYaw1;
	tf::Matrix3x3(quadOrientation1).getRPY(currentRoll1, currentPitch1, currentYaw1);
	
	double currentRoll2, currentPitch2, currentYaw2;
	tf::Matrix3x3(quadOrientation2).getRPY(currentRoll2, currentPitch2, currentYaw2);
	
	float yaw_err = currentYaw2 - currentYaw1;
	
	if (field == "heading")
	return abs(yaw_err);
	
	if (field == "position")
	return sqrt(pow(x_err,2) + pow(y_err,2) + pow(z_err,2));
	
	return sqrt(pow(x_err,2) + pow(y_err,2) + pow(z_err,2) + pow(yaw_err,2));
}
