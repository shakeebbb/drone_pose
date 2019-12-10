#include "ros/ros.h"
#include "drone_pose/trajectoryMsg.h"
#include "drone_pose/flightModeSrv.h"

using namespace std;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "sample_traj_node");
	ros::NodeHandle nh;
	
	ros::Publisher pathPub = nh.advertise<drone_pose::trajectoryMsg>("/B01/trajectory", 10);
	ros::ServiceClient flightModeClient = nh.serviceClient<drone_pose::flightModeSrv>("/B01/flight_mode");
	
	drone_pose::flightModeSrv flightModeSrv;
	flightModeSrv.request.setGet = 0;
	flightModeSrv.request.flightMode = 'W';
	
	while(flightModeSrv.response.flightMode != 'W')
	{
	flightModeClient.call(flightModeSrv);
	ROS_WARN("sample_traj_node: Could not change flight mode. Trying again ...");
	}
	
	ROS_INFO("sample_traj_node: Change of flight mode successful");

	drone_pose::trajectoryMsg pathMsg;

	pathMsg.appendRefresh = false;
	pathMsg.samplingTime = 0.2;
	geometry_msgs::Pose pose;
	
		pose.position.x = 0; pose.position.y = 0; pose.position.z = 1;
		pose.orientation.x = 0; pose.orientation.y = 0; pose.orientation.z = 0; pose.orientation.w = 1;
		pathMsg.poses.push_back(pose);
		
		pose.position.x = 0; pose.position.y = 1.5; pose.position.z = 1;
		pose.orientation.x = 0; pose.orientation.y = 0; pose.orientation.z = 0; pose.orientation.w = 1;
		pathMsg.poses.push_back(pose);
		
		pose.position.x = 1.5; pose.position.y = 1.5; pose.position.z = 1;
		pose.orientation.x = 0; pose.orientation.y = 0; pose.orientation.z = 0; pose.orientation.w = 1;
		pathMsg.poses.push_back(pose);
		
		pose.position.x = 1.5; pose.position.y = -1.5; pose.position.z = 1;
		pose.orientation.x = 0; pose.orientation.y = 0; pose.orientation.z = 0; pose.orientation.w = 1;
		pathMsg.poses.push_back(pose);
		
		pose.position.x = 0; pose.position.y = -1.5; pose.position.z = 1;
		pose.orientation.x = 0; pose.orientation.y = 0; pose.orientation.z = 0; pose.orientation.w = 1;
		pathMsg.poses.push_back(pose);
		
		pose.position.x = 0; pose.position.y = 0; pose.position.z = 1;
		pose.orientation.x = 0; pose.orientation.y = 0; pose.orientation.z = 0; pose.orientation.w = 1;
		pathMsg.poses.push_back(pose);
		
		pathPub.publish(pathMsg);
		
		pathMsg.poses.clear();
		pose.position.x = -1.5; pose.position.y = 1.5; pose.position.z = 1.5;
		pose.orientation.x = 0; pose.orientation.y = 0; pose.orientation.z = 0; pose.orientation.w = 1;
		pathMsg.poses.push_back(pose);
		
		pose.position.x = -1.5; pose.position.y = 1.5; pose.position.z = 1.5;
		pose.orientation.x = 0; pose.orientation.y = 0; pose.orientation.z = 1; pose.orientation.w = 0;
		pathMsg.poses.push_back(pose);
		
		pose.position.x = 0; pose.position.y = 0; pose.position.z = 1;
		pose.orientation.x = 0; pose.orientation.y = 0; pose.orientation.z = 1; pose.orientation.w = 0;
		pathMsg.poses.push_back(pose);
		
		pathPub.publish(pathMsg);
		

return 0;
} 


