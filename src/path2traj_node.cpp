
#include "ros/ros.h"
#include "drone_pose/trajectoryMsg.h"
#include "drone_pose/flightModeSrv.h"
#include "nav_msgs/Path.h"

using namespace std;
	
void path_cb(const nav_msgs::Path::ConstPtr&);

drone_pose::trajectoryMsg trajMsg;
ros::Publisher trajPub;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "path2traj_node");
	ros::NodeHandle nh;
	
	trajPub = nh.advertise<drone_pose::trajectoryMsg>("trajectory", 10);
	ros::Subscriber pathSub = nh.subscribe<nav_msgs::Path>("path", 10, path_cb);
	ros::ServiceClient flightModeClient = nh.serviceClient<drone_pose::flightModeSrv>("flight_mode");
	
	drone_pose::flightModeSrv flightModeSrv;
	flightModeSrv.request.setGet = 0;
	flightModeSrv.request.flightMode = 'W';
	
	while(flightModeSrv.response.flightMode != 'W')
	{
	flightModeClient.call(flightModeSrv);
	ROS_WARN("sample_traj_node: Could not change flight mode. Trying again ...");
	}
	
	ROS_INFO("sample_traj_node: Change of flight mode successful");
	
	ros::spin();
	return 0;
}
	
void path_cb(const nav_msgs::Path::ConstPtr& msg)
{
	trajMsg.poses.clear();
	trajMsg.appendRefresh = true;
	trajMsg.samplingTime = 0.2;
	
	for(int i = 0; i < msg->poses.size(), i++; )
	trajMsg.poses.push_back(msg->poses[i].pose);
	
	trajPub.publish(trajMsg);
}	

