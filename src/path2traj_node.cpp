
#include "drone_pose.h"
#include "nav_msgs/Path.h"
#include "tf2_ros/transform_listener.h"

using namespace std;
	
// Global Variables
float carrotDistance = 1.5;

drone_pose::trajectoryMsg trajMsg;
ros::Publisher trajPub;
ros::Publisher localGoalPub;

// Function Declarations
void path_cb(const nav_msgs::Path::ConstPtr&);

// Main
int main(int argc, char **argv)
{
	ros::init(argc, argv, "path2traj_node");
	ros::NodeHandle nh;
	
	while(!nh.getParam("path2traj_node/carrot_distance", carrotDistance))
	{
		ROS_WARN("Waiting for carrot_distance parameter");
		sleep(1);
	}
	
	trajPub = nh.advertise<drone_pose::trajectoryMsg>("traj", 10);
	localGoalPub = nh.advertise<geometry_msgs::Pose>("generated_local_goal", 10);
	
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
	
	cout << "Path Received: " << msg->poses.size() << endl;
	int itr;
	for(itr = 0; itr < (msg->poses.size()-1); itr++)
	{
		pathDistance += drone_pose_class::pose_distance(msg->poses[itr].pose, msg->poses[itr+1].pose, "position");
		
		if(pathDistance >= carrotDistance)
		break;
	}
	
	trajMsg.poses.push_back(msg->poses[itr].pose);
	localGoalPub.publish(msg->poses[itr].pose);
	trajPub.publish(trajMsg);
}	

