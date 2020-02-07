
#include "drone_pose.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PointStamped.h"

using namespace std;
	
// Global Variables
float carrotDistance = 1.5;

ros::Publisher trajPub;
ros::ServiceClient flightModeClient;

// Function Declarations
void path_cb(const nav_msgs::Path::ConstPtr&);
void localGoal_cb(const geometry_msgs::PointStamped&);

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
	
	ros::Subscriber pathSub = nh.subscribe("path", 10, path_cb);
	ros::Subscriber localGoalSub = nh.subscribe("local_goal", 10, localGoal_cb);
	flightModeClient = nh.serviceClient<drone_pose::flightModeSrv>("/B01/drone_pose/flight_mode_service");
	
	ros::spin();
	return 0;
}

// ************************************************************
void localGoal_cb(const geometry_msgs::PointStamped& msg)
{
	drone_pose::flightModeSrv flightModeSrv;
	flightModeSrv.request.setGet = 0;
	flightModeSrv.request.flightMode = 'W';
	
	flightModeClient.call(flightModeSrv);
		
	if(flightModeSrv.response.flightMode != 'W')
	{		
		ROS_WARN_THROTTLE(1, "path2traj_node: Could not change flight mode. Trying again ...");
		return;
	}
	
	drone_pose::trajectoryMsg trajMsg;
	
	trajMsg.poses.clear();
	trajMsg.appendRefresh = true;
	trajMsg.samplingTime = 0.1;
	trajMsg.header.stamp = ros::Time::now();
	trajMsg.header.frame_id = msg.header.frame_id;
	
	geometry_msgs::Pose localPose;
	localPose.position.x = msg.point.x;
	localPose.position.y = msg.point.y;
	localPose.position.z = msg.point.z;
	localPose.orientation.w = 1;

	trajMsg.poses.push_back(localPose);
	
	trajPub.publish(trajMsg);
}
	
// ************************************************************
void path_cb(const nav_msgs::Path::ConstPtr& msg)
{
	drone_pose::trajectoryMsg trajMsg;
	
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
	trajPub.publish(trajMsg);
}	

