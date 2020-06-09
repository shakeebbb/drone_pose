#include "drone_pose.h"

// Main
int main(int argc, char **argv)
{
	ros::init(argc, argv, "drone_pose_node");
	ros::NodeHandle nh(ros::this_node::getName());
	
	drone_pose_class dronePose(&nh);
	
	ros::spin();
	return 0;
}

// ************************************************ //



