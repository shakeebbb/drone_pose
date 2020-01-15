#include "drone_pose.h"

// Main
int main(int argc, char **argv)
{
	ros::init(argc, argv, "drone_pose_node");
	ros::NodeHandle nh;
	
	drone_pose_class dronePose(&nh);

	// Publish Rate
	ros::Rate rate(250.0);

	while(ros::ok())
	{
		switch(dronePose.get_current_flight_mode())
		{
			case 'J' : // Joystick Mode
				dronePose.stop_traj_timer();
				dronePose.increment_setpoint(0.004, 
																		 0.004,
																		 0.004,
																		 0,
																		 0,
																		 0.004,
																		 false);
																		 
				dronePose.publish_current_setpoint(false);
				break;
			case 'H' : // Hold Mode
				dronePose.start_traj_timer(dronePose.get_current_sampling_time());

				dronePose.increment_setpoint(0.04, 
																		 0.04,
																		 0.04,
																		 0,
																		 0,
																		 0.04,
																		 false);
																		 
				//dronePose.update_pf_from_joy();

				break;
			case 'L' : // Land Mode
				dronePose.start_traj_timer(dronePose.get_current_sampling_time());
				break;
			case 'T' : // Trajectory Mode				
				dronePose.start_traj_timer(dronePose.get_current_sampling_time());
				break;
			case 'W' : // Waypoint Mode
				dronePose.start_traj_timer(dronePose.get_current_sampling_time());
				break;
			default :
				//timer.stop();
				ROS_WARN("Error! Flight Mode not recognized");
				break;
		}
	
		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}

// ************************************************ //



