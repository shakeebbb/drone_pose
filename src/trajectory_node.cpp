#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/PoseStamped.h"
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include "sensor_msgs/Joy.h"
#include "math.h"
#include <time.h>
#include <fstream>
#include <iostream>


using namespace std;

// Globals
mavros_msgs::State current_state;
geometry_msgs::PoseStamped target_pose;
//setup arming code bool
mavros_msgs::CommandBool arm_status;
mavros_msgs::SetMode offb_set_mode;
//xbox joystick control
bool xbox_control = true;
float xbox_pose[6] = {0,0,0,0,0,0};
bool xbox_flag = false;
//Flight mode switch
int FlightMode = 1;
//ros time values
double begin_time = 0;
double duration = 0;
double time_stamp = 0;
double start_time = 0;

float k = 1.0;

//coefficients setup
double x_coeff1, x_coeff2, x_coeff3, x_coeff4;
double y_coeff1, y_coeff2, y_coeff3, y_coeff4;
double z_coeff1, z_coeff2, z_coeff3, z_coeff4;
int exec = 0;
float quad_x, quad_y, quad_z;

void init();
void state_cb(const mavros_msgs::State::ConstPtr& msg);
void xbox_cb(const sensor_msgs::Joy& msg);
void traj_coeff(const geometry_msgs::PoseWithCovarianceStamped& msg);
void pose_cb(const geometry_msgs::PoseStamped& msg);

//Service CLien


// Main
int main(int argc, char **argv){

	init();
	ros::init(argc, argv, "trajectory");
	ros::NodeHandle nh;

	// Subscribers
	ros::Subscriber state_subscriber = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
	ros::Subscriber joystick = nh.subscribe("/joy", 1000, xbox_cb);
	ros::Subscriber traj_coeff_subs = nh.subscribe("/trajectory_coefficients", 10, traj_coeff);
	ros::Subscriber quad_pose = nh.subscribe("/quad2_pose", 1000, pose_cb);

	// Publishers
	ros::Publisher pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);

	//Service CLient
	ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
	ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

	//open file
  	ofstream myfile;
  	//myfile.open ("/home/odroid/Desktop/RecordFile/Demo.txt", ios::trunc);

	//begin timer
	start_time = ros::Time::now().toSec(); 
	
	// Publish Rate
	ros::Rate rate(400.0);

	// wait for PixRacer
	while(ros::ok() && current_state.connected){
		ros::spinOnce();
		rate.sleep();
	}

	while(ros::ok()){

		if(xbox_flag){
			set_mode_client.call(offb_set_mode);
			arming_client.call(arm_status);
			xbox_flag = false;
		}

		// Publish the desired position
		pose_publisher.publish(target_pose);
		time_stamp = ros::Time::now().toSec(); 

				
		//xbox_control
		if(FlightMode == 1){
			target_pose.pose.position.x -= 0.01*xbox_pose[0];
			target_pose.pose.position.y += 0.01*xbox_pose[1];
			target_pose.pose.position.z += 0.002*xbox_pose[2];
		}
		
		//trajectory
		if(FlightMode == 2){/*
			if(exec == 0){
				target_pose.pose.position.x = 1;
				target_pose.pose.position.y = -1.3;
				target_pose.pose.position.z = 1.15;
//				ROS_INFO("exe = 0");
			}
			if(exec == 1){
				duration = ros::Time::now().toSec() - begin_time;
				target_pose.pose.position.x = x_coeff1*duration*duration*duration+x_coeff2*duration*duration+x_coeff3*duration+x_coeff4;
				target_pose.pose.position.y = y_coeff1*duration*duration*duration+y_coeff2*duration*duration+y_coeff3*duration+y_coeff4;
				target_pose.pose.position.z = z_coeff1*duration*duration*duration+z_coeff2*duration*duration+z_coeff3*duration+z_coeff4;
//                              ROS_INFO("exe = 1");
				//write into file
				myfile << fixed << setprecision(4) << time_stamp-start_time << "\t" << quad_x << "\t" << quad_y << "\t" << quad_z << endl;
			}
			if(exec == 2){
				target_pose.pose.position.x = x_coeff4;
				target_pose.pose.position.y = y_coeff4;
     				target_pose.pose.position.z = z_coeff4;
//                              ROS_INFO("exe = 2");
			}*/
			target_pose.pose.position.x = sin(k*(time_stamp - start_time));
			target_pose.pose.position.y = sin(0.5*k*(time_stamp - start_time));
			target_pose.pose.position.z = 1;//+0.2*sin(0.25*k*(time_stamp - start_time));
			
			//myfile << fixed << setprecision(4) << time_stamp-begin_time << "\t" << quad_x << "\t" << quad_y << "\t" << quad_z << "\t" << target_pose.pose.position.x << "\t" << target_pose.pose.position.y << "\t" << target_pose.pose.position.z << "\t" << k << endl;
		}	
		
		ros::spinOnce();
		rate.sleep();
	}
	//myfile.close();
	return 0;
}

void init(){
	// Create and set target position
	target_pose.pose.position.x = 0; //temorary offset for flying two at once
	target_pose.pose.position.y = 0;
	target_pose.pose.position.z = 0.7;
	target_pose.pose.orientation.x = 0;
	target_pose.pose.orientation.y = 0;
	target_pose.pose.orientation.z = 0;
	target_pose.pose.orientation.w = 1;
}


// State callback
void state_cb(const mavros_msgs::State::ConstPtr& msg){
	current_state = *msg;
}

//xbox callback
void xbox_cb(const sensor_msgs::Joy& msg){	
	//

	if(msg.buttons[2] == 1){
	//ARM = X
	xbox_flag = true;
	offb_set_mode.request.custom_mode = "OFFBOARD";
	arm_status.request.value = true;
	}
	if(msg.buttons[3] == 1){
	//DISARM = B
	xbox_flag = true;
	offb_set_mode.request.custom_mode = "STABILIZED";
	arm_status.request.value = false;
	}
	if(msg.buttons[0] == 1){
		xbox_flag = true;
		FlightMode = 1;
		ROS_INFO("Joystick Control");
	}
	if(msg.buttons[1] == 1){
		xbox_flag = true;
		FlightMode = 2;
		begin_time = ros::Time::now().toSec();
		ROS_INFO("trajectory");
  	}
	if(msg.buttons[5] == 1){
		k += 0.5;
	}

	if(abs(msg.axes[3])>0.1) //X
		xbox_pose[0] = msg.axes[3];
	else
		xbox_pose[0] = 0;
	if(abs(msg.axes[4])>0.1) //Y
		xbox_pose[1] = msg.axes[4];
	else
		xbox_pose[1] = 0;
	if(abs(msg.axes[1])>0.1)  //Z
		xbox_pose[2] = msg.axes[1];
	else
		xbox_pose[2] = 0;
	if(abs(msg.axes[1])>0.1)  //YAW
		xbox_pose[3] = msg.axes[1];
	else
		xbox_pose[3] = 0;
}

void traj_coeff(const geometry_msgs::PoseWithCovarianceStamped& msg){
	exec = msg.pose.covariance[0];
	ROS_INFO("exec = %i", exec);
	ROS_INFO("Here");
	x_coeff1 = msg.pose.covariance[1];
	x_coeff2 = msg.pose.covariance[2];
	x_coeff3 = msg.pose.covariance[3];
	x_coeff4 = msg.pose.covariance[4];
	y_coeff1 = msg.pose.covariance[5];
	y_coeff2 = msg.pose.covariance[6];
	y_coeff3 = msg.pose.covariance[7];
	y_coeff4 = msg.pose.covariance[8];
	z_coeff1 = msg.pose.covariance[9];
	z_coeff2 = msg.pose.covariance[10];
	z_coeff3 = msg.pose.covariance[11];
	z_coeff4 = msg.pose.covariance[12];
	
	//begin timer
	begin_time = ros::Time::now().toSec(); 
}

void pose_cb(const geometry_msgs::PoseStamped& msg){
	quad_x = msg.pose.position.x;
	quad_y = msg.pose.position.y;
	quad_z = msg.pose.position.z;

}
