#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <math.h>
#include <time.h>
#include <tf/transform_datatypes.h>
#include <drone_pose/trajectoryMsg.h>
#include <drone_pose/flightModeSrv.h>

using namespace std;

// Global publishers, subscribers, timers
ros::Publisher setpoint_pub;
ros::Timer timer;

// Trajectory variables (for mode 'T')
vector<geometry_msgs::PoseStamped> waypoints; 
int waypointPointer = 0;
float samplingTime = -1;

// Feedback variables
mavros_msgs::State current_state;
geometry_msgs::PoseStamped setpoint;
geometry_msgs::PoseStamped setpointRead;
geometry_msgs::Pose pose;

// Setup arming code bool
mavros_msgs::CommandBool arm_status;
mavros_msgs::SetMode offb_set_mode;
ros::ServiceClient arming_client;
ros::ServiceClient set_mode_client;

// Xbox joystick control
float xbox_pose[6] = {0,0,0,0,0,0};
int arm_button, disarm_button, flightMode1_button, flightMode2_button, x_axis, y_axis, z_axis, yaw_axis;

// Flight mode switch
char flightMode;

// Safety bounds
vector<float> x_bounds;
vector<float> y_bounds;
vector<float> z_bounds;

vector<float> takeoff_position;

// Topic names
string mavros_state_topic;
string joy_topic;
string odometry_topic;
string setpoints_topic;
string flight_mode_service;
string trajectory_topic;

// Functions declarations
void init();
bool isBounded(geometry_msgs::PoseStamped&);
void state_cb(const mavros_msgs::State::ConstPtr&);
void xbox_cb(const sensor_msgs::Joy&);
void odom_cb(const nav_msgs::Odometry&);
void setpoint_cb(const geometry_msgs::PoseStamped&);
void timer_cb(const ros::TimerEvent&);
void trajectory_cb(const drone_pose::trajectoryMsg&);
bool flightMode_cb(drone_pose::flightModeSrv::Request&,
									 drone_pose::flightModeSrv::Response&);


// Main
int main(int argc, char **argv)
{
	ros::init(argc, argv, "drone_pose");
	ros::NodeHandle nh;
	
	ros::param::get("drone_pose_node/x_bounds", x_bounds);
	ros::param::get("drone_pose_node/y_bounds", y_bounds);
	ros::param::get("drone_pose_node/z_bounds", z_bounds);
	
	ros::param::get("drone_pose_node/takeoff_position", takeoff_position);

	ros::param::get("drone_pose_node/arm_button", arm_button);
	ros::param::get("drone_pose_node/disarm_button", disarm_button);

	ros::param::get("drone_pose_node/flightMode1_button", flightMode1_button);
	ros::param::get("drone_pose_node/flightMode2_button", flightMode2_button);

	ros::param::get("drone_pose_node/x_axis", x_axis);
	ros::param::get("drone_pose_node/y_axis", y_axis);
	ros::param::get("drone_pose_node/z_axis", z_axis);
	ros::param::get("drone_pose_node/yaw_axis", yaw_axis);
	
	ros::param::get("drone_pose_node/mavros_state_topic", mavros_state_topic);
	ros::param::get("drone_pose_node/joy_topic", joy_topic);
	ros::param::get("drone_pose_node/odometry_topic", odometry_topic);
	ros::param::get("drone_pose_node/setpoints_topic", setpoints_topic);
	ros::param::get("drone_pose_node/flight_mode_service", flight_mode_service);
	ros::param::get("drone_pose_node/trajectory_topic", trajectory_topic);

	// Subscribers
	ros::Subscriber state_subscriber = nh.subscribe(mavros_state_topic, 10, state_cb);
	ros::Subscriber joystick = nh.subscribe(joy_topic, 1000, xbox_cb);
	ros::Subscriber odom_sub = nh.subscribe(odometry_topic, 1000, odom_cb);
	ros::Subscriber setpoint_sub = nh.subscribe(setpoints_topic, 10, setpoint_cb);
	ros::Subscriber trajectory_sub = nh.subscribe(trajectory_topic, 10, trajectory_cb);
	
	// Publishers
	setpoint_pub = nh.advertise<geometry_msgs::PoseStamped>(setpoints_topic, 10);
	
	//Service Client
	ros::ServiceServer flightMode_service = nh.advertiseService(flight_mode_service, flightMode_cb);
	
	arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
	set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
	
	//Timers
	timer = nh.createTimer(ros::Duration(1), timer_cb);
	timer.stop();

	// Publish Rate
	ros::Rate rate(250.0);

	// wait for PixRacer
//	while(ros::ok() && current_state.connected)
//	{
//		ros::spinOnce();
//		rate.sleep();
//	}

	init();

	while(ros::ok())
	{
	
		switch(flightMode)
		{
			case 'J' : // Joystick Mode
				timer.stop();
				
				setpoint.header.stamp = ros::Time::now();
    	
      	setpoint.pose.position.x += 0.004*xbox_pose[0];
      	setpoint.pose.position.y += 0.004*xbox_pose[1];
      	setpoint.pose.position.z += 0.004*xbox_pose[2];

				isBounded(setpoint);
				setpoint_pub.publish(setpoint);
				break;
			case 'H' : // Hold Mode
				timer.stop();
				
				setpoint = setpointRead;
				setpoint.header.stamp = ros::Time::now();

				isBounded(setpoint);	
				setpoint_pub.publish(setpoint);
				break;
			case 'T' : // Trajectory Mode				
				if (samplingTime == -1)
				{
					timer.stop();
					ROS_INFO("Waiting for Parameters ...");
					break;
				}
				
				timer.setPeriod(ros::Duration(samplingTime), false);
				timer.start();
				break;
			case 'S' : // Silent Mode
				timer.stop();
				break;
			default :
				timer.stop();
				cout << "Error! Mode not recognized" << endl;
				break;
		}
	
		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}

// ************************************************ //

void timer_cb(const ros::TimerEvent&)
	{
	if(flightMode != 'T')
		return;
	
	if(waypoints.empty())
	{
		flightMode = 'H';
		return;
	}
	
	cout << "Waypoint Array Size : " << waypoints.size() << endl;
	cout << "Waypoint Pointer Location : " << waypointPointer << endl;
	
	if(waypointPointer == waypoints.size())
	waypointPointer -= 1;
	
	geometry_msgs::PoseStamped waypoint = waypoints.at(waypointPointer);
	waypoint.header.stamp = ros::Time::now();
	
	setpoint_pub.publish(waypoint);
	
	waypointPointer += 1;
	
	cout << "Waypoint Pointer Location : " << waypointPointer << endl;
	}

// ************************************************ //

bool isBounded(geometry_msgs::PoseStamped& point)
{
	//float x_ref = setpoint.pose.position.x;
	//float y_ref = setpoint.pose.position.y;
	//float z_ref = setpoint.pose.position.z;

	if(point.pose.position.x < x_bounds[0])
	point.pose.position.x = x_bounds[0];
	if(point.pose.position.x > x_bounds[1])
	point.pose.position.x = x_bounds[1];

	if(point.pose.position.y < y_bounds[0])
	point.pose.position.y = y_bounds[0];
	if(point.pose.position.y > y_bounds[1])
	point.pose.position.y = y_bounds[1];

	if(point.pose.position.z < z_bounds[0])
	point.pose.position.z = z_bounds[0];
	if(point.pose.position.z > z_bounds[1])
	point.pose.position.z = z_bounds[1];
	return 0;
}

// ************************************************ //

void init() // Take-off Parameters
{
	// Create and set target position
	tf::Quaternion quad_orientation;
	quad_orientation.setRPY(0,0,0);	

	setpoint.header.stamp = ros::Time::now();

	setpoint.pose.position.x = takeoff_position[0];
	setpoint.pose.position.y = takeoff_position[1];
	setpoint.pose.position.z = takeoff_position[2];

	setpoint.pose.orientation.x = quad_orientation.x();
	setpoint.pose.orientation.y = quad_orientation.y();
	setpoint.pose.orientation.z = quad_orientation.z();
	setpoint.pose.orientation.w = quad_orientation.w();
	
	waypoints.clear();

	flightMode = 'J';
}

// ************************************************ //

void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
	current_state = *msg;
}

// ************************************************ //

void xbox_cb(const sensor_msgs::Joy& msg)
{
	if(msg.buttons[arm_button] == 1)
	{
		//ARM = X
		offb_set_mode.request.custom_mode = "OFFBOARD";
		arm_status.request.value = true;

		set_mode_client.call(offb_set_mode);
		arming_client.call(arm_status);

		ROS_INFO ("Armed");
	}

	if(msg.buttons[disarm_button] == 1)
	{
		//DISARM = B
		offb_set_mode.request.custom_mode = "STABILIZED";
		arm_status.request.value = false;

		set_mode_client.call(offb_set_mode);
		arming_client.call(arm_status);

		ROS_INFO ("DisArmed");
	}

	if(msg.buttons[flightMode1_button] == 1)
	{
		flightMode = 'J';

		setpoint.pose.position.x = setpointRead.pose.position.x;
		setpoint.pose.position.y = setpointRead.pose.position.y;
		setpoint.pose.position.z = setpointRead.pose.position.z;
		ROS_INFO("Mode : J");
	}

	if(msg.buttons[flightMode2_button] == 1)
	{
		flightMode = 'H';
		ROS_INFO("Mode : H");
 	}

	if(abs(msg.axes[x_axis])>0.2) //X
		xbox_pose[0] = msg.axes[x_axis];
	else
		xbox_pose[0] = 0;

	if(abs(msg.axes[y_axis])>0.2) //Y
		xbox_pose[1] = msg.axes[y_axis];
	else
		xbox_pose[1] = 0;

	if(abs(msg.axes[z_axis])>0.2)  //Z
		xbox_pose[2] = msg.axes[z_axis];
	else
		xbox_pose[2] = 0;

	if(abs(msg.axes[yaw_axis])>0.2)  //YAW
		xbox_pose[3] = msg.axes[yaw_axis];
	else
		xbox_pose[3] = 0;
}

// ************************************************ //

void odom_cb(const nav_msgs::Odometry& msg)
{
	pose = msg.pose.pose;
}

// ************************************************ //

void setpoint_cb(const geometry_msgs::PoseStamped& msg)
{
	setpointRead = msg;	
}

// ************************************************ //

bool flightMode_cb(drone_pose::flightModeSrv::Request& req,	
									 drone_pose::flightModeSrv::Response& res)
{
	if (req.setGet == 0 && flightMode != 'J')
	{
		flightMode = req.flightMode;
		res.flightMode = flightMode;
	}
	else
		res.flightMode = flightMode;
		
	ROS_INFO("Mode Modified To : %c",flightMode);
	
	return true;
}

// ************************************************ //

void trajectory_cb(const drone_pose::trajectoryMsg& msg)
{	
	if(msg.samplingTime != samplingTime)
	{
		samplingTime = msg.samplingTime;
		timer.setPeriod(ros::Duration(samplingTime), false);
	}
	
	if(msg.appendRefresh)
	{
		waypoints.clear();
		waypointPointer = 0;
	}
		
	if(msg.nPoints == 0)
		return;

	geometry_msgs::PoseStamped waypoint;
	for (int i = 0; i < msg.nPoints; i++)
	{
	waypoint.pose.position.x = msg.X[i];
	waypoint.pose.position.y = msg.Y[i];
	waypoint.pose.position.z = msg.Z[i];
	
	tf::Quaternion quad_orientation;
	quad_orientation.setRPY(0,0,0);	
	
	setpoint.pose.orientation.x = quad_orientation.x();
	setpoint.pose.orientation.y = quad_orientation.y();
	setpoint.pose.orientation.z = quad_orientation.z();
	setpoint.pose.orientation.w = quad_orientation.w();
	
	waypoints.push_back(waypoint);
	}
}
	/*
		waypoint.pose.position.x = msg.trajectory.data[msg.trajectory.layout.data_offset 
														 + msg.trajectory.layout.dim[1].stride*0 
														 + msg.trajectory.layout.dim[2].stride*j];
		waypoint.pose.position.y = msg.trajectory.data[msg.trajectory.layout.data_offset 
															 + msg.trajectory.layout.dim[1].stride*1 
															 + msg.trajectory.layout.dim[2].stride*j];
		waypoint.pose.position.z = msg.trajectory.data[msg.trajectory.layout.data_offset 
														 + msg.trajectory.layout.dim[1].stride*2 
														 + msg.trajectory.layout.dim[2].stride*j];
	  waypoints.push_back(waypoint);
	
	}
	
}
*/



