#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int32.h>
#include <math.h>
#include <time.h>
#include <tf/transform_datatypes.h>

using namespace std;

// Globals
mavros_msgs::State current_state;
geometry_msgs::PoseStamped setpoint;
geometry_msgs::PoseStamped setpointRead;
geometry_msgs::Pose pose;

//setup arming code bool
mavros_msgs::CommandBool arm_status;
mavros_msgs::SetMode offb_set_mode;
ros::ServiceClient arming_client;
ros::ServiceClient set_mode_client;

//xbox joystick control
float xbox_pose[6] = {0,0,0,0,0,0};

//Flight mode switch
std_msgs::Int32 flightMode;

vector<float> x_bounds;
vector<float> y_bounds;
vector<float> z_bounds;

vector<float> takeoff_position;

void init();
bool isBounded(geometry_msgs::PoseStamped&);
void state_cb(const mavros_msgs::State::ConstPtr&);
void xbox_cb(const sensor_msgs::Joy&);
void odom_cb(const nav_msgs::Odometry&);
void setpoint_cb(const geometry_msgs::PoseStamped&);

// Main
int main(int argc, char **argv)
	{
	ros::init(argc, argv, "drone_pose");
	ros::NodeHandle nh;
	
	ros::param::get("drone_pose_node/x_bounds", x_bounds);
	ros::param::get("drone_pose_node/y_bounds", y_bounds);
	ros::param::get("drone_pose_node/z_bounds", z_bounds);
	
	ros::param::get("drone_pose_node/takeoff_position", takeoff_position);

	// Subscribers
	ros::Subscriber state_subscriber = nh.subscribe("mavros/state", 10, state_cb);
	ros::Subscriber joystick = nh.subscribe("/joy", 1000, xbox_cb);
	ros::Subscriber odom_sub = nh.subscribe("mavros/local_position/odom", 1000, odom_cb);
	ros::Subscriber setpoint_sub = nh.subscribe("mavros/setpoint_position/local", 10, setpoint_cb);
	
	// Publishers
	ros::Publisher setpoint_publisher = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
	ros::Publisher flightMode_publisher = nh.advertise<std_msgs::Int32>("flight_mode", 10);
	
	//Service Client
	arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
	set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

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
		flightMode_publisher.publish(flightMode);
		if(flightMode.data == 1) // Joystick Mode
		{
        		setpoint.header.stamp = ros::Time::now();
                        setpoint.pose.position.x += 0.004*(xbox_pose[0]);
                        setpoint.pose.position.y += 0.004*xbox_pose[1];
                        setpoint.pose.position.z += 0.004*xbox_pose[2];

			isBounded(setpoint);
			setpoint_publisher.publish(setpoint);
		}
		if(flightMode.data == 2) // Autonomous Mode
		{

		}

		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}

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

	flightMode.data = 1;
}

// State callback
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
	current_state = *msg;
}

//xbox callback
void xbox_cb(const sensor_msgs::Joy& msg)
{
	if(msg.buttons[2] == 1)
	{
	//ARM = X
	offb_set_mode.request.custom_mode = "OFFBOARD";
	arm_status.request.value = true;

	set_mode_client.call(offb_set_mode);
	arming_client.call(arm_status);

	ROS_INFO ("Armed");
	}

	if(msg.buttons[3] == 1)
	{
	//DISARM = B
	offb_set_mode.request.custom_mode = "STABILIZED";
	arm_status.request.value = false;

	set_mode_client.call(offb_set_mode);
	arming_client.call(arm_status);

	ROS_INFO ("DisArmed");
	}

	if(msg.buttons[0] == 1)
	{
	flightMode.data = 1;

	setpoint.pose.position.x = setpointRead.pose.position.x;
        setpoint.pose.position.y = setpointRead.pose.position.y;
        setpoint.pose.position.z = setpointRead.pose.position.z;
	ROS_INFO("Joystick Control");
	}

	if(msg.buttons[1] == 1)
	{
	flightMode.data = 2;
	ROS_INFO("Autonomous Control");
  	}

	if(abs(msg.axes[4])>0.2) //X
		xbox_pose[0] = msg.axes[4];
	else
		xbox_pose[0] = 0;

	if(abs(msg.axes[3])>0.2) //Y
		xbox_pose[1] = msg.axes[3];
	else
		xbox_pose[1] = 0;

	if(abs(msg.axes[1])>0.2)  //Z
		xbox_pose[2] = msg.axes[1];
	else
		xbox_pose[2] = 0;

	if(abs(msg.axes[1])>0.2)  //YAW
		xbox_pose[3] = msg.axes[1];
	else
		xbox_pose[3] = 0;
}

void odom_cb(const nav_msgs::Odometry& msg)
{
	pose = msg.pose.pose;
}
void setpoint_cb(const geometry_msgs::PoseStamped& msg)
{
	setpointRead = msg;	
}


