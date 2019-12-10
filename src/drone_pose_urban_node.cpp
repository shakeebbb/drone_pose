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

class drone_pose
{

private:
	// Subscribers
	ros::Subscriber stateSub;
	ros::Subscriber joystick;
	ros::Subscriber odomSub;
	ros::Subscriber setpointSub;
	ros::Subscriber trajectorySub;
	
	// Publishers
	ros::Publisher setpointPub;
	
	// Servers
	ros::ServiceClient flightModeServer;
	
	// Clients
	ros::ServiceClient armingClient;
	ros::ServiceClient setModeClient;
	
	// Timers
	ros::Timer trajTimer;
	
	// Mission variables
	nav_msgs::Odometry currentOdom;
	geometry_msgs::Pose currentSetpoint;
	int currentWaypointId;
	geometry_msgs::PoseArray currentWaypointList;
	float currentSamplingTime;
	char currentFlightMode;
	
	// Status variables
	mavros_msgs::State currentStateMavros;
	float joystickVal[4]; //x,y,z,yaw
	
	// From Parameter Server
	float xBounds[2];
	float yBounds[2];
	float zBounds[2];
	float takeoffPosition[3];
	int armButton;
	int disarmButton;
	int aButton;
	int bButton;
	int xAxis;
	int yAxis;
	int zAxis;
	int yawAxis;
	
public:

	drone_pose(ros::NodeHandle *nh)
	{
		wait_for_params(nh);
		
		// Subscribers
		stateMavrosSub = nh->subscribe("mavros_state_topic", 10, &drone_pose::state_mavros_cb, this);
		joySub = nh->subscribe("joy_topic", 100, &drone_pose::joy_cb, this);
		odomSub = nh->subscribe("odometry_topic", 100, &drone_pose::odom_cb, this);
		setpointSub = nh->subscribe("setpoint_topic", 10, &drone_pose::setpoint_cb, this);
		trajectorySub = nh->subscribe("trajectory_topic", 10, &drone_pose::trajectory_cb, this);
		
		// Publishers
		setpointPub = nh->advertise<geometry_msgs::PoseStamped>("setpoint_topic", 10);
		
		// Servers
		flightModeServer = nh->advertiseService(flight_mode_service, &drone_pose::flightMode_cb, this);
		
		// Clients
		armingClient = nh->serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
		setModeClient = nh->serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
		
		//Timers
		trajTimer = nh.createTimer(ros::Duration(1), &drone_pose::traj_timer_cb, this);
		trajTimer.stop();
		
		init();
	}
	
	// Callbacks
	void state_mavros_cb(const mavros_msgs::State&);
	void joy_cb(const sensor_msgs::Joy&);
	void odom_cb(const nav_msgs::Odometry&);
	void traj_timer_cb(const ros::TimerEvent&);
	void trajectory_cb(const drone_pose::trajectoryMsg&);
	bool flightMode_cb(drone_pose::flightModeSrv::Request&,
										 drone_pose::flightModeSrv::Response&);
									 
	// Other cpp functions
	void wait_for_params(ros::NodeHandle*);
	void init();
	bool isBounded(geometry_msgs::PoseStamped&);
	
};


//////////////////////////////////////////////////////////////////////

void drone_pose::joy_cb(const sensor_msgs::Joy& msg)
{
	mavros_msgs::SetMode setMode;
	mavros_msgs::CommandBool armingCommand;
	
	if(msg.buttons[armButton] == 1) //ARM = X
	{
		setMode.request.custom_mode = "OFFBOARD";
		armingCommand.request.value = true;

		if(setModeClient.call(setMode))
		ROS_INFO("OFFBOARD: Service call successful");
		else
		{
		ROS_WARN("Could not set offboard mode: Service call unsuccessful");
		return;
		}
		
		if(armingClient.call(armingCommand))
		ROS_INFO("ARMED: Service call successful");
		else
		{
		ROS_WARN("Could not arm the vehicle: Service call unsuccessful");
		return;
		}
	}

	if(msg.buttons[disarmButton] == 1) //DISARM = Y
	{
		setMode.request.custom_mode = "STABILIZED";
		armingCommand.request.value = false;
		
		while(!armingClient.call(armingCommand))
		ROS_WARN("Could not disarm the vehicle: Service call unsuccessful, Trying again ...");
		
		ROS_INFO("DIS-ARMED: Service call successful");

		if(setModeClient.call(setMode))
		ROS_INFO("STABILIZED: Service call successful");
		else
		{
		ROS_WARN("Could not set stabilized mode: Service call unsuccessful");
		return;
		}
	}

	if(msg.buttons[aButton] == 1)
	{
		flightMode = 'J';

		currentSetpoint.header.stamp = ros::Time::now();
		
		currentSetpoint.pose = currentOdom.pose.pose;		
		
		ROS_INFO("Mode : J");
		ROS_INFO("Setpoint set to current pose");
	}

	if(msg.buttons[bButton] == 1)
	{
		flightMode = 'H';
		ROS_INFO("Mode : H");
 	}

	if(abs(msg.axes[xAxis])>0.2) //X
		joystickVal[0] = msg.axes[xAxis];
	else
		joystickVal[0] = 0;

	if(abs(msg.axes[yAxis])>0.2) //Y
		joystickVal[1] = msg.axes[yAxis];
	else
		joystickVal[1] = 0;

	if(abs(msg.axes[zAxis])>0.2)  //Z
		joystickVal[2] = msg.axes[zAxis];
	else
		joystickVal[2] = 0;

	if(abs(msg.axes[yawAxis])>0.2)  //YAW
		joystickVal[3] = msg.axes[yawAxis];
	else
		joystickVal[3] = 0;
}

// ********************************************************************
void drone_pose::traj_timer_cb(const ros::TimerEvent&)
{
	if(!(currentFlightMode == 'T' || currentFlightMode == 'W'))
		return;
	
	if(currentWaypointList.poses.empty())
	currentFlightMode = 'H';
	
	if (currentFlightMode == 'T')
	{
		cout << "Waypoint Array Size : " << currentWaypointList.poses.size() << endl;
		
		if(currentWaypointId == currentWaypointList.poses.size())
		currentWaypointId -= 1;
		
		cout << "Publishing waypoint " << currentWaypointId << endl;
		
		currentSetpoint.header.stamp = ros::Time::now();
		currentSetpoint.pose = currentWaypointList.poses.at(currentWaypointId);
		
		setpointPub.publish(currentSetpoint);
		
		currentWaypointId += 1;
		
		cout << "Next waypoint in queue " << currentWaypointId << endl;
		return;
	}
	
	if (currentFlightMode == 'W')
	{
		cout << "Waypoint Array Size : " << currentWaypointList.poses.size() << endl;
		
		if(currentWaypointId == currentWaypointList.poses.size())
		currentWaypointId -= 1;
		
		cout << "Publishing waypoint " << currentWaypointId << endl;
		
		currentSetpoint.header.stamp = ros::Time::now();
		currentSetpoint.pose = currentWaypointList.poses.at(currentWaypointId);
		
		setpointPub.publish(currentSetpoint);
		
		if(distance(currentSetpoint, currentPose) < successRadius)
		currentWaypointId += 1;
		
		cout << "Next waypoint in queue " << currentWaypointId << endl;
		return;
	}
	
	if(currentFlightMode == 'H')
	publish_current_waypoint();	
	
}

// *********************************************************************
void drone_pose::wait_for_params(ros::NodeHandle *nh)
{
	while(!nh->getParam("drone_pose_node/x_bounds", xBounds));
	while(!nh->getParam("drone_pose_node/y_bounds", yBounds));
	while(!nh->getParam("drone_pose_node/z_bounds", zBounds));
	
	while(!nh->getParam("drone_pose_node/takeoff_position", takeoffPosition));

	while(!nh->getParam("drone_pose_node/arm_button", armButton));
	while(!nh->getParam("drone_pose_node/disarm_button", disarmButton));

	while(!nh->getParam("drone_pose_node/A_button", aButton));
	while(!nh->getParam("drone_pose_node/B_button", bButton));

	while(!nh->getParam("drone_pose_node/x_axis", xAxis));
	while(!nh->getParam("drone_pose_node/y_axis", yAxis));
	while(!nh->getParam("drone_pose_node/z_axis", zAxis));
	while(!nh->getParam("drone_pose_node/yaw_axis", yawAxis));
	
	ROS_INFO("Parameters for drone_pose retreived from the parameter server");
}

//*****************************************************************
void drone_pose::init() 
{
	// Create and set target position
	tf::Quaternion quadOrientation;
	quadOrientation.setRPY(0,0,0);	

	currentSetpoint.header.stamp = ros::Time::now();

	currentSetpoint.pose.position.x = takeoffPosition[0];
	currentSetpoint.pose.position.y = takeoffPosition[1];
	currentSetpoint.pose.position.z = takeoffPosition[2];

	currentSetpoint.pose.orientation.x = quadOrientation.x();
	currentSetpoint.pose.orientation.y = quadOrientation.y();
	currentSetpoint.pose.orientation.z = quadOrientation.z();
	currentSetpoint.pose.orientation.w = quadOrientation.w();
	
	currentWaypointList.poses.clear();

	currentFlightMode = 'J';
	
	ROS_INFO("Initialized drone_pose");
	
	ROS_INFO("Waiting for mavros to connect ...");
}

// *******************************************************************
bool drone_pose::isBounded(geometry_msgs::PoseStamped& point)
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

// ***********************************************************************

void drone_pose::trajectory_cb(const drone_pose::trajectoryMsg& msg)
{	
	if(msg.samplingTime != currentSamplingTime && msg.samplingTime > 0)
	{
		set_current_sampling_time(msg.samplingTime);
		start_traj_timer();
	}
	else if(msg.samplingTime != currentSamplingTime && msg.samplingTime <= 0)
	{
		ROS_WARN("Trajectory received with invalid sampling time %f", msg.samplingTime);
		return;
	} 
	
	if(msg.appendRefresh)
	{
		currentWaypointList.poses.clear();
		currentWaypointId = 0;
	}
		
	if(msg.nPoints == 0)
		return;
	
	for (int i = 0; i < msg.nPoints; i++)	
	currentWaypointList.poses.push_back(msg.poses[i]);
}

// ***********************************************************************
bool drone_pose::flightMode_cb(drone_pose::flightModeSrv::Request& req,	
									 drone_pose::flightModeSrv::Response& res)
{
	if (req.setGet == 0 && currentFlightMode != 'J')
	{
		if (req.flightMode == 'H' || req.flightMode == 'W' || req.flightMode == 'T')
		{
			currentFlightMode = req.flightMode;
			res.flightMode = currentFlightMode;
			set_current_sampling_time(5);
			start_traj_timer();
		}
		else
		ROS_WARN("Unrecognized mode transition requested");
	}
	res.flightMode = currentFlightMode;
		
	ROS_INFO("Mode Modified To : %c",flightMode);
	
	return true;
}


// ***********************************************************************

void drone_pose::odom_cb(const nav_msgs::Odometry& msg)
{
	currentOdom = msg;
}

// ***********************************************************************
void drone_pose::state_mavros_cb(const mavros_msgs::State& msg)
{
	currentStateMavros = msg;
	
	if(msg.connected)
	ROS_INFO("Mavros connected");
	else
	ROS_WARN("Still waiting for mavros to connect ...");
}

// ***********************************************************************
char get_current_flight_mode()
{
	return currentFlightMode;
}

// ***********************************************************************
geometry_msgs::PoseStamped get_current_setpoint()
{
	return currentSetpoint;
}

// ***********************************************************************
float get_current_sampling_time();
{
	return currentSamplingTime;
}

// ***********************************************************************
void increment_setpoint(float dx, float dy, float dz, float droll, float dpitch, float dyaw)
{
	currentSetpoint.pose.position.x += dx;
  currentSetpoint.pose.position.y += dy;
  currentSetpoint.pose.position.z += dz;
  
  tf::Quaternion quadOrientation(currentSetpoint.pose.orientation.x,
  															 currentSetpoint.pose.orientation.y,
  															 currentSetpoint.pose.orientation.z,
  															 currentSetpoint.pose.orientation.w);
  															 
  float currentRoll, currentPitch, currentYaw;
	tf::Matrix3x3(quadOrientation).getRPY(currentRoll, currentPitch, currentYaw);
  
  currentRoll += droll;
  currentPitch += dpitch;
  currentyaw += dyaw;
  
  tf::Quaternion quadOrientation;
	quadOrientation.setRPY(currentRoll, currentPitch, currentyaw);
	
	currentSetpoint.pose.orientation.x = quadOrientation.x();
	currentSetpoint.pose.orientation.y = quadOrientation.y();
	currentSetpoint.pose.orientation.z = quadOrientation.z();
	currentSetpoint.pose.orientation.w = quadOrientation.w();
}

// *********************************************************************
void publish_current_setpoint()
{
	isBounded(currentSetpoint);
	setpointPub.publish(currentSetpoint);
}

// ***********************************************************************
void drone_pose::stop_traj_timer()
{
	trajTimer.stop();
}

// ***********************************************************************
void drone_pose::start_traj_timer()
{
	trajTimer.setPeriod(ros::Duration(currentSamplingTime), false);
	trajTimer.start();
}

// ***********************************************************************
void set_setpoint_from_raw(float x, float y, float z, float roll, float pitch, float yaw);
{
	currentSetpoint.header.stamp = ros::Time::now();
	
	currentSetpoint.pose.position.x = x;
	currentSetpoint.pose.position.y = y;
	currentSetpoint.pose.position.z = z;
	
	tf::Quaternion quadOrientation;
	quadOrientation.setRPY(roll, pitch, yaw);
	
	currentSetpoint.pose.orientation.x = quadOrientation.x();
	currentSetpoint.pose.orientation.y = quadOrientation.y();
	currentSetpoint.pose.orientation.z = quadOrientation.z();
	currentSetpoint.pose.orientation.w = quadOrientation.w();
}
/////////////////////////////////////////////////////////////////////////

// Main
int main(int argc, char **argv)
{
	ros::init(argc, argv, "drone_pose");
	ros::NodeHandle nh;
	
	drone_pose dronePose(&nh);

	// Publish Rate
	ros::Rate rate(250.0);

	while(ros::ok())
	{
		switch(dronePose.get_current_flight_mode())
		{
			case 'J' : // Joystick Mode
				dronePose.stop_traj_timer();
				
				dronePose.increment_setpoint(0.004*xbox_pose[0], 
																		 0.004*xbox_pose[1],
																		 0.004*xbox_pose[2],
																		 0,
																		 0,
																		 0.004*xbox_pose[3]);
				
				dronePose.publish_current_setpoint();
				break;
			case 'H' : // Hold Mode
				//dronePose.start_traj_timer();
				break;
			case 'T' : // Trajectory Mode				
				//dronePose.start_traj_timer();
				break;
			case 'W' : // Waypoint Mode
				//dronePose.start_traj_timer();
				break;
			case 'S' : // Silent Mode
				timer.stop();
				break;
			default :
				timer.stop();
				ROS_WARN("Error! Flight Mode not recognized") << endl;
				break;
		}
	
		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}

// ************************************************ //



