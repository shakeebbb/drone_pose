#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/PositionTarget.h>
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

class drone_pose_class
{

private:
	// Subscribers
	ros::Subscriber stateMavrosSub;
	ros::Subscriber joySub;
	ros::Subscriber poseSub;
	ros::Subscriber setpointSub;
	ros::Subscriber trajectorySub;
	
	// Publishers
	ros::Publisher setpointPub;
	
	// Servers
	ros::ServiceServer flightModeServer;
	
	// Clients
	ros::ServiceClient armingClient;
	ros::ServiceClient setModeClient;
	
	// Timers
	ros::Timer trajTimer;
	
	// Mission variables
	geometry_msgs::PoseStamped currentPose;
	geometry_msgs::PoseStamped currentSetpoint;
	int currentWaypointId;
	geometry_msgs::PoseArray currentWaypointList;
	float currentSamplingTime;
	char currentFlightMode;
	
	// Status variables
	mavros_msgs::State currentStateMavros;
	float joystickVal[4]; //x,y,z,yaw
	bool trajTimerStatus;
	
	// From Parameter Server
	vector<float> xBounds;
	vector<float> yBounds;
	vector<float> zBounds;
	vector<float> takeoffPosition;
	float successRadius;
	int armButton;
	int disarmButton;
	int aButton;
	int bButton;
	int xAxis;
	int yAxis;
	int zAxis;
	int yawAxis;
	
public:

	drone_pose_class(ros::NodeHandle *nh)
	{
		wait_for_params(nh);
		
		// Subscribers
		stateMavrosSub = nh->subscribe("mavros_state_topic", 10, &drone_pose_class::state_mavros_cb, this);
		joySub = nh->subscribe("joy_topic", 100, &drone_pose_class::joy_cb, this);
		poseSub = nh->subscribe("pose_topic", 100, &drone_pose_class::pose_cb, this);
		trajectorySub = nh->subscribe("trajectory_topic", 10, &drone_pose_class::trajectory_cb, this);
		
		// Publishers
		setpointPub = nh->advertise<mavros_msgs::PositionTarget>("setpoint_topic", 10);
		
		// Servers
		flightModeServer = nh->advertiseService("flight_mode_service", &drone_pose_class::flightMode_cb, this);
		
		// Clients
		armingClient = nh->serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
		setModeClient = nh->serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
		
		//Timers
		trajTimer = nh->createTimer(ros::Duration(0.2), &drone_pose_class::traj_timer_cb, this);
		trajTimer.stop();
		
		init();
	}
	
	// Callbacks
	void state_mavros_cb(const mavros_msgs::State&);
	void pose_cb(const geometry_msgs::PoseStamped&);
	void traj_timer_cb(const ros::TimerEvent&);
	void trajectory_cb(const drone_pose::trajectoryMsg&);
	void joy_cb(const sensor_msgs::Joy&);
	bool flightMode_cb(drone_pose::flightModeSrv::Request&,
										 drone_pose::flightModeSrv::Response&);
									 
	// Other cpp functions
	void wait_for_params(ros::NodeHandle*);
	void init();
	bool isBounded(geometry_msgs::PoseStamped&);
	void set_setpoint_from_raw(float, float, float, float, float, float);
	void increment_setpoint(float, float, float, float, float, float, bool);
	float pose_distance(geometry_msgs::Pose, geometry_msgs::Pose);
	geometry_msgs::PoseStamped get_current_setpoint();
	float get_current_sampling_time();
	char get_current_flight_mode();
	void publish_current_setpoint();
	void start_traj_timer(float);
	void stop_traj_timer();
	
};


//////////////////////////////////////////////////////////////////////

void drone_pose_class::joy_cb(const sensor_msgs::Joy& msg)
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
		
		if(armingClient.call(armingCommand))
		ROS_INFO("DIS-ARMED: Service call successful");
		else
		{
		ROS_WARN("Could not disarm the vehicle: Service call unsuccessful");
		return;
		}
		
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
		currentFlightMode = 'J';

		currentSetpoint.header.stamp = ros::Time::now();
		
		currentSetpoint.pose = currentPose.pose;		
		
		ROS_WARN("Flight Mode set to '%c'", currentFlightMode);
		ROS_INFO("Setpoint set to current pose");
	}

	if(msg.buttons[bButton] == 1)
	{
		currentFlightMode = 'H';
		ROS_WARN("Flight Mode set to '%c'", currentFlightMode);
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

// ***********************************************************************
void drone_pose_class::trajectory_cb(const drone_pose::trajectoryMsg& msg)
{	
	if(msg.samplingTime != currentSamplingTime && msg.samplingTime > 0)
	{
		start_traj_timer(msg.samplingTime);
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
	
	for (int i = 0; i < msg.poses.size(); i++)	
	currentWaypointList.poses.push_back(msg.poses[i]);
}

// ********************************************************************
void drone_pose_class::traj_timer_cb(const ros::TimerEvent&)
{
	//cout << "Traj timer callback called" <<endl;
	if(!(currentFlightMode == 'T' || currentFlightMode == 'W' || currentFlightMode == 'H'))
	return;
	
	if(currentFlightMode == 'H')
	{
	publish_current_setpoint();	
	return;
	}
	
	if(currentWaypointList.poses.empty())
	{
		publish_current_setpoint();
		return;
	}
	
	if (currentFlightMode == 'T')
	{
		cout << "Waypoint Array Size : " << currentWaypointList.poses.size() << endl;
		
		if(currentWaypointId == currentWaypointList.poses.size())
		currentWaypointId -= 1;
		
		cout << "Publishing waypoint " << currentWaypointId << endl;
		
		currentSetpoint.pose = currentWaypointList.poses.at(currentWaypointId);
		publish_current_setpoint();
		
		currentWaypointId += 1;
		
		cout << "Next waypoint in queue " << currentWaypointId << endl;
		return;
	}
	
	if (currentFlightMode == 'W')
	{
		cout << "Waypoint Array Size : " << currentWaypointList.poses.size() << endl;
		
		//if(currentWaypointId == currentWaypointList.poses.size())
		//currentWaypointId -= 1;
		
		cout << "Publishing waypoint " << currentWaypointId << endl;
		
		currentSetpoint.pose = currentWaypointList.poses.at(0);
		publish_current_setpoint();
		
		if(pose_distance(currentSetpoint.pose, currentPose.pose) < successRadius)
		{
		currentWaypointList.poses.erase(currentWaypointList.poses.begin());
		currentWaypointId += 1;
		}
		
		cout << "Next waypoint in queue " << currentWaypointId << endl;
		return;
	}
	
}

// *********************************************************************
void drone_pose_class::wait_for_params(ros::NodeHandle *nh)
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
	
	while(!nh->getParam("drone_pose_node/successRadius", successRadius));
	
	ROS_INFO("Parameters for drone_pose retreived from the parameter server");
}

//*****************************************************************
void drone_pose_class::init() 
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
	
	currentSamplingTime = 0.2;
	trajTimerStatus = true;
	stop_traj_timer();
	
	currentWaypointId = 0;
	
	joystickVal[0] = 0;
	joystickVal[1] = 0;
	joystickVal[2] = 0;
	joystickVal[3] = 0;

	currentFlightMode = 'J';
	
	ROS_INFO("Initialized drone_pose");
	ROS_WARN("Flight Mode set to '%c'", currentFlightMode);
	
	ROS_INFO("Waiting for mavros to connect ...");
	cout << currentSetpoint.pose.position.x <<endl;
	cout << currentSetpoint.pose.position.y <<endl;
	cout << currentSetpoint.pose.position.z <<endl;
}

// *******************************************************************
bool drone_pose_class::isBounded(geometry_msgs::PoseStamped& point)
{
	//float x_ref = setpoint.pose.position.x;
	//float y_ref = setpoint.pose.position.y;
	//float z_ref = setpoint.pose.position.z;

	if(point.pose.position.x < xBounds[0])
	point.pose.position.x = xBounds[0];
	if(point.pose.position.x > xBounds[1])
	point.pose.position.x = xBounds[1];

	if(point.pose.position.y < yBounds[0])
	point.pose.position.y = yBounds[0];
	if(point.pose.position.y > yBounds[1])
	point.pose.position.y = yBounds[1];

	if(point.pose.position.z < zBounds[0])
	point.pose.position.z = zBounds[0];
	if(point.pose.position.z > zBounds[1])
	point.pose.position.z = zBounds[1];
	return 0;
}

// ***********************************************************************
bool drone_pose_class::flightMode_cb(drone_pose::flightModeSrv::Request& req,	
									 drone_pose::flightModeSrv::Response& res)
{
	if (req.setGet == 0 && currentFlightMode != 'J')
	{
		if (req.flightMode == 'H' || req.flightMode == 'W' || req.flightMode == 'T')
		currentFlightMode = req.flightMode;

		else
		ROS_WARN("Unrecognized mode transition requested");
	}
	
	res.flightMode = currentFlightMode;
		
	ROS_WARN("Flight mode transition request received: set to '%c'", currentFlightMode);
	
	return true;
}


// ***********************************************************************

void drone_pose_class::pose_cb(const geometry_msgs::PoseStamped& msg)
{
	currentPose = msg;
}

// ***********************************************************************
void drone_pose_class::state_mavros_cb(const mavros_msgs::State& msg)
{
	currentStateMavros = msg;
	
	static bool status = false;
	
	if(!status && currentStateMavros.connected)
	ROS_INFO("Mavros connected");
	
	else if(status && !currentStateMavros.connected)
	ROS_WARN("Mavros to disconnected");
	
	status = currentStateMavros.connected;
}

// ***********************************************************************
char drone_pose_class::get_current_flight_mode()
{
	return currentFlightMode;
}

// ***********************************************************************
geometry_msgs::PoseStamped drone_pose_class::get_current_setpoint()
{
	return currentSetpoint;
}

// ***********************************************************************
float drone_pose_class::get_current_sampling_time()
{
	return currentSamplingTime;
}

// ***********************************************************************
void drone_pose_class::increment_setpoint(float dx, float dy, float dz, float droll, float dpitch, float dyaw, bool add)
{

	//cout << "Joystick Value: (" << joystickVal[0] << ", " 
	//														<< joystickVal[1] << ", "
	//														<< joystickVal[2] << ", "
	//														<< joystickVal[3] << ")" << endl;
	
	if(!add)
	{
		dx = dx*joystickVal[0];
		dy = dy*joystickVal[1];
		dz = dz*joystickVal[2];
		
		droll = 0;
		dpitch = 0;
		dyaw = dyaw*joystickVal[3];
	}
	
	currentSetpoint.pose.position.x += dx;
  currentSetpoint.pose.position.y += dy;
  currentSetpoint.pose.position.z += dz;
  
  tf::Quaternion quadOrientation(currentSetpoint.pose.orientation.x,
  															 currentSetpoint.pose.orientation.y,
  															 currentSetpoint.pose.orientation.z,
  															 currentSetpoint.pose.orientation.w);
  															 
  double localRoll, localPitch, localYaw;
	tf::Matrix3x3(quadOrientation).getRPY(localRoll, localPitch, localYaw);
  
  localRoll += droll;
  localPitch += dpitch;
  localYaw += dyaw;
  
	quadOrientation.setRPY(localRoll, localPitch, localYaw);
	
	currentSetpoint.pose.orientation.x = quadOrientation.x();
	currentSetpoint.pose.orientation.y = quadOrientation.y();
	currentSetpoint.pose.orientation.z = quadOrientation.z();
	currentSetpoint.pose.orientation.w = quadOrientation.w();
}

// *********************************************************************
void drone_pose_class::publish_current_setpoint()
{
	//cout << "Publishing pose setpoint : (" << currentSetpoint.pose.position.x << ", "
	//																			 << currentSetpoint.pose.position.y << ", "
	//																			 << currentSetpoint.pose.position.z << ")" << endl;
	isBounded(currentSetpoint);
	
	mavros_msgs::PositionTarget targetSetpoint;
	
	targetSetpoint.header.stamp = ros::Time::now();
	targetSetpoint.coordinate_frame = targetSetpoint.FRAME_LOCAL_NED;
	targetSetpoint.type_mask = mavros_msgs::PositionTarget::IGNORE_VX |
														 mavros_msgs::PositionTarget::IGNORE_VY |
														 mavros_msgs::PositionTarget::IGNORE_VZ |
														 mavros_msgs::PositionTarget::IGNORE_AFX |
														 mavros_msgs::PositionTarget::IGNORE_AFY |
														 mavros_msgs::PositionTarget::IGNORE_AFZ |
														 mavros_msgs::PositionTarget::FORCE |
														 mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
	targetSetpoint.position = currentSetpoint.pose.position;
	
	tf::Quaternion quadOrientation(currentSetpoint.pose.orientation.x,
  															 currentSetpoint.pose.orientation.y,
  															 currentSetpoint.pose.orientation.z,
  															 currentSetpoint.pose.orientation.w);
  															 
  double localRoll, localPitch, localYaw;
	tf::Matrix3x3(quadOrientation).getRPY(localRoll, localPitch, localYaw);			
	
	targetSetpoint.yaw = localYaw;							 
	
	setpointPub.publish(targetSetpoint);
}

// ***********************************************************************
void drone_pose_class::stop_traj_timer()
{
	trajTimer.stop();
	
	if(trajTimerStatus)
	ROS_INFO("Trajectory timer stopped");
	
	trajTimerStatus = false;
}

// ***********************************************************************
void drone_pose_class::start_traj_timer(float samplingTime)
{
	currentSamplingTime = samplingTime;
	trajTimer.setPeriod(ros::Duration(currentSamplingTime), false);
	trajTimer.start();
	
	if(!trajTimerStatus)
	ROS_INFO("Trajectory timer started with sampling time %f", currentSamplingTime);
	
	trajTimerStatus = true;
}

// ***********************************************************************
void drone_pose_class::set_setpoint_from_raw(float x, float y, float z, float roll, float pitch, float yaw)
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

// *************************************************************************
float drone_pose_class::pose_distance(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2)
{
	float x_err = pose1.position.x - pose2.position.x;
	float y_err = pose1.position.y - pose2.position.y;
	float z_err = pose1.position.z - pose2.position.z;
	
	tf::Quaternion quadOrientation1(pose1.orientation.x,
  															 pose1.orientation.y,
  															 pose1.orientation.z,
  															 pose1.orientation.w);
  															 
  tf::Quaternion quadOrientation2(pose2.orientation.x,
  															 pose2.orientation.y,
  															 pose2.orientation.z,
  															 pose2.orientation.w);
  															 
  double currentRoll1, currentPitch1, currentYaw1;
	tf::Matrix3x3(quadOrientation1).getRPY(currentRoll1, currentPitch1, currentYaw1);
	
	double currentRoll2, currentPitch2, currentYaw2;
	tf::Matrix3x3(quadOrientation2).getRPY(currentRoll2, currentPitch2, currentYaw2);
	
	float yaw_err = currentYaw1 - currentYaw2;
	
	return sqrt(pow(x_err,2) + pow(y_err,2) + pow(z_err,2) + pow(yaw_err,2));
}
/////////////////////////////////////////////////////////////////////////

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
				
				dronePose.publish_current_setpoint();
				break;
			case 'H' : // Hold Mode
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



