#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TwistStamped.h>
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
#include <mavros_msgs/ParamSet.h>
#include <mavros_msgs/ParamGet.h>
#include <tf2/LinearMath/Transform.h>

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
	ros::ServiceClient setParamClient;
	ros::ServiceClient getParamClient;
	
	// Timers
	ros::Timer trajTimer;
	
	// Mission variables
	geometry_msgs::PoseStamped currentPose;
	geometry_msgs::PoseStamped currentSetpoint;
	int currentWaypointId;
	geometry_msgs::PoseArray currentWaypointList;
	float currentSamplingTime;
	char currentFlightMode;
	float currentMaxXYVel;
	float currentMaxZVel;
	float currentMaxYawRate;
	
	// Status variables
	mavros_msgs::State currentStateMavros;
	float currentJoystickVal[4]; //x,y,z,yaw
	bool currentTrajTimerStatus;
	geometry_msgs::TwistStamped currentPotentialField;
	
	// From Parameter Server
	vector<float> xBoundsParam;
	vector<float> yBoundsParam;
	vector<float> zBoundsParam;
	vector<float> takeoffPositionParam;
	float successRadiusParam;
	int armButtonParam;
	int disarmButtonParam;
	int aButtonParam;
	int bButtonParam;
	int xAxisParam;
	int yAxisParam;
	int zAxisParam;
	int yawAxisParam;
	float maxXYVelParam;
	float maxZVelParam;
	float maxYawRateParam;
	
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
		setpointPub = nh->advertise<geometry_msgs::PoseStamped>("setpoint_topic", 10);
		
		// Servers
		flightModeServer = nh->advertiseService("flight_mode_service", &drone_pose_class::flightMode_cb, this);
		
		// Clients
		armingClient = nh->serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
		setModeClient = nh->serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
		getParamClient = nh->serviceClient<mavros_msgs::ParamGet>("mavros/param/get");
		setParamClient = nh->serviceClient<mavros_msgs::ParamSet>("mavros/param/set");
		
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
	void publish_current_setpoint(bool);
	void start_traj_timer(float);
	void stop_traj_timer();
	void update_pf_from_joy();
	void set_max_vel_params(float, float, float);
	void get_max_vel_params(float&, float&, float&);
	float get_vector_magnitude(tf2::Vector3&);
	tf2::Vector3 transform_world_to_body(geometry_msgs::Pose, geometry_msgs::Pose, bool);
	tf2::Vector3 transform_world_to_body(tf2::Vector3, geometry_msgs::Pose, bool);
	bool isPfActive();
};


//////////////////////////////////////////////////////////////////////

void drone_pose_class::joy_cb(const sensor_msgs::Joy& msg)
{
	mavros_msgs::SetMode setMode;
	mavros_msgs::CommandBool armingCommand;
	
	if(msg.buttons[armButtonParam] == 1) //ARM = X
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

	if(msg.buttons[disarmButtonParam] == 1) //DISARM = Y
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

	if(msg.buttons[aButtonParam] == 1)
	{
		currentFlightMode = 'J';

		currentSetpoint.header.stamp = ros::Time::now();
		
		currentSetpoint.pose = currentPose.pose;		
		
		ROS_WARN("Flight Mode set to '%c'", currentFlightMode);
		ROS_INFO("Setpoint set to current pose");
	}

	if(msg.buttons[bButtonParam] == 1)
	{
		currentFlightMode = 'H';
		ROS_WARN("Flight Mode set to '%c'", currentFlightMode);
 	}

	if(abs(msg.axes[xAxisParam])>0.2) //X
		currentJoystickVal[0] = msg.axes[xAxisParam];
	else
		currentJoystickVal[0] = 0;

	if(abs(msg.axes[yAxisParam])>0.2) //Y
		currentJoystickVal[1] = msg.axes[yAxisParam];
	else
		currentJoystickVal[1] = 0;

	if(abs(msg.axes[zAxisParam])>0.2)  //Z
		currentJoystickVal[2] = msg.axes[zAxisParam];
	else
		currentJoystickVal[2] = 0;

	if(abs(msg.axes[yawAxisParam])>0.2)  //YAW
		currentJoystickVal[3] = msg.axes[yawAxisParam];
	else
		currentJoystickVal[3] = 0;
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
	publish_current_setpoint(false);	
	return;
	}
	
	if(currentWaypointList.poses.empty())
	{
		publish_current_setpoint(false);
		return;
	}
	
	if (currentFlightMode == 'T')
	{
		cout << "Waypoint Array Size : " << currentWaypointList.poses.size() << endl;
		
		if(currentWaypointId == currentWaypointList.poses.size())
		currentWaypointId -= 1;
		
		cout << "Publishing waypoint " << currentWaypointId << endl;
		
		currentSetpoint.pose = currentWaypointList.poses.at(currentWaypointId);
		publish_current_setpoint(false);
		
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
		publish_current_setpoint(false);
		
		if(pose_distance(currentSetpoint.pose, currentPose.pose) < successRadiusParam)
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
	while(!nh->getParam("drone_pose_node/x_bounds", xBoundsParam));
	while(!nh->getParam("drone_pose_node/y_bounds", yBoundsParam));
	while(!nh->getParam("drone_pose_node/z_bounds", zBoundsParam));
	
	while(!nh->getParam("drone_pose_node/takeoff_position", takeoffPositionParam));

	while(!nh->getParam("drone_pose_node/arm_button", armButtonParam));
	while(!nh->getParam("drone_pose_node/disarm_button", disarmButtonParam));

	while(!nh->getParam("drone_pose_node/A_button", aButtonParam));
	while(!nh->getParam("drone_pose_node/B_button", bButtonParam));

	while(!nh->getParam("drone_pose_node/x_axis", xAxisParam));
	while(!nh->getParam("drone_pose_node/y_axis", yAxisParam));
	while(!nh->getParam("drone_pose_node/z_axis", zAxisParam));
	while(!nh->getParam("drone_pose_node/yaw_axis", yawAxisParam));
	
	while(!nh->getParam("drone_pose_node/successRadius", successRadiusParam));
	
	while(!nh->getParam("drone_pose_node/max_xy_vel", maxXYVelParam));
	while(!nh->getParam("drone_pose_node/max_z_vel", maxZVelParam));
	while(!nh->getParam("drone_pose_node/max_yaw_rate", maxYawRateParam));
	
	ROS_INFO("Parameters for drone_pose retreived from the parameter server");
}

//*****************************************************************
void drone_pose_class::init() 
{
	// Create and set target position
	tf::Quaternion quadOrientation;
	quadOrientation.setRPY(0,0,0);	

	currentSetpoint.header.stamp = ros::Time::now();

	currentSetpoint.header.frame_id = "map";
	
	currentSetpoint.pose.position.x = takeoffPositionParam[0];
	currentSetpoint.pose.position.y = takeoffPositionParam[1];
	currentSetpoint.pose.position.z = takeoffPositionParam[2];

	currentSetpoint.pose.orientation.x = quadOrientation.x();
	currentSetpoint.pose.orientation.y = quadOrientation.y();
	currentSetpoint.pose.orientation.z = quadOrientation.z();
	currentSetpoint.pose.orientation.w = quadOrientation.w();
	
	quadOrientation.setRPY(0,0,0);
	currentPose.pose.orientation.x = quadOrientation.x();
	currentPose.pose.orientation.y = quadOrientation.y();
	currentPose.pose.orientation.z = quadOrientation.z();
	currentPose.pose.orientation.w = quadOrientation.w();
	
	currentWaypointList.poses.clear();
	
	currentSamplingTime = 0.2;
	currentTrajTimerStatus = true;
	stop_traj_timer();
	
	currentWaypointId = 0;
	
	currentJoystickVal[0] = 0;
	currentJoystickVal[1] = 0;
	currentJoystickVal[2] = 0;
	currentJoystickVal[3] = 0;

	currentFlightMode = 'J';
	
	set_max_vel_params(maxXYVelParam, maxZVelParam, maxYawRateParam);
	
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

	if(point.pose.position.x < xBoundsParam[0])
	point.pose.position.x = xBoundsParam[0];
	if(point.pose.position.x > xBoundsParam[1])
	point.pose.position.x = xBoundsParam[1];

	if(point.pose.position.y < yBoundsParam[0])
	point.pose.position.y = yBoundsParam[0];
	if(point.pose.position.y > yBoundsParam[1])
	point.pose.position.y = yBoundsParam[1];

	if(point.pose.position.z < zBoundsParam[0])
	point.pose.position.z = zBoundsParam[0];
	if(point.pose.position.z > zBoundsParam[1])
	point.pose.position.z = zBoundsParam[1];
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
		dx = dx*currentJoystickVal[0];
		dy = dy*currentJoystickVal[1];
		dz = dz*currentJoystickVal[2];
		
		droll = 0;
		dpitch = 0;
		dyaw = dyaw*currentJoystickVal[3];
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
void drone_pose_class::publish_current_setpoint(bool usePf)
{
	//cout << "Publishing pose setpoint : (" << currentSetpoint.pose.position.x << ", "
	//																			 << currentSetpoint.pose.position.y << ", "
	//																			 << currentSetpoint.pose.position.z << ")" << endl;	
	
	if(usePf && isPfActive())
	{
		// Assuming only position is transformed
		tf2::Vector3 carrotVec = transform_world_to_body(currentSetpoint.pose, currentPose.pose, true);
		tf2::Vector3 pfVec(currentPotentialField.twist.linear.x, 
											 currentPotentialField.twist.linear.y, 
											 currentPotentialField.twist.linear.z);
		
		float carrotMag = get_vector_magnitude(carrotVec);
		float pfMag = get_vector_magnitude(pfVec);
		
		tf2::Vector3 resVec(pfMag*pfVec.x() + (1 - pfMag)*carrotVec.x(),
												pfMag*pfVec.y() + (1 - pfMag)*carrotVec.y(),
												pfMag*pfVec.z() + (1 - pfMag)*carrotVec.z());
		
		float resMag = get_vector_magnitude(resVec);
		
		// Assuming only position is transformed
		resVec = transform_world_to_body(resVec, currentPose.pose, false);
		
		geometry_msgs::PoseStamped localSetpoint;
		
		localSetpoint.pose.position.x = resVec.x();
		localSetpoint.pose.position.y = resVec.y();
		localSetpoint.pose.position.z = resVec.z();
		
		localSetpoint.pose.orientation = currentSetpoint.pose.orientation;
		
		localSetpoint.header.stamp = ros::Time::now();
	
		if(!currentPose.header.frame_id.empty())
		localSetpoint.header.frame_id = currentPose.header.frame_id;
		else
		localSetpoint.header.frame_id = currentSetpoint.header.frame_id;
		
		set_max_vel_params(resMag*maxXYVelParam, resMag*maxZVelParam, maxYawRateParam);
		
		isBounded(localSetpoint);
		
		setpointPub.publish(localSetpoint);
	}
	
	else
	{
	isBounded(currentSetpoint);
	
	set_max_vel_params(maxXYVelParam, maxZVelParam, maxYawRateParam);
	
	currentSetpoint.header.stamp = ros::Time::now();
	
	if(!currentPose.header.frame_id.empty())
	currentSetpoint.header.frame_id = currentPose.header.frame_id;					 
	
	setpointPub.publish(currentSetpoint);
	}
}

// ***********************************************************************
bool drone_pose_class::isPfActive()
{
	if(currentPotentialField.twist.linear.x != 0 ||
		 currentPotentialField.twist.linear.y != 0 ||
		 currentPotentialField.twist.linear.z != 0)
	return true;
	
	return false;
}

// ***********************************************************************
float drone_pose_class::get_vector_magnitude(tf2::Vector3& inputVector)
{
	float mag = sqrt(pow(inputVector.x(), 2) + pow(inputVector.y(), 2) + pow(inputVector.z(), 2));
	
	tf2::Vector3 outputVector(inputVector.x()/mag,
														inputVector.y()/mag,
														inputVector.z()/mag);
	
	inputVector = outputVector;
	return mag;
}

// ***********************************************************************
tf2::Vector3 drone_pose_class::transform_world_to_body(geometry_msgs::Pose pose, geometry_msgs::Pose robotPose, bool transformDirection)
{
	tf2::Transform localTransform;
	
	tf2::Vector3 bodyTranslation(tf2::Vector3(robotPose.position.x, 
																						robotPose.position.y, 
																						robotPose.position.z));
																						 
	tf2::Quaternion bodyRotation(tf2::Quaternion(robotPose.orientation.x, 
																							 robotPose.orientation.y, 
																							 robotPose.orientation.z, 
																							 robotPose.orientation.w));
	
	localTransform.setOrigin(bodyTranslation);
	
	localTransform.setRotation(bodyRotation);
	
	if(transformDirection)
	return localTransform.inverse()*tf2::Vector3(pose.position.x,
																					 		 pose.position.y,
																							 pose.position.z);
	else
	return localTransform*tf2::Vector3(pose.position.x,
																		 pose.position.y,
																		 pose.position.z);
}

// ***********************************************************************
tf2::Vector3 drone_pose_class::transform_world_to_body(tf2::Vector3 pose, geometry_msgs::Pose robotPose, bool transformDirection)
{
	tf2::Transform localTransform;
	
	tf2::Vector3 bodyTranslation(tf2::Vector3(robotPose.position.x, 
																						robotPose.position.y, 
																						robotPose.position.z));
																						 
	tf2::Quaternion bodyRotation(tf2::Quaternion(robotPose.orientation.x, 
																							 robotPose.orientation.y, 
																							 robotPose.orientation.z, 
																							 robotPose.orientation.w));
	
	localTransform.setOrigin(bodyTranslation);
	
	localTransform.setRotation(bodyRotation);
	
	if(transformDirection)
	return localTransform.inverse()*pose;
	else
	return localTransform*pose;
}

// ***********************************************************************
void drone_pose_class::stop_traj_timer()
{
	trajTimer.stop();
	
	if(currentTrajTimerStatus)
	ROS_INFO("Trajectory timer stopped");
	
	currentTrajTimerStatus = false;
}

// ***********************************************************************
void drone_pose_class::start_traj_timer(float samplingTime)
{
	currentSamplingTime = samplingTime;
	trajTimer.setPeriod(ros::Duration(currentSamplingTime), false);
	trajTimer.start();
	
	if(!currentTrajTimerStatus)
	ROS_INFO("Trajectory timer started with sampling time %f", currentSamplingTime);
	
	currentTrajTimerStatus = true;
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

// ***********************************************************************

void drone_pose_class::update_pf_from_joy()
{
	currentPotentialField.twist.linear.x = currentJoystickVal[0];
	currentPotentialField.twist.linear.y = currentJoystickVal[1];
	currentPotentialField.twist.linear.z = currentJoystickVal[2];
	currentPotentialField.twist.angular.z = currentJoystickVal[3];
}

// ************************************************************************

void drone_pose_class::set_max_vel_params(float xyVel, float zVel, float yawRate)
{
	currentMaxXYVel = xyVel;
	currentMaxZVel = zVel;
	currentMaxYawRate = yawRate;
	mavros_msgs::ParamSet paramSetSrv;
	
	ROS_INFO("Max velocities set to (%f, %f, %f)", currentMaxXYVel, currentMaxZVel, currentMaxYawRate);
}

// ************************************************************************

void drone_pose_class::get_max_vel_params(float& xyVel, float& zVel, float& yawRate)
{
	mavros_msgs::ParamGet paramGetSrv;
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
				
				dronePose.publish_current_setpoint(false);
				break;
			case 'H' : // Hold Mode
				//dronePose.start_traj_timer(dronePose.get_current_sampling_time());
				dronePose.stop_traj_timer();
				dronePose.update_pf_from_joy();
				dronePose.publish_current_setpoint(true);
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



