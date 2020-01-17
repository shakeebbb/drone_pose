#include "drone_pose.h"

drone_pose_class::drone_pose_class(ros::NodeHandle *nh)
{	
	wait_for_params(nh);
		
	// Subscribers
	stateMavrosSub = nh->subscribe("drone_pose/mavros_state_topic", 10, &drone_pose_class::state_mavros_cb, this);
	joySub = nh->subscribe("drone_pose/joy_topic", 100, &drone_pose_class::joy_cb, this);
	poseSub = nh->subscribe("drone_pose/pose_topic", 100, &drone_pose_class::pose_cb, this);
	trajectorySub = nh->subscribe("drone_pose/trajectory_topic", 10, &drone_pose_class::trajectory_cb, this);
	pfSub = nh->subscribe("drone_pose/pf_topic", 10, &drone_pose_class::pf_cb, this);
	estopSub = nh->subscribe("drone_pose/estop_status_topic", 10, &drone_pose_class::estop_cb, this);
	extendedStateMavrosSub = nh->subscribe("drone_pose/mavros_extended_state_topic", 10, &drone_pose_class::extended_state_mavros_cb, this);
		
	// Publishers
	setpointPub = nh->advertise<geometry_msgs::PoseStamped>("drone_pose/setpoint_topic", 10);
	setpointGoalPub = nh->advertise<geometry_msgs::PoseStamped>("drone_pose/setpoint_goal_topic", 10);
	flightModePub = nh->advertise<std_msgs::String>("drone_pose/flight_mode_topic", 10);
	estopPub = nh->advertise<std_msgs::Bool>("drone_pose/estop_command_topic", 10);
		
	// Servers
	flightModeServer = nh->advertiseService("drone_pose/flight_mode_service", &drone_pose_class::flightMode_cb, this);
		
	// Clients
	armingClient = nh->serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
	setModeClient = nh->serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
	getParamClient = nh->serviceClient<mavros_msgs::ParamGet>("mavros/param/get");
	setParamClient = nh->serviceClient<mavros_msgs::ParamSet>("mavros/param/set");
		
	// Timers
	trajTimer = nh->createTimer(ros::Duration(0.2), &drone_pose_class::traj_timer_cb, this);
	trajTimer.stop();
	
	// Transforms
	tfListenerPtr = new tf2_ros::TransformListener(tfBuffer);
		
	// Initialize Flight Variables	
	init();
}

// ***********************************************************************
void drone_pose_class::estop_cb(const std_msgs::Bool& msg)
{
	static bool prevStatus = false; 
	
	if(msg.data == true && prevStatus == false)
	{
		currentFlightMode = 'L';
		currentSetpoint = currentPose;
	}
	
	prevStatus = msg.data;
	
	if(currentFlightMode == 'L')
	{
		std_msgs::Bool localStatus;
		localStatus.data = true;
		estopPub.publish(localStatus);
	}
	else
	{
		std_msgs::Bool localStatus;
		localStatus.data = false;
		estopPub.publish(localStatus);
	}
}

// ***********************************************************************
void drone_pose_class::pf_cb(const geometry_msgs::TwistStamped& msg)
{
	currentPotentialField = msg;
}

// ***********************************************************************
drone_pose_class::~drone_pose_class()
{
	delete tfListenerPtr;
}
	
// ***********************************************************************
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
		
		//currentSetpoint.pose = currentPose.pose;
		
		double localRoll, localPitch, localYaw;
		quat_to_rpy(currentPose.pose.orientation.x,
  						  currentPose.pose.orientation.y,
  						  currentPose.pose.orientation.z,
  							currentPose.pose.orientation.w,
  							localRoll,
  							localPitch,
  							localYaw);
			
		currentSetpoint.pose = get_pose_from_raw(currentPose.pose.position.x, 
																						 currentPose.pose.position.y, 
																						 currentPose.pose.position.z, 
																						 0,
																						 0,
																						 localYaw);
		
		ROS_WARN("Flight Mode set to '%c'", currentFlightMode);
		ROS_INFO("Setpoint set to current pose");
	}

	if(msg.buttons[bButtonParam] == 1)
	{
		currentFlightMode = 'H';
		ROS_WARN("Flight Mode set to '%c'", currentFlightMode);
 	}
 	
 	if(msg.buttons[landButtonParam] == 1)
 	{
 		currentFlightMode = 'L';
		currentSetpoint = currentPose;
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
	if(!(currentFlightMode == 'T' || currentFlightMode == 'W' || currentFlightMode == 'H' || currentFlightMode == 'L'))
	return;
	
	if(currentFlightMode == 'H')
	{
		publish_current_setpoint(true);	
		return;
	}
	
	if(currentFlightMode == 'L')
	{
		if(currentExtendedStateMavros.landed_state == mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND)
		{
			mavros_msgs::CommandBool armingCommand;
			armingCommand.request.value = false;
				
			if(armingClient.call(armingCommand))
			ROS_INFO("DIS-ARMED: Service call successful");
			else
			ROS_WARN("Could not disarm the vehicle: Service call unsuccessful");
		}
		else
		{
			increment_setpoint(0,0,-1*landSpeedParam*currentSamplingTime, 0,0,0, true);
			publish_current_setpoint(false);
		}
		return;
	}
	
	if(currentWaypointList.poses.empty())
	{
		publish_current_setpoint(false);
		return;
	}
	
	if (currentFlightMode == 'T')
	{
		std::cout << "Waypoint Array Size : " << currentWaypointList.poses.size() << std::endl;
		
		if(currentWaypointId == currentWaypointList.poses.size())
		currentWaypointId -= 1;
		
		std::cout << "Publishing waypoint " << currentWaypointId << std::endl;
		
		currentSetpoint.pose = currentWaypointList.poses.at(currentWaypointId);
		publish_current_setpoint(false);
		
		currentWaypointId += 1;
		
		std::cout << "Next waypoint in queue " << currentWaypointId << std::endl;
		return;
	}
	
	if (currentFlightMode == 'W')
	{
		std::cout << "Waypoint Array Size : " << currentWaypointList.poses.size() << std::endl;
		
		//if(currentWaypointId == currentWaypointList.poses.size())
		//currentWaypointId -= 1;
		
		std::cout << "Publishing waypoint " << currentWaypointId << std::endl;
		
		currentSetpoint.pose = currentWaypointList.poses.at(0);
		publish_current_setpoint(false);
		
		if(pose_distance(currentSetpoint.pose, currentPose.pose) < successRadiusParam)
		{
		currentWaypointList.poses.erase(currentWaypointList.poses.begin());
		currentWaypointId += 1;
		}
		
		std::cout << "Next waypoint in queue " << currentWaypointId << std::endl;
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
	while(!nh->getParam("drone_pose_node/land_button", landButtonParam));

	while(!nh->getParam("drone_pose_node/x_axis", xAxisParam));
	while(!nh->getParam("drone_pose_node/y_axis", yAxisParam));
	while(!nh->getParam("drone_pose_node/z_axis", zAxisParam));
	while(!nh->getParam("drone_pose_node/yaw_axis", yawAxisParam));
	
	while(!nh->getParam("drone_pose_node/successRadius", successRadiusParam));
	
	while(!nh->getParam("drone_pose_node/max_xy_vel", maxXYVelParam));
	while(!nh->getParam("drone_pose_node/max_z_vel", maxZVelParam));
	while(!nh->getParam("drone_pose_node/max_yaw_rate", maxYawRateParam));
	while(!nh->getParam("drone_pose_node/land_speed", landSpeedParam));
	
	while(!nh->getParam("drone_pose_node/frame_id", frameId));
	
	ROS_INFO("Parameters for drone_pose retreived from the parameter server");
}

//*****************************************************************
void drone_pose_class::init() 
{
	// Create and set target position
	tf::Quaternion quadOrientation;
	quadOrientation.setRPY(0,0,0);	

	currentSetpoint.header.stamp = ros::Time::now();

	currentSetpoint.header.frame_id = frameId;
	
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
	
	currentSamplingTime = 0.1;
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
	std::cout << currentSetpoint.pose.position.x << std::endl;
	std::cout << currentSetpoint.pose.position.y << std::endl;
	std::cout << currentSetpoint.pose.position.z << std::endl;
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
	if (req.setGet == 0 && currentFlightMode != 'J' && currentFlightMode != 'L')
	{
		if (req.flightMode == 'H' || req.flightMode == 'W' || req.flightMode == 'T' || req.flightMode == 'L' || req.flightMode == 'J')
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
void drone_pose_class::extended_state_mavros_cb(const mavros_msgs::ExtendedState& msg)
{
	currentExtendedStateMavros = msg;
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
void drone_pose_class::increment_setpoint(float dx, float dy, float dz, 
																					float droll, float dpitch, float dyaw, bool add)
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
  
  double localRoll, localPitch, localYaw;
  quat_to_rpy(currentSetpoint.pose.orientation.x,
  						currentSetpoint.pose.orientation.y,
  						currentSetpoint.pose.orientation.z,
  						currentSetpoint.pose.orientation.w,
  						localRoll, localPitch, localYaw);
  
  localRoll += droll;
  localPitch += dpitch;
  localYaw += dyaw;
  
  float localQx, localQy, localQz, localQw;
  rpy_to_quat(localRoll, localPitch, localYaw,
  						localQx, localQy, localQz, localQw);
  						
  currentSetpoint.pose.orientation.x = localQx;
  currentSetpoint.pose.orientation.y = localQy;
  currentSetpoint.pose.orientation.z = localQz;
  currentSetpoint.pose.orientation.w = localQw;
}

// *********************************************************************
void drone_pose_class::publish_current_setpoint(bool usePf)
{
	//cout << "Publishing pose setpoint : (" << currentSetpoint.pose.position.x << ", "
	//																			 << currentSetpoint.pose.position.y << ", "
	//																			 << currentSetpoint.pose.position.z << ")" << endl;	

	std_msgs::String flMode;
	flMode.data = currentFlightMode;
	flightModePub.publish(flMode);

	static geometry_msgs::PoseStamped lastCommandedSetpoint;
	
	if(lastCommandedSetpoint.pose.orientation.x == 0 && lastCommandedSetpoint.pose.orientation.y == 0 &&
		 lastCommandedSetpoint.pose.orientation.z == 0 && lastCommandedSetpoint.pose.orientation.w == 0)
	{
		lastCommandedSetpoint = currentSetpoint;
		return;
	}
	
	if(!usePf)
	{
		isBounded(currentSetpoint);
		
		currentSetpoint.header.stamp = ros::Time::now();
		
		currentSetpoint.header.frame_id = frameId;					 
		
		setpointPub.publish(currentSetpoint);
		lastCommandedSetpoint = currentSetpoint;
		
		//cout << "PF is diabled or local setpoint reached" << endl;
	}
	
	else if(!isPfActive() && pose_distance(currentSetpoint.pose, lastCommandedSetpoint.pose, "position") < successRadiusParam)
	{
		lastCommandedSetpoint.header.stamp = ros::Time::now();
		setpointPub.publish(lastCommandedSetpoint);
	}
	
	else
	{
		tf2::Vector3 carrotVec = transform_world_to_body(tf2::Vector3(currentSetpoint.pose.position.x,
																																	currentSetpoint.pose.position.y,
																																	currentSetpoint.pose.position.z),
																																	lastCommandedSetpoint.pose, true);
																																	
		std::cout << "carrotVec: (" << carrotVec.x() << ", " << carrotVec.y() << ", "<< carrotVec.z() << ")" << std::endl;
		
		double localRoll, localPitch, localYaw;
		quat_to_rpy(currentPose.pose.orientation.x, currentPose.pose.orientation.y, currentPose.pose.orientation.z, currentPose.pose.orientation.w,
						 		localRoll, localPitch, localYaw);
		
		geometry_msgs::Pose inclination = get_pose_from_raw(0, 0, 0, localRoll, localPitch, 0);
		
		std::cout << "currentRoll: " << localRoll << "currentPitch: " << localPitch << std::endl;
											 
		tf2::Vector3 pfVec = transform_world_to_body(tf2::Vector3(currentPotentialField.twist.linear.x,
																								 							currentPotentialField.twist.linear.y,
																								 							currentPotentialField.twist.linear.z),
																								 							inclination, false);
											 
		std::cout << "pfVec: (" << pfVec.x() << ", " << pfVec.y() << ", "<< pfVec.z() << ")" << std::endl;
		
		float carrotMag = get_vector_magnitude(carrotVec);
		float pfMag = get_vector_magnitude(pfVec);
		
		tf2::Vector3 resVec(pfMag*pfVec.x() + (1 - pfMag)*carrotVec.x(),
												pfMag*pfVec.y() + (1 - pfMag)*carrotVec.y(),
												pfMag*pfVec.z() + (1 - pfMag)*carrotVec.z());
												
		std::cout << "resVec: (" << resVec.x() << ", " << resVec.y() << ", "<< resVec.z() << ")" << std::endl;
		
		float vx, vy, vz, vyaw, hed;
		vec_to_vel(resVec, vx, vy, vz, vyaw, hed);
		
		std::cout << "resVel: (" << vx << ", " << vy << ", "<< vz << ", " << vyaw << ")" << std::endl;
		
		geometry_msgs::PoseStamped localSetpoint;
		/*
		if(!isPfActive() && pose_distance(get_pose_from_raw(0,0,0,0,0,hed), currentPose.pose, "heading") > 0.35)
		{	
			cout << "Heading error is " << pose_distance(get_pose_from_raw(0,0,0,0,0,hed), currentPose.pose, "heading") << endl;
			localSetpoint.pose = get_pose_from_raw(0,
																						 0,
																						 0,
																						 0,
																						 0,
																						 maxYawRateParam*currentSamplingTime);
			cout << "Correcting heading before translating" << endl;
		}
		else
		*/
			localSetpoint.pose = get_pose_from_raw(vx*currentSamplingTime,
																						 vy*currentSamplingTime,
																						 vz*currentSamplingTime,
																						 0,
																						 0,
																						 vyaw*currentSamplingTime);																					 
			std::cout << "Correcting heading and translating" << std::endl;
																					 
		localSetpoint.pose = transform_world_to_body(localSetpoint.pose, lastCommandedSetpoint.pose, false);
		std::cout << "Current Robot Position: (" << currentPose.pose.position.x << ", " << currentPose.pose.position.y << ", "
																			  << currentPose.pose.position.z << ")" << std::endl;
		std::cout << "Transformed Setpoint Position: (" << localSetpoint.pose.position.x << ", " << localSetpoint.pose.position.y << ", "
																							 << localSetpoint.pose.position.z << ")" << std::endl;
		
		localSetpoint.header.stamp = ros::Time::now();
		localSetpoint.header.frame_id = frameId;
		
		isBounded(localSetpoint);
		
		setpointPub.publish(localSetpoint);
		lastCommandedSetpoint = localSetpoint;
	}
	
	currentSetpoint.header.stamp = ros::Time::now();
	currentSetpoint.header.frame_id = frameId;
	setpointGoalPub.publish(currentSetpoint);

}

// ***********************************************************************
void drone_pose_class::vec_to_vel(tf2::Vector3 inputVector, float& vx, float& vy, float& vz, float& vyaw, float& yaw)
{
	float pi = 3.14159265358979323846;
	
	vx = inputVector.x()*maxXYVelParam;
	vy = 0;
	vz = inputVector.z()*maxZVelParam;
	
	yaw = atan2(inputVector.y(), inputVector.x());
	vyaw = yaw / pi * maxYawRateParam;
	
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
	
	if(mag != 0)
	{
		tf2::Vector3 outputVector(inputVector.x()/mag,
															inputVector.y()/mag,
															inputVector.z()/mag);
		
		inputVector = outputVector;
	}
	return mag;
}

// ***********************************************************************
geometry_msgs::Pose drone_pose_class::transform_world_to_body(geometry_msgs::Pose inputPose, geometry_msgs::Pose robotPose, bool transformDirection)
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
	
	tf2::Vector3 position;
	tf2::Quaternion orientation;
	geometry_msgs::Pose outputPose;
	
	if(transformDirection)
	{
		position = localTransform.inverse()*tf2::Vector3(inputPose.position.x,
																						 				 inputPose.position.y,
																								 		 inputPose.position.z);
		orientation = localTransform.inverse()*tf2::Quaternion(inputPose.orientation.x, 
																													 inputPose.orientation.y, 
																													 inputPose.orientation.z, 
																													 inputPose.orientation.w);
	  outputPose.position.x = position.x();
	  outputPose.position.y = position.y();
	  outputPose.position.z = position.z();
	  
	  outputPose.orientation.x = orientation.x();
	  outputPose.orientation.y = orientation.y();
	  outputPose.orientation.z = orientation.z();
	  outputPose.orientation.w = orientation.w();
	  
	  return outputPose;																						 
	}
	else
	{
		position = localTransform*tf2::Vector3(inputPose.position.x,
																			 		 inputPose.position.y,
																			 		 inputPose.position.z);
		orientation = localTransform*tf2::Quaternion(inputPose.orientation.x, 
																								 inputPose.orientation.y, 
																								 inputPose.orientation.z, 
																								 inputPose.orientation.w);
		outputPose.position.x = position.x();
	  outputPose.position.y = position.y();
	  outputPose.position.z = position.z();
	  
	  outputPose.orientation.x = orientation.x();
	  outputPose.orientation.y = orientation.y();
	  outputPose.orientation.z = orientation.z();
	  outputPose.orientation.w = orientation.w();
	  
	  return outputPose;
	}
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
geometry_msgs::Pose drone_pose_class::get_pose_from_raw(float x, float y, float z, 
																												float roll, float pitch, float yaw)
{
	geometry_msgs::Pose outputPose;
	
	outputPose.position.x = x;
	outputPose.position.y = y;
	outputPose.position.z = z;
	
	tf::Quaternion quadOrientation;
	quadOrientation.setRPY(roll, pitch, yaw);
	
	outputPose.orientation.x = quadOrientation.x();
	outputPose.orientation.y = quadOrientation.y();
	outputPose.orientation.z = quadOrientation.z();
	outputPose.orientation.w = quadOrientation.w();
	
	return outputPose;
}

// *************************************************************************
float drone_pose_class::pose_distance(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2, std::string field)
{
	float x_err = pose2.position.x - pose1.position.x;
	float y_err = pose2.position.y - pose1.position.y;
	float z_err = pose2.position.z - pose1.position.z;
	
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
	
	float yaw_err = currentYaw2 - currentYaw1;
	
	if (field == "heading")
	return abs(yaw_err);
	
	if (field == "position")
	return sqrt(pow(x_err,2) + pow(y_err,2) + pow(z_err,2));
	
	if (field == "planar")
	return sqrt(pow(x_err,2) + pow(y_err,2));
	
	if (field == "height")
	return abs(z_err);
	
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

// ************************************************************************
void drone_pose_class::quat_to_rpy(float qx, float qy, float qz, float qw, 
														 double& roll, double& pitch, double& yaw)
{
	tf::Quaternion quadOrientation(qx, qy, qz, qw);
																	 
	tf::Matrix3x3(quadOrientation).getRPY(roll, pitch, yaw);		
}
	
// ************************************************************************
void drone_pose_class::rpy_to_quat(double roll, double pitch, double yaw,
																	 float& qx, float& qy, float& qz, float& qw)
{
	tf::Quaternion quadOrientation;
	quadOrientation.setRPY(roll, pitch, yaw);
	
	qx = quadOrientation.x();
	qy = quadOrientation.y();
	qz = quadOrientation.z();
	qw = quadOrientation.w();
}
/////////////////////////////////////////////////////////////////////////


