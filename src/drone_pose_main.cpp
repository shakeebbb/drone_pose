#include "drone_pose.h"

drone_pose_class::drone_pose_class(ros::NodeHandle *nh)
{	
	wait_for_params(nh);
		
	// Subscribers
	stateMavrosSub_ = nh->subscribe("mavros_state_in", 10, &drone_pose_class::state_mavros_cb, this);
	joySub_ = nh->subscribe("joy_in", 100, &drone_pose_class::joy_cb, this);
	poseSub_ = nh->subscribe("pose_in", 100, &drone_pose_class::pose_cb, this);
	estopSub_ = nh->subscribe("estop_status_in", 10, &drone_pose_class::estop_cb, this);
	extStateMavrosSub_ = nh->subscribe("mavros_extended_state_in", 10, &drone_pose_class::extended_state_mavros_cb, this);
	batteryStatusSub_ = nh->subscribe("battery_status_in", 10, &drone_pose_class::battery_status_cb, this);
	landingSafetySub_ = nh->subscribe("landing_safety_in", 10, &drone_pose_class::landing_safety_cb, this);
	
	trajSetSub_ = nh->subscribe("traj_set_in", 10, &drone_pose_class::traj_set_cb, this);
	twistSetSub_ = nh->subscribe("twist_set_in", 10, &drone_pose_class::twist_set_cb, this);
		
	// Publishers
	mavrosSetpointPub_ = nh->advertise<geometry_msgs::PoseStamped>("mavros_setpoint_out", 10);
	attSetpointPub_ = nh->advertise<geometry_msgs::PointStamped>("attractor_setpoint_out", 10);
	vizPub_ = nh->advertise<visualization_msgs::MarkerArray>("viz_out", 10);
	
	flightModePub_ = nh->advertise<std_msgs::String>("flight_mode_out", 10);
	estopPub_ = nh->advertise<std_msgs::Bool>("estop_command_out", 10);
		
	// Servers
	flightModeServer_ = nh->advertiseService("flight_mode_service", &drone_pose_class::flightMode_cb, this);
		
	// Clients
	armingClient_ = nh->serviceClient<mavros_msgs::CommandBool>("arming_client");
	setModeClient_ = nh->serviceClient<mavros_msgs::SetMode>("set_mode_client");
	getParamClient_ = nh->serviceClient<mavros_msgs::ParamGet>("get_param_client");
	setParamClient_ = nh->serviceClient<mavros_msgs::ParamSet>("set_param_client");
		
	// Timers
	trajTimer_ = nh->createTimer(ros::Duration(samplingTime_), &drone_pose_class::traj_timer_cb, this);
	trajTimer_.stop();
	
	// Transforms
	tfListenerPtr_ = new tf2_ros::TransformListener(tfBuffer_);
		
	// Initialize Flight Variables	
	init();
}

//*****************************************************************
void drone_pose_class::init() 
{
	robotPose_ = get_pose_from_raw(0, 0, 0, 0, 0, 0);
	twistSet_ = get_twist_from_raw(0, 0, 0, 0, 0, 0);
	
	waypointList_.poses.clear();

	traj_timer_start_stop("start", samplingTime_);
	
	waypointId_ = 0;
	
	joystickVal_[0] = 0;
	joystickVal_[1] = 0;
	joystickVal_[2] = 0;
	joystickVal_[3] = 0;

	//set_max_vel_params(maxXYVelParam, maxZVelParam, maxYawRateParam);
	
	ROS_INFO("Initialized drone_pose");
	ROS_WARN("Flight Mode set to '%c'", flightMode_);
	
	ROS_INFO("Waiting for mavros to connect ...");
	
	std_msgs::Bool localEstopStatus;
	localEstopStatus.data = true; // Red Light
	estopPub_.publish(localEstopStatus);
	
	isSafeToLand_ = true;
	
	flightMode_ = 'V';
	
	joySetpoint_ = 
	get_pose_from_raw(takeoffPosition_[0], takeoffPosition_[1], takeoffPosition_[2], 0, 0, 0);

	publish_setpoint("use_pose", get_twist_from_raw(), joySetpoint_);
	
	std::cout << joySetpoint_.position.x << std::endl;
	std::cout << joySetpoint_.position.y << std::endl;
	std::cout << joySetpoint_.position.z << std::endl;
}

// ********************************************************************
void drone_pose_class::traj_timer_cb(const ros::TimerEvent&)
{
	increment_setpoint(joySetpoint_, 0.02, 0.02, 0.02,
											 0, 0, 0.02, "joy");
											 
	publish_viz(robotPose_);
	if(flightMode_ == 'J')
	{											 
		publish_setpoint("use_pose", get_twist_from_raw(), joySetpoint_);
		return;
	}

	if(flightMode_ == 'V')
	{
		publish_setpoint("use_twist", twistSet_, get_pose_from_raw());
		return;
	}
	//if(!(flightMode_ == 'T' || flightMode_ == 'W' || flightMode_ == 'H' || flightMode_ == 'L'))
	//return;
	
	if(flightMode_ == 'H')
	{
		publish_setpoint("use_twist", get_twist_from_joy(), get_pose_from_raw());	// Publish last known mavros setpoint
		return;
	}
	
	if(flightMode_ == 'L')
	{
		if(extStateMavros_.landed_state == mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND)
			arm_disarm(false);
		else
		{
			geometry_msgs::Twist twist = get_twist_from_raw(0,0,-1*landSpeed_,0,0,0);
			publish_setpoint("use_twist", twist, get_pose_from_raw());
			
			//increment_setpoint(mavSetpoint_, 0,0,-1*landSpeed_*samplingTime_, 0,0,0);
		}
		return;
	}
	
	if(waypointList_.poses.empty())
	{
		publish_setpoint("use_twist", get_twist_from_raw(), get_pose_from_raw());
		return;
	}
	
	if (flightMode_ == 'T')
	{
		std::cout << "Waypoint Array Size : " << waypointList_.poses.size() << std::endl;
		
		if(waypointId_ == waypointList_.poses.size())
		waypointId_ -= 1;
		
		std::cout << "Publishing waypoint " << waypointId_ << std::endl;
		
		publish_setpoint("use_pose", get_twist_from_raw(), waypointList_.poses.at(waypointId_));
		
		waypointId_ += 1;
		
		std::cout << "Next waypoint in queue " << waypointId_ << std::endl;
		return;
	}
	
	if (flightMode_ == 'W')
	{
		std::cout << "Waypoint Array Size : " << waypointList_.poses.size() << std::endl;
		
		//if(currentWaypointId == currentWaypointList.poses.size())
		//currentWaypointId -= 1;
		
		std::cout << "Publishing waypoint " << waypointId_ << std::endl;

		publish_setpoint("use_pose", get_twist_from_raw(), waypointList_.poses.at(0));
		
		if(pose_distance(waypointList_.poses.at(0), robotPose_, "position") < successRadius_)
		{
			waypointList_.poses.erase(waypointList_.poses.begin());
			waypointId_ += 1;
		}
		
		std::cout << "Next waypoint in queue " << waypointId_ << std::endl;
		return;
	}
	
}

// ***********************************************************************
void drone_pose_class::traj_set_cb(const drone_pose::trajectoryMsg& msg)
{	
	geometry_msgs::TransformStamped trajTransform;
	trajTransform.header.stamp = ros::Time::now();
	trajTransform.transform.translation.x = 0;
	trajTransform.transform.translation.y = 0;
	trajTransform.transform.translation.z = 0;
	trajTransform.transform.rotation.x = 0;
	trajTransform.transform.rotation.y = 0;
	trajTransform.transform.rotation.z = 0;
	trajTransform.transform.rotation.w = 1;
	
	if(!msg.header.frame_id.empty()) 
	{
		try
		{
    	trajTransform = tfBuffer_.lookupTransform(frameId_, msg.header.frame_id, ros::Time(0));
    }
    catch (tf2::TransformException &ex)
    { 
      ROS_WARN("%s",ex.what());
      return;
    }
  }
	else
	{
		ROS_WARN("Trajectory received with empty frame_id, expected %s", frameId_.c_str());
		return;
	}
	
	//if(msg.samplingTime != samplingTime_ && msg.samplingTime > 0)
	//	samplingTime_ = msg.samplingTime;
	
	if(msg.samplingTime <= 0)
		ROS_WARN("Trajectory received with invalid sampling time %f", msg.samplingTime);
	
	if(msg.appendRefresh)
	{
		waypointList_.poses.clear();
		waypointId_ = 0;
	}
	
	for (int i = 0; i < msg.poses.size(); i++)
	{	
		geometry_msgs::Pose localTransformedPose;
		tf2::doTransform(msg.poses[i], localTransformedPose, trajTransform);
		waypointList_.poses.push_back(localTransformedPose);
	}
}

// *********************************************************************
void drone_pose_class::publish_setpoint(std::string doc, geometry_msgs::Twist twistIn, geometry_msgs::Pose poseIn)
{
	//cout << "Publishing pose setpoint : (" << currentSetpoint.pose.position.x << ", "
	// 				 	 << currentSetpoint.pose.position.y << ", "
	//					 << currentSetpoint.pose.position.z << ")" << endl;	

	std_msgs::String flMode;
	flMode.data = flightMode_;
	flightModePub_.publish(flMode);

	static geometry_msgs::Pose lastSetpoint = poseIn;
		
	geometry_msgs::PoseStamped mavrosSetpoint;
	mavrosSetpoint.header.stamp = ros::Time::now();
	mavrosSetpoint.header.frame_id = frameId_;
	
	if (doc == "use_pose")
	{	
		is_bounded(poseIn);
		mavrosSetpoint.pose = poseIn;
		mavrosSetpointPub_.publish(mavrosSetpoint);
		
		lastSetpoint = poseIn;
		return;
	}					 
	
	if (doc == "use_twist")
	{
		increment_setpoint(lastSetpoint, twistIn.linear.x*samplingTime_, 
																		 twistIn.linear.y*samplingTime_,
																		 twistIn.linear.z*samplingTime_,
																		 twistIn.angular.x*samplingTime_,
																		 twistIn.angular.y*samplingTime_,
																		 twistIn.angular.z*samplingTime_);
		is_bounded(lastSetpoint);
		mavrosSetpoint.pose = lastSetpoint;
		mavrosSetpointPub_.publish(mavrosSetpoint);
		return;
	}
}
// ***********************************************************************
bool drone_pose_class::arm_disarm(bool field)
{
	mavros_msgs::CommandBool armingCommand;
	armingCommand.request.value = field;
	bool isSuccess = armingClient_.call(armingCommand);
	
	if(field && isSuccess)
	{
		ROS_INFO("ARMED: Service call successful");
		std_msgs::Bool localEstopStatus;
		localEstopStatus.data = false; // Green Light
		estopPub_.publish(localEstopStatus);
	}
	else if(field && !isSuccess)
		ROS_WARN("Could not arm the vehicle: Service call unsuccessful");
	else if(!field && isSuccess)
	{
		ROS_INFO("DISARMED: Service call successful");
		std_msgs::Bool localEstopStatus;
		localEstopStatus.data = true; // Red Light
		estopPub_.publish(localEstopStatus);
	}
	else if(!field && !isSuccess)
		ROS_WARN("Could not disarm the vehicle: Service call unsuccessful");
		
	return isSuccess;
}

// ***********************************************************************
void drone_pose_class::joy_cb(const sensor_msgs::Joy& msg)
{
	mavros_msgs::SetMode setMode;
	mavros_msgs::CommandBool armingCommand;
	
	if(msg.buttons[armButton_] == 1) //ARM = X
	{
		if(set_px4_mode("OFFBOARD"))
		arm_disarm(true);
	}

	if(msg.buttons[disarmButton_] == 1) //DISARM = Y
	{
		if(arm_disarm(false))
		set_px4_mode("STABILIZED");
	}

	if(msg.buttons[aButton_] == 1)
		change_flight_mode('J', true);
		
	if(msg.buttons[landButton_] == 1)
		change_flight_mode('L', true);
		
	if(msg.buttons[bButton_] == 1)
		change_flight_mode('V', true);
		
	if(msg.buttons[sendAttButton_] == 1)
		publish_attractor(joySetpoint_);

	if(abs(msg.axes[xAxis_])>0.2) //X
		joystickVal_[0] = msg.axes[xAxis_];
	else
		joystickVal_[0] = 0;

	if(abs(msg.axes[yAxis_])>0.2) //Y
		joystickVal_[1] = msg.axes[yAxis_];
	else
		joystickVal_[1] = 0;

	if(abs(msg.axes[zAxis_])>0.2)  //Z
		joystickVal_[2] = msg.axes[zAxis_];
	else
		joystickVal_[2] = 0;

	if(abs(msg.axes[yawAxis_])>0.2)  //YAW
		joystickVal_[3] = msg.axes[yawAxis_];
	else
		joystickVal_[3] = 0;
}

// ***********************************************************************
bool drone_pose_class::flightMode_cb(drone_pose::flightModeSrv::Request& req,	
				     drone_pose::flightModeSrv::Response& res)
{
	if (req.flightMode == flightMode_)
		return true;
	
	if (flightMode_ == 'J' || flightMode_ == 'L')
	{
		ROS_WARN_THROTTLE(2, "Cannot externally change mode from %c", flightMode_);
		return true;
	}
	
	if (req.setGet == 0)
	{
		if (!change_flight_mode(req.flightMode))
			ROS_WARN("Unrecognized mode transition requested");
	}
	res.flightMode = flightMode_;
	//ROS_WARN("Flight mode transition request received: set to '%c'", currentFlightMode);
	return true;
}
	

/////////////////////////////////////////////////////////////////////////


