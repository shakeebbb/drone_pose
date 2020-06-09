#include "drone_pose.h"


// *********************************************************************
void drone_pose_class::wait_for_params(ros::NodeHandle *nh)
{
	while(!nh->getParam("x_bounds", xBounds_));
	while(!nh->getParam("y_bounds", yBounds_));
	while(!nh->getParam("z_bounds", zBounds_));
	
	while(!nh->getParam("takeoff_position", takeoffPosition_));

	while(!nh->getParam("arm_button", armButton_));
	while(!nh->getParam("disarm_button", disarmButton_));

	while(!nh->getParam("A_button", aButton_));
	while(!nh->getParam("B_button", bButton_));
	while(!nh->getParam("land_button", landButton_));
	while(!nh->getParam("send_attractor_button", sendAttButton_));

	while(!nh->getParam("x_axis", xAxis_));
	while(!nh->getParam("y_axis", yAxis_));
	while(!nh->getParam("z_axis", zAxis_));
	while(!nh->getParam("yaw_axis", yawAxis_));
	
	while(!nh->getParam("success_radius", successRadius_));
	while(!nh->getParam("sampling_time", samplingTime_));

	while(!nh->getParam("land_speed", landSpeed_));
	
	while(!nh->getParam("low_battery_voltage", lowBatteryVoltage_));
	while(!nh->getParam("critical_battery_voltage", criticalBatteryVoltage_));
	
	while(!nh->getParam("world_frame_id", frameId_));
	
	ROS_INFO("Parameters for drone_pose retreived from the parameter server");
}

// ***********************************************************************
void drone_pose_class::landing_safety_cb(const std_msgs::Bool& msg)
{
	isSafeToLand_ = msg.data;
}

// ***********************************************************************
void drone_pose_class::battery_status_cb(const sensor_msgs::BatteryState& msg)
{
	if( (msg.voltage < lowBatteryVoltage_) && isSafeToLand_)
	{
		if (flightMode_ != 'L')
			change_flight_mode('L');
		return;
	}
	
	if(msg.voltage < criticalBatteryVoltage_)
	{
		if (flightMode_ != 'L')
			change_flight_mode('L');
		return;
	}
}

// ***********************************************************************
bool drone_pose_class::set_px4_mode(std::string field)
{
	mavros_msgs::SetMode setMode;
	setMode.request.custom_mode = field;
	bool isSuccess = setModeClient_.call(setMode);
	
	if(isSuccess)
		ROS_INFO("%s: Service call successfull", field.c_str());
	else
		ROS_INFO("%s: Service call unsuccessfull", field.c_str());
		
	return isSuccess;
}

// ***********************************************************************
void drone_pose_class::estop_cb(const std_msgs::Bool& msg)
{
	if(msg.data == true && flightMode_ != 'L')
		change_flight_mode('L');
}

// ***********************************************************************
void drone_pose_class::twist_set_cb(const geometry_msgs::TwistStamped& msg)
{
	if (msg.header.frame_id != frameId_)
	{
		ROS_WARN("Unexpected twist set frame id");
		
		twistSet_.linear.x = 0; twistSet_.linear.y = 0; twistSet_.linear.z = 0; 
		twistSet_.angular.x = 0; twistSet_.angular.y = 0; twistSet_.angular.z = 0;
		return;
	}
	twistSet_ = msg.twist;
}

// ***********************************************************************
drone_pose_class::~drone_pose_class()
{
	delete tfListenerPtr_;
}

// *******************************************************************
bool drone_pose_class::is_bounded(geometry_msgs::Pose& point)
{
	//float x_ref = setpoint.pose.position.x;
	//float y_ref = setpoint.pose.position.y;
	//float z_ref = setpoint.pose.position.z;

	if(point.position.x < xBounds_[0])
	point.position.x = xBounds_[0];
	if(point.position.x > xBounds_[1])
	point.position.x = xBounds_[1];

	if(point.position.y < yBounds_[0])
	point.position.y = yBounds_[0];
	if(point.position.y > yBounds_[1])
	point.position.y = yBounds_[1];

	if(point.position.z < zBounds_[0])
	point.position.z = zBounds_[0];
	if(point.position.z > zBounds_[1])
	point.position.z = zBounds_[1];
	return 0;
}

// ***********************************************************************

void drone_pose_class::pose_cb(const geometry_msgs::PoseStamped& msg)
{
	robotPose_ = msg.pose;
}

// ***********************************************************************
void drone_pose_class::state_mavros_cb(const mavros_msgs::State& msg)
{
	stateMavros_ = msg;
	
	static bool status = false;
	
	if(!status && stateMavros_.connected)
	ROS_INFO("Mavros connected");
	
	else if(status && !stateMavros_.connected)
	ROS_WARN("Mavros disconnected");
	
	status = stateMavros_.connected;
}

// ***********************************************************************
void drone_pose_class::extended_state_mavros_cb(const mavros_msgs::ExtendedState& msg)
{
	extStateMavros_ = msg;
}

// ***********************************************************************
char drone_pose_class::get_current_flight_mode()
{
	return flightMode_;
}

// ***********************************************************************
float drone_pose_class::get_current_sampling_time()
{
	return samplingTime_;
}

// ***********************************************************************
void drone_pose_class::increment_setpoint(geometry_msgs::Pose& setpoint, 
																					float dx, float dy, float dz, 
																					float droll, float dpitch, float dyaw, std::string doc)
{	
	//cout << "Joystick Value: (" << joystickVal[0] << ", " 
	//			      << joystickVal[1] << ", "
	//			      << joystickVal[2] << ", "
	//			      << joystickVal[3] << ")" << endl;
	
	if(doc == "joy")
	{
		dx = dx*joystickVal_[0];
		dy = dy*joystickVal_[1];
		dz = dz*joystickVal_[2];
		
		droll = 0;
		dpitch = 0;
		dyaw = dyaw*joystickVal_[3];
	}
	
  setpoint.position.x += dx;
  setpoint.position.y += dy;
  setpoint.position.z += dz;
  
  double localRoll, localPitch, localYaw;
  quat_to_rpy(setpoint.orientation.x,
     	      	setpoint.orientation.y,
  	     			setpoint.orientation.z,
  	      		setpoint.orientation.w,
  	      		localRoll, localPitch, localYaw);
  
  localRoll += droll;
  localPitch += dpitch;
  localYaw += dyaw;
  
  float localQx, localQy, localQz, localQw;
  rpy_to_quat(localRoll, localPitch, localYaw,
  	      localQx, localQy, localQz, localQw);
  						
  setpoint.orientation.x = localQx;
  setpoint.orientation.y = localQy;
  setpoint.orientation.z = localQz;
  setpoint.orientation.w = localQw;
}

// ***********************************************************************
void drone_pose_class::traj_timer_start_stop(std::string doc, float samplingTime)
{	
	static bool timerStatus = true;
	
	if (doc == "start")
	{
		samplingTime_ = samplingTime;
		trajTimer_.setPeriod(ros::Duration(samplingTime_), false);
		trajTimer_.start();
	
		if(!timerStatus)
			ROS_INFO("Trajectory timer started with sampling time %f", samplingTime_);
	
		timerStatus = true;
		return;
	}
	if (doc == "stop")
	{
		trajTimer_.stop();
	
		if(timerStatus)
			ROS_INFO("Trajectory timer stopped");
	
		timerStatus = false;
		return;
	}
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
geometry_msgs::Twist drone_pose_class::get_twist_from_raw(float x, float y, float z, 
							float roll, float pitch, float yaw)
{
	geometry_msgs::Twist twist;
	twist.linear.x = x; twist.linear.y = y; twist.linear.z = z;
	twist.angular.x = x; twist.angular.y = y; twist.angular.z = z;
	return twist;
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

geometry_msgs::Twist drone_pose_class::get_twist_from_joy()
{
	geometry_msgs::Twist twist;
	
	twist.linear.x = joystickVal_[0];
	twist.linear.y = joystickVal_[1];
	twist.linear.z = joystickVal_[2];
	twist.angular.z = joystickVal_[3];
	return twist;
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

// ************************************************************************
geometry_msgs::Pose drone_pose_class::pose_to_setpoint(geometry_msgs::Pose poseIn)
{
	geometry_msgs::Pose setpointOut; 
	
	double localRoll, localPitch, localYaw;
	quat_to_rpy(poseIn.orientation.x,
  						poseIn.orientation.y,
  						poseIn.orientation.z,
  						poseIn.orientation.w,
  						localRoll,
  						localPitch,
  						localYaw);
			
	setpointOut =
	get_pose_from_raw(poseIn.position.x, poseIn.position.y, poseIn.position.z, 
										0, 0, localYaw);
										
	return setpointOut;
}
// ************************************************************************
void drone_pose_class::publish_attractor(geometry_msgs::Pose poseIn)
{
	geometry_msgs::PointStamped pointOut;
	pointOut.header.stamp = ros::Time::now();
	pointOut.header.frame_id = frameId_;
	
	pointOut.point = poseIn.position;
	attSetpointPub_.publish(pointOut);
}
// ************************************************************************
bool drone_pose_class::change_flight_mode(char flModeIn)
{
	if (flModeIn == 'J')
	{
		joySetpoint_ = pose_to_setpoint(robotPose_);
		publish_setpoint("use_pose", get_twist_from_raw(), joySetpoint_);
		ROS_INFO("Joystick marker set to current pose");
		flightMode_ = flModeIn;
		return true;
	}
		
	if (flModeIn == 'H' || flModeIn == 'L' || flModeIn == 'V')
	{
		publish_setpoint("use_pose", get_twist_from_raw(), pose_to_setpoint(robotPose_));
		flightMode_ = flModeIn;
		return true;
	}

	return false;
}

// ************************************************************************
void drone_pose_class::publish_viz(geometry_msgs::Pose poseIn)
{

	visualization_msgs::MarkerArray markerArr;
	
	visualization_msgs::Marker joyMarker;
	
	joyMarker.header.stamp = ros::Time::now();
	joyMarker.header.frame_id = frameId_;
	joyMarker.ns = "joy";
	joyMarker.id = 0;
	joyMarker.type = visualization_msgs::Marker::ARROW;
	joyMarker.action = visualization_msgs::Marker::ADD;
	joyMarker.pose = joySetpoint_;
	joyMarker.lifetime = ros::Duration(0);
	
	joyMarker.color.r = 0; 
	joyMarker.color.g = 0; 
	joyMarker.color.b = 1; 
	joyMarker.color.a = 1;
		
	joyMarker.scale.x = 0.5;
	joyMarker.scale.y = 0.05;
	joyMarker.scale.z = 0.05;
	markerArr.markers.push_back(joyMarker); //.......
	
	visualization_msgs::Marker twistSetMarker;
	twistSetMarker.header.stamp = ros::Time::now();
	twistSetMarker.header.frame_id = frameId_;
	twistSetMarker.ns = "twist_set";
	twistSetMarker.id = 0;
	twistSetMarker.type = visualization_msgs::Marker::LINE_STRIP;
	twistSetMarker.action = visualization_msgs::Marker::ADD;
	twistSetMarker.pose.orientation.w = 1;
	twistSetMarker.lifetime = ros::Duration(0);
	
	twistSetMarker.color.r = 0; 
	twistSetMarker.color.g = 1; 
	twistSetMarker.color.b = 0; 
	twistSetMarker.color.a = 1;
	
	twistSetMarker.points.clear();
	
	geometry_msgs::Point geoPt;
	geoPt.x = 1; 
	geoPt.y = 0; 
	geoPt.z = 0;
	twistSetMarker.points.push_back(geoPt);
	twistSetMarker.points.push_back(geoPt);
	
	float theta = 0;
	while(abs(theta) < abs(twistSet_.angular.z))
	{
		geoPt.x = cos(theta);
		geoPt.y = sin(theta); 
		geoPt.z = 0;
		twistSetMarker.points.push_back(geoPt);
		
		if(twistSet_.angular.z >= 0)
			theta+=0.1;
		else
			theta-=0.1;
	}
	
	twistSetMarker.scale.x = 0.03;
	twistSetMarker.scale.y = 0;
	twistSetMarker.scale.z = 0;
	markerArr.markers.push_back(twistSetMarker); //.......
	
	twistSetMarker.id = 1;
	twistSetMarker.type = visualization_msgs::Marker::ARROW;
	twistSetMarker.points.clear();

	geoPt.x = 0; 
	geoPt.y = 0; 
	geoPt.z = 0;
	twistSetMarker.points.push_back(geoPt);
	
	twistSetMarker.scale.x = 0.03;
	twistSetMarker.scale.y = 0.06;
	twistSetMarker.scale.z = 0.09;
	
	geoPt.x = twistSet_.linear.x; 
	geoPt.y = twistSet_.linear.y; 
	geoPt.z = twistSet_.linear.z;
	twistSetMarker.points.push_back(geoPt);
	markerArr.markers.push_back(twistSetMarker); //.......
	
	visualization_msgs::Marker textMarker;
	
	textMarker.header.stamp = ros::Time::now();
	textMarker.header.frame_id = frameId_;
	textMarker.ns = "text";
	textMarker.id = 0;
	textMarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	textMarker.action = visualization_msgs::Marker::ADD;
	textMarker.lifetime = ros::Duration(0);
	
	textMarker.text = flightMode_;
	
	textMarker.color.r = 1; 
	textMarker.color.g = 1; 
	textMarker.color.b = 1; 
	textMarker.color.a = 1;

	textMarker.scale.z = 1.0;
	markerArr.markers.push_back(textMarker); //.......
	
	vizPub_.publish(markerArr);
	
	
}
/////////////////////////////////////////////////////////////////////////


