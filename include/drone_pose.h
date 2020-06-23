#ifndef DRONEPOSE_H
#define DRONEPOSE_H

#include <math.h>
#include <time.h>
#include <string>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <tf/transform_datatypes.h>
#include <drone_pose/trajectoryMsg.h>
#include <drone_pose/flightModeSrv.h>
#include <mavros_msgs/ParamSet.h>
#include <mavros_msgs/ParamGet.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/BatteryState.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

class drone_pose_class
{

private:
	// Subscribers
	ros::Subscriber stateMavrosSub_;
	ros::Subscriber joySub_;
	ros::Subscriber poseSub_;
	ros::Subscriber estopSub_;
	ros::Subscriber extStateMavrosSub_;
	ros::Subscriber batteryStatusSub_;
	ros::Subscriber landingSafetySub_;
	
	ros::Subscriber trajSetSub_;
	ros::Subscriber twistSetSub_;
	
	// Publishers
	ros::Publisher mavrosSetpointPub_;
	ros::Publisher attSetpointPub_;
	ros::Publisher vizPub_;

	ros::Publisher flightModePub_;
	ros::Publisher estopPub_;
	
	// Servers
	ros::ServiceServer flightModeServer_;
	
	// Clients
	ros::ServiceClient armingClient_;
	ros::ServiceClient setModeClient_;
	ros::ServiceClient setParamClient_;
	ros::ServiceClient getParamClient_;
	
	// TF Buffer
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener* tfListenerPtr_;
	
	// Timers
	ros::Timer trajTimer_;
	
	// Mission variables
	geometry_msgs::Pose robotPose_;
	int waypointId_;
	geometry_msgs::PoseArray waypointList_;
	float samplingTime_;
	char flightMode_;
	bool isSafeToLand_;
	float joystickVal_[4];
	
	// Status variables
	mavros_msgs::State stateMavros_;
	mavros_msgs::ExtendedState extStateMavros_;
	geometry_msgs::Twist twistSet_;
	geometry_msgs::Pose joySetpoint_;
	
	// From Parameter Server
	std::vector<float> xBounds_;
	std::vector<float> yBounds_;
	std::vector<float> zBounds_;
	std::vector<float> takeoffPosition_;
	float successRadius_;
	int armButton_;
	int disarmButton_;
	int landButton_;
	int aButton_;
	int bButton_;
	int sendAttButton_;
	int xAxis_;
	int yAxis_;
	int zAxis_;
	int yawAxis_;
	float landSpeed_;
	std::string frameId_;
	float lowBatteryVoltage_;
	float criticalBatteryVoltage_;
	
public:

	// Constructor/Destructor
	drone_pose_class(ros::NodeHandle*);
	~drone_pose_class();
	
	// Callbacks
	void state_mavros_cb(const mavros_msgs::State&);
	void extended_state_mavros_cb(const mavros_msgs::ExtendedState&);
	void pose_cb(const geometry_msgs::PoseStamped&);
	void traj_timer_cb(const ros::TimerEvent&);
	void traj_set_cb(const drone_pose::trajectoryMsg&);
	void joy_cb(const sensor_msgs::Joy&);
	bool flightMode_cb(drone_pose::flightModeSrv::Request&,
	 						       drone_pose::flightModeSrv::Response&);
	void twist_set_cb(const geometry_msgs::TwistStamped&);
	void estop_cb(const std_msgs::Bool&);
	void battery_status_cb(const sensor_msgs::BatteryState&);
	void landing_safety_cb(const std_msgs::Bool&);
									 
	// Other cpp functions
	void wait_for_params(ros::NodeHandle*);
	void init();
	bool is_bounded(geometry_msgs::Pose&);
	geometry_msgs::Pose get_pose_from_raw(float=0, float=0, float=0, float=0, float=0, float=0);
	geometry_msgs::Twist get_twist_from_raw(float=0, float=0, float=0, float=0, float=0, float=0);
	void increment_setpoint(geometry_msgs::Pose&, float, float, float, float, float, float, std::string = "add");
	static float pose_distance(geometry_msgs::Pose, geometry_msgs::Pose, std::string = "all");
	float get_current_sampling_time();
	char get_current_flight_mode();
	void publish_setpoint(std::string, geometry_msgs::Twist, geometry_msgs::Pose);
	void traj_timer_start_stop(std::string, float);
	void update_twist_set_from_joy();
	void quat_to_rpy(float, float, float, float, double&, double&, double&);
	void rpy_to_quat(double, double, double, float&, float&, float&, float&);
	bool arm_disarm(bool);
	bool set_px4_mode(std::string);
	void publish_viz(geometry_msgs::Pose);
	bool change_flight_mode(char, bool=false);
	geometry_msgs::Pose pose_to_setpoint(geometry_msgs::Pose);
	geometry_msgs::Twist get_twist_from_joy();
	void publish_attractor(geometry_msgs::Pose);
};

#endif
