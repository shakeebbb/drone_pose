#ifndef DRONEPOSE_H
#define DRONEPOSE_H

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
#include <tf2_ros/transform_listener.h>

class drone_pose_class
{

private:
	// Subscribers
	ros::Subscriber stateMavrosSub;
	ros::Subscriber joySub;
	ros::Subscriber poseSub;
	ros::Subscriber setpointSub;
	ros::Subscriber trajectorySub;
	ros::Subscriber pfSub;
	
	
	// Publishers
	ros::Publisher setpointPub;
	ros::Publisher setpointGoalPub;
	
	// Servers
	ros::ServiceServer flightModeServer;
	
	// Clients
	ros::ServiceClient armingClient;
	ros::ServiceClient setModeClient;
	ros::ServiceClient setParamClient;
	ros::ServiceClient getParamClient;
	
	// TF Buffer
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener* tfListenerPtr;
	
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
	std::vector<float> xBoundsParam;
	std::vector<float> yBoundsParam;
	std::vector<float> zBoundsParam;
	std::vector<float> takeoffPositionParam;
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
	std::string frameId;
	
public:

	// Constructor/Destructor
	drone_pose_class(ros::NodeHandle*);
	~drone_pose_class();
	
	// Callbacks
	void state_mavros_cb(const mavros_msgs::State&);
	void pose_cb(const geometry_msgs::PoseStamped&);
	void traj_timer_cb(const ros::TimerEvent&);
	void trajectory_cb(const drone_pose::trajectoryMsg&);
	void joy_cb(const sensor_msgs::Joy&);
	bool flightMode_cb(drone_pose::flightModeSrv::Request&,
	 						       drone_pose::flightModeSrv::Response&);
	void pf_cb(const geometry_msgs::TwistStamped&);
									 
	// Other cpp functions
	void wait_for_params(ros::NodeHandle*);
	void init();
	bool isBounded(geometry_msgs::PoseStamped&);
	geometry_msgs::Pose get_pose_from_raw(float, float, float, float, float, float);
	void increment_setpoint(float, float, float, float, float, float, bool);
	static float pose_distance(geometry_msgs::Pose, geometry_msgs::Pose, std::string = "all");
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
	geometry_msgs::Pose transform_world_to_body(geometry_msgs::Pose, geometry_msgs::Pose, bool = true);
	tf2::Vector3 transform_world_to_body(tf2::Vector3, geometry_msgs::Pose, bool);
	bool isPfActive();
	void vec_to_vel(tf2::Vector3, float&, float&, float&, float&, float&);
	void quat_to_rpy(float, float, float, float, double&, double&, double&);
	void rpy_to_quat(double, double, double, float&, float&, float&, float&);
};

#endif
