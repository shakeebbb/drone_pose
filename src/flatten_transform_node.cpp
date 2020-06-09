
#include "nav_msgs/Path.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "drone_pose.h"

class goal_to_vel_class
{

private:
	ros::NodeHandle* nh_;
	
	// Publishers, Subscribers, Transforms, Timers
	ros::Publisher attPub_;
	ros::Publisher repPub_;
	
	ros::Subscriber goalSub_;
	ros::Subscriber repSub_;
	
	ros::ServiceClient flModClient_;
	
	tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener* tfListenerPtr_;
  
  ros::Timer timer;
	
	// Params
	bool mergeVecs_;
	float maxYaw_; //x,y,z,yaw
	std::string baseFrameId_;
	std::string flatBaseFrameId_;
	std::string worldFrameId_;
	float pubRate_;
	bool isHolonomic_;
	float foreVelDeadband_;
	
	// Local Variables
	geometry_msgs::Vector3 attVec_;
	std::vector<geometry_msgs::Vector3> repVecs_;
	std::vector<std::string> camFrameIds_;
	
	bool isInitialized_;
	
public:

	// ************************************************
	goal_to_vel_class(ros::NodeHandle* nh)
	{
		nh_ = nh;
		
		while( !nh->getParam("publish_rate", pubRate_) );
		while( !nh->getParam("max_yaw", maxYaw_) );
		while( !nh->getParam("merge_output_vectors", mergeVecs_) );
		while( !nh->getParam("holonomic", isHolonomic_) );
		while( !nh->getParam("yaw_err_bound_nonzero_fore_vel_in_rad", foreVelDeadband_) );
		while( !nh->getParam("flat_base_frame_id", flatBaseFrameId_) );
		while( !nh->getParam("base_frame_id", baseFrameId_) );
		while( !nh->getParam("world_frame_id", worldFrameId_) );
		
		ROS_INFO("Parameters for %s retrieved from parameter server", nh->getNamespace().c_str());
		
		repVecs_.clear();
		camFrameIds_.clear();
		
		attPub_ = nh->advertise<geometry_msgs::TwistStamped>("att_vel_out", 10);
		repPub_ = nh->advertise<geometry_msgs::TwistStamped>("rep_vel_out", 10);
		
		goalSub_ = nh->subscribe("goal_pt_in", 1, &goal_to_vel_class::goal_cb, this);
		repSub_ = nh->subscribe("rep_vel_in", 1, &goal_to_vel_class::rep_cb, this);
		
		timer = nh->createTimer(ros::Duration(pubRate_), &goal_to_vel_class::timer_cb, this);
		
		tfListenerPtr_ = new tf2_ros::TransformListener(tfBuffer_);
		
		isInitialized_ = false;
	}

	// ************************************************
	void goal_cb(const geometry_msgs::PointStamped& msg)
	{
		try
		{
			geometry_msgs::TransformStamped transform = tfBuffer_.lookupTransform(baseFrameId_, msg.header.frame_id, ros::Time(0)); 
			
			geometry_msgs::PointStamped msgBase;
			
			tf2::doTransform(msg, msgBase, transform); 
			attVec_.x = msgBase.point.x;
			attVec_.y = msgBase.point.y;
			attVec_.z = msgBase.point.z;
			 
			isInitialized_ = true;  	
    }
    catch (tf2::TransformException &ex)
    { 
      ROS_WARN_THROTTLE(1, "%s",ex.what());
      attVec_.x = 0;
      attVec_.y = 0;
      attVec_.z = 0;
    }
	}
	
	// ************************************************
	void rep_cb(const geometry_msgs::Vector3Stamped& msg)
	{
		int frameIndx;
		std::string frameId = msg.header.frame_id;
		bool doesExist = false;
		
		geometry_msgs::Vector3Stamped msgBase;
		try
		{
			geometry_msgs::TransformStamped transform = tfBuffer_.lookupTransform(baseFrameId_, msg.header.frame_id, ros::Time(0)); 
			
			tf2::doTransform(msg, msgBase, transform);    	
    }
    catch (tf2::TransformException &ex)
    { 
      ROS_WARN_THROTTLE(1, "%s",ex.what());
      return;
    }
		
		for (int i=0; i<camFrameIds_.size(); i++)
			if (frameId == camFrameIds_[i])
			{
				doesExist = true;
				frameIndx = i;
				break;
			}
	
		if (doesExist)
			repVecs_[frameIndx] = msgBase.vector;
		else
		{
			repVecs_.push_back(msgBase.vector);
			camFrameIds_.push_back(frameId);
		}
	}
	
	// ************************************************
	void timer_cb(const ros::TimerEvent&)
	{
		if (!isInitialized_)
		{
			ROS_WARN_THROTTLE(1, "%s : Waiting for first lookahead point ...", nh_->getNamespace().c_str());
			return;
		}
	
		change_drone_pose_mode('V');
		
		int maxMagIndx;
		bool repExists = find_max_mag_index(repVecs_, maxMagIndx);
		
		geometry_msgs::TwistStamped attMsg, repMsg;
		attMsg.header.stamp = ros::Time::now();
		repMsg.header.stamp = ros::Time::now();
		
		attMsg.header.frame_id = baseFrameId_;
		repMsg.header.frame_id = baseFrameId_;
		
		clear_msg(repMsg);
		if (repExists && !mergeVecs_)
			repMsg.twist.linear = repVecs_[maxMagIndx];
		
		clear_msg(attMsg);
		if (mergeVecs_)
			attMsg.twist.linear = add_vecs(attVec_, repVecs_[maxMagIndx]);
		else
			attMsg.twist.linear = attVec_;
		
		bool nonHolAttSuccess = true;
		bool nonHolRepSuccess = true;	
		if (!isHolonomic_)
		{
			nonHolAttSuccess = convert_to_nonholonomic(attMsg.twist.linear, attMsg.twist);
			nonHolRepSuccess = convert_to_nonholonomic(repMsg.twist.linear, repMsg.twist);
		}

		if ( !nonHolAttSuccess || !nonHolRepSuccess )
			return;
			
		attPub_.publish(attMsg);
		repPub_.publish(repMsg);
	}
	
	// ************************************************
	bool convert_to_nonholonomic(geometry_msgs::Vector3 velIn, geometry_msgs::Twist& velOut) // In: Base frame, Out: World Frame
	{
		float pi = 3.14159265358979323846;
		
		geometry_msgs::Vector3 velInFlat;
		try
		{
			geometry_msgs::TransformStamped transform = tfBuffer_.lookupTransform(flatBaseFrameId_, baseFrameId_, ros::Time(0)); 
			tf2::doTransform(velIn, velInFlat, transform);    	
    }
    catch (tf2::TransformException &ex)
    { 
      ROS_WARN_THROTTLE(1, "%s",ex.what());
      return false;
    }

    float yawErr = atan2(velInFlat.y, velInFlat.x);
    
    velOut.linear.x = velInFlat.x;
    if ( abs(yawErr) > foreVelDeadband_ )
    	velOut.linear.x = 0;
    
    velOut.linear.y = 0;
    velOut.linear.z = velInFlat.z;
    
    velOut.angular.x = 0;
    velOut.angular.y = 0;
    velOut.angular.z = yawErr / pi * maxYaw_;
    
    return true;
	}
	
	// ************************************************
	bool change_drone_pose_mode(char mode)
	{	
		drone_pose::flightModeSrv flModSrv;
		flModSrv.request.setGet = 0;
		flModSrv.request.flightMode = 'V';
		
		flModClient_.call(flModSrv);
			
		if(flModSrv.response.flightMode != 'V')
		{		
			ROS_WARN_THROTTLE(1, "%s: Could not change flight mode. Trying again ...", nh_->getNamespace().c_str());
			return false;
		}
		
		return true;
	}
	
	// ************************************************
	bool find_max_mag_index(std::vector<geometry_msgs::Vector3> vecsIn, int& maxMagIndx)
	{	
		maxMagIndx = 0;
		float maxMagSq = 0;
		
		if (vecsIn.size() == 0)
		return false;
		
		for (int i=0; i<vecsIn.size(); i++)
		{
			float vecMagSq = pow(vecsIn[i].x,2) + pow(vecsIn[i].y,2) + pow(vecsIn[i].z,2);
			if ( vecMagSq > maxMagSq )
			{
				maxMagSq = vecMagSq;
				maxMagIndx = i;
			}
		}
		
		return true;
	}
	
	// ************************************************
	void clear_msg(geometry_msgs::TwistStamped& msg)
	{
		msg.twist.linear.x = 0;
		msg.twist.linear.y = 0;
		msg.twist.linear.z = 0;
		
		msg.twist.angular.x = 0;
		msg.twist.angular.y = 0;
		msg.twist.angular.z = 0;
	}
	
	// ************************************************
	geometry_msgs::Vector3 add_vecs(const geometry_msgs::Vector3& vec1, const geometry_msgs::Vector3& vec2)
	{
		geometry_msgs::Vector3 vecRes;
		vecRes.x = vec1.x + vec2.x;
		vecRes.y = vec1.y + vec2.y;
		vecRes.y = vec1.z + vec2.z;
		
		return vecRes;
	}
	
	// ************************************************
};

// Main
int main(int argc, char **argv)
{
	ros::init(argc, argv, "flatten_transform_node");
	ros::NodeHandle nh;

	while( !nh->getParam("publish_rate", pubRate_) );
	while( !nh->getParam("fixed_frame_id", fixedFrameId_) );
	while( !nh->getParam("source_frame_id", sourceFrameId_) );
	while( !nh->getParam("flat_frame_id", flatFrameId_) );
	
	ROS_INFO("Parameters for %s retrieved from parameter server", nh->getNamespace().c_str());
	
	tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener = tf2_ros::TransformListener(tfBuffer_);
  
  while (nh.ok())
  {
  	geometry_msgs::TransformStamped transform = tfBuffer_.lookupTransform();
  }
	
	ros::spin();
	return 0;
}


