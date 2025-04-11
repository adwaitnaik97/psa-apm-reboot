#include<ros/ros.h>
#include "std_msgs/Float32.h"
#include"sensor_msgs/Imu.h"
#include"std_msgs/Bool.h"


#include "tf/transform_datatypes.h"
#include <string>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <numeric>
#include <vector>
//#include <boost/accumulators/statistics/variance.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>
#include <math.h>
template <class T> T String_To_Num(std::string str);

class Honeywell_rad_to_deg_class
{
	public:
	Honeywell_rad_to_deg_class(){

		//double HG_imu_deg

		
		imu_sub_ = nh.subscribe("/HGuide/Std/Imu", 1000, &Honeywell_rad_to_deg_class::imu_callback, this);
		imu_yaw_pub_ = nh.advertise<std_msgs::Float32>("/HGuide/Std/Imu/yaw_deg", 10);

	}


void imu_callback(const sensor_msgs::Imu::ConstPtr msg)
	{

	    double imu_roll, imu_pitch, imu_yaw;
	    tf::Quaternion imu_orientation(msg->orientation.x, msg->orientation.y,
	                                   msg->orientation.z, msg->orientation.w);
	    tf::Matrix3x3(imu_orientation).getRPY(imu_roll, imu_pitch, imu_yaw);
	    

	    // Get the orientation of the IMU an publish in degree
	    imu_yaw_msg_.data = static_cast<float>(imu_yaw * 180 / M_PI);
	    imu_yaw_pub_.publish(imu_yaw_msg_);
		
	}



private:
	ros::NodeHandle nh;
	ros::Subscriber imu_sub_;
	ros::Publisher imu_yaw_pub_;
	std_msgs::Float32 imu_yaw_msg_;
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "honeywell_rad_to_deg");
	//ros::NodeHandle main_nh;
	ROS_INFO("Honeywell_rad_to_deg initialized");
	Honeywell_rad_to_deg_class Honeywell_rad_to_deg_object;
	
	ros::spin();
}

template <class T>
T String_To_Num(std::string str)
{
	std::stringstream ss(str);
	T converted_int;
	ss >> converted_int;
	return converted_int;
}



