
#include<ros/ros.h>
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include"sensor_msgs/NavSatFix.h"
#include"sensor_msgs/Imu.h"
#include"novatel_oem7_msgs/BESTPOS.h"
#include"novatel_oem7_msgs/HEADING2.h"
#include"novatel_oem7_msgs/PPPPOS.h"
#include"novatel_oem7_msgs/PositionOrVelocityType.h"
#include"std_msgs/Bool.h"
#include<iostream>
#include<stdlib.h>

#include "tf/transform_datatypes.h"
#include "tf2/LinearMath/Quaternion.h"
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

class novatel_oem7_monitor_class
{
	public:
	novatel_oem7_monitor_class(){

		nov_bestpos_sub = nh.subscribe("/novatel/oem7/bestpos", 1000, &novatel_oem7_monitor_class::nov_bestpos_callback, this);
		nov_heading_sub = nh.subscribe("/novatel/oem7/heading2", 1000, &novatel_oem7_monitor_class::nov_heading_callback, this);
		nov_ppppos_sub = nh.subscribe("/novatel/oem7/ppppos", 1000, &novatel_oem7_monitor_class::nov_ppppos_callback, this);
		nov_fix_pub = nh.advertise<sensor_msgs::NavSatFix>("/oem7/fix", 10);
		nov_heading_pub = nh.advertise<sensor_msgs::Imu>("/oem7/heading", 10);
		nov_ppp_fix_pub = nh.advertise<sensor_msgs::NavSatFix>("/oem7/ppp/fix", 10);
		ros::NodeHandle np("~");
		np.param<double>("narrow_float_cov", narrow_float_cov, 0.05);
		np.param<double>("heading_narrow_float_cov", heading_narrow_float_cov, 1.0);
		np.param<double>("single_cov", single_cov, 0.95);
	}

void nov_bestpos_callback(const novatel_oem7_msgs::BESTPOS::ConstPtr msg)
	{
		novatel_oem7_msgs::PositionOrVelocityType pos_type;
		novatel_oem7_msgs::PositionOrVelocityType type;

		novatel_oem7_msgs::SolutionStatus sol_status;
		//unsigned int narrow_i= 50;
		//int narrow_f = 34;
		//std_msgs::UInt32 narrow_i;
		//narrow_i = 50;
		//NARROW_INT = 50;
		//msg_ins_data.YPR.x = data->Heading;
		pos_type = msg->pos_type;
		sol_status = msg->sol_status;
		msg_nov_bestpos_fix.latitude = msg->lat;
		msg_nov_bestpos_fix.longitude = msg->lon;
		msg_nov_bestpos_fix.altitude = msg->hgt;
		msg_nov_bestpos_fix.position_covariance_type = 2;
		//ROS_INFO("msg->pos_type");
		//ROS_INFO_STREAM(msg->pos_type);
		//ROS_INFO("type.NARROW_INT");
		//ROS_INFO_STREAM(type.NARROW_INT);
		//ROS_INFO("type.NARROW_FLOAT");
		//ROS_INFO_STREAM(type.NARROW_FLOAT);
		//ROS_INFO("narrow_float_cov");
		//ROS_INFO_STREAM(narrow_float_cov);
		//ROS_INFO(narrow_float_cov);
		if (msg->pos_type.type == type.NARROW_INT)
		{
			//ROS_INFO("NARROW_INT");
			msg_nov_bestpos_fix.position_covariance = {pow(msg->lon_stdev,2),0,0,pow(msg->lat_stdev,2),0,0,pow(msg->hgt_stdev,2),0,0};
		}  else if (msg->pos_type.type == type.NARROW_FLOAT)
		{
			//ROS_INFO("NARROW_FLOAT");
			msg_nov_bestpos_fix.position_covariance = {pow(msg->lon_stdev+narrow_float_cov,2),0,0,pow(msg->lat_stdev+narrow_float_cov,2),0,0,pow(msg->hgt_stdev,2),0,0};
		} else
		{
			//ROS_INFO("else");
			msg_nov_bestpos_fix.position_covariance = {pow(msg->lon_stdev+single_cov,2),0,0,pow(msg->lat_stdev+single_cov,2),0,0,pow(msg->hgt_stdev,2),0,0};
		}


		nov_fix_pub.publish(msg_nov_bestpos_fix);
	}
	void nov_heading_callback(const novatel_oem7_msgs::HEADING2::ConstPtr msg)
		{
			float nov_heading;
			float nov_heading_std;

			novatel_oem7_msgs::PositionOrVelocityType heading_type;
			nov_heading = msg->heading;
			nov_heading_std = msg->heading_stdev;

			if (msg->pos_type.type == heading_type.NARROW_INT)
			{
				msg_nov_heading_imu.orientation_covariance = {0,0,0,0,0,0,nov_heading_std,0,0};
			} else
			{
				msg_nov_heading_imu.orientation_covariance = {0,0,0,0,0,0,nov_heading_std + heading_narrow_float_cov,0,0};
			}

			/// euler to quaternion ///
			tf2::Quaternion myQuaternion;
			myQuaternion.setRPY(0.0,0.0,nov_heading*M_PI/180);
			myQuaternion=myQuaternion.normalize();
			//\\ euler to quaternion \\\

			//msg_ins_fix.position_covariance = {msg->Pos_STD.x,msg->Pos_STD.y,msg->Pos_STD.z};
			msg_nov_heading_imu.orientation.x = myQuaternion[0];
			msg_nov_heading_imu.orientation.y = myQuaternion[1];
			msg_nov_heading_imu.orientation.z = myQuaternion[2];
			msg_nov_heading_imu.orientation.w = myQuaternion[3];

			nov_heading_pub.publish(msg_nov_heading_imu);

		}
		void nov_ppppos_callback(const novatel_oem7_msgs::PPPPOS::ConstPtr msg)
			{

				msg_nov_ppppos_fix.latitude = msg->lat;
				msg_nov_ppppos_fix.longitude = msg->lon;
				msg_nov_ppppos_fix.altitude = msg->hgt;
				msg_nov_ppppos_fix.position_covariance_type = 2;
				msg_nov_ppppos_fix.position_covariance = {pow(msg->lat_stdev,2),pow(msg->lon_stdev,2),pow(msg->hgt_stdev,2)};

				nov_ppp_fix_pub.publish(msg_nov_ppppos_fix);
			}

private:
	ros::NodeHandle nh;

	ros::Subscriber nov_bestpos_sub;
	ros::Subscriber nov_heading_sub;
	ros::Subscriber nov_ppppos_sub;
	ros::Publisher nov_fix_pub;
	ros::Publisher nov_heading_pub;
	ros::Publisher nov_ppp_fix_pub;

	sensor_msgs::NavSatFix msg_nov_bestpos_fix;
	sensor_msgs::NavSatFix msg_nov_ppppos_fix;
	sensor_msgs::Imu msg_nov_heading_imu;
	novatel_oem7_msgs::PositionOrVelocityType pos_type;
	novatel_oem7_msgs::SolutionStatus sol_status;
	double narrow_float_cov;
	double heading_narrow_float_cov;
	double single_cov;


	float nov_heading;
	float nov_heading_std;
	//std_msgs::Float32 imu_yaw_msg_;
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "gnss_monitor");
	//ros::NodeHandle main_nh;
	ROS_INFO("gnss_monitor initialized");



	novatel_oem7_monitor_class gnss_monitor_object;


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
