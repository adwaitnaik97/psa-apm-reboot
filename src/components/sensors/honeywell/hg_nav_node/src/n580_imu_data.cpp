/*
	Simple example to send inertial data to tutrtle
*/

/*
HONEYWELL hereby grants to you a, perpetual, free of charge, worldwide, irrevocable, non-exclusive license to use, copy, modify, merge,
publish, distribute, sublicense the software and associated documentation (the Software), subject to the following conditions:

YOU AGREE THAT YOU ASSUME ALL THE RESPONSIBILITY AND RISK FOR YOUR USE OF THE SOFTWARE AND THE RESULTS AND PERFORMANCE THEREOF.
THE SOFTWARE IS PROVIDED TO YOU ON AN AS IS AND AS AVAILABLE BASIS, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
WITHOUT LIMITATION ANY IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.

IN NO EVENT WILL HONEYWELL BE LIABLE TO YOU FOR ANY DIRECT, SPECIAL, INDIRECT, INCIDENTAL, CONSEQUENTIAL, EXEMPLARY OR PUNITIVE DAMAGES,
INCLUDING, WITHOUT LIMITATION, DAMAGES FOR LOST DATA, LOST PROFITS, LOSS OF GOODWILL, LOST REVENUE, SERVICE INTERRUPTION, DEVICE DAMAGE
OR SYSTEM FAILURE, UNDER ANY THEORY OF LIABILITY, INCLUDING, WITHOUT LIMITATION CONTRACT OR TORT, ARISING FROM, OUT OF OR IN CONNECTION
WITH THE SOFTWARE.

All technology from the United States is subject to export regulations. This software is related to a device that has a United States
Export Commodity Classification of ECCN 7A994 with associated country chart control code of AT1. This generally will not require a license
to be exported or re-exported. However, if you plan to export this item to an embargoed or sanctioned country, to a party of concern,
or in support of a prohibited end-use, you may be required to obtain a license.
*/

#include "ros/ros.h"

//Custom messages
#include "api_to_topics.h"

//turtlesim messages
#include "geometry_msgs/Twist.h"

bool isBias;
//Container Structure
struct hgData_t
{
	float DVel[3];
	float DAngle[3];
	float velBias[3];
} hgData;


//Intertial Message callback
void hgIneDataCallback(const hg_nav_node::Msg_2311::ConstPtr& Message)
{
	if (isBias)
	{
		hgData.velBias[0] = Message->delta_velocity_x;
		hgData.velBias[1] = Message->delta_velocity_y;
		hgData.velBias[2] = Message->delta_velocity_z;
		ROS_INFO("velocity bias = %f, %f, %f",hgData.velBias[0],hgData.velBias[1],hgData.velBias[2]);
		isBias = false;
	}
	hgData.DAngle[0] = Message->delta_theta_x;
	hgData.DAngle[1] = Message->delta_theta_y;
	hgData.DAngle[2] = Message->delta_theta_z;

	hgData.DVel[0] = Message->delta_velocity_x;
	hgData.DVel[1] = Message->delta_velocity_y;
	hgData.DVel[2] = Message->delta_velocity_z;

	return;
}

//ACK/NAK Message callback
void hgAckCallback(const hg_nav_node::Msg_20FF::ConstPtr& Message)
{
	if (Message->Ack)
		ROS_INFO("ACK for 0x%.4x received - total valid messages: %d",Message->InputMessageID,Message->NoOfValidMessagesSincePowerUp);
	else
		ROS_ERROR("NAK for 0x%.4x received - total valid messages: %d",Message->InputMessageID,Message->NoOfValidMessagesSincePowerUp);

	return;
}

int main(int argc, char *argv[])
{
	//init ROS variables
	ros::init(argc,argv, "HgDataListener2");

	ros::NodeHandle n;

	ros::Rate loop_rate(100); //double the max transmission frequency

	//Configure device to send inertial data
	//ros::Publisher HgMsgEn_pub = n.advertise<hg_nav_node::Msg_1001> (MSG_1001_PATH, 5); // publish on predefined topic

	//hg_nav_node::Msg_1001 EnableMessage;
	//EnableMessage.MessageWord1.UNFILTERED_INS_DATA_2311_NC = 1; //enable inertial output
	//EnableMessage.MessageWord2.GEODETIC_POSITION_6403 = 1;
	//EnableMessage.MessageWord2.EULER_ATTITUDE_6405 = 1;
	//EnableMessage.MessageWord2.NED_VELOCITY_6504 = 1;

	//ros::Publisher HgNavIn_pub = n.advertise<hg_nav_node::Msg_1401> (MSG_1401_PATH, 5); // publish on predefined topic
	//hg_nav_node::Msg_1401 NavInputMessage;
	//NavInputMessage.RequestACKNAKReply = true;
	//NavInputMessage.AttitudeValidity = true;
	//NavInputMessage.AttitudeTimeReferenceMode = true;
	//NavInputMessage.AttitudeCoordinateFrame = true;

	//init Subscriber nodes
	ros::Subscriber HgIneData_sub = n.subscribe(MSG_2311_PATH,5,hgIneDataCallback);
	ros::Subscriber HgAck_sub = n.subscribe(MSG_20FF_PATH, 10,hgAckCallback);

	//Init data publisher to Turtlesim
	ros::Publisher n580_imu_data_pub = n.advertise<sensor_msgs::Imu>("n580_imu/data",1);

	//Send desried settings to Master control


	geometry_msgs::Twist msg;
	sensor_msgs::Imu n580_imu_msg;
	//initial settings
	msg.linear.z = 0;
	msg.linear.y = 0;
	msg.linear.x = 0;
	isBias = true;
	while (ros::ok())
	{


		//X,Y,Z arranged to work with demo board on table
		msg.linear.z = hgData.DVel[0]*100.0;
		msg.linear.y = hgData.DVel[1]*100.0;
		msg.linear.x = -hgData.DVel[2]*100.0;  // this controls the

		msg.angular.z = (-1) * hgData.DAngle[0]*100.0; // this control heading-rate of turtle
		msg.angular.y = hgData.DAngle[1]*100.0;
		msg.angular.x = hgData.DAngle[2]*100.0;
		/// euler to quaternion ///
		//tf2::Quaternion myQuaternion;
		//myQuaternion.setRPY(msg.angular.x,msg.angular.y,msg.angular.z *M_PI/180);
		//myQuaternion=myQuaternion.normalize();
		//\\ euler to quaternion \\\

		//msg_ins_fix.position_covariance = {msg->Pos_STD.x,msg->Pos_STD.y,msg->Pos_STD.z};

		//n580_imu_msg.orientation.x = myQuaternion[0];
		//n580_imu_msg.orientation.y = myQuaternion[1];
		//n580_imu_msg.orientation.z = myQuaternion[2];
		//n580_imu_msg.orientation.w = myQuaternion[3];

		n580_imu_msg.header.stamp = ros::Time::now();
		n580_imu_msg.header.frame_id = "imu_link";
		n580_imu_msg.angular_velocity.x = msg.angular.x;
		n580_imu_msg.angular_velocity.y = msg.angular.y;
		n580_imu_msg.angular_velocity.z = msg.angular.z;


		n580_imu_msg.linear_acceleration.x = msg.linear.x;
		n580_imu_msg.linear_acceleration.y = msg.linear.y;
		n580_imu_msg.linear_acceleration.z = msg.linear.z;


		n580_imu_data_pub.publish(n580_imu_msg);
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
