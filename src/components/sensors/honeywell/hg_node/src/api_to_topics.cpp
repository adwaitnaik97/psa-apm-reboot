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

All technology that leaves the United States is subject to export regulations. This manual contains technology that has an Export Commodity Classification of EAR99.
This technology generally will not require a license to be exported or reexported. 
However, if you plan to export this item to an embargoed or sanctioned country, to a party of concern, or in support of a prohibited end-use, you may be required to obtain a license.
*/

#include "api_to_topics.h"


//Standard Messages Publisher
sensor_msgs::Imu ImuMsg;
sensor_msgs::Imu ImuMsgOrg;

ros::Publisher hgImu_pub;
ros::Publisher hgImu_pub_org;


void initPubs(ros::NodeHandle * n)
{
	//Advertise standard messages
	hgImu_pub = n->advertise<sensor_msgs::Imu> ("HGuide/Std/Imu", 5);
	hgImu_pub_org = n->advertise<sensor_msgs::Imu> ("HGuide/Std/org/Imu", 5);
	//Default unknown covariance
	ImuMsg.angular_velocity_covariance[0]=-1;
	ImuMsg.linear_acceleration_covariance[0]=-1;
	ImuMsg.orientation_covariance[0]=-1;
	
	//init generic Publishers
	// publishers are started automatically
	//init_all_pubs(n);
}
void stopPubs(void)
{
	//Advertise standard messages
	hgImu_pub.shutdown();
	hgImu_pub_org.shutdown();
	
	//stop generic Publishers
	stop_all_pubs();
}

void initSubs(ros::NodeHandle * n)
{
	//init generic Subscribers
	init_all_subs(n);
}

void stopSubs(void)
{
	//stop generic Subscribers
	stop_all_subs();
}



void getImuData(uint8_t * buffer, uint32_t bufferSize, uint8_t msgID)
{
	if (msgID==0x01)
	{
		Msg_01 message;
		message.Deserialize(buffer,bufferSize);
		if (ImuMsg.angular_velocity_covariance[0] == -1)
			setImuCovariance(UNIT_HG4930);
			
		PubImu(message.AngularRateX,message.AngularRateY,message.AngularRateZ,message.LinearAccelerationX,message.LinearAccelerationY,message.LinearAccelerationZ);
	}
	if (msgID==0x02)
	{
		Msg_02 message;
		message.Deserialize(buffer,bufferSize);
		PubImu(message.AngularRateX,message.AngularRateY,message.AngularRateZ,message.LinearAccelerationX,message.LinearAccelerationY,message.LinearAccelerationZ);
	}
	if (msgID==0x0C)
	{
		Msg_0C message;
		message.Deserialize(buffer,bufferSize);
		if (ImuMsg.angular_velocity_covariance[0] == -1)
			setImuCovariance(UNIT_HG1120);
		PubImu(message.AngularRateX,message.AngularRateY,message.AngularRateZ,message.LinearAccelerationX,message.LinearAccelerationY,message.LinearAccelerationZ);
	}
	if (msgID==0x0D)
	{
		Msg_0D message;
		message.Deserialize(buffer,bufferSize);
		PubImu(message.AngularRateX,message.AngularRateY,message.AngularRateZ,message.LinearAccelerationX,message.LinearAccelerationY,message.LinearAccelerationZ);
	}
	if (msgID==0x04)
	{
		Msg_04 message;
		message.Deserialize(buffer,bufferSize);
		if (ImuMsg.angular_velocity_covariance[0] == -1)
			setImuCovariance(UNIT_HG1120);
		PubImu(message.AngularRateX,message.AngularRateY,message.AngularRateZ,message.LinearAccelerationX,message.LinearAccelerationY,message.LinearAccelerationZ);
	}
	if (msgID==0x05)
	{
		Msg_05 message;
		message.Deserialize(buffer,bufferSize);
		PubImu(message.AngularRateX,message.AngularRateY,message.AngularRateZ,message.LinearAccelerationX,message.LinearAccelerationY,message.LinearAccelerationZ);
	}
	if (msgID==0xCA)
	{
		Msg_CA message;
		message.Deserialize(buffer,bufferSize);
		PubImu(message.AngularRateX,message.AngularRateY,message.AngularRateZ,message.LinearAccelerationX,message.LinearAccelerationY,message.LinearAccelerationZ);
	}
	if (msgID==0xA1)
	{
		Msg_A1 message;
		message.Deserialize(buffer,bufferSize);
		if (ImuMsg.angular_velocity_covariance[0] == -1)
			setImuCovariance(UNIT_I300);
		PubImu(message.AngularRateX,message.AngularRateY,message.AngularRateZ,message.LinearAccelerationX,message.LinearAccelerationY,message.LinearAccelerationZ);
	}
	if (msgID==0xA2)
	{
		Msg_A2 message;
		message.Deserialize(buffer,bufferSize);
		PubImu(message.AngularRateX,message.AngularRateY,message.AngularRateZ,message.LinearAccelerationX,message.LinearAccelerationY,message.LinearAccelerationZ);
	}
	if (msgID==0xAC)
	{
		Msg_AC message;
		message.Deserialize(buffer,bufferSize);
		if (ImuMsg.angular_velocity_covariance[0] == -1)
			setImuCovariance(UNIT_I300);
		PubImu(message.AngularRateX,message.AngularRateY,message.AngularRateZ,message.LinearAccelerationX,message.LinearAccelerationY,message.LinearAccelerationZ);
	}
	if (msgID==0xAD)
	{
		Msg_AD message;
		message.Deserialize(buffer,bufferSize);
		PubImu(message.AngularRateX,message.AngularRateY,message.AngularRateZ,message.LinearAccelerationX,message.LinearAccelerationY,message.LinearAccelerationZ);
	}
	if (msgID==0xB1)
	{
		Msg_B1 message;
		message.Deserialize(buffer,bufferSize);
		if (ImuMsg.angular_velocity_covariance[0] == -1)
			setImuCovariance(UNIT_I300);
		PubImu(message.AngularRateX,message.AngularRateY,message.AngularRateZ,message.LinearAccelerationX,message.LinearAccelerationY,message.LinearAccelerationZ);
	}
	return;
}


//Imu Message
void PubImu(float rateX,float rateY,float rateZ,float accelX,float accelY,float accelZ)
{

	// Publishing in the same frame as received from the sensor

	ImuMsg.angular_velocity.x = rateX;
	ImuMsg.angular_velocity.y = rateY;
	ImuMsg.angular_velocity.z = rateZ;
	

	ImuMsg.linear_acceleration.x = -accelX; //Standardizing to ROS coordinate system
	ImuMsg.linear_acceleration.y = accelY;
	ImuMsg.linear_acceleration.z = accelZ;


	//Attitude
	//no attitude support yet

	ImuMsg.header.seq++;
	ImuMsg.header.stamp=ros::Time::now();
	ImuMsg.header.frame_id="HGuide";
	
	ROS_DEBUG("\nHGuide Imu msg[%.3f,%.3f,%.3f,%.3f]", ImuMsg.angular_velocity.x,ImuMsg.angular_velocity.y,ImuMsg.angular_velocity.z,ImuMsg.linear_acceleration.x);

	hgImu_pub.publish(ImuMsg);

}

void setImuCovariance(uint8_t unit_type)
{

	// Covariance calculated as:
	// (Bias Instability converted to required unit)^2
	// (rad/h / 3600)^2
	// (mg * 9.81 / 1000)^2
	switch (unit_type)
	{
	case UNIT_HG4930:
		ImuMsg.angular_velocity_covariance[0]=4.822530864E-09; // rad/s
		ImuMsg.angular_velocity_covariance[4]=4.822530864E-09;
		ImuMsg.angular_velocity_covariance[8]=4.822530864E-09;
		ImuMsg.linear_acceleration_covariance[0]=0.0000000601475625; //m/s
		ImuMsg.linear_acceleration_covariance[4]=0.0000000601475625;
		ImuMsg.linear_acceleration_covariance[8]=0.0000000601475625;
		ROS_INFO("HG4930 Covariance set");
		break;
	case UNIT_I300:
		ImuMsg.angular_velocity_covariance[0]=6.94444E-7; // rad/s
		ImuMsg.angular_velocity_covariance[4]=6.94444E-7;
		ImuMsg.angular_velocity_covariance[8]=6.94444E-7;
		ImuMsg.linear_acceleration_covariance[0]=0.00000003849444; // m/s
		ImuMsg.linear_acceleration_covariance[4]=0.00000003849444;
		ImuMsg.linear_acceleration_covariance[8]=0.00000003849444;
		ROS_INFO("HGuide i300 Covariance set");
		break;
	case UNIT_HG1120:
		ImuMsg.angular_velocity_covariance[0]=3.08641975E-5; // rad/s
		ImuMsg.angular_velocity_covariance[4]=3.08641975E-5;
		ImuMsg.angular_velocity_covariance[8]=3.08641975E-5;
		ImuMsg.linear_acceleration_covariance[0]=0.00000034644996; // m/s
		ImuMsg.linear_acceleration_covariance[4]=0.00000034644996;
		ImuMsg.linear_acceleration_covariance[8]=0.00000034644996;
		ROS_INFO("HG1120 Covariance set");
		break;
	default:
		ROS_WARN("Unknown Covariance!");
		break;
	}
}



