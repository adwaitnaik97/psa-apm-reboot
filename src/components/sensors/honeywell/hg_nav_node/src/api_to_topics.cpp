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

#include "api_to_topics.h"
#include<math.h>

//Standard Messages Publisher
sensor_msgs::Imu ImuMsg;
sensor_msgs::NavSatFix NavSatFixMsg;
sensor_msgs::TimeReference TimeReferenceMsg;
geometry_msgs::TwistWithCovarianceStamped TwistWithCovarianceStampedMsg;
nav_msgs::Odometry OdometryMsg;

ros::Publisher hgImu_pub;
ros::Publisher hgNavSatFix_pub;
ros::Publisher TimeReference_pub;
ros::Publisher TwistWithCovarianceStamped_pub;
ros::Publisher hgOdometry_pub;

void rotateVector(double vector[3], float quat[4], double* outVector0, double* outVector1, double* outVector2);

Msg_2001 Installation;
void api_set_Installation(Msg_2001 input)
{
	Installation = input;
}

void initPubs(ros::NodeHandle * n)
{
	// renew the world to body transform
	WorldToBody.header.frame_id = "uninitialized";
	
	//Advertise standard messages
	hgImu_pub = n->advertise<sensor_msgs::Imu> ("HGuide/Std/Imu", 5);
	hgOdometry_pub = n->advertise<nav_msgs::Odometry> ("HGuide/Std/Odometry", 5);
	hgNavSatFix_pub = n->advertise<sensor_msgs::NavSatFix> ("HGuide/Std/NavSatFix", 5);
	TimeReference_pub = n->advertise<sensor_msgs::TimeReference> ("HGuide/Std/TimeReference", 5);
	TwistWithCovarianceStamped_pub = n->advertise<geometry_msgs::TwistWithCovarianceStamped> ("HGuide/Std/TwistWithCovarianceStamped", 5);
	
	//init generic Publishers
	// generic Publishers are activated automatically
	//init_all_pubs(n);
}
void stopPubs(void)
{
	//Advertise standard messages
	hgImu_pub.shutdown();
	hgOdometry_pub.shutdown();
	hgNavSatFix_pub.shutdown();
	TimeReference_pub.shutdown();
	TwistWithCovarianceStamped_pub.shutdown();
	
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

//NavSatFix Message
void PubNavSatFix(Msg_6403 Message)
{
	NavSatFixMsg.header.seq++;
	NavSatFixMsg.header.stamp = ros::Time::now();

	NavSatFixMsg.status.service = 0xF; //Dummy full fix

	if (Message.InsGnssSummary.INSMode == 4)
	{
		switch(Message.InsGnssSummary.GPSMode)
		{
			case 15: NavSatFixMsg.status.status = -1; break;
			case 0:	NavSatFixMsg.status.status = 0; break;
			case 1:	NavSatFixMsg.status.status = 1; break;
			case 3: NavSatFixMsg.status.status = 2; break;
			case 4: NavSatFixMsg.status.status = 2; break;
		}
	}
	else
	{NavSatFixMsg.status.status = -1;}

	NavSatFixMsg.latitude = Message.Latitude*180/PI;
	NavSatFixMsg.longitude = Message.Longitude*180/PI;
	NavSatFixMsg.altitude = Message.AltitudeHeightAboveEllipsoid;

	NavSatFixMsg.position_covariance[0]=pow(Message.LatitudeSTDV,2);
	NavSatFixMsg.position_covariance[4]=pow(Message.LongitudeSTDV,2);
	NavSatFixMsg.position_covariance[8]=Message.AltitudeHeightAboveEllipsoidSTDV;

	NavSatFixMsg.position_covariance_type = 2; //COVARIANCE_TYPE_DIAGONAL_KNOWN
	

	ROS_DEBUG("Status [%f,%f,%f,%d]", NavSatFixMsg.latitude,NavSatFixMsg.longitude,NavSatFixMsg.altitude,NavSatFixMsg.status.status);
	
	hgNavSatFix_pub.publish(NavSatFixMsg);	

}


//Imu Message
void PubImu(Msg_2311 InertialOutputMsg, Msg_6405 EulerAttitudeOutputMsg)
{

	float quat[4];
	double rate[3] = {InertialOutputMsg.delta_theta_x*100,InertialOutputMsg.delta_theta_y*100,InertialOutputMsg.delta_theta_z*100};
	double accel[3] = {InertialOutputMsg.delta_velocity_x*100,InertialOutputMsg.delta_velocity_y*100,InertialOutputMsg.delta_velocity_z*100};
	double outRate[3];
	double outAccel[3];
	
	if (Installation.IMUSerialNumber[0]==0) //Installation Unknown - use N580
	{
		quat[0] = 3.0908619663E-08;
		quat[1] = -0.70710676908493;
		quat[2] = -3.0908619663E-08;
		quat[3] = -0.70710676908493;
	}
	else // Quaternion from 0x2001 message
	{
		quat[0] = Installation.CaseToNavQuaternionS;
		quat[1] = Installation.CaseToNavQuaternionI;
		quat[2] = Installation.CaseToNavQuaternionJ;
		quat[3] = Installation.CaseToNavQuaternionK;
	}
	
	//rotate to case frame
	rotateVector(rate, quat, &outRate[0], &outRate[1], &outRate[2]);
	rotateVector(accel, quat, &outAccel[0], &outAccel[1], &outAccel[2]);

	//NED to ENU Frame	
	ImuMsg.angular_velocity.x = outRate[0];
	ImuMsg.angular_velocity.y = outRate[1];
	ImuMsg.angular_velocity.z = outRate[2];

	ImuMsg.linear_acceleration.x = outAccel[0];
	ImuMsg.linear_acceleration.y = outAccel[1];
	ImuMsg.linear_acceleration.z = outAccel[2];
	
	ImuMsg.angular_velocity_covariance[0]=-1;
	ImuMsg.linear_acceleration_covariance[0]=-1;

	//Attitude
	tf2::Quaternion q;
	q.setRPY(EulerAttitudeOutputMsg.Roll,EulerAttitudeOutputMsg.Pitch,-(EulerAttitudeOutputMsg.Heading-90*DEG_TO_RAD)); // 0 heading is easting
	ImuMsg.orientation.x = q.x();
	ImuMsg.orientation.y = q.y();
	ImuMsg.orientation.z = q.z();
	ImuMsg.orientation.w = q.w();
	
	ImuMsg.orientation_covariance[0]=EulerAttitudeOutputMsg.EulerAnglesSTDVRoll;	
	ImuMsg.orientation_covariance[4]=EulerAttitudeOutputMsg.EulerAnglesSTDVPitch;
	ImuMsg.orientation_covariance[8]=EulerAttitudeOutputMsg.EulerAnglesSTDVHeading;
	
	ImuMsg.header.seq++;
	ImuMsg.header.stamp=ros::Time::now();
	ImuMsg.header.frame_id="HGuide";

	ROS_DEBUG("\n[%.3f,%.3f,%.3f,%.3f]",ImuMsg.orientation.x,ImuMsg.orientation.y,ImuMsg.orientation.z,ImuMsg.orientation.w);

	hgImu_pub.publish(ImuMsg);
}

//Odometry Message - Used when initialized from serial_publisher.cpp
double odoPrevRoll = 0;
double odoPrevPitch = 0;
double odoPrevHeading = 0;

void PubOdometry(tf2::Quaternion q, double x, double y, double z,Msg_6403 GeodeticPositionOutputMsg, Msg_6405 EulerAttitudeOutputMsg, Msg_6504 NEDVelocityOutputMsg)
{
	OdometryMsg.header.seq++;
	OdometryMsg.header.stamp = ros::Time::now();
	OdometryMsg.header.frame_id="LocalVertical";
	OdometryMsg.child_frame_id="HGuide";
	//Pose
	//Position
	OdometryMsg.pose.pose.position.x = x; 	OdometryMsg.pose.covariance[0] = GeodeticPositionOutputMsg.LatitudeSTDV;
	OdometryMsg.pose.pose.position.y = y; 	OdometryMsg.pose.covariance[7] = GeodeticPositionOutputMsg.LongitudeSTDV;
	OdometryMsg.pose.pose.position.z = z; 	OdometryMsg.pose.covariance[14] = GeodeticPositionOutputMsg.AltitudeHeightAboveEllipsoidSTDV;
	//Attitude
	OdometryMsg.pose.pose.orientation.x = q.x();
	OdometryMsg.pose.pose.orientation.y = q.y();
	OdometryMsg.pose.pose.orientation.z = q.z();
	OdometryMsg.pose.pose.orientation.w = q.w();

 	OdometryMsg.pose.covariance[21] = EulerAttitudeOutputMsg.EulerAnglesSTDVRoll;
 	OdometryMsg.pose.covariance[28] = EulerAttitudeOutputMsg.EulerAnglesSTDVPitch;
 	OdometryMsg.pose.covariance[35] = EulerAttitudeOutputMsg.EulerAnglesSTDVHeading;
	//Twist
	//Linear - NED to ENU
	/*OdometryMsg.twist.twist.linear.x = NEDVelocityOutputMsg.NorthVelocity; 	OdometryMsg.twist.covariance[0] = NEDVelocityOutputMsg.NorthVelocitySTDV;
	OdometryMsg.twist.twist.linear.y = NEDVelocityOutputMsg.EastVelocity; 	OdometryMsg.twist.covariance[7] = NEDVelocityOutputMsg.EastVelocitySTDV;
	OdometryMsg.twist.twist.linear.z = NEDVelocityOutputMsg.DownVelocity; 	OdometryMsg.twist.covariance[14] = NEDVelocityOutputMsg.DownVelocitySTDV;*/
	OdometryMsg.twist.twist.linear.y = NEDVelocityOutputMsg.NorthVelocity; 	OdometryMsg.twist.covariance[7] = NEDVelocityOutputMsg.NorthVelocitySTDV;
	OdometryMsg.twist.twist.linear.x = NEDVelocityOutputMsg.EastVelocity; 	OdometryMsg.twist.covariance[0] = NEDVelocityOutputMsg.EastVelocitySTDV;
	OdometryMsg.twist.twist.linear.z = -NEDVelocityOutputMsg.DownVelocity; 	OdometryMsg.twist.covariance[14] = NEDVelocityOutputMsg.DownVelocitySTDV;
	//Angular - NED to ENU
	/*OdometryMsg.twist.twist.angular.x = EulerAttitudeOutputMsg.Roll; 	OdometryMsg.twist.covariance[21] = EulerAttitudeOutputMsg.EulerAnglesSTDVRoll;
	OdometryMsg.twist.twist.angular.y = EulerAttitudeOutputMsg.Pitch; 	OdometryMsg.twist.covariance[28] = EulerAttitudeOutputMsg.EulerAnglesSTDVPitch;
	OdometryMsg.twist.twist.angular.z = EulerAttitudeOutputMsg.Heading; 	OdometryMsg.twist.covariance[35] = EulerAttitudeOutputMsg.EulerAnglesSTDVHeading;*/
	OdometryMsg.twist.twist.angular.y = (EulerAttitudeOutputMsg.Roll-odoPrevRoll)*100; 	OdometryMsg.twist.covariance[28] = EulerAttitudeOutputMsg.EulerAnglesSTDVRoll;
	OdometryMsg.twist.twist.angular.x = (EulerAttitudeOutputMsg.Pitch-odoPrevPitch)*100; 	OdometryMsg.twist.covariance[21] = EulerAttitudeOutputMsg.EulerAnglesSTDVPitch;
	OdometryMsg.twist.twist.angular.z = -(EulerAttitudeOutputMsg.Heading-odoPrevHeading)*100; 	OdometryMsg.twist.covariance[35] = EulerAttitudeOutputMsg.EulerAnglesSTDVHeading;
	
	//store current values for diff
	odoPrevRoll = EulerAttitudeOutputMsg.Roll;
	odoPrevPitch = EulerAttitudeOutputMsg.Pitch;
	odoPrevHeading = EulerAttitudeOutputMsg.Heading;
	
	hgOdometry_pub.publish(OdometryMsg);
	
}
//Twist message
double twistPrevRoll = 0;
double twistPrevPitch = 0;
double twistPrevHeading = 0;

void PubTwistWithCovariance(Msg_6405 EulerAttitudeOutputMsg, Msg_6504 NEDVelocityOutputMsg)
{
	TwistWithCovarianceStampedMsg.header.frame_id = "HGuide";
	TwistWithCovarianceStampedMsg.header.stamp = ros::Time::now();
	TwistWithCovarianceStampedMsg.header.seq++;

	//Linear
	TwistWithCovarianceStampedMsg.twist.twist.linear.x = NEDVelocityOutputMsg.EastVelocity; 	
	TwistWithCovarianceStampedMsg.twist.covariance[0] = NEDVelocityOutputMsg.EastVelocitySTDV;

	TwistWithCovarianceStampedMsg.twist.twist.linear.y = NEDVelocityOutputMsg.NorthVelocity; 	
	TwistWithCovarianceStampedMsg.twist.covariance[7] = NEDVelocityOutputMsg.NorthVelocitySTDV;

	TwistWithCovarianceStampedMsg.twist.twist.linear.z = -NEDVelocityOutputMsg.DownVelocity; 	
	TwistWithCovarianceStampedMsg.twist.covariance[14] = NEDVelocityOutputMsg.DownVelocitySTDV;
	//Angular
	TwistWithCovarianceStampedMsg.twist.twist.angular.x = (EulerAttitudeOutputMsg.Pitch-twistPrevPitch)*100; 	
	TwistWithCovarianceStampedMsg.twist.covariance[21] = EulerAttitudeOutputMsg.EulerAnglesSTDVPitch;

	TwistWithCovarianceStampedMsg.twist.twist.angular.y = (EulerAttitudeOutputMsg.Roll-twistPrevRoll)*100; 	
	TwistWithCovarianceStampedMsg.twist.covariance[28] = EulerAttitudeOutputMsg.EulerAnglesSTDVRoll;

	TwistWithCovarianceStampedMsg.twist.twist.angular.z = -(EulerAttitudeOutputMsg.Heading-twistPrevHeading)*100; 	
	TwistWithCovarianceStampedMsg.twist.covariance[35] = EulerAttitudeOutputMsg.EulerAnglesSTDVHeading;

	//store current values for diff
	twistPrevRoll = EulerAttitudeOutputMsg.Roll;
	twistPrevPitch = EulerAttitudeOutputMsg.Pitch;
	twistPrevHeading = EulerAttitudeOutputMsg.Heading;
	
	TwistWithCovarianceStamped_pub.publish(TwistWithCovarianceStampedMsg);
}

//Time Reference Message
void PubTimeReference(double gpsTime)
{
	TimeReferenceMsg.header.stamp = ros::Time::now();
	TimeReferenceMsg.header.seq++;

	TimeReferenceMsg.source = "gps";
	double LEAP_SECONDS = 8;
	double utcTime=0;
	//remove days
	double temp = fmod(gpsTime+LEAP_SECONDS,24*3600);
	utcTime = std::floor(temp/3600)*10000 + std::floor(fmod(temp,3600)/60)*100 + fmod(temp,60);


	TimeReferenceMsg.time_ref = ros::Time(utcTime);
	
	TimeReference_pub.publish(TimeReferenceMsg);
}


    //Convert Lat, Lon, Altitude to Earth-Centered-Earth-Fixed (ECEF)
    //Input is a three element array containing lat, lon (rads) and alt (m)
    //Returned array contains x, y, z in meters
void geo_to_ecef(double lat, double lon, double alt,double * ecef1,double * ecef2,double * ecef3)
{
	double  a = 6378137.0;              //WGS-84 semi-major axis
	double e2 = 6.6943799901377997e-3;  //WGS-84 first eccentricity squared
	double n;

	n = a/std::sqrt(1-e2*std::sin(lat)*std::sin(lat));
	*ecef1=(n+alt)*std::cos(lat)*std::cos(lon);
	*ecef2=(n+alt)*std::cos(lat)*std::sin(lon);
	*ecef3=(n*(1-e2)+alt)*std::sin(lat);

	return;

}

double earthRadius(double lat, double alt)
{
	double re = 6378137.0;
	double f = 1/298.257223563;
	double e2 = 2*f - f*f;

	double lat_sin = std::sin(lat);

	double scalefactor = 1.0 - e2 * lat_sin * lat_sin;
	double scalefactor_sqrt = std::sqrt(scalefactor);

	return re* (1.0 - e2) / (scalefactor * scalefactor_sqrt) + alt;
}

double earthRadius2(double lat, double alt)
{
	double re = 6378137.0;
	double f = 1/298.257223563;
	double e2 = 2*f - f*f;

	double lat_sin = std::sin(lat);

	double scalefactor = 1.0 - e2 * lat_sin * lat_sin;
	double scalefactor_sqrt = std::sqrt(scalefactor);

	return re / scalefactor_sqrt + alt;
}

double lat2mtr(double angle, double lat, double alt)
{
	double ns_re = earthRadius(lat,alt);
	return angle * ns_re;
}

double lon2mtr(double lon, double lat, double alt)
{
	double ew_re = earthRadius2(lat,alt);
	return lon * ew_re * std::abs(std::cos(lat));
}

void lla2mtrdiff(double in1lat, double in1lon, double in1alt,double in2lat, double in2lon, double in2alt,double * outN, double * outE, double * outD)
{
	*outN = lat2mtr(in1lat-in2lat,in1lat,in1alt);
	*outE = lon2mtr(in1lon-in2lon,in1lat,in1alt);
	*outD = -1.0f*(in1alt - in2alt);
}

//Rotate vector by quaternion
void rotateVector(double vector[3], float quat[4], double* outVector0, double* outVector1, double* outVector2)
{
	double rotMat[3][3];
	double a = quat[0];
	double b = quat[1];
	double c = quat[2];
	double d = quat[3];

	rotMat[0][0] = a*a+b*b-c*c-d*d;	rotMat[0][1] = 2*b*c - 2*a*d;			rotMat[0][2] = 2*b*d + 2*a*c;
	rotMat[1][0] = 2*b*c + 2*a*d;	rotMat[1][1] = a*a - b*b + c*c - d*d;	rotMat[1][2] = 2*c*d - 2*a*b;
	rotMat[2][0] = 2*b*d - 2*a*c;	rotMat[2][1] = 2*c*d + 2*a*b;			rotMat[2][2] = a*a - b*b - c*c + d*d;

	*outVector0 = vector[0] * rotMat[0][0] + vector[1] * rotMat[0][1] + vector[2] * rotMat[0][2];
	*outVector1 = vector[0] * rotMat[1][0] + vector[1] * rotMat[1][1] + vector[2] * rotMat[1][2];
	*outVector2 = vector[0] * rotMat[2][0] + vector[1] * rotMat[2][1] + vector[2] * rotMat[2][2];

	return;
}


//Publish Transformation
void processStdTopics(Msg_6403 GeoPosition, Msg_6405 EulerAttitudes, Msg_6504 NEDVelocity, Msg_2311 InertialData)
{
	static tf2_ros::StaticTransformBroadcaster static_br;
	static tf2_ros::TransformBroadcaster br;
	if (initLat != -999.0 && GeoPosition.InsGnssSummary.INSMode == 4) //SHALL BE PUBLISHED ONLY ONCE INITIALIZED!!
		{


			//ECEF Global - WORLD TO LOCAL VERTICAL
			//Static transformation
			if (WorldToBody.header.frame_id == "uninitialized")
			{
				double ecef[3];
				geo_to_ecef(initLat,initLon,initAlt,&ecef[0],&ecef[1],&ecef[2]);
				
				WorldToBody.transform.translation.x = ecef[0];
				WorldToBody.transform.translation.y = ecef[1];
				WorldToBody.transform.translation.z = ecef[2];

				//q.setRPY(-EulerAttitudes.Heading,(180*DEG_TO_RAD)-initLon,initLat);						
				q.setRPY(initHeading,(180*DEG_TO_RAD)-initLon,initLat);		
				WorldToBody.transform.rotation.x = q.x();
				WorldToBody.transform.rotation.y = q.y();
				WorldToBody.transform.rotation.z = q.z();
				WorldToBody.transform.rotation.w = q.w();
				
				WorldToBody.header.stamp = ros::Time::now();
				WorldToBody.header.frame_id = "world";
				WorldToBody.child_frame_id = "LocalVertical";

				static_br.sendTransform(WorldToBody);
			}

			//LOCAL VERTICAL - INIT POSITION TO INS BODY
			double ned[3];
			lla2mtrdiff(GeoPosition.Latitude,GeoPosition.Longitude,GeoPosition.AltitudeHeightAboveEllipsoid,initLat,initLon, initAlt,&ned[0], &ned[1],&ned[2]);
			bodyToLocalVertical.transform.translation.x = ned[1];
			bodyToLocalVertical.transform.translation.y = ned[0];
			bodyToLocalVertical.transform.translation.z = -ned[2];

			
			//LEGACY NED-like frame
			//discontinued in favour of ROS intertial frame
			//q.setRPY((180*DEG_TO_RAD)+Message.EulerAttitudes.EulerAttitude[0],-Message.EulerAttitudes.EulerAttitude[1],-Message.EulerAttitudes.EulerAttitude[2]);
			
			q.setRPY(EulerAttitudes.Roll,EulerAttitudes.Pitch,-(EulerAttitudes.Heading-90*DEG_TO_RAD)); // 0 heading at easting
			bodyToLocalVertical.transform.rotation.x = q.x();
			bodyToLocalVertical.transform.rotation.y = q.y();
			bodyToLocalVertical.transform.rotation.z = q.z();
			bodyToLocalVertical.transform.rotation.w = q.w();
	
			bodyToLocalVertical.header.stamp = ros::Time::now();
			bodyToLocalVertical.header.frame_id = "LocalVertical";
			bodyToLocalVertical.child_frame_id = "HGuide";

			br.sendTransform(bodyToLocalVertical);	

			//Odometry from Local Vertical to hgINS
			PubOdometry(q, ned[1], ned[0], -ned[2],GeoPosition, EulerAttitudes,NEDVelocity);		
			//Pub NavSat fix
			PubNavSatFix(GeoPosition);
		}
		
		if (InertialData.systemTov > 0){
			//Imu Message
			PubImu(InertialData,EulerAttitudes);
		}
		if (EulerAttitudes.systemTov > 0){
			//Twist With Covariance Stamped message
			PubTwistWithCovariance(EulerAttitudes, NEDVelocity);
		}
}
