#ifndef __HGuideAPI_Msg_6406_h__
#define __HGuideAPI_Msg_6406_h__
#pragma once

#include <cstdint>

#include <include/ins_gnss_summary_t.h>


// 0x6406 : Vehicle Body Rates and Linear Accelerations
//
// The 0x6406 message outputs the angular velocity and linear acceleration of the vehicle body frame with respect to the ECEF frame.  
// Note : Earth rate has been removed w_IB = w_IE + w_EB
// Note : Linear acceleration is the output and NOT specific force.  Gravity and Coriolis have been removed and what left is the best estimate of the second derivative of ECEF position with respect to time.
// Note : the message provides zero outputs until the unit begins the process of navigating (entering Coarse Level or Aided Navigation)
// 
class HGUIDE_DLL Msg_6406
{
public:
	Msg_6406();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 60;}

public:
	static const uint32_t AddressId = 0xA5C381FF; // Start of Message Flag
	static const uint32_t MessageId = 0x6406; // Message ID
	static const uint32_t MessageLength = 15; // Message Length in 32-bit word
	uint32_t Checksum; // Checksum
	double systemTov; // [s] System Time of Validity
	double gpsTov; // [sec] GPS Time of Validity in current week (Sunday 00:00 UTC). When invalid, GPS time = INS/GPS Internal System Time
	float AngularRateX; // [rad/s] Angular Velocity in Vehicle Body with respect to the ECEF coordinatized in the Vehicle Body Frame [x component]
	float AngularRateY; // [rad/s] Angular Velocity in Vehicle Body with respect to the ECEF coordinatized in the Vehicle Body Frame [y component]
	float AngularRateZ; // [rad/s] Angular Velocity in Vehicle Body with respect to the ECEF coordinatized in the Vehicle Body Frame [z component]
	ins_gnss_summary_t InsGnssSummary; // INS / GNSS summary word
	float LinearAccelerationX; // [m/s/s] Linear Acceleration in the Vehicle Body [x component]
	float LinearAccelerationY; // [m/s/s] Linear Acceleration in the Vehicle Body [y component]
	float LinearAccelerationZ; // [m/s/s] Linear Acceleration in the Vehicle Body [z component]
};

#endif // __HGuideAPI_Msg_6406_h__
