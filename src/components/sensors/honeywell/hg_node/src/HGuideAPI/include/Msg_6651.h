#ifndef __HGuideAPI_Msg_6651_h__
#define __HGuideAPI_Msg_6651_h__
#pragma once

#include <cstdint>

// list of port types
enum port_type_t
{
	input_INVALID  = 0, //  Invalid input type
	input_UART     = 1, //  UART COM port
	input_SDLC     = 2, //  SDLC Input port
	input_ETHERNET = 3, //  ETHERNET port input
	input_USB      = 4, //  USB data input
	input_CAN      = 5 //  CAN data input
};

// detailed data input status
struct data_status_t
{
	uint8_t portNum; // Port number of the port type (i.e. portType = input_UART && portNum = 1 => COM1)
	port_type_t portType; // Type of port being used to source the data
	bool active; // (0 = data NOT being received | 1 = data is being received) indicates whether the data is present

	void Default()
	{
		portNum = 0;
		portType = input_INVALID;
		active = 0;
	}
};

// information about the threshold value
struct threshold_status_t
{
	bool x_axis; // (0 = not met | 1 = met) indicates if threshold on the X axis was met
	bool y_axis; // (0 = not met | 1 = met) indicates if threshold on the Y axis was met
	bool z_axis; // (0 = not met | 1 = met) indicates if threshold on the Z axis was met
	bool enabled; // (0 = Disabled | 1 = Enabled) indicates if the threshold check is enabled

	void Default()
	{
		x_axis = 0;
		y_axis = 0;
		z_axis = 0;
		enabled = 0;
	}
};


// 0x6651 : Real Time Post Processing Good To Go
//
// The 0x6651 message is used to indicate to the user if the post processed solution will be within the defined thresholds
// assuming that the data logging is running from the start of the unit.
// The message also provides status on the input measurements of IMU and GNSS subsystems
// This check can be configured via 0x4651 input message.
// 
class HGUIDE_DLL Msg_6651
{
public:
	Msg_6651();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 64;}

public:
	static const uint32_t AddressId = 0xA5C381FF; // Start of Message
	static const uint32_t MessageId = 0x6651; // Message ID
	static const uint32_t MessageLength = 16; // Message Length
	uint32_t Checksum; // Checksum
	data_status_t ImuDataStatus; // IMU Subsystem Data Status
	data_status_t GnssDatatStatus; // GNSS Subsystem Data Status
	threshold_status_t PositionStatus; // Information about Position Threshold Status
	threshold_status_t VelocityStatus; // Information about Velocity Threshold Status
	threshold_status_t AttitudeStatus; // Information about Attitude Threshold Status
	uint8_t NavAxesSelected; // Navigation Axes Selector [0: Pos=NED, Vel=NED, Att=Euler | 1: Pos=ECEF, Vel=ECEF, Att=Gamma] same as 0x4651
};

#endif // __HGuideAPI_Msg_6651_h__
