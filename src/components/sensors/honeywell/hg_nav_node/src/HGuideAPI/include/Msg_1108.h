#ifndef __HGuideAPI_Msg_1108_h__
#define __HGuideAPI_Msg_1108_h__
#pragma once

#include <cstdint>


// 0x1108 : GPS PVT TM Aiding Input
//
// 0x1108 message is being used to input GNSS PVT measurements to the INS
// 
class HGUIDE_DLL Msg_1108
{
public:
	Msg_1108();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 112;}

public:
	static const uint32_t AddressId = 0xA5C381FF; // Start of Message
	static const uint32_t MessageId = 0x1108; // Message ID
	static const uint32_t MessageLength = 28; // Message Length [Number of 32-bit Words]
	uint32_t Checksum; // 32-bit CRC
	bool RequestAckNak; // (0 = do not request ACK/NAK | 1 = request ACK/NAK)
	bool TimeReferenceMode; // gpsPvtTov Time of Reference (0 = TOV referenced to GPS Time | 1 = TOV referenced to timestamp of message receipt)
	bool UnfilteredPointSolution; // Solution Type (0 = filtered solution | 1 = unfiltered solution)
	uint8_t GnssMode; // InputGNSS Mode (0 = standalone, 1 = sbas, 2 = differential, 3 = float rtk, 4 = fixed rtk, 5 = ppp terrastarc, 6 = ppp terrastarl)
	double gpsPvtTov; // [s] Message Time of validity
	bool PositionValidity; // Validity of the input position (0 = invalid | 1 = valid)
	bool PositionCoordinateFrame; // Input Coordinate Frame (0 = latitude/longitude/altitude | 1 = ecef [x, y, z])
	bool MslGeoidValidity; // NOT USED - Validity of input Geoid Type
	bool MslGeoidType; // NOT USED - Select input Geoid Type (0 = Altitude | 1 = Height)
	double Latitude; // [rad] Latitude in degrees //pi*2^-31
	double Longitude; // [rad] Longitude in degrees //pi*2^-31
	double Altitude; // [m] Altitude in meters
	double MslGeoidAltitudeOrGeoidHeight; // NOT USED - [m] Geoid Height or Altitude selectable by MslGeoidType
	double EcefPositionX; // [m] ecef x in meters
	double EcefPositionY; // [m] ecef y in meters
	double EcefPositionZ; // [m] ecef z in meters
	bool VelocityValidity; // Validity of the input velocity (0 = invalid | 1 = valid)
	bool VelocityCoordinateFrame; // Input Coordinate Frame (0 = north east down (ned) | 1 = ecef x y z)
	double VelocityNorthOrEcefVelocityX; // [m/s] North velocity or ecef x in meters per second
	double VelocityEastOrEcefVelocityY; // [m/s] East velocity or ecef y in meters per second
	double VelocityDownOrEcefVelocityZ; // [m/s] Down velocity or ecef z in meters per second
	bool LeapSecondValidity; // NOT USED
	bool ReceiverClockBiasErrorValidity; // NOT USED
	bool DopDataValidity; // Validity of the input Dilution of Precision (0 = invalid | 1 = valid)
	bool ConstellationChangeIndicator; // Indication whether constellation changed (0 = not changed | 1 = changed)
	bool EstimatedPositionErrorValidity; // Validity of the input position Error estimate (0 = invalid | 1 = valid)
	bool EstimatedVelocityErrorValidity; // Validity of the input velocity Error estimate (0 = invalid | 1 = valid)
	float LeapSeconds; // NOT USED - [s] Number of leap seconds
	float ReceiverClockBiasError; // NOT USED - [m] Receiver Clock bias error
	float Tdop; // Time dilution of precision (-1 = invalid)
	float Vdop; // Vertical dilution of precision (-1 = invalid)
	float Hdop; // Horizontal dilution of precision (-1 = invalid)
	float Pdop; // Position dilution of precision (-1 = invalid)
	float EstimatedVerticalPositionError; // Estimated vertical position error (1-sigma)
	float EstimatedHorizontalPositionError; // Estimated horizontal position error (1-sigma)
	float EstimatedVerticalVelocityError; // Estimated vertical velocity error (1-sigma)
	float EstimatedHorizontalVelocityError; // Estimated horizontal velocity error (1-sigma)
	uint32_t Fom; // Figure of merit
	uint32_t Tfom; // Coordinated universal time figure of merit
};

#endif // __HGuideAPI_Msg_1108_h__
