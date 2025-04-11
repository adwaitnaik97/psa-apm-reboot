#ifndef __HGuideAPI_Msg_6003_h__
#define __HGuideAPI_Msg_6003_h__
#pragma once

#include <cstdint>

#include <include/ins_gnss_summary_t.h>


// 0x6003 : HGuide Sensor Installation Settings
//
// Sensor installation attitudes and lever arms, all in one message
// this message is used in the HGuide Data Reader Guided setup
// 
class HGUIDE_DLL Msg_6003
{
public:
	Msg_6003();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 256;}

public:
	static const uint32_t AddressId = 0xA5C381FF; // Start of Message
	static const uint32_t MessageId = 0x6003; // Message ID
	static const uint32_t MessageLength = 64; // Message Length
	uint32_t Checksum; // Checksum
	double systemTov; // [sec] System Time of Validity
	double gpsTov; // [sec] GPS Time of Validity in current week (Sunday 00:00 UTC). When invalid, GPS time = INS/GPS Internal System Time
	ins_gnss_summary_t InsGnssSummary; // INS / GNSS Summary Word
	float ImuLeverArmsX; // [m] lever_arm-x CASE_BODY to IMU_BODY in CASE_BODY frame
	float ImuLeverArmsY; // [m] lever_arm-y CASE_BODY to IMU_BODY in CASE_BODY frame
	float ImuLeverArmsZ; // [m] lever_arm-z CASE_BODY to IMU_BODY in CASE_BODY frame
	float VehicleEulerAnglesRoll; // RESERVED: [rad] VEHICLE_BODY to CASE installation attitude - roll
	float VehicleEulerAnglesPitch; // RESERVED: [rad] VEHICLE_BODY to CASE installation attitude - pitch
	float VehicleEulerAnglesTrueHeading; // RESERVED: [rad] VEHICLE_BODY to CASE installation attitude - heading
	float CaseToNavQuaterionS; // CASE to VEHICLE_BODY installation attitude quaternion [S]
	float CaseToNavQuaterionI; // CASE to VEHICLE_BODY installation attitude quaternion [I]
	float CaseToNavQuaterionJ; // CASE to VEHICLE_BODY installation attitude quaternion [J]
	float CaseToNavQuaterionK; // CASE to VEHICLE_BODY installation attitude quaternion [K]
	float RF1AntennaLeverArmStdv; // [m] GNSS RF #1 Antenna Lever-arm Estimate Uncertainty RSS
	float RF1AntennaLeverArmX; // [m] CASE_BODY to GNSS RF #1 Antenna in VEHICLE_BODY lever_arm-x
	float RF1AntennaLeverArmY; // [m] CASE_BODY to GNSS RF #1 Antenna in VEHICLE_BODY lever_arm-y
	float RF1AntennaLeverArmZ; // [m] CASE_BODY to GNSS RF #1 Antenna in VEHICLE_BODY lever_arm-z
	float RF2AuxiliaryAntennaLeverArmX; // [m] CASE_BODY to GNSS RF #2 Antenna in CASE_BODY lever_arm-x
	float RF2AuxiliaryAntennaLeverArmY; // [m] CASE_BODY to GNSS RF #2 Antenna in CASE_BODY lever_arm-y
	float RF2AuxiliaryAntennaLeverArmZ; // [m] CASE_BODY to GNSS RF #2 Antenna in CASE_BODY lever_arm-z
	float RF12LOSPitchEst; // [rad] Pitch estimate from RF1 to RF2
	float RF12LOSTrueHeadingEst; // [rad] True Heading estimate from RF1 to RF2
	float AntennaBoresightPitchEstStdv; // [rad] Dual Antenna Pitch boresight from VEHICLE_BODY frame
	float AntennaBoresightTrueHeadingEstStdv; // [rad] Dual Antenna True Heading boresight from VEHICLE_BODY frame
	float RF1MainAntennaLeverArmEstX; // [m] Estimated Lever Arm X of RF1 antenna in CASE_BODY frame
	float RF1MainAntennaLeverArmEstY; // [m] Estimated Lever Arm Y of RF1 antenna in CASE_BODY frame
	float RF1MainAntennaLeverArmEstZ; // [m] Estimated Lever Arm Z of RF1 antenna in CASE_BODY frame
	float RF1MainAntennaLeverArmEstStdvX; // [m] Estimated Lever Arm X STDV of RF1 antenna
	float RF1MainAntennaLeverArmEstStdvY; // [m] Estimated Lever Arm Y STDV of RF1 antenna
	float RF1MainAntennaLeverArmEstStdvZ; // [m] Estimated Lever Arm Z STDV of RF1 antenna
	float OdometerLeverArmEstX; // [m] Estimated Lever Arm X of DMI in VEHICLE_BODY frame
	float OdometerLeverArmEstY; // [m] Estimated Lever Arm Y of DMI in VEHICLE_BODY frame
	float OdometerLeverArmEstZ; // [m] Estimated Lever Arm Z of DMI in VEHICLE_BODY frame
	float OdometerLeverArmEstStdvX; // [m] Estimated Lever Arm X STDV of DMI
	float OdometerLeverArmEstStdvY; // [m] Estimated Lever Arm Y STDV of DMI
	float OdometerLeverArmEstStdvZ; // [m] Estimated Lever Arm Z STDV of DMI
	float OdometerBoresightTrueHeading; // [m] Estimated DMI True Heading Boresight from VEHICLE_BODY frame
	float OdometerBoresightPitch; // [m] Estimated DMI Pitch Boresight from VEHICLE_BODY frame
	float OdometerSFStdv; // Estimated DMI Scale Factor Correction STDV
	float OdometerLeverArmStoredX; // [m] Stored Lever Arm X of DMI in VEHICLE_BODY frame
	float OdometerLeverArmStoredY; // [m] Stored Lever Arm Y of DMI in VEHICLE_BODY frame
	float OdometerLeverArmStoredZ; // [m] Stored Lever Arm Z of DMI in VEHICLE_BODY frame
	float RF12BoresightTrueHeadingAdjustment; // [rad] RF1 to RF2 True Heading boresight adjustement
	float RF12BoresightPitchAdjustment; // [rad] RF1 to RF2 Pitch boresight adjustement
};

#endif // __HGuideAPI_Msg_6003_h__
