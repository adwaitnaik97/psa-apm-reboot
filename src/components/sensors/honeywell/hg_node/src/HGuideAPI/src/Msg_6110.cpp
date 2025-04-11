#include <include/HGuideAPI.h>
#include <include/Msg_6110.h>

#include <cmath>
#include <cstring>

#include <include/utils/bit_utilities.h>
#include <include/utils/byte_buffer.h>
#include <include/utils/ectos_constants.h>
#include <include/utils/Checksum.h>


// Definition for Static constants
const uint32_t Msg_6110::AddressId;
const uint32_t Msg_6110::MessageId;
const uint32_t Msg_6110::MessageLength;

Msg_6110::Msg_6110()
{
	Default();
}

void Msg_6110::Default()
{
	Checksum = 0;
	systemTov = 0;
	gpsTov = 0;
	Odo_X_Body_Distance = 0;
	Odo_Y_Body_Distance = 0;
	Odo_Z_Body_Distance = 0;
	Odo_X_Body_Distance_Compensated = 0;
	Odo_Y_Body_Distance_Compensated = 0;
	Odo_Z_Body_Distance_Compensated = 0;
	Odo_status.Default();
	uart_latency = 0;
	customer_latency = 0;
	InsGnssSummary.Default();
}

bool Msg_6110::Serialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 96) return false;

	std::memset(buffer, 0, bufferSize);

	bool status_ok = true;
	ECTOS::BYTE_BUFFER::ByteOutputBuffer<unsigned int, unsigned char, ECTOS::BYTE_BUFFER::HGuideAPI_endianness> bb(buffer, bufferSize);

	bb.setOffset(0);	bb.put(AddressId);
	bb.setOffset(4);	bb.put(MessageId);
	bb.setOffset(8);	bb.put(MessageLength);
	int cIndex = 12;
	Checksum = 0;
	bb.setOffset(16);	bb.put(systemTov);
	bb.setOffset(24);	bb.put(gpsTov);
	bb.setOffset(32);	bb.put(Odo_X_Body_Distance);
	bb.setOffset(36);	bb.put(Odo_Y_Body_Distance);
	bb.setOffset(40);	bb.put(Odo_Z_Body_Distance);
	bb.setOffset(48);	bb.put(Odo_X_Body_Distance_Compensated);
	bb.setOffset(52);	bb.put(Odo_Y_Body_Distance_Compensated);
	bb.setOffset(56);	bb.put(Odo_Z_Body_Distance_Compensated);

	uint32_t Odo_status_tmp = 0; // temporary variable holding the custom bitfield
	Odo_status_tmp = ECTOS::BIT_UTILITIES::PackBool(Odo_status_tmp, 0, Odo_status.Velocity_Valid, status_ok);
	Odo_status_tmp = ECTOS::BIT_UTILITIES::PackBool(Odo_status_tmp, 1, Odo_status.Odo_Pulse_Valid, status_ok);
	Odo_status_tmp = ECTOS::BIT_UTILITIES::PackBool(Odo_status_tmp, 2, Odo_status.TOV_Mode, status_ok);
	Odo_status_tmp = ECTOS::BIT_UTILITIES::PackBool(Odo_status_tmp, 3, Odo_status.Vel_Sending_Unit_Status, status_ok);
	Odo_status_tmp = ECTOS::BIT_UTILITIES::PackBool(Odo_status_tmp, 4, Odo_status.Zupt_Requested, status_ok);
	bb.setOffset(60);	bb.put(Odo_status_tmp);

	bb.setOffset(64);	bb.put(uart_latency);
	bb.setOffset(68);	bb.put(customer_latency);

	uint32_t InsGnssSummary_tmp = 0; // temporary variable holding the custom bitfield
	InsGnssSummary_tmp = ECTOS::BIT_UTILITIES::Pack(InsGnssSummary_tmp, 0, 3, InsGnssSummary.INSMode, status_ok);
	InsGnssSummary_tmp = ECTOS::BIT_UTILITIES::PackBool(InsGnssSummary_tmp, 4, InsGnssSummary.INSStatus, status_ok);
	InsGnssSummary_tmp = ECTOS::BIT_UTILITIES::PackBool(InsGnssSummary_tmp, 5, InsGnssSummary.IMUStatus, status_ok);
	InsGnssSummary_tmp = ECTOS::BIT_UTILITIES::PackBool(InsGnssSummary_tmp, 6, InsGnssSummary.GNSSStatus, status_ok);
	InsGnssSummary_tmp = ECTOS::BIT_UTILITIES::PackBool(InsGnssSummary_tmp, 7, InsGnssSummary.MotionDetectActive, status_ok);
	InsGnssSummary_tmp = ECTOS::BIT_UTILITIES::PackBool(InsGnssSummary_tmp, 8, InsGnssSummary.StationaryMeasurementsOn, status_ok);
	InsGnssSummary_tmp = ECTOS::BIT_UTILITIES::PackBool(InsGnssSummary_tmp, 9, InsGnssSummary.MDT1RotationRate, status_ok);
	InsGnssSummary_tmp = ECTOS::BIT_UTILITIES::PackBool(InsGnssSummary_tmp, 10, InsGnssSummary.MDT2SpeedSTDV, status_ok);
	InsGnssSummary_tmp = ECTOS::BIT_UTILITIES::PackBool(InsGnssSummary_tmp, 11, InsGnssSummary.MDT3AngularRateInstantBit, status_ok);
	InsGnssSummary_tmp = ECTOS::BIT_UTILITIES::PackBool(InsGnssSummary_tmp, 12, InsGnssSummary.MDT4LinearAccelerationBit, status_ok);
	InsGnssSummary_tmp = ECTOS::BIT_UTILITIES::PackBool(InsGnssSummary_tmp, 13, InsGnssSummary.MDT5OdometerBit, status_ok);
	InsGnssSummary_tmp = ECTOS::BIT_UTILITIES::PackBool(InsGnssSummary_tmp, 16, InsGnssSummary.MDNavigationMode, status_ok);
	InsGnssSummary_tmp = ECTOS::BIT_UTILITIES::Pack(InsGnssSummary_tmp, 17, 19, InsGnssSummary.NavSmoothingStatus, status_ok);
	InsGnssSummary_tmp = ECTOS::BIT_UTILITIES::Pack(InsGnssSummary_tmp, 28, 31, InsGnssSummary.GPSMode, status_ok);
	bb.setOffset(76);	bb.put(InsGnssSummary_tmp);

	Checksum = computeChecksum((uint32_t*)buffer, 24);
	bb.setOffset(cIndex);		bb.put(Checksum);
	return status_ok;
}

int Msg_6110::Deserialize(unsigned char * buffer, const unsigned int bufferSize)
{
	bool status_ok = true;
	if (bufferSize < 96) return -2;

	ECTOS::BYTE_BUFFER::ByteInputBuffer<unsigned int, unsigned char, ECTOS::BYTE_BUFFER::HGuideAPI_endianness> bb(buffer, bufferSize);

	int constCheck = 0;
	int numConsts = 0;
	// Compare with the static const variable to ensure they match. 
	uint32_t AddressId_In;
	bb.setOffset(0);	bb.get(AddressId_In);
	constCheck |= (AddressId != AddressId_In) << numConsts++;
	// Compare with the static const variable to ensure they match. 
	uint32_t MessageId_In;
	bb.setOffset(4);	bb.get(MessageId_In);
	constCheck |= (MessageId != MessageId_In) << numConsts++;
	// Compare with the static const variable to ensure they match. 
	uint32_t MessageLength_In;
	bb.setOffset(8);	bb.get(MessageLength_In);
	constCheck |= (MessageLength != MessageLength_In) << numConsts++;
	bb.setOffset(16);	bb.get(systemTov);
	bb.setOffset(24);	bb.get(gpsTov);
	bb.setOffset(32);	bb.get(Odo_X_Body_Distance);
	bb.setOffset(36);	bb.get(Odo_Y_Body_Distance);
	bb.setOffset(40);	bb.get(Odo_Z_Body_Distance);
	bb.setOffset(48);	bb.get(Odo_X_Body_Distance_Compensated);
	bb.setOffset(52);	bb.get(Odo_Y_Body_Distance_Compensated);
	bb.setOffset(56);	bb.get(Odo_Z_Body_Distance_Compensated);

	uint32_t Odo_status_tmp = 0; // temporary variable holding the custom bitfield
	bb.setOffset(60);	bb.get(Odo_status_tmp);
	Odo_status.Velocity_Valid = ECTOS::BIT_UTILITIES::UnPackBool(Odo_status_tmp, 0,  status_ok);
	Odo_status.Odo_Pulse_Valid = ECTOS::BIT_UTILITIES::UnPackBool(Odo_status_tmp, 1,  status_ok);
	Odo_status.TOV_Mode = ECTOS::BIT_UTILITIES::UnPackBool(Odo_status_tmp, 2,  status_ok);
	Odo_status.Vel_Sending_Unit_Status = ECTOS::BIT_UTILITIES::UnPackBool(Odo_status_tmp, 3,  status_ok);
	Odo_status.Zupt_Requested = ECTOS::BIT_UTILITIES::UnPackBool(Odo_status_tmp, 4,  status_ok);
	bb.setOffset(64);	bb.get(uart_latency);
	bb.setOffset(68);	bb.get(customer_latency);

	uint32_t InsGnssSummary_tmp = 0; // temporary variable holding the custom bitfield
	bb.setOffset(76);	bb.get(InsGnssSummary_tmp);
	InsGnssSummary.INSMode = static_cast<ins_mode_table_t>(ECTOS::BIT_UTILITIES::UnPack(InsGnssSummary_tmp, 0, 3,  status_ok));
	InsGnssSummary.INSStatus = ECTOS::BIT_UTILITIES::UnPackBool(InsGnssSummary_tmp, 4,  status_ok);
	InsGnssSummary.IMUStatus = ECTOS::BIT_UTILITIES::UnPackBool(InsGnssSummary_tmp, 5,  status_ok);
	InsGnssSummary.GNSSStatus = ECTOS::BIT_UTILITIES::UnPackBool(InsGnssSummary_tmp, 6,  status_ok);
	InsGnssSummary.MotionDetectActive = ECTOS::BIT_UTILITIES::UnPackBool(InsGnssSummary_tmp, 7,  status_ok);
	InsGnssSummary.StationaryMeasurementsOn = ECTOS::BIT_UTILITIES::UnPackBool(InsGnssSummary_tmp, 8,  status_ok);
	InsGnssSummary.MDT1RotationRate = ECTOS::BIT_UTILITIES::UnPackBool(InsGnssSummary_tmp, 9,  status_ok);
	InsGnssSummary.MDT2SpeedSTDV = ECTOS::BIT_UTILITIES::UnPackBool(InsGnssSummary_tmp, 10,  status_ok);
	InsGnssSummary.MDT3AngularRateInstantBit = ECTOS::BIT_UTILITIES::UnPackBool(InsGnssSummary_tmp, 11,  status_ok);
	InsGnssSummary.MDT4LinearAccelerationBit = ECTOS::BIT_UTILITIES::UnPackBool(InsGnssSummary_tmp, 12,  status_ok);
	InsGnssSummary.MDT5OdometerBit = ECTOS::BIT_UTILITIES::UnPackBool(InsGnssSummary_tmp, 13,  status_ok);
	InsGnssSummary.MDNavigationMode = ECTOS::BIT_UTILITIES::UnPackBool(InsGnssSummary_tmp, 16,  status_ok);
	InsGnssSummary.NavSmoothingStatus = ECTOS::BIT_UTILITIES::UnPack(InsGnssSummary_tmp, 17, 19,  status_ok);
	InsGnssSummary.GPSMode = static_cast<gps_mode_table_t>(ECTOS::BIT_UTILITIES::UnPack(InsGnssSummary_tmp, 28, 31,  status_ok));
	bb.setOffset(12);	bb.get(Checksum);
	uint32_t computedCRC = computeChecksum((uint32_t*)buffer, 24);
	int retVal = 0;
	retVal = constCheck;
	if(computedCRC != Checksum)
		retVal = -1;
	return retVal;
}

