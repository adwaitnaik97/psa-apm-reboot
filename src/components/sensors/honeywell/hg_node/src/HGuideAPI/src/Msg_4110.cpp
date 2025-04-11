#include <include/HGuideAPI.h>
#include <include/Msg_4110.h>

#include <cmath>
#include <cstring>

#include <include/utils/bit_utilities.h>
#include <include/utils/byte_buffer.h>
#include <include/utils/ectos_constants.h>
#include <include/utils/Checksum.h>


// Definition for Static constants
const uint32_t Msg_4110::AddressId;
const uint32_t Msg_4110::MessageId;
const uint32_t Msg_4110::MessageLength;

Msg_4110::Msg_4110()
{
	Default();
}

void Msg_4110::Default()
{
	Checksum = 0;
	TimeOfValidityAdjustment = 0;
	GPS_TOV = 0;
	Body_Velocity_X = 0;
	Body_Velocity_Y = 0;
	Body_Velocity_Z = 0;
	Odo_Pulses = 0;
	Distance_per_Pulse = 0;
	Odo_Body_Vel_Status.Default();
	Message_Counter = 0;
	X_Y_Velocity_STDV = 0;
	Z_Velocity_STDV = 0;
}

bool Msg_4110::Serialize(unsigned char * buffer, const unsigned int bufferSize)
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
	bb.setOffset(16);	bb.put(TimeOfValidityAdjustment);
	bb.setOffset(24);	bb.put(GPS_TOV);
	bb.setOffset(32);	bb.put(Body_Velocity_X);
	bb.setOffset(36);	bb.put(Body_Velocity_Y);
	bb.setOffset(40);	bb.put(Body_Velocity_Z);
	bb.setOffset(44);	bb.put(Odo_Pulses);
	bb.setOffset(48);	bb.put(Distance_per_Pulse);

	uint32_t Odo_Body_Vel_Status_tmp = 0; // temporary variable holding the custom bitfield
	Odo_Body_Vel_Status_tmp = ECTOS::BIT_UTILITIES::PackBool(Odo_Body_Vel_Status_tmp, 0, Odo_Body_Vel_Status.Velocity_Valid, status_ok);
	Odo_Body_Vel_Status_tmp = ECTOS::BIT_UTILITIES::PackBool(Odo_Body_Vel_Status_tmp, 1, Odo_Body_Vel_Status.Odo_Pulse_Valid, status_ok);
	Odo_Body_Vel_Status_tmp = ECTOS::BIT_UTILITIES::PackBool(Odo_Body_Vel_Status_tmp, 2, Odo_Body_Vel_Status.TOV_Mode, status_ok);
	Odo_Body_Vel_Status_tmp = ECTOS::BIT_UTILITIES::PackBool(Odo_Body_Vel_Status_tmp, 3, Odo_Body_Vel_Status.Vel_Sending_Unit_Status, status_ok);
	Odo_Body_Vel_Status_tmp = ECTOS::BIT_UTILITIES::PackBool(Odo_Body_Vel_Status_tmp, 4, Odo_Body_Vel_Status.Zupt_Requested, status_ok);
	bb.setOffset(60);	bb.put(Odo_Body_Vel_Status_tmp);

	bb.setOffset(64);	bb.put(Message_Counter);
	bb.setOffset(68);	bb.put(X_Y_Velocity_STDV);
	bb.setOffset(72);	bb.put(Z_Velocity_STDV);
	Checksum = computeChecksum((uint32_t*)buffer, 24);
	bb.setOffset(cIndex);		bb.put(Checksum);
	return status_ok;
}

int Msg_4110::Deserialize(unsigned char * buffer, const unsigned int bufferSize)
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
	bb.setOffset(16);	bb.get(TimeOfValidityAdjustment);
	bb.setOffset(24);	bb.get(GPS_TOV);
	bb.setOffset(32);	bb.get(Body_Velocity_X);
	bb.setOffset(36);	bb.get(Body_Velocity_Y);
	bb.setOffset(40);	bb.get(Body_Velocity_Z);
	bb.setOffset(44);	bb.get(Odo_Pulses);
	bb.setOffset(48);	bb.get(Distance_per_Pulse);

	uint32_t Odo_Body_Vel_Status_tmp = 0; // temporary variable holding the custom bitfield
	bb.setOffset(60);	bb.get(Odo_Body_Vel_Status_tmp);
	Odo_Body_Vel_Status.Velocity_Valid = ECTOS::BIT_UTILITIES::UnPackBool(Odo_Body_Vel_Status_tmp, 0,  status_ok);
	Odo_Body_Vel_Status.Odo_Pulse_Valid = ECTOS::BIT_UTILITIES::UnPackBool(Odo_Body_Vel_Status_tmp, 1,  status_ok);
	Odo_Body_Vel_Status.TOV_Mode = ECTOS::BIT_UTILITIES::UnPackBool(Odo_Body_Vel_Status_tmp, 2,  status_ok);
	Odo_Body_Vel_Status.Vel_Sending_Unit_Status = ECTOS::BIT_UTILITIES::UnPackBool(Odo_Body_Vel_Status_tmp, 3,  status_ok);
	Odo_Body_Vel_Status.Zupt_Requested = ECTOS::BIT_UTILITIES::UnPackBool(Odo_Body_Vel_Status_tmp, 4,  status_ok);
	bb.setOffset(64);	bb.get(Message_Counter);
	bb.setOffset(68);	bb.get(X_Y_Velocity_STDV);
	bb.setOffset(72);	bb.get(Z_Velocity_STDV);
	bb.setOffset(12);	bb.get(Checksum);
	uint32_t computedCRC = computeChecksum((uint32_t*)buffer, 24);
	int retVal = 0;
	retVal = constCheck;
	if(computedCRC != Checksum)
		retVal = -1;
	return retVal;
}

