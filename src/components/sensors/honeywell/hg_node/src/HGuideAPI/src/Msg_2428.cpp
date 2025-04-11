#include <include/HGuideAPI.h>
#include <include/Msg_2428.h>

#include <cmath>
#include <cstring>

#include <include/utils/bit_utilities.h>
#include <include/utils/byte_buffer.h>
#include <include/utils/ectos_constants.h>
#include <include/utils/Checksum.h>


// Definition for Static constants
const uint32_t Msg_2428::AddressId;
const uint32_t Msg_2428::MessageId;
const uint32_t Msg_2428::MessageLength;

Msg_2428::Msg_2428()
{
	Default();
}

void Msg_2428::Default()
{
	Checksum = 0;
	INSMode = static_cast<ins_mode_table_t>(0);
	systemTov = 0;
	gpsTov = 0;
	gps_week = 0;
	GNSS_Position_X = 0;
	GNSS_Position_Y = 0;
	GNSS_Position_Z = 0;
	GNSS_Velocity_X = 0;
	GNSS_Velocity_Y = 0;
	GNSS_Velocity_Z = 0;
	Number_of_PR_channels = 0;
	GPS_PR_Channel_1 = 0;
	GPS_PR_Channel_2 = 0;
	GPS_PR_Channel_3 = 0;
	GPS_PR_Channel_4 = 0;
	GPS_PR_Channel_5 = 0;
	GPS_PR_Channel_6 = 0;
	GPS_PR_Channel_7 = 0;
	GPS_PR_Channel_8 = 0;
	GPS_PR_Channel_9 = 0;
	GPS_PR_Channel_10 = 0;
	GPS_PR_Channel_11 = 0;
	GPS_PR_Channel_12 = 0;
	Number_of_DR_channels = 0;
	GPS_DR_Channel_1 = 0;
	GPS_DR_Channel_2 = 0;
	GPS_DR_Channel_3 = 0;
	GPS_DR_Channel_4 = 0;
	GPS_DR_Channel_5 = 0;
	GPS_DR_Channel_6 = 0;
	GPS_DR_Channel_7 = 0;
	GPS_DR_Channel_8 = 0;
	GPS_DR_Channel_9 = 0;
	GPS_DR_Channel_10 = 0;
	GPS_DR_Channel_11 = 0;
	GPS_DR_Channel_12 = 0;
}

bool Msg_2428::Serialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 180) return false;

	std::memset(buffer, 0, bufferSize);

	bool status_ok = true;
	ECTOS::BYTE_BUFFER::ByteOutputBuffer<unsigned int, unsigned char, ECTOS::BYTE_BUFFER::HGuideAPI_endianness> bb(buffer, bufferSize);

	bb.setOffset(0);	bb.put(AddressId);
	bb.setOffset(4);	bb.put(MessageId);
	bb.setOffset(8);	bb.put(MessageLength);
	int cIndex = 12;
	Checksum = 0;
	bb.setOffset(20);	bb.put(static_cast<uint32_t>CHECK_MAX_UINT32(INSMode));
	bb.setOffset(24);	bb.put(systemTov);
	bb.setOffset(32);	bb.put(gpsTov);
	bb.setOffset(40);	bb.put(gps_week);
	bb.setOffset(44);	bb.put(GNSS_Position_X);
	bb.setOffset(48);	bb.put(GNSS_Position_Y);
	bb.setOffset(52);	bb.put(GNSS_Position_Z);
	bb.setOffset(56);	bb.put(GNSS_Velocity_X);
	bb.setOffset(60);	bb.put(GNSS_Velocity_Y);
	bb.setOffset(64);	bb.put(GNSS_Velocity_Z);
	bb.setOffset(72);	bb.put(Number_of_PR_channels);
	bb.setOffset(76);	bb.put(GPS_PR_Channel_1);
	bb.setOffset(80);	bb.put(GPS_PR_Channel_2);
	bb.setOffset(84);	bb.put(GPS_PR_Channel_3);
	bb.setOffset(88);	bb.put(GPS_PR_Channel_4);
	bb.setOffset(92);	bb.put(GPS_PR_Channel_5);
	bb.setOffset(96);	bb.put(GPS_PR_Channel_6);
	bb.setOffset(100);	bb.put(GPS_PR_Channel_7);
	bb.setOffset(104);	bb.put(GPS_PR_Channel_8);
	bb.setOffset(108);	bb.put(GPS_PR_Channel_9);
	bb.setOffset(112);	bb.put(GPS_PR_Channel_10);
	bb.setOffset(116);	bb.put(GPS_PR_Channel_11);
	bb.setOffset(120);	bb.put(GPS_PR_Channel_12);
	bb.setOffset(124);	bb.put(Number_of_DR_channels);
	bb.setOffset(128);	bb.put(GPS_DR_Channel_1);
	bb.setOffset(132);	bb.put(GPS_DR_Channel_2);
	bb.setOffset(136);	bb.put(GPS_DR_Channel_3);
	bb.setOffset(140);	bb.put(GPS_DR_Channel_4);
	bb.setOffset(144);	bb.put(GPS_DR_Channel_5);
	bb.setOffset(148);	bb.put(GPS_DR_Channel_6);
	bb.setOffset(152);	bb.put(GPS_DR_Channel_7);
	bb.setOffset(156);	bb.put(GPS_DR_Channel_8);
	bb.setOffset(160);	bb.put(GPS_DR_Channel_9);
	bb.setOffset(164);	bb.put(GPS_DR_Channel_10);
	bb.setOffset(168);	bb.put(GPS_DR_Channel_11);
	bb.setOffset(172);	bb.put(GPS_DR_Channel_12);
	Checksum = computeChecksum((uint32_t*)buffer, 45);
	bb.setOffset(cIndex);		bb.put(Checksum);
	return status_ok;
}

int Msg_2428::Deserialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 180) return -2;

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
	bb.setOffset(20);	INSMode = static_cast<ins_mode_table_t>(bb.get<uint32_t>());
	bb.setOffset(24);	bb.get(systemTov);
	bb.setOffset(32);	bb.get(gpsTov);
	bb.setOffset(40);	bb.get(gps_week);
	bb.setOffset(44);	bb.get(GNSS_Position_X);
	bb.setOffset(48);	bb.get(GNSS_Position_Y);
	bb.setOffset(52);	bb.get(GNSS_Position_Z);
	bb.setOffset(56);	bb.get(GNSS_Velocity_X);
	bb.setOffset(60);	bb.get(GNSS_Velocity_Y);
	bb.setOffset(64);	bb.get(GNSS_Velocity_Z);
	bb.setOffset(72);	bb.get(Number_of_PR_channels);
	bb.setOffset(76);	bb.get(GPS_PR_Channel_1);
	bb.setOffset(80);	bb.get(GPS_PR_Channel_2);
	bb.setOffset(84);	bb.get(GPS_PR_Channel_3);
	bb.setOffset(88);	bb.get(GPS_PR_Channel_4);
	bb.setOffset(92);	bb.get(GPS_PR_Channel_5);
	bb.setOffset(96);	bb.get(GPS_PR_Channel_6);
	bb.setOffset(100);	bb.get(GPS_PR_Channel_7);
	bb.setOffset(104);	bb.get(GPS_PR_Channel_8);
	bb.setOffset(108);	bb.get(GPS_PR_Channel_9);
	bb.setOffset(112);	bb.get(GPS_PR_Channel_10);
	bb.setOffset(116);	bb.get(GPS_PR_Channel_11);
	bb.setOffset(120);	bb.get(GPS_PR_Channel_12);
	bb.setOffset(124);	bb.get(Number_of_DR_channels);
	bb.setOffset(128);	bb.get(GPS_DR_Channel_1);
	bb.setOffset(132);	bb.get(GPS_DR_Channel_2);
	bb.setOffset(136);	bb.get(GPS_DR_Channel_3);
	bb.setOffset(140);	bb.get(GPS_DR_Channel_4);
	bb.setOffset(144);	bb.get(GPS_DR_Channel_5);
	bb.setOffset(148);	bb.get(GPS_DR_Channel_6);
	bb.setOffset(152);	bb.get(GPS_DR_Channel_7);
	bb.setOffset(156);	bb.get(GPS_DR_Channel_8);
	bb.setOffset(160);	bb.get(GPS_DR_Channel_9);
	bb.setOffset(164);	bb.get(GPS_DR_Channel_10);
	bb.setOffset(168);	bb.get(GPS_DR_Channel_11);
	bb.setOffset(172);	bb.get(GPS_DR_Channel_12);
	bb.setOffset(12);	bb.get(Checksum);
	uint32_t computedCRC = computeChecksum((uint32_t*)buffer, 45);
	int retVal = 0;
	retVal = constCheck;
	if(computedCRC != Checksum)
		retVal = -1;
	return retVal;
}

