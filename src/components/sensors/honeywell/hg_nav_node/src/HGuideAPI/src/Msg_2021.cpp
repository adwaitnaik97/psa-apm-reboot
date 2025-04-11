#include <include/HGuideAPI.h>
#include <include/Msg_2021.h>

#include <cmath>
#include <cstring>

#include <include/utils/bit_utilities.h>
#include <include/utils/byte_buffer.h>
#include <include/utils/ectos_constants.h>
#include <include/utils/Checksum.h>


// Definition for Static constants
const uint32_t Msg_2021::AddressId;
const uint32_t Msg_2021::MessageId;
const uint32_t Msg_2021::MessageLength;

Msg_2021::Msg_2021()
{
	Default();
}

void Msg_2021::Default()
{
	Checksum = 0;
	systemTov = 0;
	gpsTov = 0;
	gps_week = 0;
	initialization_status_1 = 0;
	initialization_status_2 = 0;
	number_tm_pps_pulses_rcvd = 0;
	number_time_msgs_rcvd = 0;
	clock_calibration_status = 0;
	oscillator_drift = 0;
	almanac_database = 0;
	ephemeris_database = 0;
	iono_database = 0;
	subframe_4_database = 0;
	subframe_5_database = 0;
}

bool Msg_2021::Serialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 224) return false;

	std::memset(buffer, 0, bufferSize);

	bool status_ok = true;
	ECTOS::BYTE_BUFFER::ByteOutputBuffer<unsigned int, unsigned char, ECTOS::BYTE_BUFFER::HGuideAPI_endianness> bb(buffer, bufferSize);

	bb.setOffset(0);	bb.put(AddressId);
	bb.setOffset(4);	bb.put(MessageId);
	bb.setOffset(8);	bb.put(MessageLength);
	int cIndex = 12;
	Checksum = 0;
	bb.setOffset(24);	bb.put(systemTov);
	bb.setOffset(32);	bb.put(gpsTov);
	bb.setOffset(40);	bb.put(gps_week);
	bb.setOffset(44);	bb.put(initialization_status_1);
	bb.setOffset(48);	bb.put(initialization_status_2);
	bb.setOffset(52);	bb.put(number_tm_pps_pulses_rcvd);
	bb.setOffset(56);	bb.put(number_time_msgs_rcvd);
	bb.setOffset(60);	bb.put(clock_calibration_status);
	bb.setOffset(64);	bb.put(oscillator_drift);
	bb.setOffset(72);	bb.put(almanac_database);
	bb.setOffset(76);	bb.put(ephemeris_database);
	bb.setOffset(80);	bb.put(iono_database);
	bb.setOffset(84);	bb.put(subframe_4_database);
	bb.setOffset(92);	bb.put(subframe_5_database);
	Checksum = computeChecksum((uint32_t*)buffer, 56);
	bb.setOffset(cIndex);		bb.put(Checksum);
	return status_ok;
}

int Msg_2021::Deserialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 224) return -2;

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
	bb.setOffset(24);	bb.get(systemTov);
	bb.setOffset(32);	bb.get(gpsTov);
	bb.setOffset(40);	bb.get(gps_week);
	bb.setOffset(44);	bb.get(initialization_status_1);
	bb.setOffset(48);	bb.get(initialization_status_2);
	bb.setOffset(52);	bb.get(number_tm_pps_pulses_rcvd);
	bb.setOffset(56);	bb.get(number_time_msgs_rcvd);
	bb.setOffset(60);	bb.get(clock_calibration_status);
	bb.setOffset(64);	bb.get(oscillator_drift);
	bb.setOffset(72);	bb.get(almanac_database);
	bb.setOffset(76);	bb.get(ephemeris_database);
	bb.setOffset(80);	bb.get(iono_database);
	bb.setOffset(84);	bb.get(subframe_4_database);
	bb.setOffset(92);	bb.get(subframe_5_database);
	bb.setOffset(12);	bb.get(Checksum);
	uint32_t computedCRC = computeChecksum((uint32_t*)buffer, 56);
	int retVal = 0;
	retVal = constCheck;
	if(computedCRC != Checksum)
		retVal = -1;
	return retVal;
}

