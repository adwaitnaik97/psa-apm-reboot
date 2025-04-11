#include <include/HGuideAPI.h>
#include <include/Msg_9910.h>

#include <cmath>
#include <cstring>

#include <include/utils/bit_utilities.h>
#include <include/utils/byte_buffer.h>
#include <include/utils/ectos_constants.h>
#include <include/utils/Checksum.h>


// Definition for Static constants
const uint32_t Msg_9910::AddressId;
const uint32_t Msg_9910::MessageId;
const uint32_t Msg_9910::MessageLength;

Msg_9910::Msg_9910()
{
	Default();
}

void Msg_9910::Default()
{
	Checksum = 0;
	own_usage_rate_0 = 0;
	own_usage_rate_1 = 0;
	own_usage_rate_2 = 0;
	own_usage_rate_3 = 0;
	own_usage_rate_4 = 0;
	own_usage_rate_5 = 0;
	own_usage_rate_6 = 0;
	own_usage_rate_7 = 0;
	total_usage_rate_0 = 0;
	total_usage_rate_1 = 0;
	total_usage_rate_2 = 0;
	total_usage_rate_3 = 0;
	total_usage_rate_4 = 0;
	total_usage_rate_5 = 0;
	total_usage_rate_6 = 0;
	total_usage_rate_7 = 0;
	max_usage_rate_0 = 0;
	max_usage_rate_1 = 0;
	max_usage_rate_2 = 0;
	max_usage_rate_3 = 0;
	max_usage_rate_4 = 0;
	max_usage_rate_5 = 0;
	max_usage_rate_6 = 0;
	max_usage_rate_7 = 0;
	total_usage = 0;
	max_usage = 0;
}

bool Msg_9910::Serialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 44) return false;

	std::memset(buffer, 0, bufferSize);

	bool status_ok = true;
	ECTOS::BYTE_BUFFER::ByteOutputBuffer<unsigned int, unsigned char, ECTOS::BYTE_BUFFER::HGuideAPI_endianness> bb(buffer, bufferSize);

	bb.setOffset(0);	bb.put(AddressId);
	bb.setOffset(4);	bb.put(MessageId);
	bb.setOffset(8);	bb.put(MessageLength);
	int cIndex = 12;
	Checksum = 0;
	bb.setOffset(16);	bb.put(own_usage_rate_0 / (100/std::pow(2, 8)));
	bb.setOffset(17);	bb.put(own_usage_rate_1 / (100/std::pow(2, 8)));
	bb.setOffset(18);	bb.put(own_usage_rate_2 / (100/std::pow(2, 8)));
	bb.setOffset(19);	bb.put(own_usage_rate_3 / (100/std::pow(2, 8)));
	bb.setOffset(20);	bb.put(own_usage_rate_4 / (100/std::pow(2, 8)));
	bb.setOffset(21);	bb.put(own_usage_rate_5 / (100/std::pow(2, 8)));
	bb.setOffset(22);	bb.put(own_usage_rate_6 / (100/std::pow(2, 8)));
	bb.setOffset(23);	bb.put(own_usage_rate_7 / (100/std::pow(2, 8)));
	bb.setOffset(24);	bb.put(total_usage_rate_0 / (100/std::pow(2, 8)));
	bb.setOffset(25);	bb.put(total_usage_rate_1 / (100/std::pow(2, 8)));
	bb.setOffset(26);	bb.put(total_usage_rate_2 / (100/std::pow(2, 8)));
	bb.setOffset(27);	bb.put(total_usage_rate_3 / (100/std::pow(2, 8)));
	bb.setOffset(28);	bb.put(total_usage_rate_4 / (100/std::pow(2, 8)));
	bb.setOffset(29);	bb.put(total_usage_rate_5 / (100/std::pow(2, 8)));
	bb.setOffset(30);	bb.put(total_usage_rate_6 / (100/std::pow(2, 8)));
	bb.setOffset(31);	bb.put(total_usage_rate_7 / (100/std::pow(2, 8)));
	bb.setOffset(32);	bb.put(max_usage_rate_0 / (100/std::pow(2, 8)));
	bb.setOffset(33);	bb.put(max_usage_rate_1 / (100/std::pow(2, 8)));
	bb.setOffset(34);	bb.put(max_usage_rate_2 / (100/std::pow(2, 8)));
	bb.setOffset(35);	bb.put(max_usage_rate_3 / (100/std::pow(2, 8)));
	bb.setOffset(36);	bb.put(max_usage_rate_4 / (100/std::pow(2, 8)));
	bb.setOffset(37);	bb.put(max_usage_rate_5 / (100/std::pow(2, 8)));
	bb.setOffset(38);	bb.put(max_usage_rate_6 / (100/std::pow(2, 8)));
	bb.setOffset(39);	bb.put(max_usage_rate_7 / (100/std::pow(2, 8)));
	bb.setOffset(40);	bb.put(total_usage / (100/std::pow(2, 8)));
	bb.setOffset(41);	bb.put(max_usage / (100/std::pow(2, 8)));
	Checksum = computeChecksum((uint32_t*)buffer, 11);
	bb.setOffset(cIndex);		bb.put(Checksum);
	return status_ok;
}

int Msg_9910::Deserialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 44) return -2;

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
	bb.setOffset(16);	own_usage_rate_0 = bb.get<uint8_t>() * (100/std::pow(2, 8));
	bb.setOffset(17);	own_usage_rate_1 = bb.get<uint8_t>() * (100/std::pow(2, 8));
	bb.setOffset(18);	own_usage_rate_2 = bb.get<uint8_t>() * (100/std::pow(2, 8));
	bb.setOffset(19);	own_usage_rate_3 = bb.get<uint8_t>() * (100/std::pow(2, 8));
	bb.setOffset(20);	own_usage_rate_4 = bb.get<uint8_t>() * (100/std::pow(2, 8));
	bb.setOffset(21);	own_usage_rate_5 = bb.get<uint8_t>() * (100/std::pow(2, 8));
	bb.setOffset(22);	own_usage_rate_6 = bb.get<uint8_t>() * (100/std::pow(2, 8));
	bb.setOffset(23);	own_usage_rate_7 = bb.get<uint8_t>() * (100/std::pow(2, 8));
	bb.setOffset(24);	total_usage_rate_0 = bb.get<uint8_t>() * (100/std::pow(2, 8));
	bb.setOffset(25);	total_usage_rate_1 = bb.get<uint8_t>() * (100/std::pow(2, 8));
	bb.setOffset(26);	total_usage_rate_2 = bb.get<uint8_t>() * (100/std::pow(2, 8));
	bb.setOffset(27);	total_usage_rate_3 = bb.get<uint8_t>() * (100/std::pow(2, 8));
	bb.setOffset(28);	total_usage_rate_4 = bb.get<uint8_t>() * (100/std::pow(2, 8));
	bb.setOffset(29);	total_usage_rate_5 = bb.get<uint8_t>() * (100/std::pow(2, 8));
	bb.setOffset(30);	total_usage_rate_6 = bb.get<uint8_t>() * (100/std::pow(2, 8));
	bb.setOffset(31);	total_usage_rate_7 = bb.get<uint8_t>() * (100/std::pow(2, 8));
	bb.setOffset(32);	max_usage_rate_0 = bb.get<uint8_t>() * (100/std::pow(2, 8));
	bb.setOffset(33);	max_usage_rate_1 = bb.get<uint8_t>() * (100/std::pow(2, 8));
	bb.setOffset(34);	max_usage_rate_2 = bb.get<uint8_t>() * (100/std::pow(2, 8));
	bb.setOffset(35);	max_usage_rate_3 = bb.get<uint8_t>() * (100/std::pow(2, 8));
	bb.setOffset(36);	max_usage_rate_4 = bb.get<uint8_t>() * (100/std::pow(2, 8));
	bb.setOffset(37);	max_usage_rate_5 = bb.get<uint8_t>() * (100/std::pow(2, 8));
	bb.setOffset(38);	max_usage_rate_6 = bb.get<uint8_t>() * (100/std::pow(2, 8));
	bb.setOffset(39);	max_usage_rate_7 = bb.get<uint8_t>() * (100/std::pow(2, 8));
	bb.setOffset(40);	total_usage = bb.get<uint8_t>() * (100/std::pow(2, 8));
	bb.setOffset(41);	max_usage = bb.get<uint8_t>() * (100/std::pow(2, 8));
	bb.setOffset(12);	bb.get(Checksum);
	uint32_t computedCRC = computeChecksum((uint32_t*)buffer, 11);
	int retVal = 0;
	retVal = constCheck;
	if(computedCRC != Checksum)
		retVal = -1;
	return retVal;
}

