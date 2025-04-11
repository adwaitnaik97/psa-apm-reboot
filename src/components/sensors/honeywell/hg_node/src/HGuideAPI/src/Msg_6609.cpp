#include <include/HGuideAPI.h>
#include <include/Msg_6609.h>

#include <cmath>
#include <cstring>

#include <include/utils/bit_utilities.h>
#include <include/utils/byte_buffer.h>
#include <include/utils/ectos_constants.h>
#include <include/utils/Checksum.h>


// Definition for Static constants
const uint32_t Msg_6609::AddressId;
const uint32_t Msg_6609::MessageId;
const uint32_t Msg_6609::MessageLength;

Msg_6609::Msg_6609()
{
	Default();
}

void Msg_6609::Default()
{
	Checksum = 0;
	casterStatus.Default();
	clientStatus.Default();
	ggaRequired = 0;
	ggaLenValid = 0;
	pvtValid = 0;
	pvtLenValid = 0;
	WaitingForReply = 0;
	FoundICY200 = 0;
	InvalidReply = 0;
	TimedOut = 0;
	currentState = static_cast<ntrip_states_t>(0);
	previousState = static_cast<ntrip_states_t>(0);
	lastErrState = static_cast<ntrip_states_t>(0);
	for (unsigned int index = 0; index < 128; index++)
	{
		lastErrSting[index] = 0;
	}
}

bool Msg_6609::Serialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 160) return false;

	std::memset(buffer, 0, bufferSize);

	bool status_ok = true;
	ECTOS::BYTE_BUFFER::ByteOutputBuffer<unsigned int, unsigned char, ECTOS::BYTE_BUFFER::HGuideAPI_endianness> bb(buffer, bufferSize);

	bb.setOffset(0);	bb.put(AddressId);
	bb.setOffset(4);	bb.put(MessageId);
	bb.setOffset(8);	bb.put(MessageLength);
	int cIndex = 12;
	Checksum = 0;

	uint8_t casterStatus_tmp = 0; // temporary variable holding the custom bitfield
	casterStatus_tmp = ECTOS::BIT_UTILITIES::PackBool(casterStatus_tmp, 0, casterStatus.PortOpened, status_ok);
	casterStatus_tmp = ECTOS::BIT_UTILITIES::PackBool(casterStatus_tmp, 1, casterStatus.PortConnected, status_ok);
	casterStatus_tmp = ECTOS::BIT_UTILITIES::PackBool(casterStatus_tmp, 2, casterStatus.EthernetLink, status_ok);
	casterStatus_tmp = ECTOS::BIT_UTILITIES::PackBool(casterStatus_tmp, 3, casterStatus.DhcpBound, status_ok);
	casterStatus_tmp = ECTOS::BIT_UTILITIES::PackBool(casterStatus_tmp, 4, casterStatus.SocketConnected, status_ok);
	bb.setOffset(16);	bb.put(casterStatus_tmp);


	uint8_t clientStatus_tmp = 0; // temporary variable holding the custom bitfield
	clientStatus_tmp = ECTOS::BIT_UTILITIES::PackBool(clientStatus_tmp, 0, clientStatus.PortOpened, status_ok);
	clientStatus_tmp = ECTOS::BIT_UTILITIES::PackBool(clientStatus_tmp, 1, clientStatus.PortConnected, status_ok);
	clientStatus_tmp = ECTOS::BIT_UTILITIES::PackBool(clientStatus_tmp, 2, clientStatus.EthernetLink, status_ok);
	clientStatus_tmp = ECTOS::BIT_UTILITIES::PackBool(clientStatus_tmp, 3, clientStatus.DhcpBound, status_ok);
	clientStatus_tmp = ECTOS::BIT_UTILITIES::PackBool(clientStatus_tmp, 4, clientStatus.SocketConnected, status_ok);
	bb.setOffset(17);	bb.put(clientStatus_tmp);


	uint8_t bitfieldAtByte18 = 0; // temporary variable holding the bitfield
	bitfieldAtByte18 = ECTOS::BIT_UTILITIES::PackBool(bitfieldAtByte18, 0, ggaRequired, status_ok);
	bitfieldAtByte18 = ECTOS::BIT_UTILITIES::PackBool(bitfieldAtByte18, 1, ggaLenValid, status_ok);
	bitfieldAtByte18 = ECTOS::BIT_UTILITIES::PackBool(bitfieldAtByte18, 2, pvtValid, status_ok);
	bitfieldAtByte18 = ECTOS::BIT_UTILITIES::PackBool(bitfieldAtByte18, 3, pvtLenValid, status_ok);
	bb.setOffset(18);	bb.put(bitfieldAtByte18);


	uint8_t bitfieldAtByte19 = 0; // temporary variable holding the bitfield
	bitfieldAtByte19 = ECTOS::BIT_UTILITIES::PackBool(bitfieldAtByte19, 0, WaitingForReply, status_ok);
	bitfieldAtByte19 = ECTOS::BIT_UTILITIES::PackBool(bitfieldAtByte19, 1, FoundICY200, status_ok);
	bitfieldAtByte19 = ECTOS::BIT_UTILITIES::PackBool(bitfieldAtByte19, 2, InvalidReply, status_ok);
	bitfieldAtByte19 = ECTOS::BIT_UTILITIES::PackBool(bitfieldAtByte19, 3, TimedOut, status_ok);
	bb.setOffset(19);	bb.put(bitfieldAtByte19);

	bb.setOffset(20);	bb.put(static_cast<uint32_t>CHECK_MAX_UINT32(currentState));
	bb.setOffset(24);	bb.put(static_cast<uint32_t>CHECK_MAX_UINT32(previousState));
	bb.setOffset(28);	bb.put(static_cast<uint32_t>CHECK_MAX_UINT32(lastErrState));
	bb.setOffset(32);
	for (unsigned int index = 0; index < 128; index++)
	{
		bb.put(lastErrSting[index]);
	}
	Checksum = computeChecksum((uint32_t*)buffer, 40);
	bb.setOffset(cIndex);		bb.put(Checksum);
	return status_ok;
}

int Msg_6609::Deserialize(unsigned char * buffer, const unsigned int bufferSize)
{
	bool status_ok = true;
	if (bufferSize < 160) return -2;

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

	uint8_t casterStatus_tmp = 0; // temporary variable holding the custom bitfield
	bb.setOffset(16);	bb.get(casterStatus_tmp);
	casterStatus.PortOpened = ECTOS::BIT_UTILITIES::UnPackBool(casterStatus_tmp, 0,  status_ok);
	casterStatus.PortConnected = ECTOS::BIT_UTILITIES::UnPackBool(casterStatus_tmp, 1,  status_ok);
	casterStatus.EthernetLink = ECTOS::BIT_UTILITIES::UnPackBool(casterStatus_tmp, 2,  status_ok);
	casterStatus.DhcpBound = ECTOS::BIT_UTILITIES::UnPackBool(casterStatus_tmp, 3,  status_ok);
	casterStatus.SocketConnected = ECTOS::BIT_UTILITIES::UnPackBool(casterStatus_tmp, 4,  status_ok);

	uint8_t clientStatus_tmp = 0; // temporary variable holding the custom bitfield
	bb.setOffset(17);	bb.get(clientStatus_tmp);
	clientStatus.PortOpened = ECTOS::BIT_UTILITIES::UnPackBool(clientStatus_tmp, 0,  status_ok);
	clientStatus.PortConnected = ECTOS::BIT_UTILITIES::UnPackBool(clientStatus_tmp, 1,  status_ok);
	clientStatus.EthernetLink = ECTOS::BIT_UTILITIES::UnPackBool(clientStatus_tmp, 2,  status_ok);
	clientStatus.DhcpBound = ECTOS::BIT_UTILITIES::UnPackBool(clientStatus_tmp, 3,  status_ok);
	clientStatus.SocketConnected = ECTOS::BIT_UTILITIES::UnPackBool(clientStatus_tmp, 4,  status_ok);

	uint8_t bitfieldAtByte18 = 0; // temporary variable holding the bitfield
	bb.setOffset(18);	bb.get(bitfieldAtByte18);
	ggaRequired = ECTOS::BIT_UTILITIES::UnPackBool(bitfieldAtByte18, 0,  status_ok);
	ggaLenValid = ECTOS::BIT_UTILITIES::UnPackBool(bitfieldAtByte18, 1,  status_ok);
	pvtValid = ECTOS::BIT_UTILITIES::UnPackBool(bitfieldAtByte18, 2,  status_ok);
	pvtLenValid = ECTOS::BIT_UTILITIES::UnPackBool(bitfieldAtByte18, 3,  status_ok);

	uint8_t bitfieldAtByte19 = 0; // temporary variable holding the bitfield
	bb.setOffset(19);	bb.get(bitfieldAtByte19);
	WaitingForReply = ECTOS::BIT_UTILITIES::UnPackBool(bitfieldAtByte19, 0,  status_ok);
	FoundICY200 = ECTOS::BIT_UTILITIES::UnPackBool(bitfieldAtByte19, 1,  status_ok);
	InvalidReply = ECTOS::BIT_UTILITIES::UnPackBool(bitfieldAtByte19, 2,  status_ok);
	TimedOut = ECTOS::BIT_UTILITIES::UnPackBool(bitfieldAtByte19, 3,  status_ok);
	bb.setOffset(20);	currentState = static_cast<ntrip_states_t>(bb.get<uint32_t>());
	bb.setOffset(24);	previousState = static_cast<ntrip_states_t>(bb.get<uint32_t>());
	bb.setOffset(28);	lastErrState = static_cast<ntrip_states_t>(bb.get<uint32_t>());
	bb.setOffset(32);
	for (unsigned int index = 0; index < 128; index++)
	{
		bb.get(lastErrSting[index]);
	}
	bb.setOffset(12);	bb.get(Checksum);
	uint32_t computedCRC = computeChecksum((uint32_t*)buffer, 40);
	int retVal = 0;
	retVal = constCheck;
	if(computedCRC != Checksum)
		retVal = -1;
	return retVal;
}

