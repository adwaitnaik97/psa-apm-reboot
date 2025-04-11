#include <include/HGuideAPI.h>
#include <include/Msg_2005.h>

#include <cmath>
#include <cstring>

#include <include/utils/bit_utilities.h>
#include <include/utils/byte_buffer.h>
#include <include/utils/ectos_constants.h>
#include <include/utils/Checksum.h>


// Definition for Static constants
const uint32_t Msg_2005::AddressId;
const uint32_t Msg_2005::MessageId;
const uint32_t Msg_2005::MessageLength;

Msg_2005::Msg_2005()
{
	Default();
}

void Msg_2005::Default()
{
	Checksum = 0;
	PortId = static_cast<port_id_t>(0);
	InterfaceSelect = static_cast<interface_protocol_t>(0);
	BaudRate = static_cast<uart_baud_type_t>(0);
	NumberOfBits = static_cast<uart_number_of_bits_t>(0);
	Parity = static_cast<uart_parity_t>(0);
	NumberOfStopBits = static_cast<uart_number_of_stop_bits_t>(0);
	SaveConfiguration = static_cast<save_configuration_t>(0);
	parameter_1 = 0;
	parameter_2 = 0;
	parameter_3 = 0;
	parameter_4 = 0;
}

bool Msg_2005::Serialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 40) return false;

	std::memset(buffer, 0, bufferSize);

	bool status_ok = true;
	ECTOS::BYTE_BUFFER::ByteOutputBuffer<unsigned int, unsigned char, ECTOS::BYTE_BUFFER::HGuideAPI_endianness> bb(buffer, bufferSize);

	bb.setOffset(0);	bb.put(AddressId);
	bb.setOffset(4);	bb.put(MessageId);
	bb.setOffset(8);	bb.put(MessageLength);
	int cIndex = 12;
	Checksum = 0;
	bb.setOffset(16);	bb.put(static_cast<uint16_t>CHECK_MAX_UINT16(PortId));
	bb.setOffset(18);	bb.put(static_cast<uint16_t>CHECK_MAX_UINT16(InterfaceSelect));
	bb.setOffset(20);	bb.put(static_cast<uint8_t>CHECK_MAX_UINT8(BaudRate));

	uint8_t bitfieldAtByte21 = 0; // temporary variable holding the bitfield
	bitfieldAtByte21 = ECTOS::BIT_UTILITIES::Pack(bitfieldAtByte21, 0, 1, NumberOfBits, status_ok);
	bitfieldAtByte21 = ECTOS::BIT_UTILITIES::Pack(bitfieldAtByte21, 2, 4, Parity, status_ok);
	bitfieldAtByte21 = ECTOS::BIT_UTILITIES::Pack(bitfieldAtByte21, 5, 7, NumberOfStopBits, status_ok);
	bb.setOffset(21);	bb.put(bitfieldAtByte21);

	bb.setOffset(22);	bb.put(static_cast<uint16_t>CHECK_MAX_UINT16(SaveConfiguration));
	bb.setOffset(24);	bb.put(parameter_1);
	bb.setOffset(28);	bb.put(parameter_2);
	bb.setOffset(32);	bb.put(parameter_3);
	bb.setOffset(36);	bb.put(parameter_4);
	Checksum = computeChecksum((uint32_t*)buffer, 10);
	bb.setOffset(cIndex);		bb.put(Checksum);
	return status_ok;
}

int Msg_2005::Deserialize(unsigned char * buffer, const unsigned int bufferSize)
{
	bool status_ok = true;
	if (bufferSize < 40) return -2;

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
	bb.setOffset(16);	PortId = static_cast<port_id_t>(bb.get<uint16_t>());
	bb.setOffset(18);	InterfaceSelect = static_cast<interface_protocol_t>(bb.get<uint16_t>());
	bb.setOffset(20);	BaudRate = static_cast<uart_baud_type_t>(bb.get<uint8_t>());

	uint8_t bitfieldAtByte21 = 0; // temporary variable holding the bitfield
	bb.setOffset(21);	bb.get(bitfieldAtByte21);
	NumberOfBits = static_cast<uart_number_of_bits_t>(ECTOS::BIT_UTILITIES::UnPack(bitfieldAtByte21, 0, 1,  status_ok));
	Parity = static_cast<uart_parity_t>(ECTOS::BIT_UTILITIES::UnPack(bitfieldAtByte21, 2, 4,  status_ok));
	NumberOfStopBits = static_cast<uart_number_of_stop_bits_t>(ECTOS::BIT_UTILITIES::UnPack(bitfieldAtByte21, 5, 7,  status_ok));
	bb.setOffset(22);	SaveConfiguration = static_cast<save_configuration_t>(bb.get<uint16_t>());
	bb.setOffset(24);	bb.get(parameter_1);
	bb.setOffset(28);	bb.get(parameter_2);
	bb.setOffset(32);	bb.get(parameter_3);
	bb.setOffset(36);	bb.get(parameter_4);
	bb.setOffset(12);	bb.get(Checksum);
	uint32_t computedCRC = computeChecksum((uint32_t*)buffer, 10);
	int retVal = 0;
	retVal = constCheck;
	if(computedCRC != Checksum)
		retVal = -1;
	return retVal;
}

