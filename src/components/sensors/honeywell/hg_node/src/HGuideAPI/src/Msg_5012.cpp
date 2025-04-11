#include <include/HGuideAPI.h>
#include <include/Msg_5012.h>

#include <cmath>
#include <cstring>

#include <include/utils/bit_utilities.h>
#include <include/utils/byte_buffer.h>
#include <include/utils/ectos_constants.h>
#include <include/utils/Checksum.h>


// Definition for Static constants
const uint32_t Msg_5012::AddressId;
const uint32_t Msg_5012::MessageId;
const uint32_t Msg_5012::MessageLength;

Msg_5012::Msg_5012()
{
	Default();
}

void Msg_5012::Default()
{
	Checksum = 0;
	systemTov = 0;
	Temperature = 0;
	RF1_SupplyVoltage = 0;
	RF1_SupplyCurrent = 0;
	RF2_SupplyVoltage = 0;
	RF2_SupplyCurrent = 0;
	Voltage3V3 = 0;
	Voltage1V8 = 0;
	Voltage1V2 = 0;
	Voltage5V0 = 0;
	ptc = 0;
	ptm = 0;
	memUsage = 0;
	mode = 0;
	power_cycle_count = 0;
	eti = 0;
}

bool Msg_5012::Serialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 80) return false;

	std::memset(buffer, 0, bufferSize);

	bool status_ok = true;
	ECTOS::BYTE_BUFFER::ByteOutputBuffer<unsigned int, unsigned char, ECTOS::BYTE_BUFFER::HGuideAPI_endianness> bb(buffer, bufferSize);

	bb.setOffset(0);	bb.put(AddressId);
	bb.setOffset(4);	bb.put(MessageId);
	bb.setOffset(8);	bb.put(MessageLength);
	int cIndex = 12;
	Checksum = 0;
	bb.setOffset(16);	bb.put(systemTov);
	bb.setOffset(20);	bb.put(Temperature);
	bb.setOffset(24);	bb.put(RF1_SupplyVoltage);
	bb.setOffset(28);	bb.put(RF1_SupplyCurrent);
	bb.setOffset(32);	bb.put(RF2_SupplyVoltage);
	bb.setOffset(36);	bb.put(RF2_SupplyCurrent);
	bb.setOffset(40);	bb.put(Voltage3V3);
	bb.setOffset(44);	bb.put(Voltage1V8);
	bb.setOffset(48);	bb.put(Voltage1V2);
	bb.setOffset(52);	bb.put(Voltage5V0);
	bb.setOffset(56);	bb.put(ptc / (1.0/255));
	bb.setOffset(57);	bb.put(ptm / (1.0/255));
	bb.setOffset(58);	bb.put(memUsage / (1.0/255));
	bb.setOffset(60);	bb.put(mode);
	bb.setOffset(64);	bb.put(power_cycle_count);
	bb.setOffset(68);	bb.put(eti);
	Checksum = computeChecksum((uint32_t*)buffer, 20);
	bb.setOffset(cIndex);		bb.put(Checksum);
	return status_ok;
}

int Msg_5012::Deserialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 80) return -2;

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
	bb.setOffset(20);	bb.get(Temperature);
	bb.setOffset(24);	bb.get(RF1_SupplyVoltage);
	bb.setOffset(28);	bb.get(RF1_SupplyCurrent);
	bb.setOffset(32);	bb.get(RF2_SupplyVoltage);
	bb.setOffset(36);	bb.get(RF2_SupplyCurrent);
	bb.setOffset(40);	bb.get(Voltage3V3);
	bb.setOffset(44);	bb.get(Voltage1V8);
	bb.setOffset(48);	bb.get(Voltage1V2);
	bb.setOffset(52);	bb.get(Voltage5V0);
	bb.setOffset(56);	ptc = bb.get<uint8_t>() * (1.0/255);
	bb.setOffset(57);	ptm = bb.get<uint8_t>() * (1.0/255);
	bb.setOffset(58);	memUsage = bb.get<uint8_t>() * (1.0/255);
	bb.setOffset(60);	bb.get(mode);
	bb.setOffset(64);	bb.get(power_cycle_count);
	bb.setOffset(68);	bb.get(eti);
	bb.setOffset(12);	bb.get(Checksum);
	uint32_t computedCRC = computeChecksum((uint32_t*)buffer, 20);
	int retVal = 0;
	retVal = constCheck;
	if(computedCRC != Checksum)
		retVal = -1;
	return retVal;
}

