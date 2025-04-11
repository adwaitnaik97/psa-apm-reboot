#include <include/HGuideAPI.h>
#include <include/Msg_5104.h>

#include <cmath>
#include <cstring>

#include <include/utils/bit_utilities.h>
#include <include/utils/byte_buffer.h>
#include <include/utils/ectos_constants.h>
#include <include/utils/Checksum.h>


// Definition for Static constants
const uint32_t Msg_5104::AddressId;
const uint32_t Msg_5104::MessageId;
const uint32_t Msg_5104::MessageLength;

Msg_5104::Msg_5104()
{
	Default();
}

void Msg_5104::Default()
{
	Checksum = 0;
	systemTov = 0;
	gpsTov = 0;
	GPSMode = static_cast<gps_mode_table_t>(0);
	slot_offset = 0;
	freq_offset = 0;
	sat_type = 0;
	e_week = 0;
	e_time = 0;
	t_offset = 0;
	Nt = 0;
	issue = 0;
	health1 = 0;
	pos_x = 0;
	pos_y = 0;
	pos_z = 0;
	vel_x = 0;
	vel_y = 0;
	vel_z = 0;
	LS_acc_x = 0;
	LS_acc_y = 0;
	LS_acc_z = 0;
	tau_n = 0;
	delta_tau_n = 0;
	gamma = 0;
	Tk = 0;
	P = 0;
	Ft = 0;
	age = 0;
	Flags = 0;
	flag_p1 = 0;
	flag_p2 = 0;
	flag_p3 = 0;
	flag_p4 = 0;
}

bool Msg_5104::Serialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 188) return false;

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
	bb.setOffset(32);	bb.put(static_cast<uint32_t>CHECK_MAX_UINT32(GPSMode));
	bb.setOffset(36);	bb.put(slot_offset);
	bb.setOffset(38);	bb.put(freq_offset);
	bb.setOffset(40);	bb.put(sat_type);
	bb.setOffset(42);	bb.put(e_week);
	bb.setOffset(44);	bb.put(e_time);
	bb.setOffset(48);	bb.put(t_offset);
	bb.setOffset(52);	bb.put(Nt);
	bb.setOffset(56);	bb.put(issue);
	bb.setOffset(60);	bb.put(health1);
	bb.setOffset(64);	bb.put(pos_x);
	bb.setOffset(72);	bb.put(pos_y);
	bb.setOffset(80);	bb.put(pos_z);
	bb.setOffset(88);	bb.put(vel_x);
	bb.setOffset(96);	bb.put(vel_y);
	bb.setOffset(104);	bb.put(vel_z);
	bb.setOffset(112);	bb.put(LS_acc_x);
	bb.setOffset(120);	bb.put(LS_acc_y);
	bb.setOffset(128);	bb.put(LS_acc_z);
	bb.setOffset(136);	bb.put(tau_n);
	bb.setOffset(144);	bb.put(delta_tau_n);
	bb.setOffset(152);	bb.put(gamma);
	bb.setOffset(160);	bb.put(Tk);
	bb.setOffset(164);	bb.put(P);
	bb.setOffset(168);	bb.put(Ft);
	bb.setOffset(172);	bb.put(age);
	bb.setOffset(176);	bb.put(Flags);

	uint32_t bitfieldAtByte180 = 0; // temporary variable holding the bitfield
	bitfieldAtByte180 = ECTOS::BIT_UTILITIES::Pack(bitfieldAtByte180, 0, 1, flag_p1, status_ok);
	bitfieldAtByte180 = ECTOS::BIT_UTILITIES::PackBool(bitfieldAtByte180, 2, flag_p2, status_ok);
	bitfieldAtByte180 = ECTOS::BIT_UTILITIES::PackBool(bitfieldAtByte180, 3, flag_p3, status_ok);
	bitfieldAtByte180 = ECTOS::BIT_UTILITIES::PackBool(bitfieldAtByte180, 4, flag_p4, status_ok);
	bb.setOffset(180);	bb.put(bitfieldAtByte180);

	Checksum = computeChecksum((uint32_t*)buffer, 47);
	bb.setOffset(cIndex);		bb.put(Checksum);
	return status_ok;
}

int Msg_5104::Deserialize(unsigned char * buffer, const unsigned int bufferSize)
{
	bool status_ok = true;
	if (bufferSize < 188) return -2;

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
	bb.setOffset(32);	GPSMode = static_cast<gps_mode_table_t>(bb.get<uint32_t>());
	bb.setOffset(36);	bb.get(slot_offset);
	bb.setOffset(38);	bb.get(freq_offset);
	bb.setOffset(40);	bb.get(sat_type);
	bb.setOffset(42);	bb.get(e_week);
	bb.setOffset(44);	bb.get(e_time);
	bb.setOffset(48);	bb.get(t_offset);
	bb.setOffset(52);	bb.get(Nt);
	bb.setOffset(56);	bb.get(issue);
	bb.setOffset(60);	bb.get(health1);
	bb.setOffset(64);	bb.get(pos_x);
	bb.setOffset(72);	bb.get(pos_y);
	bb.setOffset(80);	bb.get(pos_z);
	bb.setOffset(88);	bb.get(vel_x);
	bb.setOffset(96);	bb.get(vel_y);
	bb.setOffset(104);	bb.get(vel_z);
	bb.setOffset(112);	bb.get(LS_acc_x);
	bb.setOffset(120);	bb.get(LS_acc_y);
	bb.setOffset(128);	bb.get(LS_acc_z);
	bb.setOffset(136);	bb.get(tau_n);
	bb.setOffset(144);	bb.get(delta_tau_n);
	bb.setOffset(152);	bb.get(gamma);
	bb.setOffset(160);	bb.get(Tk);
	bb.setOffset(164);	bb.get(P);
	bb.setOffset(168);	bb.get(Ft);
	bb.setOffset(172);	bb.get(age);
	bb.setOffset(176);	bb.get(Flags);

	uint32_t bitfieldAtByte180 = 0; // temporary variable holding the bitfield
	bb.setOffset(180);	bb.get(bitfieldAtByte180);
	flag_p1 = ECTOS::BIT_UTILITIES::UnPack(bitfieldAtByte180, 0, 1,  status_ok);
	flag_p2 = ECTOS::BIT_UTILITIES::UnPackBool(bitfieldAtByte180, 2,  status_ok);
	flag_p3 = ECTOS::BIT_UTILITIES::UnPackBool(bitfieldAtByte180, 3,  status_ok);
	flag_p4 = ECTOS::BIT_UTILITIES::UnPackBool(bitfieldAtByte180, 4,  status_ok);
	bb.setOffset(12);	bb.get(Checksum);
	uint32_t computedCRC = computeChecksum((uint32_t*)buffer, 47);
	int retVal = 0;
	retVal = constCheck;
	if(computedCRC != Checksum)
		retVal = -1;
	return retVal;
}

