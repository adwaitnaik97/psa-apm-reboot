#include <include/HGuideAPI.h>
#include <include/Msg_6721.h>

#include <cmath>
#include <cstring>

#include <include/utils/bit_utilities.h>
#include <include/utils/byte_buffer.h>
#include <include/utils/ectos_constants.h>
#include <include/utils/Checksum.h>


// Definition for Static constants
const uint32_t Msg_6721::AddressId;
const uint32_t Msg_6721::MessageId;
const uint32_t Msg_6721::MessageLength;

Msg_6721::Msg_6721()
{
	Default();
}

void Msg_6721::Default()
{
	Checksum = 0;
	systemTov = 0;
	serialNumber = 0;
	version = 0;
	offsetOfData = 0;
	year = 0;
	month = 0;
	day = 0;
	hour = 0;
	minute = 0;
	seconds = 0;
	microseconds = 0;
	numOfBeams = 0;
	error = 0;
	status.Default();
	speedOfSound = 0;
	temperature = 0;
	pressure = 0;
	distBeam0 = 0;
	distBeam1 = 0;
	distBeam2 = 0;
	distBeam3 = 0;
	dt1Beam0 = 0;
	dt1Beam1 = 0;
	dt1Beam2 = 0;
	dt1Beam3 = 0;
	dt2Beam0 = 0;
	dt2Beam1 = 0;
	dt2Beam2 = 0;
	dt2Beam3 = 0;
	timeVelEstBeam0 = 0;
	timeVelEstBeam1 = 0;
	timeVelEstBeam2 = 0;
	timeVelEstBeam3 = 0;
	dt1X = 0;
	dt1Y = 0;
	dt1Z1 = 0;
	dt1Z2 = 0;
	dt2X = 0;
	dt2Y = 0;
	dt2Z1 = 0;
	dt2Z2 = 0;
	timeVelEstX = 0;
	timeVelEstY = 0;
	timeVelEstZ1 = 0;
	timeVelEstZ2 = 0;
	velBeam0 = -32.768;
	velBeam1 = -32.768;
	velBeam2 = -32.768;
	velBeam3 = -32.768;
	fomBeam0 = 10.0;
	fomBeam1 = 10.0;
	fomBeam2 = 10.0;
	fomBeam3 = 10.0;
	velX = -32.768;
	velY = -32.768;
	velZ1 = -32.768;
	velZ2 = -32.768;
	fomX = 10.0;
	fomY = 10.0;
	fomZ1 = 10.0;
	fomZ2 = 10.0;
}

bool Msg_6721::Serialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 236) return false;

	std::memset(buffer, 0, bufferSize);

	bool status_ok = true;
	ECTOS::BYTE_BUFFER::ByteOutputBuffer<unsigned int, unsigned char, ECTOS::BYTE_BUFFER::HGuideAPI_endianness> bb(buffer, bufferSize);

	bb.setOffset(0);	bb.put(AddressId);
	bb.setOffset(4);	bb.put(MessageId);
	int cIndex = 8;
	Checksum = 0;
	bb.setOffset(12);	bb.put(MessageLength);
	bb.setOffset(16);	bb.put(systemTov);
	bb.setOffset(24);	bb.put(serialNumber);
	bb.setOffset(28);	bb.put(version);
	bb.setOffset(29);	bb.put(offsetOfData);
	bb.setOffset(30);	bb.put(year);
	bb.setOffset(31);	bb.put(month);
	bb.setOffset(32);	bb.put(day);
	bb.setOffset(33);	bb.put(hour);
	bb.setOffset(34);	bb.put(minute);
	bb.setOffset(35);	bb.put(seconds);
	bb.setOffset(36);	bb.put(microseconds / (100));
	bb.setOffset(38);	bb.put(numOfBeams);
	bb.setOffset(40);	bb.put(error);

	uint32_t status_tmp = 0; // temporary variable holding the custom bitfield
	status_tmp = ECTOS::BIT_UTILITIES::PackBool(status_tmp, 0, status.beam1_velocityValid, status_ok);
	status_tmp = ECTOS::BIT_UTILITIES::PackBool(status_tmp, 1, status.beam2_velocityValid, status_ok);
	status_tmp = ECTOS::BIT_UTILITIES::PackBool(status_tmp, 2, status.beam3_velocityValid, status_ok);
	status_tmp = ECTOS::BIT_UTILITIES::PackBool(status_tmp, 3, status.beam4_velocityValid, status_ok);
	status_tmp = ECTOS::BIT_UTILITIES::PackBool(status_tmp, 4, status.beam1_distanceValid, status_ok);
	status_tmp = ECTOS::BIT_UTILITIES::PackBool(status_tmp, 5, status.beam2_distanceValid, status_ok);
	status_tmp = ECTOS::BIT_UTILITIES::PackBool(status_tmp, 6, status.beam3_distanceValid, status_ok);
	status_tmp = ECTOS::BIT_UTILITIES::PackBool(status_tmp, 7, status.beam4_distanceValid, status_ok);
	status_tmp = ECTOS::BIT_UTILITIES::PackBool(status_tmp, 8, status.beam1_fomValid, status_ok);
	status_tmp = ECTOS::BIT_UTILITIES::PackBool(status_tmp, 9, status.beam2_fomValid, status_ok);
	status_tmp = ECTOS::BIT_UTILITIES::PackBool(status_tmp, 10, status.beam3_fomValid, status_ok);
	status_tmp = ECTOS::BIT_UTILITIES::PackBool(status_tmp, 11, status.beam4_fomValid, status_ok);
	status_tmp = ECTOS::BIT_UTILITIES::PackBool(status_tmp, 12, status.x_velocityValid, status_ok);
	status_tmp = ECTOS::BIT_UTILITIES::PackBool(status_tmp, 13, status.y_velocityValid, status_ok);
	status_tmp = ECTOS::BIT_UTILITIES::PackBool(status_tmp, 14, status.z1_velocityValid, status_ok);
	status_tmp = ECTOS::BIT_UTILITIES::PackBool(status_tmp, 15, status.z2_velocityValid, status_ok);
	status_tmp = ECTOS::BIT_UTILITIES::PackBool(status_tmp, 16, status.x_fomValid, status_ok);
	status_tmp = ECTOS::BIT_UTILITIES::PackBool(status_tmp, 17, status.y_fomValid, status_ok);
	status_tmp = ECTOS::BIT_UTILITIES::PackBool(status_tmp, 18, status.z1_fomValid, status_ok);
	status_tmp = ECTOS::BIT_UTILITIES::PackBool(status_tmp, 19, status.z2_fomValid, status_ok);
	status_tmp = ECTOS::BIT_UTILITIES::Pack(status_tmp, 20, 22, status.cpu_load, status_ok);
	status_tmp = ECTOS::BIT_UTILITIES::Pack(status_tmp, 28, 31, status.wakeup_state, status_ok);
	bb.setOffset(44);	bb.put(status_tmp);

	bb.setOffset(48);	bb.put(speedOfSound);
	bb.setOffset(52);	bb.put(temperature);
	bb.setOffset(56);	bb.put(pressure);
	bb.setOffset(60);	bb.put(velBeam0);
	bb.setOffset(64);	bb.put(velBeam1);
	bb.setOffset(68);	bb.put(velBeam2);
	bb.setOffset(72);	bb.put(velBeam3);
	bb.setOffset(76);	bb.put(distBeam0);
	bb.setOffset(80);	bb.put(distBeam1);
	bb.setOffset(84);	bb.put(distBeam2);
	bb.setOffset(88);	bb.put(distBeam3);
	bb.setOffset(92);	bb.put(fomBeam0);
	bb.setOffset(96);	bb.put(fomBeam1);
	bb.setOffset(100);	bb.put(fomBeam2);
	bb.setOffset(104);	bb.put(fomBeam3);
	bb.setOffset(108);	bb.put(dt1Beam0);
	bb.setOffset(112);	bb.put(dt1Beam1);
	bb.setOffset(116);	bb.put(dt1Beam2);
	bb.setOffset(120);	bb.put(dt1Beam3);
	bb.setOffset(124);	bb.put(dt2Beam0);
	bb.setOffset(128);	bb.put(dt2Beam1);
	bb.setOffset(132);	bb.put(dt2Beam2);
	bb.setOffset(136);	bb.put(dt2Beam3);
	bb.setOffset(140);	bb.put(timeVelEstBeam0);
	bb.setOffset(144);	bb.put(timeVelEstBeam1);
	bb.setOffset(148);	bb.put(timeVelEstBeam2);
	bb.setOffset(152);	bb.put(timeVelEstBeam3);
	bb.setOffset(156);	bb.put(velX);
	bb.setOffset(160);	bb.put(velY);
	bb.setOffset(164);	bb.put(velZ1);
	bb.setOffset(168);	bb.put(velZ2);
	bb.setOffset(172);	bb.put(fomX);
	bb.setOffset(176);	bb.put(fomY);
	bb.setOffset(180);	bb.put(fomZ1);
	bb.setOffset(184);	bb.put(fomZ2);
	bb.setOffset(188);	bb.put(dt1X);
	bb.setOffset(192);	bb.put(dt1Y);
	bb.setOffset(196);	bb.put(dt1Z1);
	bb.setOffset(200);	bb.put(dt1Z2);
	bb.setOffset(204);	bb.put(dt2X);
	bb.setOffset(208);	bb.put(dt2Y);
	bb.setOffset(212);	bb.put(dt2Z1);
	bb.setOffset(216);	bb.put(dt2Z2);
	bb.setOffset(220);	bb.put(timeVelEstX);
	bb.setOffset(224);	bb.put(timeVelEstY);
	bb.setOffset(228);	bb.put(timeVelEstZ1);
	bb.setOffset(232);	bb.put(timeVelEstZ2);
	Checksum = computeChecksum((uint32_t*)buffer, 59);
	bb.setOffset(cIndex);		bb.put(Checksum);
	return status_ok;
}

int Msg_6721::Deserialize(unsigned char * buffer, const unsigned int bufferSize)
{
	bool status_ok = true;
	if (bufferSize < 236) return -2;

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
	bb.setOffset(12);	bb.get(MessageLength_In);
	constCheck |= (MessageLength != MessageLength_In) << numConsts++;
	bb.setOffset(16);	bb.get(systemTov);
	bb.setOffset(24);	bb.get(serialNumber);
	bb.setOffset(28);	bb.get(version);
	bb.setOffset(29);	bb.get(offsetOfData);
	bb.setOffset(30);	bb.get(year);
	bb.setOffset(31);	bb.get(month);
	bb.setOffset(32);	bb.get(day);
	bb.setOffset(33);	bb.get(hour);
	bb.setOffset(34);	bb.get(minute);
	bb.setOffset(35);	bb.get(seconds);
	bb.setOffset(36);	microseconds = bb.get<uint16_t>() * (100);
	bb.setOffset(38);	bb.get(numOfBeams);
	bb.setOffset(40);	bb.get(error);

	uint32_t status_tmp = 0; // temporary variable holding the custom bitfield
	bb.setOffset(44);	bb.get(status_tmp);
	status.beam1_velocityValid = ECTOS::BIT_UTILITIES::UnPackBool(status_tmp, 0,  status_ok);
	status.beam2_velocityValid = ECTOS::BIT_UTILITIES::UnPackBool(status_tmp, 1,  status_ok);
	status.beam3_velocityValid = ECTOS::BIT_UTILITIES::UnPackBool(status_tmp, 2,  status_ok);
	status.beam4_velocityValid = ECTOS::BIT_UTILITIES::UnPackBool(status_tmp, 3,  status_ok);
	status.beam1_distanceValid = ECTOS::BIT_UTILITIES::UnPackBool(status_tmp, 4,  status_ok);
	status.beam2_distanceValid = ECTOS::BIT_UTILITIES::UnPackBool(status_tmp, 5,  status_ok);
	status.beam3_distanceValid = ECTOS::BIT_UTILITIES::UnPackBool(status_tmp, 6,  status_ok);
	status.beam4_distanceValid = ECTOS::BIT_UTILITIES::UnPackBool(status_tmp, 7,  status_ok);
	status.beam1_fomValid = ECTOS::BIT_UTILITIES::UnPackBool(status_tmp, 8,  status_ok);
	status.beam2_fomValid = ECTOS::BIT_UTILITIES::UnPackBool(status_tmp, 9,  status_ok);
	status.beam3_fomValid = ECTOS::BIT_UTILITIES::UnPackBool(status_tmp, 10,  status_ok);
	status.beam4_fomValid = ECTOS::BIT_UTILITIES::UnPackBool(status_tmp, 11,  status_ok);
	status.x_velocityValid = ECTOS::BIT_UTILITIES::UnPackBool(status_tmp, 12,  status_ok);
	status.y_velocityValid = ECTOS::BIT_UTILITIES::UnPackBool(status_tmp, 13,  status_ok);
	status.z1_velocityValid = ECTOS::BIT_UTILITIES::UnPackBool(status_tmp, 14,  status_ok);
	status.z2_velocityValid = ECTOS::BIT_UTILITIES::UnPackBool(status_tmp, 15,  status_ok);
	status.x_fomValid = ECTOS::BIT_UTILITIES::UnPackBool(status_tmp, 16,  status_ok);
	status.y_fomValid = ECTOS::BIT_UTILITIES::UnPackBool(status_tmp, 17,  status_ok);
	status.z1_fomValid = ECTOS::BIT_UTILITIES::UnPackBool(status_tmp, 18,  status_ok);
	status.z2_fomValid = ECTOS::BIT_UTILITIES::UnPackBool(status_tmp, 19,  status_ok);
	status.cpu_load = ECTOS::BIT_UTILITIES::UnPack(status_tmp, 20, 22,  status_ok);
	status.wakeup_state = ECTOS::BIT_UTILITIES::UnPack(status_tmp, 28, 31,  status_ok);
	bb.setOffset(48);	bb.get(speedOfSound);
	bb.setOffset(52);	bb.get(temperature);
	bb.setOffset(56);	bb.get(pressure);
	bb.setOffset(60);	bb.get(velBeam0);
	bb.setOffset(64);	bb.get(velBeam1);
	bb.setOffset(68);	bb.get(velBeam2);
	bb.setOffset(72);	bb.get(velBeam3);
	bb.setOffset(76);	bb.get(distBeam0);
	bb.setOffset(80);	bb.get(distBeam1);
	bb.setOffset(84);	bb.get(distBeam2);
	bb.setOffset(88);	bb.get(distBeam3);
	bb.setOffset(92);	bb.get(fomBeam0);
	bb.setOffset(96);	bb.get(fomBeam1);
	bb.setOffset(100);	bb.get(fomBeam2);
	bb.setOffset(104);	bb.get(fomBeam3);
	bb.setOffset(108);	bb.get(dt1Beam0);
	bb.setOffset(112);	bb.get(dt1Beam1);
	bb.setOffset(116);	bb.get(dt1Beam2);
	bb.setOffset(120);	bb.get(dt1Beam3);
	bb.setOffset(124);	bb.get(dt2Beam0);
	bb.setOffset(128);	bb.get(dt2Beam1);
	bb.setOffset(132);	bb.get(dt2Beam2);
	bb.setOffset(136);	bb.get(dt2Beam3);
	bb.setOffset(140);	bb.get(timeVelEstBeam0);
	bb.setOffset(144);	bb.get(timeVelEstBeam1);
	bb.setOffset(148);	bb.get(timeVelEstBeam2);
	bb.setOffset(152);	bb.get(timeVelEstBeam3);
	bb.setOffset(156);	bb.get(velX);
	bb.setOffset(160);	bb.get(velY);
	bb.setOffset(164);	bb.get(velZ1);
	bb.setOffset(168);	bb.get(velZ2);
	bb.setOffset(172);	bb.get(fomX);
	bb.setOffset(176);	bb.get(fomY);
	bb.setOffset(180);	bb.get(fomZ1);
	bb.setOffset(184);	bb.get(fomZ2);
	bb.setOffset(188);	bb.get(dt1X);
	bb.setOffset(192);	bb.get(dt1Y);
	bb.setOffset(196);	bb.get(dt1Z1);
	bb.setOffset(200);	bb.get(dt1Z2);
	bb.setOffset(204);	bb.get(dt2X);
	bb.setOffset(208);	bb.get(dt2Y);
	bb.setOffset(212);	bb.get(dt2Z1);
	bb.setOffset(216);	bb.get(dt2Z2);
	bb.setOffset(220);	bb.get(timeVelEstX);
	bb.setOffset(224);	bb.get(timeVelEstY);
	bb.setOffset(228);	bb.get(timeVelEstZ1);
	bb.setOffset(232);	bb.get(timeVelEstZ2);
	bb.setOffset(8);	bb.get(Checksum);
	uint32_t computedCRC = computeChecksum((uint32_t*)buffer, 59);
	int retVal = 0;
	retVal = constCheck;
	if(computedCRC != Checksum)
		retVal = -1;
	return retVal;
}

