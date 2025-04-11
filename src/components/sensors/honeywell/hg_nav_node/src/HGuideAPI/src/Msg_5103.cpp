#include <include/HGuideAPI.h>
#include <include/Msg_5103.h>

#include <cmath>
#include <cstring>

#include <include/utils/bit_utilities.h>
#include <include/utils/byte_buffer.h>
#include <include/utils/ectos_constants.h>
#include <include/utils/Checksum.h>


// Definition for Static constants
const uint32_t Msg_5103::AddressId;
const uint32_t Msg_5103::MessageId;
const uint32_t Msg_5103::MessageLength;

Msg_5103::Msg_5103()
{
	Default();
}

void Msg_5103::Default()
{
	Checksum = 0;
	systemTov = 0;
	gpsTov = 0;
	GPSMode = static_cast<gps_mode_table_t>(0);
	prn_number = 0;
	tow = 0;
	health = 0;
	IODE1 = 0;
	IODE2 = 0;
	week = 0;
	z_week = 0;
	toe = 0;
	A = 0;
	dN = 0;
	M0 = 0;
	ecc = 0;
	w = 0;
	cuc = 0;
	cus = 0;
	crc = 0;
	crs = 0;
	cic = 0;
	cis = 0;
	Inclination_angle = 0;
	Inclination_rate = 0;
	wo = 0;
	wi = 0;
	iodc = 0;
	toc = 0;
	tgd = 0;
	af0 = 0;
	af1 = 0;
	af2 = 0;
	AS = 0;
	N = 0;
	URA = 0;
}

bool Msg_5103::Serialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 264) return false;

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
	bb.setOffset(36);	bb.put(prn_number);
	bb.setOffset(40);	bb.put(tow);
	bb.setOffset(48);	bb.put(health);
	bb.setOffset(52);	bb.put(IODE1);
	bb.setOffset(56);	bb.put(IODE2);
	bb.setOffset(60);	bb.put(week);
	bb.setOffset(64);	bb.put(z_week);
	bb.setOffset(68);	bb.put(toe);
	bb.setOffset(76);	bb.put(A);
	bb.setOffset(84);	bb.put(dN);
	bb.setOffset(92);	bb.put(M0);
	bb.setOffset(100);	bb.put(ecc);
	bb.setOffset(108);	bb.put(w);
	bb.setOffset(116);	bb.put(cuc);
	bb.setOffset(124);	bb.put(cus);
	bb.setOffset(132);	bb.put(crc);
	bb.setOffset(140);	bb.put(crs);
	bb.setOffset(148);	bb.put(cic);
	bb.setOffset(156);	bb.put(cis);
	bb.setOffset(164);	bb.put(Inclination_angle);
	bb.setOffset(172);	bb.put(Inclination_rate);
	bb.setOffset(180);	bb.put(wo);
	bb.setOffset(188);	bb.put(wi);
	bb.setOffset(196);	bb.put(iodc);
	bb.setOffset(200);	bb.put(toc);
	bb.setOffset(208);	bb.put(tgd);
	bb.setOffset(216);	bb.put(af0);
	bb.setOffset(224);	bb.put(af1);
	bb.setOffset(232);	bb.put(af2);
	bb.setOffset(240);	bb.put(AS);
	bb.setOffset(244);	bb.put(N);
	bb.setOffset(252);	bb.put(URA);
	Checksum = computeChecksum((uint32_t*)buffer, 66);
	bb.setOffset(cIndex);		bb.put(Checksum);
	return status_ok;
}

int Msg_5103::Deserialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 264) return -2;

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
	bb.setOffset(36);	bb.get(prn_number);
	bb.setOffset(40);	bb.get(tow);
	bb.setOffset(48);	bb.get(health);
	bb.setOffset(52);	bb.get(IODE1);
	bb.setOffset(56);	bb.get(IODE2);
	bb.setOffset(60);	bb.get(week);
	bb.setOffset(64);	bb.get(z_week);
	bb.setOffset(68);	bb.get(toe);
	bb.setOffset(76);	bb.get(A);
	bb.setOffset(84);	bb.get(dN);
	bb.setOffset(92);	bb.get(M0);
	bb.setOffset(100);	bb.get(ecc);
	bb.setOffset(108);	bb.get(w);
	bb.setOffset(116);	bb.get(cuc);
	bb.setOffset(124);	bb.get(cus);
	bb.setOffset(132);	bb.get(crc);
	bb.setOffset(140);	bb.get(crs);
	bb.setOffset(148);	bb.get(cic);
	bb.setOffset(156);	bb.get(cis);
	bb.setOffset(164);	bb.get(Inclination_angle);
	bb.setOffset(172);	bb.get(Inclination_rate);
	bb.setOffset(180);	bb.get(wo);
	bb.setOffset(188);	bb.get(wi);
	bb.setOffset(196);	bb.get(iodc);
	bb.setOffset(200);	bb.get(toc);
	bb.setOffset(208);	bb.get(tgd);
	bb.setOffset(216);	bb.get(af0);
	bb.setOffset(224);	bb.get(af1);
	bb.setOffset(232);	bb.get(af2);
	bb.setOffset(240);	bb.get(AS);
	bb.setOffset(244);	bb.get(N);
	bb.setOffset(252);	bb.get(URA);
	bb.setOffset(12);	bb.get(Checksum);
	uint32_t computedCRC = computeChecksum((uint32_t*)buffer, 66);
	int retVal = 0;
	retVal = constCheck;
	if(computedCRC != Checksum)
		retVal = -1;
	return retVal;
}

