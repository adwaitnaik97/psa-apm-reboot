#include <include/HGuideAPI.h>
#include <include/Msg_5106.h>

#include <cmath>
#include <cstring>

#include <include/utils/bit_utilities.h>
#include <include/utils/byte_buffer.h>
#include <include/utils/ectos_constants.h>
#include <include/utils/Checksum.h>


// Definition for Static constants
const uint32_t Msg_5106::AddressId;
const uint32_t Msg_5106::MessageId;
const uint32_t Msg_5106::MessageLength;

Msg_5106::Msg_5106()
{
	Default();
}

void Msg_5106::Default()
{
	Checksum = 0;
	systemTov = 0;
	gpsTov = 0;
	GPSMode = static_cast<gps_mode_table_t>(0);
	ephem_type = 0;
	sat_type = 0;
	satellite_ID = 0;
	Week = 0;
	URA = 0;
	health1 = 0;
	tgd1 = 0;
	tgd2 = 0;
	AODC = 0;
	tgdb2cp = 0;
	tgdb2bi = 0;
	IODC = 0;
	toc = 0;
	a0 = 0;
	a1 = 0;
	a2 = 0;
	AODE = 0;
	toe = 0;
	RootA = 0;
	ecc = 0;
	w = 0;
	dN = 0;
	M0 = 0;
	omega0 = 0;
	omegai = 0;
	i0 = 0;
	IDOT = 0;
	cuc = 0;
	cus = 0;
	crc = 0;
	crs = 0;
	cic = 0;
	cis = 0;
	tgd3 = 0;
	IscB1cd = 0;
	IscB2ad = 0;
	SISMAI = 0;
	deltaA = 0;
	Adot = 0;
	deltaN0Dot = 0;
}

bool Msg_5106::Serialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 292) return false;

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
	bb.setOffset(36);	bb.put(ephem_type);
	bb.setOffset(37);	bb.put(sat_type);
	bb.setOffset(40);	bb.put(satellite_ID);
	bb.setOffset(44);	bb.put(Week);
	bb.setOffset(48);	bb.put(URA);
	bb.setOffset(56);	bb.put(health1);
	bb.setOffset(80);	bb.put(toc);
	bb.setOffset(84);	bb.put(a0);
	bb.setOffset(92);	bb.put(a1);
	bb.setOffset(100);	bb.put(a2);
	bb.setOffset(108);	bb.put(AODE);
	bb.setOffset(112);	bb.put(toe);
	bb.setOffset(116);	bb.put(RootA);
	bb.setOffset(124);	bb.put(ecc);
	bb.setOffset(132);	bb.put(w);
	bb.setOffset(140);	bb.put(dN);
	bb.setOffset(148);	bb.put(M0);
	bb.setOffset(156);	bb.put(omega0);
	bb.setOffset(164);	bb.put(omegai);
	bb.setOffset(172);	bb.put(i0);
	bb.setOffset(180);	bb.put(IDOT);
	bb.setOffset(188);	bb.put(cuc);
	bb.setOffset(196);	bb.put(cus);
	bb.setOffset(204);	bb.put(crc);
	bb.setOffset(212);	bb.put(crs);
	bb.setOffset(220);	bb.put(cic);
	bb.setOffset(228);	bb.put(cis);

	if ( ephem_type == 0 ) {
			bb.setOffset(60);	bb.put(tgd1);
			bb.setOffset(68);	bb.put(tgd2);
			bb.setOffset(76);	bb.put(AODC);
		}

	if ( ephem_type == 1 ) {
			bb.setOffset(60);	bb.put(tgdb2cp);
			bb.setOffset(68);	bb.put(tgdb2bi);
			bb.setOffset(76);	bb.put(IODC);
			bb.setOffset(236);	bb.put(tgd3);
			bb.setOffset(244);	bb.put(IscB1cd);
			bb.setOffset(252);	bb.put(IscB2ad);
			bb.setOffset(260);	bb.put(SISMAI);
			bb.setOffset(264);	bb.put(deltaA);
			bb.setOffset(272);	bb.put(Adot);
			bb.setOffset(280);	bb.put(deltaN0Dot);
		}
	Checksum = computeChecksum((uint32_t*)buffer, 73);
	bb.setOffset(cIndex);		bb.put(Checksum);
	return status_ok;
}

int Msg_5106::Deserialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 292) return -2;

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
	bb.setOffset(36);	bb.get(ephem_type);
	bb.setOffset(37);	bb.get(sat_type);
	bb.setOffset(40);	bb.get(satellite_ID);
	bb.setOffset(44);	bb.get(Week);
	bb.setOffset(48);	bb.get(URA);
	bb.setOffset(56);	bb.get(health1);
	bb.setOffset(80);	bb.get(toc);
	bb.setOffset(84);	bb.get(a0);
	bb.setOffset(92);	bb.get(a1);
	bb.setOffset(100);	bb.get(a2);
	bb.setOffset(108);	bb.get(AODE);
	bb.setOffset(112);	bb.get(toe);
	bb.setOffset(116);	bb.get(RootA);
	bb.setOffset(124);	bb.get(ecc);
	bb.setOffset(132);	bb.get(w);
	bb.setOffset(140);	bb.get(dN);
	bb.setOffset(148);	bb.get(M0);
	bb.setOffset(156);	bb.get(omega0);
	bb.setOffset(164);	bb.get(omegai);
	bb.setOffset(172);	bb.get(i0);
	bb.setOffset(180);	bb.get(IDOT);
	bb.setOffset(188);	bb.get(cuc);
	bb.setOffset(196);	bb.get(cus);
	bb.setOffset(204);	bb.get(crc);
	bb.setOffset(212);	bb.get(crs);
	bb.setOffset(220);	bb.get(cic);
	bb.setOffset(228);	bb.get(cis);

	if ( ephem_type == 0 ) {
		 	bb.setOffset(60);	bb.get(tgd1);
		 	bb.setOffset(68);	bb.get(tgd2);
		 	bb.setOffset(76);	bb.get(AODC);
}

	if ( ephem_type == 1 ) {
		 	bb.setOffset(60);	bb.get(tgdb2cp);
		 	bb.setOffset(68);	bb.get(tgdb2bi);
		 	bb.setOffset(76);	bb.get(IODC);
		 	bb.setOffset(236);	bb.get(tgd3);
		 	bb.setOffset(244);	bb.get(IscB1cd);
		 	bb.setOffset(252);	bb.get(IscB2ad);
		 	bb.setOffset(260);	bb.get(SISMAI);
		 	bb.setOffset(264);	bb.get(deltaA);
		 	bb.setOffset(272);	bb.get(Adot);
		 	bb.setOffset(280);	bb.get(deltaN0Dot);
}
	bb.setOffset(12);	bb.get(Checksum);
	uint32_t computedCRC = computeChecksum((uint32_t*)buffer, 73);
	int retVal = 0;
	retVal = constCheck;
	if(computedCRC != Checksum)
		retVal = -1;
	return retVal;
}

