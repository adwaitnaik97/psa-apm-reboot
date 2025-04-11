#include <include/HGuideAPI.h>
#include <include/Msg_5105.h>

#include <cmath>
#include <cstring>

#include <include/utils/bit_utilities.h>
#include <include/utils/byte_buffer.h>
#include <include/utils/ectos_constants.h>
#include <include/utils/Checksum.h>


// Definition for Static constants
const uint32_t Msg_5105::AddressId;
const uint32_t Msg_5105::MessageId;
const uint32_t Msg_5105::MessageLength;

Msg_5105::Msg_5105()
{
	Default();
}

void Msg_5105::Default()
{
	Checksum = 0;
	systemTov = 0;
	gpsTov = 0;
	GPSMode = static_cast<gps_mode_table_t>(0);
	ephem_type = 0;
	satellite_ID = 0;
	E5bHealth = 0;
	E5bDVS = 0;
	E1bHealth = 0;
	E1bDVS = 0;
	E5aHealth = 0;
	E5aDVS = 0;
	IODnav = 0;
	SISA_Index = 0;
	INAV_Source = 0;
	T0e = 0;
	T0c = 0;
	M0 = 0;
	DeltaN = 0;
	Ecc = 0;
	RootA = 0;
	I0 = 0;
	IDot = 0;
	Omega0 = 0;
	Omega = 0;
	OmegaDot = 0;
	Cuc = 0;
	Cus = 0;
	Crc = 0;
	Crs = 0;
	Cic = 0;
	Cis = 0;
	Af0 = 0;
	Af1 = 0;
	Af2 = 0;
	E1E5aBGD = 0;
	E1E5bBGD = 0;
	galileo_week = 0;
	tow = 0;
}

bool Msg_5105::Serialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 232) return false;

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
	bb.setOffset(38);	bb.put(satellite_ID);
	bb.setOffset(44);	bb.put(IODnav);
	bb.setOffset(46);	bb.put(SISA_Index);
	bb.setOffset(47);	bb.put(INAV_Source);
	bb.setOffset(48);	bb.put(T0e);
	bb.setOffset(52);	bb.put(T0c);
	bb.setOffset(56);	bb.put(M0);
	bb.setOffset(64);	bb.put(DeltaN);
	bb.setOffset(72);	bb.put(Ecc);
	bb.setOffset(80);	bb.put(RootA);
	bb.setOffset(88);	bb.put(I0);
	bb.setOffset(96);	bb.put(IDot);
	bb.setOffset(104);	bb.put(Omega0);
	bb.setOffset(112);	bb.put(Omega);
	bb.setOffset(120);	bb.put(OmegaDot);
	bb.setOffset(128);	bb.put(Cuc);
	bb.setOffset(136);	bb.put(Cus);
	bb.setOffset(144);	bb.put(Crc);
	bb.setOffset(152);	bb.put(Crs);
	bb.setOffset(160);	bb.put(Cic);
	bb.setOffset(168);	bb.put(Cis);
	bb.setOffset(176);	bb.put(Af0);
	bb.setOffset(184);	bb.put(Af1);
	bb.setOffset(192);	bb.put(Af2);
	bb.setOffset(200);	bb.put(E1E5aBGD);
	bb.setOffset(216);	bb.put(galileo_week);
	bb.setOffset(220);	bb.put(tow);

	if ( ephem_type == 0 ) {
			bb.setOffset(40);	bb.put(E5bHealth);
			bb.setOffset(41);	bb.put(E5bDVS);
			bb.setOffset(42);	bb.put(E1bHealth);
			bb.setOffset(43);	bb.put(E1bDVS);
			bb.setOffset(208);	bb.put(E1E5bBGD);
		}

	if ( ephem_type == 1 ) {
			bb.setOffset(40);	bb.put(E5aHealth);
			bb.setOffset(41);	bb.put(E5aDVS);
		}
	Checksum = computeChecksum((uint32_t*)buffer, 58);
	bb.setOffset(cIndex);		bb.put(Checksum);
	return status_ok;
}

int Msg_5105::Deserialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 232) return -2;

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
	bb.setOffset(38);	bb.get(satellite_ID);
	bb.setOffset(44);	bb.get(IODnav);
	bb.setOffset(46);	bb.get(SISA_Index);
	bb.setOffset(47);	bb.get(INAV_Source);
	bb.setOffset(48);	bb.get(T0e);
	bb.setOffset(52);	bb.get(T0c);
	bb.setOffset(56);	bb.get(M0);
	bb.setOffset(64);	bb.get(DeltaN);
	bb.setOffset(72);	bb.get(Ecc);
	bb.setOffset(80);	bb.get(RootA);
	bb.setOffset(88);	bb.get(I0);
	bb.setOffset(96);	bb.get(IDot);
	bb.setOffset(104);	bb.get(Omega0);
	bb.setOffset(112);	bb.get(Omega);
	bb.setOffset(120);	bb.get(OmegaDot);
	bb.setOffset(128);	bb.get(Cuc);
	bb.setOffset(136);	bb.get(Cus);
	bb.setOffset(144);	bb.get(Crc);
	bb.setOffset(152);	bb.get(Crs);
	bb.setOffset(160);	bb.get(Cic);
	bb.setOffset(168);	bb.get(Cis);
	bb.setOffset(176);	bb.get(Af0);
	bb.setOffset(184);	bb.get(Af1);
	bb.setOffset(192);	bb.get(Af2);
	bb.setOffset(200);	bb.get(E1E5aBGD);
	bb.setOffset(216);	bb.get(galileo_week);
	bb.setOffset(220);	bb.get(tow);

	if ( ephem_type == 0 ) {
		 	bb.setOffset(40);	bb.get(E5bHealth);
		 	bb.setOffset(41);	bb.get(E5bDVS);
		 	bb.setOffset(42);	bb.get(E1bHealth);
		 	bb.setOffset(43);	bb.get(E1bDVS);
		 	bb.setOffset(208);	bb.get(E1E5bBGD);
}

	if ( ephem_type == 1 ) {
		 	bb.setOffset(40);	bb.get(E5aHealth);
		 	bb.setOffset(41);	bb.get(E5aDVS);
}
	bb.setOffset(12);	bb.get(Checksum);
	uint32_t computedCRC = computeChecksum((uint32_t*)buffer, 58);
	int retVal = 0;
	retVal = constCheck;
	if(computedCRC != Checksum)
		retVal = -1;
	return retVal;
}

