#include <include/HGuideAPI.h>
#include <include/Msg_2001.h>

#include <cmath>
#include <cstring>

#include <include/utils/bit_utilities.h>
#include <include/utils/byte_buffer.h>
#include <include/utils/ectos_constants.h>
#include <include/utils/Checksum.h>


// Definition for Static constants
const uint32_t Msg_2001::AddressId;
const uint32_t Msg_2001::MessageId;
const uint32_t Msg_2001::MessageLength;

Msg_2001::Msg_2001()
{
	Default();
}

void Msg_2001::Default()
{
	Checksum = 0;
	for (unsigned int index = 0; index < 8; index++)
	{
		IMUSerialNumber[index] = 0;
	}
	for (unsigned int index = 0; index < 8; index++)
	{
		ISA_Performance_Grade[index] = 0;
	}
	for (unsigned int index = 0; index < 16; index++)
	{
		IMUSoftwareVersion[index] = 0;
	}
	for (unsigned int index = 0; index < 16; index++)
	{
		FPGA_VERSION[index] = 0;
	}
	for (unsigned int index = 0; index < 16; index++)
	{
		FSBL_VERSION[index] = 0;
	}
	for (unsigned int index = 0; index < 16; index++)
	{
		SSBL_VERSION[index] = 0;
	}
	for (unsigned int index = 0; index < 16; index++)
	{
		HGuideSoftwareVersion[index] = 0;
	}
	for (unsigned int index = 0; index < 16; index++)
	{
		HGuideSoftwareVersionBuildDate[index] = 0;
	}
	for (unsigned int index = 0; index < 16; index++)
	{
		HGuideSoftwareVersionBuildTime[index] = 0;
	}
	for (unsigned int index = 0; index < 4; index++)
	{
		INS_CONFIG_FILEID[index] = 0;
	}
	for (unsigned int index = 0; index < 4; index++)
	{
		SSBL_REGINIT_FILEID[index] = 0;
	}
	for (unsigned int index = 0; index < 4; index++)
	{
		SSBL_FLASHLDR_FILEID[index] = 0;
	}
	for (unsigned int index = 0; index < 10; index++)
	{
		HGuideSerialNumber[index] = 0;
	}
	for (unsigned int index = 0; index < 10; index++)
	{
		HGuidePartNumber[index] = 0;
	}
	device_configuration = 0;
	CaseToNavLeverArmX = 0;
	CaseToNavLeverArmY = 0;
	CaseToNavLeverArmZ = 0;
	CaseToNavQuaternionI = 0;
	CaseToNavQuaternionJ = 0;
	CaseToNavQuaternionK = 0;
	VehicleLeverArmX = 0;
	VehicleLeverArmY = 0;
	VehicleLeverArmZ = 0;
	VehicleQuaterionI = 0;
	VehicleQuaterionJ = 0;
	VehicleQuaterionK = 0;
	RF1AntennaLeverArmRSS = 0;
	RF1AntennaLeverArmX = 0;
	RF1AntennaLeverArmY = 0;
	RF1AntennaLeverArmZ = 0;
	HGNSI_AntennaLeverArmX = 0;
	HGNSI_AntennaLeverArmY = 0;
	HGNSI_AntennaLeverArmZ = 0;
	RF2AntennaLeverArmX = 0;
	RF2AntennaLeverArmY = 0;
	RF2AntennaLeverArmZ = 0;
	GPSantToCaseQuaternionI = 0;
	GPSantToCaseQuaternionJ = 0;
	GPSantToCaseQuaternionK = 0;
	CaseToNavQuaternionS = 1.0;
	VehicleQuaterionS = 1.0;
	GPSantToCaseQuaternionS = 1.0;
}

bool Msg_2001::Serialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 320) return false;

	std::memset(buffer, 0, bufferSize);

	bool status_ok = true;
	ECTOS::BYTE_BUFFER::ByteOutputBuffer<unsigned int, unsigned char, ECTOS::BYTE_BUFFER::HGuideAPI_endianness> bb(buffer, bufferSize);

	bb.setOffset(0);	bb.put(AddressId);
	bb.setOffset(4);	bb.put(MessageId);
	bb.setOffset(8);	bb.put(MessageLength);
	int cIndex = 12;
	Checksum = 0;
	bb.setOffset(16);
	for (unsigned int index = 0; index < 8; index++)
	{
		bb.put(IMUSerialNumber[index]);
	}
	bb.setOffset(24);
	for (unsigned int index = 0; index < 8; index++)
	{
		bb.put(ISA_Performance_Grade[index]);
	}
	bb.setOffset(32);
	for (unsigned int index = 0; index < 16; index++)
	{
		bb.put(IMUSoftwareVersion[index]);
	}
	bb.setOffset(48);
	for (unsigned int index = 0; index < 16; index++)
	{
		bb.put(FPGA_VERSION[index]);
	}
	bb.setOffset(64);
	for (unsigned int index = 0; index < 16; index++)
	{
		bb.put(FSBL_VERSION[index]);
	}
	bb.setOffset(80);
	for (unsigned int index = 0; index < 16; index++)
	{
		bb.put(SSBL_VERSION[index]);
	}
	bb.setOffset(96);
	for (unsigned int index = 0; index < 16; index++)
	{
		bb.put(HGuideSoftwareVersion[index]);
	}
	bb.setOffset(112);
	for (unsigned int index = 0; index < 16; index++)
	{
		bb.put(HGuideSoftwareVersionBuildDate[index]);
	}
	bb.setOffset(128);
	for (unsigned int index = 0; index < 16; index++)
	{
		bb.put(HGuideSoftwareVersionBuildTime[index]);
	}
	bb.setOffset(144);
	for (unsigned int index = 0; index < 4; index++)
	{
		bb.put(INS_CONFIG_FILEID[index]);
	}
	bb.setOffset(148);
	for (unsigned int index = 0; index < 4; index++)
	{
		bb.put(SSBL_REGINIT_FILEID[index]);
	}
	bb.setOffset(152);
	for (unsigned int index = 0; index < 4; index++)
	{
		bb.put(SSBL_FLASHLDR_FILEID[index]);
	}
	bb.setOffset(156);
	for (unsigned int index = 0; index < 10; index++)
	{
		bb.put(HGuideSerialNumber[index]);
	}
	bb.setOffset(166);
	for (unsigned int index = 0; index < 10; index++)
	{
		bb.put(HGuidePartNumber[index]);
	}
	bb.setOffset(176);	bb.put(device_configuration);
	bb.setOffset(180);	bb.put(CaseToNavLeverArmX);
	bb.setOffset(184);	bb.put(CaseToNavLeverArmY);
	bb.setOffset(188);	bb.put(CaseToNavLeverArmZ);
	bb.setOffset(192);	bb.put(CaseToNavQuaternionS);
	bb.setOffset(196);	bb.put(CaseToNavQuaternionI);
	bb.setOffset(200);	bb.put(CaseToNavQuaternionJ);
	bb.setOffset(204);	bb.put(CaseToNavQuaternionK);
	bb.setOffset(208);	bb.put(VehicleLeverArmX);
	bb.setOffset(212);	bb.put(VehicleLeverArmY);
	bb.setOffset(216);	bb.put(VehicleLeverArmZ);
	bb.setOffset(220);	bb.put(VehicleQuaterionS);
	bb.setOffset(224);	bb.put(VehicleQuaterionI);
	bb.setOffset(228);	bb.put(VehicleQuaterionJ);
	bb.setOffset(232);	bb.put(VehicleQuaterionK);
	bb.setOffset(236);	bb.put(RF1AntennaLeverArmRSS);
	bb.setOffset(240);	bb.put(RF1AntennaLeverArmX);
	bb.setOffset(244);	bb.put(RF1AntennaLeverArmY);
	bb.setOffset(248);	bb.put(RF1AntennaLeverArmZ);
	bb.setOffset(252);	bb.put(HGNSI_AntennaLeverArmX);
	bb.setOffset(256);	bb.put(HGNSI_AntennaLeverArmY);
	bb.setOffset(260);	bb.put(HGNSI_AntennaLeverArmZ);
	bb.setOffset(292);	bb.put(RF2AntennaLeverArmX);
	bb.setOffset(296);	bb.put(RF2AntennaLeverArmY);
	bb.setOffset(300);	bb.put(RF2AntennaLeverArmZ);
	bb.setOffset(304);	bb.put(GPSantToCaseQuaternionS);
	bb.setOffset(308);	bb.put(GPSantToCaseQuaternionI);
	bb.setOffset(312);	bb.put(GPSantToCaseQuaternionJ);
	bb.setOffset(316);	bb.put(GPSantToCaseQuaternionK);
	Checksum = computeChecksum((uint32_t*)buffer, 80);
	bb.setOffset(cIndex);		bb.put(Checksum);
	return status_ok;
}

int Msg_2001::Deserialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 320) return -2;

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
	bb.setOffset(16);
	for (unsigned int index = 0; index < 8; index++)
	{
		bb.get(IMUSerialNumber[index]);
	}
	bb.setOffset(24);
	for (unsigned int index = 0; index < 8; index++)
	{
		bb.get(ISA_Performance_Grade[index]);
	}
	bb.setOffset(32);
	for (unsigned int index = 0; index < 16; index++)
	{
		bb.get(IMUSoftwareVersion[index]);
	}
	bb.setOffset(48);
	for (unsigned int index = 0; index < 16; index++)
	{
		bb.get(FPGA_VERSION[index]);
	}
	bb.setOffset(64);
	for (unsigned int index = 0; index < 16; index++)
	{
		bb.get(FSBL_VERSION[index]);
	}
	bb.setOffset(80);
	for (unsigned int index = 0; index < 16; index++)
	{
		bb.get(SSBL_VERSION[index]);
	}
	bb.setOffset(96);
	for (unsigned int index = 0; index < 16; index++)
	{
		bb.get(HGuideSoftwareVersion[index]);
	}
	bb.setOffset(112);
	for (unsigned int index = 0; index < 16; index++)
	{
		bb.get(HGuideSoftwareVersionBuildDate[index]);
	}
	bb.setOffset(128);
	for (unsigned int index = 0; index < 16; index++)
	{
		bb.get(HGuideSoftwareVersionBuildTime[index]);
	}
	bb.setOffset(144);
	for (unsigned int index = 0; index < 4; index++)
	{
		bb.get(INS_CONFIG_FILEID[index]);
	}
	bb.setOffset(148);
	for (unsigned int index = 0; index < 4; index++)
	{
		bb.get(SSBL_REGINIT_FILEID[index]);
	}
	bb.setOffset(152);
	for (unsigned int index = 0; index < 4; index++)
	{
		bb.get(SSBL_FLASHLDR_FILEID[index]);
	}
	bb.setOffset(156);
	for (unsigned int index = 0; index < 10; index++)
	{
		bb.get(HGuideSerialNumber[index]);
	}
	bb.setOffset(166);
	for (unsigned int index = 0; index < 10; index++)
	{
		bb.get(HGuidePartNumber[index]);
	}
	bb.setOffset(176);	bb.get(device_configuration);
	bb.setOffset(180);	bb.get(CaseToNavLeverArmX);
	bb.setOffset(184);	bb.get(CaseToNavLeverArmY);
	bb.setOffset(188);	bb.get(CaseToNavLeverArmZ);
	bb.setOffset(192);	bb.get(CaseToNavQuaternionS);
	bb.setOffset(196);	bb.get(CaseToNavQuaternionI);
	bb.setOffset(200);	bb.get(CaseToNavQuaternionJ);
	bb.setOffset(204);	bb.get(CaseToNavQuaternionK);
	bb.setOffset(208);	bb.get(VehicleLeverArmX);
	bb.setOffset(212);	bb.get(VehicleLeverArmY);
	bb.setOffset(216);	bb.get(VehicleLeverArmZ);
	bb.setOffset(220);	bb.get(VehicleQuaterionS);
	bb.setOffset(224);	bb.get(VehicleQuaterionI);
	bb.setOffset(228);	bb.get(VehicleQuaterionJ);
	bb.setOffset(232);	bb.get(VehicleQuaterionK);
	bb.setOffset(236);	bb.get(RF1AntennaLeverArmRSS);
	bb.setOffset(240);	bb.get(RF1AntennaLeverArmX);
	bb.setOffset(244);	bb.get(RF1AntennaLeverArmY);
	bb.setOffset(248);	bb.get(RF1AntennaLeverArmZ);
	bb.setOffset(252);	bb.get(HGNSI_AntennaLeverArmX);
	bb.setOffset(256);	bb.get(HGNSI_AntennaLeverArmY);
	bb.setOffset(260);	bb.get(HGNSI_AntennaLeverArmZ);
	bb.setOffset(292);	bb.get(RF2AntennaLeverArmX);
	bb.setOffset(296);	bb.get(RF2AntennaLeverArmY);
	bb.setOffset(300);	bb.get(RF2AntennaLeverArmZ);
	bb.setOffset(304);	bb.get(GPSantToCaseQuaternionS);
	bb.setOffset(308);	bb.get(GPSantToCaseQuaternionI);
	bb.setOffset(312);	bb.get(GPSantToCaseQuaternionJ);
	bb.setOffset(316);	bb.get(GPSantToCaseQuaternionK);
	bb.setOffset(12);	bb.get(Checksum);
	uint32_t computedCRC = computeChecksum((uint32_t*)buffer, 80);
	int retVal = 0;
	retVal = constCheck;
	if(computedCRC != Checksum)
		retVal = -1;
	return retVal;
}

