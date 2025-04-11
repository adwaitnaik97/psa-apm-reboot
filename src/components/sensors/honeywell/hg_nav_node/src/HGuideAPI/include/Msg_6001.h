#ifndef __HGuideAPI_Msg_6001_h__
#define __HGuideAPI_Msg_6001_h__
#pragma once

#include <cstdint>


// 0x6001 : INS Configuration
//
// Information about various device serial numbers and sw versions
// Use the 0x1001 message to leave this on at 0.1 Hz (enabled; default), or to have this only produced and sent once (disabled).
// 
class HGUIDE_DLL Msg_6001
{
public:
	Msg_6001();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 256;}

public:
	static const uint32_t AddressId = 0xA5C381FF; // Start of Message
	static const uint32_t MessageId = 0x6001; // Message ID
	static const uint32_t MessageLength = 64; // Message Length
	uint32_t Checksum; // Checksum
	uint8_t DeviceSerialNumber[16]; // Device Serial Number
	uint8_t DevicePartNumber[16]; // Device Part Number (End-Item)
	uint8_t SensorAssyPartNumber[16]; // "Sensor Assembly" Part Number
	uint8_t NavSoftwareVersion[16]; // Navigation Software (ECTOS) Version
	uint8_t NavSoftwareBuildDate[16]; // Navigation Software Build Date
	uint8_t IMUSerialNumber[16]; // IMU Serial Number
	uint8_t IMUPartNumber[16]; // IMU Part Number
	uint8_t IMUSoftwareVersion[16]; // IMU Software Version
	uint8_t GNSSReceiverSerialNumber[16]; // GNSS Receiver Serial Number
	uint8_t GNSSReceiverPartNumber[16]; // GNSS Receiver Part Number
	uint8_t GNSSReceiverFirmwareVersion[16]; // GNSS Receiver Firmware Version
	uint8_t ProcessorHWIdentifier[32]; // Processor/Rigid Flex PN
	uint8_t InterconnectHWIdentifier[32]; // Interconnect Flex PN
};

#endif // __HGuideAPI_Msg_6001_h__
