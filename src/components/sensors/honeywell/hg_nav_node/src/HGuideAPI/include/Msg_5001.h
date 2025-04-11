#ifndef __HGuideAPI_Msg_5001_h__
#define __HGuideAPI_Msg_5001_h__
#pragma once

#include <cstdint>


// 0x5001 : GNSS Receiver Version Information
//
// Information GNSS Receiver and its various components and versions
// 
class HGUIDE_DLL Msg_5001
{
public:
	Msg_5001();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 256;}

public:
	static const uint32_t AddressId = 0xA5C381FF; // Start of Message
	static const uint32_t MessageId = 0x5001; // Message ID
	static const uint32_t MessageLength = 64; // Message Length
	uint32_t Checksum; // Checksum
	uint8_t SerialNumber[16]; // GNSS Receiver Serial Number
	uint8_t PartNumber[16]; // GNSS Receiver Part Number
	uint8_t Model[16]; // GNSS Receiver Model (Indicates Functionality)
	uint8_t GNSSHardwareVersion[16]; // GNSS Receiver Hardware Version
	uint8_t FirmwareVersion[16]; // GNSS Receiver Firmware Version
	uint8_t FirmwareBuildDate[16]; // GNSS Receiver Firmware Build Date
	uint8_t FirmwareBuildTime[16]; // GNSS Receiver Firmware Build Time of Day
	uint8_t Component1Type[16]; // GNSS Receiver Component Type
	uint8_t Component1Version[16]; // GNSS Receiver Component Version
	uint8_t Component2Type[16]; // GNSS Receiver Component Type
	uint8_t Component2Version[16]; // GNSS Receiver Component Version
	uint8_t Component3Type[16]; // GNSS Receiver Component Type
	uint8_t Component3Version[16]; // GNSS Receiver Component Version
};

#endif // __HGuideAPI_Msg_5001_h__
