#ifndef __HGuideAPI_Msg_6601_h__
#define __HGuideAPI_Msg_6601_h__
#pragma once

#include <cstdint>

// The Blue, Green, Red are components of the RGB value
struct led_bitfield_t
{
	uint8_t blue_byte; // Blue Byte
	uint8_t green_byte; // Green Byte
	uint8_t red_byte; // Red Byte

	void Default()
	{
		blue_byte = 0;
		green_byte = 0;
		red_byte = 0;
	}
};


// 0x6601 : Virtual Status LED message
//
// Message used to report the physical LED status as well as simulate LEDs on devices without LEDs
// blue/green/red => RGB value
// 
class HGUIDE_DLL Msg_6601
{
public:
	Msg_6601();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 32;}

public:
	static const uint32_t AddressId = 0xA5C381FF; // Start of Message
	static const uint32_t MessageId = 0x6601; // Message ID
	static const uint32_t MessageLength = 8; // Message Length
	uint32_t Checksum; // Checksum
	led_bitfield_t PWR_LED_State; // Power LED State
	led_bitfield_t POS_LED_State; // Position Status LED State
	led_bitfield_t SYS_LED_State; // System Status LED State
	led_bitfield_t LOG_LED_State; // Data logging Status LED State
};

#endif // __HGuideAPI_Msg_6601_h__
