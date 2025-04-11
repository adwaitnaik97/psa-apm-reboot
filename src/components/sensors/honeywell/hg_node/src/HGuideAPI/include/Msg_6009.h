#ifndef __HGuideAPI_Msg_6009_h__
#define __HGuideAPI_Msg_6009_h__
#pragma once

#include <cstdint>


// 0x6009 : Ntrip Configuration
//
// Internal Networked Transport of RTCM via Internet Protocol (NTRIP) configuration output.
// NTRIP client refers to the sw running inside the HGuide Unit. Caster is the remote server providing RTCM corrections
// 
class HGUIDE_DLL Msg_6009
{
public:
	Msg_6009();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 480;}

public:
	static const uint32_t AddressId = 0xA5C381FF; // Start of Message
	static const uint32_t MessageId = 0x6009; // Message ID
	static const uint32_t MessageLength = 120; // Message Length
	uint32_t Checksum; // Checksum
	uint8_t casterHostName[128]; // hostname or IP address of caster
	uint32_t casterIP_Port; // IP listening port for the caster
	
	// Protocol used to interface with the caster
 	// 0 = TCP (only suppored now)
 	// 1 = UDP
	uint32_t casterProtocol;
	bool ggaRequired; // GGA Message is required by the caster (1 = Required | 0 = Not Required)
	uint32_t ggaOutputInterval; // GGA message output interval (default = 20 seconds)
	uint8_t mountPointStr[100]; // Requested Mount Point data stream
	uint8_t userAgentStr[100]; // NTRIP Client (unit) designation in HTTP request
	uint8_t userAgentVerStr[20]; // NTRIP Client (unit) version in the HTTP request
	uint8_t userIDStr[50]; // user ID used to access the NTRIP caster
	uint8_t userPasswdStr[50]; // user password used to access the NTRIP caster
};

#endif // __HGuideAPI_Msg_6009_h__
