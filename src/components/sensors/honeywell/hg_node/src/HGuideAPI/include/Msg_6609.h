#ifndef __HGuideAPI_Msg_6609_h__
#define __HGuideAPI_Msg_6609_h__
#pragma once

#include <cstdint>

// NTRIP communication status
struct ntrip_status_t
{
	bool PortOpened; // Port Opened (1 = Opened | 0 = Closed)
	bool PortConnected; // Port Connected (1 = Connected | 0 = Disconnected)
	bool EthernetLink; // Ethernet Link (1 = Active | 0 = Inactive)
	bool DhcpBound; // DHCP bound (1 = Bound | 0 = Unbound)
	bool SocketConnected; // Socket Connected (1 = Connected | 0 = Disconnected)

	void Default()
	{
		PortOpened = 0;
		PortConnected = 0;
		EthernetLink = 0;
		DhcpBound = 0;
		SocketConnected = 0;
	}
};

// NTRIP state machine states
enum ntrip_states_t
{
	UNKNOWN          = 0,
	INITIAL          = 1, //  NTRIP state machine start state
	REQ_MOUNTPOINT   = 2, //  Requesting mount point data
	MOUNTPOINT_REPLY = 3, //  Processing mount point response
	NORMAL           = 4, //  Normal Operation - RTCM stream active
	CASTER_ERROR     = 5, //  Error on Caster (NTRIP server) side
	CLIENT_ERROR     = 6 //  Error on Client (unit) side
};


// 0x6609 : Ntrip Status
//
// Internal Networked Transport of RTCM via Internet Protocol (NTRIP) status output.
// Message gives information about the status of the communications and state of the NTRIP module.
// 
class HGUIDE_DLL Msg_6609
{
public:
	Msg_6609();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 160;}

public:
	static const uint32_t AddressId = 0xA5C381FF; // Start of Message
	static const uint32_t MessageId = 0x6609; // Message ID
	static const uint32_t MessageLength = 40; // Message Length
	uint32_t Checksum; // Checksum
	ntrip_status_t casterStatus; // NTRIP Caster Communication Status (NTRIP server)
	ntrip_status_t clientStatus; // NTRIP Client Communication Status (unit)
	bool ggaRequired; // GGA required during mount point request (1 = Required | 0 = Not Required)
	bool ggaLenValid; // GGA length is valid (1 = valid | 0 = invalid)
	bool pvtValid; // Position, Velocity, Time (PVT) information is valid (1 = valid | 0 = invalid)
	bool pvtLenValid; // PVT length is valid (1 = valid | 0 = invalid)
	bool WaitingForReply; // Waiting for reply from Mount Point (1 = waiting | 0 = reply received)
	bool FoundICY200; // Connection success (1 = success | 0 = waiting). Mount point replied with ICY200
	bool InvalidReply; // Invalid Mount Point reply (1 = invalid reply | 0 = OK)
	bool TimedOut; // Mount point connection attempt timed out (1 = timeout | 0 = OK)
	ntrip_states_t currentState; // Current NTRIP State
	ntrip_states_t previousState; // Previous NTRIP State
	ntrip_states_t lastErrState; // Last error state
	uint8_t lastErrSting[128]; // Last error string detailing the mostRecentErrState
};

#endif // __HGuideAPI_Msg_6609_h__
