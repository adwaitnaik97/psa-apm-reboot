#ifndef __HGuideAPI_Msg_1005_h__
#define __HGuideAPI_Msg_1005_h__
#pragma once

#include <cstdint>

#include <include/hgnsi_message_word1_t.h>
#include <include/hgnsi_message_word2_t.h>
#include <include/hgnsi_message_word3_t.h>
#include <include/uart_baud_type_t.h>
#include <include/uart_number_of_bits_t.h>
#include <include/uart_parity_t.h>
#include <include/uart_number_of_stop_bits_t.h>
#include <include/save_configuration_t.h>
#include <include/interface_protocol_t.h>
#include <include/port_id_t.h>


// 0x1005 : Port Configuration Message
//
// The 0x1005 message allows the end-user to reconfigure the communication ports.
// The unit shall reply with 0x2005.
// This message is shared between the INS and IMU portfolio. The differentiation is by the 'InterfaceSelect' variable.
// The last 4 words function is dependant on the 'InterfaceSelect' variable
// 
class HGUIDE_DLL Msg_1005
{
public:
	Msg_1005();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 40;}

public:
	static const uint32_t AddressId = 0xA5C381FF; // Start of Message
	static const uint32_t MessageId = 0x1005; // Message ID
	static const uint32_t MessageLength = 10; // Message Length
	uint32_t Checksum; // Checksum
	port_id_t PortId; // Customer Comm Port ID (0 = Current)
	interface_protocol_t InterfaceSelect; // Protocol to be used over the interface
	uart_baud_type_t BaudRate; // Set Uart Baud Rate
	uart_number_of_bits_t NumberOfBits; // Set Uart Number of Data Bits (0 = Current)
	uart_parity_t Parity; // Set Uart Parity (0 = Current)
	uart_number_of_stop_bits_t NumberOfStopBits; // Set Uart Number of Stop Bits (0 = Current)
	
	// Save the port configuration to non-volatile memory (0 = Don't Save | 1 = Save | 4 = Bad Checksum)
	// Note For INS: Configuration is always saved to memory after successfull sequence
	save_configuration_t SaveConfiguration;
	hgnsi_message_word1_t MessageWord1; // DO NOT USE - Word 1 with SAVE DATA TO MEMORY and message enable bits
	hgnsi_message_word2_t MessageWord2; // DO NOT USE - Word 2 with message enable bits
	hgnsi_message_word3_t MessageWord3; // DO NOT USE - Word 3 with message enable bits
	uint32_t MessageWord4; // DO NOT USE - Message Select Bit Field 4
	
	// Requested IMU Message ID
 	// Selecting control message ID indicates to send only control data i.e.:
 	// 0x01 = Control Data only
 	// 0x02 = Interleaved 0x01 and 0x02
	uint32_t RequestedMessageID;
	
	// Requested Control Data Frequency
 	// has to be 6x nav frequency value
 	// 600
 	// 1200
 	// 1800
	uint32_t ControlFrequency;
	
	// Requested Navigation Data Frequency
	// has to be 1/6 of Control frequency value
	// 100
	// 200
	// 300
	uint32_t NavigationFrequency;
};

#endif // __HGuideAPI_Msg_1005_h__
