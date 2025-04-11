#ifndef __HGuideAPI_Msg_2005_h__
#define __HGuideAPI_Msg_2005_h__
#pragma once

#include <cstdint>

#include <include/uart_baud_type_t.h>
#include <include/uart_number_of_bits_t.h>
#include <include/uart_parity_t.h>
#include <include/uart_number_of_stop_bits_t.h>
#include <include/save_configuration_t.h>
#include <include/interface_protocol_t.h>
#include <include/port_id_t.h>


// 0x2005 : Port Configuration Reply Message
//
// The 0x2005 message is a reply to 0x1005 message allowing the end-user to reconfigure the communication ports.
// The message structure is identical to 0x1005
// In case of NAK, this message highlights the incorrect settings, helping the user correct the 0x1005 configuration message
// 
class HGUIDE_DLL Msg_2005
{
public:
	Msg_2005();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 40;}

public:
	static const uint32_t AddressId = 0xA5C381FF; // Start of Message
	static const uint32_t MessageId = 0x2005; // Message ID
	static const uint32_t MessageLength = 10; // Message Length
	uint32_t Checksum; // Checksum
	port_id_t PortId; // Customer Comm Port ID (0 = Current)
	interface_protocol_t InterfaceSelect; // Protocol to be used over the interface
	uart_baud_type_t BaudRate; // Set Uart Baud Rate
	uart_number_of_bits_t NumberOfBits; // Set Uart Number of Data Bits (0 = Current)
	uart_parity_t Parity; // Set Uart Parity (0 = Current)
	uart_number_of_stop_bits_t NumberOfStopBits; // Set Uart Number of Stop Bits (0 = Current)
	save_configuration_t SaveConfiguration; // Save the port configuration to non-volatile memory (0 = Don't Save | 1 = Save | 4 = Bad Checksum)
	uint32_t parameter_1; // Parameter 1 response
	uint32_t parameter_2; // Parameter 2 response
	uint32_t parameter_3; // Parameter 3 response
	uint32_t parameter_4; // Parameter 4 response
};

#endif // __HGuideAPI_Msg_2005_h__
