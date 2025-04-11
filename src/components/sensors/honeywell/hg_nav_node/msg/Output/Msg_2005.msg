# 0x2005 : Port Configuration Reply Message

#
#  The 0x2005 message is a reply to 0x1005 message allowing the end-user to reconfigure the communication ports.
#  The message structure is identical to 0x1005
#  In case of NAK, this message highlights the incorrect settings, helping the user correct the 0x1005 configuration message
# 

# Variables declaration
	uint32 AddressId  # Start of Message
	uint32 MessageId  # Message ID
	uint32 MessageLength  # Message Length
	uint32 Checksum  # Checksum
	port_id_t PortId  # Customer Comm Port ID (0 = Current)
	interface_protocol_t InterfaceSelect  # Protocol to be used over the interface
	uart_baud_type_t BaudRate  # Set Uart Baud Rate
	uart_number_of_bits_t NumberOfBits  # Set Uart Number of Data Bits (0 = Current)
	uart_parity_t Parity  # Set Uart Parity (0 = Current)
	uart_number_of_stop_bits_t NumberOfStopBits  # Set Uart Number of Stop Bits (0 = Current)
	save_configuration_t SaveConfiguration  # Save the port configuration to non-volatile memory (0 = Don't Save | 1 = Save | 4 = Bad Checksum)
	uint32 parameter_1  # Parameter 1 response
	uint32 parameter_2  # Parameter 2 response
	uint32 parameter_3  # Parameter 3 response
	uint32 parameter_4  # Parameter 4 response
