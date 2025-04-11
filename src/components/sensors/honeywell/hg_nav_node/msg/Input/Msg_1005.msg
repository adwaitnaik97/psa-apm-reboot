# 0x1005 : Port Configuration Message

#
#  The 0x1005 message allows the end-user to reconfigure the communication ports.
#  The unit shall reply with 0x2005.
#  This message is shared between the INS and IMU portfolio. The differentiation is by the 'InterfaceSelect' variable.
#  The last 4 words function is dependant on the 'InterfaceSelect' variable
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
	
	# Save the port configuration to non-volatile memory (0 = Don't Save | 1 = Save | 4 = Bad Checksum)
	# Note For INS: Configuration is always saved to memory after successfull sequence
	save_configuration_t SaveConfiguration
	hgnsi_message_word1_t MessageWord1  # DO NOT USE - Word 1 with SAVE DATA TO MEMORY and message enable bits
	
	# Requested IMU Message ID
 	# Selecting control message ID indicates to send only control data i.e.:
 	# 0x01 = Control Data only
 	# 0x02 = Interleaved 0x01 and 0x02
	uint32 RequestedMessageID
	hgnsi_message_word2_t MessageWord2  # DO NOT USE - Word 2 with message enable bits
	
	# Requested Control Data Frequency
 	# has to be 6x nav frequency value
 	# 600
 	# 1200
 	# 1800
	uint32 ControlFrequency
	hgnsi_message_word3_t MessageWord3  # DO NOT USE - Word 3 with message enable bits
	
	# Requested Navigation Data Frequency
	# has to be 1/6 of Control frequency value
	# 100
	# 200
	# 300
	uint32 NavigationFrequency
	uint32 MessageWord4  # DO NOT USE - Message Select Bit Field 4
