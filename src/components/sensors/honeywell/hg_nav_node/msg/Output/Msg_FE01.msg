# 0xFE01 : Read and Write Memory Message

#
#  The 0xFE01 message allows the end-user to read and write memory from the HGuide using the HGuide Navigation Software Interface (HGNSI).
#  The 0xFE01 message can be used for the following purposes:
#  * Perform software updates
#  * Read out software and software configuration
#  * Set factory data in flash memory
#  * Set customer data in flash memory
# 

# Variables declaration
	uint32 AddressId  # Start of Message
	uint32 MessageId  # Message ID
	uint32 MessageLength  # Message Length
	uint32 Checksum  # Checksum
	msg_FE01_parameter_1_t Parameter_1  # Parameter 1 an enumeration of command and response.
	uint32 Parameter_2  # Parameter 2 depends on the value of Parameter_1. Please see the sequence diagram for more details.
	uint32 Parameter_3  # Parameter 3 depends on the value of Parameter_1. Please see the sequence diagram for more details.
	uint32 Parameter_4  # Parameter 4 depends on the value of Parameter_1. Please see the sequence diagram for more details.
	uint8[248] Payload  # Payload depends on the value of Parameter_1. Please see the sequence diagram for more details.
