# 0xFE02 : Customer Data, Unique ID

#
#  The 0xFE02 message allows the end-user to request the unique ID stored in the customer data section of flash memory.
#  The 0xFE01 message is used to program the unique ID.
#  The 0xFE02 message is used by the end-user to query the HGuide for the current unique ID.
#  The 0xFE02 message is used to output the current value of the unique ID.
#  Usage: Send the 0xFE02 to the HGuide with the payload set to zero and the HGuide will respond with current value of the unique ID.
# 

# Variables declaration
	uint32 AddressId  # Start of Message
	uint32 MessageId  # Message ID
	uint32 MessageLength  # Message Length
	uint32 Checksum  # Checksum
	uint8[348] Payload  # Unique ID.
