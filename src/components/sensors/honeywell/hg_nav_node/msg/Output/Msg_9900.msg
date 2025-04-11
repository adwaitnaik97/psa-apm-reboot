# 0x9900 : Log Message

#
#  The string contains fault's system time, severity and message
# 

# Variables declaration
	uint32 AddressId  # Start of Message
	uint32 MessageId  # Message ID
	uint32 MessageLength  # Message Length
	uint32 Checksum  # Checksum
	uint8[160] ASCII_String  # ASCII String [pad with zeros]
