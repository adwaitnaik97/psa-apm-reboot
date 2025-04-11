# 0xCE01 : HGuide Console Command

#
#  The 0xCE01 message allows the end-user to send an ASCII string to the HGuide which
#  can be interpretted by as an HGuide console commands. 
#  The 0xCE01 is an input and output message:
#     * As an HGuide input, the ASCII string is sent to the HGuide console.  
#     * As an HGuide output, the 0xCE01 message contains the response for the HGuide console.
#  When using 0xCE01 on ethernet interface, please select PortId = 1 / PortId = 3.
# 

# Variables declaration
	uint32 AddressId  # Start of Message
	uint32 MessageId  # Message ID
	uint32 MessageLength  # Message Length
	uint32 Checksum  # Checksum
	
	# Customer Comm Port ID:
 	# 0 = Current_Port
 	# 1 = COM1
 	# 2 = COM2
 	# 3 = COM3
 	# 4 = COM4
 	# 5 = SPI
 	# 7 = USB
 	# 9 = ETHERNET
 	# 12 = CAN
	uint32 PortId
	uint32 ASCII_String_Length  # Length of ASCII String [Number of Bytes]
	
	# ASCII String [pad with zeros]
	# The string has to be ended with "\ r \ n" [0x0D 0x0A]
	uint8[360] ASCII_String
