# 0x1001 : Enable/Disable Output Messages

#
#  The 0x1001 message can be used to enable and disable output messages.  
#  The message consists of 32-bit bitfields which enable a message if the bit is 
#  set to one (1 / True) and disable the message if the bit is set to zero (0 / False).
# 

# Variables declaration
	uint32 AddressId  # Start of Message
	uint32 MessageId  # Message ID
	uint32 MessageLength  # Message Length [Number of 32-bit Words]
	uint32 Checksum  # 32-bit CRC
	hgnsi_message_word1_t MessageWord1  # Word 1 with SAVE DATA TO MEMORY and message enable bits
	hgnsi_message_word2_t MessageWord2  # Word 2 with message enable bits
	hgnsi_message_word3_t MessageWord3  # Word 3 with message enable bits
	hgnsi_message_word4_t MessageWord4  # Word 4 with message enable bits
