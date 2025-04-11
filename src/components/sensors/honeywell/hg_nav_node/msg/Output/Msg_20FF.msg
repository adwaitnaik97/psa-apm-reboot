# 0x20FF : ACK/NAK Message

#
#  Message sent in response to all input messages (unless explicitly disabled by the input message)
# 

# Variables declaration
	uint32 AddressId  # Start of Message
	uint32 MessageId  # Message ID
	uint32 MessageLength  # Message Length in 32-bit Words
	uint32 Checksum  # Checksum : <br> UART : 2's complement sum such that all data, including the header, equals 0
	uint32 Ack  # 0=NAK, 1=ACK
	uint32 InputMessageID  # ID of the meassage responding to
	uint32 NoOfValidMessagesSinceLast  # [-] number of input messages since last 0x20FF
	uint32 NoOfValidMessagesSincePowerUp  # [-] number of input messages since power up
	float64 MessageTimeOfReception  # [s] INS System time of reception
