# 0x4109 : Attitude Initialization Message

#
#  The message 0x4109 is used to provide GNSS attitude/heading measurements to the HGuide.
# 

# Variables declaration
	uint32 AddressId  # Start of Message
	uint32 MessageId  # Message ID
	uint32 MessageLength  # Message Length
	uint32 Checksum  # Checksum
	bool RequestAckNak  # 0 = do not request the ACK/NAK of message being received | 1 = request the ACK/NAK of message being received
	bool RollValid  # Input Roll is valid (1 = valid | 0 = invalid)
	bool PitchValid  # Input Pitch is valid (1 = valid | 0 = invalid)
	bool HeadingValid  # Input Heading is valid (1 = valid | 0 = invalid)
	bool TovMode  # Time of Validity Mode: 0 = TOV referenced to GPS Time | 1 = TOV referenced to timestamp of message receipt
	float64 MessageTov  # [s] Time of validity based on TovMode
	float32 Roll  # [rad] Roll
	float32 Pitch  # [rad] Pitch
	float32 TrueHdg  # [rad] Heading
	float32 RollStdv  # [rad] Roll standard deviation
	float32 PitchStdv  # [rad] Pitch standard deviation
	float32 TrueHdgStdv  # [rad] Heading standard deviation
	float32 Baseline  # [m] Baseline between both antennas (distance from RF1 to RF2 antenna)
