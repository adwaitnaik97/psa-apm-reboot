# 0x6725 : Current Profile Amplitude Data

#
#  Current profile amplitude data (variable length in Nortek DF3).
#  Message outputs data for 4 beams and 30 cells.
# 

# Variables declaration
	uint32 AddressId  # Start of message flag
	uint32 MessageId  # Message ID
	uint32 MessageLength  # Message length in 32-bit word
	uint32 Checksum  # Checksum
	float64 systemTov  # Trigger/ping time of ensemble (bottom track trigger time)
	uint8[30] ampBeam0  # Amplitude Beam 0 Cells 0-29 (1 count)
	uint8[30] ampBeam1  # Amplitude Beam 1 Cells 0-29 (1 count)
	uint8[30] ampBeam2  # Amplitude Beam 2 Cells 0-29 (1 count)
	uint8[30] ampBeam3  # Amplitude Beam 3 Cells 0-29 (1 count)
