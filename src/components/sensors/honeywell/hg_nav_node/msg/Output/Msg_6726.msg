# 0x6726 : Current Profile Correlation Data

#
#  Current profile correlation data (variable length in Nortek DF3).
#  Message outputs data for 4 beams and 30 cells.
# 

# Variables declaration
	uint32 AddressId  # Start of message flag
	uint32 MessageId  # Message ID
	uint32 MessageLength  # Message length in 32-bit word
	uint32 Checksum  # Checksum
	float64 systemTov  # Trigger/ping time of ensemble (bottom track trigger time)
	uint8[30] corBeam0  # Correlation Beam 0 Cells 0-29 [0 – 100]
	uint8[30] corBeam1  # Correlation Beam 1 Cells 0-29 [0 – 100]
	uint8[30] corBeam2  # Correlation Beam 2 Cells 0-29 [0 – 100]
	uint8[30] corBeam3  # Correlation Beam 3 Cells 0-29 [0 – 100]
