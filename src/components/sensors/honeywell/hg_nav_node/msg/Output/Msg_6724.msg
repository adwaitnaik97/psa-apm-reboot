# 0x6724 : Current Profile Velocity Data

#
#  Current profile velocity data (variable length in Nortek DF3).
#  Message outputs data for 4 beams and 30 cells.
#  Velocity Scaling is located in 0x6723 as 'velocityScaling' field
#  multiply the velBeam value by this scale and store in float32 to use the value.
# 

# Variables declaration
	uint32 AddressId  # Start of message flag
	uint32 MessageId  # Message ID
	uint32 MessageLength  # Message length in 32-bit word
	uint32 Checksum  # Checksum
	float64 systemTov  # Trigger/ping time of ensemble (bottom track trigger time)
	int16[30] velBeam0  # Velocity Beam 0 Cells 0-29 (10^velocity scaling m/s)
	int16[30] velBeam1  # Velocity Beam 1 Cells 0-29 (10^velocity scaling m/s)
	int16[30] velBeam2  # Velocity Beam 2 Cells 0-29 (10^velocity scaling m/s)
	int16[30] velBeam3  # Velocity Beam 3 Cells 0-29 (10^velocity scaling m/s)
