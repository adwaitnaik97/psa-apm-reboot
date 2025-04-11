# 0x4204 : Antenna lever arms settings

#
#  The message 0x4204 is used to provide Antenna Lever arms to the HGuide.
#  Lever arms are read upon power up.  After changing lever arms, cycle power.
#  Measure from the case frame origin to the antenna phase center.
#  The GNSS true heading reference is defined by the projection of RF1 (Main) 
#  to RF2 (Aux) on to local level.
# 

# Variables declaration
	uint32 AddressId  # Start of Message
	uint32 MessageId  # Message ID
	uint32 MessageLength  # Message Length
	uint32 Checksum  # Checksum
	float32 RF1AntennaX  # [m] X RF1 (main) antenna lever arm
	float32 RF1AntennaY  # [m] Y RF1 (main) antenna lever arm
	float32 RF1AntennaZ  # [m] Z RF1 (main) antenna lever arm
	float32 RF2AntennaX  # [m] X RF2 (aux) antenna lever arm
	float32 RF2AntennaY  # [m] Y RF2 (aux) antenna lever arm
	float32 RF2AntennaZ  # [m] Z RF2 (aux) antenna lever arm
	float32 RF1AntennaSTDV  # [m] Zero is not valid and STDV will default to 1 Meter
	float32 RF2AntennaSTDV  # [m] Zero is not valid and STDV will default to 1 Meter
	bool ChangeRF1LeverArm  # 0 = Temporary | 1 = Flash (Saves Over Power Cyle)
	bool ChangeRF2LeverArm  # 0 = Temporary | 1 = Flash (Saves Over Power Cyle)
	bool ChangeRF1STDV  # 0 = Temporary | 1 = Flash (Saves Over Power Cyle)
	bool ChangeRF2STDV  # 0 = Temporary | 1 = Flash (Saves Over Power Cyle)
