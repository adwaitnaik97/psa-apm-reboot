# 0x1101 : Input Message for Barometric Altitude

#
#  The 0x1101 message can be used to input barometric altitude.  These inputs include time and altitude.
# 

# Variables declaration
	uint32 AddressId  # Start of Message
	uint32 MessageId  # Message ID
	uint32 MessageLength  # Message Length [Number of 32-bit Words]
	uint32 Checksum  # 32-bit CRC
	float64 BarometricAltitudeTov  # [s] Time of Validity for Barometric Altitude
	bool BarometricAltitudeValidity  # Validity bit for Barometric Altitude (1 = Valid | 0 = Invalid)
	bool TimeReferenceMode  # BarometricAltitudeTov Time of Reference (0 = Referenced to GPS Tov | 1 = Referenced to Message receipt time)
	float32 BarometricAltitudeMSLGeoid  # [m] Send Barometric Altitude (MSL Geoid)
