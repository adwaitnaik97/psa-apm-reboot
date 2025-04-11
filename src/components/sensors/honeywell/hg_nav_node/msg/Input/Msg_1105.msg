# 0x1105 : Input Message for Magnetic Heading

#
#  The 0x1105 message can be used to input Magnetic Heading.  These inputs include time and Heading in radians.
# 

# Variables declaration
	uint32 AddressId  # Start of Message
	uint32 MessageId  # Message ID
	uint32 MessageLength  # Message Length [Number of 32-bit Words]
	uint32 Checksum  # 32-bit CRC
	float64 MagneticHeadingTov 
	bool MagneticHeadingValidity  # Validity bit for Magnetic Heading (1 = Valid | 0 = Invalid)
	bool TimeReferenceMode  # MagneticHeadingTov Time of Reference (0 = Referenced to GPS Tov | 1 = Referenced to Message receipt time)
	bool MagneticVariationValidity  # Validity bit for Magnetic Variation (1 = Valid | 0 = Invalid)
	float32 MagneticHeading  # [rad] Magnetic Heading
	float32 MagneticVariation  # [mgauss] Magnetic Variation
