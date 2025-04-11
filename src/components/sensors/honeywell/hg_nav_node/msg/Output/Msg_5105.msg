# 0x5105 : GALILEO Ephemeris Message

#
#  Decoded GLONASS L1 C/A ephemeris
#  The message contains the Galileo I/NAV and F/NAV ephemeris information.
#  Multiple messages are transmitted, one for each SVID with date. 
#  The I/NAV messages are only transmitted on E1 and E5b.
#  The F/NAV messages are only transmitted on E5a.
# 

# Variables declaration
	uint32 AddressId  # Start of message flag
	uint32 MessageId  # Message ID
	uint32 MessageLength  # Message length
	uint32 Checksum  # Checksum
	float64 systemTov  # [sec] System Time of Validity
	float64 gpsTov  # [sec] GPS Time of Validity in current week (Sunday 00:00 UTC). When invalid, GPS time = INS/GPS Internal System Time
	gps_mode_table_t GPSMode  # GPS Mode of operation
	
	# type of Galileo Ephemeris
 	# 0 = I/NAV
 	# 1 = F/NAV
	uint16 ephem_type
	uint16 satellite_ID  # Satellite identifier
	uint8 E5aHealth  # E5a health status bits
	uint8 E5bHealth  # E5b health status bits
	uint8 E5aDVS  # E5a data validity status
	uint8 E5bDVS  # E5b data validity status
	uint8 E1bHealth  # E1b health status bits
	uint8 E1bDVS  # E1b data validity status
	uint16 IODnav  # Issue of data ephemeris
	uint8 SISA_Index  # Signal in space accuracy (unitless)
	
	# Identifies the source signal:
 	# 0 = Unknown
 	# 1 = E1b
 	# 2 = E5b
 	# 3 = E1b and E5b
	uint8 INAV_Source
	uint32 T0e  # Ephemeris reference time (s)
	uint32 T0c  # Clock correction data reference time of week from the I/NAV message (s)
	float64 M0  # Mean anomaly at ref time (radians)
	float64 DeltaN  # Mean motion difference (radians/s)
	float64 Ecc  # Eccentricity (unitless)
	float64 RootA  # Square root of semi-major axis
	float64 I0  # Inclination angle at ref time (radians)
	float64 IDot  # Rate of inclination angle (radians/s)
	float64 Omega0  # Longitude of ascending node of orbital plane at weekly epoch (radians)
	float64 Omega  # Argument of perigee (radians)
	float64 OmegaDot  # Rate of right ascension (radians/s)
	float64 Cuc  # Amplitude of the cosine harmonic correction term to the argument of latitude (radians)
	float64 Cus  # Amplitude of the sine harmonic correction term to the argument of latitude (radians)
	float64 Crc  # Amplitude of the cosine harmonic correction term to the orbit radius (m)
	float64 Crs  # Amplitude of the sine harmonic correction term to the orbit radius (m)
	float64 Cic  # Amplitude of the cosine harmonic correction term to the angle of inclination (radians)
	float64 Cis  # Amplitude of the sine harmonic correction term to the angle of inclination (radians)
	float64 Af0  # SV clock bias correction coefficient from the I/NAV message (s)
	float64 Af1  # SV clock drift correction coefficient from the I/NAV message (s/s)
	float64 Af2  # SV clock drift rate correction coefficient from the I/NAV message (s/s^2)
	float64 E1E5aBGD  # E1, E5a broadcast group delay
	float64 E1E5bBGD  # E1, E5b broadcast group delay
