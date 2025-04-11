# 0x5106 : BEIDOU Ephemeris Message

#
#  Decoded BDS2 and BDS3 ephemeris
#  This message contains a single set of BDS2/BSD3 ephemeris parameters with appropriate scaling applied. 
#  Multiple messages are transmitted, one for each SV ephemeris collected.
# 

# Variables declaration
	uint32 AddressId  # Start of message flag
	uint32 MessageId  # Message ID
	uint32 MessageLength  # Message length
	uint32 Checksum  # Checksum
	float64 systemTov  # [sec] System Time of Validity
	float64 gpsTov  # [sec] GPS Time of Validity in current week (Sunday 00:00 UTC). When invalid, GPS time = INS/GPS Internal System Time
	gps_mode_table_t GPSMode  # GPS Mode of operation
	uint8 ephem_type  # Type of BeiDou ephemeris (0 = BDS2 | 1 = BDS3)
	
	# BeiDou Satellite type
	# 0 = Reserved
	# 1 = GEO
	# 2 = IGSO
	# 3 = MEO
	uint8 sat_type
	uint32 satellite_ID  # ID/ranging code
	uint32 Week  # BeiDou week number
	float64 URA  # User range accuracy (m). This is the evaluated URAI/URA lookup-table value.
	uint32 health1  # Autonomous satellite health flag. 0 means broadcasting satellite is good and 1 means not.
	float64 tgd1  # Equipment group delay differential for the B1 signal (s)
	float64 tgdb2cp  # [BDS3] Group delay differential between the B1C pilot component and B3I signal(sent by B2a and B1c)
	float64 tgd2  # Equipment group delay differential for the B2 signal (s)
	float64 tgdb2bi  # [BDS3] Group delay differential between the b2b and B3i (s),only sent by B2b non-GEO satellites
	uint32 AODC  # Age of data, clock
	uint32 IODC  # [BDS3] Age of data, clock
	uint32 toc  # Reference time of clock parameters (s)
	float64 a0  # Constant term of clock correction polynomial (s)
	float64 a1  # Linear term of clock correction polynomial (s/s)
	float64 a2  # Quadratic term of clock correction polynomial (s/s^2)
	uint32 AODE  # Age of data, ephemeris
	uint32 toe  # Reference time of ephemeris parameters (s)
	float64 RootA  # Square root of semi-major axis (sqrt(m))
	float64 ecc  # Eccentricity (dimensionless)
	float64 w  # (ω) Argument of perigee (radians)
	float64 dN  # (ΔN) Mean motion difference from computed value (radians/s)
	float64 M0  # Mean anomaly at reference time (radians)
	float64 omega0  # (Ω0) Longitude of ascending node of orbital of plane computed according to reference time (radians)
	float64 omegai  # (Ω`) Rate of right ascension (radians/s)
	float64 i0  # Inclination angle at reference time (radians)
	float64 IDOT  # Rate of inclination angle (radians/second)
	float64 cuc  # Amplitude of cosine harmonic correction term to the argument of latitude (radians)
	float64 cus  # Amplitude of sine harmonic correction term to the argument of latitude (radians)
	float64 crc  # Amplitude of cosine harmonic correction term to the orbit radius (m)
	float64 crs  # Amplitude of sine harmonic correction term to the orbit radius (m)
	float64 cic  # Amplitude of cosine harmonic correction term to the angle of inclination (radians)
	float64 cis  # Amplitude of sine harmonic correction term to the angle of inclination (radians)
	float64 tgd3  # [BDS3] Tgdb2ap,group delay differential between the B2a pilot component and the B3I signal(sent by both B2a and B1c)
	float64 IscB1cd  # [BDS3] Group delay differential between the B1C data and pilot components,only sent by B1c
	float64 IscB2ad  # [BDS3] Group delay differential between the B2a data and pilot components,only sent by B2a
	uint32 SISMAI  # [BDS3] RESERVED - signal in space monitoring accuracy index for BDS3 satellites, will be published in future ICD
	float64 deltaA  # [BDS3] semi-major axis difference at reference time
	float64 Adot  # [BDS3] change rate in semi-major axis
	float64 deltaN0Dot  # [BDS3] Rate of mean motion difference form computed value at reference time
