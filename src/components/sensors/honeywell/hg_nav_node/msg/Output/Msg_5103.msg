# 0x5103 : GPS Ephemeris Message

#
#  Decoded GPS L1 C/A ephemerides
#  This message contains a single set of GPS ephemeris parameters.
# 

# Variables declaration
	uint32 AddressId  # Start of message flag
	uint32 MessageId  # Message ID
	uint32 MessageLength  # Message length
	uint32 Checksum  # Checksum
	float64 systemTov  # [sec] System Time of Validity
	float64 gpsTov  # [sec] GPS Time of Validity in current week (Sunday 00:00 UTC). When invalid, GPS time = INS/GPS Internal System Time
	gps_mode_table_t GPSMode  # GPS Mode of operation
	uint32 prn_number  # Satellite PRN number
	float64 tow  # Time stamp of subframe 1 (s)
	uint32 health  # Health status - a 6-bit health code as defined in IS-GPS-200
	uint32 IODE1  # Issue of ephemeris data 1
	uint32 IODE2  # Issue of ephemeris data 2
	uint32 week  # toe week number (computed from Z count week)
	uint32 z_week  # Z count week number. This is the week number from subframe 1 of the ephemeris. The ‘toe week’ (field #7) is derived from this to account for rollover
	float64 toe  # Reference time for ephemeris (s)
	float64 A  # Semi-major axis (m)
	float64 dN  # (ΔN)Mean motion difference (radians/s)
	float64 M0  # Mean anomaly of reference time (radians)
	float64 ecc  # Eccentricity, dimensionless
	float64 w  # (ω) Argument of perigee (radians)
	float64 cuc  # Amplitude of cosine harmonic correction term to the argument of latitude (radians)
	float64 cus  # Amplitude of sine harmonic correction term to the argument of latitude (radians)
	float64 crc  # Amplitude of cosine harmonic correction term to the orbit radius (m)
	float64 crs  # Amplitude of sine harmonic correction term to the orbit radius (m)
	float64 cic  # Amplitude of cosine harmonic correction term to the angle of inclination (radians)
	float64 cis  # Amplitude of sine harmonic correction term to the angle of inclination (radians)
	float64 Inclination_angle  # Inclination angle at reference time (radians)
	float64 Inclination_rate  # Rate of inclination angle (radians/s)
	float64 wo  # (wo) Right ascension (radians)
	float64 wi  # (ώ)Rate of right ascension (radians/s)
	uint32 iodc  # Issue of data clock
	float64 toc  # SV clock correction term (s)
	float64 tgd  # Estimated group delay difference (s)
	float64 af0  # Clock aging parameter (s)
	float64 af1  # Clock aging parameter (s/s)
	float64 af2  # Clock aging parameter (s/s/s)
	
	# Anti-spoofing on:
 	# 0 = FALSE
 	# 1 = TRUE
	bool AS
	
	# Corrected mean motion (radians/s)
 	# This field is computed by the receiver.
	float64 N
	
	# User Range Accuracy variance (m^2)
 	# The ICD specifies that the URA index transmitted in the ephemerides can be converted to a nominal
 	# standard deviation value using an algorithm listed there. We publish the square of the nominal value (variance).
 	# The correspondence between the original URA index and the value output is shown in Table: URA Variance
	float64 URA
