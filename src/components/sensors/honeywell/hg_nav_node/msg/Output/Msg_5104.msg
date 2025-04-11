# 0x5104 : GLONASS Ephemeris Message

#
#  Decoded GLONASS L1 C/A ephemeris
#  This message contains GLONASS L1 C/A ephemeris information. GLONASS ephemerides are referenced to the PZ90.02 geodetic datum. 
#  No adjustment between the GPS and GLONASS reference frames are made for positioning. 
#  Multiple messages are transmitted, one for each SVID with data.
# 

# Variables declaration
	uint32 AddressId  # Start of message flag
	uint32 MessageId  # Message ID
	uint32 MessageLength  # Message length
	uint32 Checksum  # Checksum
	float64 systemTov  # [sec] System Time of Validity
	float64 gpsTov  # [sec] GPS Time of Validity in current week (Sunday 00:00 UTC). When invalid, GPS time = INS/GPS Internal System Time
	gps_mode_table_t GPSMode  # GPS Mode of operation
	
	# Slot information offset - PRN identification
	# (Slot + 37). This is also called SLOTO in Connect
	uint16 slot_offset
	uint16 freq_offset  # Frequency channel offset for satellite in the range 0 to 20
	
	# Satellite type where:
	# 0 = GLO_SAT
	# 1 = GLO_SAT_M (M type)
	# 2 = GLO_SAT_K (K type)
	uint8 sat_type
	uint16 e_week  # Reference week of ephemeris (GPS reference time)
	uint32 e_time  # Reference time of ephemeris (GPS reference time) (ms)
	int32 t_offset  # Integer seconds between GPS and GLONASS time. A positive value implies GLONASS is ahead of GPS reference time.
	uint16 Nt  # Calendar number of day within 4 year interval starting at Jan 1 of a leap year
	uint32 issue  # 15 minute interval number corresponding to ephemeris reference time
	
	# Ephemeris health where:
	# 0-3 = GOOD
	# 4-15 = BAD
	uint32 health1
	float64 pos_x  # X coordinate for satellite at reference time (PZ-90.02) (m)
	float64 pos_y  # Y coordinate for satellite at reference time (PZ-90.02) (m)
	float64 pos_z  # Z coordinate for satellite at reference time (PZ-90.02) (m)
	float64 vel_x  # X coordinate for satellite velocity at reference time (PZ-90.02) (m/s)
	float64 vel_y  # Y coordinate for satellite velocity at reference time (PZ-90.02) (m/s)
	float64 vel_z  # Z coordinate for satellite velocity at reference time (PZ-90.02), (m/s)
	float64 LS_acc_x  # X coordinate for lunisolar acceleration at reference time (PZ-90.02), (m/s/s)
	float64 LS_acc_y  # Y coordinate for lunisolar acceleration at reference time (PZ-90.02) (m/s/s)
	float64 LS_acc_z  # Z coordinate for lunisolar acceleration at reference time (PZ-90.02) (m/s/s)
	float64 tau_n  # Correction to the nth satellite time t_n relative to GLONASS time t_c (s)
	float64 delta_tau_n  # Time difference between navigation RF signal transmitted in L2 sub-band and navigation RF signal transmitted in L1 sub-band by nth satellite (s)
	float64 gamma  # Frequency correction (s/s)
	uint32 Tk  # Time of frame start (since start of GLONASS day) (s)
	uint32 P  # Technological parameter
	uint32 Ft  # User range
	uint32 age  # Age of data (days)
	uint32 Flags  # Information flags, see Table: GLONASS Ephemeris Flags Coding
	
	# P1 Flag - Time interval between adjacent values of (tb) parameter in minutes
 	# 00b (0) = 0 minutes
 	# 01b (1) = 30 minutes
 	# 10b (2) = 45 minutes
 	# 11b (3) = 60 minutes
	uint8 flag_p1
	
	# P2 Flag - Flag of oddness or evenness of the value of (tb) (for intervals of 30 or 60 minutes)
	# 0 = even
	# 1 = odd
	bool flag_p2
	
	# P3 Flag - Number of satellites with almanac information within current subframe
	# 0 = four
	# 1 = five
	bool flag_p3
	
	# P4 Flag - Flag to show that ephemeris parameters are present. "1" indicates that updated ephemeris or frequency/time parameters have been uploaded by the control
	# 0 = no updated ephemeris
	# 1 = updated ephemeris present
	bool flag_p4
