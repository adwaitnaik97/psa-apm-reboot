# 0x5102 : GNSS Satellite Position

#
#  the 0x5102 message contains Satellite positions in ECEF Cartesian coordinates.
#  When combined with a 0x5101 GNSS RANGE message, this data set contains the decoded satellite information necessary to compute the solution
#  The length of the message is based on the amount of satellites. Check the num_satellites field to properly decode the message.
#  This message can be taken as Satellite Position input by any HGuide INS system.
# 

# Variables declaration
	uint32 AddressId  # Start of Message
	uint32 MessageId  # Message ID
	
	# Message Length [Number of 32-bit Words]
	# Make sure to recalculate based on the amount satellite positions
	uint32 MessageLength
	uint32 Checksum  # 32-bit CRC
	float64 systemTov  # [sec] System Time of Validity
	float64 gpsTov  # [sec] GPS Time of Validity in current week (Sunday 00:00 UTC). When invalid, GPS time = INS/GPS Internal System Time
	gps_mode_table_t GPSMode  # GPS Mode of operation
	uint32 num_satellites  # Number of satellites to follow - defines the size of the variable size message
	
	# Satellite positions - each 18 words long
	# Repeated block - make sure to allocate memory for the sat_position via .set_sat_position_size(size) method
	# if amount of data is higher than the allocated size, only allocated size shall be populated
	satellite_pos_t[] sat_position
