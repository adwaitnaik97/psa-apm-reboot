# 0x5101 : GNSS Range Data

#
#  The 0x5101 message contains the channel measurements for the currently tracked satellites.
#  The length of the message is based on the amount of observations. Check the Num_Observations field to properly decode the message.
#  This message can be taken as GNSS Range input by any HGuide INS system.
# 

# Variables declaration
	uint32 AddressId  # Start of Message
	uint32 MessageId  # Message ID
	
	# Message Length [Number of 32-bit Words]
	# Make sure to recalculate based on the amount of observations
	uint32 MessageLength
	uint32 Checksum  # 32-bit CRC
	float64 systemTov  # [sec] System Time of Validity
	float64 gpsTov  # [sec] GPS Time of Validity in current week (Sunday 00:00 UTC). When invalid, GPS time = INS/GPS Internal System Time
	gps_mode_table_t GPSMode  # GPS Mode of operation
	uint32 Num_Observations  # Number of observations in this message - defines the size of the variable size message
	
	# GNSS Observations each 11 words long
 	# Repeated block - make sure to allocate memory for the observation via .set_observation_size(size) method
 	# if amount of ranges is higher than the allocated size, only allocated size shall be populated
	observation_t[] observation
