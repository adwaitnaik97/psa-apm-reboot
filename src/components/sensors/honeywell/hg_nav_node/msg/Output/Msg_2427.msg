# 0x2427 : Magnetometer errors and uncertainties

#
#  Error estimations for the Magnetometer system
# 

# Variables declaration
	uint32 AddressId  # Start of Message
	uint32 MessageId  # Message ID
	uint32 MessageLength  # Message Length in 32-bit Words
	uint32 Checksum  # Checksum
	ins_mode_table_t INSMode  # INS Mode table
	float64 systemTov  # Time since Power Up - Always Valid [sec]
	float64 gpsTov  # [sec] GPS Time of Validity in current week (Sunday 00:00 UTC). When invalid, GPS time = INS/GPS Internal System Time
	int32 gps_week  # Number of weeks since Jan 6, 1980 GPS Week = -1 indicates GPS Week Unknown/Invalid
	float32 MagBiasErrorEstimate 
	float32 MagBiasErrorStdv 
	float32 AttZNormalizedMeasResid 
	float32 AttCosZNormalizedMeasResid 
