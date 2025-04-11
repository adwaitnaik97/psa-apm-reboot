# 0x2429 : Transfer Alignment Normalized Residuals

#
#  Residuals of generic Transfer align measurements.
#  This message reports residuals of experimental body velocity aiding via 0x4110 input
# 

# Variables declaration
	uint32 AddressId  # Start of Message
	uint32 MessageId  # Message ID
	uint32 MessageLength  # Message Length
	uint32 Checksum  # Checksum
	ins_mode_table_t INSMode  # INS Mode table
	float64 systemTov  # Time since Power Up - Always Valid [sec]
	float64 gpsTov  # [sec] GPS Time of Validity in current week (Sunday 00:00 UTC). When invalid, GPS time = INS/GPS Internal System Time
	int32 gps_week  # Number of weeks since Jan 6, 1980 GPS Week = -1 indicates GPS Week Unknown/Invalid
	float32 TA_PositionX_MeasResid  # [sigma]
	float32 TA_PositionY_MeasResid  # [sigma]
	float32 TA_PositionZ_MeasResid  # [sigma]
	float32 TA_VelocityX_MeasResid  # [sigma]
	float32 TA_VelocityY_MeasResid  # [sigma]
	float32 TA_VelocityZ_MeasResid  # [sigma]
	float32 TA_AttitudeX_MeasResid  # [sigma]
	float32 TA_AttitudeY_MeasResid  # [sigma]
	float32 TA_AttitudeZ_MeasResid  # [sigma]
	float32 TA_AttitudeCosZ_MeasResid  # [sigma]
