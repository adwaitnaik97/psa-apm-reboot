# 0x2425 : Transfer Align Measurement errors and uncertainties

#
#  Error estimations for the Transfer Align system
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
	float32 TA_PosAidingX_ErrEst  # [m]
	float32 TA_PosAidingY_ErrEst  # [m]
	float32 TA_PosAidingZ_ErrEst  # [m]
	float32 TA_VelAidingX_ErrEst  # [m/s]
	float32 TA_VelAidingY_ErrEst  # [m/s]
	float32 TA_VelAidingZ_ErrEst  # [m/s]
	float32 TA_LeverArmX_ErrEst  # [m]
	float32 TA_LeverArmY_ErrEst  # [m]
	float32 TA_LeverArmZ_ErrEst  # [m]
	float32 TA_BoresightX_ErrEst  # [mrad]
	float32 TA_BoresightY_ErrEst  # [mrad]
	float32 TA_BoresightZ_ErrEst  # [mrad]
	float32 TA_PosAidingX_Stdv  # [m]
	float32 TA_PosAidingY_Stdv  # [m]
	float32 TA_PosAidingZ_Stdv  # [m]
	float32 TA_VelAidingX_Stdv  # [m/s]
	float32 TA_VelAidingY_Stdv  # [m/s]
	float32 TA_VelAidingZ_Stdv  # [m/s]
	float32 TA_LeverArmX_Stdv  # [m]
	float32 TA_LeverArmY_Stdv  # [m]
	float32 TA_LeverArmZ_Stdv  # [m]
	float32 TA_BoresightX_Stdv  # [mrad]
	float32 TA_BoresightY_Stdv  # [mrad]
	float32 TA_BoresightZ_Stdv  # [mrad]
