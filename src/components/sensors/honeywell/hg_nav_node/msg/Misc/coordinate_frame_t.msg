#
# Coordinate Frame enumeration
#
# coordinate_frame_t - enumeration
uint8 value

#Pre-Defined ENUM Constants
	uint8 INVALID_FRAME = 0
	uint8 VEHICLE_BODY  = 1 #  Vehicle Origin
	uint8 CASE_BODY     = 2 #  Case Frame Origin
	uint8 GNSS_BODY     = 3 #  RF1 (Main) Antenna Reference Point. Attitude defined by the RF1-RF2 baseline heading.
	uint8 IMU_BODY      = 4 #  IMU frame - the IMU can be rotated in the INS case
