# 0x2011 : INS Status Information

#
#  Detailed status of all sub systems in the INS.
# 

# Variables declaration
	uint32 AddressId  # Start of Message
	uint32 MessageId  # Message ID
	uint32 MessageLength  # Message Length in 32-bit Words
	uint32 Checksum  # Checksum
	uint32 inertial_data_source  # [INTERNAL] Inertial Data Source | 0 = Internal ISA | 1 = External Data
	uint8 navigation_initialization_status  # Navigation Initialization Status: 0 = Waiting for Nav Init Data | 1 = Nav Init Data Received
	uint8 navErrorModel  # Navigation Error Model: 0 = Undefined | 1 = Small Azimuth | 2 = Large Azimuth
	
	# [INTERNAL] INS Mode - Redundant Information
 	# 0 = Reserved
 	# 1 = Standby
 	# 2 = Coarse Level
 	# 3 = Unaided Navigation (Free Inertial)
 	# 4 = Aided Navigation (Blended GPS)
 	# 5 - 9 = Reserved
	uint8 INSMode
	ins_gnss_summary_t InsGnssSummary  # INS/GPS BIT Summary
	float64 systemTov  # Time since Power Up - Always Valid [sec]
	float64 gpsTov  # [sec] GPS Time of Validity in current week (Sunday 00:00 UTC). When invalid, GPS time = INS/GPS Internal System Time
	int32 gps_week  # Number of weeks since Jan 6, 1980 GPS Week = -1 indicates GPS Week Unknown/Invalid
	int32 power_cycle_count  # Power Cycle Count
	float64 eti  # [sec] Total Device Elapsed Time Indicator (ETI)
	float32 ins_device_temperature  # [deg C]
	
	# BIT Mode:
 	# 0 = Unknown
 	# 1 = PowerUp
 	# 2 = Continuous
 	# 3 - 11 = Internal
	int32 bitMode
	
	# INS / Blended Figure of Merit (FOM)
 	# Refer to 0x6403 Standard Deviation numbers for position accuarcy
 	# 0 : Reserved
 	# 1 : Less than 26 meters
 	# 2 : 26 - 50 meters
 	# 3 : 51 - 75 meters
 	# 4 : 76 - 100 meters
 	# 5 : 101 - 200 meters
 	# 6 : 201 - 500 meters
 	# 7 : 501 - 1000 meters
 	# 8 : 1001 - 5000 meters
 	# 9 : Greater than 5000 meters
	int32 ins_blended_fom
	
	# GPS Figure of Merit (FOM)
 	# Refer to 0x6108 Standard Deviation numbers for RAW GNSS position accuarcy
 	# 0 : Reserved
 	# 1 : Less than 26 meters
 	# 2 : 26 - 50 meters
 	# 3 : 51 - 75 meters
 	# 4 : 76 - 100 meters
 	# 5 : 101 - 200 meters
 	# 6 : 201 - 500 meters
 	# 7 : 501 - 1000 meters
 	# 8 : 1001 - 5000 meters
 	# 9 : Greater than 5000 meters
	int32 gps_fom
	
	# UTC Time Figure of Merit (TFOM)
	# 0 : External Time source indicates normal operation
	# 1 : Time Accuracy <= 1 nsec
	# 2 : Time Accuracy <= 10 nsec
	# 3 : Time Accuracy <= 100 nsec
	# 4 : Time Accuracy <= 1 usec
	# 5 : Time Accuracy <= 10 usec
	# 6 : Time Accuracy <= 100 usec
	# 7 : Time Accuracy <= 1 msec
	# 8 : Time Accuracy <= 10 msec
	# 9 : Time Accuracy Unknown/Fault
	int32 utc_tfom
	uint8 MDT_stationary_instantaneous_signal  # [milli-radians] Motion Detect Threshold stationary instantaneous signal
	uint8 MDT_stationary_accel_mag  # [cm] Motion Detect Threshold stationary accel magnitude
	uint8 MDT_stationary_speed_stdv  # [m/s] Motion Detect Threshold stationary speed stdv
	uint8 MDT_stationary_rotation_rate  # [milli-radian] Motion Detect Threshold stationary rotation rate
	hgnsi_message_word1_t MessageWord1  # Word 1 with message enable bits
	hgnsi_message_word2_t MessageWord2  # Word 2 with message enable bits
	imu_bit_status_t imu_bit_status  # Internal IMU Status
	aiding_sources_t aiding_sources  # Enabled Aiding Sources
	
	# Number of Satellites Utilized by Pseudorange/Deltarange measurement filter (The number of satellites utilized over the last second).
 	# pseudorange/deltarange aiding source has to be enabled for this number to be greater than 0
	int32 number_of_satellites_in_blended_sol
	
	# Pseudo Range (PR) Validity by GPS Receiver Channel (PR Validity over the last second).
	# Valid only if PR aiding source is enabled
	gps_channel_validity_t pr_validity_by_gps_receiver_channel
	
	# Delta pseudo Range (PR) Validity by GPS Receiver Channel (DR Validity over the last second).
	# Valid only if PR aiding source is enabled
	gps_channel_validity_t dr_validity_by_gps_receiver_channel
	ssbl_status_t ssbl_status  # [INTERNAL] - Second Stage Boot Loader (SSBL) Status Word
	
	# Solution Convergence: 0 = Solution Not Converged | 1 = Solution Converged
 	# based on the FOM of position (<= 1) and attitude (<=4)
	uint32 solution_convergence
	
	# Attitude Figure of Merit (FOM)
	# 0 : Reserved
	# 1 : Accuracy <= 1 mrad (0.06 deg)
	# 2 : Accuracy <= 2 mrad (0.12 deg)
	# 3 : Accuracy <= 4 mrad (0.23 deg)
	# 4 : Accuracy <= 8 mrad (0.46 deg)
	# 5 : Accuracy <= 16 mrad (0.92 deg)
	# 6 : Accuracy <= 32 mrad (1.84 deg)
	# 7 : Accuracy <= 64 mrad (3.67 deg)
	# 8 : Accuracy <= 128 mrad (7.34 deg)
	# 9 : Accuracy > 128 mrad (7.34 deg)
	int32 attitude_fom
	gnssr_aiding_status_t gnssr_aiding_status  # [INTERNAL] Status of GNSS Aiding sources
	fsbl_status_t fsbl_status  # [INTERNAL] First Stage Boot Loader (FSBL) Status Word 1
	system_bit_summary_t system_bit_summary  # System Built in Test status summary
	mcb_bit_status_t mcb_bit_status  # [INTERNAL] Mission Computer Board (MCB) Built in Test status word 1
	gps_bit_status_t gps_bit_status  # [INTERNAL] GPS Built in Test status
	gyro_bit_status_t gyro_bit_status  # [INTERNAL] Internal Gyroscope Built in Test status
	accel_bit_status_t accelerometer_bit_status  # [INTERNAL] Accelerometer status
	float32 accel_x_temperature_degC  # [degC] Temperature of X axis accel - not calibrated
	float32 accel_y_temperature_degC  # [degC] Temperature of Y axis accel - not calibrated
	float32 accel_z_temperature_degC  # [degC] Temperature of Z axis accel - not calibrated
	float32 gyro_x_temperature_degC  # [degC] Temperature of X axis gyro - not calibrated
	float32 gyro_y_temperature_degC  # [degC] Temperature of Y axis gyro - not calibrated
	float32 gyro_z_temperature_degC  # [degC] Temperature of Z axis gyro - not calibrated
	uint8 MDTC_zero_vel_meas_noiseX  # [cm] Motion Detect Time Constant zero velocity measurement noise in X axis
	uint8 MDTC_inv_nominal_signal_filter_fn  # [s] Motion Detect Time Constant inv nominal signal filter fn
	uint8 MDTC_instant_signal_filter_fn  # [Hz] Motion Detect Time Constant instant signal filter fn
	uint8 MDTC_settling_time  # [s] Motion Detect Time Constant settling time
