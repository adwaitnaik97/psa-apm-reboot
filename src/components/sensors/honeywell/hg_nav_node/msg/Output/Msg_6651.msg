# 0x6651 : Real Time Post Processing Good To Go

#
#  The 0x6651 message is used to indicate to the user if the post processed solution will be within the defined thresholds
#  assuming that the data logging is running from the start of the unit.
#  The message also provides status on the input measurements of IMU and GNSS subsystems
#  This check can be configured via 0x4651 input message.
# 

# Variables declaration
	uint32 AddressId  # Start of Message
	uint32 MessageId  # Message ID
	uint32 MessageLength  # Message Length
	uint32 Checksum  # Checksum
	data_status_t ImuDataStatus  # IMU Subsystem Data Status
	data_status_t GnssDatatStatus  # GNSS Subsystem Data Status
	threshold_status_t PositionStatus  # Information about Position Threshold Status
	threshold_status_t VelocityStatus  # Information about Velocity Threshold Status
	threshold_status_t AttitudeStatus  # Information about Attitude Threshold Status
	uint8 NavAxesSelected  # Navigation Axes Selector [0: Pos=NED, Vel=NED, Att=Euler | 1: Pos=ECEF, Vel=ECEF, Att=Gamma] same as 0x4651
