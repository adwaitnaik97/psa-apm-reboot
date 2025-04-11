# 0x2611 : Built In Test (BIT) History Output

#
#  When a 0x2611 message is requested the entire fault history 
#  stored in memory will be sent out via consecutive 0x2611
#  messages.
# 

# Variables declaration
	uint32 AddressId  # Start of Message
	uint32 MessageId  # Message ID
	uint32 MessageLength  # Message Length
	uint32 Checksum  # Checksum
	uint32 Last_Message  # If equal to 1, then the last 0x2611 message to be sent. The entire BIT history has been provided.
	int32 messageNumber_a  # Fault history record #
	int32 faultID_a  # Fault ID
	int32 BITmode_a  # BIT mode when fault was logged.
	int32 systemMode_a  # System mode when fault was logged.
	uint32 powerCycleCount_a  # Power cycle count when fault was logged.
	int32 faultValue_a  # The value of the fault criteria when fault was logged.
	int32 channelSubsystem_a  # Not used
	float32 eti_a  # Elapsed Time when fault was logged.
	float32 angularRateX_a  # Angular rate when fault was logged.
	float32 angularRateY_a  # Angular rate when fault was logged.
	float32 angularRateZ_a  # Angular rate when fault was logged.
	float32 accelerationX_a  # Acceleration when fault was logged.
	float32 accelerationY_a  # Acceleration when fault was logged.
	float32 accelerationZ_a  # Acceleration when fault was logged.
	float32 velocityX_a  # Velocity when fault was logged.
	float32 velocityY_a  # Velocity when fault was logged.
	float32 velocityZ_a  # Velocity when fault was logged.
	float32 insTemperature_a  # Temperature when fault was logged.
	float32 altitude_a  # Altitude when fault was logged.
	int32 messageNumber_b  # Fault history record #
	int32 faultID_b  # Fault ID
	int32 BITmode_b  # BIT mode when fault was logged.
	int32 systemMode_b  # System mode when fault was logged.
	uint32 powerCycleCount_b  # Power cycle count when fault was logged.
	int32 faultValue_b  # The value of the fault criteria when fault was logged.
	int32 channelSubsystem_b  # Not used
	float32 eti_b  # Elapsed Time when fault was logged.
	float32 angularRateX_b  # Angular rate when fault was logged.
	float32 angularRateY_b  # Angular rate when fault was logged.
	float32 angularRateZ_b  # Angular rate when fault was logged.
	float32 accelerationX_b  # Acceleration when fault was logged.
	float32 accelerationY_b  # Acceleration when fault was logged.
	float32 accelerationZ_b  # Acceleration when fault was logged.
	float32 velocityX_b  # Velocity when fault was logged.
	float32 velocityY_b  # Velocity when fault was logged.
	float32 velocityZ_b  # Velocity when fault was logged.
	float32 insTemperature_b  # Temperature when fault was logged.
	float32 altitude_b  # Altitude when fault was logged.
