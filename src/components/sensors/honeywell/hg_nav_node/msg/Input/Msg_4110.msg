# 0x4110 : Odometer Measurement Input Message

#
#  The message 0x4110 is used to provide DMI (odometer) or velocity measurements to the HGuide.
# 

# Variables declaration
	uint32 AddressId  # Start of Message
	uint32 MessageId  # Message ID
	uint32 MessageLength  # Message Length
	uint32 Checksum  # Checksum
	
	# [s] positive number to inform about a delay between sensor sampling and message transmission
	# (Maximum adjustment is 0.5s)
	float64 TimeOfValidityAdjustment
	float64 GPS_TOV  # [s] GPS time of validity (Seconds into week)
	float32 Body_Velocity_X  # Experimental Body Velocity X [m/s] maximum velocity of 200 m/s
	float32 Body_Velocity_Y  # Experimental Body Velocity Y [m/s] maximum velocity of 200 m/s
	float32 Body_Velocity_Z  # Experimental Body Velocity Z [m/s] maximum velocity of 200 m/s
	int32 Odo_Pulses  # The total cumulative sum of the number of pulses.
	float32 Distance_per_Pulse  # The distance traveled per pulse. [m/pulse]
	odo_body_vel_status_t Odo_Body_Vel_Status  # Configuration Bit field
	uint32 Message_Counter  # The total number of 0x4110 messages sent. The data will only be used when the message counter increments.
	float32 X_Y_Velocity_STDV  # Estimated velocity error in Body Velocity X and Y measurements. (1-sigma) [m/s]
	float32 Z_Velocity_STDV  # Estimated velocity error in Body Velocity Z measurements. (1-sigma) [m/s]
