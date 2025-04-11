# Odometer/Body Axis Velocity Status
#
# A custom data type which should be used to define a message which contains
# an Odometer/Body Axis Velocity Status 32-bit word.
#
# custom odo_body_vel_status_t
	bool Velocity_Valid  # Bit 0 : If 1, velocity is assumed to be valid.
	bool Odo_Pulse_Valid  # Bit 1 : If 1, odometry is assumed to be valid.
	bool TOV_Mode  # Bit 2 : If 1, then time of validity of the data will be obtained from the time of reception. If 0, then the GPS time in the message will be used as the time of validity.
	bool Vel_Sending_Unit_Status  # Bit 3 : If 1, then data in the message will be used.
	bool Zupt_Requested  # Bit 4 : If 1, then ZUPT measurements will be enabled.
