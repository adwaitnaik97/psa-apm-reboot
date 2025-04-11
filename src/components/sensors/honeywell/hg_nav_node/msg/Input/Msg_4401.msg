# 0x4401 : Position, Velocity, Attitude Input Message

#
#  Input Position / Velocity / Attitude to the System. The `input_coordinate_frame` enumeration specifies the referenced frame.
#  Corresponding validity values has to be marked as 1 = valid for the System to use the value
#  sending a STDV <= 0 will result to reset to default value.
# 

# Variables declaration
	uint32 AddressId  # Start of Message
	uint32 MessageId  # Message ID
	uint32 MessageLength  # Message Length [Number of 32-bit Words]
	uint32 Checksum  # 32-bit CRC
	bool RequestACKNAKReply  # Request ACK/NAK to this message (0 = false | 1 = true)
	
	# If 1, then time of validity of the data will be obtained from the message time of reception.
 	# If 0, then the GPS time in the message will be used as the time of validity.
	bool TOV_Mode
	coordinate_frame_t input_coordinate_frame  # Position is referenced to the selected frame
	float64 solutionTov  # [s] data validity time (based on TOV_Mode)
	bool PositionValidity  # 1 = Use Input Position | 0 = Do not use position fields
	float64 Latitude  # [rad] Input Latitude position (WGS-84)
	float64 Longitude  # [rad] Input Longitude position (WGS-84)
	float64 Altitude  # [m] Input Altitude above ellipsoid (WGS-84)
	bool VelocityValidity  # 1 = Use Input Velocity | 0 = Do not use velocity fields
	float64 NorthVelocity  # [m/s] Input North velocity
	float64 EastVelocity  # [m/s] Input East velocity
	float64 DownVelocity  # [m/s] Input Down velocity
	bool AttitudeValidity  # 1 = Use Input Attitude | 0 = Do not use attitude fields
	float64 Roll  # [rad] Input Roll
	float64 Pitch  # [rad] Input Pitch
	float64 TrueHeading  # [rad] Input True Heading
	float32 PositionStdvNorth  # [m] STDV North, East, Down (passing 0 equals default)
	float32 PositionStdvEast  # [m] STDV North, East, Down (passing 0 equals default)
	float32 PositionStdvDown  # [m] STDV North, East, Down (passing 0 equals default)
	float32 VelocityStdvNorth  # [m/s] STDV North(passing 0 means default)
	float32 VelocityStdvEast  # [m/s] STDV East(passing 0 means default)
	float32 VelocityStdvDown  # [m/s] STDV Down (passing 0 means default)
	float32 AttitudeStdvRoll  # [rad] STDV Roll (passing 0 means default)
	float32 AttitudeStdvPitch  # [rad] STDV Pitch (passing 0 means default)
	float32 AttitudeStdvTrueHeading  # [rad] STDV True Heading (passing 0 means default)
