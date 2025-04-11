# 0x1401 : Navigation Input Message for INS

#
#  Input Position / Velocity / Attitude to the INS
#  Corresponding validity values has to be marked as 1 = valid for the INS to use the value
#  sending a STDV <= 0 will result to reset to default value in the INS.
#  Read the detailed description below before usage. If unsure, consult with Honeywell.
# 

# Variables declaration
	uint32 AddressId  # Start of Message
	uint32 MessageId  # Message ID
	uint32 MessageLength  # Message Length [Number of 32-bit Words]
	uint32 Checksum  # 32-bit CRC
	bool RequestACKNAKReply  # Request ACK/NAK to this message (0 = false | 1 = true)
	float64 PositionTov  # [s] data validity time (based on PositionTimeReferenceMode)
	bool PositionValidity  # Validity of the input position (0 = invalid | 1 = valid)
	bool PositionTimeReferenceMode  # PositionTov Time of Reference (0 = TOV referenced to GPS Time | 1 = TOV referenced to timestamp of message receipt)
	bool PositionCoordinateFrame  # Input Position Coordinate Frame (0 = Latitude/Longitude/Altitude | 1 = ECEF [x,y,z])
	bool PositionStdvValidity  # Validity of the input position STDV (0 = invalid | 1 = valid)
	float32 Latitude  # [rad] Input Latitude position
	float32 EcefPositionX  # [m] Input ECEF X position
	float32 Longitude  # [rad] Input Longitude position
	float32 EcefPositionY  # [m] Input ECEF Y position
	float32 AltitudeHeightAboveEllipsoid  # [m] Input Altitude above ellipsoid
	float32 EcefPositionZ  # [m] Input ECEF Z position
	float64 VelocityTov  # data validity time (based on VelocityTimeReferenceMode)
	bool VelocityValidity  # Validity of the input velocity (0 = invalid | 1 = valid)
	bool VelocityTimeReferenceMode  # VelocityTov Time of Reference (0 = TOV referenced to GPS Time | 1 = TOV referenced to timestamp of message receipt)
	bool VelocityCoordinateFrame  # Input Velocity Coordinate Frame (0 = Latitude/Longitude/Altitude | 1 = ECEF [x,y,z])
	bool VelocityStdvValidity  # Validity of the input velocity STDV (0 = invalid | 1 = valid)
	float32 VelocityNorthOrEcefVelocityX  # [m/s] Input North velocity or ECEF X Velocity
	float32 VelocityEastOrEcefVelocityY  # [m/s] Input East velocity or ECEF Y Velocity
	float32 VelocityDownOrEcefVelocityZ  # [m/s] Input Down velocity or ECEF Z Velocity
	float64 AttitudeTov  # [s] data validity time (based on AttitudeTimeReferenceMode)
	bool AttitudeValidity  # Validity of the input attitude (0 = invalid | 1 = valid)
	bool AttitudeTimeReferenceMode  # AttitudeTov Time of Reference (0 = TOV referenced to GPS Time | 1 = TOV referenced to timestamp of message receipt)
	bool AttitudeCoordinateFrame  # Input Attitude Coordinate Frame (0 = DCM [matrix 3x3] | 1 = Roll/Pitch/Heading)
	bool AttitudeStdvValidity  # Validity of the input Attitude STDV (0 = invalid | 1 = valid)
	float32 EulerAnglesRoll  # [rad] Input Roll
	float32 DCM11  # DCM for Input Attitude [xx]
	float32 EulerAnglesPitch  # [rad] Input Pitch
	float32 DCM12  # DCM for Input Attitude [xy]
	float32 EulerAnglesTrueHeading  # [rad] Input True Heading
	float32 DCM13  # DCM for Input Attitude [xz]
	float32 DCM21  # DCM for Input Attitude [yx]
	float32 DCM22  # DCM for Input Attitude [yy]
	float32 DCM23  # DCM for Input Attitude [yz]
	float32 DCM31  # DCM for Input Attitude [zx]
	float32 DCM32  # DCM for Input Attitude [zy]
	float32 DCM33  # DCM for Input Attitude [zz]
	float32 PositionStdvNorth  # [m] STDV North, East, Down (passing 0 equals default = 100 m)
	float32 PositionStdvEast  # [m] STDV North, East, Down (passing 0 equals default = 100 m)
	float32 PositionStdvDown  # [m] STDV North, East, Down (passing 0 equals default = 100 m)
	float32 VelocityStdvNorth  # [m/s] STDV North(passing 0 means default = 1 m/s)
	float32 VelocityStdvEast  # [m/s] STDV East(passing 0 means default = 1 m/s)
	float32 VelocityStdvDown  # [m/s] STDV Down (passing 0 means default = 1 m/s)
	float32 EulerAnglesStdvRoll  # [rad] STDV Roll (passing 0 means default = 0.0873 rad)
	float32 EulerAnglesStdvPitch  # [rad] STDV Pitch (passing 0 means default = 0.0873 rad)
	float32 EulerAnglesStdvTrueHeading  # [rad] STDV True Heading (passing 0 means default = 0.0873 rad)
