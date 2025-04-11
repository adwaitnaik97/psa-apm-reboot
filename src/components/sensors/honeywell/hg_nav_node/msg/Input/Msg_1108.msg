# 0x1108 : GPS PVT TM Aiding Input

#
#  0x1108 message is being used to input GNSS PVT measurements to the INS
# 

# Variables declaration
	uint32 AddressId  # Start of Message
	uint32 MessageId  # Message ID
	uint32 MessageLength  # Message Length [Number of 32-bit Words]
	uint32 Checksum  # 32-bit CRC
	bool RequestAckNak  # (0 = do not request ACK/NAK | 1 = request ACK/NAK)
	bool TimeReferenceMode  # gpsPvtTov Time of Reference (0 = TOV referenced to GPS Time | 1 = TOV referenced to timestamp of message receipt)
	bool UnfilteredPointSolution  # Solution Type (0 = filtered solution | 1 = unfiltered solution)
	uint8 GnssMode  # InputGNSS Mode (0 = standalone, 1 = sbas, 2 = differential, 3 = float rtk, 4 = fixed rtk, 5 = ppp terrastarc, 6 = ppp terrastarl)
	float64 gpsPvtTov  # [s] Message Time of validity
	bool PositionValidity  # Validity of the input position (0 = invalid | 1 = valid)
	bool PositionCoordinateFrame  # Input Coordinate Frame (0 = latitude/longitude/altitude | 1 = ecef [x, y, z])
	bool MslGeoidValidity  # NOT USED - Validity of input Geoid Type
	bool MslGeoidType  # NOT USED - Select input Geoid Type (0 = Altitude | 1 = Height)
	float64 Latitude  # [rad] Latitude in degrees #pi*2^-31
	float64 EcefPositionX  # [m] ecef x in meters
	float64 Longitude  # [rad] Longitude in degrees #pi*2^-31
	float64 EcefPositionY  # [m] ecef y in meters
	float64 Altitude  # [m] Altitude in meters
	float64 EcefPositionZ  # [m] ecef z in meters
	float64 MslGeoidAltitudeOrGeoidHeight  # NOT USED - [m] Geoid Height or Altitude selectable by MslGeoidType
	bool VelocityValidity  # Validity of the input velocity (0 = invalid | 1 = valid)
	bool VelocityCoordinateFrame  # Input Coordinate Frame (0 = north east down (ned) | 1 = ecef x y z)
	float64 VelocityNorthOrEcefVelocityX  # [m/s] North velocity or ecef x in meters per second
	float64 VelocityEastOrEcefVelocityY  # [m/s] East velocity or ecef y in meters per second
	float64 VelocityDownOrEcefVelocityZ  # [m/s] Down velocity or ecef z in meters per second
	bool LeapSecondValidity  # NOT USED
	bool ReceiverClockBiasErrorValidity  # NOT USED
	bool DopDataValidity  # Validity of the input Dilution of Precision (0 = invalid | 1 = valid)
	bool ConstellationChangeIndicator  # Indication whether constellation changed (0 = not changed | 1 = changed)
	bool EstimatedPositionErrorValidity  # Validity of the input position Error estimate (0 = invalid | 1 = valid)
	bool EstimatedVelocityErrorValidity  # Validity of the input velocity Error estimate (0 = invalid | 1 = valid)
	float32 LeapSeconds  # NOT USED - [s] Number of leap seconds
	float32 ReceiverClockBiasError  # NOT USED - [m] Receiver Clock bias error
	float32 Tdop  # Time dilution of precision (-1 = invalid)
	float32 Vdop  # Vertical dilution of precision (-1 = invalid)
	float32 Hdop  # Horizontal dilution of precision (-1 = invalid)
	float32 Pdop  # Position dilution of precision (-1 = invalid)
	float32 EstimatedVerticalPositionError  # Estimated vertical position error (1-sigma)
	float32 EstimatedHorizontalPositionError  # Estimated horizontal position error (1-sigma)
	float32 EstimatedVerticalVelocityError  # Estimated vertical velocity error (1-sigma)
	float32 EstimatedHorizontalVelocityError  # Estimated horizontal velocity error (1-sigma)
	uint32 Fom  # Figure of merit
	uint32 Tfom  # Coordinated universal time figure of merit
