# 0x4438 : DMI configuration message

#
#  The message 0x4438 is used to configure the device for a specific odometer.
#       -  The lever arms must be measured be in the vehicle frame.
#       -  The vehicle frame must be in line with the odometer +Direction of Travel.
#       -  The lever arm distance is from the vehicle frame origin to the wheel center.
#       -  The odometer should be on the rear wheel of a four wheel vehicle.
#       -  Ideally, the HGuide device should be mounted centrally on the rear axles.
#       -  Lever arms are read by the navigation filter upon power up.  
#       -  Cycle power to ensure that the navigation filter is properly initialized.
#       -  Direct Quadrature input shares the HW lines with COM1 RS422 port and will disable it.
#  The default values for parameters listed in words 44 through 72 have been well tested with a
#  3600 pules per revolution quadrature rotary encoder sensor. Do not change. 
# 

# Variables declaration
	uint32 AddressId  # Start of Message
	uint32 MessageId  # Message ID
	uint32 MessageLength  # Message Length
	uint32 Checksum  # Checksum
	
	# Enable the Direct Quadrature (DMI) Input (0 = Disabled (Default) | 1 = Enabled)
 	# NOTE: Enabling the Quadrature Decoder will disable the COM1 RS422 port
	bool QdecEnable
	
	# [m] Distance per pulse from external Quadrature Encoder.
	# This value is used by the direct quadrature (DMI) input
	# QdecDistancePerPulse = wheelCircumference / numberOfPulsesPerRevolution
	float32 QdecDistancePerPulse
	float32 OdometerLeverArmX  # [m] X Lever Arm from Vehicle Frame
	float32 OdometerLeverArmY  # [m] Y Lever Arm from Vehicle Frame
	float32 OdometerLeverArmZ  # [m] Z Lever Arm from Vehicle Frame
	float32 OdometerMeasurementNoise  # [m/sample] Expected noise in measurement (STDV)
	float32 OdometerTheshold  # [sigma] Rejection sigma
	float32 OdometerInitialScaleFactorUncertainty  # [0..1] Initial Scale Factor Uncertainty
	float32 OdometerScaleFactorNoise  # [1/sqrt(hz)] Expected Scale Factor Noise
	float32 OdometerYawBoresightStdv  # [rad] Heading Error STDV
	float32 OdometerYawBoresightProcessNoise  # [rad/sqrt(hz)]
	float32 OdometerInitialPitchBoresightUncertainty  # [rad] Pitch Error STDV
	float32 OdometerInitialPitchBoresightProcessNoise  # [rad/sqrt(hz)]
	bool ChangeOdometerLeverArm  # 0 = Temporary | 1 = Flash (Saves Over Power Cyle)
