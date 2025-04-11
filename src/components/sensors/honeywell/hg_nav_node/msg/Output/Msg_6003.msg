# 0x6003 : HGuide Sensor Installation Settings

#
#  Sensor installation attitudes and lever arms, all in one message
#  this message is used in the HGuide Data Reader Guided setup
# 

# Variables declaration
	uint32 AddressId  # Start of Message
	uint32 MessageId  # Message ID
	uint32 MessageLength  # Message Length
	uint32 Checksum  # Checksum
	float64 systemTov  # [sec] System Time of Validity
	float64 gpsTov  # [sec] GPS Time of Validity in current week (Sunday 00:00 UTC). When invalid, GPS time = INS/GPS Internal System Time
	ins_gnss_summary_t InsGnssSummary  # INS / GNSS Summary Word
	float32 ImuLeverArmsX  # [m] lever_arm-x CASE_BODY to IMU_BODY in CASE_BODY frame
	float32 ImuLeverArmsY  # [m] lever_arm-y CASE_BODY to IMU_BODY in CASE_BODY frame
	float32 ImuLeverArmsZ  # [m] lever_arm-z CASE_BODY to IMU_BODY in CASE_BODY frame
	float32 VehicleEulerAnglesRoll  # RESERVED: [rad] VEHICLE_BODY to CASE installation attitude - roll
	float32 VehicleEulerAnglesPitch  # RESERVED: [rad] VEHICLE_BODY to CASE installation attitude - pitch
	float32 VehicleEulerAnglesTrueHeading  # RESERVED: [rad] VEHICLE_BODY to CASE installation attitude - heading
	float32 CaseToNavQuaterionS  # CASE to VEHICLE_BODY installation attitude quaternion [S]
	float32 CaseToNavQuaterionI  # CASE to VEHICLE_BODY installation attitude quaternion [I]
	float32 CaseToNavQuaterionJ  # CASE to VEHICLE_BODY installation attitude quaternion [J]
	float32 CaseToNavQuaterionK  # CASE to VEHICLE_BODY installation attitude quaternion [K]
	float32 RF1AntennaLeverArmStdv  # [m] GNSS RF #1 Antenna Lever-arm Estimate Uncertainty RSS
	float32 RF1AntennaLeverArmX  # [m] CASE_BODY to GNSS RF #1 Antenna in VEHICLE_BODY lever_arm-x
	float32 RF1AntennaLeverArmY  # [m] CASE_BODY to GNSS RF #1 Antenna in VEHICLE_BODY lever_arm-y
	float32 RF1AntennaLeverArmZ  # [m] CASE_BODY to GNSS RF #1 Antenna in VEHICLE_BODY lever_arm-z
	float32 RF2AuxiliaryAntennaLeverArmX  # [m] CASE_BODY to GNSS RF #2 Antenna in CASE_BODY lever_arm-x
	float32 RF2AuxiliaryAntennaLeverArmY  # [m] CASE_BODY to GNSS RF #2 Antenna in CASE_BODY lever_arm-y
	float32 RF2AuxiliaryAntennaLeverArmZ  # [m] CASE_BODY to GNSS RF #2 Antenna in CASE_BODY lever_arm-z
	float32 RF12LOSPitchEst  # [rad] Pitch estimate from RF1 to RF2
	float32 RF12LOSTrueHeadingEst  # [rad] True Heading estimate from RF1 to RF2
	float32 AntennaBoresightPitchEstStdv  # [rad] Dual Antenna Pitch boresight from VEHICLE_BODY frame
	float32 AntennaBoresightTrueHeadingEstStdv  # [rad] Dual Antenna True Heading boresight from VEHICLE_BODY frame
	float32 RF1MainAntennaLeverArmEstX  # [m] Estimated Lever Arm X of RF1 antenna in CASE_BODY frame
	float32 RF1MainAntennaLeverArmEstY  # [m] Estimated Lever Arm Y of RF1 antenna in CASE_BODY frame
	float32 RF1MainAntennaLeverArmEstZ  # [m] Estimated Lever Arm Z of RF1 antenna in CASE_BODY frame
	float32 RF1MainAntennaLeverArmEstStdvX  # [m] Estimated Lever Arm X STDV of RF1 antenna
	float32 RF1MainAntennaLeverArmEstStdvY  # [m] Estimated Lever Arm Y STDV of RF1 antenna
	float32 RF1MainAntennaLeverArmEstStdvZ  # [m] Estimated Lever Arm Z STDV of RF1 antenna
	float32 OdometerLeverArmEstX  # [m] Estimated Lever Arm X of DMI in VEHICLE_BODY frame
	float32 OdometerLeverArmEstY  # [m] Estimated Lever Arm Y of DMI in VEHICLE_BODY frame
	float32 OdometerLeverArmEstZ  # [m] Estimated Lever Arm Z of DMI in VEHICLE_BODY frame
	float32 OdometerLeverArmEstStdvX  # [m] Estimated Lever Arm X STDV of DMI
	float32 OdometerLeverArmEstStdvY  # [m] Estimated Lever Arm Y STDV of DMI
	float32 OdometerLeverArmEstStdvZ  # [m] Estimated Lever Arm Z STDV of DMI
	float32 OdometerBoresightTrueHeading  # [m] Estimated DMI True Heading Boresight from VEHICLE_BODY frame
	float32 OdometerBoresightPitch  # [m] Estimated DMI Pitch Boresight from VEHICLE_BODY frame
	float32 OdometerSFStdv  # Estimated DMI Scale Factor Correction STDV
	float32 OdometerLeverArmStoredX  # [m] Stored Lever Arm X of DMI in VEHICLE_BODY frame
	float32 OdometerLeverArmStoredY  # [m] Stored Lever Arm Y of DMI in VEHICLE_BODY frame
	float32 OdometerLeverArmStoredZ  # [m] Stored Lever Arm Z of DMI in VEHICLE_BODY frame
	float32 RF12BoresightTrueHeadingAdjustment  # [rad] RF1 to RF2 True Heading boresight adjustement
	float32 RF12BoresightPitchAdjustment  # [rad] RF1 to RF2 Pitch boresight adjustement
