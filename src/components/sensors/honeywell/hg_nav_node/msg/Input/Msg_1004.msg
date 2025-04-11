# 0x1004 : Configure Initialization Input

#
#  DO NOT USE. This message is deprecated, use 0x4404, 0x4204 messages instead
# 

# Variables declaration
	uint32 AddressId  # Start of Message
	uint32 MessageId  # Message ID
	uint32 MessageLength  # Message Length
	uint32 Checksum  # Checksum
	bool ULV_GPSSigLatencyL1  # 0 = Use Default | 1 = Update
	bool ULV_GPSSigLatencyL2  # 0 = Use Default | 1 = Update
	
	# Configure Vehicle Frame Rotation
 	# 0 = No Change
 	# 1 = Given as DCM
 	# 2 = Given as Euler
	uint8 ULV_VehFrametoCaseFrame
	
	# Configure Vehicle Frame Lever Arms
	# 0 = No Change
	# 1 = Given as Vehicle to Case
	# 2 = Given as Case to Vehicle
	uint8 ULV_Vehicle_Case_FrameLeverArm
	
	# Configure Port4 to GPS Antenna Lever Arms
 	# 0 = No Change
 	# 1 = Given as Vehicle to Port4
 	# 2 = Given as Port4 to Vehicle
	uint8 ULV_LeverArmToPort4GPSAnt
	
	# Configure Measurement Frame Rotation
 	# 0 = No Change
 	# 1 = Given as DCM
 	# 2 = Given as Euler
	uint8 ULV_MeasFrametoVehFrame
	
	# Configure Measurement Frame Lever Arms
	# 0 = No Change
	# 1 = Given as Measurement to Nav Body in Vehicle Coordinates
	# 2 = Given as Measurement to Nav Body in Measurement Coordinates
	uint8 ULV_LeverArmToICDNav
	
	# Configure GPS Antenna Lever Arms
 	# 0 = No Change
 	# 1 = Given as Vehicle to ICD Body in Vehicle Coordinates
 	# 2 = Given as Vehicle to ICD Body in Measurement Coordinates
	uint8 ULV_LeverArmToICDGPSAntenna
	
	# Configure Location of Autopilot FC Data
 	# 0 = Use Default
 	# 1 = Use IMU Body Origin
 	# 2 = Use Case Frame Origin
 	# 3 = Use Vehicle Frame Origin
	uint8 ULV_LocationOfAutopilotFCData
	float32 L1SignalLatency 
	float32 L2SignalLatency 
	float32 Vehicle_to_Case_Roll  # [rad] Installation Roll Attitude Vehicle to Case
	float32 Vehicle_to_Case_C11  # [C11] Installation Attitude Vehicle to Case
	float32 Vehicle_to_Case_Pitch  # [rad] Installation Pitch Attitude Vehicle to Case
	float32 Vehicle_to_Case_C12  # [C12] Installation Attitude Vehicle to Case
	float32 Vehicle_to_Case_Heading  # [rad] Installation Heading Attitude Vehicle to Case
	float32 Vehicle_to_Case_C13  # [C13] Installation Attitude Vehicle to Case
	float32 Vehicle_to_Case_C21  # [C21] Installation Attitude Vehicle to Case
	float32 Vehicle_to_Case_C22  # [C22] Installation Attitude Vehicle to Case
	float32 Vehicle_to_Case_C23  # [C23] Installation Attitude Vehicle to Case
	float32 Vehicle_to_Case_C31  # [C31] Installation Attitude Vehicle to Case
	float32 Vehicle_to_Case_C32  # [C32] Installation Attitude Vehicle to Case
	float32 Vehicle_to_Case_C33  # [C33] Installation Attitude Vehicle to Case
	float32 VehCaseLvrArm_x  # [m] Vehicle to Case Lever arm [x]
	float32 VehCaseLvrArm_y  # [m] Vehicle to Case Lever arm [y]
	float32 VehCaseLvrArm_z  # [m] Vehicle to Case Lever arm [z]
	float32 VehGPSPort4LvrArm_x 
	float32 VehGPSPort4LvrArm_y 
	float32 VehGPSPort4LvrArm_z 
	float32 Meas_to_Vehicle_Roll  # [rad] Installation Roll Measurement to Vehicle as Euler Angles
	float32 Meas_to_Vehicle_C11  # [C11] Installation Attitude Measurement to Vehicle as Direction Cosine Matrix
	float32 Meas_to_Vehicle_Pitch  # [rad] Installation Pitch Measurement to Vehicle as Euler Angles
	float32 Meas_to_Vehicle_C12  # [C12] Installation Attitude Measurement to Vehicle as Direction Cosine Matrix
	float32 Meas_to_Vehicle_Heading  # [rad] Installation Heading Measurement to Vehicle as Euler Angles
	float32 Meas_to_Vehicle_C13  # [C13] Installation Attitude Measurement to Vehicle as Direction Cosine Matrix
	float32 Meas_to_Vehicle_C21  # [C21] Installation Attitude Measurement to Vehicle as Direction Cosine Matrix
	float32 Meas_to_Vehicle_C22  # [C22] Installation Attitude Measurement to Vehicle as Direction Cosine Matrix
	float32 Meas_to_Vehicle_C23  # [C23] Installation Attitude Measurement to Vehicle as Direction Cosine Matrix
	float32 Meas_to_Vehicle_C31  # [C31] Installation Attitude Measurement to Vehicle as Direction Cosine Matrix
	float32 Meas_to_Vehicle_C32  # [C32] Installation Attitude Measurement to Vehicle as Direction Cosine Matrix
	float32 Meas_to_Vehicle_C33  # [C33] Installation Attitude Measurement to Vehicle as Direction Cosine Matrix
	float32 VehICDNavLvrArm_x  # [m] HGNSI to Navigation Body Lever arm [x]
	float32 VehICDNavLvrArm_y  # [m] HGNSI to Navigation Body Lever arm [y]
	float32 VehICDNavLvrArm_z  # [m] HGNSI to Navigation Body Lever arm [z]
	float32 VehGPSPort4LvrArm_x_2  # [m] Internal GPS to Navigation Body Lever arm [x]
	float32 VehGPSPort4LvrArm_y_2  # [m] Internal GPS to Navigation Body Lever arm [y]
	float32 VehGPSPort4LvrArm_z_2  # [m] Internal GPS to Navigation Body Lever arm [z]
