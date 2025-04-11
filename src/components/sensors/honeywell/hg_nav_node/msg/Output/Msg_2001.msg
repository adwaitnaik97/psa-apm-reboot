# 0x2001 : INS Configuration

#
#  Message contains a complete configuration of the device.
#  Use message 0x6001 for information on SN and SW versions
#  Use the 0x1001 message to leave this on at 0.1 Hz (enabled; default), or to have this only produced and sent once (disabled).
# 

# Variables declaration
	uint32 AddressId  # Start of Message
	uint32 MessageId  # Message ID
	uint32 MessageLength  # Message Length
	uint32 Checksum  # Checksum
	uint8[8] IMUSerialNumber  # INS Device Serial Number
	uint8[8] ISA_Performance_Grade  # INS Performance Grade
	uint8[16] IMUSoftwareVersion  # IMU Software Version
	uint8[16] FPGA_VERSION  # FPGA
	uint8[16] FSBL_VERSION  # First Stage Bootloader
	uint8[16] SSBL_VERSION  # Second Stage Bootloader
	uint8[16] HGuideSoftwareVersion  # Application Software Version
	uint8[16] HGuideSoftwareVersionBuildDate  # Application Software Build Date
	uint8[16] HGuideSoftwareVersionBuildTime  # Application Software Build Time
	uint8[4] INS_CONFIG_FILEID  # INC Configuration File ID
	uint8[4] SSBL_REGINIT_FILEID  # Reginit File ID
	uint8[4] SSBL_FLASHLDR_FILEID  # Flash loader File ID
	uint8[10] HGuideSerialNumber  # RESERVED : Not populated
	uint8[10] HGuidePartNumber  # RESERVED : Not populated
	uint32 device_configuration 
	float32 CaseToNavLeverArmX  # [m] Case Origin to Nav Body Origin Lever Arm in Case Frame [x]
	float32 CaseToNavLeverArmY  # [m] Case Origin to Nav Body Origin Lever Arm in Case Frame [y]
	float32 CaseToNavLeverArmZ  # [m] Case Origin to Nav Body Origin Lever Arm in Case Frame [z]
	float32 CaseToNavQuaternionS  # Case Frame to Nav Body Quaternion [s]
	float32 CaseToNavQuaternionI  # Case Frame to Nav Body Quaternion [i]
	float32 CaseToNavQuaternionJ  # Case Frame to Nav Body Quaternion [j]
	float32 CaseToNavQuaternionK  # Case Frame to Nav Body Quaternion [k]
	float32 VehicleLeverArmX  # [m] Vehicle Origin to Case Origin Lever Arm in Vehicle Frame [x]
	float32 VehicleLeverArmY  # [m] Vehicle Origin to Case Origin Lever Arm in Vehicle Frame [y]
	float32 VehicleLeverArmZ  # [m] Vehicle Origin to Case Origin Lever Arm in Vehicle Frame [z]
	float32 VehicleQuaterionS  # Case Frame to Vehicle Frame Quaternion [s]
	float32 VehicleQuaterionI  # Case Frame to Vehicle Frame Quaternion [i]
	float32 VehicleQuaterionJ  # Case Frame to Vehicle Frame Quaternion [j]
	float32 VehicleQuaterionK  # Case Frame to Vehicle Frame Quaternion [k]
	float32 RF1AntennaLeverArmRSS  # Root Sum Squared of the Lever-arm estimated uncertainty. This will be used to determine if the lever-arm estimation function is on
	float32 RF1AntennaLeverArmX  # [m] Vehicle Origin to Integrated GPSR Main Antenna Origin Lever Arm in Vehicle Frame [x]
	float32 RF1AntennaLeverArmY  # [m] Vehicle Origin to Integrated GPSR Main Antenna Origin Lever Arm in Vehicle Frame [y]
	float32 RF1AntennaLeverArmZ  # [m] Vehicle Origin to Integrated GPSR Main Antenna Origin Lever Arm in Vehicle Frame [z]
	float32 HGNSI_AntennaLeverArmX  # RESERVED [m] Vehicle Origin to HGNSI GPSR Antenna Origin Lever Arm in HGNSI frame [x]
	float32 HGNSI_AntennaLeverArmY  # RESERVED [m] Vehicle Origin to HGNSI GPSR Antenna Origin Lever Arm in HGNSI frame [y]
	float32 HGNSI_AntennaLeverArmZ  # RESERVED [m] Vehicle Origin to HGNSI GPSR Antenna Origin Lever Arm in HGNSI frame [z]
	float32 RF2AntennaLeverArmX  # [m] Vehicle Origin to Integrated GPSR Auxiliary Antenna Origin Lever Arm in Case Frame [x]
	float32 RF2AntennaLeverArmY  # [m] Vehicle Origin to Integrated GPSR Auxiliary Antenna Origin Lever Arm in Case Frame [y]
	float32 RF2AntennaLeverArmZ  # [m] Vehicle Origin to Integrated GPSR Auxiliary Antenna Origin Lever Arm in Case Frame [z]
	float32 GPSantToCaseQuaternionS  # GPS Antenna to Integrated Case Frame Quaternion [s]
	float32 GPSantToCaseQuaternionI  # GPS Antenna to Integrated Case Frame Quaternion [i]
	float32 GPSantToCaseQuaternionJ  # GPS Antenna to Integrated Case Frame Quaternion [j]
	float32 GPSantToCaseQuaternionK  # GPS Antenna to Integrated Case Frame Quaternion [k]
