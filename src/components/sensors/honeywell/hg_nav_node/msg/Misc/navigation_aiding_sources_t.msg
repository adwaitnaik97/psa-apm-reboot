# Navigation sources for the INS
#
# A custom data type defining the possible sources of navigation data
# the Navigation sources 32-bit word .
#
# custom navigation_aiding_sources_t
	bool Enable_GNSS_Psuedorange  # 1 : Enable GNSS Psuedorange Measurement
	bool Enable_GNSS_Deltarange  # 2 : Enable GNSS Deltarange Measurement
	bool Enable_GNSS_Code_Delay  # 3 : Enable GNSS Code Delay Measurement
	bool Enable_GNSS_Carrier_Frequency  # 4 : Enable GNSS Carrier Frequency Measurement
	bool Enable_GNSS_Velocity  # 5 : Enable GNSS Velocity Measurement
	bool Enable_GNSS_Position  # 6 : Enable GNSS Position Measurement
	bool Enable_GNSS_Attitude  # 7 : Enable GNSS Attitude Measurement
	bool Enable_Attitude_Reference  # 8 : Enable Attitude Reference Measurement
	bool Enable_Velocity_Reference  # 9 : Enable Velocity Reference Measurement - Needs to be activated in order for Body Velocity aiding via 0x4110 message to work
	bool Enable_Position_Reference  # 10 : Enable Position Reference Measurement
	bool Enable_ZUPT  # 11 : Enable Zero Velocity Update (ZUPT) Measurement - instantaneous activation
	bool Enable_Zero_Heading_Change  # 12 : Enable Zero Heading Change Measurement - instantaneous activation
	bool Enable_Barometric_Altitude  # 13 : Enable Barometric Altitude Measurement
	bool Enable_Odometer  # 15 : Enable Odometer Measurement
	bool Enable_Magnetic_Heading  # 16 : Enable Magnetic Heading Measurement
	bool Enable_Automotive_Land_Car_Profile  # 18 : Enable Enable Automotive Land Car Profile - Enable_Odometer must be activated for Land Car Profile to work
	bool Enable_Motion_Detect  # 20 : Enable Motion Detection - recommendation is to send 2 consecutive 0x1002 messages - first with deactivated motion detection and second with activated motion detection. This forces the MD system to update it's aiding sources properly
	bool Enable_Aiding_Source  # 31 : Data Change Switch (1 = Use the settings in this list | 0 = Do not use the list settings)
