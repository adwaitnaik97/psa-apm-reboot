# Transfer Align aiding status
# custom ta_aiding_status_t
	uint8 position_ecef_X  # 0 = Source Disabled | 1 = Invalid | 2 = Rejected | 3 = Successfully Processed
	uint8 position_ecef_Y  # 0 = Source Disabled | 1 = Invalid | 2 = Rejected | 3 = Successfully Processed
	uint8 position_ecef_Z  # 0 = Source Disabled | 1 = Invalid | 2 = Rejected | 3 = Successfully Processed
	uint8 velocity_ecef_X  # 0 = Source Disabled | 1 = Invalid | 2 = Rejected | 3 = Successfully Processed
	uint8 velocity_ecef_Y  # 0 = Source Disabled | 1 = Invalid | 2 = Rejected | 3 = Successfully Processed
	uint8 velocity_ecef_Z  # 0 = Source Disabled | 1 = Invalid | 2 = Rejected | 3 = Successfully Processed
	uint8 attitude_roll  # 0 = Source Disabled | 1 = Invalid | 2 = Rejected | 3 = Successfully Processed
	uint8 attitude_pitch  # 0 = Source Disabled | 1 = Invalid | 2 = Rejected | 3 = Successfully Processed
	uint8 attitude_heading  # 0 = Source Disabled | 1 = Invalid | 2 = Rejected | 3 = Successfully Processed
