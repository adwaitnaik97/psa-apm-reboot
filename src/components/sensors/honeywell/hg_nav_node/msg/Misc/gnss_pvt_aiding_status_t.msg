# GNSS PVT aiding status
# custom gnss_pvt_aiding_status_t
	uint8 latitude  # 0 = Source Disabled | 1 = Invalid | 2 = Rejected | 3 = Successfully Processed
	uint8 longitude  # 0 = Source Disabled | 1 = Invalid | 2 = Rejected | 3 = Successfully Processed
	uint8 altitude  # 0 = Source Disabled | 1 = Invalid | 2 = Rejected | 3 = Successfully Processed
	uint8 velocity_north  # 0 = Source Disabled | 1 = Invalid | 2 = Rejected | 3 = Successfully Processed
	uint8 velocity_east  # 0 = Source Disabled | 1 = Invalid | 2 = Rejected | 3 = Successfully Processed
	uint8 velocity_down  # 0 = Source Disabled | 1 = Invalid | 2 = Rejected | 3 = Successfully Processed
