# DVL Status bit fields (1=valid)
# taken over directly from the Nortek DVL message
# custom dvl_track_status_t
	bool beam1_velocityValid  # (0 = Invalid | 1 = Valid) Velocity validity from Beam 1
	bool beam2_velocityValid  # (0 = Invalid | 1 = Valid) Velocity validity from Beam 2
	bool beam3_velocityValid  # (0 = Invalid | 1 = Valid) Velocity validity from Beam 3
	bool beam4_velocityValid  # (0 = Invalid | 1 = Valid) Velocity validity from Beam 4
	bool beam1_distanceValid  # (0 = Invalid | 1 = Valid) Distance validity from Beam 1
	bool beam2_distanceValid  # (0 = Invalid | 1 = Valid) Distance validity from Beam 2
	bool beam3_distanceValid  # (0 = Invalid | 1 = Valid) Distance validity from Beam 3
	bool beam4_distanceValid  # (0 = Invalid | 1 = Valid) Distance validity from Beam 4
	bool beam1_fomValid  # (0 = Invalid | 1 = Valid) Figure of Merit (FOM) validity for Beam 1
	bool beam2_fomValid  # (0 = Invalid | 1 = Valid) Figure of Merit (FOM) validity for Beam 2
	bool beam3_fomValid  # (0 = Invalid | 1 = Valid) Figure of Merit (FOM) validity for Beam 3
	bool beam4_fomValid  # (0 = Invalid | 1 = Valid) Figure of Merit (FOM) validity for Beam 4
	bool x_velocityValid  # (0 = Invalid | 1 = Valid) X axis velocity validity
	bool y_velocityValid  # (0 = Invalid | 1 = Valid) Y axis velocity validity
	bool z1_velocityValid  # (0 = Invalid | 1 = Valid) Z axis 1 velocity validity
	bool z2_velocityValid  # (0 = Invalid | 1 = Valid) Z axis 2 velocity validity
	bool x_fomValid  # (0 = Invalid | 1 = Valid) X axis Figure of Merit (FOM) validity
	bool y_fomValid  # (0 = Invalid | 1 = Valid) Y axis Figure of Merit (FOM) validity
	bool z1_fomValid  # (0 = Invalid | 1 = Valid) Z axis 1 Figure of Merit (FOM) validity
	bool z2_fomValid  # (0 = Invalid | 1 = Valid) Z axis 2 Figure of Merit (FOM) validity
	
	# value indicates excesive DVL processor load
 	# (7) 111 < 3% dvl processing capicity left
 	# (6) 110 < 6% dvl processing capicity left
 	# (4) 100 < 12% dvl processing capicity left
	uint8 cpu_load
	
	# 0010=break
 	# 0011=rtc alarm
 	# 0000=bad power
 	# 0001=power applied
	uint8 wakeup_state
