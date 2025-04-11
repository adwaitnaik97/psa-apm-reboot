#
# DVL Status bit fields
# custom dvl_status_t
	bool bdScaling  # indicates cm scaling of blanking distance (0 = mm scaling | 1 = cm scaling)
	uint8 echoFrequency  # echo sounder frequency bin
	bool boostRunning  # boost running (0 = not running | 1 = running)
	bool telemetryData  # telmetry data ? (0 = not available | 1 = available)
	uint8 echoIndex  # echo sounder index
	bool configurationActive  # active configuration ? (0 = not active | 1 = active)
	bool lastMeasVoltage  # last measurement voltage [0=normal | 1=last measurement skipped due to low input voltage]
	
	# previous wakeup state
 	# (0) 0000=bad power
 	# (1) 0001=power applied
 	# (2) 0010=break
 	# (3) 0011=rtc alarm
	uint8 prevWakeup
	uint8 orientationAuto  # auto orientation
	uint8 orientation  # orientation
	
	# wakeup state
 	# (0) 0000=bad power
 	# (1) 0001=power applied
 	# (2) 0010=break
 	# (3) 0011=rtc alarm
	uint8 wakeup
