# Bit fields as defined from DVL integrator's guide.
# Record configuration bit field
# custom record_configuration_t
	bool pressureValid  # (0 = Invalid | 1 = Valid) pressure sensor value valid
	bool temperatureValid  # (0 = Invalid | 1 = Valid) temperature sensor value valid
	bool compassValid  # (0 = Invalid | 1 = Valid) compass sensor values valid
	bool tiltValid  # (0 = Invalid | 1 = Valid) tilt sensor values valid
	bool velocityIncluded  # (0 = NOT included | 1 = Included) velocity data included
	bool amplitudeIncluded  # (0 = NOT included | 1 = Included) amplitude data included
	bool correlationIncluded  # (0 = NOT included | 1 = Included) correlation data included
	bool altimeterIncluded  # (0 = NOT included | 1 = Included) altimeter data included
	bool altimeterRawIncluded  # (0 = NOT included | 1 = Included) altimeter raw data included
	bool astIncluded  # (0 = NOT included | 1 = Included) ast data included
	bool echoSounderIncluded  # (0 = NOT included | 1 = Included) echo sounder data included
	bool ahrsIncluded  # (0 = NOT included | 1 = Included) ahrs data included
	bool percentageGoodIncluded  # (0 = NOT included | 1 = Included) percentage good data included
	bool stdvIncluded  # (0 = NOT included | 1 = Included) std. dev. data included
