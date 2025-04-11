#ifndef __HGuideAPI_Msg_6723_h__
#define __HGuideAPI_Msg_6723_h__
#pragma once

#include <cstdint>

// Bit fields as defined from DVL integrator's guide.
// Record configuration bit field
struct record_configuration_t
{
	bool pressureValid; // (0 = Invalid | 1 = Valid) pressure sensor value valid
	bool temperatureValid; // (0 = Invalid | 1 = Valid) temperature sensor value valid
	bool compassValid; // (0 = Invalid | 1 = Valid) compass sensor values valid
	bool tiltValid; // (0 = Invalid | 1 = Valid) tilt sensor values valid
	bool velocityIncluded; // (0 = NOT included | 1 = Included) velocity data included
	bool amplitudeIncluded; // (0 = NOT included | 1 = Included) amplitude data included
	bool correlationIncluded; // (0 = NOT included | 1 = Included) correlation data included
	bool altimeterIncluded; // (0 = NOT included | 1 = Included) altimeter data included
	bool altimeterRawIncluded; // (0 = NOT included | 1 = Included) altimeter raw data included
	bool astIncluded; // (0 = NOT included | 1 = Included) ast data included
	bool echoSounderIncluded; // (0 = NOT included | 1 = Included) echo sounder data included
	bool ahrsIncluded; // (0 = NOT included | 1 = Included) ahrs data included
	bool percentageGoodIncluded; // (0 = NOT included | 1 = Included) percentage good data included
	bool stdvIncluded; // (0 = NOT included | 1 = Included) std. dev. data included

	void Default()
	{
		pressureValid = 0;
		temperatureValid = 0;
		compassValid = 0;
		tiltValid = 0;
		velocityIncluded = 0;
		amplitudeIncluded = 0;
		correlationIncluded = 0;
		altimeterIncluded = 0;
		altimeterRawIncluded = 0;
		astIncluded = 0;
		echoSounderIncluded = 0;
		ahrsIncluded = 0;
		percentageGoodIncluded = 0;
		stdvIncluded = 0;
	}
};

// DVL Error Status bit fields 
struct dvl_error_t
{
	bool data_retrieval_fifo; // (1=error, 0=ok) data retrieval fifo error
	bool data_retrieval_overflow; // (1=error, 0=ok) data retrieval overflow
	bool data_retrieval_overflow2; // (1=error, 0=ok) data retrieval overflow
	bool data_retrieval_underrun; // (1=error, 0=ok) data retrieval underrun
	bool data_retrieval_missing_samples; // (1=error, 0=ok) data retrieval samples missing
	bool sensor_read_failure; // (1=error, 0=ok) sensor read failure
	bool beam0_in_phase_tag_error; // (1=error, 0=ok) tag error beam 0 (in-phase)
	bool beam0_quad_phase_tag_error; // (1=error, 0=ok) tag error beam 0 (quadrature-phase)
	bool beam1_in_phase_tag_error; // (1=error, 0=ok) tag error beam 1 (in-phase)
	bool beam1_quad_phase_tag_error; // (1=error, 0=ok) tag error beam 1 (quadrature-phase)
	bool beam2_in_phase_tag_error; // (1=error, 0=ok) tag error beam 2 (in-phase)
	bool beam2_quad_phase_tag_error; // (1=error, 0=ok) tag error beam 2 (quadrature-phase)
	bool beam3_in_phase_tag_error; // (1=error, 0=ok) tag error beam 3 (in-phase)
	bool beam3_quad_phase_tag_error; // (1=error, 0=ok) tag error beam 3 (quadrature-phase)

	void Default()
	{
		data_retrieval_fifo = 0;
		data_retrieval_overflow = 0;
		data_retrieval_overflow2 = 0;
		data_retrieval_underrun = 0;
		data_retrieval_missing_samples = 0;
		sensor_read_failure = 0;
		beam0_in_phase_tag_error = 0;
		beam0_quad_phase_tag_error = 0;
		beam1_in_phase_tag_error = 0;
		beam1_quad_phase_tag_error = 0;
		beam2_in_phase_tag_error = 0;
		beam2_quad_phase_tag_error = 0;
		beam3_in_phase_tag_error = 0;
		beam3_quad_phase_tag_error = 0;
	}
};

//
// DVL Status bit fields
struct dvl_status_t
{
	bool bdScaling; // indicates cm scaling of blanking distance (0 = mm scaling | 1 = cm scaling)
	uint8_t echoFrequency; // echo sounder frequency bin
	bool boostRunning; // boost running (0 = not running | 1 = running)
	bool telemetryData; // telmetry data ? (0 = not available | 1 = available)
	uint8_t echoIndex; // echo sounder index
	bool configurationActive; // active configuration ? (0 = not active | 1 = active)
	bool lastMeasVoltage; // last measurement voltage [0=normal | 1=last measurement skipped due to low input voltage]
	
	// previous wakeup state
 	// (0) 0000=bad power
 	// (1) 0001=power applied
 	// (2) 0010=break
 	// (3) 0011=rtc alarm
	uint8_t prevWakeup;
	uint8_t orientationAuto; // auto orientation
	uint8_t orientation; // orientation
	
	// wakeup state
 	// (0) 0000=bad power
 	// (1) 0001=power applied
 	// (2) 0010=break
 	// (3) 0011=rtc alarm
	uint8_t wakeup;

	void Default()
	{
		bdScaling = 0;
		echoFrequency = 0;
		boostRunning = 0;
		telemetryData = 0;
		echoIndex = 0;
		configurationActive = 0;
		lastMeasVoltage = 0;
		prevWakeup = 0;
		orientationAuto = 0;
		orientation = 0;
		wakeup = 0;
	}
};


// 0x6723 : Nortek Current Profile Data
//
// Non-variable length current profile data.
// Reference Nortek DVL binary data format DF23.
// 
class HGUIDE_DLL Msg_6723
{
public:
	Msg_6723();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 100;}

public:
	static const uint32_t AddressId = 0xA5C381FF; // Start of message flag
	static const uint32_t MessageId = 0x6723; // Message ID
	static const uint32_t MessageLength = 25; // Message length in 32-bit word
	uint32_t Checksum; // Checksum
	double systemTov; // [s] Time of validity - trigger/ping time of message ensemble
	uint8_t version; // DVL Version
	uint32_t serialNumber; // DVL Serial Number
	uint8_t offsetOfData; // NOT APPLICABLE to this message. Number of bytes from start of data from Nortek DVL msg format.
	record_configuration_t profileConfiguration; // Current profile record configuration bit field
	uint8_t year; // Years since 1900
	uint8_t month; // Month, zero based (Jan = 0, Feb = 1, etc.)
	uint8_t day; // Day
	uint8_t hour; // [hr] Hour
	uint8_t minute; // [min] Minute
	uint8_t seconds; // [sec] Seconds
	uint16_t microseconds; // [usec] Microseconds (raw value in 100 usec)
	float speedOfSounds; // [m/s] Speed of sound
	float temperature; // [deg C] Temperature
	float pressure; // [dBar] Pressure
	float heading; // [deg] Heading
	float pitch; // [deg] Pitch
	float roll; // [deg] Roll
	uint16_t numCells; // number of cells
	
	// coordinate system type
 	// (1) b01=XYZ
 	// (2) b10=beam
 	// (3) b11=not defined
	uint8_t coorinateSystem;
	uint8_t numBeams; // number of beams
	uint16_t cellSize; // [mm] Cell size
	uint16_t blanking; // [mm] Blanking
	uint8_t nomCorrelation; // Nominal correlation for configured combination of cell size and velocity range in %
	float tempPressureSensor; // Temperature pressure sensor in degree Celsius; t = (val/5) - 4.0
	float batteryVoltage; // Battery voltage in volts
	int16_t magX; // Magnetometer Raw X Axis
	int16_t magY; // Magnetometer Raw Y Axis
	int16_t magZ; // Magnetometer Raw Z Axis
	int16_t accelX; // Accel X Axis (16384 = 1.0)
	int16_t accelY; // Accel Y Axis (16384 = 1.0)
	int16_t accelZ; // Accel Z Axis (16384 = 1.0)
	uint16_t ambiguityVelocity; // Ambiguity velocity, corrected for sound velocity (10^ velocity scaling) m/s.
	uint8_t dataSet1_beam; // physical beam used for 1st data set
	uint8_t dataSet2_beam; // physical beam used for 2nd data set
	uint8_t dataSet3_beam; // physical beam used for 3rd data set
	uint8_t dataSet4_beam; // physical beam used for 4th data set
	uint16_t transmitEnergy; // Transmit Energy
	int8_t velocityScaling; // Velocity scaling
	int8_t powerLevel; // [dB] Power level
	int16_t tempMag; // Temperature of magnetometer (not calibrated)
	float tempRtc; // Temperature real time clock in degree Celsius
	dvl_error_t error; // Error bitfield
	
	// value indicates excesive DVL processor load
	// (7) 111 < 3% dvl processing capicity left
	// (6) 110 < 6% dvl processing capicity left
	// (4) 100 < 12% dvl processing capicity left
	uint8_t cpu_load;
	bool status0_used; // (1 = status0 fields should be processed | 0 = status0 bitfiled not used)
	dvl_status_t status; // DVL Status
	uint32_t ensembleCounter; // Counts the number of ensembles in both averaged and burst data
};

#endif // __HGuideAPI_Msg_6723_h__
