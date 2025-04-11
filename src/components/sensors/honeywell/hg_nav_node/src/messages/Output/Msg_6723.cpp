#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_nav_node/Msg_6723.h>
#include <hg_nav_node/dvl_error_t.h>
#include <hg_nav_node/dvl_status_t.h>
#include <hg_nav_node/record_configuration_t.h>
hg_nav_node::Msg_6723 msgStruct_6723;

bool Msg_6723_pub_initialized = false;

ros::Publisher Msg_6723_pub;
void init_6723(ros::NodeHandle * n){
	Msg_6723_pub = n->advertise<hg_nav_node::Msg_6723>(MSG_6723_PATH, 5);
	Msg_6723_pub_initialized = true;
	ROS_INFO("Starting pub %s",MSG_6723_PATH);
	return;
}

void stop_6723(void){
	Msg_6723_pub.shutdown();
	Msg_6723_pub_initialized = false;
	ROS_INFO("0x6723 stopped");
	return;
}

// Msg_6723 to Topic
void convert(Msg_6723 messageIn, hg_nav_node::Msg_6723 * messageOut)
{
	messageOut->AddressId = messageIn.AddressId;
	messageOut->MessageId = messageIn.MessageId;
	messageOut->MessageLength = messageIn.MessageLength;
	messageOut->Checksum = messageIn.Checksum;
	messageOut->systemTov = messageIn.systemTov;
	messageOut->version = messageIn.version;
	messageOut->serialNumber = messageIn.serialNumber;
	messageOut->offsetOfData = messageIn.offsetOfData;

	messageOut->profileConfiguration.pressureValid = messageIn.profileConfiguration.pressureValid;
	messageOut->profileConfiguration.temperatureValid = messageIn.profileConfiguration.temperatureValid;
	messageOut->profileConfiguration.compassValid = messageIn.profileConfiguration.compassValid;
	messageOut->profileConfiguration.tiltValid = messageIn.profileConfiguration.tiltValid;
	messageOut->profileConfiguration.velocityIncluded = messageIn.profileConfiguration.velocityIncluded;
	messageOut->profileConfiguration.amplitudeIncluded = messageIn.profileConfiguration.amplitudeIncluded;
	messageOut->profileConfiguration.correlationIncluded = messageIn.profileConfiguration.correlationIncluded;
	messageOut->profileConfiguration.altimeterIncluded = messageIn.profileConfiguration.altimeterIncluded;
	messageOut->profileConfiguration.altimeterRawIncluded = messageIn.profileConfiguration.altimeterRawIncluded;
	messageOut->profileConfiguration.astIncluded = messageIn.profileConfiguration.astIncluded;
	messageOut->profileConfiguration.echoSounderIncluded = messageIn.profileConfiguration.echoSounderIncluded;
	messageOut->profileConfiguration.ahrsIncluded = messageIn.profileConfiguration.ahrsIncluded;
	messageOut->profileConfiguration.percentageGoodIncluded = messageIn.profileConfiguration.percentageGoodIncluded;
	messageOut->profileConfiguration.stdvIncluded = messageIn.profileConfiguration.stdvIncluded;
	messageOut->profileConfiguration.pressureValid = messageIn.profileConfiguration.pressureValid;
	messageOut->profileConfiguration.temperatureValid = messageIn.profileConfiguration.temperatureValid;
	messageOut->profileConfiguration.compassValid = messageIn.profileConfiguration.compassValid;
	messageOut->profileConfiguration.tiltValid = messageIn.profileConfiguration.tiltValid;
	messageOut->profileConfiguration.velocityIncluded = messageIn.profileConfiguration.velocityIncluded;
	messageOut->profileConfiguration.amplitudeIncluded = messageIn.profileConfiguration.amplitudeIncluded;
	messageOut->profileConfiguration.correlationIncluded = messageIn.profileConfiguration.correlationIncluded;
	messageOut->profileConfiguration.altimeterIncluded = messageIn.profileConfiguration.altimeterIncluded;
	messageOut->profileConfiguration.altimeterRawIncluded = messageIn.profileConfiguration.altimeterRawIncluded;
	messageOut->profileConfiguration.astIncluded = messageIn.profileConfiguration.astIncluded;
	messageOut->profileConfiguration.echoSounderIncluded = messageIn.profileConfiguration.echoSounderIncluded;
	messageOut->profileConfiguration.ahrsIncluded = messageIn.profileConfiguration.ahrsIncluded;
	messageOut->profileConfiguration.percentageGoodIncluded = messageIn.profileConfiguration.percentageGoodIncluded;
	messageOut->profileConfiguration.stdvIncluded = messageIn.profileConfiguration.stdvIncluded;
	messageOut->year = messageIn.year;
	messageOut->month = messageIn.month;
	messageOut->day = messageIn.day;
	messageOut->hour = messageIn.hour;
	messageOut->minute = messageIn.minute;
	messageOut->seconds = messageIn.seconds;
	messageOut->microseconds = messageIn.microseconds;
	messageOut->speedOfSounds = messageIn.speedOfSounds;
	messageOut->temperature = messageIn.temperature;
	messageOut->pressure = messageIn.pressure;
	messageOut->heading = messageIn.heading;
	messageOut->pitch = messageIn.pitch;
	messageOut->roll = messageIn.roll;
	messageOut->numCells = messageIn.numCells;
	messageOut->coorinateSystem = messageIn.coorinateSystem;
	messageOut->numBeams = messageIn.numBeams;
	messageOut->cellSize = messageIn.cellSize;
	messageOut->blanking = messageIn.blanking;
	messageOut->nomCorrelation = messageIn.nomCorrelation;
	messageOut->tempPressureSensor = messageIn.tempPressureSensor;
	messageOut->batteryVoltage = messageIn.batteryVoltage;
	messageOut->magX = messageIn.magX;
	messageOut->magY = messageIn.magY;
	messageOut->magZ = messageIn.magZ;
	messageOut->accelX = messageIn.accelX;
	messageOut->accelY = messageIn.accelY;
	messageOut->accelZ = messageIn.accelZ;
	messageOut->ambiguityVelocity = messageIn.ambiguityVelocity;
	messageOut->dataSet1_beam = messageIn.dataSet1_beam;
	messageOut->dataSet2_beam = messageIn.dataSet2_beam;
	messageOut->dataSet3_beam = messageIn.dataSet3_beam;
	messageOut->dataSet4_beam = messageIn.dataSet4_beam;
	messageOut->transmitEnergy = messageIn.transmitEnergy;
	messageOut->velocityScaling = messageIn.velocityScaling;
	messageOut->powerLevel = messageIn.powerLevel;
	messageOut->tempMag = messageIn.tempMag;
	messageOut->tempRtc = messageIn.tempRtc;

	messageOut->error.data_retrieval_fifo = messageIn.error.data_retrieval_fifo;
	messageOut->error.data_retrieval_overflow = messageIn.error.data_retrieval_overflow;
	messageOut->error.data_retrieval_overflow2 = messageIn.error.data_retrieval_overflow2;
	messageOut->error.data_retrieval_underrun = messageIn.error.data_retrieval_underrun;
	messageOut->error.data_retrieval_missing_samples = messageIn.error.data_retrieval_missing_samples;
	messageOut->error.sensor_read_failure = messageIn.error.sensor_read_failure;
	messageOut->error.beam0_in_phase_tag_error = messageIn.error.beam0_in_phase_tag_error;
	messageOut->error.beam0_quad_phase_tag_error = messageIn.error.beam0_quad_phase_tag_error;
	messageOut->error.beam1_in_phase_tag_error = messageIn.error.beam1_in_phase_tag_error;
	messageOut->error.beam1_quad_phase_tag_error = messageIn.error.beam1_quad_phase_tag_error;
	messageOut->error.beam2_in_phase_tag_error = messageIn.error.beam2_in_phase_tag_error;
	messageOut->error.beam2_quad_phase_tag_error = messageIn.error.beam2_quad_phase_tag_error;
	messageOut->error.beam3_in_phase_tag_error = messageIn.error.beam3_in_phase_tag_error;
	messageOut->error.beam3_quad_phase_tag_error = messageIn.error.beam3_quad_phase_tag_error;
	messageOut->error.data_retrieval_fifo = messageIn.error.data_retrieval_fifo;
	messageOut->error.data_retrieval_overflow = messageIn.error.data_retrieval_overflow;
	messageOut->error.data_retrieval_overflow2 = messageIn.error.data_retrieval_overflow2;
	messageOut->error.data_retrieval_underrun = messageIn.error.data_retrieval_underrun;
	messageOut->error.data_retrieval_missing_samples = messageIn.error.data_retrieval_missing_samples;
	messageOut->error.sensor_read_failure = messageIn.error.sensor_read_failure;
	messageOut->error.beam0_in_phase_tag_error = messageIn.error.beam0_in_phase_tag_error;
	messageOut->error.beam0_quad_phase_tag_error = messageIn.error.beam0_quad_phase_tag_error;
	messageOut->error.beam1_in_phase_tag_error = messageIn.error.beam1_in_phase_tag_error;
	messageOut->error.beam1_quad_phase_tag_error = messageIn.error.beam1_quad_phase_tag_error;
	messageOut->error.beam2_in_phase_tag_error = messageIn.error.beam2_in_phase_tag_error;
	messageOut->error.beam2_quad_phase_tag_error = messageIn.error.beam2_quad_phase_tag_error;
	messageOut->error.beam3_in_phase_tag_error = messageIn.error.beam3_in_phase_tag_error;
	messageOut->error.beam3_quad_phase_tag_error = messageIn.error.beam3_quad_phase_tag_error;
	messageOut->cpu_load = messageIn.cpu_load;
	messageOut->status0_used = messageIn.status0_used;

	messageOut->status.bdScaling = messageIn.status.bdScaling;
	messageOut->status.echoFrequency = messageIn.status.echoFrequency;
	messageOut->status.boostRunning = messageIn.status.boostRunning;
	messageOut->status.telemetryData = messageIn.status.telemetryData;
	messageOut->status.echoIndex = messageIn.status.echoIndex;
	messageOut->status.configurationActive = messageIn.status.configurationActive;
	messageOut->status.lastMeasVoltage = messageIn.status.lastMeasVoltage;
	messageOut->status.prevWakeup = messageIn.status.prevWakeup;
	messageOut->status.orientationAuto = messageIn.status.orientationAuto;
	messageOut->status.orientation = messageIn.status.orientation;
	messageOut->status.wakeup = messageIn.status.wakeup;
	messageOut->status.bdScaling = messageIn.status.bdScaling;
	messageOut->status.echoFrequency = messageIn.status.echoFrequency;
	messageOut->status.boostRunning = messageIn.status.boostRunning;
	messageOut->status.telemetryData = messageIn.status.telemetryData;
	messageOut->status.echoIndex = messageIn.status.echoIndex;
	messageOut->status.configurationActive = messageIn.status.configurationActive;
	messageOut->status.lastMeasVoltage = messageIn.status.lastMeasVoltage;
	messageOut->status.prevWakeup = messageIn.status.prevWakeup;
	messageOut->status.orientationAuto = messageIn.status.orientationAuto;
	messageOut->status.orientation = messageIn.status.orientation;
	messageOut->status.wakeup = messageIn.status.wakeup;
	messageOut->ensembleCounter = messageIn.ensembleCounter;
}

// Topic to Msg_6723
void convert(hg_nav_node::Msg_6723 messageIn, Msg_6723 * messageOut)
{
	messageOut->Checksum = messageIn.Checksum;
	messageOut->systemTov = messageIn.systemTov;
	messageOut->version = messageIn.version;
	messageOut->serialNumber = messageIn.serialNumber;
	messageOut->offsetOfData = messageIn.offsetOfData;

	messageOut->profileConfiguration.pressureValid = messageIn.profileConfiguration.pressureValid;
	messageOut->profileConfiguration.temperatureValid = messageIn.profileConfiguration.temperatureValid;
	messageOut->profileConfiguration.compassValid = messageIn.profileConfiguration.compassValid;
	messageOut->profileConfiguration.tiltValid = messageIn.profileConfiguration.tiltValid;
	messageOut->profileConfiguration.velocityIncluded = messageIn.profileConfiguration.velocityIncluded;
	messageOut->profileConfiguration.amplitudeIncluded = messageIn.profileConfiguration.amplitudeIncluded;
	messageOut->profileConfiguration.correlationIncluded = messageIn.profileConfiguration.correlationIncluded;
	messageOut->profileConfiguration.altimeterIncluded = messageIn.profileConfiguration.altimeterIncluded;
	messageOut->profileConfiguration.altimeterRawIncluded = messageIn.profileConfiguration.altimeterRawIncluded;
	messageOut->profileConfiguration.astIncluded = messageIn.profileConfiguration.astIncluded;
	messageOut->profileConfiguration.echoSounderIncluded = messageIn.profileConfiguration.echoSounderIncluded;
	messageOut->profileConfiguration.ahrsIncluded = messageIn.profileConfiguration.ahrsIncluded;
	messageOut->profileConfiguration.percentageGoodIncluded = messageIn.profileConfiguration.percentageGoodIncluded;
	messageOut->profileConfiguration.stdvIncluded = messageIn.profileConfiguration.stdvIncluded;
	messageOut->profileConfiguration.pressureValid = messageIn.profileConfiguration.pressureValid;
	messageOut->profileConfiguration.temperatureValid = messageIn.profileConfiguration.temperatureValid;
	messageOut->profileConfiguration.compassValid = messageIn.profileConfiguration.compassValid;
	messageOut->profileConfiguration.tiltValid = messageIn.profileConfiguration.tiltValid;
	messageOut->profileConfiguration.velocityIncluded = messageIn.profileConfiguration.velocityIncluded;
	messageOut->profileConfiguration.amplitudeIncluded = messageIn.profileConfiguration.amplitudeIncluded;
	messageOut->profileConfiguration.correlationIncluded = messageIn.profileConfiguration.correlationIncluded;
	messageOut->profileConfiguration.altimeterIncluded = messageIn.profileConfiguration.altimeterIncluded;
	messageOut->profileConfiguration.altimeterRawIncluded = messageIn.profileConfiguration.altimeterRawIncluded;
	messageOut->profileConfiguration.astIncluded = messageIn.profileConfiguration.astIncluded;
	messageOut->profileConfiguration.echoSounderIncluded = messageIn.profileConfiguration.echoSounderIncluded;
	messageOut->profileConfiguration.ahrsIncluded = messageIn.profileConfiguration.ahrsIncluded;
	messageOut->profileConfiguration.percentageGoodIncluded = messageIn.profileConfiguration.percentageGoodIncluded;
	messageOut->profileConfiguration.stdvIncluded = messageIn.profileConfiguration.stdvIncluded;
	messageOut->year = messageIn.year;
	messageOut->month = messageIn.month;
	messageOut->day = messageIn.day;
	messageOut->hour = messageIn.hour;
	messageOut->minute = messageIn.minute;
	messageOut->seconds = messageIn.seconds;
	messageOut->microseconds = messageIn.microseconds;
	messageOut->speedOfSounds = messageIn.speedOfSounds;
	messageOut->temperature = messageIn.temperature;
	messageOut->pressure = messageIn.pressure;
	messageOut->heading = messageIn.heading;
	messageOut->pitch = messageIn.pitch;
	messageOut->roll = messageIn.roll;
	messageOut->numCells = messageIn.numCells;
	messageOut->coorinateSystem = messageIn.coorinateSystem;
	messageOut->numBeams = messageIn.numBeams;
	messageOut->cellSize = messageIn.cellSize;
	messageOut->blanking = messageIn.blanking;
	messageOut->nomCorrelation = messageIn.nomCorrelation;
	messageOut->tempPressureSensor = messageIn.tempPressureSensor;
	messageOut->batteryVoltage = messageIn.batteryVoltage;
	messageOut->magX = messageIn.magX;
	messageOut->magY = messageIn.magY;
	messageOut->magZ = messageIn.magZ;
	messageOut->accelX = messageIn.accelX;
	messageOut->accelY = messageIn.accelY;
	messageOut->accelZ = messageIn.accelZ;
	messageOut->ambiguityVelocity = messageIn.ambiguityVelocity;
	messageOut->dataSet1_beam = messageIn.dataSet1_beam;
	messageOut->dataSet2_beam = messageIn.dataSet2_beam;
	messageOut->dataSet3_beam = messageIn.dataSet3_beam;
	messageOut->dataSet4_beam = messageIn.dataSet4_beam;
	messageOut->transmitEnergy = messageIn.transmitEnergy;
	messageOut->velocityScaling = messageIn.velocityScaling;
	messageOut->powerLevel = messageIn.powerLevel;
	messageOut->tempMag = messageIn.tempMag;
	messageOut->tempRtc = messageIn.tempRtc;

	messageOut->error.data_retrieval_fifo = messageIn.error.data_retrieval_fifo;
	messageOut->error.data_retrieval_overflow = messageIn.error.data_retrieval_overflow;
	messageOut->error.data_retrieval_overflow2 = messageIn.error.data_retrieval_overflow2;
	messageOut->error.data_retrieval_underrun = messageIn.error.data_retrieval_underrun;
	messageOut->error.data_retrieval_missing_samples = messageIn.error.data_retrieval_missing_samples;
	messageOut->error.sensor_read_failure = messageIn.error.sensor_read_failure;
	messageOut->error.beam0_in_phase_tag_error = messageIn.error.beam0_in_phase_tag_error;
	messageOut->error.beam0_quad_phase_tag_error = messageIn.error.beam0_quad_phase_tag_error;
	messageOut->error.beam1_in_phase_tag_error = messageIn.error.beam1_in_phase_tag_error;
	messageOut->error.beam1_quad_phase_tag_error = messageIn.error.beam1_quad_phase_tag_error;
	messageOut->error.beam2_in_phase_tag_error = messageIn.error.beam2_in_phase_tag_error;
	messageOut->error.beam2_quad_phase_tag_error = messageIn.error.beam2_quad_phase_tag_error;
	messageOut->error.beam3_in_phase_tag_error = messageIn.error.beam3_in_phase_tag_error;
	messageOut->error.beam3_quad_phase_tag_error = messageIn.error.beam3_quad_phase_tag_error;
	messageOut->error.data_retrieval_fifo = messageIn.error.data_retrieval_fifo;
	messageOut->error.data_retrieval_overflow = messageIn.error.data_retrieval_overflow;
	messageOut->error.data_retrieval_overflow2 = messageIn.error.data_retrieval_overflow2;
	messageOut->error.data_retrieval_underrun = messageIn.error.data_retrieval_underrun;
	messageOut->error.data_retrieval_missing_samples = messageIn.error.data_retrieval_missing_samples;
	messageOut->error.sensor_read_failure = messageIn.error.sensor_read_failure;
	messageOut->error.beam0_in_phase_tag_error = messageIn.error.beam0_in_phase_tag_error;
	messageOut->error.beam0_quad_phase_tag_error = messageIn.error.beam0_quad_phase_tag_error;
	messageOut->error.beam1_in_phase_tag_error = messageIn.error.beam1_in_phase_tag_error;
	messageOut->error.beam1_quad_phase_tag_error = messageIn.error.beam1_quad_phase_tag_error;
	messageOut->error.beam2_in_phase_tag_error = messageIn.error.beam2_in_phase_tag_error;
	messageOut->error.beam2_quad_phase_tag_error = messageIn.error.beam2_quad_phase_tag_error;
	messageOut->error.beam3_in_phase_tag_error = messageIn.error.beam3_in_phase_tag_error;
	messageOut->error.beam3_quad_phase_tag_error = messageIn.error.beam3_quad_phase_tag_error;
	messageOut->cpu_load = messageIn.cpu_load;
	messageOut->status0_used = messageIn.status0_used;

	messageOut->status.bdScaling = messageIn.status.bdScaling;
	messageOut->status.echoFrequency = messageIn.status.echoFrequency;
	messageOut->status.boostRunning = messageIn.status.boostRunning;
	messageOut->status.telemetryData = messageIn.status.telemetryData;
	messageOut->status.echoIndex = messageIn.status.echoIndex;
	messageOut->status.configurationActive = messageIn.status.configurationActive;
	messageOut->status.lastMeasVoltage = messageIn.status.lastMeasVoltage;
	messageOut->status.prevWakeup = messageIn.status.prevWakeup;
	messageOut->status.orientationAuto = messageIn.status.orientationAuto;
	messageOut->status.orientation = messageIn.status.orientation;
	messageOut->status.wakeup = messageIn.status.wakeup;
	messageOut->status.bdScaling = messageIn.status.bdScaling;
	messageOut->status.echoFrequency = messageIn.status.echoFrequency;
	messageOut->status.boostRunning = messageIn.status.boostRunning;
	messageOut->status.telemetryData = messageIn.status.telemetryData;
	messageOut->status.echoIndex = messageIn.status.echoIndex;
	messageOut->status.configurationActive = messageIn.status.configurationActive;
	messageOut->status.lastMeasVoltage = messageIn.status.lastMeasVoltage;
	messageOut->status.prevWakeup = messageIn.status.prevWakeup;
	messageOut->status.orientationAuto = messageIn.status.orientationAuto;
	messageOut->status.orientation = messageIn.status.orientation;
	messageOut->status.wakeup = messageIn.status.wakeup;
	messageOut->ensembleCounter = messageIn.ensembleCounter;
}

void Msg_6723_pub_callback(uint8_t * buffer)
{
	Msg_6723 Message;

	int status = 0;
	status = Message.Deserialize(buffer,getBufSize());

	 if (status != 0) {ROS_WARN("Message 0x6723 deserialization failed! %d returned",status); return;}
	convert(Message, &msgStruct_6723);
	ROS_DEBUG("Message 0x6723 Received");

	// Initialize Publisher if not initialized yet
	if (Msg_6723_pub_initialized == false){
		init_6723(getRosHandle());}
	// Publish the message
	Msg_6723_pub.publish(msgStruct_6723);
	return;
}
