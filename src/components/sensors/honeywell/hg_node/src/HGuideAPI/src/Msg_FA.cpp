#include <include/HGuideAPI.h>
#include <include/Msg_FA.h>

#include <cmath>
#include <cstring>

#include <include/utils/bit_utilities.h>
#include <include/utils/byte_buffer.h>
#include <include/utils/ectos_constants.h>
#include <include/utils/Checksum.h>


// Definition for Static constants
const uint8_t Msg_FA::SyncByte;
const uint8_t Msg_FA::MessageID;

Msg_FA::Msg_FA()
{
	Default();
}

void Msg_FA::Default()
{
	active_sensor_axes.Default();
	saturated_sensor_axes.Default();
	sensor_stat_fail.Default();
	sensor_temp_fail.Default();
	Checksum = 0;
}

bool Msg_FA::Serialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 12) return false;

	std::memset(buffer, 0, bufferSize);

	bool status_ok = true;
	ECTOS::BYTE_BUFFER::ByteOutputBuffer<unsigned int, unsigned char, ECTOS::BYTE_BUFFER::HGuideAPI_endianness> bb(buffer, bufferSize);

	bb.setOffset(0);	bb.put(SyncByte);
	bb.setOffset(1);	bb.put(MessageID);

	uint16_t active_sensor_axes_tmp = 0; // temporary variable holding the custom bitfield
	active_sensor_axes_tmp = ECTOS::BIT_UTILITIES::PackBool(active_sensor_axes_tmp, 0, active_sensor_axes.accel_x, status_ok);
	active_sensor_axes_tmp = ECTOS::BIT_UTILITIES::PackBool(active_sensor_axes_tmp, 1, active_sensor_axes.accel_y, status_ok);
	active_sensor_axes_tmp = ECTOS::BIT_UTILITIES::PackBool(active_sensor_axes_tmp, 2, active_sensor_axes.accel_z, status_ok);
	active_sensor_axes_tmp = ECTOS::BIT_UTILITIES::PackBool(active_sensor_axes_tmp, 3, active_sensor_axes.gyro_x, status_ok);
	active_sensor_axes_tmp = ECTOS::BIT_UTILITIES::PackBool(active_sensor_axes_tmp, 4, active_sensor_axes.gyro_y, status_ok);
	active_sensor_axes_tmp = ECTOS::BIT_UTILITIES::PackBool(active_sensor_axes_tmp, 5, active_sensor_axes.gyro_z, status_ok);
	active_sensor_axes_tmp = ECTOS::BIT_UTILITIES::PackBool(active_sensor_axes_tmp, 6, active_sensor_axes.mag_x, status_ok);
	active_sensor_axes_tmp = ECTOS::BIT_UTILITIES::PackBool(active_sensor_axes_tmp, 7, active_sensor_axes.mag_y, status_ok);
	active_sensor_axes_tmp = ECTOS::BIT_UTILITIES::PackBool(active_sensor_axes_tmp, 8, active_sensor_axes.mag_z, status_ok);
	bb.setOffset(2);	bb.put(active_sensor_axes_tmp);


	uint16_t saturated_sensor_axes_tmp = 0; // temporary variable holding the custom bitfield
	saturated_sensor_axes_tmp = ECTOS::BIT_UTILITIES::PackBool(saturated_sensor_axes_tmp, 0, saturated_sensor_axes.accel_x, status_ok);
	saturated_sensor_axes_tmp = ECTOS::BIT_UTILITIES::PackBool(saturated_sensor_axes_tmp, 1, saturated_sensor_axes.accel_y, status_ok);
	saturated_sensor_axes_tmp = ECTOS::BIT_UTILITIES::PackBool(saturated_sensor_axes_tmp, 2, saturated_sensor_axes.accel_z, status_ok);
	saturated_sensor_axes_tmp = ECTOS::BIT_UTILITIES::PackBool(saturated_sensor_axes_tmp, 3, saturated_sensor_axes.gyro_x, status_ok);
	saturated_sensor_axes_tmp = ECTOS::BIT_UTILITIES::PackBool(saturated_sensor_axes_tmp, 4, saturated_sensor_axes.gyro_y, status_ok);
	saturated_sensor_axes_tmp = ECTOS::BIT_UTILITIES::PackBool(saturated_sensor_axes_tmp, 5, saturated_sensor_axes.gyro_z, status_ok);
	saturated_sensor_axes_tmp = ECTOS::BIT_UTILITIES::PackBool(saturated_sensor_axes_tmp, 6, saturated_sensor_axes.mag_x, status_ok);
	saturated_sensor_axes_tmp = ECTOS::BIT_UTILITIES::PackBool(saturated_sensor_axes_tmp, 7, saturated_sensor_axes.mag_y, status_ok);
	saturated_sensor_axes_tmp = ECTOS::BIT_UTILITIES::PackBool(saturated_sensor_axes_tmp, 8, saturated_sensor_axes.mag_z, status_ok);
	bb.setOffset(4);	bb.put(saturated_sensor_axes_tmp);


	uint16_t sensor_stat_fail_tmp = 0; // temporary variable holding the custom bitfield
	sensor_stat_fail_tmp = ECTOS::BIT_UTILITIES::PackBool(sensor_stat_fail_tmp, 0, sensor_stat_fail.accel_x, status_ok);
	sensor_stat_fail_tmp = ECTOS::BIT_UTILITIES::PackBool(sensor_stat_fail_tmp, 1, sensor_stat_fail.accel_y, status_ok);
	sensor_stat_fail_tmp = ECTOS::BIT_UTILITIES::PackBool(sensor_stat_fail_tmp, 2, sensor_stat_fail.accel_z, status_ok);
	sensor_stat_fail_tmp = ECTOS::BIT_UTILITIES::PackBool(sensor_stat_fail_tmp, 3, sensor_stat_fail.gyro_x, status_ok);
	sensor_stat_fail_tmp = ECTOS::BIT_UTILITIES::PackBool(sensor_stat_fail_tmp, 4, sensor_stat_fail.gyro_y, status_ok);
	sensor_stat_fail_tmp = ECTOS::BIT_UTILITIES::PackBool(sensor_stat_fail_tmp, 5, sensor_stat_fail.gyro_z, status_ok);
	sensor_stat_fail_tmp = ECTOS::BIT_UTILITIES::PackBool(sensor_stat_fail_tmp, 6, sensor_stat_fail.mag_x, status_ok);
	sensor_stat_fail_tmp = ECTOS::BIT_UTILITIES::PackBool(sensor_stat_fail_tmp, 7, sensor_stat_fail.mag_y, status_ok);
	sensor_stat_fail_tmp = ECTOS::BIT_UTILITIES::PackBool(sensor_stat_fail_tmp, 8, sensor_stat_fail.mag_z, status_ok);
	bb.setOffset(6);	bb.put(sensor_stat_fail_tmp);


	uint16_t sensor_temp_fail_tmp = 0; // temporary variable holding the custom bitfield
	sensor_temp_fail_tmp = ECTOS::BIT_UTILITIES::PackBool(sensor_temp_fail_tmp, 0, sensor_temp_fail.accel_x, status_ok);
	sensor_temp_fail_tmp = ECTOS::BIT_UTILITIES::PackBool(sensor_temp_fail_tmp, 1, sensor_temp_fail.accel_y, status_ok);
	sensor_temp_fail_tmp = ECTOS::BIT_UTILITIES::PackBool(sensor_temp_fail_tmp, 2, sensor_temp_fail.accel_z, status_ok);
	sensor_temp_fail_tmp = ECTOS::BIT_UTILITIES::PackBool(sensor_temp_fail_tmp, 3, sensor_temp_fail.gyro_x, status_ok);
	sensor_temp_fail_tmp = ECTOS::BIT_UTILITIES::PackBool(sensor_temp_fail_tmp, 4, sensor_temp_fail.gyro_y, status_ok);
	sensor_temp_fail_tmp = ECTOS::BIT_UTILITIES::PackBool(sensor_temp_fail_tmp, 5, sensor_temp_fail.gyro_z, status_ok);
	sensor_temp_fail_tmp = ECTOS::BIT_UTILITIES::PackBool(sensor_temp_fail_tmp, 6, sensor_temp_fail.mag_x, status_ok);
	sensor_temp_fail_tmp = ECTOS::BIT_UTILITIES::PackBool(sensor_temp_fail_tmp, 7, sensor_temp_fail.mag_y, status_ok);
	sensor_temp_fail_tmp = ECTOS::BIT_UTILITIES::PackBool(sensor_temp_fail_tmp, 8, sensor_temp_fail.mag_z, status_ok);
	bb.setOffset(8);	bb.put(sensor_temp_fail_tmp);

	int cIndex = 10;
	Checksum = 0;
	Checksum = computeChecksum16Bit((uint8_t*)buffer, 12);
	bb.setOffset(cIndex);		bb.put(Checksum);
	return status_ok;
}

int Msg_FA::Deserialize(unsigned char * buffer, const unsigned int bufferSize)
{
	bool status_ok = true;
	if (bufferSize < 12) return -2;

	ECTOS::BYTE_BUFFER::ByteInputBuffer<unsigned int, unsigned char, ECTOS::BYTE_BUFFER::HGuideAPI_endianness> bb(buffer, bufferSize);

	int constCheck = 0;
	int numConsts = 0;
	// Compare with the static const variable to ensure they match. 
	uint8_t SyncByte_In;
	bb.setOffset(0);	bb.get(SyncByte_In);
	constCheck |= (SyncByte != SyncByte_In) << numConsts++;
	// Compare with the static const variable to ensure they match. 
	uint8_t MessageID_In;
	bb.setOffset(1);	bb.get(MessageID_In);
	constCheck |= (MessageID != MessageID_In) << numConsts++;

	uint16_t active_sensor_axes_tmp = 0; // temporary variable holding the custom bitfield
	bb.setOffset(2);	bb.get(active_sensor_axes_tmp);
	active_sensor_axes.accel_x = ECTOS::BIT_UTILITIES::UnPackBool(active_sensor_axes_tmp, 0,  status_ok);
	active_sensor_axes.accel_y = ECTOS::BIT_UTILITIES::UnPackBool(active_sensor_axes_tmp, 1,  status_ok);
	active_sensor_axes.accel_z = ECTOS::BIT_UTILITIES::UnPackBool(active_sensor_axes_tmp, 2,  status_ok);
	active_sensor_axes.gyro_x = ECTOS::BIT_UTILITIES::UnPackBool(active_sensor_axes_tmp, 3,  status_ok);
	active_sensor_axes.gyro_y = ECTOS::BIT_UTILITIES::UnPackBool(active_sensor_axes_tmp, 4,  status_ok);
	active_sensor_axes.gyro_z = ECTOS::BIT_UTILITIES::UnPackBool(active_sensor_axes_tmp, 5,  status_ok);
	active_sensor_axes.mag_x = ECTOS::BIT_UTILITIES::UnPackBool(active_sensor_axes_tmp, 6,  status_ok);
	active_sensor_axes.mag_y = ECTOS::BIT_UTILITIES::UnPackBool(active_sensor_axes_tmp, 7,  status_ok);
	active_sensor_axes.mag_z = ECTOS::BIT_UTILITIES::UnPackBool(active_sensor_axes_tmp, 8,  status_ok);

	uint16_t saturated_sensor_axes_tmp = 0; // temporary variable holding the custom bitfield
	bb.setOffset(4);	bb.get(saturated_sensor_axes_tmp);
	saturated_sensor_axes.accel_x = ECTOS::BIT_UTILITIES::UnPackBool(saturated_sensor_axes_tmp, 0,  status_ok);
	saturated_sensor_axes.accel_y = ECTOS::BIT_UTILITIES::UnPackBool(saturated_sensor_axes_tmp, 1,  status_ok);
	saturated_sensor_axes.accel_z = ECTOS::BIT_UTILITIES::UnPackBool(saturated_sensor_axes_tmp, 2,  status_ok);
	saturated_sensor_axes.gyro_x = ECTOS::BIT_UTILITIES::UnPackBool(saturated_sensor_axes_tmp, 3,  status_ok);
	saturated_sensor_axes.gyro_y = ECTOS::BIT_UTILITIES::UnPackBool(saturated_sensor_axes_tmp, 4,  status_ok);
	saturated_sensor_axes.gyro_z = ECTOS::BIT_UTILITIES::UnPackBool(saturated_sensor_axes_tmp, 5,  status_ok);
	saturated_sensor_axes.mag_x = ECTOS::BIT_UTILITIES::UnPackBool(saturated_sensor_axes_tmp, 6,  status_ok);
	saturated_sensor_axes.mag_y = ECTOS::BIT_UTILITIES::UnPackBool(saturated_sensor_axes_tmp, 7,  status_ok);
	saturated_sensor_axes.mag_z = ECTOS::BIT_UTILITIES::UnPackBool(saturated_sensor_axes_tmp, 8,  status_ok);

	uint16_t sensor_stat_fail_tmp = 0; // temporary variable holding the custom bitfield
	bb.setOffset(6);	bb.get(sensor_stat_fail_tmp);
	sensor_stat_fail.accel_x = ECTOS::BIT_UTILITIES::UnPackBool(sensor_stat_fail_tmp, 0,  status_ok);
	sensor_stat_fail.accel_y = ECTOS::BIT_UTILITIES::UnPackBool(sensor_stat_fail_tmp, 1,  status_ok);
	sensor_stat_fail.accel_z = ECTOS::BIT_UTILITIES::UnPackBool(sensor_stat_fail_tmp, 2,  status_ok);
	sensor_stat_fail.gyro_x = ECTOS::BIT_UTILITIES::UnPackBool(sensor_stat_fail_tmp, 3,  status_ok);
	sensor_stat_fail.gyro_y = ECTOS::BIT_UTILITIES::UnPackBool(sensor_stat_fail_tmp, 4,  status_ok);
	sensor_stat_fail.gyro_z = ECTOS::BIT_UTILITIES::UnPackBool(sensor_stat_fail_tmp, 5,  status_ok);
	sensor_stat_fail.mag_x = ECTOS::BIT_UTILITIES::UnPackBool(sensor_stat_fail_tmp, 6,  status_ok);
	sensor_stat_fail.mag_y = ECTOS::BIT_UTILITIES::UnPackBool(sensor_stat_fail_tmp, 7,  status_ok);
	sensor_stat_fail.mag_z = ECTOS::BIT_UTILITIES::UnPackBool(sensor_stat_fail_tmp, 8,  status_ok);

	uint16_t sensor_temp_fail_tmp = 0; // temporary variable holding the custom bitfield
	bb.setOffset(8);	bb.get(sensor_temp_fail_tmp);
	sensor_temp_fail.accel_x = ECTOS::BIT_UTILITIES::UnPackBool(sensor_temp_fail_tmp, 0,  status_ok);
	sensor_temp_fail.accel_y = ECTOS::BIT_UTILITIES::UnPackBool(sensor_temp_fail_tmp, 1,  status_ok);
	sensor_temp_fail.accel_z = ECTOS::BIT_UTILITIES::UnPackBool(sensor_temp_fail_tmp, 2,  status_ok);
	sensor_temp_fail.gyro_x = ECTOS::BIT_UTILITIES::UnPackBool(sensor_temp_fail_tmp, 3,  status_ok);
	sensor_temp_fail.gyro_y = ECTOS::BIT_UTILITIES::UnPackBool(sensor_temp_fail_tmp, 4,  status_ok);
	sensor_temp_fail.gyro_z = ECTOS::BIT_UTILITIES::UnPackBool(sensor_temp_fail_tmp, 5,  status_ok);
	sensor_temp_fail.mag_x = ECTOS::BIT_UTILITIES::UnPackBool(sensor_temp_fail_tmp, 6,  status_ok);
	sensor_temp_fail.mag_y = ECTOS::BIT_UTILITIES::UnPackBool(sensor_temp_fail_tmp, 7,  status_ok);
	sensor_temp_fail.mag_z = ECTOS::BIT_UTILITIES::UnPackBool(sensor_temp_fail_tmp, 8,  status_ok);
	bb.setOffset(10);	bb.get(Checksum);
	uint16_t computedCRC = computeChecksum16Bit((uint8_t*)buffer, 12);
	int retVal = 0;
	retVal = constCheck;
	if(computedCRC != Checksum)
		retVal = -1;
	return retVal;
}

