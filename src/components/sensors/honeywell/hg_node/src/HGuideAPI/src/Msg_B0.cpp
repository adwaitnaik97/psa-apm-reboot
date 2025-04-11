#include <include/HGuideAPI.h>
#include <include/Msg_B0.h>

#include <cmath>
#include <cstring>

#include <include/utils/bit_utilities.h>
#include <include/utils/byte_buffer.h>
#include <include/utils/ectos_constants.h>
#include <include/utils/Checksum.h>


// Definition for Static constants
const uint8_t Msg_B0::SyncByte;
const uint8_t Msg_B0::MessageID;

Msg_B0::Msg_B0()
{
	Default();
}

void Msg_B0::Default()
{
	fw_major = 0;
	fw_minor = 0;
	for (unsigned int index = 0; index < 4; index++)
	{
		device_type[index] = 0;
	}
	part_number = 0;
	hardware_version = 0;
	device_config = 0;
	for (unsigned int index = 0; index < 8; index++)
	{
		serial_number[index] = 0;
	}
	gyro_filter_config.Default();
	accel_filter_config.Default();
	gyro_range = 0;
	accel_range = 0;
	hours_of_operation = 0;
	boot_count = 0;
	baud_rate = 0;
	sampling_frequency = 0;
	bytes_transmitted = 0;
	percent_fr = 0;
	percent_ffr = 0;
	percent_cr = 0;
	percent_gr = 0;
	percent_100_fr = 0;
	percent_10_fr = 0;
	percent_1_fr = 0;
	device_current = 0;
	device_voltage = 0;
	active_sensor_axes.Default();
	saturated_sensor_axes.Default();
	Checksum = 0;
}

bool Msg_B0::Serialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 82) return false;

	std::memset(buffer, 0, bufferSize);

	bool status_ok = true;
	ECTOS::BYTE_BUFFER::ByteOutputBuffer<unsigned int, unsigned char, ECTOS::BYTE_BUFFER::HGuideAPI_endianness> bb(buffer, bufferSize);

	bb.setOffset(0);	bb.put(SyncByte);
	bb.setOffset(1);	bb.put(MessageID);
	bb.setOffset(2);	bb.put(fw_major);
	bb.setOffset(3);	bb.put(fw_minor);
	bb.setOffset(4);
	for (unsigned int index = 0; index < 4; index++)
	{
		bb.put(device_type[index]);
	}
	bb.setOffset(8);	bb.put(part_number);
	bb.setOffset(12);	bb.put(hardware_version);
	bb.setOffset(16);	bb.put(device_config);
	bb.setOffset(20);
	for (unsigned int index = 0; index < 8; index++)
	{
		bb.put(serial_number[index]);
	}

	uint16_t gyro_filter_config_tmp = 0; // temporary variable holding the custom bitfield
	gyro_filter_config_tmp = ECTOS::BIT_UTILITIES::Pack(gyro_filter_config_tmp, 0, 14, static_cast<uint16_t>CHECK_MAX_UINT16(gyro_filter_config.cutoff_frequency / (3.051850947599719e-05)), status_ok);
	gyro_filter_config_tmp = ECTOS::BIT_UTILITIES::PackBool(gyro_filter_config_tmp, 15, gyro_filter_config.enabled, status_ok);
	bb.setOffset(28);	bb.put(gyro_filter_config_tmp);


	uint16_t accel_filter_config_tmp = 0; // temporary variable holding the custom bitfield
	accel_filter_config_tmp = ECTOS::BIT_UTILITIES::Pack(accel_filter_config_tmp, 0, 14, static_cast<uint16_t>CHECK_MAX_UINT16(accel_filter_config.cutoff_frequency / (3.051850947599719e-05)), status_ok);
	accel_filter_config_tmp = ECTOS::BIT_UTILITIES::PackBool(accel_filter_config_tmp, 15, accel_filter_config.enabled, status_ok);
	bb.setOffset(30);	bb.put(accel_filter_config_tmp);

	bb.setOffset(32);	bb.put(gyro_range);
	bb.setOffset(36);	bb.put(accel_range / (0.3048));
	bb.setOffset(40);	bb.put(hours_of_operation);
	bb.setOffset(44);	bb.put(boot_count);
	bb.setOffset(48);	bb.put(baud_rate);
	bb.setOffset(52);	bb.put(sampling_frequency);
	bb.setOffset(56);	bb.put(bytes_transmitted);
	bb.setOffset(60);	bb.put(static_cast<uint8_t>CHECK_MAX_UINT8(percent_fr / (1.0/255)));
	bb.setOffset(61);	bb.put(static_cast<uint8_t>CHECK_MAX_UINT8(percent_ffr / (1.0/255)));
	bb.setOffset(62);	bb.put(static_cast<uint8_t>CHECK_MAX_UINT8(percent_cr / (1.0/255)));
	bb.setOffset(63);	bb.put(static_cast<uint8_t>CHECK_MAX_UINT8(percent_gr / (1.0/255)));
	bb.setOffset(64);	bb.put(static_cast<uint8_t>CHECK_MAX_UINT8(percent_100_fr / (1.0/255)));
	bb.setOffset(65);	bb.put(static_cast<uint8_t>CHECK_MAX_UINT8(percent_10_fr / (1.0/255)));
	bb.setOffset(66);	bb.put(static_cast<uint8_t>CHECK_MAX_UINT8(percent_1_fr / (1.0/255)));
	bb.setOffset(68);	bb.put(device_current);
	bb.setOffset(72);	bb.put(device_voltage);

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
	bb.setOffset(76);	bb.put(active_sensor_axes_tmp);


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
	bb.setOffset(78);	bb.put(saturated_sensor_axes_tmp);

	int cIndex = 80;
	Checksum = 0;
	Checksum = computeChecksum16Bit((uint8_t*)buffer, 82);
	bb.setOffset(cIndex);		bb.put(Checksum);
	return status_ok;
}

int Msg_B0::Deserialize(unsigned char * buffer, const unsigned int bufferSize)
{
	bool status_ok = true;
	if (bufferSize < 82) return -2;

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
	bb.setOffset(2);	bb.get(fw_major);
	bb.setOffset(3);	bb.get(fw_minor);
	bb.setOffset(4);
	for (unsigned int index = 0; index < 4; index++)
	{
		bb.get(device_type[index]);
	}
	bb.setOffset(8);	bb.get(part_number);
	bb.setOffset(12);	bb.get(hardware_version);
	bb.setOffset(16);	bb.get(device_config);
	bb.setOffset(20);
	for (unsigned int index = 0; index < 8; index++)
	{
		bb.get(serial_number[index]);
	}

	uint16_t gyro_filter_config_tmp = 0; // temporary variable holding the custom bitfield
	bb.setOffset(28);	bb.get(gyro_filter_config_tmp);
	gyro_filter_config.cutoff_frequency = static_cast<float>(ECTOS::BIT_UTILITIES::UnPack(gyro_filter_config_tmp, 0, 14,  status_ok) * (3.051850947599719e-05));
	gyro_filter_config.enabled = ECTOS::BIT_UTILITIES::UnPackBool(gyro_filter_config_tmp, 15,  status_ok);

	uint16_t accel_filter_config_tmp = 0; // temporary variable holding the custom bitfield
	bb.setOffset(30);	bb.get(accel_filter_config_tmp);
	accel_filter_config.cutoff_frequency = static_cast<float>(ECTOS::BIT_UTILITIES::UnPack(accel_filter_config_tmp, 0, 14,  status_ok) * (3.051850947599719e-05));
	accel_filter_config.enabled = ECTOS::BIT_UTILITIES::UnPackBool(accel_filter_config_tmp, 15,  status_ok);
	bb.setOffset(32);	bb.get(gyro_range);
	bb.setOffset(36);	accel_range = bb.get<float>() * (0.3048);
	bb.setOffset(40);	bb.get(hours_of_operation);
	bb.setOffset(44);	bb.get(boot_count);
	bb.setOffset(48);	bb.get(baud_rate);
	bb.setOffset(52);	bb.get(sampling_frequency);
	bb.setOffset(56);	bb.get(bytes_transmitted);
	bb.setOffset(60);	percent_fr = static_cast<float>(bb.get<uint8_t>()) * (1.0/255);
	bb.setOffset(61);	percent_ffr = static_cast<float>(bb.get<uint8_t>()) * (1.0/255);
	bb.setOffset(62);	percent_cr = static_cast<float>(bb.get<uint8_t>()) * (1.0/255);
	bb.setOffset(63);	percent_gr = static_cast<float>(bb.get<uint8_t>()) * (1.0/255);
	bb.setOffset(64);	percent_100_fr = static_cast<float>(bb.get<uint8_t>()) * (1.0/255);
	bb.setOffset(65);	percent_10_fr = static_cast<float>(bb.get<uint8_t>()) * (1.0/255);
	bb.setOffset(66);	percent_1_fr = static_cast<float>(bb.get<uint8_t>()) * (1.0/255);
	bb.setOffset(68);	bb.get(device_current);
	bb.setOffset(72);	bb.get(device_voltage);

	uint16_t active_sensor_axes_tmp = 0; // temporary variable holding the custom bitfield
	bb.setOffset(76);	bb.get(active_sensor_axes_tmp);
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
	bb.setOffset(78);	bb.get(saturated_sensor_axes_tmp);
	saturated_sensor_axes.accel_x = ECTOS::BIT_UTILITIES::UnPackBool(saturated_sensor_axes_tmp, 0,  status_ok);
	saturated_sensor_axes.accel_y = ECTOS::BIT_UTILITIES::UnPackBool(saturated_sensor_axes_tmp, 1,  status_ok);
	saturated_sensor_axes.accel_z = ECTOS::BIT_UTILITIES::UnPackBool(saturated_sensor_axes_tmp, 2,  status_ok);
	saturated_sensor_axes.gyro_x = ECTOS::BIT_UTILITIES::UnPackBool(saturated_sensor_axes_tmp, 3,  status_ok);
	saturated_sensor_axes.gyro_y = ECTOS::BIT_UTILITIES::UnPackBool(saturated_sensor_axes_tmp, 4,  status_ok);
	saturated_sensor_axes.gyro_z = ECTOS::BIT_UTILITIES::UnPackBool(saturated_sensor_axes_tmp, 5,  status_ok);
	saturated_sensor_axes.mag_x = ECTOS::BIT_UTILITIES::UnPackBool(saturated_sensor_axes_tmp, 6,  status_ok);
	saturated_sensor_axes.mag_y = ECTOS::BIT_UTILITIES::UnPackBool(saturated_sensor_axes_tmp, 7,  status_ok);
	saturated_sensor_axes.mag_z = ECTOS::BIT_UTILITIES::UnPackBool(saturated_sensor_axes_tmp, 8,  status_ok);
	bb.setOffset(80);	bb.get(Checksum);
	uint16_t computedCRC = computeChecksum16Bit((uint8_t*)buffer, 82);
	int retVal = 0;
	retVal = constCheck;
	if(computedCRC != Checksum)
		retVal = -1;
	return retVal;
}

