#include <include/HGuideAPI.h>
#include <include/Msg_2423.h>

#include <cmath>
#include <cstring>

#include <include/utils/bit_utilities.h>
#include <include/utils/byte_buffer.h>
#include <include/utils/ectos_constants.h>
#include <include/utils/Checksum.h>


// Definition for Static constants
const uint32_t Msg_2423::AddressId;
const uint32_t Msg_2423::MessageId;
const uint32_t Msg_2423::MessageLength;

Msg_2423::Msg_2423()
{
	Default();
}

void Msg_2423::Default()
{
	Checksum = 0;
	INSMode = static_cast<ins_mode_table_t>(0);
	systemTov = 0;
	gpsTov = 0;
	gps_week = 0;
	gyro_bias_x_std_dev = 0;
	gyro_bias_y_std_dev = 0;
	gyro_bias_z_std_dev = 0;
	gyro_bias_inrun_x_std_dev = 0;
	gyro_bias_inrun_y_std_dev = 0;
	gyro_bias_inrun_z_std_dev = 0;
	gyro_scale_factor_x_std_dev = 0;
	gyro_scale_factor_y_std_dev = 0;
	gyro_scale_factor_z_std_dev = 0;
	gyro_nonorthogonality_yz_std_dev = 0;
	gyro_nonorthogonality_zx_std_dev = 0;
	gyro_nonorthogonality_xy_std_dev = 0;
	accelerometer_bias_x_std_dev = 0;
	accelerometer_bias_y_std_dev = 0;
	accelerometer_bias_z_std_dev = 0;
	accelerometer_bias_inrun_x_std_dev = 0;
	accelerometer_bias_inrun_y_std_dev = 0;
	accelerometer_bias_inrun_z_std_dev = 0;
	accelerometer_scale_factor_x_std_dev = 0;
	accelerometer_scale_factor_y_std_dev = 0;
	accelerometer_scale_factor_z_std_dev = 0;
	accelerometer_nonorthogonality_yz_std_dev = 0;
	accelerometer_nonorthogonality_zx_std_dev = 0;
	accelerometer_nonorthogonality_xy_std_dev = 0;
	accelerometer_misalignment_x_std_dev = 0;
	accelerometer_misalignment_y_std_dev = 0;
	accelerometer_misalignment_z_std_dev = 0;
	accelerometer_scale_factor_nonlinearity_x_std_dev = 0;
	accelerometer_scale_factor_nonlinearity_y_std_dev = 0;
	accelerometer_scale_factor_nonlinearity_z_std_dev = 0;
}

bool Msg_2423::Serialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 200) return false;

	std::memset(buffer, 0, bufferSize);

	bool status_ok = true;
	ECTOS::BYTE_BUFFER::ByteOutputBuffer<unsigned int, unsigned char, ECTOS::BYTE_BUFFER::HGuideAPI_endianness> bb(buffer, bufferSize);

	bb.setOffset(0);	bb.put(AddressId);
	bb.setOffset(4);	bb.put(MessageId);
	bb.setOffset(8);	bb.put(MessageLength);
	int cIndex = 12;
	Checksum = 0;
	bb.setOffset(20);	bb.put(static_cast<uint32_t>CHECK_MAX_UINT32(INSMode));
	bb.setOffset(24);	bb.put(systemTov);
	bb.setOffset(32);	bb.put(gpsTov);
	bb.setOffset(40);	bb.put(gps_week);
	bb.setOffset(44);	bb.put(gyro_bias_x_std_dev);
	bb.setOffset(48);	bb.put(gyro_bias_y_std_dev);
	bb.setOffset(52);	bb.put(gyro_bias_z_std_dev);
	bb.setOffset(56);	bb.put(gyro_bias_inrun_x_std_dev);
	bb.setOffset(60);	bb.put(gyro_bias_inrun_y_std_dev);
	bb.setOffset(64);	bb.put(gyro_bias_inrun_z_std_dev);
	bb.setOffset(68);	bb.put(gyro_scale_factor_x_std_dev);
	bb.setOffset(72);	bb.put(gyro_scale_factor_y_std_dev);
	bb.setOffset(76);	bb.put(gyro_scale_factor_z_std_dev);
	bb.setOffset(80);	bb.put(gyro_nonorthogonality_yz_std_dev);
	bb.setOffset(84);	bb.put(gyro_nonorthogonality_zx_std_dev);
	bb.setOffset(88);	bb.put(gyro_nonorthogonality_xy_std_dev);
	bb.setOffset(104);	bb.put(accelerometer_bias_x_std_dev);
	bb.setOffset(108);	bb.put(accelerometer_bias_y_std_dev);
	bb.setOffset(112);	bb.put(accelerometer_bias_z_std_dev);
	bb.setOffset(116);	bb.put(accelerometer_bias_inrun_x_std_dev);
	bb.setOffset(120);	bb.put(accelerometer_bias_inrun_y_std_dev);
	bb.setOffset(124);	bb.put(accelerometer_bias_inrun_z_std_dev);
	bb.setOffset(128);	bb.put(accelerometer_scale_factor_x_std_dev);
	bb.setOffset(132);	bb.put(accelerometer_scale_factor_y_std_dev);
	bb.setOffset(136);	bb.put(accelerometer_scale_factor_z_std_dev);
	bb.setOffset(140);	bb.put(accelerometer_nonorthogonality_yz_std_dev);
	bb.setOffset(144);	bb.put(accelerometer_nonorthogonality_zx_std_dev);
	bb.setOffset(148);	bb.put(accelerometer_nonorthogonality_xy_std_dev);
	bb.setOffset(152);	bb.put(accelerometer_misalignment_x_std_dev);
	bb.setOffset(156);	bb.put(accelerometer_misalignment_y_std_dev);
	bb.setOffset(160);	bb.put(accelerometer_misalignment_z_std_dev);
	bb.setOffset(164);	bb.put(accelerometer_scale_factor_nonlinearity_x_std_dev);
	bb.setOffset(168);	bb.put(accelerometer_scale_factor_nonlinearity_y_std_dev);
	bb.setOffset(172);	bb.put(accelerometer_scale_factor_nonlinearity_z_std_dev);
	Checksum = computeChecksum((uint32_t*)buffer, 50);
	bb.setOffset(cIndex);		bb.put(Checksum);
	return status_ok;
}

int Msg_2423::Deserialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 200) return -2;

	ECTOS::BYTE_BUFFER::ByteInputBuffer<unsigned int, unsigned char, ECTOS::BYTE_BUFFER::HGuideAPI_endianness> bb(buffer, bufferSize);

	int constCheck = 0;
	int numConsts = 0;
	// Compare with the static const variable to ensure they match. 
	uint32_t AddressId_In;
	bb.setOffset(0);	bb.get(AddressId_In);
	constCheck |= (AddressId != AddressId_In) << numConsts++;
	// Compare with the static const variable to ensure they match. 
	uint32_t MessageId_In;
	bb.setOffset(4);	bb.get(MessageId_In);
	constCheck |= (MessageId != MessageId_In) << numConsts++;
	// Compare with the static const variable to ensure they match. 
	uint32_t MessageLength_In;
	bb.setOffset(8);	bb.get(MessageLength_In);
	constCheck |= (MessageLength != MessageLength_In) << numConsts++;
	bb.setOffset(20);	INSMode = static_cast<ins_mode_table_t>(bb.get<uint32_t>());
	bb.setOffset(24);	bb.get(systemTov);
	bb.setOffset(32);	bb.get(gpsTov);
	bb.setOffset(40);	bb.get(gps_week);
	bb.setOffset(44);	bb.get(gyro_bias_x_std_dev);
	bb.setOffset(48);	bb.get(gyro_bias_y_std_dev);
	bb.setOffset(52);	bb.get(gyro_bias_z_std_dev);
	bb.setOffset(56);	bb.get(gyro_bias_inrun_x_std_dev);
	bb.setOffset(60);	bb.get(gyro_bias_inrun_y_std_dev);
	bb.setOffset(64);	bb.get(gyro_bias_inrun_z_std_dev);
	bb.setOffset(68);	bb.get(gyro_scale_factor_x_std_dev);
	bb.setOffset(72);	bb.get(gyro_scale_factor_y_std_dev);
	bb.setOffset(76);	bb.get(gyro_scale_factor_z_std_dev);
	bb.setOffset(80);	bb.get(gyro_nonorthogonality_yz_std_dev);
	bb.setOffset(84);	bb.get(gyro_nonorthogonality_zx_std_dev);
	bb.setOffset(88);	bb.get(gyro_nonorthogonality_xy_std_dev);
	bb.setOffset(104);	bb.get(accelerometer_bias_x_std_dev);
	bb.setOffset(108);	bb.get(accelerometer_bias_y_std_dev);
	bb.setOffset(112);	bb.get(accelerometer_bias_z_std_dev);
	bb.setOffset(116);	bb.get(accelerometer_bias_inrun_x_std_dev);
	bb.setOffset(120);	bb.get(accelerometer_bias_inrun_y_std_dev);
	bb.setOffset(124);	bb.get(accelerometer_bias_inrun_z_std_dev);
	bb.setOffset(128);	bb.get(accelerometer_scale_factor_x_std_dev);
	bb.setOffset(132);	bb.get(accelerometer_scale_factor_y_std_dev);
	bb.setOffset(136);	bb.get(accelerometer_scale_factor_z_std_dev);
	bb.setOffset(140);	bb.get(accelerometer_nonorthogonality_yz_std_dev);
	bb.setOffset(144);	bb.get(accelerometer_nonorthogonality_zx_std_dev);
	bb.setOffset(148);	bb.get(accelerometer_nonorthogonality_xy_std_dev);
	bb.setOffset(152);	bb.get(accelerometer_misalignment_x_std_dev);
	bb.setOffset(156);	bb.get(accelerometer_misalignment_y_std_dev);
	bb.setOffset(160);	bb.get(accelerometer_misalignment_z_std_dev);
	bb.setOffset(164);	bb.get(accelerometer_scale_factor_nonlinearity_x_std_dev);
	bb.setOffset(168);	bb.get(accelerometer_scale_factor_nonlinearity_y_std_dev);
	bb.setOffset(172);	bb.get(accelerometer_scale_factor_nonlinearity_z_std_dev);
	bb.setOffset(12);	bb.get(Checksum);
	uint32_t computedCRC = computeChecksum((uint32_t*)buffer, 50);
	int retVal = 0;
	retVal = constCheck;
	if(computedCRC != Checksum)
		retVal = -1;
	return retVal;
}

