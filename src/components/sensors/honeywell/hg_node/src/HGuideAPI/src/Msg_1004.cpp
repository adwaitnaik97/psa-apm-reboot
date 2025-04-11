#include <include/HGuideAPI.h>
#include <include/Msg_1004.h>

#include <cmath>
#include <cstring>

#include <include/utils/bit_utilities.h>
#include <include/utils/byte_buffer.h>
#include <include/utils/ectos_constants.h>
#include <include/utils/Checksum.h>


// Definition for Static constants
const uint32_t Msg_1004::AddressId;
const uint32_t Msg_1004::MessageId;
const uint32_t Msg_1004::MessageLength;

Msg_1004::Msg_1004()
{
	Default();
}

void Msg_1004::Default()
{
	Checksum = 0;
	ULV_GPSSigLatencyL1 = 0;
	ULV_GPSSigLatencyL2 = 0;
	ULV_VehFrametoCaseFrame = 0;
	ULV_Vehicle_Case_FrameLeverArm = 0;
	ULV_LeverArmToPort4GPSAnt = 0;
	ULV_MeasFrametoVehFrame = 0;
	ULV_LeverArmToICDNav = 0;
	ULV_LeverArmToICDGPSAntenna = 0;
	ULV_LocationOfAutopilotFCData = 0;
	L1SignalLatency = 0;
	L2SignalLatency = 0;
	Vehicle_to_Case_C11 = 0;
	Vehicle_to_Case_C12 = 0;
	Vehicle_to_Case_C13 = 0;
	Vehicle_to_Case_C21 = 0;
	Vehicle_to_Case_C22 = 0;
	Vehicle_to_Case_C23 = 0;
	Vehicle_to_Case_C31 = 0;
	Vehicle_to_Case_C32 = 0;
	Vehicle_to_Case_C33 = 0;
	Vehicle_to_Case_Roll = 0;
	Vehicle_to_Case_Pitch = 0;
	Vehicle_to_Case_Heading = 0;
	VehCaseLvrArm_x = 0;
	VehCaseLvrArm_y = 0;
	VehCaseLvrArm_z = 0;
	VehGPSPort4LvrArm_x = 0;
	VehGPSPort4LvrArm_y = 0;
	VehGPSPort4LvrArm_z = 0;
	Meas_to_Vehicle_C11 = 0;
	Meas_to_Vehicle_C12 = 0;
	Meas_to_Vehicle_C13 = 0;
	Meas_to_Vehicle_C21 = 0;
	Meas_to_Vehicle_C22 = 0;
	Meas_to_Vehicle_C23 = 0;
	Meas_to_Vehicle_C31 = 0;
	Meas_to_Vehicle_C32 = 0;
	Meas_to_Vehicle_C33 = 0;
	Meas_to_Vehicle_Roll = 0;
	Meas_to_Vehicle_Pitch = 0;
	Meas_to_Vehicle_Heading = 0;
	VehICDNavLvrArm_x = 0;
	VehICDNavLvrArm_y = 0;
	VehICDNavLvrArm_z = 0;
	VehGPSPort4LvrArm_x_2 = 0;
	VehGPSPort4LvrArm_y_2 = 0;
	VehGPSPort4LvrArm_z_2 = 0;
}

bool Msg_1004::Serialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 180) return false;

	std::memset(buffer, 0, bufferSize);

	bool status_ok = true;
	ECTOS::BYTE_BUFFER::ByteOutputBuffer<unsigned int, unsigned char, ECTOS::BYTE_BUFFER::HGuideAPI_endianness> bb(buffer, bufferSize);

	bb.setOffset(0);	bb.put(AddressId);
	bb.setOffset(4);	bb.put(MessageId);
	bb.setOffset(8);	bb.put(MessageLength);
	int cIndex = 12;
	Checksum = 0;

	uint32_t bitfieldAtByte16 = 0; // temporary variable holding the bitfield
	bitfieldAtByte16 = ECTOS::BIT_UTILITIES::PackBool(bitfieldAtByte16, 0, ULV_GPSSigLatencyL1, status_ok);
	bitfieldAtByte16 = ECTOS::BIT_UTILITIES::PackBool(bitfieldAtByte16, 1, ULV_GPSSigLatencyL2, status_ok);
	bitfieldAtByte16 = ECTOS::BIT_UTILITIES::Pack(bitfieldAtByte16, 4, 5, ULV_VehFrametoCaseFrame, status_ok);
	bitfieldAtByte16 = ECTOS::BIT_UTILITIES::Pack(bitfieldAtByte16, 6, 7, ULV_Vehicle_Case_FrameLeverArm, status_ok);
	bitfieldAtByte16 = ECTOS::BIT_UTILITIES::Pack(bitfieldAtByte16, 8, 9, ULV_LeverArmToPort4GPSAnt, status_ok);
	bitfieldAtByte16 = ECTOS::BIT_UTILITIES::Pack(bitfieldAtByte16, 10, 11, ULV_MeasFrametoVehFrame, status_ok);
	bitfieldAtByte16 = ECTOS::BIT_UTILITIES::Pack(bitfieldAtByte16, 12, 13, ULV_LeverArmToICDNav, status_ok);
	bitfieldAtByte16 = ECTOS::BIT_UTILITIES::Pack(bitfieldAtByte16, 14, 15, ULV_LeverArmToICDGPSAntenna, status_ok);
	bitfieldAtByte16 = ECTOS::BIT_UTILITIES::Pack(bitfieldAtByte16, 16, 17, ULV_LocationOfAutopilotFCData, status_ok);
	bb.setOffset(16);	bb.put(bitfieldAtByte16);

	bb.setOffset(20);	bb.put(L1SignalLatency);
	bb.setOffset(24);	bb.put(L2SignalLatency);
	bb.setOffset(72);	bb.put(VehCaseLvrArm_x);
	bb.setOffset(76);	bb.put(VehCaseLvrArm_y);
	bb.setOffset(80);	bb.put(VehCaseLvrArm_z);
	bb.setOffset(84);	bb.put(VehGPSPort4LvrArm_x);
	bb.setOffset(88);	bb.put(VehGPSPort4LvrArm_y);
	bb.setOffset(92);	bb.put(VehGPSPort4LvrArm_z);
	bb.setOffset(132);	bb.put(VehICDNavLvrArm_x);
	bb.setOffset(136);	bb.put(VehICDNavLvrArm_y);
	bb.setOffset(140);	bb.put(VehICDNavLvrArm_z);
	bb.setOffset(144);	bb.put(VehGPSPort4LvrArm_x_2);
	bb.setOffset(148);	bb.put(VehGPSPort4LvrArm_y_2);
	bb.setOffset(152);	bb.put(VehGPSPort4LvrArm_z_2);

	if ( ULV_VehFrametoCaseFrame == 1 ) {
			bb.setOffset(36);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(Vehicle_to_Case_C11 / (std::pow(2, -31))));
			bb.setOffset(40);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(Vehicle_to_Case_C12 / (std::pow(2, -31))));
			bb.setOffset(44);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(Vehicle_to_Case_C13 / (std::pow(2, -31))));
			bb.setOffset(48);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(Vehicle_to_Case_C21 / (std::pow(2, -31))));
			bb.setOffset(52);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(Vehicle_to_Case_C22 / (std::pow(2, -31))));
			bb.setOffset(56);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(Vehicle_to_Case_C23 / (std::pow(2, -31))));
			bb.setOffset(60);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(Vehicle_to_Case_C31 / (std::pow(2, -31))));
			bb.setOffset(64);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(Vehicle_to_Case_C32 / (std::pow(2, -31))));
			bb.setOffset(68);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(Vehicle_to_Case_C33 / (std::pow(2, -31))));
		}

	if ( ULV_VehFrametoCaseFrame == 2 ) {
			bb.setOffset(36);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(Vehicle_to_Case_Roll / (ECTOS::CONSTANTS::pi)));
			bb.setOffset(40);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(Vehicle_to_Case_Pitch / (ECTOS::CONSTANTS::pi)));
			bb.setOffset(44);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(Vehicle_to_Case_Heading / (ECTOS::CONSTANTS::pi)));
		}

	if ( ULV_MeasFrametoVehFrame == 1 ) {
			bb.setOffset(96);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(Meas_to_Vehicle_C11 / (std::pow(2, -31))));
			bb.setOffset(100);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(Meas_to_Vehicle_C12 / (std::pow(2, -31))));
			bb.setOffset(104);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(Meas_to_Vehicle_C13 / (std::pow(2, -31))));
			bb.setOffset(108);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(Meas_to_Vehicle_C21 / (std::pow(2, -31))));
			bb.setOffset(112);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(Meas_to_Vehicle_C22 / (std::pow(2, -31))));
			bb.setOffset(116);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(Meas_to_Vehicle_C23 / (std::pow(2, -31))));
			bb.setOffset(120);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(Meas_to_Vehicle_C31 / (std::pow(2, -31))));
			bb.setOffset(124);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(Meas_to_Vehicle_C32 / (std::pow(2, -31))));
			bb.setOffset(128);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(Meas_to_Vehicle_C33 / (std::pow(2, -31))));
		}

	if ( ULV_MeasFrametoVehFrame == 2 ) {
			bb.setOffset(96);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(Meas_to_Vehicle_Roll / (ECTOS::CONSTANTS::pi)));
			bb.setOffset(100);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(Meas_to_Vehicle_Pitch / (ECTOS::CONSTANTS::pi)));
			bb.setOffset(104);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(Meas_to_Vehicle_Heading / (ECTOS::CONSTANTS::pi)));
		}
	Checksum = computeChecksum((uint32_t*)buffer, 45);
	bb.setOffset(cIndex);		bb.put(Checksum);
	return status_ok;
}

int Msg_1004::Deserialize(unsigned char * buffer, const unsigned int bufferSize)
{
	bool status_ok = true;
	if (bufferSize < 180) return -2;

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

	uint32_t bitfieldAtByte16 = 0; // temporary variable holding the bitfield
	bb.setOffset(16);	bb.get(bitfieldAtByte16);
	ULV_GPSSigLatencyL1 = ECTOS::BIT_UTILITIES::UnPackBool(bitfieldAtByte16, 0,  status_ok);
	ULV_GPSSigLatencyL2 = ECTOS::BIT_UTILITIES::UnPackBool(bitfieldAtByte16, 1,  status_ok);
	ULV_VehFrametoCaseFrame = ECTOS::BIT_UTILITIES::UnPack(bitfieldAtByte16, 4, 5,  status_ok);
	ULV_Vehicle_Case_FrameLeverArm = ECTOS::BIT_UTILITIES::UnPack(bitfieldAtByte16, 6, 7,  status_ok);
	ULV_LeverArmToPort4GPSAnt = ECTOS::BIT_UTILITIES::UnPack(bitfieldAtByte16, 8, 9,  status_ok);
	ULV_MeasFrametoVehFrame = ECTOS::BIT_UTILITIES::UnPack(bitfieldAtByte16, 10, 11,  status_ok);
	ULV_LeverArmToICDNav = ECTOS::BIT_UTILITIES::UnPack(bitfieldAtByte16, 12, 13,  status_ok);
	ULV_LeverArmToICDGPSAntenna = ECTOS::BIT_UTILITIES::UnPack(bitfieldAtByte16, 14, 15,  status_ok);
	ULV_LocationOfAutopilotFCData = ECTOS::BIT_UTILITIES::UnPack(bitfieldAtByte16, 16, 17,  status_ok);
	bb.setOffset(20);	bb.get(L1SignalLatency);
	bb.setOffset(24);	bb.get(L2SignalLatency);
	bb.setOffset(72);	bb.get(VehCaseLvrArm_x);
	bb.setOffset(76);	bb.get(VehCaseLvrArm_y);
	bb.setOffset(80);	bb.get(VehCaseLvrArm_z);
	bb.setOffset(84);	bb.get(VehGPSPort4LvrArm_x);
	bb.setOffset(88);	bb.get(VehGPSPort4LvrArm_y);
	bb.setOffset(92);	bb.get(VehGPSPort4LvrArm_z);
	bb.setOffset(132);	bb.get(VehICDNavLvrArm_x);
	bb.setOffset(136);	bb.get(VehICDNavLvrArm_y);
	bb.setOffset(140);	bb.get(VehICDNavLvrArm_z);
	bb.setOffset(144);	bb.get(VehGPSPort4LvrArm_x_2);
	bb.setOffset(148);	bb.get(VehGPSPort4LvrArm_y_2);
	bb.setOffset(152);	bb.get(VehGPSPort4LvrArm_z_2);

	if ( ULV_VehFrametoCaseFrame == 1 ) {
		 	bb.setOffset(36);	Vehicle_to_Case_C11 = static_cast<float>(bb.get<int32_t>()) * (std::pow(2, -31));
		 	bb.setOffset(40);	Vehicle_to_Case_C12 = static_cast<float>(bb.get<int32_t>()) * (std::pow(2, -31));
		 	bb.setOffset(44);	Vehicle_to_Case_C13 = static_cast<float>(bb.get<int32_t>()) * (std::pow(2, -31));
		 	bb.setOffset(48);	Vehicle_to_Case_C21 = static_cast<float>(bb.get<int32_t>()) * (std::pow(2, -31));
		 	bb.setOffset(52);	Vehicle_to_Case_C22 = static_cast<float>(bb.get<int32_t>()) * (std::pow(2, -31));
		 	bb.setOffset(56);	Vehicle_to_Case_C23 = static_cast<float>(bb.get<int32_t>()) * (std::pow(2, -31));
		 	bb.setOffset(60);	Vehicle_to_Case_C31 = static_cast<float>(bb.get<int32_t>()) * (std::pow(2, -31));
		 	bb.setOffset(64);	Vehicle_to_Case_C32 = static_cast<float>(bb.get<int32_t>()) * (std::pow(2, -31));
		 	bb.setOffset(68);	Vehicle_to_Case_C33 = static_cast<float>(bb.get<int32_t>()) * (std::pow(2, -31));
}

	if ( ULV_VehFrametoCaseFrame == 2 ) {
		 	bb.setOffset(36);	Vehicle_to_Case_Roll = static_cast<float>(bb.get<int32_t>()) * (ECTOS::CONSTANTS::pi);
		 	bb.setOffset(40);	Vehicle_to_Case_Pitch = static_cast<float>(bb.get<int32_t>()) * (ECTOS::CONSTANTS::pi);
		 	bb.setOffset(44);	Vehicle_to_Case_Heading = static_cast<float>(bb.get<int32_t>()) * (ECTOS::CONSTANTS::pi);
}

	if ( ULV_MeasFrametoVehFrame == 1 ) {
		 	bb.setOffset(96);	Meas_to_Vehicle_C11 = static_cast<float>(bb.get<int32_t>()) * (std::pow(2, -31));
		 	bb.setOffset(100);	Meas_to_Vehicle_C12 = static_cast<float>(bb.get<int32_t>()) * (std::pow(2, -31));
		 	bb.setOffset(104);	Meas_to_Vehicle_C13 = static_cast<float>(bb.get<int32_t>()) * (std::pow(2, -31));
		 	bb.setOffset(108);	Meas_to_Vehicle_C21 = static_cast<float>(bb.get<int32_t>()) * (std::pow(2, -31));
		 	bb.setOffset(112);	Meas_to_Vehicle_C22 = static_cast<float>(bb.get<int32_t>()) * (std::pow(2, -31));
		 	bb.setOffset(116);	Meas_to_Vehicle_C23 = static_cast<float>(bb.get<int32_t>()) * (std::pow(2, -31));
		 	bb.setOffset(120);	Meas_to_Vehicle_C31 = static_cast<float>(bb.get<int32_t>()) * (std::pow(2, -31));
		 	bb.setOffset(124);	Meas_to_Vehicle_C32 = static_cast<float>(bb.get<int32_t>()) * (std::pow(2, -31));
		 	bb.setOffset(128);	Meas_to_Vehicle_C33 = static_cast<float>(bb.get<int32_t>()) * (std::pow(2, -31));
}

	if ( ULV_MeasFrametoVehFrame == 2 ) {
		 	bb.setOffset(96);	Meas_to_Vehicle_Roll = static_cast<float>(bb.get<int32_t>()) * (ECTOS::CONSTANTS::pi);
		 	bb.setOffset(100);	Meas_to_Vehicle_Pitch = static_cast<float>(bb.get<int32_t>()) * (ECTOS::CONSTANTS::pi);
		 	bb.setOffset(104);	Meas_to_Vehicle_Heading = static_cast<float>(bb.get<int32_t>()) * (ECTOS::CONSTANTS::pi);
}
	bb.setOffset(12);	bb.get(Checksum);
	uint32_t computedCRC = computeChecksum((uint32_t*)buffer, 45);
	int retVal = 0;
	retVal = constCheck;
	if(computedCRC != Checksum)
		retVal = -1;
	return retVal;
}

