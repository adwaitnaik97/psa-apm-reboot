#ifndef __HGuideAPI_Msg_2011_h__
#define __HGuideAPI_Msg_2011_h__
#pragma once

#include <cstdint>

#include <include/ins_gnss_summary_t.h>
#include <include/hgnsi_message_word1_t.h>
#include <include/hgnsi_message_word2_t.h>

// Imu Built-in Test status
// Summary status information on the internal IMU
struct imu_bit_status_t
{
	bool imu_failed; // IMU Failed
	bool processor_failed; // Processor Failed
	bool memory_failed; // Memory Failed
	bool other_failed; // All others Failed
	bool accelerometer_failed; // Accelerometer Failed
	bool gyro_failed; // Gyro Failed
	bool external_sync_fault; // External Sync Fault [Reserved]
	
	// IMU BIT Mode:
	// 0 = Power Up
	// 1 = Continuous
	// 2 = Initiated
	// 3 = Undefined
	// Note: IBIT response is less than 1 second (~30 msec) and may be missed by 1 Hz message rate.
	bool imu_bit_mode;

	void Default()
	{
		imu_failed = 0;
		processor_failed = 0;
		memory_failed = 0;
		other_failed = 0;
		accelerometer_failed = 0;
		gyro_failed = 0;
		external_sync_fault = 0;
		imu_bit_mode = 0;
	}
};

// List of currently utilized aiding sources
// note: this list is updated based on ZUPT detection (a switch between Dynamic and Static aiding sources is visible here)
struct aiding_sources_t
{
	bool zero_velocity; // 0 = Not Enabled | 1 = Enabled
	bool GNSS_PVT_P; // 0 = Not Enabled | 1 = Enabled
	bool GNSS_PVT_V; // 0 = Not Enabled | 1 = Enabled
	bool GNSS_ATT_A; // 0 = Not Enabled | 1 = Enabled
	bool GNSS_pseudo_range; // 0 = Not Enabled | 1 = Enabled
	bool GNSS_delta_range; // 0 = Not Enabled | 1 = Enabled
	bool baro_altitude; // 0 = Not Enabled | 1 = Enabled
	bool position_match; // 0 = Not Enabled | 1 = Enabled
	bool velocity_match; // 0 = Not Enabled | 1 = Enabled
	bool attitude_match; // 0 = Not Enabled | 1 = Enabled
	bool mag_heading; // 0 = Not Enabled | 1 = Enabled
	bool static_heading; // 0 = Not Enabled | 1 = Enabled
	bool motion_detect; // 0 = Not Enabled | 1 = Enabled
	bool odometer; // 0 = Not Enabled | 1 = Enabled
	bool land_constraints; // 0 = Not Enabled | 1 = Enabled

	void Default()
	{
		zero_velocity = 0;
		GNSS_PVT_P = 0;
		GNSS_PVT_V = 0;
		GNSS_ATT_A = 0;
		GNSS_pseudo_range = 0;
		GNSS_delta_range = 0;
		baro_altitude = 0;
		position_match = 0;
		velocity_match = 0;
		attitude_match = 0;
		mag_heading = 0;
		static_heading = 0;
		motion_detect = 0;
		odometer = 0;
		land_constraints = 0;
	}
};

// Information about channel 0 to 31 validity
struct gps_channel_validity_t
{
	bool channel_0; // 0 = Invalid | 1 = Valid
	bool channel_1; // 0 = Invalid | 1 = Valid
	bool channel_2; // 0 = Invalid | 1 = Valid
	bool channel_3; // 0 = Invalid | 1 = Valid
	bool channel_4; // 0 = Invalid | 1 = Valid
	bool channel_5; // 0 = Invalid | 1 = Valid
	bool channel_6; // 0 = Invalid | 1 = Valid
	bool channel_7; // 0 = Invalid | 1 = Valid
	bool channel_8; // 0 = Invalid | 1 = Valid
	bool channel_9; // 0 = Invalid | 1 = Valid
	bool channel_10; // 0 = Invalid | 1 = Valid
	bool channel_11; // 0 = Invalid | 1 = Valid
	bool channel_12; // 0 = Invalid | 1 = Valid
	bool channel_13; // 0 = Invalid | 1 = Valid
	bool channel_14; // 0 = Invalid | 1 = Valid
	bool channel_15; // 0 = Invalid | 1 = Valid
	bool channel_16; // 0 = Invalid | 1 = Valid
	bool channel_17; // 0 = Invalid | 1 = Valid
	bool channel_18; // 0 = Invalid | 1 = Valid
	bool channel_19; // 0 = Invalid | 1 = Valid
	bool channel_20; // 0 = Invalid | 1 = Valid
	bool channel_21; // 0 = Invalid | 1 = Valid
	bool channel_22; // 0 = Invalid | 1 = Valid
	bool channel_23; // 0 = Invalid | 1 = Valid
	bool channel_24; // 0 = Invalid | 1 = Valid
	bool channel_25; // 0 = Invalid | 1 = Valid
	bool channel_26; // 0 = Invalid | 1 = Valid
	bool channel_27; // 0 = Invalid | 1 = Valid
	bool channel_28; // 0 = Invalid | 1 = Valid
	bool channel_29; // 0 = Invalid | 1 = Valid
	bool channel_30; // 0 = Invalid | 1 = Valid
	bool channel_31; // 0 = Invalid | 1 = Valid

	void Default()
	{
		channel_0 = 0;
		channel_1 = 0;
		channel_2 = 0;
		channel_3 = 0;
		channel_4 = 0;
		channel_5 = 0;
		channel_6 = 0;
		channel_7 = 0;
		channel_8 = 0;
		channel_9 = 0;
		channel_10 = 0;
		channel_11 = 0;
		channel_12 = 0;
		channel_13 = 0;
		channel_14 = 0;
		channel_15 = 0;
		channel_16 = 0;
		channel_17 = 0;
		channel_18 = 0;
		channel_19 = 0;
		channel_20 = 0;
		channel_21 = 0;
		channel_22 = 0;
		channel_23 = 0;
		channel_24 = 0;
		channel_25 = 0;
		channel_26 = 0;
		channel_27 = 0;
		channel_28 = 0;
		channel_29 = 0;
		channel_30 = 0;
		channel_31 = 0;
	}
};

// Second stage boot loader status
struct ssbl_status_t
{
	bool flash_register_init_table_missing; // Flash Loader Table Missing
	bool flash_register_init_file_checksum_error; // Flash Loader Table File Checksum
	bool flash_table_engry_checksum_error; // Flash Loader Table ENTRY Checksum Error
	bool flash_table_entry_address_error; // Flash Loader Table ENTRY Address Error
	bool ssbl_register_init_table_missing; // Register Initialization Table Missing
	bool ssbl_register_init_file_checksum_error; // Register Initialization Table File Checksum Error
	bool ssbl_table_engry_checksum_error; // Register Initialization Table ENTRY Checksum Error
	bool ssbl_table_entry_address_error; // Register Initialization Table ENTRY Address Error

	void Default()
	{
		flash_register_init_table_missing = 0;
		flash_register_init_file_checksum_error = 0;
		flash_table_engry_checksum_error = 0;
		flash_table_entry_address_error = 0;
		ssbl_register_init_table_missing = 0;
		ssbl_register_init_file_checksum_error = 0;
		ssbl_table_engry_checksum_error = 0;
		ssbl_table_entry_address_error = 0;
	}
};

// GNSS receiver aiding status
struct gnssr_aiding_status_t
{
	bool gnssr_receiver_inertial_aiding_capable; // Receiver Inertial Aiding (1 = Enabled)
	bool gnssr_inertial_aiding_valid; // Inertial Aiding Valid (1 = Valid)
	bool gnssr_inertial_aiding_in_use; // Inertial Aiding Use (1 = In Use)
	bool gnssr_drs_aiding_valid; // DRS Aiding Valid (1 = Valid)
	bool gnssr_drs_aiding_in_use; // DRS Aiding Use (1 = In Use)
	bool gnssr_baro_aiding_valid; // BARO Aiding Valid (1 = Valid)
	bool gnssr_baro_aiding_in_use; // BARO Aiding Use (1 = In Use)
	bool gnssr_tas_aiding_valid; // TAS Aiding Valid (1 = Valid)
	bool gnssr_tas_aiding_in_use; // TAS Aiding Use (1 = In Use)
	bool gnssr_mag_aiding_valid; // MAG Aiding Valid (1 = Valid)
	bool gnssr_mag_aiding_in_use; // MAG Aiding Use (1 = In Use)
	bool gnssr_attitude_aiding_valid; // Attitude Aiding Valid (1 = Valid)
	bool gnssr_attitude_aiding_in_use; // Attitude Aiding Use (1 = In Use)
	bool gnssr_std_frequency_valid; // STD Frequency Valid (1 = Valid)
	bool gnssr_std_frequency_in_use; // STD Frequency Use (1 = In Use)
	bool gnssr_std_time_valid; // STD Time Valid (1 = Valid)
	bool gnssr_std_time_in_use; // STD Time Use (1 = In Use)

	void Default()
	{
		gnssr_receiver_inertial_aiding_capable = 0;
		gnssr_inertial_aiding_valid = 0;
		gnssr_inertial_aiding_in_use = 0;
		gnssr_drs_aiding_valid = 0;
		gnssr_drs_aiding_in_use = 0;
		gnssr_baro_aiding_valid = 0;
		gnssr_baro_aiding_in_use = 0;
		gnssr_tas_aiding_valid = 0;
		gnssr_tas_aiding_in_use = 0;
		gnssr_mag_aiding_valid = 0;
		gnssr_mag_aiding_in_use = 0;
		gnssr_attitude_aiding_valid = 0;
		gnssr_attitude_aiding_in_use = 0;
		gnssr_std_frequency_valid = 0;
		gnssr_std_frequency_in_use = 0;
		gnssr_std_time_valid = 0;
		gnssr_std_time_in_use = 0;
	}
};

// First stage bootloader status
struct fsbl_status_t
{
	uint16_t fsbl_boot_rom_error_code; // Boot ROM Error Code - Describes errors that occur during the boot proceess.
	bool fslb_system_wdt_reset; // System Watch Dog Timer Reset - Last reset was due to system watchdog timeout, if set.
	bool fsbl_apu_0_wdt_reset; // APU 0 Watch Dog Timer Reset - Last reset was due to APU watchdog timer 0, if set.
	bool fsbl_apu_1_wdt_reset; // APU 1 Watch Dog Timer Reset - Last reset was due to APU watchdog timer 1, if set.
	bool fsbl_software_reset; // Software Reset - Last reset was due to SLC soft reset, if set.
	bool fsbl_debug_system_reset; // Debug System Reset - Last reset was due to debug system, if set.
	bool fsbl_external_system_reset; // External system Reset - Last reset was due to SRST_B (soft reset), if set.
	bool fsbl_power_on_reset; // Power On Reset - Last reset was due to POR (power on reset), if set.
	uint8_t fsbl_reboot_state; // Reboot State - Last known reset reason.
	bool fsbl_pmic_boost_status; // PMIC Boost Status Enable 0 = PMIC Boost Circuits Disabled 1 = PMIC Boost Circuits Enabled
	bool fsbl_gpga_initialization_done; // FPGA Initialization Done (1 = Done)

	void Default()
	{
		fsbl_boot_rom_error_code = 0;
		fslb_system_wdt_reset = 0;
		fsbl_apu_0_wdt_reset = 0;
		fsbl_apu_1_wdt_reset = 0;
		fsbl_software_reset = 0;
		fsbl_debug_system_reset = 0;
		fsbl_external_system_reset = 0;
		fsbl_power_on_reset = 0;
		fsbl_reboot_state = 0;
		fsbl_pmic_boost_status = 0;
		fsbl_gpga_initialization_done = 0;
	}
};

// INS/GNSS System status summary
// Summary bits for each INS/GNSS sub-system. In case of fail, investigate specific sub-system BIT
struct system_bit_summary_t
{
	bool Go_NoGo_BIT_Summary; // Go/No Go BIT Summary
	bool MPB_Subsystem_BIT_Summary; // MPB Subsystem BIT Summary
	bool GPS_Subsystem_BIT_Summary; // GPS Subsystem BIT Summary
	bool Gyro_Subsystem_BIT_Summary; // Gyro Subsystem BIT Summary
	bool Accelerometer_Subsystem_BIT_Summary; // Accelerometer Subsystem BIT Summary
	bool IMU_Subsystem_BIT_Summary; // IMU Subsystem BIT Summary

	void Default()
	{
		Go_NoGo_BIT_Summary = 0;
		MPB_Subsystem_BIT_Summary = 0;
		GPS_Subsystem_BIT_Summary = 0;
		Gyro_Subsystem_BIT_Summary = 0;
		Accelerometer_Subsystem_BIT_Summary = 0;
		IMU_Subsystem_BIT_Summary = 0;
	}
};

// Mission Computer Board Built-in test results
struct mcb_bit_status_t
{
	bool mcb_INS_Executable_Code_CRC; // INS Executable Code CRC
	bool mcb_INS_Configuration_File_CRC; // INS Configuration File CRC
	bool mcb_Coefficient; // Coefficient Table CRC
	bool mcb_IMU_Configuration_Table_CRC; // IMU Configuration Table CRC
	bool mcb_IMU_Normal_Mode_Code_CRC; // IMU Normal Mode Code CRC
	bool mcb_External_IMU_GFR_Sync; // External IMU GFR Sync
	bool mcb_External_IMU_CFR_Sync; // External IMU CFR Sync
	bool mcb_IMU_FPGA; // IMU FPGA
	bool mcb_IMU_Serial_Communications; // IMU Serial Communications
	bool mcb_IMU_SPI; // IMU SPI Bus Transfer
	bool mcb_INS_Memory_Interface; // INS Memory Interface
	bool mcb_INS_Memory_Static_Data; // INS Memory Static Data
	bool mcb_IMU_Non_Comprehensive_RAM; // IMU Non-Comprehensive RAM
	bool mcb_INS_Temperature; // INS Temperature
	bool mcb_IMU_Loop_Completion; // IMU Loop Completion
	bool mcb_IMU_WDT; // IMU WDT
	bool mcb_IMU_Processor; // IMU Processor
	bool mcb_INS_Processor; // INS Processor
	bool mcb_Processor_Interrupt; // Processor Interrupt
	bool mcb_Frame_Timer; // Frame Timer
	bool mcb_Time_Tagging; // Time Tagging
	bool mcb_IMU_Stack_Overflow; // IMU Stack Overflow
	bool mcb_Power_Up_BIT_Timer; // Power Up BIT Timer
	bool mcb_INS_Stack_Overflow; // INS Stack Overflow
	bool mcb_Programmable_Logic_Read_Back; // Programmable Logic Read Back
	bool mcb_INS_Loop_Completion; // INS Loop Completion
	bool mcb_First_Stage_Boot_Loader; // First Stage Boot Loader
	bool mcb_Flash_Loader_Table; // Flash Loader Table
	bool mcb_Register_Initialization_Table; // Register Initialization Table

	void Default()
	{
		mcb_INS_Executable_Code_CRC = 0;
		mcb_INS_Configuration_File_CRC = 0;
		mcb_Coefficient = 0;
		mcb_IMU_Configuration_Table_CRC = 0;
		mcb_IMU_Normal_Mode_Code_CRC = 0;
		mcb_External_IMU_GFR_Sync = 0;
		mcb_External_IMU_CFR_Sync = 0;
		mcb_IMU_FPGA = 0;
		mcb_IMU_Serial_Communications = 0;
		mcb_IMU_SPI = 0;
		mcb_INS_Memory_Interface = 0;
		mcb_INS_Memory_Static_Data = 0;
		mcb_IMU_Non_Comprehensive_RAM = 0;
		mcb_INS_Temperature = 0;
		mcb_IMU_Loop_Completion = 0;
		mcb_IMU_WDT = 0;
		mcb_IMU_Processor = 0;
		mcb_INS_Processor = 0;
		mcb_Processor_Interrupt = 0;
		mcb_Frame_Timer = 0;
		mcb_Time_Tagging = 0;
		mcb_IMU_Stack_Overflow = 0;
		mcb_Power_Up_BIT_Timer = 0;
		mcb_INS_Stack_Overflow = 0;
		mcb_Programmable_Logic_Read_Back = 0;
		mcb_INS_Loop_Completion = 0;
		mcb_First_Stage_Boot_Loader = 0;
		mcb_Flash_Loader_Table = 0;
		mcb_Register_Initialization_Table = 0;
	}
};

// GPS board status
// The GPS Receiver has 96 fault words, each 16 bits wide. This message can output 3 unique GPS fault words per message. If there are more than 3 GPS faults words, multiple occurrences of this message are required - using the 8 MSB as an identifier.
struct gps_bit_status_t
{
	bool gpssw_GPS_Function; // GPS Function
	bool gpssw_GPS_Communication; // GPS Communication
	bool gpssw_GPS_Time_Mark; // GPS Time Mark
	bool gpssw_GPS_T20_Synchronization; // GPS T20 Synchronization

	void Default()
	{
		gpssw_GPS_Function = 0;
		gpssw_GPS_Communication = 0;
		gpssw_GPS_Time_Mark = 0;
		gpssw_GPS_T20_Synchronization = 0;
	}
};

// Gyroscope status
// detailed information on Gyroscope systems status
struct gyro_bit_status_t
{
	bool gyro_status_Gyro_Run_X; // Gyro Run X
	bool gyro_status_Gyro_Run_Y; // Gyro Run Y
	bool gyro_status_Gyro_Run_Z; // Gyro Run Z
	bool gyro_status_Gyro_Register_Read_Back_X; // Gyro Register Read Back X
	bool gyro_status_Gyro_Register_Read_Back_Y; // Gyro Register Read Back Y
	bool gyro_status_Gyro_Register_Read_Back_Z; // Gyro Register Read Back Z
	bool gyro_status_Gyro_Temperature_X; // Gyro Temperature X
	bool gyro_status_Gyro_Temperature_Y; // Gyro Temperature Y
	bool gyro_status_Gyro_Temperature_Z; // Gyro Temperature Z
	bool gyro_status_Gyro_AGC_X; // Gyro AGC X
	bool gyro_status_Gyro_AGC_Y; // Gyro AGC Y
	bool gyro_status_Gyro_AGC_Z; // Gyro AGC Z
	bool gyro_status_Gyro_Motor_Bias_X; // Gyro Motor Bias X
	bool gyro_status_Gyro_Motor_Bias_Y; // Gyro Motor Bias Y
	bool gyro_status_Gyro_Motor_Bias_Z; // Gyro Motor Bias Z
	bool gyro_status_Gyro_NVREF_X; // Gyro NVREF X
	bool gyro_status_Gyro_NVREF_Y; // Gyro NVREF Y
	bool gyro_status_Gyro_NVREF_Z; // Gyro NVREF Z
	bool gyro_status_Gyro_Sensor_X; // Gyro Sensor X
	bool gyro_status_Gyro_Sensor_Y; // Gyro Sensor Y
	bool gyro_status_Gyro_Sensor_Z; // Gyro Sensor Z

	void Default()
	{
		gyro_status_Gyro_Run_X = 0;
		gyro_status_Gyro_Run_Y = 0;
		gyro_status_Gyro_Run_Z = 0;
		gyro_status_Gyro_Register_Read_Back_X = 0;
		gyro_status_Gyro_Register_Read_Back_Y = 0;
		gyro_status_Gyro_Register_Read_Back_Z = 0;
		gyro_status_Gyro_Temperature_X = 0;
		gyro_status_Gyro_Temperature_Y = 0;
		gyro_status_Gyro_Temperature_Z = 0;
		gyro_status_Gyro_AGC_X = 0;
		gyro_status_Gyro_AGC_Y = 0;
		gyro_status_Gyro_AGC_Z = 0;
		gyro_status_Gyro_Motor_Bias_X = 0;
		gyro_status_Gyro_Motor_Bias_Y = 0;
		gyro_status_Gyro_Motor_Bias_Z = 0;
		gyro_status_Gyro_NVREF_X = 0;
		gyro_status_Gyro_NVREF_Y = 0;
		gyro_status_Gyro_NVREF_Z = 0;
		gyro_status_Gyro_Sensor_X = 0;
		gyro_status_Gyro_Sensor_Y = 0;
		gyro_status_Gyro_Sensor_Z = 0;
	}
};

// Accelerometer status
// detailed information on Accelerometer systems status
struct accel_bit_status_t
{
	bool accel_status_Accel_Register_Read_Back_X; // Accel Register Read Back X
	bool accel_status_Accel_Register_Read_Back_Y; // Accel Register Read Back Y
	bool accel_status_Accel_Register_Read_Back_Z; // Accel Register Read Back Z
	bool accel_status_Accel_Temperature_X; // Accel Temperature X
	bool accel_status_Accel_Temperature_Y; // Accel Temperature Y
	bool accel_status_Accel_Temperature_Z; // Accel Temperature Z
	bool accel_status_Accel_Sensor_X; // Accel Sensor X
	bool accel_status_Accel_Sensor_Y; // Accel Sensor Y
	bool accel_status_Accel_Sensor_Z; // Accel Sensor Z
	bool accel_status_Accel_AC_Stim_X; // Accel AC Stim X
	bool accel_status_Accel_AC_Stim_Y; // Accel AC Stim Y
	bool accel_status_Accel_AC_Stim_Z; // Accel AC Stim Z
	bool accel_status_Accel_Analog_Continuity_X; // Accel Analog Continuity X
	bool accel_status_Accel_Analog_Continuity_Y; // Accel Analog Continuity Y
	bool accel_status_Accel_Analog_Continuity_Z; // Accel Analog Continuity Z
	bool accel_status_Accel_Bypass_X; // Accel Bypass X
	bool accel_status_Accel_Bypass_Y; // Accel Bypass Y
	bool accel_status_Accel_Bypass_Z; // Accel Bypass Z
	bool accel_status_Accel_Level_X; // Accel Level X
	bool accel_status_Accel_Level_Y; // Accel Level Y
	bool accel_status_Accel_Level_Z; // Accel Level Z

	void Default()
	{
		accel_status_Accel_Register_Read_Back_X = 0;
		accel_status_Accel_Register_Read_Back_Y = 0;
		accel_status_Accel_Register_Read_Back_Z = 0;
		accel_status_Accel_Temperature_X = 0;
		accel_status_Accel_Temperature_Y = 0;
		accel_status_Accel_Temperature_Z = 0;
		accel_status_Accel_Sensor_X = 0;
		accel_status_Accel_Sensor_Y = 0;
		accel_status_Accel_Sensor_Z = 0;
		accel_status_Accel_AC_Stim_X = 0;
		accel_status_Accel_AC_Stim_Y = 0;
		accel_status_Accel_AC_Stim_Z = 0;
		accel_status_Accel_Analog_Continuity_X = 0;
		accel_status_Accel_Analog_Continuity_Y = 0;
		accel_status_Accel_Analog_Continuity_Z = 0;
		accel_status_Accel_Bypass_X = 0;
		accel_status_Accel_Bypass_Y = 0;
		accel_status_Accel_Bypass_Z = 0;
		accel_status_Accel_Level_X = 0;
		accel_status_Accel_Level_Y = 0;
		accel_status_Accel_Level_Z = 0;
	}
};


// 0x2011 : INS Status Information
//
// Detailed status of all sub systems in the INS.
// 
class HGUIDE_DLL Msg_2011
{
public:
	Msg_2011();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 220;}

public:
	static const uint32_t AddressId = 0xA5C381FF; // Start of Message
	static const uint32_t MessageId = 0x2011; // Message ID
	static const uint32_t MessageLength = 55; // Message Length in 32-bit Words
	uint32_t Checksum; // Checksum
	uint32_t inertial_data_source; // [INTERNAL] Inertial Data Source | 0 = Internal ISA | 1 = External Data
	uint8_t navigation_initialization_status; // Navigation Initialization Status: 0 = Waiting for Nav Init Data | 1 = Nav Init Data Received
	uint8_t navErrorModel; // Navigation Error Model: 0 = Undefined | 1 = Small Azimuth | 2 = Large Azimuth
	
	// [INTERNAL] INS Mode - Redundant Information
 	// 0 = Reserved
 	// 1 = Standby
 	// 2 = Coarse Level
 	// 3 = Unaided Navigation (Free Inertial)
 	// 4 = Aided Navigation (Blended GPS)
 	// 5 - 9 = Reserved
	uint8_t INSMode;
	ins_gnss_summary_t InsGnssSummary; // INS/GPS BIT Summary
	double systemTov; // Time since Power Up - Always Valid [sec]
	double gpsTov; // [sec] GPS Time of Validity in current week (Sunday 00:00 UTC). When invalid, GPS time = INS/GPS Internal System Time
	int32_t gps_week; // Number of weeks since Jan 6, 1980 GPS Week = -1 indicates GPS Week Unknown/Invalid
	int32_t power_cycle_count; // Power Cycle Count
	double eti; // [sec] Total Device Elapsed Time Indicator (ETI)
	float ins_device_temperature; // [deg C]
	
	// BIT Mode:
 	// 0 = Unknown
 	// 1 = PowerUp
 	// 2 = Continuous
 	// 3 - 11 = Internal
	int32_t bitMode;
	
	// INS / Blended Figure of Merit (FOM)
 	// Refer to 0x6403 Standard Deviation numbers for position accuarcy
 	// 0 : Reserved
 	// 1 : Less than 26 meters
 	// 2 : 26 - 50 meters
 	// 3 : 51 - 75 meters
 	// 4 : 76 - 100 meters
 	// 5 : 101 - 200 meters
 	// 6 : 201 - 500 meters
 	// 7 : 501 - 1000 meters
 	// 8 : 1001 - 5000 meters
 	// 9 : Greater than 5000 meters
	int32_t ins_blended_fom;
	
	// GPS Figure of Merit (FOM)
 	// Refer to 0x6108 Standard Deviation numbers for RAW GNSS position accuarcy
 	// 0 : Reserved
 	// 1 : Less than 26 meters
 	// 2 : 26 - 50 meters
 	// 3 : 51 - 75 meters
 	// 4 : 76 - 100 meters
 	// 5 : 101 - 200 meters
 	// 6 : 201 - 500 meters
 	// 7 : 501 - 1000 meters
 	// 8 : 1001 - 5000 meters
 	// 9 : Greater than 5000 meters
	int32_t gps_fom;
	
	// UTC Time Figure of Merit (TFOM)
	// 0 : External Time source indicates normal operation
	// 1 : Time Accuracy <= 1 nsec
	// 2 : Time Accuracy <= 10 nsec
	// 3 : Time Accuracy <= 100 nsec
	// 4 : Time Accuracy <= 1 usec
	// 5 : Time Accuracy <= 10 usec
	// 6 : Time Accuracy <= 100 usec
	// 7 : Time Accuracy <= 1 msec
	// 8 : Time Accuracy <= 10 msec
	// 9 : Time Accuracy Unknown/Fault
	int32_t utc_tfom;
	uint8_t MDT_stationary_instantaneous_signal; // [milli-radians] Motion Detect Threshold stationary instantaneous signal
	uint8_t MDT_stationary_accel_mag; // [cm] Motion Detect Threshold stationary accel magnitude
	uint8_t MDT_stationary_speed_stdv; // [m/s] Motion Detect Threshold stationary speed stdv
	uint8_t MDT_stationary_rotation_rate; // [milli-radian] Motion Detect Threshold stationary rotation rate
	hgnsi_message_word1_t MessageWord1; // Word 1 with message enable bits
	hgnsi_message_word2_t MessageWord2; // Word 2 with message enable bits
	imu_bit_status_t imu_bit_status; // Internal IMU Status
	aiding_sources_t aiding_sources; // Enabled Aiding Sources
	
	// Number of Satellites Utilized by Pseudorange/Deltarange measurement filter (The number of satellites utilized over the last second).
 	// pseudorange/deltarange aiding source has to be enabled for this number to be greater than 0
	int32_t number_of_satellites_in_blended_sol;
	
	// Pseudo Range (PR) Validity by GPS Receiver Channel (PR Validity over the last second).
	// Valid only if PR aiding source is enabled
	gps_channel_validity_t pr_validity_by_gps_receiver_channel;
	
	// Delta pseudo Range (PR) Validity by GPS Receiver Channel (DR Validity over the last second).
	// Valid only if PR aiding source is enabled
	gps_channel_validity_t dr_validity_by_gps_receiver_channel;
	ssbl_status_t ssbl_status; // [INTERNAL] - Second Stage Boot Loader (SSBL) Status Word
	
	// Solution Convergence: 0 = Solution Not Converged | 1 = Solution Converged
 	// based on the FOM of position (<= 1) and attitude (<=4)
	uint32_t solution_convergence;
	
	// Attitude Figure of Merit (FOM)
	// 0 : Reserved
	// 1 : Accuracy <= 1 mrad (0.06 deg)
	// 2 : Accuracy <= 2 mrad (0.12 deg)
	// 3 : Accuracy <= 4 mrad (0.23 deg)
	// 4 : Accuracy <= 8 mrad (0.46 deg)
	// 5 : Accuracy <= 16 mrad (0.92 deg)
	// 6 : Accuracy <= 32 mrad (1.84 deg)
	// 7 : Accuracy <= 64 mrad (3.67 deg)
	// 8 : Accuracy <= 128 mrad (7.34 deg)
	// 9 : Accuracy > 128 mrad (7.34 deg)
	int32_t attitude_fom;
	gnssr_aiding_status_t gnssr_aiding_status; // [INTERNAL] Status of GNSS Aiding sources
	fsbl_status_t fsbl_status; // [INTERNAL] First Stage Boot Loader (FSBL) Status Word 1
	system_bit_summary_t system_bit_summary; // System Built in Test status summary
	mcb_bit_status_t mcb_bit_status; // [INTERNAL] Mission Computer Board (MCB) Built in Test status word 1
	gps_bit_status_t gps_bit_status; // [INTERNAL] GPS Built in Test status
	gyro_bit_status_t gyro_bit_status; // [INTERNAL] Internal Gyroscope Built in Test status
	accel_bit_status_t accelerometer_bit_status; // [INTERNAL] Accelerometer status
	float accel_x_temperature_degC; // [degC] Temperature of X axis accel - not calibrated
	float accel_y_temperature_degC; // [degC] Temperature of Y axis accel - not calibrated
	float accel_z_temperature_degC; // [degC] Temperature of Z axis accel - not calibrated
	float gyro_x_temperature_degC; // [degC] Temperature of X axis gyro - not calibrated
	float gyro_y_temperature_degC; // [degC] Temperature of Y axis gyro - not calibrated
	float gyro_z_temperature_degC; // [degC] Temperature of Z axis gyro - not calibrated
	uint8_t MDTC_zero_vel_meas_noiseX; // [cm] Motion Detect Time Constant zero velocity measurement noise in X axis
	uint8_t MDTC_inv_nominal_signal_filter_fn; // [s] Motion Detect Time Constant inv nominal signal filter fn
	uint8_t MDTC_instant_signal_filter_fn; // [Hz] Motion Detect Time Constant instant signal filter fn
	uint8_t MDTC_settling_time; // [s] Motion Detect Time Constant settling time
};

#endif // __HGuideAPI_Msg_2011_h__
