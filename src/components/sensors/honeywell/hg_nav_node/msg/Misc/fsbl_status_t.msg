# First stage bootloader status
# custom fsbl_status_t
	uint16 fsbl_boot_rom_error_code  # Boot ROM Error Code - Describes errors that occur during the boot proceess.
	bool fslb_system_wdt_reset  # System Watch Dog Timer Reset - Last reset was due to system watchdog timeout, if set.
	bool fsbl_apu_0_wdt_reset  # APU 0 Watch Dog Timer Reset - Last reset was due to APU watchdog timer 0, if set.
	bool fsbl_apu_1_wdt_reset  # APU 1 Watch Dog Timer Reset - Last reset was due to APU watchdog timer 1, if set.
	bool fsbl_software_reset  # Software Reset - Last reset was due to SLC soft reset, if set.
	bool fsbl_debug_system_reset  # Debug System Reset - Last reset was due to debug system, if set.
	bool fsbl_external_system_reset  # External system Reset - Last reset was due to SRST_B (soft reset), if set.
	bool fsbl_power_on_reset  # Power On Reset - Last reset was due to POR (power on reset), if set.
	uint8 fsbl_reboot_state  # Reboot State - Last known reset reason.
	bool fsbl_pmic_boost_status  # PMIC Boost Status Enable 0 = PMIC Boost Circuits Disabled 1 = PMIC Boost Circuits Enabled
	bool fsbl_gpga_initialization_done  # FPGA Initialization Done (1 = Done)
