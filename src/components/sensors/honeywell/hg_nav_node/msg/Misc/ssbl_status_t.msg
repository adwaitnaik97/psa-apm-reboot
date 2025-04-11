# Second stage boot loader status
# custom ssbl_status_t
	bool flash_register_init_table_missing  # Flash Loader Table Missing
	bool flash_register_init_file_checksum_error  # Flash Loader Table File Checksum
	bool flash_table_engry_checksum_error  # Flash Loader Table ENTRY Checksum Error
	bool flash_table_entry_address_error  # Flash Loader Table ENTRY Address Error
	bool ssbl_register_init_table_missing  # Register Initialization Table Missing
	bool ssbl_register_init_file_checksum_error  # Register Initialization Table File Checksum Error
	bool ssbl_table_engry_checksum_error  # Register Initialization Table ENTRY Checksum Error
	bool ssbl_table_entry_address_error  # Register Initialization Table ENTRY Address Error
