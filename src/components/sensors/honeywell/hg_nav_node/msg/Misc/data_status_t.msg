# detailed data input status
# custom data_status_t
	uint8 portNum  # Port number of the port type (i.e. portType = input_UART && portNum = 1 => COM1)
	port_type_t portType  # Type of port being used to source the data
	bool active  # (0 = data NOT being received | 1 = data is being received) indicates whether the data is present
