# DVL Error Status bit fields 
# custom dvl_error_t
	bool data_retrieval_fifo  # (1=error, 0=ok) data retrieval fifo error
	bool data_retrieval_overflow  # (1=error, 0=ok) data retrieval overflow
	bool data_retrieval_overflow2  # (1=error, 0=ok) data retrieval overflow
	bool data_retrieval_underrun  # (1=error, 0=ok) data retrieval underrun
	bool data_retrieval_missing_samples  # (1=error, 0=ok) data retrieval samples missing
	bool sensor_read_failure  # (1=error, 0=ok) sensor read failure
	bool beam0_in_phase_tag_error  # (1=error, 0=ok) tag error beam 0 (in-phase)
	bool beam0_quad_phase_tag_error  # (1=error, 0=ok) tag error beam 0 (quadrature-phase)
	bool beam1_in_phase_tag_error  # (1=error, 0=ok) tag error beam 1 (in-phase)
	bool beam1_quad_phase_tag_error  # (1=error, 0=ok) tag error beam 1 (quadrature-phase)
	bool beam2_in_phase_tag_error  # (1=error, 0=ok) tag error beam 2 (in-phase)
	bool beam2_quad_phase_tag_error  # (1=error, 0=ok) tag error beam 2 (quadrature-phase)
	bool beam3_in_phase_tag_error  # (1=error, 0=ok) tag error beam 3 (in-phase)
	bool beam3_quad_phase_tag_error  # (1=error, 0=ok) tag error beam 3 (quadrature-phase)
