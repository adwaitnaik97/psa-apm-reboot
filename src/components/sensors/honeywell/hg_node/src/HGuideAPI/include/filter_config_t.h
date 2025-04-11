#ifndef __HGuideAPI_filter_config_t_h__
#define __HGuideAPI_filter_config_t_h__
#pragma once

//
// Data Filter Configuration
//
struct filter_config_t
{
	float cutoff_frequency; // cutoff frequency, normalized by filter rate
	bool enabled; // RESERVED (1 = filter enabled | 0 = filter disabled)

	void Default()
	{
		cutoff_frequency = 0;
		enabled = 0;
	}
};

#endif // __HGuideAPI_filter_config_t_h__
