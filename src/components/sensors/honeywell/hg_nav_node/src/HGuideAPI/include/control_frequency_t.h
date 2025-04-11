#ifndef __HGuideAPI_control_frequency_t_h__
#define __HGuideAPI_control_frequency_t_h__
#pragma once

//List of available control frequencies
enum control_frequency_t
{
	MSG_CTRL_0_RATE    = 0, //  Message is not being outputted
	MSG_CTRL_600_RATE  = 1,
	MSG_CTRL_1200_RATE = 2,
	MSG_CTRL_1800_RATE = 3,
	MSG_CTRL_3600_RATE = 4,
	MSG_CTRL_1000_RATE = 5,
	MSG_CTRL_900_RATE  = 6,
	MSG_CTRL_1500_RATE = 7
};

#endif // __HGuideAPI_control_frequency_t_h__
