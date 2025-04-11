#ifndef __HGuideAPI_port_id_t_h__
#define __HGuideAPI_port_id_t_h__
#pragma once

//List of port IDs
enum port_id_t
{
	port_Current  = 0, //  Configure the port which receives the 0x1005 message
	port_COM1     = 1,
	port_COM2     = 2,
	port_COM3     = 3,
	port_COM4     = 4,
	port_SPI      = 5,
	port_USB      = 7,
	port_ETHERNET = 9,
	port_CAN      = 12
};

#endif // __HGuideAPI_port_id_t_h__
