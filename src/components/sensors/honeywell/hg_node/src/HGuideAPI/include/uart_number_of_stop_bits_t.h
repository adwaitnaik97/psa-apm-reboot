#ifndef __HGuideAPI_uart_number_of_stop_bits_t_h__
#define __HGuideAPI_uart_number_of_stop_bits_t_h__
#pragma once

// Uart number of stop bits types
enum uart_number_of_stop_bits_t
{
	uart_stop_bits_Current    = 0, //  use the current number of stop bits on selected port
	uart_stop_bits_NotUsed    = 1,
	uart_stop_bits_One        = 2,
	uart_stop_bits_Two        = 3,
	uart_stop_bits_OneAndHalf = 4
};

#endif // __HGuideAPI_uart_number_of_stop_bits_t_h__
