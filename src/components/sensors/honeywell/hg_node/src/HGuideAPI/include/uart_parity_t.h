#ifndef __HGuideAPI_uart_parity_t_h__
#define __HGuideAPI_uart_parity_t_h__
#pragma once

// Uart Parity types
enum uart_parity_t
{
	uart_parity_Current = 0, //  Use the current Parity on selected port
	uart_parity_Odd     = 1,
	uart_parity_Even    = 2,
	uart_parity_Mark    = 3,
	uart_parity_Space   = 4,
	uart_parity_None    = 5
};

#endif // __HGuideAPI_uart_parity_t_h__
