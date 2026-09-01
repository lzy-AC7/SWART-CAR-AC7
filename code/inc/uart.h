#ifndef _UART_H_
#define _UART_H_

#include "common.h"

void uart_rec_init();

void uart_map_init();

void uart_rec_rx_interrupt_handler (void);

void uart_map_rx_interrupt_handler (void);

bool wait_for_response(fifo_struct *fifo,uint8* buffer, uint32 *count,uint32_t timeout_ms);

void process_pos_data(void);

void process_map_data(void);

bool process_rec_data(bool mod);

#endif
