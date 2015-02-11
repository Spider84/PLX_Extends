#ifndef _USART_FIFO_H_
#define _USART_FIFO_H_

#include "stm8l15x.h"

typedef struct {
     uint8_t * buf;
     uint8_t head;
     uint8_t count;
     uint8_t size;
} usart_fifo_t;

void usart_fifo_init(usart_fifo_t * f, uint8_t * buf, uint8_t size); 
void usart_fifo_clear(usart_fifo_t * f);
uint8_t usart_fifo_full(usart_fifo_t * f);
uint8_t usart_fifo_empty(usart_fifo_t * f);
uint8_t usart_fifo_read(usart_fifo_t * f, uint8_t * buf, uint8_t n);
bool usart_fifo_add(usart_fifo_t * f, uint8_t byte, bool block);
uint8_t usart_fifo_write(usart_fifo_t * f, const uint8_t * buf, uint8_t n, bool block);

#endif //_USART_FIFO_H_