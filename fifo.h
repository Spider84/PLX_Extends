#ifndef _FIFO_H_
#define _FIFO_H_

#include "stm8l15x.h"

typedef struct {
     uint16_t * buf;
     uint8_t head;
     uint8_t count;
     uint8_t size;
} fifo_t;

void fifo_init(fifo_t * f, uint16_t * buf, uint8_t size); 
uint8_t fifo_full(fifo_t * f);
uint8_t fifo_empty(fifo_t * f);
uint8_t fifo_read(fifo_t * f, uint16_t * buf, uint8_t n);
uint8_t fifo_write(fifo_t * f, const uint16_t * buf, uint8_t n, uint8_t block);

#endif 