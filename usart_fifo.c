
#include "usart_fifo.h"
#ifdef USE_DEBUG
#include <stdio.h>
#endif

//This initializes the FIFO structure with the given buffer and size
void usart_fifo_init(usart_fifo_t * f, uint8_t * buf, uint8_t size){
     f->head = 0;
     f->count = 0; 
     f->size = size;
     f->buf = buf;
}

uint8_t usart_fifo_full(usart_fifo_t * f)
{
	return (f->count == f->size);
}

void usart_fifo_clear(usart_fifo_t * f)
{
	f->count = 0;
	f->head = 0;
}

uint8_t usart_fifo_empty(usart_fifo_t * f)
{
	return (f->count == 0);
}

//This reads nbytes bytes from the FIFO
//The number of bytes read is returned
uint8_t usart_fifo_read(usart_fifo_t * f, uint8_t * buf, uint8_t n){
     uint8_t * p = buf;
     for(uint8_t i=0; i < n; i++)
	 {
	   if (usart_fifo_empty(f))
			return i;
	   else {
#ifdef USE_DEBUG		 
			//printf("(0x%04x)",(uint16_t)&(f->buf[f->head]));
#endif		 
			*p = f->buf[f->head];
			*p++;
			f->head = (f->head + 1) % f->size;
			f->count--;
		}
	   
/*          if( f->tail != f->head ){ //see if any data is available
               
               *p = f->buf[f->tail++];  //grab a byte from the buffer
               *p++;
               if( f->tail == f->size ) f->tail = 0; //check for wrap-around
          } else {
               return i; //number of bytes read 
          }*/
     }
     return n;
}

bool usart_fifo_add(usart_fifo_t * f, uint8_t byte, bool block)
{
 	if (usart_fifo_full(f)) {
		if (block) return FALSE; 
		else
		f->head = (f->head + 1) % f->size; /* full, overwrite */
	}
	else {
		uint8_t tail = (f->head + f->count) % f->size;
		f->buf[tail] = byte;				
		f->count++;				
#ifdef USE_DEBUG			
		//printf("(0x%04x)",(uint16_t)&(f->buf[tail]));
#endif			
	}
	return TRUE;
}

//This writes up to nbytes bytes to the FIFO
//If the head runs in to the tail, not all bytes are written
//The number of bytes written is returned
uint8_t usart_fifo_write(usart_fifo_t * f, const uint8_t * buf, uint8_t n, bool block){
	const uint8_t * p = buf;
	for(uint8_t i=0; i < n; i++)
		if (!usart_fifo_add(f,*p++,block)) return i;
    return n;
}
 