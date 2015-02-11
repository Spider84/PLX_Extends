
#include "fifo.h"
#include <stdio.h>

//This initializes the FIFO structure with the given buffer and size
void fifo_init(fifo_t * f, uint16_t * buf, uint8_t size){
     f->head = 0;
     f->count = size; 
     f->size = size;
     f->buf = buf;
}

uint8_t fifo_full(fifo_t * f)
{
	return (f->count == f->size);
}

uint8_t fifo_empty(fifo_t * f)
{
	return (f->count == 0);
}

//This reads nbytes bytes from the FIFO
//The number of bytes read is returned
uint8_t fifo_read(fifo_t * f, uint16_t * buf, uint8_t n){
     uint16_t * p = buf;
     for(uint8_t i=0; i < n; i++)
	 {
	   if (fifo_empty(f))
			return i;
	   else {
			//printf("(0x%04x)",(uint16_t)&(f->buf[f->head]));
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
 
//This writes up to nbytes bytes to the FIFO
//If the head runs in to the tail, not all bytes are written
//The number of bytes written is returned
uint8_t fifo_write(fifo_t * f, const uint16_t * buf, uint8_t n, uint8_t block){
	const uint16_t * p = buf;
	for(uint8_t i=0; i < n; i++)
	{
		if (fifo_full(f)) {
		  if (block)
			return i; 
		  else
			f->head = (f->head + 1) % f->size; /* full, overwrite */
		}
		else {
			uint8_t tail = (f->head + f->count) % f->size;
			f->buf[tail] = *p++;				
			f->count++;				
			//printf("(0x%04x)",(uint16_t)&(f->buf[tail]));
		}
     }
     return n;
}
 