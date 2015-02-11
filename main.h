#ifndef _MAIN_H_
#define _MAIN_H_

#include "stm8l15x.h"
#include "fifo.h"
#include "usart_fifo.h"

#define Factory_VREFINT 0x4910
#define ADC_CH_CNT	8

#define TFLAG_ENABLE_ADC 0x01

typedef struct {
  	uint16_t dma_buff[ADC_CH_CNT];
	uint16_t adc_summs[ADC_CH_CNT];
	fifo_t adc_fifo[ADC_CH_CNT];
	uint8_t adc_cnt;
} ADC_info;

//extern __IO uint32_t SignalDutyCycle;
extern __IO uint16_t IC3Value, IC2Value, IC1Value;

extern __IO uint8_t timers_flags;
extern __IO uint8_t t4_cnt;
extern usart_fifo_t rx_buff;

extern __IO ADC_info adc_info;
//extern __IO uint16_t adcs[ADC_CH_CNT];
//extern fifo_t adc_fifo[ADC_CH_CNT];
//extern __IO uint8_t adc_cnt;
//extern __IO uint16_t adc_summs[ADC_CH_CNT];

void sendSensorsData(void);

#endif //_MAIN_H_