/**
  ******************************************************************************
  * @file    Project/STM8L15x_StdPeriph_Template/main.c
  * @author  MCD Application Team
  * @version V1.5.0
  * @date    13-May-2011
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */  
	
/* Includes ------------------------------------------------------------------*/
#include "stm8l15x.h"

#ifdef USE_DEBUG
#include <stdio.h>
#endif
#include "main.h"
#include "fifo.h"
#include "usart_fifo.h"
#include "fluidtemp.h"
#include "swuart.h"


/* Private typedef -----------------------------------------------------------*/
typedef struct {  
//FluidTemp - raw = Degrees Celsius Water 
  uint8_t FluidTemp;
  uint16_t OilTemp;
//Boost     - raw/329.47 = kg/sm^2
  uint16_t Boost;
//MAF       - raw (grams per second)
  uint16_t  MAF;
//ThrtlPos  - raw %
  uint8_t  ThrtlPos;
//Speed     - raw/3.97 km/h
//FuelDty   - raw/10.23 Duty
//----
//EngRPM    - raw*19.55 RPM
  uint16_t EngRPM;
//GazTemp   - raw = Degrees Celsius Water
  uint8_t  GazTemp;
//AKBVolt   - raw/51.15 Volts
  uint16_t AKBVolt;
//GasLvl    - raw %
  uint8_t  GasLvl;

  uint16_t SupVolt;
} _sensorsData;
/* Private define ------------------------------------------------------------*/
#define OPT_BL_ADDR_L 0x480B
#define OPT_BL_ADDR_H 0x480C
/* Private macro -------------------------------------------------------------*/
#define countof(a)            (sizeof(a) / sizeof(*(a)))
/* Private variables ---------------------------------------------------------*/
__IO uint8_t timers_flags = TFLAG_ENABLE_ADC;
__IO uint8_t t4_cnt;
__IO ADC_info adc_info;

static uint16_t fifo_buffs[ADC_CH_CNT][10];
static _sensorsData sensorsData;

//USART
usart_fifo_t tx_buff;
usart_fifo_t rx_buff;
static uint8_t tx_fifo_buff[2+5*13];
static uint8_t rx_fifo_buff[32];
/* Private function prototypes -----------------------------------------------*/
#ifdef USE_DEBUG
#ifdef _RAISONANCE_
#define PUTCHAR_PROTOTYPE int putchar (char c)
#define GETCHAR_PROTOTYPE int getchar (void)
#elif defined (_COSMIC_)
#define PUTCHAR_PROTOTYPE char putchar (char c)
#define GETCHAR_PROTOTYPE char getchar (void)
#else /* _IAR_ */
#define PUTCHAR_PROTOTYPE int putchar (int c)
#define GETCHAR_PROTOTYPE int getchar (void)
#endif /* _RAISONANCE_ */
#endif
/* Private functions ---------------------------------------------------------*/
void SelectHSE(void);
void USART1_Rx_Init(void);
void TIM2_Init(void);
void TIM3_Init(void);
void TIM4_Init(void);
void ADC1_DMA0_Init(void);
uint16_t encodeValue(uint16_t);
void addSensor(uint8_t code,uint8_t num, uint16_t value);
void BuildAGATpkt(uint8_t ch);
void ParseAGATpkt(void);
void DecodeAGATpkt(void);

void SelectHSE(void)
{
  CLK->SWCR |= CLK_SWCR_SWEN;
  CLK->SWR = CLK_SYSCLKSource_HSE;
  
  while (CLK->SWCR & CLK_FLAG_SWBSY) {}  
  CLK->CKDIVR = CLK_SYSCLKDiv_1;  
  
  if ((CLK->ECKCR & CLK_ECKCR_HSEON) && (CLK->SCSR == CLK_SYSCLKSource_HSE)) {
    CLK->ICKCR &= (uint8_t)(~CLK_ICKCR_HSION);
  }
  
  CLK->PCKENR1 |= CLK_PCKENR1_TIM4 | CLK_PCKENR1_USART1 | CLK_PCKENR1_TIM3 | CLK_PCKENR1_TIM2;
  CLK->PCKENR2 |= CLK_PCKENR2_TIM1 | CLK_PCKENR2_ADC1 | CLK_PCKENR2_DMA1;
}

void TIM4_Init(void)
{
  t4_cnt = 0;
  TIM4->CR1 = 0;
  //TIM4->ARR = 195; //= 100ms
  TIM4->ARR = 156;   //= 5ms
  //TIM4->PSCR = TIM4_Prescaler_8192;
  TIM4->PSCR = TIM4_Prescaler_512;  
  TIM4->CNTR = 0;
  TIM4->SR1 = 0;
  TIM4->DER = 0;
  TIM4->IER = 0x01;
  TIM4->CR2 = 0;
  TIM4->SMCR = 0;
  TIM4->CR1 |= TIM4_CR1_CEN;
}

void TIM2_Init(void)
{
  GPIO_Init(GPIOB, GPIO_Pin_0, GPIO_Mode_In_FL_No_IT);
  
  TIM2_DeInit();
   
  /* Time base configuration */
  TIM2_TimeBaseInit(TIM2_Prescaler_128, TIM2_CounterMode_Up, 0xFFFF);
  TIM2_PWMIConfig(TIM2_Channel_1, TIM2_ICPolarity_Falling, TIM2_ICSelection_DirectTI, TIM2_ICPSC_DIV1, 2);
   
  /* Select the TIM1 Input Trigger: TI1FP1 */
  TIM2_SelectInputTrigger(TIM2_TRGSelection_TI1FP1);
  TIM2_SelectSlaveMode(TIM2_SlaveMode_Reset);
  
  /* Enable CC1 interrupt request */
  TIM2_ITConfig(TIM2_IT_CC1, ENABLE);
  
  /* Enable TIM2 */
  TIM2_Cmd(ENABLE);
}

void TIM3_Init(void)
{        
  GPIO_Init(GPIOB, GPIO_Pin_1, GPIO_Mode_In_PU_No_IT);
  
  TIM3_DeInit();
   
  /* Time base configuration */
  TIM3_TimeBaseInit(TIM3_Prescaler_128, TIM3_CounterMode_Up, 0xFFFF);
  TIM3_ICInit(TIM3_Channel_1, TIM3_ICPolarity_Falling, TIM3_ICSelection_DirectTI, TIM3_ICPSC_DIV8, 0);
   
  /* Select the TIM3 Input Trigger: TI1FP1 */
  TIM3_SelectInputTrigger(TIM3_TRGSelection_TI1FP1);
  TIM3_SelectSlaveMode(TIM3_SlaveMode_Reset);

  /* Enable CC1 interrupt request */
  TIM3_ITConfig(TIM3_IT_CC1, ENABLE);
  
  //TIM3_ClearFlag(TIM3_FLAG_Update);    
  //TIM3_ITConfig(TIM3_IT_Update, ENABLE);
  
  /* Enable TIM3 */
  TIM3_Cmd(ENABLE);
}

void ADC1_DMA0_Init(void)
{
  for (uint8_t i=0; i<ADC_CH_CNT;i++) {
    for (uint8_t j=0;j<10;j++) fifo_buffs[i][j] = 0;
    fifo_init((fifo_t *)&(adc_info.adc_fifo[i]),(uint16_t *)(&fifo_buffs[i]),10);
    adc_info.adc_summs[i] = 0;
#ifdef USE_DEBUG    
    //printf("%u: 0x%04x\r\n",i,(uint16_t)(&fifo_buffs[i][0]));
#endif
  }
  
  ADC1->CR1 = ADC_Resolution_12Bit | /*ADC_CR1_CONT | */ADC_CR1_ADON;
  ADC1->TRIGR[0] = ADC_TRIGR1_TSON | ADC_TRIGR1_VREFINTON;
  ADC1->SQR[3] = (uint8_t)ADC_Channel_7;
  ADC1->SQR[2] = (uint8_t)ADC_Channel_15 | (uint8_t)ADC_Channel_10 | (uint8_t)ADC_Channel_9;
  ADC1->SQR[1] = (uint8_t)ADC_Channel_22 /*| (uint8_t)ADC_Channel_18 | (uint8_t)ADC_Channel_17*/ | (uint8_t)ADC_Channel_16;
  ADC1->SQR[0] = (uint8_t)ADC_Channel_TempSensor | (uint8_t)ADC_Channel_Vrefint;
  ADC1->SR = 0;
  ADC1->CR3 = (ADC_SamplingTime_384Cycles << 5);
  ADC1->CR2 = ADC_CR2_PRESC | ADC_SamplingTime_384Cycles;  
  
  SYSCFG->RMPCR1 &= ~(SYSCFG_RMPCR1_ADC1DMA_REMAP);
  
  DMA1_Channel0->CNBTR = ADC_CH_CNT;
  DMA1_Channel0->CM0ARH = (uint8_t)((uint16_t)(adc_info.dma_buff)>>8);
  DMA1_Channel0->CM0ARL = (uint8_t)((uint16_t)adc_info.dma_buff & 0xFF);
      
  DMA1_Channel0->CPARH = (ADC1_BASE+4)>>8;
  DMA1_Channel0->CPARL = (uint8_t)(ADC1_BASE+4);  
  
  DMA1_Channel0->CSPR |= DMA_CSPR_16BM;
  DMA1_Channel0->CCR |= DMA_CCR_ARM | DMA_CCR_CE | DMA_CCR_IDM | DMA_CCR_TCIE;
  
  DMA1->GCSR |= DMA_GCSR_GE;
}

void USART1_Rx_Init(void)
{    
  usart_fifo_init((usart_fifo_t *)&rx_buff,(uint8_t *)rx_fifo_buff,sizeof(rx_fifo_buff));
  
  USART_DeInit(USART1);
  USART_Init(USART1,115200,USART_WordLength_8b,USART_StopBits_1,USART_Parity_No,USART_Mode_Rx);
   
  GPIO_Init(GPIOC, GPIO_Pin_6, GPIO_Mode_In_FL_No_IT); 
  
  SYSCFG->RMPCR1 |= 0x20;
  
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
  
  USART_Cmd(USART1, ENABLE);  
}

uint16_t encodeValue(uint16_t value)
{
  return ((value & 0xFC0)<<2) | (value & 0x3F);
}

void addSensor(uint8_t code, uint8_t num, uint16_t value)
{  	
  	uint16_t tmp16 = encodeValue((uint16_t)code);
	usart_fifo_write((usart_fifo_t *)&tx_buff,(uint8_t *)&tmp16,2,FALSE);
	usart_fifo_add((usart_fifo_t *)&tx_buff,num,FALSE);
	tmp16 = encodeValue(value);
	usart_fifo_write((usart_fifo_t *)&tx_buff,(uint8_t *)&tmp16,2,FALSE);		
}

void sendSensorsData(void)
{	
	sim();
	uint16_t _IC3Value=IC3Value;
	uint16_t _IC2Value=IC2Value;
	uint16_t _IC1Value=IC1Value;
	rim();

	usart_fifo_clear((usart_fifo_t *)&tx_buff);
	timers_flags &= ~TFLAG_ENABLE_ADC;
	//TIM1_Cmd(DISABLE);		
	usart_fifo_add((usart_fifo_t *)&tx_buff,0x80,FALSE);
	
	addSensor(2,0,sensorsData.FluidTemp); //Fluid Temp                                      //1
	addSensor(2,1,sensorsData.GazTemp);   //GazTemp   - raw = Degrees Celsius Water         //2
/*	70   92 Om
	90   48.5
	110  27.3
	130  16.3
	150  10.2 */	
	addSensor(2,2,sensorsData.OilTemp);   //Fluid Temp 2 Oil                                //3
	addSensor(4,0,sensorsData.Boost);     //Boost     - raw/329.47 = kg/sm^2                //4
	addSensor(6,0,sensorsData.EngRPM);    //EngRPM    - raw*19.55 RPM                       //5
	addSensor(7,0,getSpeed(_IC3Value));   //Speed     - raw/3.97 km/h                       //6
    /* 	Open = 4.5 */	
// FIXME	
	addSensor(8,0,sensorsData.ThrtlPos);  //ThrtlPos  - raw %                               //7
	addSensor(13,0,sensorsData.MAF);      //MAF       - raw (grams per second)              //8
	addSensor(17,0,sensorsData.GasLvl);   //GasLvl    - raw %
// FIXME                                                                                    //9
    addSensor(18,0,sensorsData.AKBVolt);  //AKBVolt   - raw/51.15 Volts	                    //10
    addSensor(18,1,sensorsData.SupVolt);  //SupVolt   - raw/51.15 Volts	                    //11
//	addSensor(19,0,0); //raw/204.6;       //Knock volts 0-5	                                //12
	addSensor(20,0,(uint16_t)(((uint32_t)((uint32_t)_IC2Value * 10230UL) / _IC1Value)/10U)); //FuelDty   - raw/10.23 Duty  //13
	usart_fifo_add((usart_fifo_t *)&tx_buff,0x40,FALSE);
}


//AGAT
static uint8_t cmd = 0;
static uint8_t pkt_data[7];
static uint8_t pkt_len = 0;

/*
D1 = (R1<<2) | ((R2>>5) & 0x03);  0=(i+2)=(5-i)
D2 = (R2<<3) | ((R3>>4) & 0x07);  2=3=4
D3 = (R3<<4) | ((R4>>3) & 0x0F);
D4 = (R4<<5) | ((R5>>2) & 0x1F);
D5 = (R5<<6) | ((R6>>1) & 0x3F);
D6 = (R6<<7) | (R7 & 0x7F);
*/
void DecodeAGATpkt(void)
{
	uint8_t next;
			
	for (uint8_t i=0;i<pkt_len;i++) {
		if (i<(pkt_len-1)) next=pkt_data[i+1]; else next=0;
		pkt_data[i] = (uint8_t)((pkt_data[i] << ((i % 6)+2)) | ((next >> (5-(i % 6))) & ((2<<((i % 6)+1)))-1));
	}
	pkt_len--;
}

void ParseAGATpkt(void)
{
	switch (cmd) {
	  //PutFrame(0x84, DispOut, (Sensors.Freq >> 8) & 0xFF, (Sensors.Freq & 0xFF), Sensors.Tgas, Sensors.Treducer, Sensors.Ubat); //
	  //0x84, 0x40 0x03 0x58 0x40 0x40 0x89 
	  //0x84 - Header
	  //0x40 - DispOut           0 
	  //0x03 - Sensors.Freq HI   1 0x0358
	  //0x58 - Sensors.Freq LO   2
	  //0x40 - Sensors.Tgas      3
	  //0x40 - Sensors.Treducer  4
	  //0x89 - Sensors.Ubat      5 137 -> 13.7V
	  
		case 0x84:
		  	if (pkt_len<7) {
			  cmd = 0;
			  break;
			}
	        DecodeAGATpkt();
			uint8_t g_lvl;
			for (uint8_t i=0;i<3;i++) 
			  if (pkt_data[0] & (1<<i)) g_lvl+=25;
			if ((pkt_data[0] & (1<<4)) && (!g_lvl)) g_lvl=10;
			sensorsData.GasLvl = g_lvl;	
			sensorsData.EngRPM = (uint16_t)(((*(uint16_t *)&pkt_data[1])*100UL)/1955UL);
			sensorsData.GazTemp = pkt_data[3];			
			sensorsData.AKBVolt = (uint16_t)(((uint32_t)(pkt_data[5]+4)*5115UL)/1000UL);
#ifdef USE_DEBUG			
//			printf("RPM:%u %u\r\n",(*(uint16_t *)&pkt_data[1]),sensorsData.EngRPM);
//			printf("VOLT: %u\r\n",pkt_data[5]);
//			printf("VOLT: %u\r\n",sensorsData.AKBVolt);						
#endif
			break;
		default:
	  		break;
	}
	pkt_len = 0;
}

void BuildAGATpkt(uint8_t ch)
{
  if (ch & 0x80) {
	if ((cmd & 0x80) && (pkt_len>1)) ParseAGATpkt();
	cmd = ch;
  } else 
  if (pkt_len<=sizeof(pkt_data)) {
	pkt_data[pkt_len++] = ch;
    if ((cmd & 0x80) && (pkt_len>6)) ParseAGATpkt();
  } else
  {
  	pkt_len=0;
	cmd=0;
  }
}

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
void main(void)
{
  SelectHSE();  
  
  GPIO_DeInit(GPIOC);
  usart_fifo_init((usart_fifo_t *)&tx_buff,(uint8_t *)tx_fifo_buff,sizeof(tx_fifo_buff));
  uart_init();  
  
#ifndef USE_DEBUG  
  USART1_Rx_Init(); 
#else
  GPIO_Init(GPIOC, GPIO_Pin_6, GPIO_Mode_Out_PP_High_Slow);
  GPIO_ResetBits(GPIOC, GPIO_Pin_6);

  GPIO_Init(GPIOC, GPIO_Pin_4, GPIO_Mode_Out_PP_High_Fast);
  GPIO_ResetBits(GPIOC,GPIO_Pin_4);

/*  printf("\r\nBuild Date is %s (%s)\r\n", __DATE__, __TIME__);
  uint8_t div = CLK->CKDIVR;
  printf("CPU Freq: %lu Hz, 0x%x 0x%x\r\n", CLK_GetClockFreq(), CLK->SCSR, div);
  
  uint16_t ref = 0x600 + FLASH_ReadByte(Factory_VREFINT);
    
  printf("Vref %u\r\n", ref);*/  
#endif
  
  FLASH_DeInit();
  FLASH_Unlock(FLASH_MemType_Data);
  
  uint16_t optbl = ((uint16_t)FLASH_ReadByte(OPT_BL_ADDR_L)<<8) | FLASH_ReadByte(OPT_BL_ADDR_H);

  if (optbl!=0x55AA) {
#ifdef USE_DEBUG	
    //printf("OPT_BL: 0x%x", optbl);
#endif  
    FLASH_ProgramOptionByte(OPT_BL_ADDR_L,0x55);
    FLASH_ProgramOptionByte(OPT_BL_ADDR_H,0xAA);
  }  
 
  TIM4_Init();
  TIM3_Init();
  TIM2_Init();
  ADC1_DMA0_Init();
  
  enableInterrupts();
  
  IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
  IWDG_SetPrescaler(IWDG_Prescaler_256);
  IWDG_SetReload(0xFF);  
  IWDG_ReloadCounter();
  //IWDG_Enable();
  IWDG_WriteAccessCmd(IWDG_WriteAccess_Disable);
  
  /* Infinite loop */
  while (1)
  {  
#ifndef USE_DEBUG	
	if (test_status(transmit_data_reg_empty) && (!usart_fifo_empty((usart_fifo_t *)&tx_buff))) {
	  uint8_t tx_byte;
	  usart_fifo_read((usart_fifo_t *)&tx_buff,&tx_byte,1);
	  //GPIO_ToggleBits(GPIOC,GPIO_Pin_6);
	  uart_send(tx_byte);
	  if (usart_fifo_empty((usart_fifo_t *)&tx_buff)) timers_flags |= TFLAG_ENABLE_ADC;
	}
#endif
	if ((adc_info.adc_cnt) && (timers_flags & TFLAG_ENABLE_ADC)) {	  
	  static uint16_t _adc_summs[ADC_CH_CNT];
	  static uint16_t _adc_count[ADC_CH_CNT];
	  
	  sim();	  
	  adc_info.adc_cnt = 0;
	  for (uint8_t i=0; i<ADC_CH_CNT; i++) {
		_adc_count[i] = adc_info.adc_fifo[i].count;
	  	_adc_summs[i] = adc_info.adc_summs[i];
	  }
	  rim();
	  sensorsData.ThrtlPos = calcThrtlPos(_adc_summs[0]/_adc_count[0]);
	  sensorsData.Boost = getBoost(_adc_summs[1]/_adc_count[1]);
	  sensorsData.OilTemp = getOilTemp(_adc_summs[2]/_adc_count[2],sensorsData.AKBVolt);
      sensorsData.SupVolt = getVoltValue(_adc_summs[3]/_adc_count[3]);
	  sensorsData.MAF = getMAFValue(_adc_summs[4]/_adc_count[4]);
	  sensorsData.FluidTemp=getTemp(_adc_summs[5]/_adc_count[5]);
	}
	if (rx_buff.count) {
		uint8_t ch;
		if (usart_fifo_read((usart_fifo_t *)&rx_buff,&ch,1)) BuildAGATpkt(ch);
	}
#ifdef USE_DEBUG
	if (t4_cnt>100) {
  		t4_cnt=0;
		timers_flags &= ~TFLAG_ENABLE_ADC;
		printf("\r\nDuty: %u %u %u %u\r\n", IC1Value, IC2Value, (uint16_t)((uint32_t)(IC2Value*1000UL)/IC1Value), (uint16_t)(((uint32_t)((uint32_t)IC2Value * 10230UL) / IC1Value)/10));
		printf("Speed: %u %u\r\n", IC3Value, getSpeed(IC3Value));
		printf("WTemp1: %u %u\r\n", sensorsData.FluidTemp,adc_summs[4]/adc_fifo[4].count);
		uint16_t adc = adc_summs[2]/adc_fifo[2].count;
		printf("OilTemp: %u %u %u\r\n", sensorsData.OilTemp, adc, (uint16_t)((uint32_t)(1256UL*(uint32_t)adc)/4096U));
		printf("MAF: %u %u\r\n", sensorsData.MAF, adc_summs[3]/adc_fifo[3].count);
		
/*		uart_send(0x55);
		while (test_status(transmit_in_progress));		
		uart_send(ADC1->CR1);
		while (test_status(transmit_in_progress));*/
		timers_flags |= TFLAG_ENABLE_ADC;
	}
#else
	if ((t4_cnt>20) && (usart_fifo_empty((usart_fifo_t *)&tx_buff))) //5ms * 20 = 100ms
	{
  		t4_cnt=0;
		IWDG_ReloadCounter();
  		sendSensorsData();
	}
#endif
#ifdef USE_DEBUG
/*    if (t4_cnt>9) {    
      t4_cnt = 0;      
      //TIM4->CR1 &= ~TIM4_CR1_CEN;

//      for (uint8_t j=0; j<ADC_CH_CNT;j++) {
//		uint16_t value = adc_summs[j]/adc_fifo[j].count;
//		uint16_t volt = (uint16_t)(((uint32_t)value*3300UL)/4096UL);
//        printf("ADC[%u]: %u (%u.%.2u)\r\n",j, value,volt/1000,volt%1000);
//      }
	  
	  printf("Temp: %d\r\n",sensorsData.FluidTemp);
	  printf("Boost: %u %u\r\n",sensorsData.Boost,adc_summs[1]/adc_fifo[1].count);
	  printf("Oil Temp: %u\r\n",sensorsData.OilTemp);
	  printf("ThrtlPos: %u %u\r\n",sensorsData.ThrtlPos,adc_summs[3]/adc_fifo[3].count);
	  printf("RPM: %u\r\n",(uint16_t)(((uint32_t)sensorsData.EngRPM*1955UL)/100));
	  printf("G_Temp: %u\r\n",sensorsData.GazTemp);
	  printf("Volt: %u\r\n",(uint16_t)(((uint32_t)sensorsData.AKBVolt*1000)/5115));
      printf("\r\n");
    }*/
#endif
  }
}

#ifdef USE_DEBUG
PUTCHAR_PROTOTYPE
{
  /* Write a character to the USART */
  //USART_SendData8(USART1, c);
  /* Loop until the end of transmission */
  //while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
  uart_send(c);
  while (test_status(transmit_in_progress));

  return (c);
}
#endif
#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
#ifdef USE_DEBUG 
  printf("Wrong parameters value: file %s on line %d\r\n", file, line);
#endif  
  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
