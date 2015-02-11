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
#include <stdio.h>
#include "fifo.h"
#include "usart_fifo.h"
#include "fluidtemp.h"
#include "swuart.h"

volatile uint8_t t4_cnt;
volatile uint8_t adc_cnt;

void SelectHSE(void);
void USART1_Rx_Init(void);
void TIM1_Init(void);
void TIM4_Init(void);
void ADC1_DMA0_Init(void);
uint16_t encodeValue(uint16_t);
void sendSensorsData(void);
void BuildAGATpkt(uint8_t ch);
void ParseAGATpkt(void);
void DecodeAGATpkt(void);

/** @addtogroup STM8L15x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define OPT_BL_ADDR_L 0x480B
#define OPT_BL_ADDR_H 0x480C

#define Factory_VREFINT 0x4910
#define ADC_CH_CNT	6
/* Private macro -------------------------------------------------------------*/
#define countof(a)            (sizeof(a) / sizeof(*(a)))
/* Private variables ---------------------------------------------------------*/
volatile uint16_t adcs[ADC_CH_CNT];
volatile uint16_t adc_summs[ADC_CH_CNT];
fifo_t adc_fifo[ADC_CH_CNT];
static uint16_t fifo_buffs[ADC_CH_CNT][10];

//USART
usart_fifo_t tx_buff;
usart_fifo_t rx_buff;
static uint8_t tx_fifo_buff[2+5*11];
static uint8_t rx_fifo_buff[32];
/* Private function prototypes -----------------------------------------------*/
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
/* Private functions ---------------------------------------------------------*/

typedef struct {  
//FluidTemp - raw = Degrees Celsius Water 
  uint8_t FluidTemp;
  uint8_t OilTemp;
//Boost     - raw/329.47 = kg/sm^2
  uint16_t Boost;
//MAF       - raw (grams per second)
//  uint8_t  MAF;
//ThrtlPos  - raw %
  uint8_t  ThrtlPos;
//Speed     - raw/3.97 km/h
  uint16_t Speed;
//FuelDty   - raw/10.23 Duty
  uint16_t FuelDty;
//----
//EngRPM    - raw*19.55 RPM
  uint16_t EngRPM;
//GazTemp   - raw = Degrees Celsius Water
  uint8_t  GazTemp;
//AKBVolt   - raw/51.15 Volts
  uint16_t AKBVolt;
//GasLvl    - raw %
  uint8_t  GasLvl;
} _sensorsData;

static _sensorsData sensorsData;

void SelectHSE(void)
{
  CLK->SWCR |= CLK_SWCR_SWEN;
  CLK->SWR = CLK_SYSCLKSource_HSE;
  
  while (CLK->SWCR & CLK_FLAG_SWBSY) {}  
  CLK->CKDIVR = CLK_SYSCLKDiv_1;  
  
  if ((CLK->ECKCR & CLK_ECKCR_HSEON) && (CLK->SCSR == CLK_SYSCLKSource_HSE)) {
    CLK->ICKCR &= (uint8_t)(~CLK_ICKCR_HSION);
  }
  
  CLK->PCKENR1 |= (1<<(CLK_Peripheral_TIM4 & (uint8_t)0x0F)) | (1<<(CLK_Peripheral_USART1 & (uint8_t)0x0F));
  CLK->PCKENR2 |= (1<<(CLK_Peripheral_TIM1 & (uint8_t)0x0F)) | (1<<(CLK_Peripheral_ADC1 & (uint8_t)0x0F)) | (1<<(CLK_Peripheral_DMA1 & (uint8_t)0x0F));
}

void TIM4_Init(void)
{
  t4_cnt = 0;
  
  TIM4->CR1 = 0;
  //TIM4->ARR = 195;
  TIM4->ARR = 150;
  TIM4->PSCR = TIM4_Prescaler_8192;
  TIM4->CNTR = 0;
  TIM4->SR1 = 0;
  TIM4->DER = 0;
  TIM4->IER = 0x01;
  TIM4->CR2 = 0;
  TIM4->SMCR = 0;
  TIM4->CR1 |= TIM4_CR1_CEN;
}

void TIM1_Init(void)
{
  TIM1_DeInit();
   
  /* Time base configuration */
  TIM1_TimeBaseInit(16000U, TIM1_CounterMode_Up, 50, 0);
  TIM1_ARRPreloadConfig(ENABLE);
  
  /* Clear TIM4 update flag */
  TIM1_ClearFlag(TIM1_FLAG_Update);    
  /* Enable update interrupt */
  TIM1_ITConfig(TIM1_IT_Update, ENABLE);

  /* Enable TIM4 */
  TIM1_Cmd(ENABLE);
}

void ADC1_DMA0_Init(void)
{
  for (uint8_t i=0; i<ADC_CH_CNT;i++) {
    for (uint8_t j=0;j<10;j++) fifo_buffs[i][j] = 0;
    fifo_init((fifo_t *)&adc_fifo[i],(uint16_t *)(&fifo_buffs[i]),10);
    adc_summs[i] = 0;
    
    //printf("%u: 0x%04x\r\n",i,(uint16_t)(&fifo_buffs[i][0]));
  }
  
  ADC1->CR1 = ADC_Resolution_12Bit | /*ADC_CR1_CONT | */ADC_CR1_ADON;
  ADC1->TRIGR[0] = ADC_TRIGR1_TSON | ADC_TRIGR1_VREFINTON;
  ADC1->SQR[3] = (uint8_t)ADC_Channel_7;
  ADC1->SQR[2] = (uint8_t)ADC_Channel_10 | (uint8_t)ADC_Channel_9;
  ADC1->SQR[1] = (uint8_t)ADC_Channel_22 /*| (uint8_t)ADC_Channel_18 | (uint8_t)ADC_Channel_17 | (uint8_t)ADC_Channel_16*/;
  ADC1->SQR[0] = (uint8_t)ADC_Channel_TempSensor | (uint8_t)ADC_Channel_Vrefint;
  ADC1->SR = 0;
  ADC1->CR3 = (ADC_SamplingTime_384Cycles << 5);
  ADC1->CR2 = ADC_CR2_PRESC | ADC_SamplingTime_384Cycles;  
  
  SYSCFG->RMPCR1 &= ~(SYSCFG_RMPCR1_ADC1DMA_REMAP);
  
  DMA1_Channel0->CNBTR = ADC_CH_CNT;
  DMA1_Channel0->CM0ARH = (uint8_t)((uint16_t)adcs>>8);
  DMA1_Channel0->CM0ARL = (uint8_t)((uint16_t)((uint16_t *)adcs) & 0xFF);
      
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
  USART_Init(USART1,19200,USART_WordLength_8b,USART_StopBits_1,USART_Parity_No,(USART_Mode_TypeDef)(USART_Mode_Tx | USART_Mode_Rx));
  USART_Cmd(USART1, DISABLE);
  
  GPIO_DeInit(GPIOC);
  GPIO_Init(GPIOC, GPIO_Pin_5, GPIO_Mode_Out_PP_Low_Fast);
  GPIO_Init(GPIOC, GPIO_Pin_6, GPIO_Mode_In_FL_No_IT); 
  
  SYSCFG->RMPCR1 |= 0x20;
  //GPIO_Init(GPIOC, GPIO_Pin_6, GPIO_Mode_Out_PP_High_Fast);
  //GPIO_ResetBits(GPIOC,GPIO_Pin_6);  
  
  /* Deinitialize DMA channels */
  DMA_GlobalDeInit();

  DMA_DeInit(DMA1_Channel1);
  DMA_DeInit(DMA1_Channel2);

  /* DMA channel Rx of USART Configuration */
  //DMA_Init(DMA1_Channel2, (uint16_t)rx_fifo_buff, (uint16_t)0x5231, countof(rx_fifo_buff), DMA_DIR_PeripheralToMemory, DMA_Mode_Circular, DMA_MemoryIncMode_Inc, DMA_Priority_Low, DMA_MemoryDataSize_Byte);  

  /* Enable the USART Tx DMA requests */
  USART_DMACmd(USART1, USART_DMAReq_TX, ENABLE);

  /* Global DMA Enable */
  DMA_GlobalCmd(ENABLE);

  /* Enable the USART Tx DMA channel */
  DMA_Cmd(DMA1_Channel1, DISABLE);
 
  /* Enable the USART Receive interrupt: this interrupt is generated when the USART
    receive data register is not empty */
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
  
  USART_Cmd(USART1, ENABLE);  
}

uint16_t encodeValue(uint16_t value)
{
  return ((value & 0xFC0)<<2) | (value & 0x3F);
}

void sendSensorsData(void)
{	
  	uint8_t tmp;
	uint16_t tmp16;

	usart_fifo_clear((usart_fifo_t *)&tx_buff);
	
	tmp = 0x80;
    usart_fifo_write((usart_fifo_t *)&tx_buff,&tmp,1,0);	
	
	tmp16 = encodeValue(2); //Fluid Temp
	usart_fifo_write((usart_fifo_t *)&tx_buff,(uint8_t *)&tmp16,2,0);
	tmp = 0x00;
    usart_fifo_write((usart_fifo_t *)&tx_buff,&tmp,1,0);
	tmp16 = encodeValue(sensorsData.FluidTemp);
	usart_fifo_write((usart_fifo_t *)&tx_buff,(uint8_t *)&tmp16,2,0);		
	
/*	70   92 Om
	90   48.5
	110  27.3
	130  16.3
	150  10.2 */	
/*	tmp16 = encodeValue(2); //Fluid Temp 2 Oil
	usart_fifo_write((usart_fifo_t *)&tx_buff,(uint8_t *)&tmp16,2,0);
	tmp = 0x01;
    usart_fifo_write((usart_fifo_t *)&tx_buff,&tmp,1,0);
	tmp16 = encodeValue(sensorsData.OilTemp);
	usart_fifo_write((usart_fifo_t *)&tx_buff,(uint8_t *)&tmp16,2,0);		
*/
	tmp16 = encodeValue(2); //GazTemp   - raw = Degrees Celsius Water
	usart_fifo_write((usart_fifo_t *)&tx_buff,(uint8_t *)&tmp16,2,0);
	tmp = 0x02;
    usart_fifo_write((usart_fifo_t *)&tx_buff,&tmp,1,0);
	tmp16 = encodeValue(sensorsData.GazTemp);
	usart_fifo_write((usart_fifo_t *)&tx_buff,(uint8_t *)&tmp16,2,0);

	tmp16 = encodeValue(4); //Boost     - raw/329.47 = kg/sm^2
	usart_fifo_write((usart_fifo_t *)&tx_buff,(uint8_t *)&tmp16,2,0);
	tmp = 0x00;
    usart_fifo_write((usart_fifo_t *)&tx_buff,&tmp,1,0);
	tmp16 = encodeValue(sensorsData.Boost);
	usart_fifo_write((usart_fifo_t *)&tx_buff,(uint8_t *)&tmp16,2,0);	
/* FIXME
	tmp16 = encodeValue(6); //EngRPM    - raw*19.55 RPM
	usart_fifo_write((usart_fifo_t *)&tx_buff,(uint8_t *)&tmp16,2,0);
	tmp = 0x00;
    usart_fifo_write((usart_fifo_t *)&tx_buff,&tmp,1,0);
	tmp16 = encodeValue(sensorsData.EngRPM);
	usart_fifo_write((usart_fifo_t *)&tx_buff,(uint8_t *)&tmp16,2,0);			

/*	tmp16 = encodeValue(0007); //Speed     - raw/3.97 km/h
	usart_fifo_write((usart_fifo_t *)&tx_buff,(uint8_t *)&tmp16,2,0);
	tmp = 0x00;
    usart_fifo_write((usart_fifo_t *)&tx_buff,&tmp,1,0);
	tmp16 = encodeValue(sensorsData.Speed);
	usart_fifo_write((usart_fifo_t *)&tx_buff,(uint8_t *)&tmp16,2,0);	
*/  
    /*
	Close = 0.5
	Open = 4.5
	*/
	
// FIXME	
/*	tmp16 = encodeValue(8); //ThrtlPos  - raw %
    usart_fifo_write((usart_fifo_t *)&tx_buff,(uint8_t *)&tmp16,2,0);
	tmp = 0x00;
    usart_fifo_write((usart_fifo_t *)&tx_buff,&tmp,1,0);
	tmp16 = encodeValue(sensorsData.ThrtlPos);
	usart_fifo_write((usart_fifo_t *)&tx_buff,(uint8_t *)&tmp16,2,0);	

/*	tmp16 = encodeValue(0013); //MAF       - raw (grams per second)
	usart_fifo_write((usart_fifo_t *)&tx_buff,(uint8_t *)&tmp16,2,0);
	tmp = 0x00;
    usart_fifo_write((usart_fifo_t *)&tx_buff,&tmp,1,0);
	tmp16 = encodeValue(sensorsData.MAF);
	usart_fifo_write((usart_fifo_t *)&tx_buff,(uint8_t *)&tmp16,2,0);

	tmp16 = encodeValue(17); //GasLvl    - raw %
	usart_fifo_write((usart_fifo_t *)&tx_buff,(uint8_t *)&tmp16,2,0);
	tmp = 0x00;
    usart_fifo_write((usart_fifo_t *)&tx_buff,&tmp,1,0);
	tmp16 = encodeValue(sensorsData.GasLvl);
	usart_fifo_write((usart_fifo_t *)&tx_buff,(uint8_t *)&tmp16,2,0);	
*/
// FIXME
    tmp16 = encodeValue(18); //AKBVolt   - raw/51.15 Volts
	usart_fifo_write((usart_fifo_t *)&tx_buff,(uint8_t *)&tmp16,2,0);
	tmp = 0x00;
    usart_fifo_write((usart_fifo_t *)&tx_buff,&tmp,1,0);
	tmp16 = encodeValue(sensorsData.AKBVolt);
	usart_fifo_write((usart_fifo_t *)&tx_buff,(uint8_t *)&tmp16,2,0);	
	
/*	tmp16 = encodeValue(0020); //FuelDty   - raw/10.23 Duty
	usart_fifo_write((usart_fifo_t *)&tx_buff,(uint8_t *)&tmp16,2,0);
	tmp = 0x00;
    usart_fifo_write((usart_fifo_t *)&tx_buff,&tmp,1,0);
	tmp16 = encodeValue(sensorsData.FuelDty);
	usart_fifo_write((usart_fifo_t *)&tx_buff,(uint8_t *)&tmp16,2,0);	
*/

	tmp = 0x40;
    usart_fifo_write((usart_fifo_t *)&tx_buff,&tmp,1,0);	
	
    // DMA channel Tx of USART Configuration
    DMA_Init(DMA1_Channel1, (uint16_t)tx_buff.buf, (uint16_t)0x5231, tx_buff.count, DMA_DIR_MemoryToPeripheral, DMA_Mode_Normal, DMA_MemoryIncMode_Inc, DMA_Priority_High, DMA_MemoryDataSize_Byte);	
	DMA_ITConfig(DMA1_Channel1,DMA_ITx_TC,ENABLE);
  
	// Enable the USART Tx DMA channel
	DMA_Cmd(DMA1_Channel1, ENABLE);	
}

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
	  //0x84,0x40 0x03 0x58 0x40 0x40 0x89 ,0x10 0x00 0x35 0x42 0x01 0x01 0x09 
	  
		case 0x84:
		  	if (pkt_len<7) {
			  cmd = 0;
			  break;
			}
	        DecodeAGATpkt();			
			//sensorsData.GasLvl = pkt_data[1]
			sensorsData.EngRPM = (uint16_t)(((*(uint16_t *)&pkt_data[1])*100UL)/1955UL);
			printf("RPM:%u %u\r\n",(*(uint16_t *)&pkt_data[1]),sensorsData.EngRPM);
			sensorsData.GazTemp = pkt_data[3];			
			sensorsData.AKBVolt = ((uint32_t)pkt_data[5]*5115UL)/1000;
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
  } else {
	pkt_data[pkt_len++] = ch;
    if ((cmd & 0x80) && (pkt_len>6)) ParseAGATpkt();
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
  
  uart_init();
  USART1_DMA1_Init();  
  
/*  printf("\r\nBuild Date is %s (%s)\r\n", __DATE__, __TIME__);
  uint8_t div = CLK->CKDIVR;
  printf("CPU Freq: %lu Hz, 0x%x 0x%x\r\n", CLK_GetClockFreq(), CLK->SCSR, div);
  
  uint16_t ref = 0x600 + FLASH_ReadByte(Factory_VREFINT);
    
  printf("Vref %u\r\n", ref);*/

  
  FLASH_DeInit();
  FLASH_Unlock(FLASH_MemType_Data);
  
  uint16_t optbl = ((uint16_t)FLASH_ReadByte(OPT_BL_ADDR_L)<<8) | FLASH_ReadByte(OPT_BL_ADDR_H);

  if (optbl!=0x55AA) {
    //printf("OPT_BL: 0x%x", optbl);
  
    FLASH_ProgramOptionByte(OPT_BL_ADDR_L,0x55);
    FLASH_ProgramOptionByte(OPT_BL_ADDR_H,0xAA);
  }
 
  /*GPIO_DeInit(GPIOC);
  GPIO_Init(GPIOC,GPIO_Pin_5|GPIO_Pin_6,GPIO_Mode_Out_PP_High_Fast);
  GPIO_ResetBits(GPIOC,GPIO_Pin_5|GPIO_Pin_6);*/
  
  TIM4_Init();
  TIM1_Init();
  ADC1_DMA0_Init();
  
  enableInterrupts();
  
  /* Infinite loop */
  while (1)
  {  
	if (adc_cnt) {
	  adc_cnt = 0;
	  
	  sim();
	  sensorsData.FluidTemp=getTemp(adc_summs[0]/adc_fifo[0].count);	  
	  sensorsData.Boost = getBoost(adc_summs[1]/adc_fifo[1].count);
	  sensorsData.OilTemp = getOilTemp(adc_summs[2]/adc_fifo[2].count);
	  sensorsData.ThrtlPos = calcThrtlPos(adc_summs[3]/adc_fifo[3].count);
	  rim();
	}
	if (rx_buff.count) {
		uint8_t ch;
		if (usart_fifo_read((usart_fifo_t *)&rx_buff,&ch,1)) BuildAGATpkt(ch);
	}
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
  }
}

PUTCHAR_PROTOTYPE
{
  /* Write a character to the USART */
  USART_SendData8(USART1, c);
  /* Loop until the end of transmission */
  while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);

  return (c);
}

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
 
  printf("Wrong parameters value: file %s on line %d\r\n", file, line);
  
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
