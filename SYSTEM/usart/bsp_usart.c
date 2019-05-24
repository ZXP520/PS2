#include "bsp_usart.h"
#include <stdio.h>
#include <string.h>
#include "stm32f10x_dma.h"
#include "usart.h"
#include "dealdata.h"
#include "stdarg.h"      


static u8 USART2_Rx_Buff[MAX_RX_CNT];
static u8 USART2_Tx_Buff[MAX_RX_CNT];
//共用体定义
union Data TXData,RXData;

//串口2DMA发送 DMA1 通道7
uint8_t USART2_DMA_TX(const uint8_t *TxBuff , uint8_t Byte_Cnt)
{ 
	DMA_Cmd(DMA1_Channel7, DISABLE);
  
  if(Byte_Cnt>(sizeof(USART2_Tx_Buff)/sizeof(*USART2_Tx_Buff))){return 0;}
  
  memcpy(USART2_Tx_Buff,TxBuff,Byte_Cnt);
 	DMA_SetCurrDataCounter(DMA1_Channel7,Byte_Cnt);
 	DMA_Cmd(DMA1_Channel7, ENABLE);
  
  return 1;
}


void USART2_Config(uint32_t BaudRate)
{
	GPIO_InitTypeDef  gpio_init;
	USART_InitTypeDef usart_init;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA| RCC_APB2Periph_AFIO, ENABLE);

  gpio_init.GPIO_Pin = GPIO_Pin_2;
  gpio_init.GPIO_Mode = GPIO_Mode_AF_PP;
  gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &gpio_init);
	    
  gpio_init.GPIO_Pin = GPIO_Pin_3;
  gpio_init.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &gpio_init);

	usart_init.USART_BaudRate = BaudRate;
	usart_init.USART_WordLength = USART_WordLength_8b;
	usart_init.USART_StopBits = USART_StopBits_1;
	usart_init.USART_Parity = USART_Parity_No ;
	usart_init.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	usart_init.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &usart_init); 
	
  USART2_NVIC_Config();
  USART2_DMA_Config();
  
  
	USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);
	USART_Cmd(USART2, ENABLE);
	USART_ClearFlag(USART2, USART_FLAG_TC);
  

}


static void USART2_DMA_Config(void)
{
  DMA_InitTypeDef dma_init;

  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

  DMA_DeInit(DMA1_Channel6);
  DMA_DeInit(DMA1_Channel7);
  
  dma_init.DMA_PeripheralBaseAddr = (uint32_t)&(USART2->DR);
  dma_init.DMA_MemoryBaseAddr = (uint32_t)USART2_Rx_Buff;
  dma_init.DMA_DIR = DMA_DIR_PeripheralSRC ;	
  dma_init.DMA_BufferSize = MAX_RX_CNT;
  dma_init.DMA_PeripheralInc = DMA_PeripheralInc_Disable; 
  dma_init.DMA_MemoryInc = DMA_MemoryInc_Enable;	
  dma_init.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  dma_init.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;	 
  dma_init.DMA_Mode = DMA_Mode_Normal ;
  dma_init.DMA_Priority = DMA_Priority_VeryHigh;
  dma_init.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel6, &dma_init); 	   
  DMA_Cmd (DMA1_Channel6,ENABLE);

  dma_init.DMA_PeripheralBaseAddr = (uint32_t)&(USART2->DR);
  dma_init.DMA_MemoryBaseAddr = (uint32_t)USART2_Tx_Buff;
  dma_init.DMA_DIR = DMA_DIR_PeripheralDST ;	
  dma_init.DMA_BufferSize = 9;
  dma_init.DMA_PeripheralInc = DMA_PeripheralInc_Disable; 
  dma_init.DMA_MemoryInc = DMA_MemoryInc_Enable;	
  dma_init.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  dma_init.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;	 
  dma_init.DMA_Mode = DMA_Mode_Normal ;
  dma_init.DMA_Priority = DMA_Priority_Medium;
  dma_init.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel7, &dma_init); 	   
//  DMA_Cmd (DMA1_Channel7,ENABLE);//传输的时候再开启

	USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);
  USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);
}



static void USART2_NVIC_Config(void)
{
  NVIC_InitTypeDef  usart_nvic_init; 
  
	usart_nvic_init.NVIC_IRQChannel = DMA1_Channel6_IRQn ;	
	usart_nvic_init.NVIC_IRQChannelPreemptionPriority = 0;
	usart_nvic_init.NVIC_IRQChannelSubPriority = 1;
	usart_nvic_init.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&usart_nvic_init);
	
	usart_nvic_init.NVIC_IRQChannel = USART2_IRQn;	
	usart_nvic_init.NVIC_IRQChannelPreemptionPriority = 0;
	usart_nvic_init.NVIC_IRQChannelSubPriority = 2;
	usart_nvic_init.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&usart_nvic_init);
}





/********串口2空闲中断函数*******/	
void USART2_IRQHandler(void)
{
  if(USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)
  {
    uint8_t temp;
    temp = USART2->SR;
    temp = USART2->DR;
    
    DMA_Cmd (DMA1_Channel6,DISABLE);
		memcpy(RXData.ChRxData,USART2_Rx_Buff,DMA_GetCurrDataCounter(DMA1_Channel6));
    //Rx_Data_2_Queue( USART2_Rx_Buff, MAX_RX_CNT - DMA_GetCurrDataCounter(DMA1_Channel6));
    DMA_SetCurrDataCounter(DMA1_Channel6,MAX_RX_CNT);
    DMA_Cmd (DMA1_Channel6,ENABLE);
		//DealRXData();
  }
  
}

/**********DMA1通道6接收中断****/
void DMA1_Channel6_IRQHandler(void)
{
    if(DMA_GetFlagStatus(DMA1_FLAG_TC6)!=RESET) 
    {
      DMA_ClearITPendingBit(DMA1_IT_TC6);
    }
}




