#include "timer.h"
#include "led.h"
#include "stm32f10x_tim.h"
#include "control.h"	
#include "include.h"
//////////////////////////////////////////////////////////////////////////////////	 
//PWM输出初始化
//arr：自动重装值
//psc：时钟预分频数
void TIM8_PWM_Init(u16 arr,u16 psc)
{  
	
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);   //增加了I2C: RCC_APB1Periph_I2C1
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC|RCC_APB2Periph_AFIO, ENABLE);  //使能GPIO外设时钟使能	                                                                     	
	//GPIO_PinRemapConfig(GPIO_Remap_TIM8, ENABLE); //完全重映射 TIM8_CH2->PC6/7/8/9
	
   //设置该引脚为复用输出功能,输出TIM8 CH1\CH2\CH3\CH4的PWM脉冲波形
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	 80K
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值  不分频
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //选择定时器模式:TIM脉冲宽度调制模式2
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_Pulse = 0; //设置待装入捕获比较寄存器的脉冲值
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 	//TIM_OCPolarity_Low; //输出极性:TIM输出比较极性高
	TIM_OC1Init(TIM8, &TIM_OCInitStructure);  //根据TIM_OCInitStruct中指定的参数初始化外设TIMx CH1 PC6
	TIM_OC2Init(TIM8, &TIM_OCInitStructure);  //根据TIM_OCInitStruct中指定的参数初始化外设TIMx CH2 PC7
	TIM_OC3Init(TIM8, &TIM_OCInitStructure);  //根据TIM_OCInitStruct中指定的参数初始化外设TIMx CH3 PC8
	TIM_OC4Init(TIM8, &TIM_OCInitStructure);  //根据TIM_OCInitStruct中指定的参数初始化外设TIMx CH4 PC9
	
	
  TIM_CtrlPWMOutputs(TIM8,ENABLE);	//MOE 主输出使能	

	TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);  //CH1预装载使能	PB6  
	TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);  //CH2预装载使能	PB7
	TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);  //CH2预装载使能	PB8
	TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable);  //CH2预装载使能	PB9
	
	TIM_ARRPreloadConfig(TIM8, ENABLE); //使能TIMx在ARR上的预装载寄存器
	TIM_Cmd(TIM8, ENABLE);  //使能TIM8
	
	TIM_SetCompare1(TIM8,0); //左转
	TIM_SetCompare2(TIM8,0);
	TIM_SetCompare3(TIM8,0);  
	TIM_SetCompare4(TIM8,0);	
		
}

//PWM输出初始化
//arr：自动重装值
//psc：时钟预分频数
void TIM1_PWM_Init(u16 arr,u16 psc)
{  
	
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);   //增加了I2C: RCC_APB1Periph_I2C1
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO, ENABLE);  //使能GPIO外设时钟使能	                                                                     	
	//GPIO_PinRemapConfig(GPIO_Remap_TIM1, ENABLE); //完全重映射 TIM1_CH2->PC6/7/8/9
	
   //设置该引脚为复用输出功能,输出TIM1 CH1\CH2\CH3\CH4的PWM脉冲波形
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	 80K
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值  不分频
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //选择定时器模式:TIM脉冲宽度调制模式2
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_Pulse = 0; //设置待装入捕获比较寄存器的脉冲值
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 	//TIM_OCPolarity_Low; //输出极性:TIM输出比较极性高
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);  //根据TIM_OCInitStruct中指定的参数初始化外设TIMx CH1 PC6
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);  //根据TIM_OCInitStruct中指定的参数初始化外设TIMx CH2 PC7
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);  //根据TIM_OCInitStruct中指定的参数初始化外设TIMx CH3 PC8
	TIM_OC4Init(TIM1, &TIM_OCInitStructure);  //根据TIM_OCInitStruct中指定的参数初始化外设TIMx CH4 PC9
	
	
  TIM_CtrlPWMOutputs(TIM1,ENABLE);	//MOE 主输出使能	

	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);  //CH1预装载使能	PB6  
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);  //CH2预装载使能	PB7
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);  //CH2预装载使能	PB8
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);  //CH2预装载使能	PB9
	
	TIM_ARRPreloadConfig(TIM1, ENABLE); //使能TIMx在ARR上的预装载寄存器
	TIM_Cmd(TIM1, ENABLE);  //使能TIM1
	
	TIM_SetCompare1(TIM1,0); //左转
	TIM_SetCompare2(TIM1,0);
	TIM_SetCompare3(TIM1,0);  
	TIM_SetCompare4(TIM1,0);	
		
}



void TIM3_Configuration(void)//编码器接口设置TIM3/PA6-A相  PA7-B相
{
  GPIO_InitTypeDef GPIO_InitStructure; 
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef  TIM_ICInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE);  
	                                                       				
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;          
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	   
	GPIO_Init(GPIOA, &GPIO_InitStructure);				 
	GPIO_WriteBit(GPIOA, GPIO_Pin_6,Bit_SET);
	GPIO_WriteBit(GPIOA, GPIO_Pin_7,Bit_SET); 

  TIM_TimeBaseStructure.TIM_Period = EncoderPeriod; 
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;    
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); 
	
	//设置定时器3位编码器模式 IT1 IT2位上升沿计数
	TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12,TIM_ICPolarity_BothEdge,TIM_ICPolarity_BothEdge);
	TIM_ICStructInit(&TIM_ICInitStructure);
  TIM_ICInitStructure.TIM_ICFilter = 8;      
  TIM_ICInit(TIM3, &TIM_ICInitStructure);
  TIM_ClearFlag(TIM3, TIM_FLAG_Update);     
  TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE); 
  TIM3->CNT = 0;
	TIM_Cmd(TIM3, ENABLE);
}


void TIM4_Configuration(void)//编码器接口设置TIM4/PA6-A相  PA7-B相
{
  GPIO_InitTypeDef GPIO_InitStructure; 
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef  TIM_ICInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE);  
	                                                       				
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;          
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	   
	GPIO_Init(GPIOB, &GPIO_InitStructure);				 
	GPIO_WriteBit(GPIOB, GPIO_Pin_6,Bit_SET);
	GPIO_WriteBit(GPIOB, GPIO_Pin_7,Bit_SET); 

  TIM_TimeBaseStructure.TIM_Period = EncoderPeriod; 
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;    
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); 
	
	//设置定时器3位编码器模式 IT1 IT2位上升沿计数
	TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12,TIM_ICPolarity_BothEdge,TIM_ICPolarity_BothEdge);
	TIM_ICStructInit(&TIM_ICInitStructure);
  TIM_ICInitStructure.TIM_ICFilter = 8;      
  TIM_ICInit(TIM4, &TIM_ICInitStructure);
  TIM_ClearFlag(TIM4, TIM_FLAG_Update);     
  TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE); 
  TIM4->CNT = 0;
	TIM_Cmd(TIM4, ENABLE);
}



void TIM5_Configuration(void)//编码器接口设置TIM5/PA0-A相  PA1-B相
{
  GPIO_InitTypeDef GPIO_InitStructure; 
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef  TIM_ICInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE); 
	                                                       				
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;         
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	 
	GPIO_Init(GPIOA, &GPIO_InitStructure);				 
	GPIO_WriteBit(GPIOA, GPIO_Pin_0,Bit_SET);
	GPIO_WriteBit(GPIOA, GPIO_Pin_1,Bit_SET); 

  TIM_TimeBaseStructure.TIM_Period = EncoderPeriod;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure); 
	
	//设置定时器5位编码器模式 IT1 IT2位上升沿计数
	TIM_EncoderInterfaceConfig(TIM5, TIM_EncoderMode_TI12,TIM_ICPolarity_BothEdge,TIM_ICPolarity_BothEdge);
	TIM_ICStructInit(&TIM_ICInitStructure);
  TIM_ICInitStructure.TIM_ICFilter = 8;  
  TIM_ICInit(TIM5, &TIM_ICInitStructure);
  TIM_ClearFlag(TIM5, TIM_FLAG_Update); 
  TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
  TIM5->CNT = 0;
	TIM_Cmd(TIM5, ENABLE);
}




void Time_Config(void)
{
	//中断处理函数在stm32f10x_it.c里面
	SysTick_Config(SystemCoreClock/1000);//72000000/1000=72000  1MS中断
/*	
#if VERSION == 1
	TIM4_Configuration();
	TIM1_PWM_Init(TIM8_Period-1,30-1);  //分频。PWM频率=72000/5/1200=12Khz
#endif
	TIM3_Configuration();
	TIM5_Configuration();
	TIM8_PWM_Init(TIM8_Period-1,30-1);	//分频。PWM频率=72000/5/1200=12Khz
	*/
}



