#ifndef __DEALDATA_H
#define __DEALDATA_H 
#include "sys.h"
#include "include.h"

#define	DATAHEAD    0xA5A5 //数据头
#define RXDATALENTH 0x000A //接收数据长度
#define TXDATALENTH 0x0A //发送数据长度

#if  	VERSION==0
	#define DATACMD 0X02
#elif VERSION==1
	#define DATACMD 0X03
#elif VERSION==2
	#define DATACMD 0X04
#endif

#define DATALENTH 0x06

typedef struct
{
	s16 LeftSpeed;		//左轮速度
	s16 RightSpeed;		//右轮
	s16 FLeftSpeed;		//前左
	s16 FRightSpeed;	//前右
	u8  StopFlag;			//停车标志
	u8  Navigation;		//导航标志
	u8  IMUNum;				//陀螺仪轴数
}TX;



void SendData_To_Ros(void);
void TestSendData_To_Ros(void);
void DealRXData(void);
#endif
