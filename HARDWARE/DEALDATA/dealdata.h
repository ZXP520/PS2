#ifndef __DEALDATA_H
#define __DEALDATA_H 
#include "sys.h"
#include "include.h"

#define	DATAHEAD    0xDEED //数据头
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

typedef enum 
{
	WhellDiameter=0,	//轮子直径
	WheelBase,				//轮子轴距
	WhellRollSpeed,   //轮子转速
	WhellSpeed,				//轮子速度
	ReductionRatio,   //轮子减速比
	WhellAcceleration,//轮子加速度
	EncoderLine, 			//编码器线数
	EncoderValue,     //编码器值
	IMUData,					//陀螺仪数据
	UltrasonicData,   //超声波数据
	EmergencyStop,    //急停状态
	VersionNumber,    //版本号
	RemainingBattery  //剩余电量
	
}InquireCMD;//查询命令

typedef enum 
{
	SWhellRollSpeed=0x8000,   //轮子转速
	SWhellSpeed,							//轮子速度
	STurningRadius,     			//轮子拐弯半径
	SWhellAcceleration,       //轮子加速度
	SChassisAttitude  				//底盘姿态
	
}SetCMD;//设置命令

extern union TEMPDATA{
    s16  InTempData[9];
    u8 	 ChTempData[18];
}TempData;

void SendData_To_Ros(void);
void TestSendData_To_Ros(void);
void DealRXData(void);
void Respond_To_Ros(void);
#endif
