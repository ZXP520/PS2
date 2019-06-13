#include "dealdata.h"
#include "sys.h" 
#include "delay.h"
#include "control.h"
#include "timer.h"
#include "usart.h"	
#include "gy85.h"
#include "Encoder.h"
#include "bsp_usart.h"
#include "pstwo.h"
#include "include.h"

//左摇杆控制任意方向，右摇杆控制旋转
//定义结构体
TX Tx;

//PS2遥控器数据处理50ms进一次
void DealPs2Data(void )
{
	s16 speed_RX=0,speed_RY=0,speed_LX=0,speed_LY=0; 
	static u8 key=0;
	key=PS2_DataKey();
	if( Data[1] == PS_ID) //判断是否为红灯模式(id正确)
	{
		speed_RX=PS2_AnologData(PSS_RX);
		speed_RY=PS2_AnologData(PSS_RY);
		speed_LX=PS2_AnologData(PSS_LX);
		speed_LY=PS2_AnologData(PSS_LY);
		//printf(" LX:%5d LY:%5d RX:%5d RY:%5d\r\n",speed_LX,speed_LY,speed_RX, speed_RY);
	
	#if 	VERSION==0
		speed_RX=(speed_RX-128)*2.6;
		speed_LY=(127-speed_LY)*2.5;
		
		if(speed_LY>0) //前
		{
			if(speed_RX>0)//前右
			{
				Tx.LeftSpeed=speed_LY+speed_RX/2;		//左轮速度
				Tx.RightSpeed=speed_LY-speed_RX/2;		        //右轮速度
				Tx.StopFlag=0;		//停车信号
			}
			else if(speed_RX<0)//前左
			{
				Tx.LeftSpeed=speed_LY+speed_RX/2;		//左轮速度
				Tx.RightSpeed=speed_LY-speed_RX/2;		//右轮速度
				Tx.StopFlag=0;		//停车信号
			}
			else
			{
				Tx.LeftSpeed=speed_LY;		//左轮速度
				Tx.RightSpeed=speed_LY;		        //右轮速度
				Tx.StopFlag=0;		//停车信号
			}
		}
		else if(speed_LY<0)//后
		{
			if(speed_RX>0)//后左
			{
				Tx.LeftSpeed=speed_LY+speed_RX/2;		//左轮速度
				Tx.RightSpeed=speed_LY-speed_RX/2;		//右轮速度
				Tx.StopFlag=0;		//停车信号
			}
			else if(speed_RX<0)//后右
			{
				Tx.LeftSpeed=speed_LY+speed_RX/2;		//左轮速度
				Tx.RightSpeed=speed_LY-speed_RX/2;		//右轮速度
				Tx.StopFlag=0;		//停车信号
			}
			else
			{
				Tx.LeftSpeed =speed_LY;						//左轮速度
				Tx.RightSpeed=speed_LY;		        //右轮速度
				Tx.StopFlag=0;		//停车信号
			}
		}
		else if(key==0)//停
		{
			Tx.LeftSpeed=0;		  //左轮速度
			Tx.RightSpeed=0;		//右轮速度
			Tx.StopFlag=0;		//停车信号
		}
	#elif VERSION==1
		
		speed_LX=(speed_LX-128)*2.5;
		speed_LY=(127-speed_LY)*2.5;
		speed_RX=(speed_RX-128)/50;
		
		//按键和摇杆结合
		switch(key)
		{
			case 0:break;
			case PSB_PAD_UP:   speed_LY=+300;break;
			case PSB_PAD_RIGHT:speed_LX=+300;break;
			case PSB_PAD_DOWN: speed_LY=-300;break;
			case PSB_PAD_LEFT: speed_LX=-300;break;
			default:break;
		}
		
		Tx.FLeftSpeed=speed_LX;		  //X轴速度
		Tx.FRightSpeed=speed_LY;		  //Y轴速度
		Tx.LeftSpeed=speed_RX*100/2;		  //角速度 0-100
		Tx.RightSpeed=key;							//扩展功能键
		
		
		if(speed_LX==0&&speed_LY==0&&speed_RX==0)
		{
				Tx.StopFlag=1;		//停车信号
		}
		else
		{
			Tx.StopFlag=0;		//停车信号
		}
	#endif
	//printf("%d	%d	%d\n",Tx.LeftSpeed,Tx.RightSpeed,TXData.InRxData[4]);
	}
	else	//手柄不是红灯模式  --停车，且左右轮速度为0
	{
		//如果是全向轮小车，则为Y,X轴速度
		
		Tx.LeftSpeed=0;		//左轮速度
		Tx.RightSpeed=0;	//右轮速度
		Tx.FLeftSpeed=0;
		Tx.FRightSpeed=0;
		Tx.StopFlag=1;		//停车信号
	}

}

//转换数据共用体定义
union TEMPDATA TempData;
//返回数据给ROS
void Respond_To_Ros(void)
{
	s16 Cheksum=0;//校验和
	u8 i=0; 
	
	DealPs2Data();
	
	TXData.InRxData[0]=DATAHEAD;										//头
#if 	VERSION==0
	TXData.ChRxData[2]=0x0D;			//帧长度
	TXData.ChRxData[3]=SWhellSpeed&0xFF;			//命令   小端模式先低后高
	TXData.ChRxData[4]=(SWhellSpeed>>8)&0xFF;
	TXData.ChRxData[5]=0x01;
	TXData.ChRxData[6]=0x04;					//数据个数
#elif VERSION==1
	TXData.ChRxData[2]=0X11;//0x0F;			//帧长度
	TXData.ChRxData[3]=SChassisAttitude&0xFF;			//命令   小端模式先低后高
	TXData.ChRxData[4]=(SChassisAttitude>>8)&0xFF;
	TXData.ChRxData[5]=0x01;
	TXData.ChRxData[6]=0x08;//0x06;					//数据个数
#endif
	
	TempData.InTempData[0]=Tx.FLeftSpeed;
	TempData.InTempData[1]=Tx.FRightSpeed;
	TempData.InTempData[2]=Tx.LeftSpeed;
	TempData.InTempData[3]=Tx.RightSpeed;
	
	
	switch(TXData.ChRxData[6])
	{
		case 4:for(i=0;i<4;i++){TXData.ChRxData[7+i]=TempData.ChTempData[i];}break;
		case 6:for(i=0;i<6;i++){TXData.ChRxData[7+i]=TempData.ChTempData[i];}break;
		case 8:for(i=0;i<8;i++){TXData.ChRxData[7+i]=TempData.ChTempData[i];}break;
		default:break;
	}
	
	
	//计算校验值
	for(i=0;i<TXData.ChRxData[2]-2;i++)
	{
		Cheksum+=TXData.ChRxData[i];
	}
	TXData.ChRxData[TXData.ChRxData[2]-2]=Cheksum&0xFF; //校验和
	TXData.ChRxData[TXData.ChRxData[2]-1]=(Cheksum>>8)&0xFF;
	//DMA串口发送数据
	USART2_DMA_TX(TXData.ChRxData,TXData.ChRxData[2]);
}

