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
	
	if( !PS2_RedLight()) //判断是否为红灯模式
	{
		Tdelay_us(50000);
		key=PS2_DataKey();
		
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
		
		Tx.LeftSpeed=speed_LX;		  //X轴速度
		Tx.RightSpeed=speed_LY;		  //Y轴速度
		Tx.FLeftSpeed=speed_RX;		  //角速度
		
		
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
		Tx.StopFlag=1;		//停车信号
	}

}


//数据测试函数
void TestSendData_To_Ros(void)
{
	int Cheksum=0;//校验和
	u8 i=0;
	
	DealPs2Data();
	
	//0
	TXData.InRxData[0]=DATAHEAD;  					//头
	
	//1-2
	TXData.ChRxData[2]=TXDATALENTH*2;				//贞长度
	TXData.ChRxData[3]=0x01;								//命令个数
	TXData.ChRxData[4]=DATACMD;							//命令
	TXData.ChRxData[5]=DATALENTH;						//数据长度
	
	//3-6
	TXData.InRxData[3]=Tx.LeftSpeed;				//左轮速度
	TXData.InRxData[4]=Tx.RightSpeed;				//右
	TXData.InRxData[5]=Tx.FLeftSpeed;				//前左
	TXData.InRxData[6]=Tx.FRightSpeed;			//前右
	//7
	TXData.ChRxData[15]=Tx.StopFlag;				//停车标志
	TXData.ChRxData[16]=Tx.Navigation;			//导航标志
	//8
  TXData.InRxData[8] =Tx.IMUNum;					//陀螺仪轴数
	
	for(i=0;i<TXDATALENTH-1;i++)
	{
		Cheksum+=TXData.InRxData[i];
	}
	//9
	TXData.InRxData[TXDATALENTH-1]=Cheksum; //校验和
	
	//DMA串口发送数据
	USART2_DMA_TX(TXData.ChRxData,TXData.InRxData[1]);
	
}
