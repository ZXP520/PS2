#include "mpu6050.h"
#include "myiic.h"

void InitMPU6050(void) //初始化Mpu6050
{
	 IIC_Init();
	 Single_Write(0x1A,0x0B,0x01); 
   Single_Write(0x1A,0x20,0x40);
   Single_Write(0x1A,0x21,0x01);
   Single_Write(0x1A,0x09,0x0d); //
	
	 Single_Write(0xA6,0x31,0x0B);   //测量范围,正负16g，13位模式
   Single_Write(0xA6,0x2D,0x08);   //选择电源模式   参考pdf24页
   Single_Write(0xA6,0x2E,0x80);   //使能 DATA_READY 中断
	
	 Single_Write(SlaveAddress,PWR_M, 0x80);   //
   Single_Write(SlaveAddress,SMPL, 0x07);    //
   Single_Write(SlaveAddress,DLPF, 0x1E);    //±2000°
   Single_Write(SlaveAddress,INT_C, 0x00 );  //
   Single_Write(SlaveAddress,PWR_M, 0x00);   //
	 
}

unsigned int GetData(uint8_t SlaveAddr,unsigned char REG_Address) //获得16位数据
{
	char H,L;
	H=GetData(SlaveAddr,REG_Address);
	L=GetData(SlaveAddr,REG_Address+1);
	return (H<<8)+L;   //合成数据
}

unsigned int GetQMC5883Data(uint8_t SlaveAddr,unsigned char REG_Address) //获得16位数据
{
	char H,L;
	L=GetData(SlaveAddr,REG_Address);
	H=GetData(SlaveAddr,REG_Address+1);
	return (H<<8)+L;   //合成数据
}




