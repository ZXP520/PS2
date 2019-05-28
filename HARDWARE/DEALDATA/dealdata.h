#ifndef __DEALDATA_H
#define __DEALDATA_H 
#include "sys.h"
#include "include.h"

#define	DATAHEAD    0xDEED //����ͷ
#define RXDATALENTH 0x000A //�������ݳ���
#define TXDATALENTH 0x0A //�������ݳ���

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
	s16 LeftSpeed;		//�����ٶ�
	s16 RightSpeed;		//����
	s16 FLeftSpeed;		//ǰ��
	s16 FRightSpeed;	//ǰ��
	u8  StopFlag;			//ͣ����־
	u8  Navigation;		//������־
	u8  IMUNum;				//����������
}TX;

typedef enum 
{
	WhellDiameter=0,	//����ֱ��
	WheelBase,				//�������
	WhellRollSpeed,   //����ת��
	WhellSpeed,				//�����ٶ�
	ReductionRatio,   //���Ӽ��ٱ�
	WhellAcceleration,//���Ӽ��ٶ�
	EncoderLine, 			//����������
	EncoderValue,     //������ֵ
	IMUData,					//����������
	UltrasonicData,   //����������
	EmergencyStop,    //��ͣ״̬
	VersionNumber,    //�汾��
	RemainingBattery  //ʣ�����
	
}InquireCMD;//��ѯ����

typedef enum 
{
	SWhellRollSpeed=0x8000,   //����ת��
	SWhellSpeed,							//�����ٶ�
	STurningRadius,     			//���ӹ���뾶
	SWhellAcceleration,       //���Ӽ��ٶ�
	SChassisAttitude  				//������̬
	
}SetCMD;//��������

extern union TEMPDATA{
    s16  InTempData[9];
    u8 	 ChTempData[18];
}TempData;

void SendData_To_Ros(void);
void TestSendData_To_Ros(void);
void DealRXData(void);
void Respond_To_Ros(void);
#endif
