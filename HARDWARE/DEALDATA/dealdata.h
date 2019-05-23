#ifndef __DEALDATA_H
#define __DEALDATA_H 
#include "sys.h"
#include "include.h"

#define	DATAHEAD    0xA5A5 //����ͷ
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



void SendData_To_Ros(void);
void TestSendData_To_Ros(void);
void DealRXData(void);
#endif
