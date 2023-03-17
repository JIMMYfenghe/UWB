#ifndef __FALLDETECTION_H 
#define __FALLDETECTION_H 
#include "adxl345.h"

#define RT_SPORT 0xB5 				//�˶�״̬
#define RT_STATIC 0xA5					//��ֹ״̬

//���ڵ�������㷨�Ķ���
#define FREE_FALL_THRESHOLD 		0x09 //62.5mg/LSB, 0x0C=0.75g   ʧ���жϼ��ٶ���ֵ
#define FREE_FALL_TIME 				0x06 //5ms/LSB, 0x06=30ms 	    ʧ���ж�ʱ����ֵ
#define FREE_FALL_OVERTIME 			0x0F //20ms/LSB, 0x0F=15=300ms  ����0.45m�ĸ߶ȵ���ʱFF�����жϲ�������ʱ��
#define FREE_FALL_INTERVAL 			0x05 //20ms/LSB, 0x05=100ms	    ����0.45m�ĸ߶ȵ���ʱFF�����жϲ����ļ��
#define STRIKE_THRESHOLD 			0x20 //62.5mg/LSB, 0x20=2g 		ײ���жϼ��ٶ���ֵ
#define STRIKE_WINDOW 				0x0A //20ms/LSB, 0x0A=10=200ms 	FF��ACT��ʱ����
#define STABLE_THRESHOLD 			0x10 //62.5mg/LSB, 0x10=0.5g 	��������ACT�ж�
#define STABLE_TIME 				0x02 //1s/LSB, 0x02=2s 			��ֹ�ж�ʱ����ֵ
#define STABLE_WINDOW 				0xAF //20ms/LSB, 0xAF=175=3.5s 	ACT��INACT��ʱ����
#define NOMOVEMENT_THRESHOLD 		0x03 //62.5mg/LSB, 0x03=0.1875g ��ֹ�жϼ��ٶ���ֵ
#define NOMOVEMENT_TIME 			0x0A //1s/LSB, 0x0A=10s			������ֹʱ��
#define DELTA_VECTOR_SUM_THRESHOLD  0x7D70 //1g=0xFF, 0x7D70=0.7g^2
//���ڵ�������㷨�ı���
extern unsigned char DetectionStatus; 
// Detection status: 
// 0xF0: ��ʼ 
// 0xF1: ʧ�� 
// 0xF2: ʧ�غ�ײ�� 
// 0xF3: ײ�����ȶ�����Ч�������
// 0xF4: ���ʼ״̬�Ƚϣ���Ч�ٽ������ֵ
// 0xFF: FF�������Ӹߵĵط����� 
extern unsigned char TimerWaitForStable; // ײ����ȴ��ȶ���ʱ�������
extern unsigned char TimerWaitForStrike; // ʧ�غ�ȴ�ײ����ʱ�������
extern unsigned char TimerFreeFall; // ����FF��ʱ������� 
extern short InitialStatus[3]; //  X-, Y-, Z- ��ĳ�ʼ״̬���ٶ� 
extern short Acceleration[3]; // X-, Y-, Z- ��ĵ�ǰ���ٶ�
extern short Acceleration_recv[3];
extern unsigned long int DeltaAcceleration[3]; // ��ǰ���ٶ����ʼ��״̬���ٶȵĲ�ֵ
extern unsigned long int DeltaVectorSum; // ���ٶȲ�ֵ��������

u8 ADXL345_Init(void);
void printshow(unsigned char status);
void FallDetection(void);
void buzzer_check(void);
void get_status(void);

#endif
