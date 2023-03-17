#ifndef __FALLDETECTION_H 
#define __FALLDETECTION_H 
#include "adxl345.h"

#define RT_SPORT 0xB5 				//运动状态
#define RT_STATIC 0xA5					//静止状态

//用于跌倒检测算法的定义
#define FREE_FALL_THRESHOLD 		0x09 //62.5mg/LSB, 0x0C=0.75g   失重中断加速度阈值
#define FREE_FALL_TIME 				0x06 //5ms/LSB, 0x06=30ms 	    失重中断时间阈值
#define FREE_FALL_OVERTIME 			0x0F //20ms/LSB, 0x0F=15=300ms  超过0.45m的高度跌落时FF连续中断产生持续时间
#define FREE_FALL_INTERVAL 			0x05 //20ms/LSB, 0x05=100ms	    超过0.45m的高度跌落时FF连续中断产生的间隔
#define STRIKE_THRESHOLD 			0x20 //62.5mg/LSB, 0x20=2g 		撞击中断加速度阈值
#define STRIKE_WINDOW 				0x0A //20ms/LSB, 0x0A=10=200ms 	FF与ACT的时间间隔
#define STABLE_THRESHOLD 			0x10 //62.5mg/LSB, 0x10=0.5g 	跌倒后检测ACT中断
#define STABLE_TIME 				0x02 //1s/LSB, 0x02=2s 			静止中断时间阈值
#define STABLE_WINDOW 				0xAF //20ms/LSB, 0xAF=175=3.5s 	ACT与INACT的时间间隔
#define NOMOVEMENT_THRESHOLD 		0x03 //62.5mg/LSB, 0x03=0.1875g 静止中断加速度阈值
#define NOMOVEMENT_TIME 			0x0A //1s/LSB, 0x0A=10s			跌倒后静止时间
#define DELTA_VECTOR_SUM_THRESHOLD  0x7D70 //1g=0xFF, 0x7D70=0.7g^2
//用于跌倒检测算法的变量
extern unsigned char DetectionStatus; 
// Detection status: 
// 0xF0: 开始 
// 0xF1: 失重 
// 0xF2: 失重后撞击 
// 0xF3: 撞击后稳定，有效跌倒检测
// 0xF4: 与初始状态比较，有效临界跌倒阈值
// 0xFF: FF连续，从高的地方跌落 
extern unsigned char TimerWaitForStable; // 撞击后等待稳定的时间计数器
extern unsigned char TimerWaitForStrike; // 失重后等待撞击的时间计数器
extern unsigned char TimerFreeFall; // 连续FF的时间计数器 
extern short InitialStatus[3]; //  X-, Y-, Z- 轴的初始状态加速度 
extern short Acceleration[3]; // X-, Y-, Z- 轴的当前加速度
extern short Acceleration_recv[3];
extern unsigned long int DeltaAcceleration[3]; // 当前加速度与初始化状态加速度的差值
extern unsigned long int DeltaVectorSum; // 加速度差值的向量和

u8 ADXL345_Init(void);
void printshow(unsigned char status);
void FallDetection(void);
void buzzer_check(void);
void get_status(void);

#endif
