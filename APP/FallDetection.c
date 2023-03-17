/*---------------------------------------------------------------
*file:		falldetection.c
*content:	检测跌倒
---------------------------------------------------------------*/
#include "FallDetection.h"
#include "oled.h"
//#include "usart.h"
extern u8 buzzer;
unsigned char DetectionStatus; 
unsigned char TimerWaitForStable; // 撞击后等待稳定的时间计数器
unsigned char TimerWaitForStrike; // 失重后等待撞击的时间计数器
unsigned char TimerFreeFall; // 连续FF的时间计数器 
short InitialStatus[3]; //  X-, Y-, Z- 轴的初始状态加速度 
short Acceleration[3]; // X-, Y-, Z- 轴的当前加速度
short Acceleration_recv[3];
unsigned long int DeltaAcceleration[3]; // 当前加速度与初始化状态加速度的差值
unsigned long int DeltaVectorSum; // 加速度差值的向量和



static int i,j,status_flag;
static int sampling_cnt=0;
short buf[3],rebuf0[4],rebuf1[4],rebuf2[4],rebuf3[4],regulation[4];
short filter_out[3],filter_max[3]={0,0,0},filter_min[3]={0xFFF,0xFFF,0xFFF};
short Vpp[3],Vdc[3],old_sample[3],new_sample[3],Dynamic_precision[3];//峰峰值，门限值，新旧采样值，动态精度
u8 error_flag[3];




/*-----------------------------------------
*函数：		ADXL345_Init
*功能：		ADXL345配置初始化
*返回值：	
			0	：	成功
			1	：	失败
------------------------------------------*/
u8 ADXL345_Init(void)
{
	
	if(ADXL345_RD_Reg(XL345_DEVID) == XL345_ID) //读取器件ID
    { 
			ADXL345_WR_Reg(XL345_OFSX,0xFF);
			ADXL345_WR_Reg(XL345_OFSY,0x05);
			ADXL345_WR_Reg(XL345_OFSZ,0xFF);
			ADXL345_WR_Reg(XL345_THRESH_ACT,STRIKE_THRESHOLD);
			ADXL345_WR_Reg(XL345_THRESH_INACT,NOMOVEMENT_THRESHOLD);
			ADXL345_WR_Reg(XL345_TIME_INACT,STABLE_TIME);
			ADXL345_WR_Reg(XL345_ACT_INACT_CTL,0x7F);	//ACT--DC耦合，INACT--AC耦合，XYZ耦合使能
			
			ADXL345_WR_Reg(XL345_THRESH_FF,FREE_FALL_THRESHOLD);
			ADXL345_WR_Reg(XL345_TIME_FF,FREE_FALL_TIME);
					
			ADXL345_WR_Reg(XL345_POWER_CTL,XL345_STANDBY);//待机模式
			ADXL345_WR_Reg(XL345_BW_RATE,XL345_RATE_50);//数据输出速率100Hz
			ADXL345_WR_Reg(XL345_INT_MAP,0x00);	//所有中断都设置到INT1
			//ADXL345_WR_Reg(XL345_FIFO_CTL,0x1f);	//不适用FIFO，旁路模式
			ADXL345_WR_Reg(XL345_INT_ENABLE,0x1C);//使能ACT INACT FF中断
			ADXL345_WR_Reg(XL345_DATA_FORMAT,0x2B);		//全分辨率，右对齐，+-16g
			ADXL345_WR_Reg(XL345_POWER_CTL,XL345_MEASURE);
			ADXL345_RD_Reg(XL345_INT_SOURCE);

			DetectionStatus=0xF0; 		//开始	
			printshow(DetectionStatus);
			
			InitialStatus[0]=0x1000; // X axis=0g, unsigned short int, 13 bit resolution, 0x1000 = 4096 = 0g, +/-0xFF = +/-256 = +/-1g 
			InitialStatus[1]=0x0F00; // Y axis=-1g 
			InitialStatus[2]=0x1000; // Z axis=0g
			
			return 0;
    }  
	else
	{	
		return 1;  
	}	
}

/*---------------------------------------
*函数：		printshow
*功能：		打印状态
*输入：		status:	状态值
*输出：		无
*返回值：	无
---------------------------------------*/
void printshow(unsigned char status)
{
	switch(status)
	{
		case(0xF0):
			#ifdef DEBUG_TEST
				printf("0-start!\n");
			#endif
				break;
		case(0xF1):
			#ifdef DEBUG_TEST
				printf("1-weightless!\n");
			#endif	
				break;
		case(0xF2):
			#ifdef DEBUG_TEST
				printf("2-activity after weightless!\n");
			#endif
				break;
		case(0xF3):
			#ifdef DEBUG_TEST
				printf("3-inavtivity after activity!\n");
			#endif
				break;
		case(0xF4):
			#ifdef DEBUG_TEST
				printf("4-falled!\n");
			#endif
				break;
		case(0xF5):
			#ifdef DEBUG_TEST
				printf("5-keep inactivity after falled for 10 seconds!\n");
			#endif
				break;
		case(0xFF):
			#ifdef DEBUG_TEST
				printf("6-falled from over 0.45meters high place!\n");
			#endif
				break;
		default:break;
	}
}

/*-------------------------------------
*函数：		EXTI1_IRQHandler
*功能：		ADXL345中断处理函数，此处用于检测跌倒的算法
*返回值：	无
--------------------------------------*/
u8 test=0;
void EXTI1_IRQHandler(void) 
{ 
	u8 i;
	EXTI_ClearITPendingBit(EXTI_Line1);
	NVIC_DisableIRQ(EXTI1_IRQn);	
	test=ADXL345_RD_Reg(XL345_INT_SOURCE);
		if((test & XL345_ACTIVITY)==XL345_ACTIVITY)//撞击中断
		{			
			
			if(DetectionStatus == 0xF1)//等待撞击，且撞击被检测到
			{
				DetectionStatus=0xF2;
				buzzer=0;
				#ifdef DEBUG_TEST
				printshow(DetectionStatus);
				#endif
				ADXL345_WR_Reg(XL345_THRESH_ACT,STRIKE_THRESHOLD);//2g?
				ADXL345_WR_Reg(XL345_THRESH_INACT,NOMOVEMENT_THRESHOLD);//0.1875g
				ADXL345_WR_Reg(XL345_TIME_INACT,STABLE_TIME);//2s
				ADXL345_WR_Reg(XL345_ACT_INACT_CTL,0xFF);
				TIM_Cmd(TIM2,ENABLE); //使能定时器
				TimerWaitForStable=0;
			}
			else if(DetectionStatus == 0xF4)//撞击后长时间静止不动
			{
				DetectionStatus=0xF0;
				buzzer=0;
				#ifdef DEBUG_TEST
				printshow(DetectionStatus);
				#endif
				ADXL345_WR_Reg(XL345_THRESH_ACT,STRIKE_THRESHOLD);//2g
				ADXL345_WR_Reg(XL345_THRESH_INACT,NOMOVEMENT_THRESHOLD);//0.1875g
				ADXL345_WR_Reg(XL345_TIME_INACT,STABLE_TIME);//2s
				ADXL345_WR_Reg(XL345_ACT_INACT_CTL,0x7F);
			}
		}
		
		else if((test & XL345_INACTIVITY)==XL345_INACTIVITY)//静止中断
		{
			
			if(DetectionStatus == 0xF2)//等待静止，且静止被检测到
			{
				DetectionStatus=0xF3;
				buzzer=0;
				#ifdef DEBUG_TEST
				printshow(DetectionStatus);
				#endif
				TIM_Cmd(TIM2,DISABLE); //disable定时器
				ADXL345_RD_XYZ(&Acceleration_recv[0],&Acceleration_recv[1],&Acceleration_recv[2]);
				DeltaVectorSum=0;
				for(i=0;i<3;i++)
				{
					Acceleration[i]=(Acceleration_recv[i]>>8)&0x1F; 
					Acceleration[i]=(Acceleration[i]<<8)|(Acceleration_recv[i]&0xff);
					if(Acceleration[i] < 0x1000)//0x1000 = 0g
						Acceleration[i] += 0x1000;
					else if(Acceleration[i] >= 0x1000)
						Acceleration[i] -= 0x1000;
					if(Acceleration[i] > InitialStatus[i])
						DeltaAcceleration[i] = Acceleration[i] - InitialStatus[i];
					else	
						DeltaAcceleration[i] = InitialStatus[i] - Acceleration[i];
					DeltaVectorSum += DeltaAcceleration[i]*DeltaAcceleration[i];
				}
				
				//与初始状态比较检测，若矢量差超过0.7g则认为是有效跌倒
				if(DeltaVectorSum > DELTA_VECTOR_SUM_THRESHOLD)
				{
					DetectionStatus=0xF4;//有效跌倒检测
					buzzer=1;
					#ifdef DEBUG_TEST
					printshow(DetectionStatus);
					#endif
					ADXL345_WR_Reg(XL345_THRESH_ACT,STABLE_THRESHOLD);//0.5g
					ADXL345_WR_Reg(XL345_THRESH_INACT,NOMOVEMENT_THRESHOLD);//0.1875g
					ADXL345_WR_Reg(XL345_TIME_INACT,NOMOVEMENT_TIME);//10s
					ADXL345_WR_Reg(XL345_ACT_INACT_CTL,0xFF);
				}
				else
				{
					DetectionStatus=0xF0;
					buzzer=0;
					#ifdef DEBUG_TEST
					printshow(DetectionStatus);
					#endif
					ADXL345_WR_Reg(XL345_THRESH_ACT,STRIKE_THRESHOLD);//2g
					ADXL345_WR_Reg(XL345_THRESH_INACT,NOMOVEMENT_THRESHOLD);//0.1875g
					ADXL345_WR_Reg(XL345_TIME_INACT,STABLE_TIME);//2s
					ADXL345_WR_Reg(XL345_ACT_INACT_CTL,0x7F);
				}
					
			}
			
			else if(DetectionStatus == 0xF4)//长时间稳定静止
			{
				DetectionStatus=0xF5;//跌倒后静止10s,有效检测到严重跌倒
				buzzer=1;
				#ifdef DEBUG_TEST
				printshow(DetectionStatus);
				#endif
				ADXL345_WR_Reg(XL345_THRESH_ACT,STRIKE_THRESHOLD);
				ADXL345_WR_Reg(XL345_THRESH_INACT,NOMOVEMENT_THRESHOLD);
				ADXL345_WR_Reg(XL345_TIME_INACT,STABLE_TIME);
				ADXL345_WR_Reg(XL345_ACT_INACT_CTL,0x7F);
				DetectionStatus=0xF0;
				#ifdef DEBUG_TEST
				printshow(DetectionStatus);
				#endif
			}
		}
		
		else if((test & XL345_FREEFALL)==XL345_FREEFALL)//失重中断
		{
//			printf("1-%d\n",ADXL345_RD_Reg(XL345_INT_SOURCE));
			if(DetectionStatus == 0xF0)//等待失重，且失重被检测到
			{
				DetectionStatus=0xF1;
				buzzer=0;
				#ifdef DEBUG_TEST
				printshow(DetectionStatus);
				#endif
				ADXL345_WR_Reg(XL345_THRESH_ACT,STRIKE_THRESHOLD);
				ADXL345_WR_Reg(XL345_THRESH_INACT,NOMOVEMENT_THRESHOLD);
				ADXL345_WR_Reg(XL345_TIME_INACT,STABLE_TIME);
				ADXL345_WR_Reg(XL345_ACT_INACT_CTL,0x7F);
				TIM_Cmd(TIM2,ENABLE); //定时器IRQ使能
				TimerWaitForStrike=0;
				TimerFreeFall=0;
			}
			else if(DetectionStatus == 0xF1)//失重后等待撞击，且一个新的FF被检测到
			{	
				//若检测到FF中断产生的间隔小于100ms则认为人体处于连续的跌落状态
				if(TimerWaitForStrike < FREE_FALL_INTERVAL)
					TimerFreeFall += TimerWaitForStrike;//连续FF持续时间
				else	TimerFreeFall=0;	//不是连续FF
			
				TimerWaitForStrike=0;	//判断结束清零
			
				//若检测到连续FF持续时间发生300ms则认为人从高处跌落
				if(TimerFreeFall >= FREE_FALL_OVERTIME)
				{
					DetectionStatus = 0xFF;
					buzzer=1;
					#ifdef DEBUG_TEST
					printshow(DetectionStatus);
					#endif
					ADXL345_WR_Reg(XL345_THRESH_ACT,STRIKE_THRESHOLD);
					ADXL345_WR_Reg(XL345_THRESH_INACT,NOMOVEMENT_THRESHOLD);
					ADXL345_WR_Reg(XL345_TIME_INACT,STABLE_TIME);
					ADXL345_WR_Reg(XL345_ACT_INACT_CTL,0x7F);
				
					DetectionStatus = 0xF0;
					#ifdef DEBUG_TEST
					printshow(DetectionStatus);
					#endif
				}
			}
			else	TimerFreeFall=0;	
		}


		if((test & XL345_DATAREADY) == XL345_DATAREADY) //数据更新事件
		{
			ADXL345_RD_XYZ(buf,buf+1,buf+2);//获取X/Y/Z值
			for(i=0;i<3;i++)
				new_sample[i] = buf[i];
			sampling_cnt++;//采样次数
			status_flag=1;

		}	
		
		NVIC_EnableIRQ(EXTI1_IRQn);
}

u8 rt_status=RT_STATIC;
void get_status()
{
	if(status_flag)
	{
		status_flag=0;
		//------------------数字滤波 平滑信号--------------------	
		for(i=0;i<3;i++)
		{
			rebuf3[i]=rebuf2[i];
			rebuf2[i]=rebuf1[i];
			rebuf1[i]=rebuf0[i];
			rebuf0[i]=buf[i];
			filter_out[i]=(rebuf0[i]+rebuf1[i]+rebuf2[i]+rebuf3[i])/4;			
			if(filter_out[i]>filter_max[i])
				filter_max[i]=filter_out[i];
			if(filter_out[i]<filter_min[i])
				filter_min[i]=filter_out[i];
		}
		//------------------计算出峰峰值和门限值--------------------	
		if(sampling_cnt==50 || (sampling_cnt>50 && sampling_cnt <55)) //在这范围内均可作为采样范围，提防中断打断，数值不能清零
		{
			sampling_cnt=0;
			for(i=0;i<3;i++)
			{
				Vpp[i]=filter_max[i]-filter_min[i];
				Vdc[i]=(filter_max[i]+filter_min[i])/2;
				error_flag[i]=0;
				if(Vpp[i] >= 250)
					Dynamic_precision[i] = Vpp[i]/50;		
				else if( Vpp[i] >= 100 && Vpp[i]<250 )
					Dynamic_precision[i] = 3;
				else 
				{
					Dynamic_precision[i] = 2;
					error_flag[i] = 1;		
				}
				filter_max[i]=0,filter_min[i]=0xFFF;
			}		
		}
		//------------------线性移位 消除高频噪声--------------------	
		for(i=0;i<3;i++)
		{
			old_sample[i] = new_sample[i];
			if(filter_out[i] >= new_sample[i])	
			{					
				if((filter_out[i] - new_sample[i]) > Dynamic_precision[i])
					new_sample[i] = filter_out[i];		
			}						
			else if(filter_out[i] < new_sample[i])			
			{					
				if((new_sample[i] - filter_out[i]) > Dynamic_precision[i])
				new_sample[i] = filter_out[i];		
			}	
		}
		//--------------------判断运动----------------
		for(j=0;j<3;j++)
			if( (old_sample[j] > Vdc[j]) && (new_sample[j] < Vdc[j]) && (error_flag[j] == 0) )
			{
				if(Vpp[j] >= 500)
				{
				#ifdef DEBUG_TEST
					printf("运动\n");
				#endif	
					rt_status=RT_SPORT;
				}
				else
				{
					rt_status=RT_STATIC;
				}
			}
	}
}


