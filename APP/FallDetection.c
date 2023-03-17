/*---------------------------------------------------------------
*file:		falldetection.c
*content:	������
---------------------------------------------------------------*/
#include "FallDetection.h"
#include "oled.h"
//#include "usart.h"
extern u8 buzzer;
unsigned char DetectionStatus; 
unsigned char TimerWaitForStable; // ײ����ȴ��ȶ���ʱ�������
unsigned char TimerWaitForStrike; // ʧ�غ�ȴ�ײ����ʱ�������
unsigned char TimerFreeFall; // ����FF��ʱ������� 
short InitialStatus[3]; //  X-, Y-, Z- ��ĳ�ʼ״̬���ٶ� 
short Acceleration[3]; // X-, Y-, Z- ��ĵ�ǰ���ٶ�
short Acceleration_recv[3];
unsigned long int DeltaAcceleration[3]; // ��ǰ���ٶ����ʼ��״̬���ٶȵĲ�ֵ
unsigned long int DeltaVectorSum; // ���ٶȲ�ֵ��������



static int i,j,status_flag;
static int sampling_cnt=0;
short buf[3],rebuf0[4],rebuf1[4],rebuf2[4],rebuf3[4],regulation[4];
short filter_out[3],filter_max[3]={0,0,0},filter_min[3]={0xFFF,0xFFF,0xFFF};
short Vpp[3],Vdc[3],old_sample[3],new_sample[3],Dynamic_precision[3];//���ֵ������ֵ���¾ɲ���ֵ����̬����
u8 error_flag[3];




/*-----------------------------------------
*������		ADXL345_Init
*���ܣ�		ADXL345���ó�ʼ��
*����ֵ��	
			0	��	�ɹ�
			1	��	ʧ��
------------------------------------------*/
u8 ADXL345_Init(void)
{
	
	if(ADXL345_RD_Reg(XL345_DEVID) == XL345_ID) //��ȡ����ID
    { 
			ADXL345_WR_Reg(XL345_OFSX,0xFF);
			ADXL345_WR_Reg(XL345_OFSY,0x05);
			ADXL345_WR_Reg(XL345_OFSZ,0xFF);
			ADXL345_WR_Reg(XL345_THRESH_ACT,STRIKE_THRESHOLD);
			ADXL345_WR_Reg(XL345_THRESH_INACT,NOMOVEMENT_THRESHOLD);
			ADXL345_WR_Reg(XL345_TIME_INACT,STABLE_TIME);
			ADXL345_WR_Reg(XL345_ACT_INACT_CTL,0x7F);	//ACT--DC��ϣ�INACT--AC��ϣ�XYZ���ʹ��
			
			ADXL345_WR_Reg(XL345_THRESH_FF,FREE_FALL_THRESHOLD);
			ADXL345_WR_Reg(XL345_TIME_FF,FREE_FALL_TIME);
					
			ADXL345_WR_Reg(XL345_POWER_CTL,XL345_STANDBY);//����ģʽ
			ADXL345_WR_Reg(XL345_BW_RATE,XL345_RATE_50);//�����������100Hz
			ADXL345_WR_Reg(XL345_INT_MAP,0x00);	//�����ж϶����õ�INT1
			//ADXL345_WR_Reg(XL345_FIFO_CTL,0x1f);	//������FIFO����·ģʽ
			ADXL345_WR_Reg(XL345_INT_ENABLE,0x1C);//ʹ��ACT INACT FF�ж�
			ADXL345_WR_Reg(XL345_DATA_FORMAT,0x2B);		//ȫ�ֱ��ʣ��Ҷ��룬+-16g
			ADXL345_WR_Reg(XL345_POWER_CTL,XL345_MEASURE);
			ADXL345_RD_Reg(XL345_INT_SOURCE);

			DetectionStatus=0xF0; 		//��ʼ	
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
*������		printshow
*���ܣ�		��ӡ״̬
*���룺		status:	״ֵ̬
*�����		��
*����ֵ��	��
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
*������		EXTI1_IRQHandler
*���ܣ�		ADXL345�жϴ��������˴����ڼ��������㷨
*����ֵ��	��
--------------------------------------*/
u8 test=0;
void EXTI1_IRQHandler(void) 
{ 
	u8 i;
	EXTI_ClearITPendingBit(EXTI_Line1);
	NVIC_DisableIRQ(EXTI1_IRQn);	
	test=ADXL345_RD_Reg(XL345_INT_SOURCE);
		if((test & XL345_ACTIVITY)==XL345_ACTIVITY)//ײ���ж�
		{			
			
			if(DetectionStatus == 0xF1)//�ȴ�ײ������ײ������⵽
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
				TIM_Cmd(TIM2,ENABLE); //ʹ�ܶ�ʱ��
				TimerWaitForStable=0;
			}
			else if(DetectionStatus == 0xF4)//ײ����ʱ�侲ֹ����
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
		
		else if((test & XL345_INACTIVITY)==XL345_INACTIVITY)//��ֹ�ж�
		{
			
			if(DetectionStatus == 0xF2)//�ȴ���ֹ���Ҿ�ֹ����⵽
			{
				DetectionStatus=0xF3;
				buzzer=0;
				#ifdef DEBUG_TEST
				printshow(DetectionStatus);
				#endif
				TIM_Cmd(TIM2,DISABLE); //disable��ʱ��
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
				
				//���ʼ״̬�Ƚϼ�⣬��ʸ�����0.7g����Ϊ����Ч����
				if(DeltaVectorSum > DELTA_VECTOR_SUM_THRESHOLD)
				{
					DetectionStatus=0xF4;//��Ч�������
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
			
			else if(DetectionStatus == 0xF4)//��ʱ���ȶ���ֹ
			{
				DetectionStatus=0xF5;//������ֹ10s,��Ч��⵽���ص���
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
		
		else if((test & XL345_FREEFALL)==XL345_FREEFALL)//ʧ���ж�
		{
//			printf("1-%d\n",ADXL345_RD_Reg(XL345_INT_SOURCE));
			if(DetectionStatus == 0xF0)//�ȴ�ʧ�أ���ʧ�ر���⵽
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
				TIM_Cmd(TIM2,ENABLE); //��ʱ��IRQʹ��
				TimerWaitForStrike=0;
				TimerFreeFall=0;
			}
			else if(DetectionStatus == 0xF1)//ʧ�غ�ȴ�ײ������һ���µ�FF����⵽
			{	
				//����⵽FF�жϲ����ļ��С��100ms����Ϊ���崦�������ĵ���״̬
				if(TimerWaitForStrike < FREE_FALL_INTERVAL)
					TimerFreeFall += TimerWaitForStrike;//����FF����ʱ��
				else	TimerFreeFall=0;	//��������FF
			
				TimerWaitForStrike=0;	//�жϽ�������
			
				//����⵽����FF����ʱ�䷢��300ms����Ϊ�˴Ӹߴ�����
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


		if((test & XL345_DATAREADY) == XL345_DATAREADY) //���ݸ����¼�
		{
			ADXL345_RD_XYZ(buf,buf+1,buf+2);//��ȡX/Y/Zֵ
			for(i=0;i<3;i++)
				new_sample[i] = buf[i];
			sampling_cnt++;//��������
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
		//------------------�����˲� ƽ���ź�--------------------	
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
		//------------------��������ֵ������ֵ--------------------	
		if(sampling_cnt==50 || (sampling_cnt>50 && sampling_cnt <55)) //���ⷶΧ�ھ�����Ϊ������Χ������жϴ�ϣ���ֵ��������
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
		//------------------������λ ������Ƶ����--------------------	
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
		//--------------------�ж��˶�----------------
		for(j=0;j<3;j++)
			if( (old_sample[j] > Vdc[j]) && (new_sample[j] < Vdc[j]) && (error_flag[j] == 0) )
			{
				if(Vpp[j] >= 500)
				{
				#ifdef DEBUG_TEST
					printf("�˶�\n");
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


