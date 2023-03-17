/*-----------------------------------------
*文件：	tx_main.c
*内容：	用于UWB标签端
------------------------------------------*/
#include "dw1000.h"
#include "dwm1000_timestamp.h"
#include "port.h"
#include "stdio.h"
//#include "delay.h"
#include "oled.h"
#include "filter.h"
//#include "location.h"
#include "rtc_time.h"
#include "mq.h"
#include "ds18b20.h"
#include "FallDetection.h"
#include "buzzer.h"
#include "max30102.h"
/*只在发送端即标签使用*/
#ifdef TX_NODE


/* Buffer to store received response message.
 * Its size is adjusted to longest frame that this example code is supposed to handle. */
//最大接收数据包长度 20

#define RX_BUF_LEN 20
static uint8 rx_buffer[RX_BUF_LEN];
	
/* Time-stamps of frames transmission/reception, expressed in device time units.
* As they are 40-bit wide, we need to define a 64-bit int type to handle them. */
static uint64 poll_tx_ts;
static uint64 resp_rx_ts;
static uint64 final_tx_ts;


/* Frame sequence number, incremented after each transmission. */
static uint8 frame_seq_nb = 0;

/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
static uint32 status_reg = 0;

extern srd_msg_dsss msg_f_send;//声明外部变量 srd_msg_dsss结构体
srd_msg_dsss *msg_f_recv;//作为中间体存储接收帧数据

int seng_poll_flag=0;//接收数据标志位
int Tag_receive_conflict=0;//接收数据冲突标志位

/*
定义距离数组，用来存放各基站的距离
[0]---------0001
[1]---------0002
[n]---------000n+1
*/
static int distance_group[MAX_ANTHOR];
char dist_str[4][20];
char tag_vector[20];




//static void Send_Dis_To_Anthor0(void);//发送距离到总基站0
//anthor range
#define SEPC_ADDRESS 0x0000   //特殊地址，可选择是否发送距离到总基站
#define DEST_BEGIN_ADDR 0x0001 //起始基站地址
#define DEST_END_ADDR   DEST_BEGIN_ADDR + MAX_ANTHOR - 1 //anthro address 0x001 0x002 0x003 for 2D ,0x0001 0x0002 0x0003 0x0004 for 3D
uint16 Dest_Address =  DEST_BEGIN_ADDR; //开始地址

struct vec3d *tag_vec;   	//标签坐标



/*--------------------------------
*函数：	tx_rx_main
*功能：	用于与基站通信，包括接收RESPONE包，发送FINAL包
*参数：	无
*返回值：无
---------------------------------*/
void tx_rx_main()
{
		u32 frame_len,final_tx_time;
		u8 ret;

		memset(rx_buffer,0,20);


		//status_reg=dwt_read32bitreg(SYS_STATUS_ID);//读取状态位

        /* We assume that the transmission is achieved correctly, poll for reception of a frame or error/timeout. See NOTE 9 below. */
        while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
        { };

		dwt_enableframefilter(DWT_FF_NOTYPE_EN);//关闭帧过滤

		//frame_seq_nb++;
		if(status_reg & SYS_STATUS_RXFCG)//接收到数据帧，且通过了帧过滤
		{
			/* Clear good RX frame event in the DW1000 status register. */
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG );

			/* A frame has been received, read it into the local buffer. */
            frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;

			if(frame_len<127 || frame_len==127)
			{
				dwt_readrxdata(rx_buffer,frame_len,0);//读取帧数据
				msg_f_recv=(srd_msg_dsss *)rx_buffer;
				msg_f_send.destAddr[0]=msg_f_recv->sourceAddr[0];
				msg_f_send.destAddr[1]=msg_f_recv->sourceAddr[1];//保存目标基地址,接收帧的源地址为发送的基地址
				msg_f_send.seqNum=msg_f_recv->seqNum;

				if(msg_f_recv->messageData[0]==RESPONE_MSG)//判定是否为respone数据
				{
					/*
					for(i=0;i<12;i++)
                        sendChar(rx_buffer[i]);
                    printf("\n");
					*/
					/*接收上一次测距的距离*/
					distance_group[(msg_f_recv->sourceAddr[1]<<8) | msg_f_recv->sourceAddr[0]-1]=\
						msg_f_recv->messageData[1]*100 + msg_f_recv->messageData[2];//单位为cm	
					
					distance_group[(msg_f_recv->sourceAddr[1]<<8) | msg_f_recv->sourceAddr[0]-1]=\
						filter(distance_group[(msg_f_recv->sourceAddr[1]<<8) | msg_f_recv->sourceAddr[0]-1],\
								(msg_f_recv->sourceAddr[1]<<8) | msg_f_recv->sourceAddr[0]-1);	
								
					resp_rx_ts=get_rx_timestamp_u64();//获得接收respone数据帧的时间戳

					/* Compute final message transmission time. See NOTE 10 below. */
					//resp_rx_ts=dwt_readsystimestamphi32();
					//final_tx_time = (resp_rx_ts + (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
					final_tx_time =   dwt_readsystimestamphi32()  + 0x17cdc00/80;//10ms/8
					dwt_setdelayedtrxtime(final_tx_time);	
					/* Final TX timestamp is the transmission time we programmed plus the TX antenna delay. */
					final_tx_ts = (((uint64)(final_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

					/*保存final数据帧的信息*/
					msg_f_send.messageData[0]=FINAL_MSG;
					/*
					msg_f_send.messageData[1]=poll_tx_ts;
					msg_f_send.messageData[5]=resp_rx_ts;
					msg_f_send.messageData[9]=final_tx_ts;
					*/
					/* Write all timestamps in the final message. See NOTE 11 below. *///1 5 9
					final_msg_set_ts(&msg_f_send.messageData[FINAL_MSG_POLL_TX_TS_IDX], poll_tx_ts);
					final_msg_set_ts(&msg_f_send.messageData[FINAL_MSG_RESP_RX_TS_IDX], resp_rx_ts);
					final_msg_set_ts(&msg_f_send.messageData[FINAL_MSG_FINAL_TX_TS_IDX], final_tx_ts);

					dwt_writetxdata(24, (u8 *)&msg_f_send, 0); /* Zero offset in TX buffer. 前序码 9byte 数据 13byte*/
					dwt_writetxfctrl(24, 0, 0); /* Zero offset in TX buffer, ranging. */
					ret=dwt_starttx(DWT_START_TX_DELAYED);		//延迟发送，发送时间到达才会执行
					/* If dwt_starttx() returns an error, abandon this ranging exchange and proceed to the next one. See NOTE 12 below. */
					if (ret == DWT_SUCCESS)
					{
						/* Poll DW1000 until TX frame sent event set. See NOTE 9 below. */
						while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
						{ };
						/* Clear TXFRS event. */
						dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
					}
				}
			}		
		}

		else		//没有接收数据或者数据没有通过帧过滤被抛弃
		{
        	if(!(status_reg & SYS_STATUS_RXRFTO))//没有出现timeout超时，可能为数据冲突错误
            	Tag_receive_conflict = 1;//接收数据冲突标志

            /* Clear RX error/timeout events in the DW1000 status register. */
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG |SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);

            /* Reset RX to properly reinitialise LDE operation. */
            dwt_rxreset();			
		}		
}


extern u8 back_flag;//声明外部变量usart中断标志位
/*------------------------------------------
*函数：		tx_main
*功能：		系统轮询检测，用于标签
*参数：		无
*返回值：	无
-------------------------------------------*/
void tx_main()
{
	INT_callback(tx_rx_main);//注册中断回调函数
	while (1)
	{
		/* code */
		if( (!ALARM_A_CON) && (back_flag))	//使用用户设置闹钟时，参数无效
		{
			back_flag=0;
			rtc_set_alarm_a(0,0,0,0);
		#ifdef DEBUG_TEST
			show_temp();
			show_ppm();
			showtime();
		#endif
		}

		/*-----检测心率和血氧*/
		 max30102_test();/*开始第一次读取，等待5s时间*/
		 max30102_continue();
		 /*-----end*/

		/*----检测是否有跌倒*/
		buzzer_check();
		/*----end*/
		
		/*-----检测状态*/
		get_status();
		/*------end*/
		/*----显示数据*/
	#ifdef DEBUG_TEST
		// show_ppm();
		// show_temp();
		// showtime();
		// show_hr_spo2();
	#endif
		/*----end*/
		
		// OLED_Refresh_Gram();
	}
	
}



/*------------------------------
*函数：		tag_send_poll
*功能：		标签发送POLL请求
*参数：		无
*返回值：	无
------------------------------*/
void tag_send_poll()
{
	//int result;
	//tag_vec->x=0;tag_vec->y=0;tag_vec->z=0;
	msg_f_send.destAddr[0]=Dest_Address & 0xff;
	msg_f_send.destAddr[1]=(Dest_Address>>8) & 0xff;
		

	
	msg_f_send.seqNum=frame_seq_nb;
	msg_f_send.messageData[0]=POLL_MSG;
	
#if 0
	 /* Set expected response's delay and timeout. See NOTE 4 and 5 below.
     * As this example only handles one incoming frame with always the same delay and timeout, those values can be set here once for all. */
    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);//设置发送完成后的接收延迟时间
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);						//设置接收检测超时时间
#endif

	dwt_writetxdata(12, (u8 *)&msg_f_send, 0); /* Zero offset in TX buffer. */
    dwt_writetxfctrl(12, 0, 0); /* Zero offset in TX buffer, ranging. */
	
	  /* Start transmission, indicating that a response is expected so that reception is enabled automatically after the frame is sent and the delay
       * set by dwt_setrxaftertxdelay() has elapsed. */
	dwt_starttx(DWT_START_TX_IMMEDIATE);
     /* We assume that the transmission is achieved correctly, poll for reception of a frame or error/timeout. See NOTE 8 below. */
    while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))//等待发送完成
    { };
	dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);//清除发送完成标志位
	if(++frame_seq_nb==255)
		frame_seq_nb=0;//最大接收帧数 255
    poll_tx_ts = get_tx_timestamp_u64();//得到发送时间戳
	dwt_enableframefilter(DWT_FF_DATA_EN);	//打开帧过滤
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS*10);//设置接收超时时间
    dwt_rxenable(0);//立即使能接收器
	
	
}
/*------------------------------------------------------
*函数：		senddis_to_anthor0
*功能：		与全部基站通信完毕后，标签发送距离信息包给基站0x0004
*参数：		无
*返回值：	无
-----------------------------------------------------*/
void senddis_to_anthor0()
{

	// sprintf(dist_str[0],"%.2f",(float)distance_group[0]/100);
	// printf("dis1:%s\n",dist_str[0]);
	// sprintf(dist_str[1],"%.2f",(float)distance_group[1]/100);
	// printf("dis2:%s\n",dist_str[1]);
	// sprintf(dist_str[2],"%.2f",(float)distance_group[2]/100);
	// printf("dis3:%s\n",dist_str[2]);
	// sprintf(dist_str[3],"%.2f",(float)distance_group[3]/100);
	// printf("dis4:%s\n",dist_str[3]);
    static int framenum = 0 ;
    
    msg_f_send.destAddr[0] =(0x0004) &0xFF;
    msg_f_send.destAddr[1] =  ((0x0001)>>8) &0xFF;

    msg_f_send.seqNum = frame_seq_nb;
    msg_f_send.messageData[0]=DISTANCE_MSG;
    {
        uint8 len = 0;
        uint8 LOCATION_INFO_START_IDX = 1;
			
					/*OLED显示用于测试距离*/
		
//		sprintf(dist_str[0],"dist1:%3.2fm",(float)distance_group[0]/100);
//		sprintf(dist_str[1],"dist2:%3.2fm",(float)distance_group[1]/100);
//		sprintf(dist_str[2],"dist3:%3.2fm",(float)distance_group[2]/100);
//		sprintf(dist_str[3],"dist4:%3.2fm",(float)distance_group[3]/100);
//		OLED_ShowString(5,0,dist_str[0],12);
//		OLED_ShowString(5,14,dist_str[1],12);
//		OLED_ShowString(5,28,dist_str[2],12);
//		OLED_ShowString(5,42,dist_str[3],12);
//		OLED_Refresh_Gram();

        msg_f_send.messageData[LOCATION_INFO_START_IDX + (len++)] = 'm';
        msg_f_send.messageData[LOCATION_INFO_START_IDX + (len++)] = 'r';
        msg_f_send.messageData[LOCATION_INFO_START_IDX + (len++)] = 0x02;
        msg_f_send.messageData[LOCATION_INFO_START_IDX + (len++)] = SHORT_ADDR;//TAG ID
        msg_f_send.messageData[LOCATION_INFO_START_IDX + (len++)] = (uint8)(framenum&0xFF);
        msg_f_send.messageData[LOCATION_INFO_START_IDX + (len++)] = (uint8)((framenum>>8)&0xFF);

        msg_f_send.messageData[LOCATION_INFO_START_IDX + (len++)] = (uint8)((distance_group[0]>0?distance_group[0]:0xFFFF)&0xFF);
        msg_f_send.messageData[LOCATION_INFO_START_IDX + (len++)] = (uint8)(((distance_group[0]>0?distance_group[0]:0xFFFF) >>8)&0xFF);

        msg_f_send.messageData[LOCATION_INFO_START_IDX + (len++)] = (uint8)((distance_group[1]>0?distance_group[1]:0xFFFF)&0xFF);
        msg_f_send.messageData[LOCATION_INFO_START_IDX + (len++)] = (uint8)(((distance_group[1]>0?distance_group[1]:0xFFFF) >>8)&0xFF);

        msg_f_send.messageData[LOCATION_INFO_START_IDX + (len++)] = (uint8)((distance_group[2]>0?distance_group[2]:0xFFFF)&0xFF);
        msg_f_send.messageData[LOCATION_INFO_START_IDX + (len++)] = (uint8)(((distance_group[2]>0?distance_group[2]:0xFFFF) >>8)&0xFF);

        if(MAX_ANTHOR > 3)
        {
            msg_f_send.messageData[LOCATION_INFO_START_IDX + (len++)] = (uint8)((distance_group[MAX_ANTHOR-1]>0?distance_group[MAX_ANTHOR-1]:0xFFFF)&0xFF);
            msg_f_send.messageData[LOCATION_INFO_START_IDX + (len++)] = (uint8)(((distance_group[MAX_ANTHOR-1]>0?distance_group[MAX_ANTHOR-1]:0xFFFF) >>8)&0xFF);
        }
        else
        {
            msg_f_send.messageData[LOCATION_INFO_START_IDX + (len++)] = (uint8)((distance_group[0]>0?distance_group[0]:0xFFFF)&0xFF);
            msg_f_send.messageData[LOCATION_INFO_START_IDX + (len++)] = (uint8)(((distance_group[0]>0?distance_group[0]:0xFFFF) >>8)&0xFF);
        }

        msg_f_send.messageData[LOCATION_INFO_START_IDX + (len++)] = '\n';
        msg_f_send.messageData[LOCATION_INFO_START_IDX + (len++)] = '\r';

    }
	memset(distance_group,0,sizeof(distance_group));
    dwt_writetxdata(11 + 17,(uint8 *)&msg_f_send, 0) ;  // write the frame data
    dwt_writetxfctrl(11 + 17, 0,0);
    dwt_starttx(DWT_START_TX_IMMEDIATE);
    while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
    { };
	dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);

    framenum++;
}
/*-----------------------------------------------------
*函数：		TIM3_init
*功能：		定时器初始化,peri/10 ms进入一次中断，DW1000刷新频率
*参数：		无
*返回值：	无
------------------------------------------------------*/
#define peri 1650  //刷新率6HZ

void TIM3_init()
{
	TIM_TimeBaseInitTypeDef TIM_Timebasestructure;
	NVIC_InitTypeDef	NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	
	TIM_Timebasestructure.TIM_Period=peri-1;
	TIM_Timebasestructure.TIM_Prescaler=4200-1;//42mhz/4200=10KHZ  0.1ms/次数
	TIM_Timebasestructure.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_Timebasestructure.TIM_ClockDivision=TIM_CKD_DIV1;
	
	TIM_TimeBaseInit(TIM3,&TIM_Timebasestructure);
	
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE); 

	TIM_Cmd(TIM3,ENABLE); 
	//TIM_Cmd(TIM3,DISABLE); 
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM3_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=7; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0; 
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}	


/*-----------------------------------------------
*函数：		TIM3_IRQHandler
*功能：		TIM3定时器中断周期性发送POLL请求或发送距离包
*参数：		无
*返回值：	无
-----------------------------------------------*/
extern int timers;
void TIM3_IRQHandler()
{
	if(TIM_GetITStatus(TIM3,TIM_IT_Update)==SET)
	{
		TIM_ClearITPendingBit(TIM3,TIM_IT_Update);
		TIM_Cmd(TIM3,DISABLE);//失能TIM，防止重新进入中断
		if(timers<30)
			timers++;
		if(!Tag_receive_conflict)//没有出现数据冲突，正常发送
		{
			dwt_forcetrxoff();//关闭收发器
			if(Dest_Address==DEST_END_ADDR+1)//当发送完一个循环，即发送距离包给特殊基站
			{
				senddis_to_anthor0();
				Dest_Address=DEST_BEGIN_ADDR;
			}
			else
			{
				tag_send_poll();//开始发送poll
				Dest_Address++;
			}

			TIM3->ARR=peri-1;
		}
		else				//出现数据冲突,可以调整周期发送间隔
		{
			TIM3->ARR=50*(SHORT_ADDR%10+1);
			Tag_receive_conflict=0;
		}

		TIM_Cmd(TIM3,ENABLE);
	}
}



#endif
