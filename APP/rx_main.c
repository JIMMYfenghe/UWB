#include "dw1000.h"
#include "dwm1000_timestamp.h"
#include "stdio.h"
#include "port.h"
#include "kalman.h"

/*ֻ�ڽ��ն˼���վʹ��*/
#ifdef RX_NODE


//��Ϊ��վ���壬������Ϣ��󳤶�Ϊ30

#define RX_BUFFER_LEN 30
static uint8 rx_buffer[RX_BUFFER_LEN];

/* Speed of light in air, in metres per second. */
#define SPEED_OF_LIGHT 299702547
	
/* Timestamps of frames transmission/reception.
* As they are 40-bit wide, we need to define a 64-bit int type to handle them. */

static uint64 poll_rx_ts;
static uint64 resp_tx_ts;
static uint64 final_rx_ts;



/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
static uint32 status_reg = 0;
	
extern srd_msg_dsss msg_f_send;//�����ⲿ���� srd_msg_dsss�ṹ��
srd_msg_dsss *msg_f_recv;//��Ϊ�м���洢����֡����

/* Hold copies of computed time of flight and distance here for reference so that it can be examined at a debug breakpoint. */
static double tof;
static double distance;
uint32_t distance_cm;
	
/* String used to display measured distance on UART. */
char dist_str[16] = {0};

/*�����ⲿ�������������뺯��*/
extern double dwt_getrangebias(uint8 chan, float range, uint8 prf);
/*�����ⲿ����*/
extern dwt_config_t config;


/*
*       �жϽ��պ�����ֻ�ڲ����ⲿ�ж�ʱ�����
*/
void rx_tx_main()
{
    u8 frame_len;
    uint32 poll_tx_ts,resp_rx_ts,final_tx_ts;
    u32 poll_rx_ts_32,resp_tx_ts_32,final_rx_ts_32;
    double Ra,Rb,Da,Db;
    int64 tof_dtu;


    memset(rx_buffer,0,30);

    /*�ر�֡����*/
    dwt_enableframefilter(DWT_FF_NOTYPE_EN);

	status_reg = dwt_read32bitreg(SYS_STATUS_ID);
    /*�������ݣ�ͨ��֡����*/
    if(status_reg & SYS_STATUS_RXFCG)
    {
		/* Clear good RX frame event in the DW1000 status register. */
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG );

		/* A frame has been received, read it into the local buffer. */
        frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;

		if(frame_len<127 || frame_len==127)
		{
			dwt_readrxdata(rx_buffer,frame_len,0);//��ȡ֡����

            msg_f_recv=(srd_msg_dsss *)rx_buffer;
            msg_f_send.destAddr[0]=msg_f_recv->sourceAddr[0];
            msg_f_send.destAddr[1]=msg_f_recv->sourceAddr[1];
            msg_f_send.seqNum=msg_f_recv->seqNum;

            switch (msg_f_recv->messageData[0])
            {
                /*�յ����Ա�ǩ��POLL��Ϣ*/
                case POLL_MSG:
                /*
                    for(i=0;i<10;i++)
                        sendChar(rx_buffer[i]);
                    printf("\n");
                */
                    poll_rx_ts=get_rx_timestamp_u64();//poll_rxʱ���
                    msg_f_send.messageData[0]=RESPONE_MSG;

                    /*��RESPONE���ݰ��а���һ�εĲ�õľ��뷢�͸���ǩ*/
                    
                    msg_f_send.messageData[1]=(u8)(distance_cm/100);
                    msg_f_send.messageData[2]=(u8)(distance_cm%100);
                        
                        /* Set send time for response. See NOTE 9 below. */
                        //resp_tx_time = (poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
                        //dwt_setdelayedtrxtime(resp_tx_time);    
                        

					//д������
					dwt_writetxdata(14, (uint8 *)&msg_f_send, 0) ; // write the frame data
					dwt_writetxfctrl(14, 0,0);
                    dwt_starttx(DWT_START_TX_IMMEDIATE); 

                    while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS)) //�ȴ��������
                    { };     
                    distance_cm=0;//�����������
                    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);//������ͱ�־  

                    break;

                /*�յ����Ա�ǩ��FINAL��Ϣ*/
                case FINAL_MSG:
                /*
					for(i=0;i<10;i++)
                        sendChar(rx_buffer[i]);
                    printf("\n");
                */
                    /*��÷���RESPONE�ͽ���FINAL��ʱ���*/
                    resp_tx_ts=get_tx_timestamp_u64();
                    final_rx_ts=get_rx_timestamp_u64();

                    /* Get timestamps embedded in the final message. */
                    final_msg_get_ts(&msg_f_recv->messageData[FINAL_MSG_POLL_TX_TS_IDX], &poll_tx_ts);
                    final_msg_get_ts(&msg_f_recv->messageData[FINAL_MSG_RESP_RX_TS_IDX], &resp_rx_ts);
                    final_msg_get_ts(&msg_f_recv->messageData[FINAL_MSG_FINAL_TX_TS_IDX], &final_tx_ts);     

                    /* Compute time of flight. 32-bit subtractions give correct answers even if clock has wrapped. See NOTE 12 below. */
                    poll_rx_ts_32 = (uint32)poll_rx_ts;
                    resp_tx_ts_32 = (uint32)resp_tx_ts;
                    final_rx_ts_32 = (uint32)final_rx_ts;
                    Ra = (double)(resp_rx_ts - poll_tx_ts);
                    Rb = (double)(final_rx_ts_32 - resp_tx_ts_32);
                    Da = (double)(final_tx_ts - resp_rx_ts);
                    Db = (double)(resp_tx_ts_32 - poll_rx_ts_32);
                    tof_dtu = (int64)((Ra * Rb - Da * Db) / (Ra + Rb + Da + Db));

                    tof = tof_dtu * DWT_TIME_UNITS;
                    distance = tof * SPEED_OF_LIGHT;

                    //��ȥ����ϵ��
                    distance=distance-dwt_getrangebias(config.chan,(float)distance,config.prf);
                    /*�������˲��㷨�������*/
					distance=KalMan(distance);
                    /*ת����λ���ڷ���*/
                    distance_cm=(uint32_t)(distance*100);

                    /* Display computed distance. */
                    sprintf(dist_str, "DIST: %3.2lf m", distance);   
                    printf("%s\n",dist_str);                     

                    break;

                /*ֻ�л�վ0���գ����;�����Ϣ����λ��*/
                case DISTANCE_MSG:
                    USART_puts(&msg_f_recv->messageData[1],16);
					break;

                default:
                    break;
            }
		}
        dwt_write32bitreg(SYS_STATUS_ID, (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR));//������ձ�־
        /*��֡����*/
        dwt_enableframefilter(DWT_FF_DATA_EN);

        /* Clear reception timeout to start next ranging process. */
        dwt_setrxtimeout(0);

        /* Activate reception immediately. */
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
    }
    /*û�н��յ����ݻ��߳��ִ���*/
    else
    {
        dwt_write32bitreg(SYS_STATUS_ID, (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR));//������ձ�־
        /*��֡����*/
        dwt_enableframefilter(DWT_FF_DATA_EN);

        /* Clear reception timeout to start next ranging process. */
        dwt_setrxtimeout(0);

        /* Activate reception immediately. */
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
    }
 
}

/*
*       ������������������֡����
*/
void rx_main()
{
	
	
	INT_callback(rx_tx_main);//ע���жϻص�����
    //KalMan_Init();
    /*��֡����*/
    dwt_enableframefilter(DWT_FF_DATA_EN);

    /* Clear reception timeout to start next ranging process. */
    dwt_setrxtimeout(0);

    /* Activate reception immediately. */
    dwt_rxenable(DWT_START_RX_IMMEDIATE);


	while (1)
	{
		
	
	}
	
}

#endif

