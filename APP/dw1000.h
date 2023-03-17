#ifndef __DW1000_H
#define __DW1000_H

#include "stm32f4xx.h"
#include "deca_device_api.h"


//通过以下两个宏定义决定模块角色 RX_NODE 基站 TX_NODE标签
//同时有且只有一个宏定义打开
//#define RX_NODE		//基站
#define TX_NODE //标签

//通过MAX_ANTHOR 定义系统中基站个数，2D定位3个基站，3D定位4个基站
//MAX_ANTHOR 3 --> 2D
//MAX_ANTHOR 4 --> 3D
#define MAX_ANTHOR 4

//基站节点地址0x0001 0x0002 0x0003 0x0004
//部署完毕基站0x0001 链接串口
#ifdef RX_NODE
 #define SHORT_ADDR 0x0004
 //#define LCD_ENABLE //没有液晶的时候，把这个宏定义注释掉
#endif  

//标签和基站地址不能重叠
//标签节点地址 0x0005 0x0006 0x0007
#ifdef TX_NODE
 #define SHORT_ADDR 0x0005
#endif 

/* Default antenna delay values for 64 MHz PRF. See NOTE 1 below. */
#define TX_ANT_DLY 0
#define RX_ANT_DLY 32890

/*
 * 2. The messages here are similar to those used in the DecaRanging ARM application (shipped with EVK1000 kit). They comply with the IEEE
 *    802.15.4 standard MAC data frame encoding and they are following the ISO/IEC:24730-62:2013 standard. The messages used are:
 *     - a poll message sent by the initiator to trigger the ranging exchange.
 *     - a response message sent by the responder allowing the initiator to go on with the process
 *     - a final message sent by the initiator to complete the exchange and provide all information needed by the responder to compute the
 *       time-of-flight (distance) estimate.
 *    The first 10 bytes of those frame are common and are composed of the following fields:
 *     - byte 0/1: frame control (0x8841 to indicate a data frame using 16-bit addressing).
 *     - byte 2: sequence number, incremented for each new frame.
 *     - byte 3/4: PAN ID (0xDECA).
 *     - byte 5/6: destination address, see NOTE 3 below.
 *     - byte 7/8: source address, see NOTE 3 below.
 *     - byte 9: function code (specific values to indicate which message it is in the ranging process).
 *    The remaining bytes are specific to each message as follows:
 *    Poll message:
 *     - no more data
 *    Response message:
 *     - byte 10: activity code (0x02 to tell the initiator to go on with the ranging exchange).
 *     - byte 11/12: activity parameter, not used here for activity code 0x02.
 *    Final message:
 *     - byte 10 -> 13: poll message transmission timestamp.
 *     - byte 14 -> 17: response message reception timestamp.
 *     - byte 18 -> 21: final message transmission timestamp.
 *    All messages end with a 2-byte checksum automatically set by DW1000.
 */
/* Indexes to access some of the fields in the frames defined above. */

#define FINAL_MSG_POLL_TX_TS_IDX 1
#define FINAL_MSG_RESP_RX_TS_IDX 5
#define FINAL_MSG_FINAL_TX_TS_IDX 9
#define FINAL_MSG_TS_LEN 4


/*定义信息类型的标志字节*/
#define POLL_MSG  0x21
#define RESPONE_MSG 0x10
#define FINAL_MSG   0x23
#define DISTANCE_MSG  0x30

#ifdef TX_NODE
	/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor.
	 * 1 uus = 512 / 499.2 ? and 1 ? = 499.2 * 128 dtu. */
	#define UUS_TO_DWT_TIME 65536

	/* Delay between frames, in UWB microseconds. See NOTE 4 below. */
	/* This is the delay from the end of the frame transmission to the enable of the receiver, as programmed for the DW1000's wait for response feature. */
	#define POLL_TX_TO_RESP_RX_DLY_UUS 150
	/* This is the delay from Frame RX timestamp to TX reply timestamp used for calculating/setting the DW1000's delayed TX function. This includes the
	 * frame length of approximately 2.66 ms with above configuration. */
	#define RESP_RX_TO_FINAL_TX_DLY_UUS 3100
	/* Receive response timeout. See NOTE 5 below. */
	#define RESP_RX_TIMEOUT_UUS 5700
#endif

#ifdef RX_NODE
	/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor.
	 * 1 uus = 512 / 499.2 ? and 1 ? = 499.2 * 128 dtu. */
	#define UUS_TO_DWT_TIME 65536
	
	/* Delay between frames, in UWB microseconds. See NOTE 4 below. */
	/* This is the delay from Frame RX timestamp to TX reply timestamp used for calculating/setting the DW1000's delayed TX function. This includes the
	 * frame length of approximately 2.46 ms with above configuration. */
	#define POLL_RX_TO_RESP_TX_DLY_UUS 2750
	/* This is the delay from the end of the frame transmission to the enable of the receiver, as programmed for the DW1000's wait for response feature. */
	#define RESP_TX_TO_FINAL_RX_DLY_UUS 500
	/* Receive final timeout. See NOTE 5 below. */
	#define FINAL_RX_TIMEOUT_UUS 3300
#endif

/* The preamble timeout allows the receiver to stop listening in situations where preamble is not starting (which might be because the responder is
 *    out of range or did not receive the message to respond to). This saves the power waste of listening for a message that is not coming. We
 *    recommend a minimum preamble timeout of 5 PACs for short range applications and a larger value (e.g. in the range of 50% to 80% of the preamble
 *    length) for more challenging longer range, NLOS or noisy environments.*/
#define PRE_TIMEOUT 8

#if 0
dwt_config_t config = {
    2,               /* Channel number. */
    DWT_PRF_64M,     /* Pulse repetition frequency. */
    DWT_PLEN_1024,   /* Preamble length. Used in TX only. */
    DWT_PAC32,       /* Preamble acquisition chunk size. Used in RX only. */
    9,               /* TX preamble code. Used in TX only. */
    9,               /* RX preamble code. Used in RX only. */
    1,               /* 0 to use standard SFD, 1 to use non-standard SFD. */
    DWT_BR_110K,     /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    (1025 + 64 - 32) /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};
#endif
typedef struct
{
    uint8 frameCtrl[2];                             //  frame control bytes 00-01
    uint8 seqNum;                                   //  sequence_number 02
    uint8 panID[2];                                 //  PAN ID 03-04
    uint8 destAddr[2];               //  05-06
    uint8 sourceAddr[2];             //  07-08
    uint8 messageData[100] ; //  09-124 (application data and any user payload)
    uint8 fcs[2] ;                                  //  125-126  we allow space for the CRC as it is logically part of the message. However ScenSor TX calculates and adds these bytes.
}srd_msg_dsss ;


int dw1000_init(void);
void dw1000_msg_init(void);
void dwt_SetTxPower(void);
void dwt_enable_pa(void);
void dw1000_config(void);


void rx_main(void);
void tx_main(void);
void tag_send_poll(void);
void TIM3_init(void);
void senddis_to_anthor0(void);

#endif
