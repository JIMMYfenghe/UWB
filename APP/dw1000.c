/*-------------------------------------------
*file:  dw1000.c
*content:   config dw1000
----------------------------------------------*/
#include <string.h>

#include "deca_device_api.h"
#include "deca_regs.h"
#include "stdio.h"
#include "deca_spi.h"
#include "port.h"
#include "dw1000.h"


//extern dwt_config_t config;
/*------------------------------------------
*config：   用于配置DW1000配置的结构体
------------------------------------------*/
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





srd_msg_dsss msg_f_send;//信息数据包

/*-------------------------------
*函数：     dw1000_init
*功能： 	dw1000 基础初始化
*参数：     无
*返回值：   无
--------------------------------*/
int dw1000_init()
{
	set_spi_rate_low();
	if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR)
    {
		return -1;
    }
	set_spi_rate_high();
    return 0;
}

/*----------------------------------
*函数：     dw1000_msg_init
*功能： 	DW1000信息填充
*参数：     无
*返回值：   无
-----------------------------------*/
void dw1000_msg_init()
{

    msg_f_send.frameCtrl[0] = 0x1 /*frame type 0x1 == data*/ | 0x40 /*PID comp*/|0x20/* ACK request*/;
    msg_f_send.frameCtrl[1] = 0x8 /*dest extended address (16bits)*/ | 0x80 /*src extended address (16bits)*/;
    msg_f_send.panID[0] = 0xF0;
    msg_f_send.panID[1] = 0xF0;

    msg_f_send.seqNum = 0; //copy sequence number and then increment
    msg_f_send.sourceAddr[0] = SHORT_ADDR & 0xFF; //copy the address
    msg_f_send.sourceAddr[1] =(SHORT_ADDR>>8)& 0xFF; //copy the address

    msg_f_send.destAddr[0] = 0x01;  //set the destination address (broadcast == 0xffff)
    msg_f_send.destAddr[1] = 0x01;  //set the destination address (broadcast == 0xffff)
}
/*--------------------------------
*函数：     dw1000_config
*功能： 	配置DW1000工作模式
*参数：     无
*返回值：   无
---------------------------------*/
void dw1000_config()
{
	dwt_configure(&config);//配置工作模式
    dwt_setleds(1);//控制GPIO2 3，分别连接TXLED,RXLED   
    dwt_SetTxPower();//设置发射功率
    dwt_setpanid(0xF0F0);//设置pan id
    dwt_setaddress16(SHORT_ADDR);//设置自身的短地址
	/* Apply default antenna delay value. See NOTE 1 below. */
    dwt_setrxantennadelay(RX_ANT_DLY);//设置收发天线延迟
    dwt_settxantennadelay(TX_ANT_DLY);
    dwt_enable_pa();//外部功率放大
	
#ifdef INT_MODE_ENABLE
    /* Enable wanted interrupts (TX confirmation, RX good frames, RX timeouts and RX errors). */
    //dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | DWT_INT_RFTO | DWT_INT_RXPTO | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFSL | DWT_INT_SFDT, 1);
    dwt_setinterrupt(DWT_INT_RFCG | (DWT_INT_ARFE | DWT_INT_RFSL | DWT_INT_SFDT | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFTO /*| DWT_INT_RXPTO*/), 1);
#endif

}
/*=====================================================================================================*
**函数 : dwt_SetTxPower
**功能 : 设置dw1000的发射功率
**输入 : 无
**输出 : 无
**使用 : dwt_SetTxPower();
**作者 : BPhero
**日期 : 2019/07/02
**=====================================================================================================*/
//当使用6.8Mbps默认开启智能发送功率控制
void dwt_SetTxPower()
{
    u32 reg;
	dwt_txconfig_t *configTX;
    configTX->PGdly =  0xc2;
    //Configure TX power
    //configTX->power = 0x67676767;
	configTX->power = 0x1F1F1F1F;
    //configure the tx spectrum parameters (power and PG delay)
    reg=dwt_read32bitreg(SYS_CFG_ID);
    reg=reg | 0x00040000;//关闭智能功率,开启手动调整发射功率
    dwt_write32bitreg(SYS_CFG_ID,reg);
    dwt_configuretxrf(configTX);//设置发送频谱

}

/*-------------------------------------------
*函数：     dwt_enable_pa
*功能：     设置外部功率放大
*参数：     无
*返回值：   无
----------------------------------------------*/
void dwt_enable_pa(void)
{
    uint32 reg;
    reg = dwt_read32bitreg(GPIO_CTRL_ID); // read the current GPIO_CTRL_ID register 
    reg |= 0x00054000; // set the appropriate GPIOs
    dwt_write32bitreg(GPIO_CTRL_ID,reg); // write the modified value back ,外部功率放大
    dwt_write16bitoffsetreg(PMSC_ID,PMSC_RES3_OFFSET+2,0);//禁用TX功率排序
} 
