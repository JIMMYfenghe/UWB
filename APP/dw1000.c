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
*config��   ��������DW1000���õĽṹ��
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





srd_msg_dsss msg_f_send;//��Ϣ���ݰ�

/*-------------------------------
*������     dw1000_init
*���ܣ� 	dw1000 ������ʼ��
*������     ��
*����ֵ��   ��
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
*������     dw1000_msg_init
*���ܣ� 	DW1000��Ϣ���
*������     ��
*����ֵ��   ��
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
*������     dw1000_config
*���ܣ� 	����DW1000����ģʽ
*������     ��
*����ֵ��   ��
---------------------------------*/
void dw1000_config()
{
	dwt_configure(&config);//���ù���ģʽ
    dwt_setleds(1);//����GPIO2 3���ֱ�����TXLED,RXLED   
    dwt_SetTxPower();//���÷��书��
    dwt_setpanid(0xF0F0);//����pan id
    dwt_setaddress16(SHORT_ADDR);//��������Ķ̵�ַ
	/* Apply default antenna delay value. See NOTE 1 below. */
    dwt_setrxantennadelay(RX_ANT_DLY);//�����շ������ӳ�
    dwt_settxantennadelay(TX_ANT_DLY);
    dwt_enable_pa();//�ⲿ���ʷŴ�
	
#ifdef INT_MODE_ENABLE
    /* Enable wanted interrupts (TX confirmation, RX good frames, RX timeouts and RX errors). */
    //dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | DWT_INT_RFTO | DWT_INT_RXPTO | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFSL | DWT_INT_SFDT, 1);
    dwt_setinterrupt(DWT_INT_RFCG | (DWT_INT_ARFE | DWT_INT_RFSL | DWT_INT_SFDT | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFTO /*| DWT_INT_RXPTO*/), 1);
#endif

}
/*=====================================================================================================*
**���� : dwt_SetTxPower
**���� : ����dw1000�ķ��书��
**���� : ��
**��� : ��
**ʹ�� : dwt_SetTxPower();
**���� : BPhero
**���� : 2019/07/02
**=====================================================================================================*/
//��ʹ��6.8MbpsĬ�Ͽ������ܷ��͹��ʿ���
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
    reg=reg | 0x00040000;//�ر����ܹ���,�����ֶ��������书��
    dwt_write32bitreg(SYS_CFG_ID,reg);
    dwt_configuretxrf(configTX);//���÷���Ƶ��

}

/*-------------------------------------------
*������     dwt_enable_pa
*���ܣ�     �����ⲿ���ʷŴ�
*������     ��
*����ֵ��   ��
----------------------------------------------*/
void dwt_enable_pa(void)
{
    uint32 reg;
    reg = dwt_read32bitreg(GPIO_CTRL_ID); // read the current GPIO_CTRL_ID register 
    reg |= 0x00054000; // set the appropriate GPIOs
    dwt_write32bitreg(GPIO_CTRL_ID,reg); // write the modified value back ,�ⲿ���ʷŴ�
    dwt_write16bitoffsetreg(PMSC_ID,PMSC_RES3_OFFSET+2,0);//����TX��������
} 
