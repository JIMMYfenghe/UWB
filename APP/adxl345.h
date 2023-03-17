#ifndef __ADXL345_H 
#define __ADXL345_H
#include "sys.h"

/* ------- IIC地址及其读写地址 ------- */
//ALT ADDRESS引脚处于高电平时
#define XL345_SDOH_ADDR		0x1D
#define XL345_SDOH_WRITE	XL345_SDOH_ADDR<<1 | 0x00
#define XL345_SDOH_READ		XL345_SDOH_ADDR<<1 | 0x01
//ALT ADDRESS引脚处于低电平时
#define XL345_SDOL_ADDR		0x53
#define XL345_SDOL_WRITE	XL345_SDOL_ADDR<<1 |0x00
#define XL345_SDOL_READ		XL345_SDOL_ADDR<<1 |0x01


/* ------- 寄存器地址 ------- */
#define XL345_DEVID           0x00
#define XL345_RESERVED1       0x01
#define XL345_THRESH_TAP      0x1d
#define XL345_OFSX            0x1e
#define XL345_OFSY            0x1f
#define XL345_OFSZ            0x20
#define XL345_DUR             0x21
#define XL345_LATENT          0x22
#define XL345_WINDOW          0x23
#define XL345_THRESH_ACT      0x24
#define XL345_THRESH_INACT    0x25
#define XL345_TIME_INACT      0x26
#define XL345_ACT_INACT_CTL   0x27
#define XL345_THRESH_FF       0x28
#define XL345_TIME_FF         0x29
#define XL345_TAP_AXES        0x2a
#define XL345_ACT_TAP_STATUS  0x2b
#define XL345_BW_RATE         0x2c
#define XL345_POWER_CTL       0x2d
#define XL345_INT_ENABLE      0x2e
#define XL345_INT_MAP         0x2f
#define XL345_INT_SOURCE      0x30
#define XL345_DATA_FORMAT     0x31
#define XL345_DATAX0          0x32
#define XL345_DATAX1          0x33
#define XL345_DATAY0          0x34
#define XL345_DATAY1          0x35
#define XL345_DATAZ0          0x36
#define XL345_DATAZ1          0x37
#define XL345_FIFO_CTL        0x38
#define XL345_FIFO_STATUS     0x39

/*------------------------------------
		位字段定义和寄存器值
  ------------------------------------*/
/* 		设备ID的复位值      */
// 设备ID应该始终读取这个值，以检查设备是否可以通信 
#define XL345_ID              0xE5

/* 		保留软件复位值       */
#define XL345_SOFT_RESET      0x52

/* 寄存器THRESH_TAP到TIME_INACT只取8位值在这些寄存器中没有特定的位字段*/

/* 	ACT_INACT_CTL的位值       */
//设置为0时，选择直流耦合操作，设置为1时，使能交流耦合操作
#define XL345_INACT_Z_ENABLE  0x01
#define XL345_INACT_Z_DISABLE 0x00
#define XL345_INACT_Y_ENABLE  0x02
#define XL345_INACT_Y_DISABLE 0x00
#define XL345_INACT_X_ENABLE  0x04
#define XL345_INACT_X_DISABLE 0x00
#define XL345_INACT_AC        0x08
#define XL345_INACT_DC        0x00
#define XL345_ACT_Z_ENABLE    0x10
#define XL345_ACT_Z_DISABLE   0x00
#define XL345_ACT_Y_ENABLE    0x20
#define XL345_ACT_Y_DISABLE   0x00
#define XL345_ACT_X_ENABLE    0x40
#define XL345_ACT_X_DISABLE   0x00
#define XL345_ACT_AC          0x80
#define XL345_ACT_DC          0x00

/*	寄存器THRESH_FF和TIME_FF只取8位值
	在TIME_FF5ms的THRESH_FF 1/16/gee/LSB单元中没有特定的位字段。    */

/*  TAP_AXES 的位值  */
#define XL345_TAP_Z_ENABLE    0x01
#define XL345_TAP_Z_DISABLE   0x00
#define XL345_TAP_Y_ENABLE    0x02
#define XL345_TAP_Y_DISABLE   0x00
#define XL345_TAP_X_ENABLE    0x04
#define XL345_TAP_X_DISABLE   0x00
#define XL345_TAP_SUPPRESS    0x08

/*  ACT_TAP_STATUS 的位值   */
#define XL345_TAP_Z_SOURCE    0x01
#define XL345_TAP_Y_SOURCE    0x02
#define XL345_TAP_X_SOURCE    0x04
#define XL345_STAT_ASLEEP     0x08
#define XL345_ACT_Z_SOURCE    0x10
#define XL345_ACT_Y_SOURCE    0x20
#define XL345_ACT_X_SOURCE    0x40

/*  BW_RATE 的位值 */
/* 速率 */
#define XL345_RATE_3200       0x0f
#define XL345_RATE_1600       0x0e
#define XL345_RATE_800        0x0d
#define XL345_RATE_400        0x0c
#define XL345_RATE_200        0x0b
#define XL345_RATE_100        0x0a
#define XL345_RATE_50         0x09
#define XL345_RATE_25         0x08
#define XL345_RATE_12_5       0x07
#define XL345_RATE_6_25       0x06
#define XL345_RATE_3_125      0x05
#define XL345_RATE_1_563      0x04
#define XL345_RATE__782       0x03
#define XL345_RATE__39        0x02
#define XL345_RATE__195       0x01
#define XL345_RATE__098       0x00

/* 带宽	*/
#define XL345_BW_1600         0x0f
#define XL345_BW_800          0x0e
#define XL345_BW_400          0x0d
#define XL345_BW_200          0x0c
#define XL345_BW_100          0x0b
#define XL345_BW_50           0x0a
#define XL345_BW_25           0x09
#define XL345_BW_12_5         0x08
#define XL345_BW_6_25         0x07
#define XL345_BW_3_125        0x06
#define XL345_BW_1_563        0x05
#define XL345_BW__782         0x04
#define XL345_BW__39          0x03
#define XL345_BW__195         0x02
#define XL345_BW__098         0x01
#define XL345_BW__048         0x00

/* 正常状态和低噪声模式下的位值 */
#define XL345_LOW_POWER       0x08
#define XL345_LOW_NOISE       0x00

/* 		 POWER_CTL 的位值       */
#define XL345_WAKEUP_8HZ           0x00
#define XL345_WAKEUP_4HZ           0x01
#define XL345_WAKEUP_2HZ           0x02
#define XL345_WAKEUP_1HZ           0x03
#define XL345_SLEEP                0x04
#define XL345_MEASURE              0x08
#define XL345_STANDBY              0x00
#define XL345_AUTO_SLEEP           0x10
#define XL345_ACT_INACT_SERIAL     0x20
#define XL345_ACT_INACT_CONCURRENT 0x00

/* INT_ENABLE、INT_MAP和INT_SOURCE中的位值是相同的，使用这些位值来读取或写入任何这些寄存器。*/
#define XL345_OVERRUN              0x01
#define XL345_WATERMARK            0x02
#define XL345_FREEFALL             0x04
#define XL345_INACTIVITY           0x08
#define XL345_ACTIVITY             0x10
#define XL345_DOUBLETAP            0x20
#define XL345_SINGLETAP            0x40
#define XL345_DATAREADY            0x80

/* 		 DATA_FORMAT 的位值      */
/* 从DATAX0到DATAZ1中读取的寄存器值取决于数据格式中指定的值。 */
#define XL345_RANGE_2G             0x00
#define XL345_RANGE_4G             0x01
#define XL345_RANGE_8G             0x02
#define XL345_RANGE_16G            0x03
#define XL345_DATA_JUST_RIGHT      0x00
#define XL345_DATA_JUST_LEFT       0x04
#define XL345_10BIT                0x00
#define XL345_FULL_RESOLUTION      0x08
#define XL345_INT_LOW              0x20
#define XL345_INT_HIGH             0x00
#define XL345_SPI3WIRE             0x40
#define XL345_SPI4WIRE             0x00
#define XL345_SELFTEST             0x80


/* 		 FIFO_CTL 的位值         */
/* 	低位是用于水印或在触发模式下的预触发样本数   */
#define XL345_TRIGGER_INT1         0x00
#define XL345_TRIGGER_INT2         0x20
#define XL345_FIFO_MODE_BYPASS     0x00
#define XL345_FIFO_RESET           0x00
#define XL345_FIFO_MODE_FIFO       0x40
#define XL345_FIFO_MODE_STREAM     0x80
#define XL345_FIFO_MODE_TRIGGER    0xc0


/* 		 FIFO_STATUS 的位值       */
/* 低位是一个值0-32，显示了fifo缓冲区中当前可用的条目数量*/
#define XL345_FIFO_TRIGGERED       0x80

/*		ADXL345模块的初始化及读写寄存器等操作		*/
#define ADXL345_INT_GPIO_Port		GPIOB
#define ADXL345_INT1_Pin			GPIO_Pin_1
//#define ADXL345_INT2_Pin			GPIO_Pin_3

void ADXL345_INT(void);
void ADXL345_WR_Reg(u8 addr,u8 val);
u8 ADXL345_RD_Reg(u8 addr);
u8 ADXL345_ReadBytes(u8 addr,u8 length);
void ADXL345_WriteBytes(u8 addr,u8* data,u8 dataLength);
void ADXL345_RD_XYZ(short *x,short *y,short *z);
void ADXL345_RD_Avval(short *x,short *y,short *z);
void ADXL345_AUTO_Adjust(char *xval,char *yval,char *zval);
void ADXL345_Read_Average(short *x,short *y,short *z,u8 times);
float ADXL345_Get_Angle(float x,float y,float z,u8 dir);

void adxl345_exti(void);
#endif
