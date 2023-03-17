#include "adxl345.h"
#include "iic.h"
#include "delay.h"                                    
#include "math.h"
#define PI 3.1415926
extern void deca_sleep(unsigned int time_ms);
/*---------------------------------------
*函数：     ADXL345_INT
*功能：     初始化ADXL345中断管脚
*返回值：   无
---------------------------------------*/
void ADXL345_INT(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	//RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = ADXL345_INT1_Pin;	//PB1
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;	//输入
	GPIO_Init(ADXL345_INT_GPIO_Port, &GPIO_InitStructure);
	
}
/*-----------------------------------------
*函数：     adxl345_exti
*功能：     配置ADXL345外部中断线
*返回值：   无
-----------------------------------------*/
void adxl345_exti()
{
	  NVIC_InitTypeDef NVIC_InitStructure;
	  EXTI_InitTypeDef EXTI_InitStructure;
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
		/*跌倒模块中断*/
		SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource1);
	
		EXTI_InitStructure.EXTI_Line = EXTI_Line1;
		EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
		EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
		EXTI_InitStructure.EXTI_LineCmd = ENABLE;
		EXTI_Init(&EXTI_InitStructure);
		
		NVIC_InitStructure.NVIC_IRQChannel=EXTI1_IRQn;  
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x02; //中断抢占优先级
		NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x00;    //子优先级
		NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
		NVIC_Init(&NVIC_InitStructure);   
}

/*---------------------------------------
*函数：     ADXL345_WR_Reg
*功能：     写ADXL345寄存器
*输入：     
            addr:寄存器地址
            val:要写入的值
*返回值:    无
----------------------------------------*/
void ADXL345_WR_Reg(u8 addr,u8 val) 
{
    ADXL_IIC_Start();                 
    ADXL_IIC_Send_Byte(XL345_SDOL_WRITE);      //发送写器件指令    
    ADXL_IIC_Wait_Ack();    
    ADXL_IIC_Send_Byte(addr);            //发送寄存器地址
    ADXL_IIC_Wait_Ack();                                                        
    ADXL_IIC_Send_Byte(val);             //发送值                      
    ADXL_IIC_Wait_Ack();                    
    ADXL_IIC_Stop();                     //产生一个停止条件   
	delay_us(10);
} 

/*-----------------------------------------
*函数：     ADXL345_RD_Reg
*功能：     读ADXL345寄存器
*输入：     addr:寄存器地址
*输出：     无
*返回值:    temp    :   读取的数据
-----------------------------------------*/
u8 ADXL345_RD_Reg(u8 addr)      
{
    u8 temp=0;       
    ADXL_IIC_Start();                 
    ADXL_IIC_Send_Byte(XL345_SDOL_WRITE);  //发送写器件指令    
    ADXL_IIC_Wait_Ack();       
    ADXL_IIC_Send_Byte(addr);        //发送寄存器地址
    ADXL_IIC_Wait_Ack();                       
	
    ADXL_IIC_Start();                //重新启动
    ADXL_IIC_Send_Byte(XL345_SDOL_READ);   //发送读器件指令    
    ADXL_IIC_Wait_Ack();       
    temp=ADXL_IIC_Read_Byte(0);      //读取一个字节,不继续再读,发送NAK               
    ADXL_IIC_Stop();                 //产生一个停止条件 
	delay_us(10);
    return temp;                //返回读到的值
} 

/*-----------------------------------
*函数：     ADXL345_ReadBytes
*功能：     读多个字节
*输入：     
            addr:寄存器地址
            length:读的字节长度
*输出：     无
*返回值：   *temp   :   读到的字节数据
------------------------------------*/
u8 ADXL345_ReadBytes(u8 addr,u8 length)
{
	u8* temp=0;  
	u8 i;
    ADXL_IIC_Start();                 
    ADXL_IIC_Send_Byte(XL345_SDOL_WRITE);  //发送写器件指令    
    ADXL_IIC_Wait_Ack();       
    ADXL_IIC_Send_Byte(addr);        //发送寄存器地址
    ADXL_IIC_Wait_Ack();                                                       
    ADXL_IIC_Start();                //重新启动
    ADXL_IIC_Send_Byte(XL345_SDOL_READ);   //发送读器件指令    
    ADXL_IIC_Wait_Ack();  
	for(i = 0;i < length-1; i++)
	{
		temp[i]=ADXL_IIC_Read_Byte(1);      //读取一个字节,不继续再读,发送NAK               
    }
	temp[length-1] = ADXL_IIC_Read_Byte(0);
		ADXL_IIC_Stop();                 //产生一个停止条件 
	delay_us(10);
    return *temp;                //返回读到的值
}

/*--------------------------------------
*函数：     ADXL345_WriteBytes
*功能：     写多个字节
*输入：     
            addr:寄存器地址
            *data:要写入的字节 
            datalength:写入的字节长度
*输出：     无
*返回值：   无
----------------------------------------*/
void ADXL345_WriteBytes(u8 addr,u8* data,u8 dataLength)
{
	u8 i;
	ADXL_IIC_Start();                 
    ADXL_IIC_Send_Byte(XL345_SDOL_WRITE);      //发送写器件指令    
    ADXL_IIC_Wait_Ack();    
    ADXL_IIC_Send_Byte(addr);            //发送寄存器地址
    ADXL_IIC_Wait_Ack();    
	for(i = 0;i < dataLength; i++)
	{
		ADXL_IIC_Send_Byte(data[i]);             //发送值                      
		ADXL_IIC_Wait_Ack();        
	}
    ADXL_IIC_Stop();                     //产生一个停止条件   
	delay_us(10);
}

/*------------------------------
*函数：     ADXL345_RD_XYZ
*功能：     读取3个轴的数据
*输入：     无
*输出：     
            x： x轴数据
            y： y轴数据
            z:  z轴数据
*返回值：   无
--------------------------------*/
void ADXL345_RD_XYZ(short *x,short *y,short *z)
{
    u8 buf[6];
    u8 i;
    ADXL_IIC_Start();                 
    ADXL_IIC_Send_Byte(XL345_SDOL_WRITE);  //发送写器件指令    
    ADXL_IIC_Wait_Ack();    
    ADXL_IIC_Send_Byte(XL345_DATAX0);        //发送XL345_DATAX0地址(数据缓存的起始地址)
    ADXL_IIC_Wait_Ack();                                                        
    
    ADXL_IIC_Start();                //重新启动
    ADXL_IIC_Send_Byte(XL345_SDOL_READ);   //发送读器件指令
    ADXL_IIC_Wait_Ack();
    for(i=0;i<6;i++)
    {
        if(i==5)buf[i]=ADXL_IIC_Read_Byte(0);//读取一个字节,不继续再读,发送NACK  
        else buf[i]=ADXL_IIC_Read_Byte(1);   //读取一个字节,继续读,发送ACK 
    }                  
    ADXL_IIC_Stop();                 //产生一个停止条件
    
    *x=(short)(((u16)buf[1]<<8)+buf[0]); //低字节先读，两个字节表示一个方向位的值    
    *y=(short)(((u16)buf[3]<<8)+buf[2]);        
    *z=(short)(((u16)buf[5]<<8)+buf[4]);       

}
 
/*-------------------------------------
*函数：     ADXL345_AUTO_Adjust
*功能：     自动校准
*输入：     无
*输出：     
            xval：x轴的校准值
            yval：y轴的校准值
            zval: z轴的校准值
*返回值：   无
---------------------------------------*/
void ADXL345_AUTO_Adjust(char *xval,char *yval,char *zval)
{
    short tx,ty,tz;
    u8 i;
    short offx=0,offy=0,offz=0;
    ADXL345_WR_Reg(XL345_POWER_CTL,0x00);     //先进入休眠模式.
    deca_sleep(100);
	ADXL345_WR_Reg(XL345_DATA_FORMAT,0X0B);   //低电平中断输出,13位全分辨率,输出数据右对齐,16g量程 
    ADXL345_WR_Reg(XL345_BW_RATE,0x0A);       //数据输出速度为100Hz
    ADXL345_WR_Reg(XL345_POWER_CTL,0x08);     //测量模式
    ADXL345_WR_Reg(XL345_INT_ENABLE,0x80);    //仅Data_Ready使用中断,其他中断不开启      
    ADXL345_WR_Reg(XL345_OFSX,0x00);
    ADXL345_WR_Reg(XL345_OFSY,0x00);
    ADXL345_WR_Reg(XL345_OFSZ,0x00);

    deca_sleep(11);
    for(i=0;i<10;i++)		//100Hz的数据速率10个样本
    {
        ADXL345_RD_Avval(&tx,&ty,&tz);
        offx+=tx;
        offy+=ty;
        offz+=tz;
    }           
    offx/=10;
    offy/=10;
    offz/=10;
    *xval=-offx/4;
    *yval=-offy/4;
    *zval=-(offz-256)/4;      
    ADXL345_WR_Reg(XL345_OFSX,*xval);
    ADXL345_WR_Reg(XL345_OFSY,*yval);
    ADXL345_WR_Reg(XL345_OFSZ,*zval); 
}
 
/*---------------------------------
*函数：     ADXL345_RD_Avval
*功能：     读取ADXL的平均值
*输入：     无
*输出：     
            x：读取10次后取平均值
            y：读取10次后取平均值
            z: 读取10次后取平均值
*返回值：   无
----------------------------------*/
void ADXL345_RD_Avval(short *x,short *y,short *z)
{
    u8 i;
    short tx,ty,tz; 
	*x=0;
	*y=0;
	*z=0;
    for(i=0;i<10;i++)//连续读取times次
        {
            ADXL345_RD_XYZ(&tx,&ty,&tz);
			deca_sleep(10);
            *x += tx;
			*y += ty;
			*z += tz; 
        }
        *x/=10;
        *y/=10;
        *z/=10;
		
}  

/*--------------------------------------
*函数：     ADXL345_Read_Average
*功能：     读取ADXL345的数据times次,再取平均
*输入：     times:读取次数
*输出：     
            x：读到的数据
            y：读到的数据
            z: 读到的数据
*返回值：   无
----------------------------------------*/
void ADXL345_Read_Average(short *x,short *y,short *z,u8 times)
{
    u8 i;
    short tx,ty,tz; 
	*x=0;
	*y=0;
	*z=0;
    if(times)//读取次数不为0
    {
        for(i=0;i<times;i++)//连续读取times次
        {
            ADXL345_RD_XYZ(&tx,&ty,&tz);
            *x += tx;
			*y += ty;
			*z += tz;
            deca_sleep(5);
        }
        *x/=times;
        *y/=times;
        *z/=times;
    }
} 
 
/*------------------------------------------------------------
*函数：     ADXL345_Get_Angle
*功能：     得到角度
*输入：     
            x：x方向的重力加速度分量(不需要单位,直接数值即可)
            y：y方向的重力加速度分量(不需要单位,直接数值即可)
            z: z方向的重力加速度分量(不需要单位,直接数值即可)
            dir:要获得的角度：
                            0,与Z轴的角度
                            1,与X轴的角度
                            2,与Y轴的角度
*输出：     无
*返回值:    角度值.单位0.1°.
//res得到的是弧度值，需要将其转换为角度值也就是*180/3.14
--------------------------------------------------------------*/
float ADXL345_Get_Angle(float x,float y,float z,u8 dir)
{
    float temp;
    float res=0;
    switch(dir)
    {
        case 0://与自然Z轴的角度
            temp=sqrt((x*x+y*y))/z;
            res=atan(temp);
            break;
        case 1://与自然X轴的角度
            temp=x/sqrt((y*y+z*z));
            res=atan(temp);
            break;
        case 2://与自然Y轴的角度
            temp=y/sqrt((x*x+z*z));
            res=atan(temp);
            break;
    }
    return res*180.0/PI;
}
 
