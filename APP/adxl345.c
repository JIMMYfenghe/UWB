#include "adxl345.h"
#include "iic.h"
#include "delay.h"                                    
#include "math.h"
#define PI 3.1415926
extern void deca_sleep(unsigned int time_ms);
/*---------------------------------------
*������     ADXL345_INT
*���ܣ�     ��ʼ��ADXL345�жϹܽ�
*����ֵ��   ��
---------------------------------------*/
void ADXL345_INT(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	//RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = ADXL345_INT1_Pin;	//PB1
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;	//����
	GPIO_Init(ADXL345_INT_GPIO_Port, &GPIO_InitStructure);
	
}
/*-----------------------------------------
*������     adxl345_exti
*���ܣ�     ����ADXL345�ⲿ�ж���
*����ֵ��   ��
-----------------------------------------*/
void adxl345_exti()
{
	  NVIC_InitTypeDef NVIC_InitStructure;
	  EXTI_InitTypeDef EXTI_InitStructure;
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
		/*����ģ���ж�*/
		SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource1);
	
		EXTI_InitStructure.EXTI_Line = EXTI_Line1;
		EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
		EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
		EXTI_InitStructure.EXTI_LineCmd = ENABLE;
		EXTI_Init(&EXTI_InitStructure);
		
		NVIC_InitStructure.NVIC_IRQChannel=EXTI1_IRQn;  
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x02; //�ж���ռ���ȼ�
		NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x00;    //�����ȼ�
		NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
		NVIC_Init(&NVIC_InitStructure);   
}

/*---------------------------------------
*������     ADXL345_WR_Reg
*���ܣ�     дADXL345�Ĵ���
*���룺     
            addr:�Ĵ�����ַ
            val:Ҫд���ֵ
*����ֵ:    ��
----------------------------------------*/
void ADXL345_WR_Reg(u8 addr,u8 val) 
{
    ADXL_IIC_Start();                 
    ADXL_IIC_Send_Byte(XL345_SDOL_WRITE);      //����д����ָ��    
    ADXL_IIC_Wait_Ack();    
    ADXL_IIC_Send_Byte(addr);            //���ͼĴ�����ַ
    ADXL_IIC_Wait_Ack();                                                        
    ADXL_IIC_Send_Byte(val);             //����ֵ                      
    ADXL_IIC_Wait_Ack();                    
    ADXL_IIC_Stop();                     //����һ��ֹͣ����   
	delay_us(10);
} 

/*-----------------------------------------
*������     ADXL345_RD_Reg
*���ܣ�     ��ADXL345�Ĵ���
*���룺     addr:�Ĵ�����ַ
*�����     ��
*����ֵ:    temp    :   ��ȡ������
-----------------------------------------*/
u8 ADXL345_RD_Reg(u8 addr)      
{
    u8 temp=0;       
    ADXL_IIC_Start();                 
    ADXL_IIC_Send_Byte(XL345_SDOL_WRITE);  //����д����ָ��    
    ADXL_IIC_Wait_Ack();       
    ADXL_IIC_Send_Byte(addr);        //���ͼĴ�����ַ
    ADXL_IIC_Wait_Ack();                       
	
    ADXL_IIC_Start();                //��������
    ADXL_IIC_Send_Byte(XL345_SDOL_READ);   //���Ͷ�����ָ��    
    ADXL_IIC_Wait_Ack();       
    temp=ADXL_IIC_Read_Byte(0);      //��ȡһ���ֽ�,�������ٶ�,����NAK               
    ADXL_IIC_Stop();                 //����һ��ֹͣ���� 
	delay_us(10);
    return temp;                //���ض�����ֵ
} 

/*-----------------------------------
*������     ADXL345_ReadBytes
*���ܣ�     ������ֽ�
*���룺     
            addr:�Ĵ�����ַ
            length:�����ֽڳ���
*�����     ��
*����ֵ��   *temp   :   �������ֽ�����
------------------------------------*/
u8 ADXL345_ReadBytes(u8 addr,u8 length)
{
	u8* temp=0;  
	u8 i;
    ADXL_IIC_Start();                 
    ADXL_IIC_Send_Byte(XL345_SDOL_WRITE);  //����д����ָ��    
    ADXL_IIC_Wait_Ack();       
    ADXL_IIC_Send_Byte(addr);        //���ͼĴ�����ַ
    ADXL_IIC_Wait_Ack();                                                       
    ADXL_IIC_Start();                //��������
    ADXL_IIC_Send_Byte(XL345_SDOL_READ);   //���Ͷ�����ָ��    
    ADXL_IIC_Wait_Ack();  
	for(i = 0;i < length-1; i++)
	{
		temp[i]=ADXL_IIC_Read_Byte(1);      //��ȡһ���ֽ�,�������ٶ�,����NAK               
    }
	temp[length-1] = ADXL_IIC_Read_Byte(0);
		ADXL_IIC_Stop();                 //����һ��ֹͣ���� 
	delay_us(10);
    return *temp;                //���ض�����ֵ
}

/*--------------------------------------
*������     ADXL345_WriteBytes
*���ܣ�     д����ֽ�
*���룺     
            addr:�Ĵ�����ַ
            *data:Ҫд����ֽ� 
            datalength:д����ֽڳ���
*�����     ��
*����ֵ��   ��
----------------------------------------*/
void ADXL345_WriteBytes(u8 addr,u8* data,u8 dataLength)
{
	u8 i;
	ADXL_IIC_Start();                 
    ADXL_IIC_Send_Byte(XL345_SDOL_WRITE);      //����д����ָ��    
    ADXL_IIC_Wait_Ack();    
    ADXL_IIC_Send_Byte(addr);            //���ͼĴ�����ַ
    ADXL_IIC_Wait_Ack();    
	for(i = 0;i < dataLength; i++)
	{
		ADXL_IIC_Send_Byte(data[i]);             //����ֵ                      
		ADXL_IIC_Wait_Ack();        
	}
    ADXL_IIC_Stop();                     //����һ��ֹͣ����   
	delay_us(10);
}

/*------------------------------
*������     ADXL345_RD_XYZ
*���ܣ�     ��ȡ3���������
*���룺     ��
*�����     
            x�� x������
            y�� y������
            z:  z������
*����ֵ��   ��
--------------------------------*/
void ADXL345_RD_XYZ(short *x,short *y,short *z)
{
    u8 buf[6];
    u8 i;
    ADXL_IIC_Start();                 
    ADXL_IIC_Send_Byte(XL345_SDOL_WRITE);  //����д����ָ��    
    ADXL_IIC_Wait_Ack();    
    ADXL_IIC_Send_Byte(XL345_DATAX0);        //����XL345_DATAX0��ַ(���ݻ������ʼ��ַ)
    ADXL_IIC_Wait_Ack();                                                        
    
    ADXL_IIC_Start();                //��������
    ADXL_IIC_Send_Byte(XL345_SDOL_READ);   //���Ͷ�����ָ��
    ADXL_IIC_Wait_Ack();
    for(i=0;i<6;i++)
    {
        if(i==5)buf[i]=ADXL_IIC_Read_Byte(0);//��ȡһ���ֽ�,�������ٶ�,����NACK  
        else buf[i]=ADXL_IIC_Read_Byte(1);   //��ȡһ���ֽ�,������,����ACK 
    }                  
    ADXL_IIC_Stop();                 //����һ��ֹͣ����
    
    *x=(short)(((u16)buf[1]<<8)+buf[0]); //���ֽ��ȶ��������ֽڱ�ʾһ������λ��ֵ    
    *y=(short)(((u16)buf[3]<<8)+buf[2]);        
    *z=(short)(((u16)buf[5]<<8)+buf[4]);       

}
 
/*-------------------------------------
*������     ADXL345_AUTO_Adjust
*���ܣ�     �Զ�У׼
*���룺     ��
*�����     
            xval��x���У׼ֵ
            yval��y���У׼ֵ
            zval: z���У׼ֵ
*����ֵ��   ��
---------------------------------------*/
void ADXL345_AUTO_Adjust(char *xval,char *yval,char *zval)
{
    short tx,ty,tz;
    u8 i;
    short offx=0,offy=0,offz=0;
    ADXL345_WR_Reg(XL345_POWER_CTL,0x00);     //�Ƚ�������ģʽ.
    deca_sleep(100);
	ADXL345_WR_Reg(XL345_DATA_FORMAT,0X0B);   //�͵�ƽ�ж����,13λȫ�ֱ���,��������Ҷ���,16g���� 
    ADXL345_WR_Reg(XL345_BW_RATE,0x0A);       //��������ٶ�Ϊ100Hz
    ADXL345_WR_Reg(XL345_POWER_CTL,0x08);     //����ģʽ
    ADXL345_WR_Reg(XL345_INT_ENABLE,0x80);    //��Data_Readyʹ���ж�,�����жϲ�����      
    ADXL345_WR_Reg(XL345_OFSX,0x00);
    ADXL345_WR_Reg(XL345_OFSY,0x00);
    ADXL345_WR_Reg(XL345_OFSZ,0x00);

    deca_sleep(11);
    for(i=0;i<10;i++)		//100Hz����������10������
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
*������     ADXL345_RD_Avval
*���ܣ�     ��ȡADXL��ƽ��ֵ
*���룺     ��
*�����     
            x����ȡ10�κ�ȡƽ��ֵ
            y����ȡ10�κ�ȡƽ��ֵ
            z: ��ȡ10�κ�ȡƽ��ֵ
*����ֵ��   ��
----------------------------------*/
void ADXL345_RD_Avval(short *x,short *y,short *z)
{
    u8 i;
    short tx,ty,tz; 
	*x=0;
	*y=0;
	*z=0;
    for(i=0;i<10;i++)//������ȡtimes��
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
*������     ADXL345_Read_Average
*���ܣ�     ��ȡADXL345������times��,��ȡƽ��
*���룺     times:��ȡ����
*�����     
            x������������
            y������������
            z: ����������
*����ֵ��   ��
----------------------------------------*/
void ADXL345_Read_Average(short *x,short *y,short *z,u8 times)
{
    u8 i;
    short tx,ty,tz; 
	*x=0;
	*y=0;
	*z=0;
    if(times)//��ȡ������Ϊ0
    {
        for(i=0;i<times;i++)//������ȡtimes��
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
*������     ADXL345_Get_Angle
*���ܣ�     �õ��Ƕ�
*���룺     
            x��x������������ٶȷ���(����Ҫ��λ,ֱ����ֵ����)
            y��y������������ٶȷ���(����Ҫ��λ,ֱ����ֵ����)
            z: z������������ٶȷ���(����Ҫ��λ,ֱ����ֵ����)
            dir:Ҫ��õĽǶȣ�
                            0,��Z��ĽǶ�
                            1,��X��ĽǶ�
                            2,��Y��ĽǶ�
*�����     ��
*����ֵ:    �Ƕ�ֵ.��λ0.1��.
//res�õ����ǻ���ֵ����Ҫ����ת��Ϊ�Ƕ�ֵҲ����*180/3.14
--------------------------------------------------------------*/
float ADXL345_Get_Angle(float x,float y,float z,u8 dir)
{
    float temp;
    float res=0;
    switch(dir)
    {
        case 0://����ȻZ��ĽǶ�
            temp=sqrt((x*x+y*y))/z;
            res=atan(temp);
            break;
        case 1://����ȻX��ĽǶ�
            temp=x/sqrt((y*y+z*z));
            res=atan(temp);
            break;
        case 2://����ȻY��ĽǶ�
            temp=y/sqrt((x*x+z*z));
            res=atan(temp);
            break;
    }
    return res*180.0/PI;
}
 
