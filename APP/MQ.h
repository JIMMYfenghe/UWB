#ifndef __MQ7_H
#define __MQ7_H

#include "stm32f4xx.h"

#define CO_CAL_PPM  10 //标准浓度
#define MQ7_RL 10 //负载阻值

#define CAL_PPM_SMOKE  20 //标准浓度
#define MQ2_RL 10 //负载阻值

void MQ_port_init(void);
void MQ_get_ad_value();
void MQ_get_ad_avg(u16 *buffer);
void DMA_ADC_config(DMA_Stream_TypeDef *DMA_Streamx,u32 chx,u32 per_addr,u32 target_addr,u16 ndtr );
float MQ_get_ppm(float *buffer);
void MQ7_calibration(float RS);
void MQ2_calibration(float RS);
void show_ppm(void);



#endif
