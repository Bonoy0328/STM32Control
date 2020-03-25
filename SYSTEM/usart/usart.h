#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "stm32f4xx_conf.h"
#include "sys.h" 

extern	uint8_t stop; //������Ϣ��ӡ��־λ��extern��ʾ�������ļ��ڿ������øñ���
extern	int32_t v,real_v,a,real_a; //ʵ���ٶȣ�Ŀ���ٶȣ�Ŀ����ٶȶ���
extern uint8_t alldata[51]; //���ݰ��������벶�񲿷���Ҫ�õ�
extern double wz,kp,ki,kp2,ki2,kp_a,ki_a; //PID�����ĸ��ֲ���

void uart_init(u32 bound); //��������1��ʼ������
void uart2_init(u32 bound); //��������2��ʼ������
void uart3_init(u32 bound); //��������3��ʼ������
#endif
