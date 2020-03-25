#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "stm32f4xx_conf.h"
#include "sys.h" 

extern	uint8_t stop; //调试信息打印标志位，extern表示在其他文件内可以引用该变量
extern	int32_t v,real_v,a,real_a; //实际速度，目标速度，目标角速度定义
extern uint8_t alldata[51]; //数据包，在输入捕获部分需要用到
extern double wz,kp,ki,kp2,ki2,kp_a,ki_a; //PID整定的各种参数

void uart_init(u32 bound); //声明串口1初始化函数
void uart2_init(u32 bound); //声明串口2初始化函数
void uart3_init(u32 bound); //声明串口3初始化函数
#endif
