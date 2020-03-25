#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "pwm.h"

int main(void)
{ 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	delay_init(168);  //初始化延时函数
	uart_init(115200);//串口1初始化，ROS系统通信接口
	uart2_init(115200);//串口2初始化，接收IMU数据
	uart3_init(115200);//串口3初始化，负责调试信息打印
 	TIM1_PWM_Init(16800-1,9);	//168M/10=16800000hz的计数频率,重装载值16800，所以PWM频率为 16800KM/16800=1Khz. 
	TIM3_Cap_Init();
	TIM2_PID_Init();
	alldata[0] = 0x33;
	alldata[1] = 0x22;
  while(1);
}

