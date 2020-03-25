#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "pwm.h"

int main(void)
{ 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//����ϵͳ�ж����ȼ�����2
	delay_init(168);  //��ʼ����ʱ����
	uart_init(115200);//����1��ʼ����ROSϵͳͨ�Žӿ�
	uart2_init(115200);//����2��ʼ��������IMU����
	uart3_init(115200);//����3��ʼ�������������Ϣ��ӡ
 	TIM1_PWM_Init(16800-1,9);	//168M/10=16800000hz�ļ���Ƶ��,��װ��ֵ16800������PWMƵ��Ϊ 16800KM/16800=1Khz. 
	TIM3_Cap_Init();
	TIM2_PID_Init();
	alldata[0] = 0x33;
	alldata[1] = 0x22;
  while(1);
}

