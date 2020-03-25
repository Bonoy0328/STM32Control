#include "pwm.h"
#include "led.h"
#include "usart.h"
 
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F407������
//��ʱ��PWM ��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2014/5/4
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	 


//TIM14 PWM���ֳ�ʼ�� 
//PWM�����ʼ��
//arr���Զ���װֵ
//psc��ʱ��Ԥ��Ƶ��
void TIM1_PWM_Init(u32 arr,u32 psc)
{		 					 
	//�˲������ֶ��޸�IO������
	
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);  	//TIM14ʱ��ʹ��    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE|RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOD, ENABLE); 	//ʹ��PORTFʱ��	

	GPIO_PinAFConfig(GPIOE,GPIO_PinSource9,GPIO_AF_TIM1); //GPIOE9����Ϊ��ʱ��1
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource11,GPIO_AF_TIM1); //GPIOE11����Ϊ��ʱ��1
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_11;//GPIOF9
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //����
	GPIO_Init(GPIOE,&GPIO_InitStructure);              //��ʼ��PF9
	
	/**************���Ʒ����IO�ڳ�ʼ��****************/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;//GPIOF9
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;        //���ù���
	GPIO_Init(GPIOD,&GPIO_InitStructure);              //��ʼ��PF9	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_Init(GPIOC,&GPIO_InitStructure);  
	/***********************************************/
	
	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //��ʱ����Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_Period=arr;   //�Զ���װ��ֵ
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseStructure);//��ʼ����ʱ��14
	
	//��ʼ��TIM14 Channel1 PWMģʽ	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;//????,TIM_OCNPolarity_High?????
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM1 4OC1
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);  //ʹ��TIM14��CCR1�ϵ�Ԥװ�ؼĴ���
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
  TIM_ARRPreloadConfig(TIM1,ENABLE);//ARPEʹ�� 
	
	TIM_Cmd(TIM1, ENABLE);  //ʹ��TIM14
	TIM_CtrlPWMOutputs(TIM1, ENABLE);									  
}  


//��ʱ��5ͨ��1���벶������
//arr���Զ���װֵ(TIM2,TIM5��32λ��!!)
//psc��ʱ��Ԥ��Ƶ��
void TIM3_Cap_Init(void)
{
	TIM_ICInitTypeDef  TIM3_ICInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  	//TIM5ʱ��ʹ��    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 	//ʹ��PORTAʱ��	
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_TIM3); //PA0����λ��ʱ��5
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_TIM3); //PA0����λ��ʱ��5
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7; //GPIOA0
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; //����
	GPIO_Init(GPIOA,&GPIO_InitStructure); //��ʼ��PA0
	GPIO_WriteBit(GPIOA,GPIO_Pin_6|GPIO_Pin_7,Bit_RESET);
	
	TIM_TimeBaseStructure.TIM_Prescaler=84-1;  //��ʱ����Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_Period=0xFFFF;   //�Զ���װ��ֵ
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);
	

	//��ʼ��TIM5���벶�����
	TIM3_ICInitStructure.TIM_Channel = TIM_Channel_1; //CC1S=01 	ѡ������� IC1ӳ�䵽TI1��
  TIM3_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//�����ز���
  TIM3_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
  TIM3_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //���������Ƶ,����Ƶ 
  TIM3_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 ���������˲��� ���˲�
  TIM_ICInit(TIM3, &TIM3_ICInitStructure);
	TIM3_ICInitStructure.TIM_Channel = TIM_Channel_2;
	TIM3_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInit(TIM3, &TIM3_ICInitStructure);
	TIM_ITConfig(TIM3,TIM_IT_Update|TIM_IT_CC1|TIM_IT_CC2,ENABLE);//��������ж� ,����CC1IE�����ж�	
  TIM_Cmd(TIM3,ENABLE ); 	//ʹ�ܶ�ʱ��5

  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����
}

//��ʱ��3�жϷ������	 
u8 index1=0,index2=0;
int32_t IC1Fre=0,IC2Fre=0;
int32_t Odom1 = 0,Odom2 = 0;
void TIM3_IRQHandler(void)
{ 		  
	static int32_t IC1Value1,IC1Value2,IC2Value1,IC2Value2,IC2DiffValue,IC1DiffValue;
	if(TIM_GetITStatus(TIM3, TIM_IT_CC1)!= RESET){
		if(real_v < 0){
			Odom1--;
		}else{
			Odom1++;
		}
		if(index1==0){
			IC1Value1 = TIM3->CCR1;
			index1 = 1;
			TIM3->SR &= 0x0000;
		}else if(index1==1){
			IC1Value2 = TIM3->CCR1;
			if(IC1Value2>=IC1Value1){
				IC1DiffValue = IC1Value2 - IC1Value1;
			}else{
				IC1DiffValue = ((0xFFFF - IC1Value1) + IC1Value2)+1;
			}
			IC1Fre = 1000000/IC1DiffValue;
			TIM3->SR &= 0x0000;
			if(real_v>=0){
				for(int t = 0;t<4;t++){
					alldata[t+2] = IC1Fre >> 8*t;
					alldata[t+10] = Odom1 >> 8*t;
				}			
			}else{
				for(int t = 0;t<4;t++){
					alldata[t+2] = (0-IC1Fre) >> 8*t;
					alldata[t+10] = Odom1 >> 8*t;
				}
			}
			index1 = 0;
		}
	}else if(TIM_GetITStatus(TIM3,TIM_IT_CC2) != RESET){
		if(real_v < 0){
			Odom2--;
		}else{
			Odom2++;
		}
		if(index2==0){
			IC2Value1 = TIM3->CCR2;
			index2 = 1;
			TIM3->SR &= 0x0000;
		}else if(index2==1){
			IC2Value2 = TIM3->CCR2;
			if(IC2Value2>=IC2Value1){
				IC2DiffValue = IC2Value2 - IC2Value1;
			}else{
				IC2DiffValue = ((0xFFFF - IC2Value1) + IC2Value2)+1;
			}
			IC2Fre = 1000000/IC2DiffValue;
			TIM3->SR &= 0x0000;
			if(real_v>=0){
				for(int t = 0;t<4;t++){
					alldata[t+6] = IC2Fre >> 8*t;
					alldata[t+14] = Odom2 >> 8*t;
				}			
			}else{
				for(int t = 0;t<4;t++){
					alldata[t+6] = (0-IC2Fre) >> 8*t;
					alldata[t+14] = Odom2 >> 8*t;
				}
			}
			index2 = 0;
		}
	}else{
		TIM3->SR &= 0x0000;
	}
//	TIM_ClearITPendingBit(TIM3, TIM_IT_CC1|TIM_IT_CC2); //����жϱ�־λ
}

void TIM2_PID_Init(void)
{
	TIM_ICInitTypeDef  TIM2_ICInitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);  	//TIM5ʱ��ʹ��    

	TIM_TimeBaseStructure.TIM_Prescaler=8400-1;  //��ʱ����Ƶ84000000/8400=10khz 10khz/1000hz=10hz
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_Period=500-1;   //�Զ���װ��ֵ
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);
	
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);//��������ж� ,����CC1IE�����ж�	
  TIM_Cmd(TIM2,ENABLE ); 	//ʹ�ܶ�ʱ��5

  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����
}
int32_t PWM1=0,PWM2=0,OldFre1=0,OldFre2=0;
u8 cout1=0,cout2=0;
//��ʱ��2�жϷ������	 
void TIM2_IRQHandler(void)
{ 		  
	if(IC1Fre==OldFre1)cout1++;
	else	cout1=0;
	if(IC2Fre==OldFre2)cout2++;
	else	cout2=0;
	if(cout1>8)IC1Fre=0;
	if(cout2>8)IC2Fre=0;
	if(TIM_GetITStatus(TIM2,TIM_IT_Update)!= RESET){
		if(real_v>=0){
			PWM1 += kp*(real_v - (IC1Fre+IC2Fre)/2.0) - kp2*(real_a - wz);
			PWM2 += kp*(real_v - (IC1Fre+IC2Fre)/2.0) + kp2*(real_a - wz);		
		}else{
			PWM1 += kp*(-real_v - (IC1Fre+IC2Fre)/2.0) + kp2*(real_a - wz);
			PWM2 += kp*(-real_v - (IC1Fre+IC2Fre)/2.0) - kp2*(real_a - wz);	
		}
		if(PWM1<0)PWM1=0;
		if(PWM2<0)PWM2=0;
		TIM_SetCompare1(TIM1,900+PWM1);//���� IC2Fre wz>0
		TIM_SetCompare2(TIM1,900+PWM2);//���� IC1Fre	wz<0
		if(stop==0)
			printf("%d %d %2.2f %d %d %d %d\r\n",real_v,real_a,wz,IC1Fre,IC2Fre,PWM1,PWM2);
		OldFre1 = IC1Fre;
		OldFre2 = IC2Fre;
	}
	TIM2->SR &= 0x0000;
//	TIM_ClearITPendingBit(TIM3, TIM_IT_CC1|TIM_IT_CC2); //����жϱ�־λ
}

