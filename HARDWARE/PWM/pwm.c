#include "pwm.h"
#include "led.h"
#include "usart.h"
 
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//定时器PWM 驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2014/5/4
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	 


//TIM14 PWM部分初始化 
//PWM输出初始化
//arr：自动重装值
//psc：时钟预分频数
void TIM1_PWM_Init(u32 arr,u32 psc)
{		 					 
	//此部分需手动修改IO口设置
	
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);  	//TIM14时钟使能    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE|RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOD, ENABLE); 	//使能PORTF时钟	

	GPIO_PinAFConfig(GPIOE,GPIO_PinSource9,GPIO_AF_TIM1); //GPIOE9复用为定时器1
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource11,GPIO_AF_TIM1); //GPIOE11复用为定时器1
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_11;//GPIOF9
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
	GPIO_Init(GPIOE,&GPIO_InitStructure);              //初始化PF9
	
	/**************控制方向的IO口初始化****************/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;//GPIOF9
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;        //复用功能
	GPIO_Init(GPIOD,&GPIO_InitStructure);              //初始化PF9	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_Init(GPIOC,&GPIO_InitStructure);  
	/***********************************************/
	
	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period=arr;   //自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseStructure);//初始化定时器14
	
	//初始化TIM14 Channel1 PWM模式	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;//????,TIM_OCNPolarity_High?????
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM1 4OC1
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);  //使能TIM14在CCR1上的预装载寄存器
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
  TIM_ARRPreloadConfig(TIM1,ENABLE);//ARPE使能 
	
	TIM_Cmd(TIM1, ENABLE);  //使能TIM14
	TIM_CtrlPWMOutputs(TIM1, ENABLE);									  
}  


//定时器5通道1输入捕获配置
//arr：自动重装值(TIM2,TIM5是32位的!!)
//psc：时钟预分频数
void TIM3_Cap_Init(void)
{
	TIM_ICInitTypeDef  TIM3_ICInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  	//TIM5时钟使能    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 	//使能PORTA时钟	
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_TIM3); //PA0复用位定时器5
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_TIM3); //PA0复用位定时器5
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7; //GPIOA0
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; //下拉
	GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化PA0
	GPIO_WriteBit(GPIOA,GPIO_Pin_6|GPIO_Pin_7,Bit_RESET);
	
	TIM_TimeBaseStructure.TIM_Prescaler=84-1;  //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period=0xFFFF;   //自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);
	

	//初始化TIM5输入捕获参数
	TIM3_ICInitStructure.TIM_Channel = TIM_Channel_1; //CC1S=01 	选择输入端 IC1映射到TI1上
  TIM3_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//上升沿捕获
  TIM3_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
  TIM3_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //配置输入分频,不分频 
  TIM3_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 配置输入滤波器 不滤波
  TIM_ICInit(TIM3, &TIM3_ICInitStructure);
	TIM3_ICInitStructure.TIM_Channel = TIM_Channel_2;
	TIM3_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInit(TIM3, &TIM3_ICInitStructure);
	TIM_ITConfig(TIM3,TIM_IT_Update|TIM_IT_CC1|TIM_IT_CC2,ENABLE);//允许更新中断 ,允许CC1IE捕获中断	
  TIM_Cmd(TIM3,ENABLE ); 	//使能定时器5

  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、
}

//定时器3中断服务程序	 
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
//	TIM_ClearITPendingBit(TIM3, TIM_IT_CC1|TIM_IT_CC2); //清除中断标志位
}

void TIM2_PID_Init(void)
{
	TIM_ICInitTypeDef  TIM2_ICInitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);  	//TIM5时钟使能    

	TIM_TimeBaseStructure.TIM_Prescaler=8400-1;  //定时器分频84000000/8400=10khz 10khz/1000hz=10hz
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period=500-1;   //自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);
	
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);//允许更新中断 ,允许CC1IE捕获中断	
  TIM_Cmd(TIM2,ENABLE ); 	//使能定时器5

  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、
}
int32_t PWM1=0,PWM2=0,OldFre1=0,OldFre2=0;
u8 cout1=0,cout2=0;
//定时器2中断服务程序	 
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
		TIM_SetCompare1(TIM1,900+PWM1);//左轮 IC2Fre wz>0
		TIM_SetCompare2(TIM1,900+PWM2);//右轮 IC1Fre	wz<0
		if(stop==0)
			printf("%d %d %2.2f %d %d %d %d\r\n",real_v,real_a,wz,IC1Fre,IC2Fre,PWM1,PWM2);
		OldFre1 = IC1Fre;
		OldFre2 = IC2Fre;
	}
	TIM2->SR &= 0x0000;
//	TIM_ClearITPendingBit(TIM3, TIM_IT_CC1|TIM_IT_CC2); //清除中断标志位
}

