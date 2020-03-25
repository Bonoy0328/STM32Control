#include "sys.h"
#include "usart.h"	
  
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{ 	
	while((USART3->SR&0X40)==0);//循环发送,直到发送完毕   
	USART3->DR = (u8) ch;      
	return ch;
}
 
/********************************串口1初始化，ROS系统接口**********************************************************/

uint8_t alldata[51];//数据包
double wz;//z轴角速度
uint8_t readbuf2[44];//读取IMU数据缓冲区
void uart_init(u32 bound){
	//GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure; //定义GPIO配置结构体
	USART_InitTypeDef USART_InitStructure; //定义串口配置结构体
	NVIC_InitTypeDef NVIC_InitStructure; //定义中断配置结构体
	DMA_InitTypeDef DMA_InitStructure;  //定义DMA配置结构体
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_DMA2,ENABLE); //使能GPIOA时钟，DMA2时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//使能USART1时钟

	//串口1对应引脚复用映射
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //GPIOA9复用为USART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOA10复用为USART1

	//USART1端口配置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; //GPIOA9与GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化PA9，PA10

	//USART1 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
	USART_Init(USART1, &USART_InitStructure); //初始化串口1

	USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);//开启空闲中断

	//Usart1 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//串口1中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;//抢占优先级2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//子优先级0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、
	
	//DMA配置
	DMA_DeInit(DMA2_Stream5); //失能DMA2，DMA具有写保护，失能后才可以配置
	while(DMA_GetCmdStatus(DMA2_Stream5)!=DISABLE); //等待失能成功
	DMA_InitStructure.DMA_BufferSize = 33; //DMA搬运大小
	DMA_InitStructure.DMA_Channel = DMA_Channel_4; //选择DMA通道
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory; //选择外设到内存搬运方向
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable; //失能FIFO模式
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full; //FIFO模式为FULL，即FIFO寄存器满后再搬运数据，失能FIFO后此项其实没作用
	DMA_InitStructure.DMA_Memory0BaseAddr = (u32)readbuf2; //内存起始地址
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single; //突发传输字节数，即每次DMA事件需要搬运的数据量，搬运数据buff大小一定要为此项的倍数
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; //内存数据格式
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; //内存地址自加使能
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular; //循环模式
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&(USART1->DR); //外设地址
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single; //外设突发字节传输个数，同上
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//外设数据格式
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设地址自增失能
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;//DMA优先级，非常高
	DMA_Init(DMA2_Stream5,&DMA_InitStructure); //初始化DMA
	DMA_ITConfig(DMA2_Stream5,DMA_IT_TE,ENABLE); //设置DMA传输错误中断，随便不设置也可 
	USART_ClearFlag(USART1,USART_IT_TC);//清除串口发送中断标志位
	DMA2->HIFCR = 0xFFFF; //清除DMA2相关中断
	DMA_Cmd(DMA2_Stream5,ENABLE);//注意一定要先使能DMA功能，否则串口缓存区有数据会导致DMA错误
	USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);//使能串口DMA功能
	USART_Cmd(USART1, ENABLE);  //使能串口1 
}

uint8_t v_cout_flag=0,a_cout_flag=0,right_flag=0,cnt=0,a_cnt=0;
uint8_t v_control[7],a_control[5];
int32_t v=0,real_v=0,a=0,real_a=0,aa=0;
void USART1_IRQHandler(void)                	//串口1中断服务程序
{
	if(USART_GetITStatus(USART1,USART_IT_IDLE)!=RESET){
		DMA_Cmd(DMA2_Stream5,DISABLE);
		v = USART1->SR;
		v = USART1->DR;
		cnt = DMA_GetCurrDataCounter(DMA2_Stream5);
		if(33-cnt==9){
			for(int j=0;j<9;j++){
				if(readbuf2[0]==0x0a){
					real_v = ((uint32_t)readbuf2[1]<<24)|((uint32_t)readbuf2[2]<<16)|((uint32_t)readbuf2[3]<<8)|((uint32_t)readbuf2[4]);
					real_a = ((uint32_t)readbuf2[5]<<24)|((uint32_t)readbuf2[6]<<16)|((uint32_t)readbuf2[7]<<8)|((uint32_t)readbuf2[8]);
					if(real_v>=0){
						GPIO_WriteBit(GPIOD,GPIO_Pin_11,Bit_RESET);//right_wheel,+=反转 -=正转
						GPIO_WriteBit(GPIOC,GPIO_Pin_2,Bit_SET);//left_wheel,+=正转 -+反转
					}else{
						GPIO_WriteBit(GPIOD,GPIO_Pin_11,Bit_SET);//right_wheel,+=反转 -=正转
						GPIO_WriteBit(GPIOC,GPIO_Pin_2,Bit_RESET);//left_wheel,+=正转 -+反转
					}
				}
			}
		}
		if(33-cnt==2){
			if((readbuf2[0]==0x43)&(readbuf2[1]==0x44)){
				for(uint8_t tt=0;tt<51;tt++){
					while((USART1->SR&0x40)==0);
					USART1->DR = alldata[tt];
				}
			}
		}
		DMA2_Stream5->NDTR = 33;
		DMA_Cmd(DMA2_Stream5,ENABLE);
	}
	USART_ClearITPendingBit(USART1,USART_IT_IDLE);
} 

void uart3_init(u32 bound){
	//GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE); //使能GPIOB USART3时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	//串口3对应引脚复用映射
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3); 
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3);

	//USART3端口配置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOB,&GPIO_InitStructure); //初始化PA9，PA10
	
	//USART1 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
	USART_Init(USART3, &USART_InitStructure); //初始化串口1

	USART_Cmd(USART3, ENABLE);  //使能串口1 

	//USART_ClearFlag(USART1, USART_FLAG_TC);

	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//开启相关中断

	//Usart1 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;//串口1中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、

}
double kp=5.5,ki=0.0,kp2=1.0,ki2=0.0,kp_a=0.0,ki_a=0.0;
u8 stop=1;
void USART3_IRQHandler(void)
{
	u16 res;
	if(USART_GetITStatus(USART3,USART_IT_RXNE)!=RESET){
//		res = USART_ReceiveData(USART3);
//		USART_SendData(USART2,res);
			//printf("进入串口3接收中断\r\n");
		u8 opr;
		opr = USART3->DR;
		if(opr==0x01){
			kp+=0.1;
			//printf("kp:%.1f ki:%.3f kd:%f kp2=%.1f kd2=%f ki2=%.3f kp_a:%.1f ki_a:%f kd_a=%f\r\n",kp,ki,kd,kp2,ki2,kd2,kp_a,ki_a,kd_a);
		}else if(opr==0x02){
			kp-=0.1;
			//printf("kp:%f ki:%f kd:%f kp2=%f kd2=%f ki2=%f kp_a:%f ki_a:%f kd_a=%f\r\n",kp,ki,kd,kp2,ki2,kd2,kp_a,ki_a,kd_a);
		}else if(opr==0x03){
			ki+=0.01;
			//printf("kp:%f ki:%f kd:%f kp2=%f kd2=%f ki2=%f kp_a:%f ki_a:%f kd_a=%f\r\n",kp,ki,kd,kp2,ki2,kd2,kp_a,ki_a,kd_a);
		}else if(opr==0x04){
			ki-=0.01;
			//printf("kp:%f ki:%f kd:%f kp2=%f kd2=%f ki2=%f kp_a:%f ki_a:%f kd_a=%f\r\n",kp,ki,kd,kp2,ki2,kd2,kp_a,ki_a,kd_a);
		}else if(opr==0x05){
			kp2+=0.1;
			//printf("kp:%f ki:%f kd:%f kp2=%f kd2=%f ki2=%f kp_a:%f ki_a:%f kd_a=%f\r\n",kp,ki,kd,kp2,ki2,kd2,kp_a,ki_a,kd_a);
		}else if(opr==0x06){
			kp2-=0.1;
			//printf("kp:%f ki:%f kd:%f kp2=%f kd2=%f ki2=%f kp_a:%f ki_a:%f kd_a=%f\r\n",kp,ki,kd,kp2,ki2,kd2,kp_a,ki_a,kd_a);
		}else if(opr==0x07){
			ki2+=0.01;
			//printf("kp:%f ki:%f kd:%f kp2=%f kd2=%f ki2=%f kp_a:%f ki_a:%f kd_a=%f\r\n",kp,ki,kd,kp2,ki2,kd2,kp_a,ki_a,kd_a);
		}else if(opr==0x08){
			ki2-=0.01;
			//printf("kp:%f ki:%f kd:%f kp2=%f kd2=%f ki2=%f kp_a:%f ki_a:%f kd_a=%f\r\n",kp,ki,kd,kp2,ki2,kd2,kp_a,ki_a,kd_a);
		}else if(opr==0x09){
			kp_a+=0.1;
			//printf("kp:%f ki:%f kd:%f kp2=%f kd2=%f ki2=%f kp_a:%f ki_a:%f kd_a=%f\r\n",kp,ki,kd,kp2,ki2,kd2,kp_a,ki_a,kd_a);
		}else if(opr==0x0a){
			kp_a-=0.1;
			//printf("kp:%f ki:%f kd:%f kp2=%f kd2=%f ki2=%f kp_a:%f ki_a:%f kd_a=%f\r\n",kp,ki,kd,kp2,ki2,kd2,kp_a,ki_a,kd_a);
		}else if(opr==0x0b){
			ki_a+=0.01;
			//printf("kp:%f ki:%f kd:%f kp2=%f kd2=%f ki2=%f kp_a:%f ki_a:%f kd_a=%f\r\n",kp,ki,kd,kp2,ki2,kd2,kp_a,ki_a,kd_a);
		}else if(opr==0x0c){
			ki_a-=0.01;
			//printf("kp:%f ki:%f kd:%f kp2=%f kd2=%f ki2=%f kp_a:%f ki_a:%f kd_a=%f\r\n",kp,ki,kd,kp2,ki2,kd2,kp_a,ki_a,kd_a);
		}else if(opr==0x0d){
			real_v +=50;
		}else if(opr==0x0e){
			real_v -=50;
		}else if(opr==0x0f){
			real_a +=5;
		}else if(opr==0x10){
			real_a -=5;
		}else if(opr==0x11){
			stop=1;
		}else if(opr==0x12){
			stop=0;
		}
		//printf("kp:%.1f ki:%.3f kd:%.3f kp2=%.1f kd2=%.3f ki2=%.3f kp_a:%.1f ki_a:%.3f kd_a=%.3f\r\n",kp,ki,kd,kp2,ki2,kd2,kp_a,ki_a,kd_a);
		printf("kp1:%.1f kp2:%.1f v: %d a:%d \r\n",kp,kp2,real_v,real_a);
	}
	USART3->SR=0x00;
} 
uint8_t readbuf[44];
void uart2_init(u32 bound){
	//GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;	
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_DMA1,ENABLE); //使能GPIOA时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);//使能USART2时钟
	
	//串口1对应引脚复用映射
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2); //GPIOA2复用为USART2
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2); //GPIOA3复用为USART2

	//USART2端口配置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3; //GPIOA2与GPIOA3
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化PA9，PA10

	//USART2 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
	USART_Init(USART2, &USART_InitStructure); //初始化串口2
	
	
	//DMA配置
	DMA_DeInit(DMA1_Stream5);
	while(DMA_GetCmdStatus(DMA1_Stream5)!=DISABLE);
	DMA_InitStructure.DMA_BufferSize = 33;
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_Memory0BaseAddr = (u32)readbuf;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&(USART2->DR);
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
	DMA_Init(DMA1_Stream5,&DMA_InitStructure);
//		USART_ClearFlag(USART2, USART_FLAG_TC);
//		USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream5_IRQn;//串口2 DMA中断
//	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;//串口2 DMA中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;//抢占优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//子优先级1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、
	DMA_ITConfig(DMA1_Stream5,DMA_IT_TC, ENABLE);//开启相关中断
	DMA_Cmd(DMA1_Stream5,ENABLE);
	USART_DMACmd(USART2,USART_DMAReq_Rx,ENABLE);
	USART_Cmd(USART2, ENABLE);  //使能串口2
}

void DMA1_Stream5_IRQHandler(void){
	if(DMA_GetITStatus(DMA1_Stream5,DMA_IT_TCIF5)!=RESET){
		uint8_t yawflag = 0,index1=0,index2=0;
		for(int i_flag=0;i_flag<33;i_flag++){
			if(readbuf[i_flag]==0x55&&readbuf[i_flag+1]==0x51){
				yawflag = i_flag;//角速度
				break;
			}
		}
		for(int i_flag=yawflag;i_flag<33;i_flag++){
			alldata[18+i_flag-yawflag]=readbuf[i_flag];
		}
		for(int i_flag=0;i_flag<yawflag;i_flag++){
			alldata[18+33-yawflag+i_flag]=readbuf[i_flag];
		}
		if(yawflag+17>32)index1=yawflag+17-33; //z轴角速度低八位
		else index1=yawflag+17;
		if(yawflag+18>32)index2=yawflag+18-33; //z轴角速度高八位
		else	index2=yawflag+18;
		wz = ((short)((((uint16_t)readbuf[index2]<<8)|readbuf[index1])))/32768.0*2000*10;
		DMA1->HIFCR = 0xFFFF;
	}
	DMA1->HIFCR=0xFFFF;
}

void USART2_IRQHandler(void){
	u16 res;
	if(USART_GetITStatus(USART2,USART_IT_RXNE)!=RESET){
		res = USART_ReceiveData(USART2);
		USART_SendData(USART3,res);
	}
	USART2->SR=0x00;
}