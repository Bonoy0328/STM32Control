#include "sys.h"
#include "usart.h"	
  
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{ 	
	while((USART3->SR&0X40)==0);//ѭ������,ֱ���������   
	USART3->DR = (u8) ch;      
	return ch;
}
 
/********************************����1��ʼ����ROSϵͳ�ӿ�**********************************************************/

uint8_t alldata[51];//���ݰ�
double wz;//z����ٶ�
uint8_t readbuf2[44];//��ȡIMU���ݻ�����
void uart_init(u32 bound){
	//GPIO�˿�����
	GPIO_InitTypeDef GPIO_InitStructure; //����GPIO���ýṹ��
	USART_InitTypeDef USART_InitStructure; //���崮�����ýṹ��
	NVIC_InitTypeDef NVIC_InitStructure; //�����ж����ýṹ��
	DMA_InitTypeDef DMA_InitStructure;  //����DMA���ýṹ��
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_DMA2,ENABLE); //ʹ��GPIOAʱ�ӣ�DMA2ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//ʹ��USART1ʱ��

	//����1��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //GPIOA9����ΪUSART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOA10����ΪUSART1

	//USART1�˿�����
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; //GPIOA9��GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOA,&GPIO_InitStructure); //��ʼ��PA9��PA10

	//USART1 ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
	USART_Init(USART1, &USART_InitStructure); //��ʼ������1

	USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);//���������ж�

	//Usart1 NVIC ����
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//����1�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;//��ռ���ȼ�2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//�����ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����
	
	//DMA����
	DMA_DeInit(DMA2_Stream5); //ʧ��DMA2��DMA����д������ʧ�ܺ�ſ�������
	while(DMA_GetCmdStatus(DMA2_Stream5)!=DISABLE); //�ȴ�ʧ�ܳɹ�
	DMA_InitStructure.DMA_BufferSize = 33; //DMA���˴�С
	DMA_InitStructure.DMA_Channel = DMA_Channel_4; //ѡ��DMAͨ��
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory; //ѡ�����赽�ڴ���˷���
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable; //ʧ��FIFOģʽ
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full; //FIFOģʽΪFULL����FIFO�Ĵ��������ٰ������ݣ�ʧ��FIFO�������ʵû����
	DMA_InitStructure.DMA_Memory0BaseAddr = (u32)readbuf2; //�ڴ���ʼ��ַ
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single; //ͻ�������ֽ�������ÿ��DMA�¼���Ҫ���˵�����������������buff��Сһ��ҪΪ����ı���
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; //�ڴ����ݸ�ʽ
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; //�ڴ��ַ�Լ�ʹ��
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular; //ѭ��ģʽ
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&(USART1->DR); //�����ַ
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single; //����ͻ���ֽڴ��������ͬ��
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//�������ݸ�ʽ
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//�����ַ����ʧ��
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;//DMA���ȼ����ǳ���
	DMA_Init(DMA2_Stream5,&DMA_InitStructure); //��ʼ��DMA
	DMA_ITConfig(DMA2_Stream5,DMA_IT_TE,ENABLE); //����DMA��������жϣ���㲻����Ҳ�� 
	USART_ClearFlag(USART1,USART_IT_TC);//������ڷ����жϱ�־λ
	DMA2->HIFCR = 0xFFFF; //���DMA2����ж�
	DMA_Cmd(DMA2_Stream5,ENABLE);//ע��һ��Ҫ��ʹ��DMA���ܣ����򴮿ڻ����������ݻᵼ��DMA����
	USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);//ʹ�ܴ���DMA����
	USART_Cmd(USART1, ENABLE);  //ʹ�ܴ���1 
}

uint8_t v_cout_flag=0,a_cout_flag=0,right_flag=0,cnt=0,a_cnt=0;
uint8_t v_control[7],a_control[5];
int32_t v=0,real_v=0,a=0,real_a=0,aa=0;
void USART1_IRQHandler(void)                	//����1�жϷ������
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
						GPIO_WriteBit(GPIOD,GPIO_Pin_11,Bit_RESET);//right_wheel,+=��ת -=��ת
						GPIO_WriteBit(GPIOC,GPIO_Pin_2,Bit_SET);//left_wheel,+=��ת -+��ת
					}else{
						GPIO_WriteBit(GPIOD,GPIO_Pin_11,Bit_SET);//right_wheel,+=��ת -=��ת
						GPIO_WriteBit(GPIOC,GPIO_Pin_2,Bit_RESET);//left_wheel,+=��ת -+��ת
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
	//GPIO�˿�����
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE); //ʹ��GPIOB USART3ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	//����3��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3); 
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3);

	//USART3�˿�����
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOB,&GPIO_InitStructure); //��ʼ��PA9��PA10
	
	//USART1 ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
	USART_Init(USART3, &USART_InitStructure); //��ʼ������1

	USART_Cmd(USART3, ENABLE);  //ʹ�ܴ���1 

	//USART_ClearFlag(USART1, USART_FLAG_TC);

	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//��������ж�

	//Usart1 NVIC ����
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;//����1�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����

}
double kp=5.5,ki=0.0,kp2=1.0,ki2=0.0,kp_a=0.0,ki_a=0.0;
u8 stop=1;
void USART3_IRQHandler(void)
{
	u16 res;
	if(USART_GetITStatus(USART3,USART_IT_RXNE)!=RESET){
//		res = USART_ReceiveData(USART3);
//		USART_SendData(USART2,res);
			//printf("���봮��3�����ж�\r\n");
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
	//GPIO�˿�����
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;	
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_DMA1,ENABLE); //ʹ��GPIOAʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);//ʹ��USART2ʱ��
	
	//����1��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2); //GPIOA2����ΪUSART2
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2); //GPIOA3����ΪUSART2

	//USART2�˿�����
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3; //GPIOA2��GPIOA3
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOA,&GPIO_InitStructure); //��ʼ��PA9��PA10

	//USART2 ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
	USART_Init(USART2, &USART_InitStructure); //��ʼ������2
	
	
	//DMA����
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
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream5_IRQn;//����2 DMA�ж�
//	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;//����2 DMA�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;//��ռ���ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//�����ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����
	DMA_ITConfig(DMA1_Stream5,DMA_IT_TC, ENABLE);//��������ж�
	DMA_Cmd(DMA1_Stream5,ENABLE);
	USART_DMACmd(USART2,USART_DMAReq_Rx,ENABLE);
	USART_Cmd(USART2, ENABLE);  //ʹ�ܴ���2
}

void DMA1_Stream5_IRQHandler(void){
	if(DMA_GetITStatus(DMA1_Stream5,DMA_IT_TCIF5)!=RESET){
		uint8_t yawflag = 0,index1=0,index2=0;
		for(int i_flag=0;i_flag<33;i_flag++){
			if(readbuf[i_flag]==0x55&&readbuf[i_flag+1]==0x51){
				yawflag = i_flag;//���ٶ�
				break;
			}
		}
		for(int i_flag=yawflag;i_flag<33;i_flag++){
			alldata[18+i_flag-yawflag]=readbuf[i_flag];
		}
		for(int i_flag=0;i_flag<yawflag;i_flag++){
			alldata[18+33-yawflag+i_flag]=readbuf[i_flag];
		}
		if(yawflag+17>32)index1=yawflag+17-33; //z����ٶȵͰ�λ
		else index1=yawflag+17;
		if(yawflag+18>32)index2=yawflag+18-33; //z����ٶȸ߰�λ
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