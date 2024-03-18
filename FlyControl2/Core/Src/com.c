#include "com.h"

extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;

void Uart1_Send_Data(uint8_t Tx_Num1);
void Uart2_Send_Data(uint8_t Tx_Num1);
void Uart3_Send_Data(uint8_t Tx_Num1);
void Uart4_Send_Data(uint8_t Tx_Num1);
void Uart5_Send_Data(uint8_t Tx_Num1);
void Uart6_Send_Data(uint8_t Tx_Num1);

void ComRX(void);
void ComTX(void);
extern uint16_t Run_State ; 
extern uint16_t Sys_timer;
extern uint16_t Sys_timer2;

uint8_t RX_CheckSum(uint8_t *buf, uint8_t len); //buf为数组，len为数组长度

uint8_t Tx_Data1[10] ={0,0,0,0,0,0,0,0,0,0};
uint8_t Tx_Data2[10] ={0,0,0,0,0,0,0,0,0,0};
uint8_t Tx_Data3[10] ={0,0,0,0,0,0,0,0,0,0};
uint8_t Tx_Data4[10] ={0,0,0,0,0,0,0,0,0,0};
uint8_t Tx_Data5[10] ={0,0,0,0,0,0,0,0,0,0};
uint8_t Tx_Data6[10] ={0,0,0,0,0,0,0,0,0,0};
uint8_t RollH, RollL, PitchH, PitchL, YawH, YawL ;	
uint8_t TEST01[20] ={0};
typedef struct
{
  float Roll;
	float Pitch;
	float Yaw;	
	float backup3;
} S1;
S1 Angel ={0,0,0,0};

void ComTX()
{
		Uart1_Send_Data(10);
    Uart2_Send_Data(10);
    Uart4_Send_Data(10);
    Uart5_Send_Data(10);
//	else if(Sys_timer1 == 200)
//	{
//		Uart2_Send_Data(10);
//	}
}		

void ComRX()
{
	//串口3 陀螺
	if((Com_RxLen[3] == 11) & (USART3_RX_BUF[2] == 0x01))
	{		
		RollL =  USART3_RX_BUF[4] ;
		RollH =  USART3_RX_BUF[5] ;
		PitchL = USART3_RX_BUF[6] ;
		PitchH = USART3_RX_BUF[7] ;
		YawL = USART3_RX_BUF[8] ; 
		YawH = USART3_RX_BUF[9] ; 
    Com_RxLen[3] = 0 ;
	}	
	//横滚角
	Angel.Roll =  (float)((int16_t)(RollH << 8) | RollL) / (float)32768.0 * (float)180.0 ;
	//俯仰角
	Angel.Pitch =  (float)((int16_t)(PitchH << 8) | PitchL) / (float)32768.0 * (float)180.0 ;

	//航向角
	Angel.Yaw = (float)((int16_t)(YawH << 8) | YawL) / (float)32768.0 * (float)180.0 ;
}

void Uart1_Send_Data(uint8_t Tx_Num1)
{
	Tx_Data1[0] = 0x5A;	Tx_Data1[1] = 0xA5;	Tx_Data1[2] = 0x10;
	HAL_UART_Transmit_DMA(&huart1,Tx_Data1,Tx_Num1);	
	huart1.gState = HAL_UART_STATE_READY;	
}

void Uart2_Send_Data(uint8_t Tx_Num1)
{
	Tx_Data2[0] = 0x5A;	Tx_Data2[1] = 0xA5;	Tx_Data2[2] = 0x10;
	HAL_UART_Transmit_DMA(&huart2,Tx_Data2,Tx_Num1);
	huart2.gState = HAL_UART_STATE_READY;		
}
void Uart3_Send_Data(uint8_t Tx_Num1)
{
	Tx_Data3[0] = 0x5A;	Tx_Data3[1] = 0xA5;	Tx_Data3[2] = 0x10;
	HAL_UART_Transmit_DMA(&huart3,Tx_Data2,Tx_Num1);
	huart3.gState = HAL_UART_STATE_READY;		
}
void Uart4_Send_Data(uint8_t Tx_Num1)
{
	Tx_Data4[0] = 0x5A;	Tx_Data4[1] = 0xA5;	Tx_Data4[2] = 0x10;
	HAL_UART_Transmit_DMA(&huart4,Tx_Data2,Tx_Num1);
	huart4.gState = HAL_UART_STATE_READY;		
}
void Uart5_Send_Data(uint8_t Tx_Num1)
{
	Tx_Data5[0] = 0x5A;	Tx_Data5[1] = 0xA5;	Tx_Data5[2] = 0x10;
	HAL_UART_Transmit_DMA(&huart5,Tx_Data2,Tx_Num1);
	huart5.gState = HAL_UART_STATE_READY;		
}
void Uart6_Send_Data(uint8_t Tx_Num1)
{
	Tx_Data6[0] = 0x5A;	Tx_Data6[1] = 0xA5;	Tx_Data6[2] = 0x10;
	HAL_UART_Transmit_DMA(&huart6,Tx_Data2,Tx_Num1);
	huart6.gState = HAL_UART_STATE_READY;		
}


Remote Remote_data={0,0,100,100};

uint8_t sbusbuf[25];
uint8_t sbusbuf1[25];
uint16_t channeldate[16];												//这个里面就是解析之后通道的原始数据

#define Data 1
//解析遥控器数据
void Analysis()
{
	uint32_t t;  
	uint32_t pick;
	uint32_t piece;
	uint16_t tempdate;
	const struct sbus_bit_pick *decode;
	uint8_t datat_an = 1;
	
	if(datat_an == 1)// datat_an是自锁防止解析数据的时候传输新的参数
	{
		datat_an = 0;
		memcpy(sbusbuf1,sbusbuf,25);
		
		/*第一次解析数据*/
		for(t = 0; t < 16; t ++)
		{
			tempdate = 0;
			for(pick = 0; pick < 3; pick ++)
			{
				decode = &sbus_decoder[t][pick];
					
				if (decode->mask != 0) 
				{
					piece = sbusbuf1[1 + decode->byte];
					piece >>= decode->rshift;
					piece &= decode->mask;							//该字节取得位数
					piece <<= decode->lshift;

					tempdate |= piece;
				 }
			 }
			 channeldate[t] = tempdate;
		}
		datat_an = 1;
	}
	
#if Data == 0
	//原始数据
	Remote_data.A = channeldate[4];
	Remote_data.Left_X = channeldate[3];
	Remote_data.Right_X = channeldate[0];
#else	
	//转换百分比0-200
	if(Remote_data.Lost_sta == 1) //remote no lost
	{
		percentage(channeldate[4],channeldate[3],channeldate[0]);
	}
	else if(Remote_data.Lost_sta == 0)
	{
		Remote_data.A = 0;
		Remote_data.Left_X = 100;
		Remote_data.Right_X = 100;	
	}
#endif
	
}

void percentage(uint16_t a,uint16_t left,uint16_t right)
{
	if(a == 0x161)
	{
		Remote_data.A = 1;
	}
	else if(a == 0x69f)
	{
		Remote_data.A = 0;
	}
	
	if(left>=0x69f)left=0x69f;
	else if(left<=0x161)left=0x161;
	if(right>=0x69f)right=0x69f;
	else if(right<=0x161)right=0x161;
	
	Remote_data.Left_X = (float)(left - 0x161)/(0x69f - 0x161) * 200;
	Remote_data.Right_X = (float)(right - 0x161)/(0x69f - 0x161) * 200;
}
