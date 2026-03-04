#include "Yobotics.h"
#include "fdcan.h"
#include "stdio.h"
#include <math.h>
#include "usart.h"

u8 channel=1;
moto_measure_Yobotics Yobotics_moto[8];


FDCAN_TxHeaderTypeDef yobotics_tx_message;
static uint8_t  yobotics_can_send_data[8];
static uint8_t  USART_send_data[15];


#define CPU_FREQUENCY_MHZ    170		// STM32时钟主频
void delay_us(__IO uint32_t delay)
{
 int last, curr, val;
    int temp;

    while (delay != 0)
    {
        temp = delay > 900 ? 900 : delay;
        last = SysTick->VAL;
        curr = last - CPU_FREQUENCY_MHZ * temp;
        if (curr >= 0)
        {
            do
            {
                val = SysTick->VAL;
            }
            while ((val < last) && (val >= curr));
        }
        else
        {
            curr += CPU_FREQUENCY_MHZ * 1000;
            do
            {
                val = SysTick->VAL;
            }
            while ((val <= last) || (val > curr));
        }
        delay -= temp;
    }

}
#define delay_time 360

void SET_MOTOR_MODE(unsigned char channel,unsigned char Yobotics_ID){
	
	delay_us(delay_time);
	yobotics_tx_message.Identifier 	=	Yobotics_ID;
	yobotics_tx_message.IdType		=	FDCAN_STANDARD_ID;
	yobotics_tx_message.TxFrameType = 	FDCAN_DATA_FRAME;
	yobotics_tx_message.DataLength 	= 	0x08;
	yobotics_tx_message.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	yobotics_tx_message.BitRateSwitch = FDCAN_BRS_ON;
	yobotics_tx_message.FDFormat = FDCAN_FD_CAN;
	yobotics_tx_message.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	yobotics_tx_message.MessageMarker = 0;
	
	yobotics_can_send_data[0]=0xFF;
	yobotics_can_send_data[1]=0xFF;
	yobotics_can_send_data[2]=0xFF;
	yobotics_can_send_data[3]=0xFF;
	yobotics_can_send_data[4]=0xFF;
	yobotics_can_send_data[5]=0xFF;
	yobotics_can_send_data[6]=0xFF;
	yobotics_can_send_data[7]=0xFC;

	if(channel==1){
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &yobotics_tx_message, yobotics_can_send_data);
	}
	else if(channel==2)
	{
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &yobotics_tx_message, yobotics_can_send_data);

	}

}

void back_to_zero(unsigned char channel,unsigned char Yobotics_ID){
	
	delay_us(delay_time);
	yobotics_tx_message.Identifier 	=	Yobotics_ID;
	yobotics_tx_message.IdType		=	FDCAN_STANDARD_ID;
	yobotics_tx_message.TxFrameType = 	FDCAN_DATA_FRAME;
	yobotics_tx_message.DataLength 	= 	0x08;
	yobotics_tx_message.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	yobotics_tx_message.BitRateSwitch = FDCAN_BRS_ON;
	yobotics_tx_message.FDFormat = FDCAN_FD_CAN;
	yobotics_tx_message.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	yobotics_tx_message.MessageMarker = 0;
	
//	yobotics_can_send_data[0]=0x80;
//	yobotics_can_send_data[1]=0x00;
//	yobotics_can_send_data[2]=0x00;
//	yobotics_can_send_data[3]=0x00;
//	yobotics_can_send_data[4]=0x26;
//	yobotics_can_send_data[5]=0x00;
//	yobotics_can_send_data[6]=0x07;
//	yobotics_can_send_data[7]=0xFF;
		yobotics_can_send_data[0]=0x00;
	yobotics_can_send_data[1]=0x00;
	yobotics_can_send_data[2]=0x00;
	yobotics_can_send_data[3]=0x00;
	yobotics_can_send_data[4]=0x00;
	yobotics_can_send_data[5]=0x00;
	yobotics_can_send_data[6]=0x0F;
	yobotics_can_send_data[7]=0xFF;

	if(channel==1){
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &yobotics_tx_message, yobotics_can_send_data);
	}
	else if(channel==2)
	{
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &yobotics_tx_message, yobotics_can_send_data);

	}

}


void SET_CAN_ID(unsigned char Yobotics_Init_ID,unsigned char Yobotics_ID,unsigned char channel){

	yobotics_tx_message.Identifier 	=	Yobotics_Init_ID;
	yobotics_tx_message.IdType		=	FDCAN_STANDARD_ID;
	yobotics_tx_message.TxFrameType = 	FDCAN_DATA_FRAME;
	yobotics_tx_message.DataLength 	= 	0x08;
	yobotics_tx_message.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	yobotics_tx_message.BitRateSwitch = FDCAN_BRS_ON;
	yobotics_tx_message.FDFormat = FDCAN_FD_CAN;
	yobotics_tx_message.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	yobotics_tx_message.MessageMarker = 0;
	
	yobotics_can_send_data[0]=0xFF;
	yobotics_can_send_data[1]=0xFF;
	yobotics_can_send_data[2]=0xFF;
	yobotics_can_send_data[3]=0xFF;
	yobotics_can_send_data[4]=0xFF;
	yobotics_can_send_data[5]=0xFF;
	yobotics_can_send_data[6]=0xF0;
	yobotics_can_send_data[7]=Yobotics_ID;
	
	
	
	if(channel==1){
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &yobotics_tx_message, yobotics_can_send_data);
	}
	else if(channel==2)
	{
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &yobotics_tx_message, yobotics_can_send_data);

	}
	
}

void SET_RESET_MODE(unsigned char channel,unsigned char Yobotics_ID){
	
		yobotics_tx_message.Identifier 	=	Yobotics_ID;
	yobotics_tx_message.IdType		=	FDCAN_STANDARD_ID;
	yobotics_tx_message.TxFrameType = 	FDCAN_DATA_FRAME;
	yobotics_tx_message.DataLength 	= 	0x08;
	yobotics_tx_message.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	yobotics_tx_message.BitRateSwitch = FDCAN_BRS_ON;
	yobotics_tx_message.FDFormat = FDCAN_FD_CAN;
	yobotics_tx_message.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	yobotics_tx_message.MessageMarker = 0;
	
		yobotics_can_send_data[0]=0xFF;
	yobotics_can_send_data[1]=0xFF;
	yobotics_can_send_data[2]=0xFF;
	yobotics_can_send_data[3]=0xFF;
	yobotics_can_send_data[4]=0xFF;
	yobotics_can_send_data[5]=0xFF;
	yobotics_can_send_data[6]=0xFF;
	yobotics_can_send_data[7]=0xFD;
	
	if(channel==1){
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &yobotics_tx_message, yobotics_can_send_data);
	}
	else if(channel==2)
	{
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &yobotics_tx_message, yobotics_can_send_data);

	}
}


void SET_MOTOR_ZERO0(unsigned char channel,unsigned char Yobotics_ID){
	
	yobotics_tx_message.Identifier 	=	Yobotics_ID;
	yobotics_tx_message.IdType		=	FDCAN_STANDARD_ID;
	yobotics_tx_message.TxFrameType = 	FDCAN_DATA_FRAME;
	yobotics_tx_message.DataLength 	= 	0x08;
	yobotics_tx_message.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	yobotics_tx_message.BitRateSwitch = FDCAN_BRS_ON;
	yobotics_tx_message.FDFormat = FDCAN_FD_CAN;
	yobotics_tx_message.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	yobotics_tx_message.MessageMarker = 0;
	
	yobotics_can_send_data[0]=0xFF;
	yobotics_can_send_data[1]=0xFF;
	yobotics_can_send_data[2]=0xFF;
	yobotics_can_send_data[3]=0xFF;
	yobotics_can_send_data[4]=0xFF;
	yobotics_can_send_data[5]=0xFF;
	yobotics_can_send_data[6]=0xF1;
	yobotics_can_send_data[7]=0x00;
	
	if(channel==1){
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &yobotics_tx_message, yobotics_can_send_data);
	}
	else if(channel==2)
	{
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &yobotics_tx_message, yobotics_can_send_data);

	}
}


/*
	16bits:  0x0000~0xFFFF
	12bits:  0x000~0xFFF	
	扭矩：16位 -18NM~18NM       7fff   0
	kd	:	12位 0~100            7ff    0
	kp  : 12位 0~500            7ff    0
	veloctiy: 12位 -30~30       7ff    0
	position:  16位 -4pi~4pi    7fff   0
	
	电机驱动集成FOC控制，详细请看Yobotics说明书扩展.pdf
	iqref=1/Kt*tref
	tref=((position-theta)*Kp+torque+(velocity-d(theta))*Kd);
	Kt为转矩常数
	theta机械角度
	dtheta机械角速度
	tref为参考转矩
*/

static u16 Tor, Pos, Vec,Kp,Kd;
/**
  * @brief  CAN 报文打包与发送（FDCAN 1）
  * @param  p_des: 期望位置（目标角度）。
  * @param  v_des: 期望速度。
  * @param  kp, kd: 刚度/阻尼（PID参数）。
  * @param  t_ff: 力矩前馈。
  * @param  channel: FDCAN通道号 (1/2)。
  * @param  CANID: 电机通信ID。
  * @retval 无
  * @用法及调用要求  
  * 在 newIK() 解算出目标角度后被立即调用。函数内部对所有参数进行限幅和量化（float_to_uint），然后填充 yobotics_can_send_data 缓冲区，并通过 FDCAN1 发送。
  * @其它  
  * 负责与电机驱动器进行位置/力矩混合控制。
  */
void Cal_Yobotics_Data(float p_des,float v_des,float kp,float kd,float t_ff,unsigned char channel,unsigned char CANID){
     p_des = fminf(fmaxf(P_MIN, p_des), P_MAX);                    
     v_des = fminf(fmaxf(V_MIN, v_des), V_MAX);
     kp = fminf(fmaxf(Kp_MIN, kp), Kd_MAX);
     kd = fminf(fmaxf(Kd_MIN, kd), Kd_MAX);
     t_ff = fminf(fmaxf(T_MIN, t_ff), T_MAX);
     
		Pos = float_to_uint(p_des, P_MIN, P_MAX, 16);            
    Vec = float_to_uint(v_des, V_MIN, V_MAX, 12);
    Kp = float_to_uint(kp, Kd_MIN, Kp_MAX, 12);
    Kd = float_to_uint(kd, Kd_MIN, Kd_MAX, 12);
    Tor = float_to_uint(t_ff, T_MIN, T_MAX, 12);
		yobotics_cmd_gimbal(Tor,Kd,Kp,Vec,Pos,channel,CANID);
}


void yobotics_cmd_gimbal(u16 torque,u16 kd,u16 kp,u16 velocity,u16 position,unsigned char channel,unsigned char CANID)
{
	delay_us(delay_time);
	
	yobotics_tx_message.Identifier 	=	CANID;
	yobotics_tx_message.IdType		=	FDCAN_STANDARD_ID;
	yobotics_tx_message.TxFrameType = 	FDCAN_DATA_FRAME;
	yobotics_tx_message.DataLength 	= 	0x08;
	yobotics_tx_message.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	yobotics_tx_message.BitRateSwitch = FDCAN_BRS_ON;
	yobotics_tx_message.FDFormat = FDCAN_FD_CAN;
	yobotics_tx_message.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	yobotics_tx_message.MessageMarker = 0;

	yobotics_can_send_data[0]=position>>8; 
	yobotics_can_send_data[1] = position&0xFF;
	yobotics_can_send_data[2]=velocity>>4;  //用s16存一个12位数，取速度高8位
	yobotics_can_send_data[3]=((velocity&0xF)<<4)|(kp>>8);//速度低四位，kp高四位 
	yobotics_can_send_data[4]= (kp&0xFF);//kp低8位
	yobotics_can_send_data[5]=  kd>>4;//kd高8位
	yobotics_can_send_data[6]=((kd&0xF)<<4)|(torque>>8);//kd低4位，转矩高4位
	yobotics_can_send_data[7]=(torque&0xFF);//扭矩低8位
		
	if(channel==1)
	{
		HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &yobotics_tx_message, yobotics_can_send_data);
	}
	else if(channel==2)
	{
		HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &yobotics_tx_message, yobotics_can_send_data);

	}
}


float uint_to_float(int x_int,float x_min,float x_max,int bits){
	float span=x_max-x_min;
	float offset=x_min;
	return ((float)x_int)*span/((float)((1<<bits)-1))+offset;
}
int float_to_uint(float x,float x_min,float x_max,int bits){
	float span=x_max-x_min;
	float offset=x_min;
	return (int)((x-offset)*((float)(1<<bits)-1)/span);
}
/**
  * @brief  CAN 报文打包与发送（FDCAN 2）
  * @param  p_des: 期望位置（目标角度）。
  * @param  v_des: 期望速度。
  * @param  kp, kd: 刚度/阻尼（PID参数）。
  * @param  t_ff: 力矩前馈。
  * @param  channel: FDCAN通道号 (1/2)。
  * @param  CANID: 电机通信ID。
  * @retval 无
  * @用法及调用要求  
  * 功能与 Cal_Yobotics_Data 相同，但用于通过 FDCAN2 发送指令，可能控制另一组电机（如后腿）。
  * @其它  
  * 提供了双 CAN 总线的支持。
  */
void Cal_Yobotics_Data2(float p_des,float v_des,float kp,float kd,float t_ff,unsigned char channel,unsigned char CANID){
     HAL_Delay(1);
		 p_des = fminf(fmaxf(P_MIN, p_des), P_MAX);                    
     v_des = fminf(fmaxf(V_MIN, v_des), V_MAX);
     kp = fminf(fmaxf(Kp_MIN, kp), Kd_MAX);
     kd = fminf(fmaxf(Kd_MIN, kd), Kd_MAX);
     t_ff = fminf(fmaxf(T_MIN, t_ff), T_MAX);
     
		Pos = float_to_uint(p_des, P_MIN, P_MAX, 16);            
    Vec = float_to_uint(v_des, V_MIN, V_MAX, 12);
    Kp = float_to_uint(kp, Kd_MIN, Kp_MAX, 12);
    Kd = float_to_uint(kd, Kd_MIN, Kd_MAX, 12);
    Tor = float_to_uint(t_ff, T_MIN, T_MAX, 12);

		USART_send_data[0]=0xFC;
		USART_send_data[1]=(uint8_t)channel;
		USART_send_data[2]=(uint8_t)CANID;
		USART_send_data[3]=Pos>>8; 
		USART_send_data[4] = Pos&0xFF;
		USART_send_data[5]=Vec>>4;  //用s16存一个12位数，取速度高8位
		USART_send_data[6]=((Vec&0xF)<<4)|(Kp>>8);//速度低四位，kp高四位 
		USART_send_data[7]= (Kp&0xFF);//kp低8位
		USART_send_data[8]=  Kd>>4;//kd高8位
		USART_send_data[9]=((Kd&0xF)<<4)|(Tor>>8);//kd低4位，转矩高4位
		USART_send_data[10]=(Tor&0xFF);//扭矩低8位
		USART_send_data[11]=0xFD;
 		HAL_UART_Transmit_IT(&huart4,USART_send_data,12);//这里要修改的
} 