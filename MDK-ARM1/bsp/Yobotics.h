#ifndef __YOBOTICS_H
#define __YOBOTICS_H

#include "stm32G4xx_hal.h"
#define P_MIN -12.56f
#define P_MAX 12.56f
#define V_MIN -30
#define V_MAX 30
#define I_Max 40
#define T_MAX 18
#define T_MIN -18
#define Kp_MAX 500
#define Kp_MIN 0
#define Kd_MAX 100
#define Kd_MIN 0
typedef uint16_t u16;
typedef uint8_t  u8;

typedef struct
{ 
	float	 		speed_rpm;   //存放电机返回的数据 
  float  		real_current;
	float 		angle;	
	float 		given_angel; //存放给定值
	float			given_speed;
	float     given_torque;
	
	
  int16_t  	given_current;
	uint16_t 	last_angle;	
	int64_t		total_angle;
	uint8_t		temperature;
  uint16_t	initial_angle;
	uint16_t  first  ; //用于判断是否为第一次发送信号
}moto_measure_Yobotics;

extern u8 CAN_ID_NUM ;
extern u8 channel;

extern union fi64_to_u8 YoboticsMotor_Tx;
extern union fi64_to_u8 YoboticsMotor_Rx;


int float_to_uint(float x,float x_min,float x_max,int bits);
float uint_to_float(int x_int,float x_min,float x_max,int bits);
void SET_CAN_ID(unsigned char Yobotics_Init_ID,unsigned char CAN_ID_NUM,unsigned char channel);
void SET_RESET_MODE(unsigned char channel,unsigned char CAN_ID_NUM);
void SET_MOTOR_MODE(unsigned char channel,unsigned char Yobotics_Init_ID);
void SET_MOTOR_ZERO0(unsigned char channel,unsigned char CAN_ID_NUM);
void Yobotics_Handler(void);
int float_to_uint(float x,float x_min,float x_max,int bits);
float uint_to_float(int x_int,float x_min,float x_max,int bits);

void yobotics_cmd_gimbal(u16 torque,u16 kd,u16 kp,u16 velocity,u16 position,unsigned char channel,unsigned char CANID);
void Cal_Yobotics_Data(float p_des,float v_des,float kp,float kd,float t_ff,unsigned char channel,unsigned char CANID);
void Cal_Yobotics_Data2(float p_des,float v_des,float kp,float kd,float t_ff,unsigned char channel,unsigned char CANID);
void delay_us(__IO uint32_t delay);


void back_to_zero(unsigned char channel,unsigned char Yobotics_ID);

#endif

