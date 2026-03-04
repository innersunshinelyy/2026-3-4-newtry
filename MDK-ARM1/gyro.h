#include "main.h"
#include "fdcan.h"
#include "usart.h"
#include "gpio.h"
#include "string.h"
#include "stdlib.h"
#include "stdio.h"
#include "math.h"
//#define jiaozhun
#define floating_numbers 200

extern volatile uint8_t gyro_buffer[33];  //陀螺仪数据接收
extern uint8_t if_gyro_right;
extern double gyro_angle;
void get_gyro_data();
static float invSqrt(float x);
void gyro_err_clear(float delat_t );
void q_angle_update(float delat_t );
float get_gyro_angle(int i);
void gyrodata_update();
#ifdef jiaozhun
extern int jiaozhun_flag;
extern int start_flag_jiaozhun;
extern int jiaozhun_flag;
extern int start_flag_jiaozhun;
extern int now_out_time;
extern int last_out_time;
extern float gyro_angle_jiaozhun_start;
extern int gyro_start_time_jiaozhun;
extern float gyro_angle_jiaozhun;
extern int cnt;
extern double floating;
void go_jiaozhun(void);
#endif