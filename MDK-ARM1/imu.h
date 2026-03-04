#ifndef IMU_H
#define IMU_H

#include "stm32g4xx_hal.h" // 根据你的芯片型号修改，如 stm32f4xx_hal.h
#include <string.h>
#include "gyro.h"

// 协议常量定义 
#define IMU_FRAME_LEN    66      // 一帧数据的总长度
#define IMU_HEADER_1     0xAB
#define IMU_HEADER_2     0x54
#define IMU_HEADER_3     0x65
#define IMU_HEADER_4     0x00

typedef struct {
    float yaw;          // 航向角
    float pitch;        // 俯仰角 (可选)
    float roll;         // 横滚角 (可选)
    uint8_t nav_status; // 导航状态
} IMU_Data_t;

extern IMU_Data_t g_imu_data;  // 全局变量存储解析后的数据
extern uint8_t rx_byte;
// 接收相关变量
extern uint8_t imu_rx_buffer[IMU_FRAME_LEN* 2]; // 临时接收缓冲区
extern uint8_t imu_rx_cnt;               // 接收计数器
extern uint8_t imu_state;                // 状态机状态
extern volatile uint8_t imu_new_data_flag; 
void IMU_Process_Byte(uint8_t byte);
void IMU_DMA_Handler(UART_HandleTypeDef *huart);
void IMU_Init(void);
#endif 