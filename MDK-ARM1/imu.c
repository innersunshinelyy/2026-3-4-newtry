#include "imu.h"
#include "stdio.h"
#include "string.h"
#include "usart.h"
#define DMA_BUF_SIZE (IMU_FRAME_LEN * 2)
uint8_t imu_rx_buffer[IMU_FRAME_LEN* 2]; // 临时接收缓冲区
uint8_t imu_rx_cnt;               // 接收计数器
uint8_t imu_state;                // 状态机状态
IMU_Data_t g_imu_data;
uint8_t imu_dma_buffer[IMU_FRAME_LEN * 2]; // DMA 接收缓冲区，大小设为帧长的2倍
volatile uint8_t imu_new_data_flag = 0;             // 数据更新标志位

/**
 * @brief 初始化 IMU 接收 (DMA + IDLE)
 */
void IMU_Init(void) {
    // 【修改3】长度使用 DMA_BUF_SIZE
    HAL_UART_Receive_DMA(&huart2, imu_dma_buffer, DMA_BUF_SIZE);
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
}

void IMU_DMA_Handler(UART_HandleTypeDef *huart) {
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) != RESET) {
        __HAL_UART_CLEAR_IDLEFLAG(huart);

        static uint32_t old_pos = 0;
        // 计算当前位置
        uint32_t pos = DMA_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart->hdmarx);

        // 处理数据
        while (old_pos != pos) {
            // 现在 old_pos 永远不会超过 DMA_BUF_SIZE，安全了
            IMU_Process_Byte(imu_dma_buffer[old_pos]);
            old_pos++;
            if (old_pos >= DMA_BUF_SIZE) {
                old_pos = 0;
            }
        }
    }
}

/**
 * @brief 保持你原有的状态机逻辑，但移除 printf
 */
void IMU_Process_Byte(uint8_t byte) {
    switch (imu_state) {
        case 0:
            if (byte == IMU_HEADER_1) {
                imu_rx_cnt = 0;
                imu_rx_buffer[imu_rx_cnt++] = byte;
                imu_state = 1;
            }
            break;
        case 1:
            if (byte == IMU_HEADER_2) {
                imu_rx_buffer[imu_rx_cnt++] = byte;
                imu_state = 2;
            } else imu_state = 0;
            break;
        case 2:
            if (byte == IMU_HEADER_3) {
                imu_rx_buffer[imu_rx_cnt++] = byte;
                imu_state = 3;
            } else imu_state = 0;
            break;
        case 3:
            if (byte == IMU_HEADER_4) {
                imu_rx_buffer[imu_rx_cnt++] = byte;
                imu_state = 4;
            } else imu_state = 0;
            break;
        case 4:
            imu_rx_buffer[imu_rx_cnt++] = byte;
            if (imu_rx_cnt >= IMU_FRAME_LEN) {
                // 解析 Yaw
                memcpy(&g_imu_data.yaw, &imu_rx_buffer[19], 4);
								gyro_angle = -g_imu_data.yaw;
                // 设置标志位，不在中断里 printf
                imu_new_data_flag = 1; 
                imu_state = 0;
                imu_rx_cnt = 0;
            }
            break;
        default:
            imu_state = 0;
            break;
    }
}