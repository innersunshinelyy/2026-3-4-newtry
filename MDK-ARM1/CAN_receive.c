#include "CAN_receive.h"
#include "main.h"
#include "usart.h"
#include "stdio.h"
#include "YoBotics.h"
#include "pid.h"
#include "bianliang.h"


extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;


moto_measure_Yobotics  motor[8]={0};
moto_measure_Yobotics  Yobotics[8]={0};

double laser_x;
double laser_y;
double laser_r;
extern RxText rx_text;
extern int flag_print;

extern double x_reset;
extern double y_reset;
extern double angle_reset;

/**
  * @brief          hal CAN fifo call back, receive motor data
  * @param[in]      hcan, the point to CAN handle
  * @retval         none
  */
/**
  * @brief          hal库CAN回调函数,接收电机数据
  * @param[in]      hcan:CAN句柄指针
  * @retval         none
  */


//void HAL_FDCAN_RxFifo0MsgPendingCallback(FDCAN_HandleTypeDef *hfdcan)
//{
//    FDCAN_RxHeaderTypeDef rx_header1,rx_header2;
//    uint8_t rx_data1[6],rx_data2[6];
//    HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rx_header1, rx_data1);
//		HAL_FDCAN_GetRxMessage(&hfdcan2, FDCAN_RX_FIFO0, &rx_header2, rx_data2);

//     switch (rx_data1[0])
//    {
//					case 1:
//					case 2:
//				{
//				uint16_t measure_angle=(rx_data1[1]<<8)|(rx_data1[2]);
//				int16_t measure_speed=((rx_data1[3]<<4)|((rx_data1[4]&0xF0)>>4))&(0x0FFF);
//				int16_t measure_current=(((rx_data1[4]&0xF)<<8)|(rx_data1[5]))&0x0FFF;
//				Yobotics[rx_data1[0]-1].angle=uint_to_float(measure_angle,-12.56f,12.56f,16);
//        Yobotics[rx_data1[0]-1].speed_rpm=uint_to_float(measure_speed,-30,30,12);
//        Yobotics[rx_data1[0]-1].real_current=uint_to_float(measure_current,0,40,12);
//				}
//				
//    }
//		switch (rx_data2[0])
//    {
//					case 1:
//					case 2:
//				{
//				uint16_t measure_angle=(rx_data2[1]<<8)|(rx_data2[2]);
//				int16_t measure_speed=((rx_data2[3]<<4)|((rx_data2[4]&0xF0)>>4))&(0x0FFF);
//				int16_t measure_current=(((rx_data2[4]&0xF)<<8)|(rx_data2[5]))&0x0FFF;
//				Yobotics[rx_data2[0]+1].angle=uint_to_float(measure_angle,-12.56f,12.56f,16);
//        Yobotics[rx_data2[0]+1].speed_rpm=uint_to_float(measure_speed,-30,30,12);
//        Yobotics[rx_data2[0]+1].real_current=uint_to_float(measure_current,0,40,12);
//				}
//				
//    }
//		
//}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan,uint32_t RxFifo1ITs)
{
	flag_print=1;
	if(hfdcan==&hfdcan3){
	flag_print=2;
//		printf("1111");
		FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[4];
		static uint32_t data_base;
    HAL_FDCAN_GetRxMessage(&hfdcan3, FDCAN_RX_FIFO1, &rx_header, rx_data);
		if(rx_header.Identifier==0x12)
		{
			  data_base =(uint32_t)rx_data[3]<<24|(uint64_t)rx_data[2]<<16|(uint64_t)rx_data[1]<<8|(uint64_t)rx_data[0];
				laser_x = (*(float*)(&data_base));
				rx_text.x = (float)laser_x;//由于全篇几乎都使用lcResult，难以更改，故将laser的值赋予lcResult
				rx_text.x=rx_text.x-x_reset;
		}
		else if(rx_header.Identifier==0x13)
		{
			  data_base =(uint32_t)rx_data[3]<<24|(uint64_t)rx_data[2]<<16|(uint64_t)rx_data[1]<<8|(uint64_t)rx_data[0];
				laser_y = (*(float*)(&data_base));
				rx_text.y = (float)laser_y;
				rx_text.y=rx_text.y-y_reset;
		}
		else if(rx_header.Identifier==0x14)
		{
			  data_base =(uint32_t)rx_data[3]<<24|(uint64_t)rx_data[2]<<16|(uint64_t)rx_data[1]<<8|(uint64_t)rx_data[0];
				laser_r = (*(float*)(&data_base));
				rx_text.yaw = (float)laser_r;//由于全篇几乎都使用lcResult，难以更改，故将laser的值赋予lcResult
				rx_text.yaw=rx_text.yaw-angle_reset;
				if(rx_text.yaw>=180.0){
					rx_text.yaw=rx_text.yaw-360.0;
				}
				else if(rx_text.yaw<=-180.0){
					rx_text.yaw=rx_text.yaw+360.0;
				}
		}
//    FDCAN_RxHeaderTypeDef rx_header;
//    uint8_t rx_data[8];
//		static uint64_t data_base;
//    HAL_FDCAN_GetRxMessage(&hfdcan3, FDCAN_RX_FIFO1, &rx_header, rx_data);
//		if(rx_header.Identifier==0x12)
//		{
//			  data_base = (uint64_t)rx_data[7]<<56|(uint64_t)rx_data[6]<<48|(uint64_t)rx_data[5]<<40|(uint64_t)rx_data[4]<<32|(uint64_t)rx_data[3]<<24|(uint64_t)rx_data[2]<<16|(uint64_t)rx_data[1]<<8|(uint64_t)rx_data[0];
//				laser_x = (*(double*)(&data_base));
//				rx_text.x = (float)laser_x;//由于全篇几乎都使用lcResult，难以更改，故将laser的值赋予lcResult
//		}
//		else if(rx_header.Identifier==0x13)
//		{
//			  data_base = (uint64_t)rx_data[7]<<56|(uint64_t)rx_data[6]<<48|(uint64_t)rx_data[5]<<40|(uint64_t)rx_data[4]<<32|(uint64_t)rx_data[3]<<24|(uint64_t)rx_data[2]<<16|(uint64_t)rx_data[1]<<8|(uint64_t)rx_data[0];
//				laser_y = (*(double*)(&data_base));
//				rx_text.y = (float)laser_y;
//		}
//		else if(rx_header.Identifier==0x14)
//		{
//			  data_base = (uint64_t)rx_data[7]<<56|(uint64_t)rx_data[6]<<48|(uint64_t)rx_data[5]<<40|(uint64_t)rx_data[4]<<32|(uint64_t)rx_data[3]<<24|(uint64_t)rx_data[2]<<16|(uint64_t)rx_data[1]<<8|(uint64_t)rx_data[0];
//				laser_r = (*(double*)(&data_base));
//				rx_text.yaw = (float)laser_r;//由于全篇几乎都使用lcResult，难以更改，故将laser的值赋予lcResult

//		}
		
	}
}