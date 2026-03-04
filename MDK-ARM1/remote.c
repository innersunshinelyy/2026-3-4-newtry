#include "remote.h"
#include "string.h"
#include "usart.h"
#define max_spedx 1000; //毫米每秒
#define max_spedy 1000; //毫米每秒
#define max_spedx_zuo 1000; //毫米每秒
#define max_spedy_zuo 1000; //毫米每秒
#define max_cir (5); //弧度每秒.用7之后转弯很快

extern int trotDirectionState;
extern uint8_t RemoteBuffer_s[100];
extern int sw1_last_state;
rc_info_t rc;


uint8_t Remote_Frame_Error = 0;
uint8_t Remote_Send_Error = 0;
uint16_t Frame_Bias = 0;

int bounceball_flag=0;
volatile int remote_failed_time;
volatile int waittime=0;

extern int iffirst;
extern int temp;

double remote_spedx, remote_spedy, remote_cir=0;  //右摇杆速度
double remote_spedx_zuo, remote_spedy_zuo;//左摇杆速度


/**
  * @brief  遥控器数据更新与速度指令映射
  * @param  Remote_RawData: 指向遥控器（如SBUS）原始字节数据缓冲区。
  * @retval 无（通过修改全局变量 remote_spedx, remote_spedy, remote_cir 输出结果）
  * @用法及调用要求  
  * 1. 在主循环中周期性调用，以更新机器人的期望速度。
  * 2. 将 rc.ch1, rc.ch2, rc.ch4 通道值归一化到 [-1, 1] 范围，再乘以最大速度宏定义（max_spedx/y/cir），得到物理量速度。
  * @其它  
  * remote_spedx/y（前后/侧移速度）和 remote_cir（转向角速度）是运动控制系统的主要输入。
  */
void Remote_DataUpdate(uint8_t *Remote_RawData)
{
//	printf("%s\n",Remote_temp);
	static uint32_t data_base;
	uint8_t remote_check_bit=0;
	//step1 检验帧头 目前帧头数据待定
	if(Remote_RawData[0] != 'a')
	{
		memset(&rc, 0, sizeof(rc));
		remote_spedx =0;
		remote_spedy = 0;
		remote_cir = 0;
		return;
	}
	data_base = (Remote_RawData[1] - '0') * 1000 + (Remote_RawData[2] - '0') * 100 + (Remote_RawData[3] - '0') * 10 + (Remote_RawData[4] - '0');
	rc.ch1 = 660 - (*((int *)(&data_base)));//前推为正
	data_base = (Remote_RawData[5] - '0') * 1000 + (Remote_RawData[6] - '0') * 100 + (Remote_RawData[7] - '0') * 10 + (Remote_RawData[8] - '0');
	rc.ch2 = *((int *)(&data_base))-660;//左推为正
	data_base = (Remote_RawData[9] - '0') * 1000 + (Remote_RawData[10] - '0') * 100 + (Remote_RawData[11] - '0') * 10 + (Remote_RawData[12] - '0');
	rc.cir = 660 -(*((int *)(&data_base)));//顺时针拨动为正
	data_base = (Remote_RawData[13] - '0');
	rc.sw1 = *((int *)(&data_base));//第一个拨杆，上中下为123
	data_base = (Remote_RawData[14] - '0');
	rc.sw2 = *((int *)(&data_base));//第二个拨杆，上下为13
	data_base = (Remote_RawData[15] - '0');
	rc.sw3 = *((int *)(&data_base));//第三个拨杆，左中右为123
	data_base = (Remote_RawData[18] - '0');
	rc.button1 = *((int *)(&data_base));//按键，常态为0，按下瞬间发1
	data_base = (Remote_RawData[19] - '0');
	rc.button2 = *((int *)(&data_base));//按键，常态为0，按下瞬间发1
	data_base = (Remote_RawData[20] - '0');
	rc.button3 = *((int *)(&data_base));//按键，常态为0，按下瞬间发1
	data_base = (Remote_RawData[21] - '0');
	rc.button4 = *((int *)(&data_base));//按键，常态为0，按下瞬间发1
	data_base = (Remote_RawData[22] - '0');
	rc.button5 = *((int *)(&data_base));//按键，常态为0，按下瞬间发1
	data_base = (Remote_RawData[23] - '0');
	rc.button6 = *((int *)(&data_base));//按键，常态为0，按下瞬间发1
	for(int i=1;i<=23;i++)
	{
		remote_check_bit += Remote_RawData[i];
	}

	//printf("good\n");
	memset(&rc, 0, sizeof(rc));
//	memset(Remote_temp,0,sizeof(Remote_temp));
}





void get_remote_sped(void)
{
      remote_spedx = rc.ch1;
        remote_spedy = rc.ch2;
      
        remote_cir = rc.cir;
        remote_spedx/=660;
        remote_spedy/=660;
        remote_cir/=660;
      remote_spedx*=max_spedx;
      remote_spedy*=max_spedy;
      remote_cir*=max_cir;
    
}  
/**
  * @brief  遥控器原始数据解包
  * @param  rc: 遥控器信息结构体指针，用于存储解码后的通道值。
  * @param  buff: UART 接收到的原始8字节遥控数据。
  * @retval 无
  * @用法及调用要求  
  * 必须在 UART 接收完成的中断回调函数中被调用。采用位操作解析11位精度的通道数据。
  * @其它  
  * rc->ch1-ch4 经过中心点归零处理 ( -= 1024 )。rc->sw1/sw2 存储拨动开关状态，用于模式切换。
  */
void rc_callback_handler(rc_info_t *rc, uint8_t *buff)
{
  rc->ch1 = (buff[0] | buff[1] << 8) & 0x07FF;
  rc->ch1 -= 1024;
  rc->ch2 = (buff[1] >> 3 | buff[2] << 5) & 0x07FF;
  rc->ch2 -= 1024;
  rc->ch3 = (buff[2] >> 6 | buff[3] << 2 | buff[4] << 10) & 0x07FF;
  rc->ch3 -= 1024;
  rc->ch4 = (buff[4] >> 1 | buff[5] << 7) & 0x07FF;
  rc->ch4 -= 1024;

  rc->sw1 = ((buff[5] >> 4) & 0x000C) >> 2;
  rc->sw2 = (buff[5] >> 4) & 0x0003;
    rc->cir = ((((uint16_t)buff[17]& 0x0007 )<<8) | ( (int16_t)buff[16] & 0x00FF ))-1024;
  
  if ((abs(rc->ch1) > 660) || \
      (abs(rc->ch2) > 660) || \
      (abs(rc->ch3) > 660) || \
      (abs(rc->ch4) > 660)||
                rc->sw1>3||
                rc->sw2>3
  )
  {
    memset(rc, 0, sizeof(rc_info_t));
        waittime++;
        remote_failed_time=HAL_GetTick();
  }
    else
    {
        waittime=0;
    }
   get_remote_sped();
}
/*串口空闲中断用于接收来自遥控器的信息*/
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if(huart == &huart5)
    {
        rc_callback_handler(&rc,RemoteBuffer_s);
//				printf("1111");							if(rc.sw1==1)

        HAL_UARTEx_ReceiveToIdle_DMA(&huart5,RemoteBuffer_s,100);
				__HAL_DMA_DISABLE_IT(huart5.hdmarx,DMA_IT_HT);
    }
}
