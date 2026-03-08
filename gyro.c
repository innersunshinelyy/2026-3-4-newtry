#include "gyro.h"


volatile uint8_t gyro_buffer[33];  //陀螺仪数据接收
uint8_t if_gyro_right=0;

double gyro_angle=0;
volatile float angle_now;
volatile float angle_last=0;
volatile float dealt_angle;
float q_angle[4]={1,0,0,0};     //旋转四元数
float Gyro_v[3]={0,0,0};        //分别对应绕x y z旋转的角速度
float Gyro_a[3]={0,0,0};        //分别对应沿x轴 y轴 z轴方向的加速度
float gyro_dealt_t=0;	        //用于存间隔时间
float gyro_start_time=0;		//用于记录初始时间，解决零漂
float gyrotime=0;	        	//记录陀螺仪时间戳
float gyro_lasttime=0;			//记录上次获取陀螺仪的时间戳
int if_gyro_start_first=1;		//初始时间用到的标志位
int32_t gyro_vel[3]={0,0,0};   //用于接收角速度数据
int32_t gyro_a[3]={0,0,0};     //用于接收轴向加速度数据
float accex=0;					//用于存储误差积分值 
float accey=0;
float accez=0;
int if_gyro_err=0;


/**
  * @brief  陀螺仪数据获取与航向角（Yaw）积分更新
  * @param  无
  * @retval 无（通过更新全局变量 gyro_angle 输出结果）
  * @用法及调用要求  
  * 1. 依赖外部的陀螺仪数据接收（如UART/SPI/CAN中断）来更新 Gyro_v 数组。
  * 2. 在主控制循环或高频定时器中周期性调用，以保证时间积分的精确性。
  * @其它  
  * 使用时间戳 gyrotime 和 gyro_lasttime 计算时间间隔 gyro_dealt_t，并对 Gyro_v[2]（Z轴角速度）进行积分，得到航向角 gyro_angle。包含 ±180° 归一化逻辑。
  */
void get_gyro_data()
{
	gyro_vel[0]=(int32_t)(gyro_buffer[5]<<24) | (gyro_buffer[4]<<16)|(gyro_buffer[3]<<8)|gyro_buffer[2];
	gyro_vel[1]=(int32_t)(gyro_buffer[9]<<24) | (gyro_buffer[8]<<16)|(gyro_buffer[7]<<8)|gyro_buffer[6];
	gyro_vel[2]=(int32_t)(gyro_buffer[13]<<24) | (gyro_buffer[12]<<16)|(gyro_buffer[11]<<8)|gyro_buffer[10];
	gyro_a[0]=(int32_t)(gyro_buffer[17]<<24) | (gyro_buffer[16]<<16)|(gyro_buffer[15]<<8)|gyro_buffer[14];
	gyro_a[1]=(int32_t)(gyro_buffer[21]<<24) | (gyro_buffer[20]<<16)|(gyro_buffer[19]<<8)|gyro_buffer[18];
	gyro_a[2]=(int32_t)(gyro_buffer[25]<<24) | (gyro_buffer[24]<<16)|(gyro_buffer[23]<<8)|gyro_buffer[22];
	for(int i=0;i<3;i++)
	{
		Gyro_v[i]=(*((float *)(&gyro_vel[i])))*0.01745329252;  //转化为弧度
		Gyro_a[i]=*((float *)(&gyro_a[i]));
		//Gyro_v[i]=(*((float *)(&gyro_vel[i])));
	}
	Gyro_a[0]*=-1;
	Gyro_a[2]*=-1;
	Gyro_v[0]*=-1;
	Gyro_v[2]*=-1;
	#if 1
	 if(Gyro_v[2]>4.98||Gyro_v[2]<-4.98)
	{
		if_gyro_err=1;	
	}
	#endif
}

//求平方根倒数函数
static float invSqrt(float x)
{
	float halfx=0.5f*x;
	float y=x;
	long  i=*(long*)&y;
	i=0x5f3759df-(i>>1);
	y=*(float*)&i;
	y=y*(1.5f-(halfx*y*y));
	return y;
}

//利用加速度进行陀螺仪误差修正    输入时间差；
void gyro_err_clear(float delat_t )
{
	//加速度归一化

	float xishu=invSqrt(Gyro_a[0]*Gyro_a[0]+Gyro_a[1]*Gyro_a[1]+Gyro_a[2]*Gyro_a[2]);
	Gyro_a[0]*=xishu;
	Gyro_a[1]*=xishu;
	Gyro_a[2]*=xishu;
	//提取姿态矩阵中的重力分量
	float Vx=2*(q_angle[1]*q_angle[3]-q_angle[0]*q_angle[2]);
	float Vy=2*(q_angle[1]*q_angle[0]+q_angle[3]*q_angle[2]);
	float Vz=1-2*(q_angle[1]*q_angle[1]+q_angle[2]*q_angle[2]);
	//USART_printf(" q0=%f\n",q_angle[0]);
	//求出姿态误差
	float ex=Gyro_a[1]*Vz-Gyro_a[2]*Vy;
	float ey=Gyro_a[2]*Vx-Gyro_a[0]*Vz;
	float ez=Gyro_a[0]*Vy-Gyro_a[1]*Vx;
	//USART_printf(" %f\n",ez);
	//误差积分
	float ki=0.001;
	float kp=0.01;

	accex+=ex*ki*delat_t;
	accey+=ey*ki*delat_t;
	accez+=ez*ki*delat_t;
	
	//角速度修正
	Gyro_v[0]+=kp*ex+accex;
	Gyro_v[1]+=kp*ey+accey;
	Gyro_v[2]+=kp*ez+accez;
	//USART_printf(" v2=%f\n",Gyro_v[2]);
	
}

//四元数数据更新，输入间隔时间；
void q_angle_update(float delat_t ) 
{
	q_angle[0]=q_angle[0]+0.5*delat_t*(-1*Gyro_v[0]*q_angle[1]-Gyro_v[1]*q_angle[2]-Gyro_v[2]*q_angle[3]);
	q_angle[1]=q_angle[1]+0.5*delat_t*( 1*Gyro_v[0]*q_angle[0]-Gyro_v[1]*q_angle[3]+Gyro_v[2]*q_angle[2]);
	q_angle[2]=q_angle[2]+0.5*delat_t*( 1*Gyro_v[0]*q_angle[3]+Gyro_v[1]*q_angle[0]-Gyro_v[2]*q_angle[1]);
	q_angle[3]=q_angle[3]+0.5*delat_t*(-1*Gyro_v[0]*q_angle[2]+Gyro_v[1]*q_angle[1]+Gyro_v[2]*q_angle[0]);
	
	//四元数归一化
	float xishu=invSqrt(q_angle[0]*q_angle[0]+q_angle[1]*q_angle[1]+q_angle[2]*q_angle[2]+q_angle[3]*q_angle[3]);//分母
	for(int i=0;i<=3;i++)
	{
		q_angle[i]*=xishu;//最终归一化后的的四元数
	}
}

//陀螺仪角度的反解
float get_gyro_angle(int i)
{
	float g1=0,g2=0,g3=0,g4=0,g5=0;
  float angle[3]={0};
 
	g1=2*(q_angle[1]*q_angle[3]-q_angle[0]*q_angle[2]);
	g2=2*(q_angle[1]*q_angle[0]+q_angle[3]*q_angle[2]);
	g3=   q_angle[0]*q_angle[0]-q_angle[1]*q_angle[1]-q_angle[2]*q_angle[2]+q_angle[3]*q_angle[3];
	g4=2*(q_angle[1]*q_angle[2]+q_angle[3]*q_angle[0]);
	g5=   q_angle[0]*q_angle[0]+q_angle[1]*q_angle[1]-q_angle[2]*q_angle[2]-q_angle[3]*q_angle[3];
	//角度值分别对应俯仰角、翻滚角、偏航角；
	angle[0]=-1*asinf(g1);
	angle[1]=atanf(g2/g3);
	angle[2]=atan2f(g4,g5);
	//USART_printf("g5=%f\n",g4/g5);
	return angle[i];
}

void gyrodata_update()
{
	//USART_printf("get");
	get_gyro_data();
	gyrotime=HAL_GetTick();
	if(if_gyro_start_first)  //获取初始时刻
	{
		gyro_start_time=gyrotime;
		if_gyro_start_first=0;
	}
	gyro_dealt_t=(gyrotime-gyro_lasttime)*0.001;
	gyro_err_clear(gyro_dealt_t);
	q_angle_update(gyro_dealt_t);
	angle_now=get_gyro_angle(2);
	//gyro_angle-=0.000078*(gyrotime-gyro_start_time)*0.001; //0.000832 
	angle_now-=-0.000018008*(gyrotime-gyro_start_time)*0.001; //0.000832 //将结构静止不动，选择参数用时间的一次函数消除0漂
	//angle_now-=(gyrotime-gyro_start_time)*0.001;
	//USART_printf("angz %f \n",Gyroz);
	//Gyroz=Gyroz+0.2056-(gyrotime-gyro_lasttime)*0.001*0.0031;
	#if 1
	dealt_angle=angle_now-angle_last;
	angle_last=angle_now;
	if(dealt_angle>100||dealt_angle<-100) dealt_angle=0;//感觉没意义？已经突变了，再置零反而丢失了初始方向的定义
	dealt_angle=dealt_angle*180/pi;//化变化角度为弧度
	gyro_angle+=dealt_angle;//增量式计算当前角度：每次增加变化的角度
	if (gyro_angle >= 180)
	{
			gyro_angle -= 2*180;
	}//转向突变
	else if (gyro_angle <= -180)
	{
			gyro_angle += 2*180;
	}
	
//	printf("now_angle%f\n",angle_now);
//	printf("dealt_anglr=%f\n",dealt_angle);
//	   printf("gyro_angle=%.4f",gyro_angle);

	#endif
	#if 0
		if (gyro_angle >= 180)
	{
			gyro_angle -= 2*180;
	}
	else if (gyro_angle <= -180)
	{
			gyro_angle += 2*180;
	}
	#endif
	gyro_lasttime=gyrotime;
	if_gyro_right=0;	//？不理解
	//y = -0.0434x + 0.0031  角度制下零漂	
}
#ifdef jiaozhun
int jiaozhun_flag=0;
int start_flag_jiaozhun=1;
int now_out_time=0;
int last_out_time=0;
float gyro_angle_jiaozhun_start=0;
int gyro_start_time_jiaozhun=0;
float gyro_angle_jiaozhun=0;
int cnt=0;
double floating=0;
/**
  * @brief  陀螺仪零漂校准（静止校准）
  * @param  无
  * @retval 无
  * @用法及调用要求  
  * 1. 仅在宏定义 jiaozhun 启用时编译。
  * 2. 在程序启动时或机器人静止时被调用，用于计算陀螺仪的零偏或漂移，以补偿积分误差。
  * @其它  
  * 使用标志位 start_flag_jiaozhun 记录开始时间，并在一段时间内累积角度变化，计算出浮动零漂 floating。
  */
void go_jiaozhun(void)
{
	now_out_time=HAL_GetTick();
	//静止校准零漂
		if(jiaozhun_flag==1)
		{
			
			if(start_flag_jiaozhun)
			{
				gyro_angle_jiaozhun_start=gyro_angle;
				start_flag_jiaozhun=0;
				gyro_start_time_jiaozhun=HAL_GetTick();
			}
			gyro_angle_jiaozhun=gyro_angle-gyro_angle_jiaozhun_start;
			cnt++;
			if(now_out_time-last_out_time>400){
					printf("gyro_angle=%f\n",gyro_angle_jiaozhun);
				printf("cnt=%d\n",cnt);
				last_out_time=now_out_time;
			}
			

			
			//floating_total+=gyro_angle_jiaozhun;
			//调整校准零漂时间
			if(cnt>floating_numbers&&jiaozhun_flag==1)
			{
				int gyro_end_time_jiaozhun=HAL_GetTick();
				floating=gyro_angle_jiaozhun/((gyro_end_time_jiaozhun-gyro_start_time_jiaozhun)*0.001);
				printf("floating=%f\n",floating);
				printf("time=%d\n",gyro_end_time_jiaozhun-gyro_start_time_jiaozhun);
				printf("floating=%f\n",floating);
				printf("floating=%f\n",floating);
				jiaozhun_flag=0;
				printf("end floating jiaozhun!\n");
			}
		}
}
#endif
