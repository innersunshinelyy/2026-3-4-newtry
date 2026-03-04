#include "bound.h"
#include "newIK.h"
#include "newTrot.h"
#include "math.h"
#include "main.h"
#include "gyro.h"
#include "IMU.h"
#include "bianliang.h"
#include "remote.h"

//******************************0.变量声明***************************************//
//*******a.足端轨迹与机身姿态********//
extern struct canshu M_leg[5]; // 【核心结构体】存储每条腿的逆运动学(IK)目标坐标 (expx, expz等)。索引1-4对应四条腿。
double walk_CG[5];             // 预留变量，通常用于计算行走的重心(Center of Gravity)偏移量。

double xpre = -0.04;           // 【步态参数】足端轨迹的后摆极限点 (后脚跟位置，单位:米)。
double xfinal = 0.02;          // 【步态参数】足端轨迹的前摆极限点 (前脚尖位置，单位:米)。步幅 = xfinal - xpre。
double zHeight = 0.01;         // 【步态参数】抬腿高度 (单位:米)。摆动相时腿抬多高。
double heightControlNow = 0.26;// 【姿态参数】机身当前的离地高度 (单位:米)。
double M_T = 1.5;              // 【步态参数】步态周期 (单位:秒)。值越小走得越快，越大越慢。

double heightControlNow_L = 0.25;  // 左侧机身高度 (用于斜坡行走 ramp_trot)。
double heightControlNow_R = 0.196; // 右侧机身高度 (用于斜坡行走)。
double zHeight_L;              // 左侧抬腿高度 (用于斜坡)。
double zHeight_R;              // 右侧抬腿高度 (用于斜坡)。

float leg_Mode[5];             // 【关键变量】各腿步幅缩放系数。
                               // 1.0=正常直行; >1.0=增大步幅; <0=反向; 
                               // 左右腿设置不同值可实现差速转向。
//***********************************//
//********b.电机 PD 控制参数*********//
float __kp[5];                 // 当前使用的位置增益 (Stiffness/刚度)。
float __kd[5];                 // 当前使用的速度增益 (Damping/阻尼)。

// 预设的PD参数组 (可能对应特定高爆发动作如跳跃或特殊地形)
float __kpp[5] = {1, 65, 100, 65, 75}; // 预设Kp数组。
float __kdd[5] = {1, 7, 20, 19, 6};    // 预设Kd数组。

double TrotKpSwing, TrotKdSwing; // 摆动相(腿在空中)时的Kp, Kd (通常较软，便于跟随轨迹)。
double TrotKpStand, TrotKdStand; // 支撑相(腿在着地)时的Kp, Kd (通常较硬，用于支撑机身)。

float kd_s = 25;               // 特定状态下的Kd初始值 (如竞速模式下的动态阻尼调整)。
float detla_kp1 = 0, detla_kp2 = 0; // 用于动态微调Kp的增量变量。
//***********************************//
//**********c.雷达与定位*************//
extern RxText rx_text;         // 【传感器数据】接收到的定位数据结构体 (包含 x, y, yaw/航向角)。
int places_have_reached = 0;   // 导航计数器，表示当前已经到达了第几个路径点。
path pathArray[16] = {//比赛右边赛道
    {3.77, -10.60, 0.0, 0.0, 39, 0.0},   // 低姿匍匐必达区1 (0)
    {4.88, -10.81, 0.0, 0.0, 39, 0.0},  
    {5.46, -11.37, 0.0, 0.0, 39, 0.0},  
    {6.59, -11.35, 0.0, 0.0, 39, 0.0},   
    {11.07-0.02, -10.90, 0.0, 0.0, 2, 1.24},   // 高栏必达区1(4)
    {15.9, -10.95, 0.0, 0.0, 19, 2.13},   // 断桥必达区1(5)
    {17.33, -10.95, 0.0, 0.0, 19, 2.55},   // 断桥第2个点(6)
    {22.86, -11.04, 0.0, 0.0, 43, 2.71},   // 沙坑必达区1(7)
    {29.19, -10.96, 0.0, 0.0, 57, 0.2},   // 斜坡必达区1(8)
    {34.27, -11.05, 0.0, 0.0, 39, 0.0},  // 绕杆必达区1(9)
    {34.79, -11.87, 0.0, 0.0, 39, 0.0}, // 
    {36.06, -10.40, 0.0, 0.0, 39, 0.0}, // 
    {37.21, -11.87, 0.0, 0.0, 39, 0.0}, // 
    {38.49, -10.40, 0.0, 0.0, 39, 0.0}, // 
    {39.72, -11.97, 0.0, 0.0, 39, 0.0}, // 
    {40.06, -11.20, 0.0, 0.0, 39, 0.0}  // 绕杆必达区2(15)
};
int turn_flag = 1;             // 转向标志位，1表示需要执行原地转向对准目标。
int if_first_turn_near_the_target = 1; // 标志位，用于判断是否是接近目标时的第一次调整。
double distance_now = 0.0;     // 实时记录当前位置到目标点的距离。
int ZhangaiFirstFlag = 1;      // "障碍"标志位，用于初始化障碍物处理逻辑 (如寻找最近的路径点)。
//***********************************//
//******d.状态机与逻辑标志位*********//
int trotDirectionState = 0;    // 【核心状态机变量】决定机器人当前做什么。
                               // 0=停止, 1=快走, 2=跨栏, 39=雷达巡航 等。
int trot_in_small_circle=0;		 // 小碎步调整计数器
int count__ = 0;               // 通用计数器。
int times_of_turn = 0;         // 记录转向次数或时间的变量。
int qiaoqiaoflag = 0;          // "悄悄"(可能指低姿态或特定任务)标志。
int qiaoqiaofinish = 0;        // 任务完成标志。

float bufu, xp, xf, centre, centre_2; // 临时中间变量，用于计算步幅(bufu)、起点(xp)、终点(xf)等。
int step_n = 1;                // 步数计数器。
int xx = 0, crtl_num = 0;      // 循环或控制逻辑的辅助计数器。

float flag_mt = 0;             // 可能是 "Motion Time" 或 "Motor Task" 的计数/标志。
extern int flag;               // 外部通用标志位。
//***********************************//
//******e.闭环控制与算法调节*********//
float error = 0;               // 通用误差变量 (目标值 - 实际值)。
float kp = 0, ki = 0, kd = 0;  // 实时控制算法使用的 PID 系数。
float error_kp = 0, error_ki = 0, error_kd = 0; // PID 计算中的误差项 (P误差, I积分误差, D微分误差)。
float K = 0;                   // PID 计算的最终输出量 (通常叠加到 leg_Mode 上进行纠偏)。

float k1, k2 = 0;              // 左右腿的差速系数，用于转向或纠偏。
extern float k_try1, k_try2;   // 可能是调试用的或动态调整的差速系数。
float detlam1 = 0, detlam2 = 0;// 用于微调 leg_Mode 的增量。

float kp_imu = 0;              // 基于 IMU (惯性测量单元) 的控制参数。
int flag_imu = 0;              // IMU 控制使能标志。
//***********************************//
//**********f.平滑处理***************//
float xpre_flag = 0.0;         // 用于渐变 xpre 的步长 (实现平滑起步)。
float xfinal_flag = 0.0;       // 用于渐变 xfinal 的步长。
int JingSu_steps = 0;          // "竞速"模式下的步数统计，用于分阶段加速。
//***********************************//
//*******************************************************************************//
//******************************1.调度函数M**************************************//
void M_statechange()
{
	if(trotDirectionState==0)//stop
	{
		TrotKpSwing=35; 
		TrotKdSwing=10;
		TrotKpStand=35;
		TrotKdStand=10;
		__kp[1]=TrotKpSwing;
		__kd[1]=TrotKpSwing;
		__kp[2]=TrotKpStand;
		__kd[2]=TrotKpStand;
		__kp[3]=TrotKpSwing;
		__kd[3]=TrotKpSwing;
		__kp[4]=TrotKpStand;
		__kd[4]=TrotKpStand;

		if(xpre<=-0.01)xpre*=0.75;
		else xpre=0;
		if(xfinal>=0.01)xfinal*=0.75;
		else xfinal=0;
		if(zHeight>=0.01)zHeight*=0.75;
		else zHeight=0;
		M_T=1;//0.6;//1
		trot_run();
	}
	else if(trotDirectionState==1)//快快前
	{
		__kp[1]=55;
		__kd[1]=35;//45
		__kp[2]=55 ;
		__kd[2]=35;//65
		__kp[3]=55;//51
		__kd[3]=35;//+detla_kp1;
		__kp[4]=55;
		__kd[4]=35;//+detla_kp1 ;
		xpre=-0.04;//-0.015
		xfinal=0.04;//0.05  //0.05 0.05 
		zHeight=0.02;
		heightControlNow=0.25;
		M_T=0.2;//0.4//0.75
 		leg_Mode[1]=1+detlam2;
		leg_Mode[2]=1+detlam2;
		leg_Mode[3]=1+detlam1;
		leg_Mode[4]=1+detlam1;
		trot();
	}
	else if(trotDirectionState==2)//跨栏完整//起跳距离范围10cm-30cm均可，近一点更有稳健
	{
		__kp[1]=12;
		__kd[1]=100;
		__kp[2]=12;
		__kd[2]=100;
		__kp[3]=12;
		__kd[3]=100;
		__kp[4]=12;
		__kd[4]=100;
		for(int legnumber =1;legnumber<2;legnumber++)	
		{
		 M_leg[legnumber].M_expx=-0.04;
		 M_leg[legnumber].M_expz=-0.14;	
		}
		for(int legnumber =2;legnumber<3;legnumber++)	
		{
		 M_leg[legnumber].M_expx=-0.053;
		 M_leg[legnumber].M_expz=-0.135;	
		}
		for(int legnumber =3;legnumber<4;legnumber++)	
		{
		 M_leg[legnumber].M_expx=-0.055;
		 M_leg[legnumber].M_expz=-0.14;	
		}
		for(int legnumber =4;legnumber<5;legnumber++)	
		{
		 M_leg[legnumber].M_expx=-0.04;
		 M_leg[legnumber].M_expz=-0.14;	
		}
		__kp[1]=17;
		__kd[1]=30;
		__kp[2]=17;
		__kd[2]=30;
		__kp[3]=17;
		__kd[3]=30;
		__kp[4]=17;
		__kd[4]=30;
		newIK();
		HAL_Delay(1500);
		M_T=0.2;//0.2
		hurdle();
		HAL_Delay(500);
		heightControlNow=0.25;
//    trotDirectionState=39;
	}
	else if(trotDirectionState==3)//低姿匍匐
	{
		
		__kp[1]=55;
		__kd[1]=35;//45
		__kp[2]=55 ;
		__kd[2]=35;//65
		__kp[3]=54;//51
		__kd[3]=35;//+detla_kp1;
		__kp[4]=54;
		__kd[4]=35;//+detla_kp1 ;
		xpre=-0.06;//-0.015
		xfinal=0.04;//0.05  //0.05 0.05 
		zHeight=0.02;
		heightControlNow=0.24;//0.25
		M_T=0.5;//0.4//0.75//0.55
 		leg_Mode[1]=1+detlam2;
		leg_Mode[2]=1+detlam2;
		leg_Mode[3]=1+detlam1;
		leg_Mode[4]=1+detlam1;
		trot();
	}
	else if(trotDirectionState==4){//跳上斜坡

		__kp[1]=12;
		__kd[1]=100;
		__kp[2]=12;
		__kd[2]=100;
		__kp[3]=12;
		__kd[3]=100;
		__kp[4]=12;
		__kd[4]=100;
		for(int legnumber =1;legnumber<2;legnumber++)	
		{
		 M_leg[legnumber].M_expx=-0.12;
		 M_leg[legnumber].M_expz=-0.13;	
		}
		for(int legnumber =2;legnumber<4;legnumber++)	
		{
		 M_leg[legnumber].M_expx=-0.08;
		 M_leg[legnumber].M_expz=-0.13;	
		}
		for(int legnumber =4;legnumber<5;legnumber++)	
		{
		 M_leg[legnumber].M_expx=-0.12;
		 M_leg[legnumber].M_expz=-0.13;	
		}
		__kp[1]=17;
		__kd[1]=30;
		__kp[2]=17;
		__kd[2]=30;
		__kp[3]=17;
		__kd[3]=30;
		__kp[4]=17;
		__kd[4]=30;
		newIK();
		HAL_Delay(200);
		M_T=0.2;//0.2
		incline10JumpUp();	
		HAL_Delay(1000);
		heightControlNow=0.25;
	}
	else if(trotDirectionState==5)//斜坡向前走
	{
		__kp[1]=55;
		__kd[1]=35;//45
		__kp[2]=55 ;
		__kd[2]=35;//65
		__kp[3]=55;//51
		__kd[3]=35;//+detla_kp1;
		__kp[4]=55;
		__kd[4]=35;//+detla_kp1 ;
		xpre=-0.02;//-0.015
		xfinal=0.02;//0.05  //0.05 0.05 
		zHeight_L=0.06;
		zHeight_R=0.06;
		heightControlNow_L=0.25;
		heightControlNow_R=0.25;
		M_T=1.5;//0.4//0.75
 		leg_Mode[1]=1+detlam2;
		leg_Mode[2]=1+detlam2;
		leg_Mode[3]=1+detlam1;
		leg_Mode[4]=1+detlam1;
	ramp_trot();
	}
	else if(trotDirectionState==6){//左转一圈（基于慢慢走）
				__kp[1]=55;
		__kd[1]=35;//45
		__kp[2]=55 ;
		__kd[2]=35;//65
		__kp[3]=55;//51
		__kd[3]=35;//+detla_kp1;
		__kp[4]=55;
		__kd[4]=35;//+detla_kp1 ;
		xpre=-0.06;//-0.015
		xfinal=0.06;//0.05  //0.05 0.05 
		zHeight=0.06;
		heightControlNow=0.25;
		M_T=1.5;//0.4//0.75
		
		leg_Mode[3]=-1+detlam1;
		leg_Mode[4]=-1+detlam1;
 		leg_Mode[1]=1+detlam2;
		leg_Mode[2]=1+detlam2;
		for(int i=0;i<8;i++){
			trot();
		}
		trotDirectionState=0;
	}
	else if(trotDirectionState==7){//右转一圈（基于慢慢走）
			__kp[1]=55;
	__kd[1]=35;//45
	__kp[2]=55 ;
	__kd[2]=35;//65
	__kp[3]=55;//51
	__kd[3]=35;//+detla_kp1;
	__kp[4]=55;
	__kd[4]=35;//+detla_kp1 ;
	xpre=-0.06;//-0.015
	xfinal=0.06;//0.05  //0.05 0.05 
	zHeight=0.06;
	heightControlNow=0.25;
	M_T=1.5;//0.4//0.75
	
	leg_Mode[3]=-1+detlam1;
	leg_Mode[4]=-1+detlam1;
	leg_Mode[1]=1+detlam2;
	leg_Mode[2]=1+detlam2;
	for(int i=0;i<8;i++){
		trot();
	}
	trotDirectionState=0;
}
	else if(trotDirectionState==8){//圆周绕杆
				__kp[1]=55;
		__kd[1]=35;//45
		__kp[2]=55 ;
		__kd[2]=35;//65
		__kp[3]=55;//51
		__kd[3]=35;//+detla_kp1;
		__kp[4]=55;
		__kd[4]=35;//+detla_kp1 ;
		xpre=-0.06;//-0.015
		xfinal=0.06;//0.05  //0.05 0.05 
		zHeight=0.06;
		heightControlNow=0.25;
		M_T=1.5;
		leg_Mode[3]=2+detlam1;
		leg_Mode[4]=2+detlam1;
 		leg_Mode[1]=0.35+detlam2;
		leg_Mode[2]=0.35+detlam2;
		for(int i=0;i<8;i++){
			trot();
		}
		leg_Mode[3]=0.35+detlam1;
		leg_Mode[4]=0.35+detlam1;
 		leg_Mode[1]=2+detlam2;
		leg_Mode[2]=2+detlam2;
		for(int i=0;i<9;i++){
			trot();
		}
		leg_Mode[3]=2+detlam1;
		leg_Mode[4]=2+detlam1;
 		leg_Mode[1]=0.35+detlam2;
		leg_Mode[2]=0.35+detlam2;
		for(int i=0;i<8;i++){
			trot();
		}
				leg_Mode[3]=0.35+detlam1;
		leg_Mode[4]=0.35+detlam1;
 		leg_Mode[1]=2+detlam2;
		leg_Mode[2]=2+detlam2;
		for(int i=0;i<9;i++){
			trot();
		}
				leg_Mode[3]=2+detlam1;
		leg_Mode[4]=2+detlam1;
 		leg_Mode[1]=0.35+detlam2;
		leg_Mode[2]=0.35+detlam2;
		for(int i=0;i<9;i++){
			trot();
		}
	}
	else if(trotDirectionState==13)//慢慢左转
	{
	 left_slow();
	}
	
	else if(trotDirectionState==14)//慢慢右转
	{
	right_slow();
	}
	else if(trotDirectionState==15)//跳高栏
	{
	__kp[1]=12;
		__kd[1]=30;
		__kp[2]=12;
		__kd[2]=30;
		__kp[3]=12;
		__kd[3]=30;
		__kp[4]=12;
		__kd[4]=30;
		for(int legnumber =1;legnumber<5;legnumber++)	
		{
		 M_leg[legnumber].M_expx=-0.07;
		 M_leg[legnumber].M_expz=-0.17;	
		}
		newIK();
		newIK();
		newIK();
		HAL_Delay(2050);
		M_T=0.2;
		gaolan();
	trotDirectionState=0;
	}
	else if(trotDirectionState==16)//小跳
	{
	__kp[1]=12;
		__kd[1]=100;
		__kp[2]=12;
		__kd[2]=100;
		__kp[3]=12;
		__kd[3]=100;
		__kp[4]=12;
		__kd[4]=100;
		for(int legnumber =1;legnumber<2;legnumber++)	
		{
		 M_leg[legnumber].M_expx=-0.05;
		 M_leg[legnumber].M_expz=-0.15;	
		}
		for(int legnumber =2;legnumber<3;legnumber++)	
		{
		 M_leg[legnumber].M_expx=-0.07;
		 M_leg[legnumber].M_expz=-0.15;	
		}
		for(int legnumber =3;legnumber<4;legnumber++)	
		{
		 M_leg[legnumber].M_expx=-0.07;
		 M_leg[legnumber].M_expz=-0.15;	
		}
		for(int legnumber =4;legnumber<5;legnumber++)	
		{
		 M_leg[legnumber].M_expx=-0.06;
		 M_leg[legnumber].M_expz=-0.15;	
		}
		__kp[1]=17;
		__kd[1]=30;
		__kp[2]=17;
		__kd[2]=30;
		__kp[3]=17;
		__kd[3]=30;
		__kp[4]=17;
		__kd[4]=30;
		newIK();
		HAL_Delay(500);
		M_T=0.2;//0.2
		louti2();
		HAL_Delay(500);
		heightControlNow=0.25;
    trotDirectionState=0;
	}
	else if(trotDirectionState==17)//低姿匍匐???
	{
		int i=0;
		jump_best();
		trotDirectionState=0;
	}
	else if(trotDirectionState==18)//匍匐加小跑
	{
		int j=0;
		for(j=0;j<5;j++)
		{
			pufu(1);
		}
		for(j=5;j<10;j++)
		{
			pufu(2);
		}
		
		for(j=10;j<18;j++)
		{
			pufu(1);
		}
		trotDirectionState=0;
	}
	else if(trotDirectionState==19)//完整断桥
	{
		jump_small();
		HAL_Delay(500);
		jump_small();
		HAL_Delay(500);
		back_on_DuanQiao(1);
		jump_small();
		HAL_Delay(500);
		back_on_DuanQiao(1);
		jump_small();
		HAL_Delay(500);
		rotate_in_place_to_targetAngle(pathArray[places_have_reached].next_angle);//断桥上第二个点记录的角度
		float A=tan(pathArray[places_have_reached].next_angle);
		float B=-1;
		float C=-A*pathArray[places_have_reached].x+pathArray[places_have_reached].y;
		float distance=(A*rx_text.x+B*rx_text.y+C)/sqrt(A*A+B*B);
		int num=0;
		while(fabs(distance)>0.15){
			if(distance<0){
				jump_right();
			}
			else if(distance>=0){
				jump_left();
			}
			num++;
			if(num==2){
				rotate_in_place_to_targetAngle(pathArray[places_have_reached].next_angle);
				num=0;
			}
			distance=(A*rx_text.x+B*rx_text.y+C)/sqrt(A*A+B*B);
		}
		rotate_in_place_to_targetAngle(pathArray[places_have_reached].next_angle);
		big_jump();
		HAL_Delay(500);
		places_have_reached++;
		for(int p=0;p<8;p++)
		{
			walkDOWN_with_radar(pathArray[places_have_reached-2].next_angle);
		}
		trotDirectionState=39;
	}
	else if(trotDirectionState==20)//大跳，小跳，大跳，斜坡行走（组合）
	{
		jump_best();
		HAL_Delay(500);
		jump_small();
		HAL_Delay(500);
		jump_best();
		HAL_Delay(500);
		for(int p=0;p<12;p++)
		{
		ramp_walk();
		}
		trotDirectionState=0;
	}
	else if(trotDirectionState==21)//距离第一个杆中心93cm启动
	{
			int j=0;
		for(j=0;j<3;j++)
		{
			raogan(1);
		}
		for(j=0;j<5;j++)
		{
			raogan(2);
		}
		for(j=0;j<14;j++)
		{
			raogan(3);
		}
		for(j=0;j<14;j++)
		{
			raogan(2);
		}
		for(j=0;j<14;j++)
		{
			raogan(3);
		}
		for(j=0;j<14;j++)
		{
			raogan(2);
		}
		for(j=0;j<14;j++)
		{
			raogan(3);
		}
		for(j=0;j<14;j++)
		{
			raogan(2);
		}
		for(j=0;j<14;j++)
		{
			raogan(3);
		}
		trotDirectionState=23;
	}
	else if(trotDirectionState==22)//低姿匍匐
	{
			int j=0;
		for(j=0;j<16;j++)
		{
			raogan(1);
		}
		for(j=0;j<12;j++)
		{
			raogan(2);
		}
		for(j=0;j<16;j++)
		{
			raogan(1);
		}
		trotDirectionState=23;
	}
	else if(trotDirectionState==23)//慢慢前
	{
		__kp[1]=55;
		__kd[1]=15;//45
		__kp[2]=55 ;
		__kd[2]=15;//65
		__kp[3]=55;//51
		__kd[3]=15;//+detla_kp1;
		__kp[4]=55;
		__kd[4]=15;//+detla_kp1 ;
		xpre=-0.05;//-0.015
		xfinal=0.05;//0.05  //0.05 0.05 
		zHeight=0.05;
		heightControlNow=0.25;
		M_T=3;//0.4//0.75
 		leg_Mode[1]=1+0.05;
		leg_Mode[2]=1+0.05;
		leg_Mode[3]=1+0.05;
		leg_Mode[4]=1+0.05;
		trot_slow();
	}
	else if(trotDirectionState==24)//沙坑走
	{
		__kp[1]=55;
		__kd[1]=15;//45
		__kp[2]=55 ;
		__kd[2]=15;//65
		__kp[3]=55;//51
		__kd[3]=15;//+detla_kp1;
		__kp[4]=55;
		__kd[4]=15;//+detla_kp1 ;
		xpre=-0.06;//-0.015
		xfinal=0.06;//0.05  //0.05 0.05 
		zHeight=0.08;
		heightControlNow=0.25;
		M_T=1.2;//0.4//0.75
 		leg_Mode[1]=1+detlam2;
		leg_Mode[2]=1+detlam2;
		leg_Mode[3]=1+detlam1;
		leg_Mode[4]=1+detlam1;
		trot();
	}
	else if(trotDirectionState==25)//沙坑加陀螺仪
	{
		shakeng_with_Gyro(1);
	}
	else if(trotDirectionState==26)//分段转弯
	{
		PID_walk(0);
	}
	else if(trotDirectionState==27)//不分段转弯
	{
		raogan(1);
		ramp_walk();
	}
	else if(trotDirectionState==28)//斜坡直走
	{
		ramp_walk();
	}
	else if(trotDirectionState==29)//竞速带陀螺仪
	{
		dash_with_gyro();
	}
	else if(trotDirectionState==30)//高速绕杆
	{
		void high_speed_raogan(int flag);
					int j=0;
		for(j=0;j<3;j++)
		{
			high_speed_raogan(1);
		}
		
		for(j=0;j<5;j++)
		{
			high_speed_raogan(2);
		}
		
		for(j=0;j<11;j++)
		{
			high_speed_raogan(3);
		}
		
		for(j=0;j<10;j++)
		{
			high_speed_raogan(2);
		}
		
		for(j=0;j<10;j++)
		{
			high_speed_raogan(3);
		}
		trotDirectionState=0;
	}
	else if(trotDirectionState==31){//左转45度（绕杆基础）//转完基本是45度，但如果后面没有接着的动作，由于惯性，会转到60度，如果直接接着动作，不会有小碎步//增大步幅，抬腿高度，减小kd都会使转角增加
				__kp[1]=55;
		__kd[1]=35;//45
		__kp[2]=55 ;
		__kd[2]=35;//65
		__kp[3]=55;//51
		__kd[3]=35;//+detla_kp1;
		__kp[4]=55;
		__kd[4]=35;//+detla_kp1 ;
		xpre=-0.06;//-0.015
		xfinal=0.06;//0.05  //0.05 0.05 
		zHeight=0.06;
		heightControlNow=0.25;
		M_T=1.5;//0.4//0.75
		
		leg_Mode[3]=-1+detlam1;
		leg_Mode[4]=-1+detlam1;
 		leg_Mode[1]=1+detlam2;
		leg_Mode[2]=1+detlam2;
		for(int i=0;i<2;i++){
			trot();
		}
		trotDirectionState=0;
	}
	else if(trotDirectionState==32){//右转45度（绕杆基础）
				__kp[1]=55;
		__kd[1]=35;//45
		__kp[2]=55 ;
		__kd[2]=35;//65
		__kp[3]=55;//51
		__kd[3]=35;//+detla_kp1;
		__kp[4]=55;
		__kd[4]=35;//+detla_kp1 ;
		xpre=-0.06;//-0.015
		xfinal=0.06;//0.05  //0.05 0.05 
		zHeight=0.06;
		heightControlNow=0.25;
		M_T=0.5;
		leg_Mode[3]=1+detlam1;
		leg_Mode[4]=1+detlam1;
 		leg_Mode[1]=-1+detlam2;
		leg_Mode[2]=-1+detlam2;
		for(int i=0;i<2;i++){
			trot();
		}
		trotDirectionState=0;
	}
	else if(trotDirectionState==33){//绕杆（原地转＋直走）（真正的高速绕杆）
		rotate_in_place_by22_5_right(3);
		move_forward_for_steps(6);
		rotate_in_place_by22_5_left(7);
		move_forward_for_steps(10);
		rotate_in_place_by22_5_right(7);
		move_forward_for_steps(10);
		rotate_in_place_by22_5_left(6);
		trotDirectionState=0;
	}
	else if(trotDirectionState==34){//闭环绕杆（原地转＋直走）（真正的高速绕杆）
		rotate_in_place_to_targetAngle1(-30.5);//角度越大稳定性越好
		move_forward_for_steps(5);//由于是5步而不是4步，起f始位置可偏后一些
		rotate_in_place_to_targetAngle1(35.5);
		move_forward_for_steps(10);
		move_forward_for_small_steps(4);
		rotate_in_place_to_targetAngle1(-35.5);
		move_forward_for_steps(10);
		move_forward_for_small_steps(4);
		rotate_in_place_to_targetAngle1(35.5);
		move_forward_for_steps(10);
		move_forward_for_small_steps(4);
		rotate_in_place_to_targetAngle1(-35.5);
		move_forward_for_steps(10);
		move_forward_for_small_steps(4);
		trotDirectionState=0;
	}
	else if(trotDirectionState==35){//方波绕杆（原地转＋直走）（真正的高速绕杆）
		rotate_in_place_to_targetAngle1(-90);//角度越大稳定性越好
		move_forward_for_steps(1);//由于是5步而不是4步，起f始位置可偏后一些
		rotate_in_place_to_targetAngle1(0);
		move_forward_for_steps(7);
		rotate_in_place_to_targetAngle(90);
		move_forward_for_steps(5);
		rotate_in_place_to_targetAngle1(0);
		move_forward_for_steps(8);
		rotate_in_place_to_targetAngle1(-90);//角度越大稳定性越好
		move_forward_for_steps(5);//由于是5步而不是4步，起f始位置可偏后一些
		rotate_in_place_to_targetAngle1(0);
		move_forward_for_steps(8);
		rotate_in_place_to_targetAngle1(90);
		move_forward_for_steps(5);
		rotate_in_place_to_targetAngle1(0);
		move_forward_for_steps(8);
		rotate_in_place_to_targetAngle1(-90);//角度越大稳定性越好
		move_forward_for_steps(5);//由于是5步而不是4步，起f始位置可偏后一些		
		trotDirectionState=0;
	}
	else if(trotDirectionState==36)//跨栏架上细调
	{
		bench_testing();
		heightControlNow=0.25;
      trotDirectionState=0;
	}
	else if(trotDirectionState==37)//雷达巡航
	{
		//from_a2b();
	}
	else if(trotDirectionState==38)//雷达巡航v2
	{
		//from_a2b_v2();
	}
	else if(trotDirectionState==39)//雷达巡航v3
	{
		if(ZhangaiFirstFlag==1){
			int i=0;
			float nearestDistance=10000.0;
			int nearestPoint=0;
			for(i=0;i<16;i++){
				if((rx_text.x-pathArray[i].x)*(rx_text.x-pathArray[i].x)+(rx_text.y-pathArray[i].y)*(rx_text.y-pathArray[i].y)<nearestDistance){
					nearestDistance=(rx_text.x-pathArray[i].x)*(rx_text.x-pathArray[i].x)+(rx_text.y-pathArray[i].y)*(rx_text.y-pathArray[i].y);
					nearestPoint=i;
				}
			}
			places_have_reached=nearestPoint;
			ZhangaiFirstFlag=0;
		}
		from_a2b_v3();
	}
	else if(trotDirectionState==40)//快快前-架上调试
	{
	__kp[1]=55;
	__kd[1]=15;//45
	__kp[2]=55 ;
	__kd[2]=15;//65
	__kp[3]=55;//51
	__kd[3]=15;//+detla_kp1;
	__kp[4]=55;
	__kd[4]=15;//+detla_kp1 ;
	xpre=-0.05;//-0.015
	xfinal=0.05;//0.05  //0.05 0.05 
	zHeight=0.02;
	heightControlNow=0.25;
	M_T=0.6;//0.4//0.75
 		leg_Mode[1]=1+detlam2;
		leg_Mode[2]=1+detlam2;
		leg_Mode[3]=1+detlam1;
		leg_Mode[4]=1+detlam1;
		trot_testing();
	}
	else if(trotDirectionState==41)//4脚同时蹦跳快快前
	{
		__kp[1]=55;
		__kd[1]=15;//45
		__kp[2]=55 ;
		__kd[2]=15;//65
		__kp[3]=55;//51
		__kd[3]=15;//+detla_kp1;
		__kp[4]=55;
		__kd[4]=15;//+detla_kp1 ;
		xpre=-0.07;//-0.015
		xfinal=0.07;//0.05  //0.05 0.05 
		zHeight=0.03;
		heightControlNow=0.25;
		M_T=0.4;
 		leg_Mode[1]=1+detlam2;
		leg_Mode[2]=1+detlam2;
		leg_Mode[3]=1+detlam1;
		leg_Mode[4]=1+detlam1;
		trot_testing_without_delay();
	}
	else if(trotDirectionState==42)//bounding步态快快前
	{
		__kp[1]=55;
		__kd[1]=15;//45
		__kp[2]=55 ;
		__kd[2]=15;//65
		__kp[3]=55;//51
		__kd[3]=15;//+detla_kp1;
		__kp[4]=55;
		__kd[4]=15;//+detla_kp1 ;
		xpre=-0.07;//-0.015
		xfinal=0.07;//0.05  //0.05 0.05 
		zHeight=0.03;
		heightControlNow=0.25;
		M_T=1;//0.4//0.75
 		leg_Mode[1]=1+detlam2;
		leg_Mode[2]=1+detlam2;
		leg_Mode[3]=1+detlam1;
		leg_Mode[4]=1+detlam1;
		trot_testing_running();
	}
	else if(trotDirectionState==43)//沙坑完整
	{
		sandBox_all();
		trotDirectionState=39;
	}
	else if(trotDirectionState==44)//雷达巡航v3-低姿匍匐
	{
		from_a2b_v3_dizipufu();
	}
	else if(trotDirectionState==45)//高腿快快前
	{
		__kp[1]=55;
		__kd[1]=15;//45
		__kp[2]=55 ;
		__kd[2]=15;//65
		__kp[3]=55;//51
		__kd[3]=15;//+detla_kp1;
		__kp[4]=55;
		__kd[4]=15;//+detla_kp1 ;
		xpre=-0.08;//-0.015
		xfinal=0.12;//0.05  //0.05 0.05 
		zHeight=0.12;
		heightControlNow=0.32;
		M_T=1.0;//0.4//0.75
 		leg_Mode[1]=1+detlam2;
		leg_Mode[2]=1+detlam2;
		leg_Mode[3]=1+detlam1;
		leg_Mode[4]=1+detlam1;		
		trot();	
		if(xpre_flag<0.04){
			xpre_flag+=0.01;
		}
		xpre-=xpre_flag;
	}
	else if(trotDirectionState==46)//高腿快快前-准备
	{
		__kp[1]=15;
		__kd[1]=15;//45
		__kp[2]=15 ;
		__kd[2]=15;//65
		__kp[3]=15;//51
		__kd[3]=15;//+detla_kp1;
		__kp[4]=15;
		__kd[4]=15;//+detla_kp1 ;
		xpre=-0.08;//-0.015
		xfinal=0.12;//0.05  //0.05 0.05 
		zHeight=0.03;
		heightControlNow=0.32;
		M_T=0.5;//0.4//0.75
 		leg_Mode[1]=1+detlam2;
		leg_Mode[2]=1+detlam2;
		leg_Mode[3]=1+detlam1;
		leg_Mode[4]=1+detlam1;
		high_leg_prepare();
		HAL_Delay(500);
		trotDirectionState=45;
	}
	else if(trotDirectionState==47)//低腿快快前-闭环不断矫正
	{
		__kp[1]=55;
		__kd[1]=kd_s;//45kd_s
		__kp[2]=55 ;
		__kd[2]=kd_s;//65
		__kp[3]=55;//51
		__kd[3]=kd_s;//+detla_kp1;
		__kp[4]=55;
		__kd[4]=kd_s;//+detla_kp1 ;
		xpre=-0.03;//-0.015
		xfinal=0.03;//0.05  //0.05 0.05 
		zHeight=0.02;
		heightControlNow=0.25;
		M_T=0.5;//0.4//0.75		
		float x1=19.03,y1=6.24,x2=58.27,y2=5.81;
		float A=y1-y2;
		float B=x2-x1;
		float C=x1*y2-x2*y1;
		float distance=(A*rx_text.x+B*rx_text.y+C)/sqrt(A*A+B*B);		
		float k=distance/0.5/12;		
			leg_Mode[1]=1;
			leg_Mode[2]=1;
			leg_Mode[3]=1;
			leg_Mode[4]=1;
			
	if(xpre_flag<0.04){
			xpre_flag+=0.005;
			k=k*1.5;
		}
		else{
			M_T=0.4;
		}		
		if(k>0.05){
			leg_Mode[1]=1+k;
			leg_Mode[2]=1+k;
		}
		else if(k<-0.05){
			leg_Mode[3]=1-k;
			leg_Mode[4]=1-k;
		}
		if(xfinal_flag<0.04){
			xfinal_flag+=0.005;
		}
		xpre-=xpre_flag;
		xfinal+=xfinal_flag;
		M_T=xfinal*0.4/0.07;//变步幅，平稳启动
		if(kd_s>15)
		{
		kd_s=kd_s-5;
		}
		trot();
	}
	else if(trotDirectionState==48)//低腿快快前-大步幅-
	{
		__kp[1]=55;
		__kd[1]=15;//45
		__kp[2]=55 ;
		__kd[2]=15;//65
		__kp[3]=55;//51
		__kd[3]=15;//+detla_kp1;
		__kp[4]=55;
		__kd[4]=15;//+detla_kp1 ;
		xpre=-0.15;//-0.015
		xfinal=0.15;//0.05  //0.05 0.05 
		zHeight=0.02;
		heightControlNow=0.25;
		M_T=1;
 		leg_Mode[1]=1+detlam2;
		leg_Mode[2]=1+detlam2;
		leg_Mode[3]=1+detlam1;
		leg_Mode[4]=1+detlam1;
		trot();
	}
	else if(trotDirectionState==49)//快快前-架上调试-同向伸腿
	{
		__kp[1]=55;
		__kd[1]=15;//45
		__kp[2]=55 ;
		__kd[2]=15;//65
		__kp[3]=55;//51
		__kd[3]=15;//+detla_kp1;
		__kp[4]=55;
		__kd[4]=15;//+detla_kp1 ;
		xpre=-0.04;//-0.015
		xfinal=0.04;//0.05  //0.05 0.05 
		zHeight=0.03;
		heightControlNow=0.25;
		M_T=0.4;
 		leg_Mode[1]=1;
		leg_Mode[2]=1;
		leg_Mode[3]=1;
		leg_Mode[4]=1;
		trot_testing();
	}
	else if(trotDirectionState==50)//低姿匍匐
	{
		int j=0;
		for(j=0;j<12;j++)
		{
			printf("低姿匍匐中");
			printf("#实际转角：%f",rx_text.yaw);
			pufu(1);
		}
		
		for(j=0;j<10;j++)
		{
			printf("低姿匍匐中");
			printf("#实际转角：%f",rx_text.yaw);
			pufu(3);
		}
		
		for(j=0;j<12;j++)
		{
			printf("低姿匍匐中");
			printf("#实际转角：%f",rx_text.yaw);
			pufu(1);
		}
		trotDirectionState=23;
	}
	else if(trotDirectionState==51)//引入imu走直线
	{
		control_angle(0.0);
	}
	else if(trotDirectionState==52)////开环调试内容
	{
		JingSuBiHuan_1_0();
	}
	//之后全为组合
	else if(trotDirectionState==53){//低姿右转45度（for dizipufu）
		turn_45_right_dizipufu();
		trotDirectionState=0;
	}
	else if(trotDirectionState==54){//低姿左转45度（for dizipufu）
		turn_45_left_dizipufu();
		trotDirectionState=0;
	}
	else if(trotDirectionState==55){//先爬8步 -> 原地转35度 -> 爬6步 -> 转回0度 -> 爬10步。
		dizipufu_all();
		trotDirectionState=39;
	}
	else if(trotDirectionState==56){//同上
		dizipufu_all();
		trotDirectionState=0;
	}
	else if(trotDirectionState==57){//斜坡大跳，斜坡上调整角度走几十步，连续两次小跳，切入雷达巡航
		real_ramp_jump();
		for(int i=0;i<2;i++){
		ramp_walk_2_0(90);
		}
		for(int i=0;i<20;i++){
		ramp_walk_2_0(20);
		}
		small_jump();
		HAL_Delay(500);
		small_jump();
		HAL_Delay(500);
		trotDirectionState=39;
	}
	
	else if(trotDirectionState==58){//小跳两次，大跳一次，停
		jump_small();
		HAL_Delay(500);
		jump_small();
		HAL_Delay(500);
		big_jump();
		trotDirectionState=0;
	}
	else if(trotDirectionState==59){//利用雷达数据保持 0 度角直线上坡
		walkUP_with_radar(0.0);
	}
	else if(trotDirectionState==60){//利用雷达数据保持 0 度角直线下坡
		walkDOWN_with_radar(0.0);
	}
	else if(trotDirectionState==61)//断桥分解1，连续小跳
	{
		jump_small();
		HAL_Delay(500);		
		jump_small();
		HAL_Delay(500);		
		jump_small();
		HAL_Delay(500);		
		jump_small();
		HAL_Delay(500);
		trotDirectionState=39;
	}
	else if(trotDirectionState==62)//断桥分解2，终结大跳
	{
		big_jump();
		HAL_Delay(500);
		trotDirectionState=39;
	}
	else if(trotDirectionState==63)//中等跳跃
	{
	jump_middle();	
	}
	else if(trotDirectionState==64)//跨栏架上细调
	{
		ramp_big_jump();
		heightControlNow=0.25;
    trotDirectionState=0;
	}
	else if(trotDirectionState==65)//低腿快快前-大步幅-
	{
		__kp[1]=55;
		__kd[1]=35;//45
		__kp[2]=55 ;
		__kd[2]=35;//65
		__kp[3]=55;//51
		__kd[3]=35;//+detla_kp1;
		__kp[4]=55;
		__kd[4]=35;//+detla_kp1 ;
		xpre=-0.05;//-0.015
		xfinal=0.05;//0.05  //0.05 0.05 
		zHeight=0.02;
		heightControlNow=0.25;
		M_T=1;
 		leg_Mode[1]=1+detlam2;
		leg_Mode[2]=1+detlam2;
		leg_Mode[3]=1+detlam1;
		leg_Mode[4]=1+detlam1;
		trot_walk_right_or_left();
	}
	else if(trotDirectionState==66)//跨栏完整//起跳距离范围10cm-30cm均可，近一点更有稳健
	{
		__kp[1]=12;
		__kd[1]=100;
		__kp[2]=12;
		__kd[2]=100;
		__kp[3]=12;
		__kd[3]=100;
		__kp[4]=12;
		__kd[4]=100;
		for(int legnumber =1;legnumber<2;legnumber++)	
		{
		 M_leg[legnumber].M_expx=-0.04;
		 M_leg[legnumber].M_expz=-0.14;	
		}
		for(int legnumber =2;legnumber<3;legnumber++)	
		{
		 M_leg[legnumber].M_expx=-0.053;
		 M_leg[legnumber].M_expz=-0.135;	
		}
		for(int legnumber =3;legnumber<4;legnumber++)	
		{
		 M_leg[legnumber].M_expx=-0.055;
		 M_leg[legnumber].M_expz=-0.14;	
		}
		for(int legnumber =4;legnumber<5;legnumber++)	
		{
		 M_leg[legnumber].M_expx=-0.04;
		 M_leg[legnumber].M_expz=-0.14;	
		}
		__kp[1]=17;
		__kd[1]=30;
		__kp[2]=17;
		__kd[2]=30;
		__kp[3]=17;
		__kd[3]=30;
		__kp[4]=17;
		__kd[4]=30;
		newIK();
		HAL_Delay(1500);
		M_T=0.2;
		ramp_hurdle();
		HAL_Delay(500);
		heightControlNow=0.25;
      trotDirectionState=0;
	}
	else if(trotDirectionState==67)//低腿快快前-大步幅-
	{
		jump_right();
	}
	else if(trotDirectionState==68)//低腿快快前-大步幅-
	{
		jump_left();
	}
	else if(trotDirectionState==69)//使用雷达数据（rx_text.yaw）将航向角锁定在 0 度。
	{
	control_angle1(0.0);
	}
	else if(trotDirectionState==70)//低腿快快前-PID闭环不断矫正（竞速PID闭环 ）
	{
		__kp[1]=55;
		__kd[1]=kd_s;//45kd_s
		__kp[2]=55 ;
		__kd[2]=kd_s;//65
		__kp[3]=55;//51
		__kd[3]=kd_s;//+detla_kp1;
		__kp[4]=55;
		__kd[4]=kd_s;//+detla_kp1 ;
		xpre=-0.07;//-0.015
		xfinal=0.07;//0.05  //0.05 0.05 
		zHeight=0.02;
		heightControlNow=0.25;
		M_T=0.5;//0.4//0.75
		
		float x1=-5.18,y1=1.56,x2=33.94,y2=4.2;
		float A=y1-y2;
		float B=x2-x1;
		float C=x1*y2-x2*y1;
		float distance=(A*rx_text.x+B*rx_text.y+C)/sqrt(A*A+B*B);
			leg_Mode[1]=1;
			leg_Mode[2]=1;
			leg_Mode[3]=1;
			leg_Mode[4]=1;
			error_kd=distance-error_kp;
			error_kp=distance;
			error_ki+=distance;
			if(JingSu_steps<2){
				M_T=0.6;
				kp=0.2;
				ki=0.1;
				kd=0.02;
			}
			else if(JingSu_steps<4){
				M_T=0.5;
				kp=0.17;
				ki=0.1;
				kd=0.02;
			}
			else if(JingSu_steps<6){
				M_T=0.4;
				kp=0.13;
				ki=0.1;
				kd=0.02;
			}
			else{
				M_T=0.3;
				kp=0.1;
				ki=0.1;
				kd=0.02;
			}
				K=kp*error_kp+ki*error_ki+kd*error_kd;
			if(K>0.3){
				K=0.3;
			}
			else if(K<-0.3){
				K=-0.3;
			}
			printf("K:%f",K);
			
			
		if(K>0.0){
			leg_Mode[3]=1-K;
			leg_Mode[4]=1-K;
		}
		else if(K<0.0){
			leg_Mode[1]=1+K;
			leg_Mode[2]=1+K;
		}
		if(kd_s>15)
		{
		kd_s=kd_s-5;
		}
		trot_JingSuBiHuan();
	}
	else if(trotDirectionState==71)//断桥反面
	{
		rotate_in_place_to_targetAngle(pathArray[places_have_reached-1].next_angle);//断桥上第二个点记录的角度
		float A=tan(pathArray[places_have_reached-1].next_angle);
		float B=-1;
		float C=-A*pathArray[places_have_reached-1].x+pathArray[places_have_reached-1].y;
		float distance=(A*rx_text.x+B*rx_text.y+C)/sqrt(A*A+B*B);
		int num=0;
		while(fabs(distance)>0.15){
			if(distance<0){
				jump_right();
			}
			else if(distance>=0){
				jump_left();
			}
			num++;
			if(num==2){
				rotate_in_place_to_targetAngle(pathArray[places_have_reached].next_angle);
				num=0;
			}
			distance=(A*rx_text.x+B*rx_text.y+C)/sqrt(A*A+B*B);
		}
		rotate_in_place_to_targetAngle(pathArray[places_have_reached].next_angle);
		big_jump();
		HAL_Delay(500);
		for(int i=0;i<10;i++){
			walkDOWN_with_radar(pathArray[places_have_reached-1].next_angle);
		}
	}
	else if(trotDirectionState==72)//标准导航 V3
	{
		from_a2b_v3();
	}
	else if(trotDirectionState==73)//直线位置锁死前进（radar）
	{
		control_position(0.0);
	}
	else if(trotDirectionState==74)//盲走上坡
	{
		walkUP_withOUT_radar();
	}
	else if(trotDirectionState==75)//连续小跳
	{
		jump_small();
		HAL_Delay(500);
		jump_small();
		HAL_Delay(500);
		jump_small();
		HAL_Delay(500);
		jump_small();
		HAL_Delay(500);
	}
	else if(trotDirectionState==76)//直线位置锁死 2.0 (Position Control II - 场馆版)
	{
		control_position_2(0.0);
	}
	
	else if(trotDirectionState==77)//上楼梯小跳调试2026.2.5
	{
		jump_small();
		HAL_Delay(500);
		trotDirectionState=0;
	}
	else if(trotDirectionState==78)//匍匐调试imu2026.2.5
	{
		control_angle_pufu(0.0);
	}
	else if(trotDirectionState==34){//闭环绕杆（原地转＋直走）（真正的高速绕杆）
		rotate_in_place_to_targetAngle1(-30.5);//角度越大稳定性越好
		move_forward_for_steps(5);//由于是5步而不是4步，起f始位置可偏后一些
		rotate_in_place_to_targetAngle1(35.5);
		move_forward_for_steps(10);
		move_forward_for_small_steps(4);
		rotate_in_place_to_targetAngle1(-35.5);
		move_forward_for_steps(10);
		move_forward_for_small_steps(4);
		rotate_in_place_to_targetAngle1(35.5);
		move_forward_for_steps(10);
		move_forward_for_small_steps(4);
		rotate_in_place_to_targetAngle1(-35.5);
		move_forward_for_steps(10);
		move_forward_for_small_steps(4);
		trotDirectionState=0;
	}
	}
	
//*******************************************************************************//
//****************************2.导航与路径跟随***********************************//
//a1.点到点导航。计算当前位置到下一个目标点的角度和距离，调用旋转和直线行走函数。v3 版本增加了直线纠偏逻辑。
void from_a2b_v3(){							
	pathArray[places_have_reached].pitch_now=acos(1*(pathArray[places_have_reached].x-rx_text.x)/\
		sqrt(pow(pathArray[places_have_reached].x-rx_text.x,2)+pow(pathArray[places_have_reached].y-rx_text.y,2)))*\
		(180.0 / 3.14159265358979323846);//当前位置到下一个目标点向量与狗正前方向量的夹角
	if(pathArray[places_have_reached].y-rx_text.y>0){
		pathArray[places_have_reached].pitch_now=-pathArray[places_have_reached].pitch_now;
	}
	rotate_in_place_to_targetAngle_by_radar_for_line_v3();
	move_to_certain_line_v3();
}

//a2.低姿匍匐导航。专为低矮环境设计的点到点导航逻辑。
void from_a2b_v3_dizipufu(){     					
		pathArray[places_have_reached].pitch_now=acos(1*(pathArray[places_have_reached].x-rx_text.x)/\
			sqrt(pow(pathArray[places_have_reached].x-rx_text.x,2)+pow(pathArray[places_have_reached].y-rx_text.y,2)))*\
			(180.0 / 3.14159265358979323846);//当前位置到下一个目标点向量与狗正前方向量的夹角
	if(pathArray[places_have_reached].y-rx_text.y>0){
		pathArray[places_have_reached].pitch_now=-pathArray[places_have_reached].pitch_now;
	}
	rotate_in_place_to_targetAngle_by_radar_for_line_v3_dizipufu();
	move_to_certain_line_v3_dizipufu();
}

//b1.走直线/纠偏。计算机器人当前位置到理想路径向量的垂直距离，通过调整左右侧腿的步幅系数 (leg_Mode) 进行纠偏，直到到达目标点。
void move_to_certain_line_v3(){							
	int num_of_places=sizeof(pathArray)/sizeof(pathArray[0]);
		__kp[1]=55;
		__kd[1]=35;//45
		__kp[2]=55 ;
		__kd[2]=35;//65
		__kp[3]=54;//51
		__kd[3]=35;//+detla_kp1;
		__kp[4]=54;
		__kd[4]=35;//+detla_kp1 ;
		xpre=-0.065;//-0.015
		xfinal=0.065;//0.05  //0.05 0.05 
		zHeight=0.06;
		heightControlNow=0.23;//0.25
		M_T=0.5;
		if(places_have_reached>=9){
		__kp[1]=35;
		__kd[1]=35;//45
		__kp[2]=35 ;
		__kd[2]=35;//65
		__kp[3]=35;//51
		__kd[3]=35;//+detla_kp1;
		__kp[4]=35;
		__kd[4]=35;//+detla_kp1 ;
		}
		k_try1=1;
		k_try2=1;
	double d_p2ab=0.0;
	double A=rx_text.x-pathArray[places_have_reached].x;
	double B=rx_text.y-pathArray[places_have_reached].y;
	double C=-(A*pathArray[places_have_reached].x+B*pathArray[places_have_reached].y);
	d_p2ab=(A*rx_text.x+B*rx_text.y+C)/sqrt(A*A+B*B);

	int direction_flag=0;//初始点位于向量的位置（左/右）
	if(d_p2ab>0){
		direction_flag=1;
	}
	else if(d_p2ab<0){
		direction_flag=-1;
	}
	float k=1;
	float k1=0,k2=0;
		float error=0;

	while(fabs(d_p2ab)>0.03){
		printf("正常行走中");
		printf("#目标转角：%f",pathArray[places_have_reached].pitch_now);
		printf("#雷达实际转角：%f",rx_text.yaw);
		printf("#陀螺仪实际转角：%f",gyro_angle);
		printf("#当前距离：%f",d_p2ab);
		d_p2ab=(A*rx_text.x+B*rx_text.y+C)/sqrt(A*A+B*B);
		if(fabs(d_p2ab)<0.35){
			k=d_p2ab*direction_flag/0.32;
		}
		else if(d_p2ab>0){
			k=direction_flag;
		}
		else if(d_p2ab<0){
			k=-direction_flag;
		}
		if(k<0.3&&k>0){
			k=0.3;
		}
		else if(k>-0.3&&k<0){
			k=-0.3;
		}
		
		if (fabs(sqrt(pow(pathArray[places_have_reached].x-rx_text.x,2)+pow(pathArray[places_have_reached].y-rx_text.y,2)))>0.2){
			pathArray[places_have_reached].pitch_now=acos(1*(pathArray[places_have_reached].x-rx_text.x)/\
									sqrt(pow(pathArray[places_have_reached].x-rx_text.x,2)+pow(pathArray[places_have_reached].y-rx_text.y,2)))*\
									(180.0 / 3.14159265358979323846);//当前位置到下一个目标点向量与狗正前方向量的夹角
			if(pathArray[places_have_reached].y-rx_text.y>0){
				pathArray[places_have_reached].pitch_now=-pathArray[places_have_reached].pitch_now;
			}
		}
		error=rx_text.yaw-pathArray[places_have_reached].pitch_now;
		float adjust_error_times=fabs(error/90)<1?fabs(error/90):1;
		if(error<0){
			adjust_error_times=-adjust_error_times;
		}
		if(fabs(error)<2.0)
		{
		k1=k_try1;
		k2=k_try2;
		}
		else if(error<-2.0)
		{
			k1=k_try1-adjust_error_times;
			k2=k_try2+adjust_error_times;
		}
		else if(error>2.0)
		{
			k1=k_try1-adjust_error_times;
			k2=k_try2+adjust_error_times;
		}
		
		leg_Mode[1]=k*k1;
		leg_Mode[2]=k*k1;
		leg_Mode[3]=k*k2;
		leg_Mode[4]=k*k2;
		if(trotDirectionState==72){
			trot_walkUP();
		}
		else{
			trot();
		}
	}
	if(pathArray[places_have_reached].next_state!=trotDirectionState){
		rotate_in_place_to_targetAngle_by_radar_for_line_v3_plus();//新切换状态后才会考虑是否转向，可能不转向
		trotDirectionState=pathArray[places_have_reached].next_state;
	}
	places_have_reached++;
}

//b2
void move_to_certain_line_v3_dizipufu(){
	int num_of_places=sizeof(pathArray)/sizeof(pathArray[0]);
		__kp[1]=55;
		__kd[1]=35;//45
		__kp[2]=55 ;
		__kd[2]=35;//65
		__kp[3]=54;//51
		__kd[3]=35;//+detla_kp1;
		__kp[4]=54;
		__kd[4]=35;//+detla_kp1 ;
		xpre=-0.06;//-0.015
		xfinal=0.04;//0.05  //0.05 0.05 
		zHeight=0.02;
		heightControlNow=0.2;//0.25
		M_T=0.5;//0.4//0.75//0.55
		k_try1=0.951298;
		k_try2=1.048702;
	double d_p2ab=0.0;
	double A=rx_text.x-pathArray[places_have_reached].x;
	double B=rx_text.y-pathArray[places_have_reached].y;
	double C=-(A*pathArray[places_have_reached].x+B*pathArray[places_have_reached].y);
	d_p2ab=(A*rx_text.x+B*rx_text.y+C)/sqrt(A*A+B*B);
	int direction_flag=0;//初始点位于向量的位置（左/右）
	if(d_p2ab>0){
		direction_flag=1;
	}
	else if(d_p2ab<0){
		direction_flag=-1;
	}
	float k=1;
	float k1=0,k2=0;
	float error=0;
	while(fabs(d_p2ab)>0.03){
		printf("正常行走中");
		printf("#目标转角：%f",pathArray[places_have_reached].pitch_now);
		printf("#雷达实际转角：%f",rx_text.yaw);
		printf("#陀螺仪实际转角：%f",gyro_angle);
		printf("#当前距离：%f",d_p2ab);
		d_p2ab=(A*rx_text.x+B*rx_text.y+C)/sqrt(A*A+B*B);

		if(fabs(d_p2ab)<0.35){
			k=d_p2ab*direction_flag/0.32;
		}
		else if(d_p2ab>0){
			k=direction_flag;
		}
		else if(d_p2ab<0){
			k=-direction_flag;
		}
		if(k<0.3&&k>0){
			k=0.3;
		}
		else if(k>-0.3&&k<0){
			k=-0.3;
		}
		if (fabs(sqrt(pow(pathArray[places_have_reached].x-rx_text.x,2)+pow(pathArray[places_have_reached].y-rx_text.y,2)))>0.2){
			pathArray[places_have_reached].pitch_now=acos(1*(pathArray[places_have_reached].x-rx_text.x)/\
									sqrt(pow(pathArray[places_have_reached].x-rx_text.x,2)+pow(pathArray[places_have_reached].y-rx_text.y,2)))*\
									(180.0 / 3.14159265358979323846);//当前位置到下一个目标点向量与狗正前方向量的夹角
			if(pathArray[places_have_reached].y-rx_text.y>0){
				pathArray[places_have_reached].pitch_now=-pathArray[places_have_reached].pitch_now;
			}
		}
		error=rx_text.yaw-pathArray[places_have_reached].pitch_now;
		float adjust_error_times=fabs(error/90)<1?fabs(error/90):1;
		if(error<0){
			adjust_error_times=-adjust_error_times;
		}
		if(fabs(error)<2.0)
		{
		k1=k_try1;
		k2=k_try2;
		}
		else if(error<-2.0)
		{
			k1=k_try1-adjust_error_times;
			k2=k_try2+adjust_error_times;
		}
		else if(error>2.0)
		{
			k1=k_try1-adjust_error_times;
			k2=k_try2+adjust_error_times;
		}
		
		leg_Mode[1]=k*k1;
		leg_Mode[2]=k*k1;
		leg_Mode[3]=k*k2;
		leg_Mode[4]=k*k2;
		trot();
	}
	if(pathArray[places_have_reached].next_state!=trotDirectionState){
		rotate_in_place_to_targetAngle_by_radar_for_line_v3_plus();
		trotDirectionState=pathArray[places_have_reached].next_state;
	}
	
	places_have_reached++;
}

//c.距离控制。判断机器人是否到达目标点的有效半径内，控制是否切换到下一个航点。
void move_to_targetDistance(){						
	int num_of_places=sizeof(pathArray)/sizeof(pathArray[0]);
	__kp[1]=55;
	__kd[1]=15;//45
	__kp[2]=55 ;
	__kd[2]=15;//65
	__kp[3]=55;//51
	__kd[3]=15;//+detla_kp1;
	__kp[4]=55;
	__kd[4]=15;//+detla_kp1 ;
	xpre=-0.05;//-0.015
	xfinal=0.05;//0.05  //0.05 0.05 
	zHeight=0.02;
	heightControlNow=0.25;
	M_T=1.0;//0.4//0.75
	
	leg_Mode[1]=1+detlam2;
	leg_Mode[2]=1+detlam2;
	leg_Mode[3]=1+detlam1;
	leg_Mode[4]=1+detlam1;
	
	pathArray[places_have_reached].distance_square_to_next_point=pow(pathArray[places_have_reached].x-rx_text.x,2)+pow(pathArray[places_have_reached].y-rx_text.y,2);//距离第places_have_reached个位置点的距离平方
	
	if(pathArray[places_have_reached].distance_square_to_next_point<0.01){
		places_have_reached++;
		turn_flag=1;
		if_first_turn_near_the_target=1;
	}
	else if((pathArray[places_have_reached].distance_square_to_next_point<0.25)&&if_first_turn_near_the_target){
		xpre=-0.02;//-0.015
		xfinal=0.02;//0.05  //0.05 0.05
		turn_flag=1;
		if_first_turn_near_the_target=0;
		trot();
	}
	else if(pathArray[places_have_reached].distance_square_to_next_point<0.25){
		xpre=-0.03;//-0.015
		xfinal=0.03;//0.05  //0.05 0.05
		trot_in_small_circle++;
		trot();
	}
	else{
		trot();
	}
	if(places_have_reached+1>num_of_places){
		places_have_reached=0;
	}
}

//d1.原地旋转。根据雷达(rx_text.yaw)或陀螺仪数据，调整左右腿的反向运动，使机器人原地旋转至目标角度。包含多个变体（针对普通行走、低姿匍匐、高抬腿等不同模式）。
void rotate_in_place_to_targetAngle_by_radar_for_line_v3(){								

		__kp[1]=55;
		__kd[1]=35;//45
		__kp[2]=55 ;
		__kd[2]=35;//65
		__kp[3]=55;//51
		__kd[3]=35;//+detla_kp1;
		__kp[4]=55;
		__kd[4]=35;//+detla_kp1 ;
		xpre=-0.04;//-0.015
		xfinal=0.04;//0.05  //0.05 0.05 
		zHeight=0.06;
		heightControlNow=0.25;
		M_T=0.5;//0.4//0.75
		
		float k=1;
		double angle=pathArray[places_have_reached].pitch_now;
		
		int i=0;
	
 		double fake_rx_yaw=0;
			if(rx_text.yaw>=0){
				fake_rx_yaw=rx_text.yaw-360.0;
			}
			else{
				fake_rx_yaw=rx_text.yaw+360.0;
			}
			double acute_angle=fabs(rx_text.yaw-angle)<fabs(fake_rx_yaw-angle)?rx_text.yaw-angle:fake_rx_yaw-angle;
			
		while(fabs(acute_angle)>2.5){//实际角度-目标角度//0.25
			printf("正常转");
			printf("%d",i++);
			printf("#目标转角：%f",angle);
			printf("#实际转角：%f",rx_text.yaw);
			printf("#k：%f",k);
			//printf("#turn_flag：%f",turn_flag );
							printf("x:%f",rx_text.x);
				printf("y:%f",rx_text.y);
			k=acute_angle/15;

			if(k>2.5){//也可以为2限值
				k=2.5;
			}
			else if(k<-2.5){
				k=-2.5;
			}
			if(fabs(k)<0.3){
				if(k<0)k=-0.3;
				else if(k>0)k=0.3;
				else if(k==0)break;//与前提矛盾，不可能发生，这句是多余的
			}
			leg_Mode[1]=-k*0.4;
			leg_Mode[2]=-k*0.4;
			leg_Mode[3]=k*0.4;
			leg_Mode[4]=k*0.4;
			trot();
			
			if(rx_text.yaw>=0){
				fake_rx_yaw=rx_text.yaw-360.0;
			}
			else{
				fake_rx_yaw=rx_text.yaw+360.0;
			}
			acute_angle=fabs(rx_text.yaw-angle)<fabs(fake_rx_yaw-angle)?rx_text.yaw-angle:fake_rx_yaw-angle;
		}


}

//d2
void rotate_in_place_to_targetAngle_by_radar_for_line_v3_dizipufu(){

		__kp[1]=55;
		__kd[1]=35;//45
		__kp[2]=55 ;
		__kd[2]=35;//65
		__kp[3]=54;//51
		__kd[3]=35;//+detla_kp1;
		__kp[4]=54;
		__kd[4]=35;//+detla_kp1 ;
		xpre=-0.04;//-0.015
		xfinal=0.04;//0.05  //0.05 0.05 
		zHeight=0.02;
		heightControlNow=0.2;//0.25
		M_T=0.5;//0.4//0.75//0.55
		float k=1;
		double angle=pathArray[places_have_reached].pitch_now;
		
		int i=0;
	
 		double fake_rx_yaw=0;
			if(rx_text.yaw>=0){
				fake_rx_yaw=rx_text.yaw-360.0;
			}
			else{
				fake_rx_yaw=rx_text.yaw+360.0;
			}
			double acute_angle=fabs(rx_text.yaw-angle)<fabs(fake_rx_yaw-angle)?rx_text.yaw-angle:fake_rx_yaw-angle;
			
		while(fabs(acute_angle)>2.5){//实际角度-目标角度
			printf("低姿匍匐转");
			printf("%d",i++);
			printf("#目标转角：%f",angle);
			printf("#实际转角：%f",rx_text.yaw);
			printf("#k：%f",k);
			//printf("#turn_flag：%f",turn_flag );
							printf("x:%f",rx_text.x);
				printf("y:%f",rx_text.y);
						k=acute_angle/10;

			if(k>2.5){//也可以为2限值
				k=2.5;
			}
			else if(k<2.5){
				k=-2.5;
			}
			if(fabs(k)<0.3){
				if(k<=0)k=-0.3;
				else if(k>0)k=0.3;
			}
			leg_Mode[1]=-k*0.4;
			leg_Mode[2]=-k*0.4;
			leg_Mode[3]=k*0.4;
			leg_Mode[4]=k*0.4;
			trot();
			
			if(rx_text.yaw>=0){
				fake_rx_yaw=rx_text.yaw-360.0;
			}
			else{
				fake_rx_yaw=rx_text.yaw+360.0;
			}
			acute_angle=fabs(rx_text.yaw-angle)<fabs(fake_rx_yaw-angle)?rx_text.yaw-angle:fake_rx_yaw-angle;
		}
}

//d3
void rotate_in_place_to_targetAngle_by_radar_for_line_v3_plus(){

		__kp[1]=55;
		__kd[1]=35;//45
		__kp[2]=55 ;
		__kd[2]=35;//65
		__kp[3]=55;//51
		__kd[3]=35;//+detla_kp1;
		__kp[4]=55;
		__kd[4]=35;//+detla_kp1 ;
		xpre=-0.04;//-0.015
		xfinal=0.04;//0.05  //0.05 0.05 
		zHeight=0.06;
		heightControlNow=0.25;
		M_T=0.5;//0.4//0.75
		
		float k=1;
		double angle=pathArray[places_have_reached].next_angle;
		if(fabs(angle)>0.1){
			int i=0;
		
			double fake_rx_yaw=0;
				if(rx_text.yaw>=0){
					fake_rx_yaw=rx_text.yaw-360.0;
				}
				else{
					fake_rx_yaw=rx_text.yaw+360.0;
				}
				double acute_angle=fabs(rx_text.yaw-angle)<fabs(fake_rx_yaw-angle)?rx_text.yaw-angle:fake_rx_yaw-angle;
				
			while(fabs(acute_angle)>2.5){//实际角度-目标角度
				printf("plus转");
				printf("%d",i++);
				printf("#目标转角：%f",angle);
				printf("#实际转角：%f",rx_text.yaw);
				printf("#k：%f",k);
				//printf("#turn_flag：%f",turn_flag );
								printf("x:%f",rx_text.x);
					printf("y:%f",rx_text.y);
							k=acute_angle/10;

				if(k>2.5){//也可以为2限值
					k=2.5;
				}
				else if(k<-2.5){
					k=-2.5;
				}
				
				if(fabs(k)<0.3){
					if(k<=0)k=-0.3;
					else if(k>0)k=0.3;
				}
				
				leg_Mode[1]=-k*0.4;
				leg_Mode[2]=-k*0.4;
				leg_Mode[3]=k*0.4;
				leg_Mode[4]=k*0.4;
				trot();
				
				if(rx_text.yaw>=0){
					fake_rx_yaw=rx_text.yaw-360.0;
				}
				else{
					fake_rx_yaw=rx_text.yaw+360.0;
				}
				acute_angle=fabs(rx_text.yaw-angle)<fabs(fake_rx_yaw-angle)?rx_text.yaw-angle:fake_rx_yaw-angle;
			}
		}
}

//e//雷达控制接口。传入目标XY坐标，计算角度偏差并调用原地旋转函数。
void radarcontrol(float x,float y)					
{
	float dx=x-rx_text.x;
	float dy=y-rx_text.y;
	float target_angle =atan2f(dy,dx);
	rotate_in_place_to_targetAngle(target_angle);
}

//else
void rotate_in_place_to_targetAngle(float angle){
		__kp[1]=55;
		__kd[1]=35;//45
		__kp[2]=55 ;
		__kd[2]=35;//65
		__kp[3]=55;//51
		__kd[3]=35;//+detla_kp1;
		__kp[4]=55;
		__kd[4]=35;//+detla_kp1 ;
		xpre=-0.04;//-0.015
		xfinal=0.04;//0.05  //0.05 0.05 
		zHeight=0.06;
		heightControlNow=0.23;
		M_T=0.5;//0.4//0.75
		
		float k=1;
		while(fabs(angle-rx_text.yaw)>2.0){
				k=(angle-rx_text.yaw)/22.5;
			if(k>1.2){//也可以为2限值
				k=1.2;
			}
			else if(k<-1.2){
				k=-1.2;
			}
			leg_Mode[1]=k;
			leg_Mode[2]=k;
			leg_Mode[3]=-k;
			leg_Mode[4]=-k;
			trot();
		}
}
void rotate_in_place_to_targetAngle1(float angle){
		__kp[1]=55;
		__kd[1]=35;//45
		__kp[2]=55 ;
		__kd[2]=35;//65
		__kp[3]=55;//51
		__kd[3]=35;//+detla_kp1;
		__kp[4]=55;
		__kd[4]=35;//+detla_kp1 ;
		xpre=-0.04;//-0.015
		xfinal=0.04;//0.05  //0.05 0.05 
		zHeight=0.06;
		heightControlNow=0.23;
		M_T=0.5;//0.4//0.75
		
		float k=1;
		while(fabs(angle-gyro_angle)>2.0){
				k=(angle-gyro_angle)/22.5;
			if(k>1.2){//也可以为2限值
				k=1.2;
			}
			else if(k<-1.2){
				k=-1.2;
			}
			leg_Mode[1]=k;
			leg_Mode[2]=k;
			leg_Mode[3]=-k;
			leg_Mode[4]=-k;
			trot();
		}
}

void rotate_in_place_to_targetAngle_for_high_knee(float angle){
	__kp[1]=55;
		__kd[1]=15;//45
		__kp[2]=55 ;
		__kd[2]=15;//65
		__kp[3]=55;//51
		__kd[3]=15;//+detla_kp1;
		__kp[4]=55;
		__kd[4]=15;//+detla_kp1 ;
		xpre=-0.12;//-0.015
		xfinal=0.12;//0.05  //0.05 0.05 
		zHeight=0.12;
		heightControlNow=0.32;
		M_T=1.0;//0.4//0.75
		float k=1;
		while(fabs(angle-rx_text.yaw)>2.0){
				k=(angle-rx_text.yaw)/22.5;
			if(k>1.2){//也可以为2限值
				k=1.2;
			}
			else if(k<-1.2){
				k=-1.2;
			}
			leg_Mode[1]=k;
			leg_Mode[2]=k;
			leg_Mode[3]=-k;
			leg_Mode[4]=-k;
			trot();
		}
}

void rotate_in_place_by22_5_left(int number){
	__kp[1]=55;
		__kd[1]=35;//45
		__kp[2]=55 ;
		__kd[2]=35;//65
		__kp[3]=55;//51
		__kd[3]=35;//+detla_kp1;
		__kp[4]=55;
		__kd[4]=35;//+detla_kp1 ;
		xpre=-0.04;//-0.015
		xfinal=0.04;//0.05  //0.05 0.05 
		zHeight=0.06;
		heightControlNow=0.25;
		M_T=0.5;//0.4//0.75
		
		leg_Mode[3]=-1+detlam1;
		leg_Mode[4]=-1+detlam1;
 		leg_Mode[1]=1+detlam2;
		leg_Mode[2]=1+detlam2;
		for(int i=0;i<number;i++){
			trot();
		}
}

void rotate_in_place_by22_5_right(int number){
	__kp[1]=55;
		__kd[1]=35;//45
		__kp[2]=55 ;
		__kd[2]=35;//65
		__kp[3]=55;//51
		__kd[3]=35;//+detla_kp1;
		__kp[4]=55;
		__kd[4]=35;//+detla_kp1 ;
		xpre=-0.04;//-0.015
		xfinal=0.04;//0.05  //0.05 0.05 
		zHeight=0.06;
		heightControlNow=0.25;
		M_T=0.5;//0.4//0.75
		
		leg_Mode[3]=1+detlam1;
		leg_Mode[4]=1+detlam1;
 		leg_Mode[1]=-1+detlam2;
		leg_Mode[2]=-1+detlam2;
		for(int i=0;i<number;i++){
			trot();
		}
}

void turn_45_right_dizipufu(){
	__kp[1]=55;
	__kd[1]=35;//45
	__kp[2]=55 ;
	__kd[2]=35;//65
	__kp[3]=55;//51
	__kd[3]=35;//+detla_kp1;
	__kp[4]=55;
	__kd[4]=35;//+detla_kp1 ;
	xpre=-0.05;//-0.015
	xfinal=0.05;//0.05  //0.05 0.05 
	zHeight=0.06;
	heightControlNow=0.22;
	M_T=1;//0.4//0.75
	leg_Mode[3]=-1+detlam1;
	leg_Mode[4]=-1+detlam1;
	leg_Mode[1]=1+detlam2;
	leg_Mode[2]=1+detlam2;
	for(int i=0;i<2;i++){
		trot();
	}
}

void turn_45_left_dizipufu(){
	__kp[1]=55;
	__kd[1]=35;//45
	__kp[2]=55 ;
	__kd[2]=35;//65
	__kp[3]=55;//51
	__kd[3]=35;//+detla_kp1;
	__kp[4]=55;
	__kd[4]=35;//+detla_kp1 ;
	xpre=-0.05;//-0.015
	xfinal=0.05;//0.05  //0.05 0.05 
	zHeight=0.06;
	heightControlNow=0.22;
	M_T=1;//0.4//0.75
	int k1,k2=0;
	k1=1;k2=-1;
	leg_Mode[3]=(1+detlam1)*k1;
	leg_Mode[4]=(1+detlam1)*k1;
	leg_Mode[1]=(1+detlam2)*k2;
	leg_Mode[2]=(1+detlam2)*k2;
	for(int i=0;i<2;i++){
		trot();
	}
}

void walkUP_with_radar(float target_angle)
{		
		float error=0;
		float k1=1;
		float k2=1;
		float kp,ki,kd,prev_error=0;
	
		__kp[1]=55;
		__kd[1]=35;//45
		__kp[2]=55 ;
		__kd[2]=35;//65
		__kp[3]=54;//51
		__kd[3]=35;//+detla_kp1;
		__kp[4]=54;
		__kd[4]=35;//+detla_kp1 ;
		xpre=-0.065;//-0.015
		xfinal=0.065;//0.05  //0.05 0.05 
		zHeight=0.04;
		heightControlNow=0.24;//0.25
		M_T=0.4;
error=target_angle-rx_text.yaw;
		k_try1=1.093218;
		k_try2=0.906782;
		if(fabs(error)<2.0)
		{
		k_try1=1.093218;
		k_try2=0.906782;
		}
		else if(error<-2.0)
		{
			k_try1=k_try1+error/180;
			k_try2=k_try2-error/180;
		}
		else if(error>2.0)
		{
			k_try1=k_try1+error/180;
			k_try2=k_try2-error/180;
		}
 		leg_Mode[1]=(1+detlam2)*k_try1;
		leg_Mode[2]=(1+detlam2)*k_try1;
		leg_Mode[3]=(1+detlam1)*k_try2;
		leg_Mode[4]=(1+detlam1)*k_try2;
		trot_walkUP();
}

void walkDOWN_with_radar(float target_angle)
{		
		float error=0;
		float k1=1;
		float k2=1;
		float kp,ki,kd,prev_error=0;
	
		__kp[1]=55;
		__kd[1]=35;//45
		__kp[2]=55 ;
		__kd[2]=35;//65
		__kp[3]=54;//51
		__kd[3]=35;//+detla_kp1;
		__kp[4]=54;
		__kd[4]=35;//+detla_kp1 ;
		xpre=-0.065;//-0.015
		xfinal=0.065;//0.05  //0.05 0.05 
		zHeight=0.04;
		heightControlNow=0.24;
		M_T=1.2;
error=target_angle-rx_text.yaw;
				k_try1=1;
		k_try2=1;
		if(fabs(error)<2.0)
		{
		k_try1=1;
		k_try2=1;
		}
		else if(error<-2.0)
		{
			k_try1=k_try1+error/90;
			k_try2=k_try2-error/90;
		}
		else if(error>2.0)
		{
			k_try1=k_try1+error/90;
			k_try2=k_try2-error/90;
		}
 		leg_Mode[1]=(1+detlam2)*k_try1;
		leg_Mode[2]=(1+detlam2)*k_try1;
		leg_Mode[3]=(1+detlam1)*k_try2;
		leg_Mode[4]=(1+detlam1)*k_try2;
		trot_walkDOWN();

}

void move_forward_for_steps(int number){
	__kp[1]=55;
		__kd[1]=35;//45
		__kp[2]=55 ;
		__kd[2]=35;//65
		__kp[3]=55;//51
		__kd[3]=35;//+detla_kp1;
		__kp[4]=55;
		__kd[4]=35;//+detla_kp1 ;
		xpre=-0.041;//-0.015
		xfinal=0.042;//0.05  //0.05 0.05 
		zHeight=0.02;
		heightControlNow=0.25;
		M_T=0.5;//0.4//0.75
		
		leg_Mode[3]=1+detlam1;
		leg_Mode[4]=1+detlam1;
 		leg_Mode[1]=1+detlam2;
		leg_Mode[2]=1+detlam2;
		for(int i=0;i<number;i++){
			trot();
		}
}
void move_forward_for_small_steps(int number){
	__kp[1]=55;
		__kd[1]=35;//45
		__kp[2]=55 ;
		__kd[2]=35;//65
		__kp[3]=55;//51
		__kd[3]=35;//+detla_kp1;
		__kp[4]=55;
		__kd[4]=35;//+detla_kp1 ;
		xpre=-0.02;//-0.015
		xfinal=0.02;//0.05  //0.05 0.05 
		zHeight=0.02;
		heightControlNow=0.25;
		M_T=0.5;//0.4//0.75
		
		leg_Mode[3]=1+detlam1;
		leg_Mode[4]=1+detlam1;
 		leg_Mode[1]=1+detlam2;
		leg_Mode[2]=1+detlam2;
		for(int i=0;i<number;i++){
			trot();
		}
}

void walkUP_withOUT_radar()
{		
		float error=0;
		float k1=1;
		float k2=1;
		float kp,ki,kd,prev_error=0;
	
		__kp[1]=55;
		__kd[1]=15;//45
		__kp[2]=55 ;
		__kd[2]=15;//65
		__kp[3]=54;//51
		__kd[3]=15;//+detla_kp1;
		__kp[4]=54;
		__kd[4]=15;//+detla_kp1 ;
		xpre=-0.065;//-0.015
		xfinal=0.065;//0.05  //0.05 0.05 
		zHeight=0.02;
		heightControlNow=0.24;//0.25
		M_T=0.6;
				k_try1=1;
		k_try2=1;
		if(fabs(error)<2.0)
		{

		k_try1=1;
		k_try2=1;
		}
		else if(error<-2.0)
		{
			k_try1=k_try1;
			k_try2=k_try2;
		}
		else if(error>2.0)
		{
			k_try1=k_try1;
			k_try2=k_try2;
		}
 		leg_Mode[1]=(1+detlam2)*k_try1;
		leg_Mode[2]=(1+detlam2)*k_try1;
		leg_Mode[3]=(1+detlam1)*k_try2;
		leg_Mode[4]=(1+detlam1)*k_try2;
		trot_walkDOWN();
}

void control_angle(float target_angle)
{		
		float error=0;
		float k1=1;
		float k2=1;
		float kp,ki=0.01,kd,prev_error=0;
		__kp[1]=55;
		__kd[1]=35;//45
		__kp[2]=55 ;
		__kd[2]=35;//65
		__kp[3]=54;//51
		__kd[3]=35;//+detla_kp1;
		__kp[4]=54;
		__kd[4]=35;//+detla_kp1 ;
		xpre=-0.065;//-0.015
		xfinal=0.065;//0.05  //0.05 0.05 
		zHeight=0.02;
		heightControlNow=0.24;//0.25
		M_T=0.5;
		error=gyro_angle-target_angle;
		k_try1=1;k_try2=1;
		if(fabs(error)<2.0)
		{
		k_try1=1;
		k_try2=1;
		}
		else if(error<-2.0)
		{
			k_try1=1+error/180;
			k_try2=1-error/180;
		}
		else if(error>2.0)
		{
			k_try1=1+error/180;
			k_try2=1-error/180;
		}
//		k_try1=0.951298;
//		k_try2=1.048702;
		
 		leg_Mode[1]=(1+detlam2)*k_try1;
		leg_Mode[2]=(1+detlam2)*k_try1;
		leg_Mode[3]=(1+detlam1)*k_try2;
		leg_Mode[4]=(1+detlam1)*k_try2;
		flag_mt++;
		trot();


}
void control_angle_pufu(float target_angle){
		float error=0;
		float k1=1;
		float k2=1;
		float kp,ki=0.01,kd,prev_error=0;
		__kp[1]=55;
		__kd[1]=35;//45
		__kp[2]=55 ;
		__kd[2]=35;//65
		__kp[3]=54;//51
		__kd[3]=35;//+detla_kp1;
		__kp[4]=54;
		__kd[4]=35;//+detla_kp1 ;
		xpre=-0.06;//-0.015
		xfinal=0.04;//0.05  //0.05 0.05 
		zHeight=0.02;
		heightControlNow=0.24;//0.25
		M_T=0.5;//0.4//0.75//0.55
		error=gyro_angle-target_angle;
		k_try1=1;k_try2=1;
		if(fabs(error)<2.0)
		{
		k_try1=1;
		k_try2=1;
		}
		else if(error<-2.0)
		{
			k_try1=1+error/180;
			k_try2=1-error/180;
		}
		else if(error>2.0)
		{
			k_try1=1+error/180;
			k_try2=1-error/180;
		}
 		leg_Mode[1]=1+detlam2;
		leg_Mode[2]=1+detlam2;
		leg_Mode[3]=1+detlam1;
		leg_Mode[4]=1+detlam1;
		leg_Mode[1]=(1+detlam2)*k_try1;
		leg_Mode[2]=(1+detlam2)*k_try1;
		leg_Mode[3]=(1+detlam1)*k_try2;
		leg_Mode[4]=(1+detlam1)*k_try2;
		flag_mt++;
		trot();
}
void control_angle1(float target_angle)
{
		
		float error=0;
		float k1=1;
		float k2=1;
		float kp,ki,kd,prev_error=0;
	
		__kp[1]=55;
		__kd[1]=kd_s;//45
		__kp[2]=55 ;
		__kd[2]=kd_s;//65
		__kp[3]=55;//51
		__kd[3]=kd_s;//+detla_kp1;
		__kp[4]=55;
		__kd[4]=kd_s;//+detla_kp1 ;
		xpre=-0.065;//-0.015
		xfinal=0.065;//0.05  //0.05 0.05 
		zHeight=0.02;
		heightControlNow=0.24;//0.25
		M_T=0.3;
		error=target_angle-rx_text.yaw;
		k_try1=1;k_try2=1;
		if(fabs(error)<2.0)
		{
		k_try1=0.955;
		k_try2=1.045;
		}
		else if(error<-2.0)
		{
			k_try1=0.955+error/180;
			k_try2=1.045-error/180;
		}
		else if(error>2.0)
		{
			k_try1=0.955+error/180;
			k_try2=1.045-error/180;
		}
 		leg_Mode[1]=(1+detlam2)*k_try1;
		leg_Mode[2]=(1+detlam2)*k_try1;
		leg_Mode[3]=(1+detlam1)*k_try2;
		leg_Mode[4]=(1+detlam1)*k_try2;
		if(kd_s>16)
		{
		kd_s--;
		}
		flag_mt++;
		trot();
}

void control_position(float target_angle)
{
		
		float error=0;
		float k1=1;
		float k2=1;
		float kp,ki,kd,prev_error=0;
	
		__kp[1]=55;
		__kd[1]=kd_s;//45
		__kp[2]=55 ;
		__kd[2]=kd_s;//65
		__kp[3]=54;//51
		__kd[3]=kd_s;//+detla_kp1;
		__kp[4]=54;
		__kd[4]=kd_s;//+detla_kp1 ;
		xpre=-0.07;//-0.015
		xfinal=0.07;//0.05  //0.05 0.05 
		zHeight=0.02;
		heightControlNow=0.24;
		M_T=0.3;
		float x1=0.63,y1=-0.07,angle1=0,x2=38.25,y2=-1.68,angle2=0;
		float A=y1-y2;
		float B=x2-x1;
		float C=x1*y2-x2*y1;
		float distance=(A*rx_text.x+B*rx_text.y+C)/sqrt(A*A+B*B);
		
		float k=distance/0.5;
		if(fabs(distance)>0.15){
			k=distance/180;
		}
		else{
			k=0;
		}
		error=target_angle-rx_text.yaw;

		if(fabs(error)<2.0)
		{
		k_try1=1;
		k_try2=1;
		}
		else if(error<-2.0)
		{
			k_try1=1+error/180+k;
			k_try2=1-error/180-k;
		}
		else if(error>2.0)
		{
			k_try1=1+error/180+k;
			k_try2=1-error/180-k;
		}
 		leg_Mode[1]=(1+detlam2)*k_try1;
		leg_Mode[2]=(1+detlam2)*k_try1;
		leg_Mode[3]=(1+detlam1)*k_try2;
		leg_Mode[4]=(1+detlam1)*k_try2;
		if(kd_s>15)
		{
		kd_s--;
		}
		flag_mt++;
		trot();
		if(((rx_text.x-x1)*(rx_text.x-x1)+(rx_text.y-y1)*(rx_text.y-y1))>900){
			trotDirectionState=0;
		}


}

void control_position_2(float target_angle)
{
		
		float error=0;
		float k1=1;
		float k2=1;
		float kp,ki,kd,prev_error=0;
	
		__kp[1]=55;
		__kd[1]=kd_s;//45
		__kp[2]=55 ;
		__kd[2]=kd_s;//65
		__kp[3]=55;//51
		__kd[3]=kd_s;//+detla_kp1;
		__kp[4]=55;
		__kd[4]=kd_s;//+detla_kp1 ;
		xpre=-0.07;//-0.015
		xfinal=0.07;//0.05  //0.05 0.05 
		zHeight=0.02;
		heightControlNow=0.24;//0.25
		M_T=0.3;
		float x1=0.63,y1=-0.07,angle1=0,x2=38.25,y2=-1.68,angle2=0;//备馆调试
		float A=y1-y2;
		float B=x2-x1;
		float C=x1*y2-x2*y1;
		float distance=(A*rx_text.x+B*rx_text.y+C)/sqrt(A*A+B*B);
		
		error_ki+=distance;
		float k=error_ki;
		if(fabs(distance)>0.05){
			if(distance>0){
				k=(distance-0.05)/1;
			}
			else if(distance<-0.05){
				k=(distance+0.05)/1;
			}
		}
		else{
			k=0;
		}
		if(k>0.05){
			k=0.05;
		}
		else if(k<-0.05){
			k=-0.05;
		}
		error=target_angle-rx_text.yaw;

		if(fabs(error)<2.0)
		{
		k_try1=1+k+error_ki*ki;
		k_try2=1-k-error_ki*ki;
		}
		else if(error<-2.0)
		{
			k_try1=1+error/180+k+error_ki*ki;
			k_try2=1-error/180-k-error_ki*ki;
		}
		else if(error>2.0)
		{
			k_try1=1+error/180+k+error_ki*ki;
			k_try2=1-error/180-k-error_ki*ki;
		}
 		leg_Mode[1]=(1+detlam2)*k_try1;
		leg_Mode[2]=(1+detlam2)*k_try1;
		leg_Mode[3]=(1+detlam1)*k_try2;
		leg_Mode[4]=(1+detlam1)*k_try2;
		if(kd_s>15)
		{
		kd_s--;
		}
//		}
		flag_mt++;
		//trot();
		trot_run();
		if(((rx_text.x-x1)*(rx_text.x-x1)+(rx_text.y-y1)*(rx_text.y-y1))>900){
			trotDirectionState=0;
		}


}

void PID_walk(double target_direction){
		__kp[1]=55;
		__kd[1]=15;//45
		__kp[2]=55 ;
		__kd[2]=15;//65
		__kp[3]=55;//51
		__kd[3]=15;//+detla_kp1;
		__kp[4]=55;
		__kd[4]=15;//+detla_kp1 ;
		xpre=-0.05;//-0.015
		xfinal=0.05;//0.05  //0.05 0.05 
		zHeight=0.05;
		heightControlNow=0.25;
		M_T=1;//0.4//0.75
	float k1=1,k2=1;
	float eRRoR=0;
	eRRoR=gyro_angle-target_direction;
	if(eRRoR>=5){
		k1=1.6;
		k2=0.4;
	}
	else if(eRRoR<=-5){
		k1=0.4;
		k2=1.6;
	}
	else if(eRRoR>-5&&eRRoR<-2){
		k1=0.4+(eRRoR+5)*0.4/3;
		k2=1.6-(eRRoR+5)*0.4/3;
	}
	else if(eRRoR<5&&eRRoR>2){
		k1=1.6-(5-eRRoR)*0.4/3;
		k2=0.4+(5-eRRoR)*0.4/3;
	}
	else if(eRRoR<2&&eRRoR>1){
		k1=1.1;
		k2=0.9;
	}
	else if(eRRoR<-1&&eRRoR>-2){
		k1=0.9;
		k2=1.1;
	}
	 	leg_Mode[1]=1*k1;
		leg_Mode[2]=1*k1;
		leg_Mode[3]=1*k2;
		leg_Mode[4]=1*k2;
		trot();
}

void dash_with_gyro()
{
		float k1=0,k2=0;
		float error=0;
		__kp[1]=55;
		__kd[1]=15;//45
		__kp[2]=55 ;
		__kd[2]=15;//65
		__kp[3]=55;//51
		__kd[3]=15;//+detla_kp1;
		__kp[4]=55;
		__kd[4]=15;//+detla_kp1 ;
		xpre=-0.06;//-0.015
		xfinal=0.06;//0.05  //0.05 0.05 
		zHeight=0.02;
		heightControlNow=0.25;
		M_T=0.4;//0.4//0.75
		error=gyro_angle-0.0;
		if(fabs(error)<=2.0)
		{
		k1=1;
		k2=1;
		}
		else if(error>2.0)
		{
		k2=0.8;
		k1=1.2;
		}
		else if(error<-2.0)
		{
		k2=1.2;
		k1=0.8;
		} 
 		leg_Mode[1]=(1+detlam2)*k1;
		leg_Mode[2]=(1+detlam2)*k1;
		leg_Mode[3]=(1+detlam1)*k2;
		leg_Mode[4]=(1+detlam1)*k2;
		trot();
}

void JingSuBiHuan_1_0()
{
		float k1=1,k2=1;
		float error=0;
		__kp[1]=55;
		__kd[1]=15;//45
		__kp[2]=55 ;
		__kd[2]=15;//65
		__kp[3]=55;//51
		__kd[3]=15;//+detla_kp1;
		__kp[4]=55;
		__kd[4]=15;//+detla_kp1 ;
		xpre=-0.0650;//-0.015
		xfinal=0.0;//0.05  //0.05 0.05 
		zHeight=0.045;
		heightControlNow=0.25;
		M_T=0.8;//0.4//0.75//0.25
 		leg_Mode[1]=(1+detlam2)*k1;
		leg_Mode[2]=(1+detlam2)*k1;
		leg_Mode[3]=(1+detlam1)*k2;
		leg_Mode[4]=(1+detlam1)*k2;
		trot_slow();

}

//*******************************************************************************//
//******************************3.步态生成***************************************//
//a.标准小跑步态。根据当前相位计算足端轨迹。摆动相使用 sigma - sin(sigma) 曲线，支撑相直线后移。最后调用 newIK 解算关节角。
void trot()								
{
	float Ts = M_T;//Ts:一个行走周期
	float faai = 0.5;
	float t=0;
  float sigma=0;	
	for (t = 0; t <= Ts*faai; t += 0.025)//0.025
	{
		sigma = 2 * pi * t / (faai * Ts);//sigma范围化成0到2pi
		float xep_b = (xfinal - xpre) * ((sigma - sin(sigma)) / (2 * pi)) + xpre;    
		float xep_z = (xpre - xfinal) * ((sigma - sin(sigma)) / (2 * pi)) + xfinal;  
		float zep = zHeight * (1 - cos(sigma)) / 2;  
		M_leg[1].M_expx = xep_b*leg_Mode[1];  //-0.0315 +0.02    //     xep_b摆动腿，xep_z支撑腿
		M_leg[2].M_expx = xep_z*leg_Mode[2];  //-0.02
		M_leg[3].M_expx = xep_b*leg_Mode[3];  // -0.017-0.02
		M_leg[4].M_expx = xep_z*leg_Mode[4];  //+0.02
		M_leg[1].M_expz = zep-heightControlNow;  //-0.0096                    
		M_leg[2].M_expz = 0-heightControlNow;  //-0.02
		M_leg[3].M_expz = zep-heightControlNow;  //-0.01
		M_leg[4].M_expz = 0-heightControlNow;
		newIK();
		
   }
  for(t=Ts*faai;t<=Ts;t+=0.025)
		{		
		sigma = 2 * pi * (t - faai * Ts) / (faai * Ts);
		float xep_b = (xfinal - xpre) * ((sigma - sin(sigma)) / (2 * pi)) + xpre;
		float xep_z = (xpre - xfinal) * ((sigma - sin(sigma)) / (2 * pi)) + xfinal;
		float zep = zHeight * (1 - cos(sigma)) / 2;
		M_leg[1].M_expx = xep_z*leg_Mode[1];
		M_leg[2].M_expx = xep_b*leg_Mode[2];
		M_leg[3].M_expx = xep_z*leg_Mode[3];
		M_leg[4].M_expx = xep_b*leg_Mode[4];
		M_leg[1].M_expz = 0-heightControlNow;
		M_leg[2].M_expz = zep-heightControlNow;
		M_leg[3].M_expz = 0-heightControlNow;
		M_leg[4].M_expz = zep-heightControlNow;
		newIK();
		
     } 
}

//b.奔跑小跑。参数调整后的 trot，通常用于更快的速度。
void trot_run()					
{
	float Ts = M_T;//Ts:一个行走周期
	float faai = 0.5;
	float t=0;
  float sigma=0;
	
	for (t = 0; t <= Ts*faai; t += 0.025)//0.025
	{
		sigma = 2 * pi * t / (faai * Ts);//sigma范围化成0到2pi
		float xep_b = (xfinal - xpre) * ((sigma - sin(sigma)) / (2 * pi)) + xpre;    
		float xep_z = (xpre - xfinal) * ((sigma - sin(sigma)) / (2 * pi)) + xfinal;  
		float zep = zHeight * (1 - cos(sigma)) / 2;  
		M_leg[1].M_expx = xep_b*leg_Mode[1];  //-0.0315+0.02         //     xep_b摆动腿，xep_z支撑腿
		M_leg[2].M_expx = xep_z*leg_Mode[2];  //-0.02
		M_leg[3].M_expx = xep_b*leg_Mode[3];  // -0.017
		M_leg[4].M_expx = xep_z*leg_Mode[4];  //+0.02 
		M_leg[1].M_expz = zep-heightControlNow;   //-0.0096                  
		M_leg[2].M_expz = 0-heightControlNow;  //-0.02
		M_leg[3].M_expz = zep-heightControlNow;  //-0.0025
		M_leg[4].M_expz = 0-heightControlNow;
		newIK();
   }
  for(t=Ts*faai;t<=Ts;t+=0.025)
		{		
		sigma = 2 * pi * (t - faai * Ts) / (faai * Ts);
		float xep_b = (xfinal - xpre) * ((sigma - sin(sigma)) / (2 * pi)) + xpre;
		float xep_z = (xpre - xfinal) * ((sigma - sin(sigma)) / (2 * pi)) + xfinal;
		float zep = zHeight * (1 - cos(sigma)) / 2;
		M_leg[1].M_expx = xep_z*leg_Mode[1];
		M_leg[2].M_expx = xep_b*leg_Mode[2];
		M_leg[3].M_expx = xep_z*leg_Mode[3];
		M_leg[4].M_expx = xep_b*leg_Mode[4];
		M_leg[1].M_expz = 0-heightControlNow;
		M_leg[2].M_expz = zep-heightControlNow;
		M_leg[3].M_expz = 0-heightControlNow;
		M_leg[4].M_expz = zep-heightControlNow;
		newIK();
     } 
}

//c.斜坡步态。允许左右腿设置不同的抬腿高度 (zHeight) 和机身高度，用于适应横向斜坡。
void ramp_trot()
{
	float Ts = M_T;//Ts:一个行走周期
	float faai = 0.5;
	float t=0;
  float sigma=0;
	
	for (t = 0; t <= Ts*faai; t += 0.025)
	{
		sigma = 2 * pi * t / (faai * Ts);//sigma范围化成0到2pi
		float xep_b = (xfinal - xpre) * ((sigma - sin(sigma)) / (2 * pi)) + xpre;    
		float xep_z = (xpre - xfinal) * ((sigma - sin(sigma)) / (2 * pi)) + xfinal;  //final在前，pre在后
		float zep_L = zHeight_L * (1 - cos(sigma)) / 2; 
		float zep_R = zHeight_R * (1 - cos(sigma)) / 2;
		M_leg[1].M_expx = xep_b*leg_Mode[1] ;  //     xep_b摆动腿，xep_z支撑腿
		M_leg[2].M_expx = xep_z*leg_Mode[2] ;
		M_leg[3].M_expx = xep_b*leg_Mode[3] ;
		M_leg[4].M_expx = xep_z*leg_Mode[4] ;
		M_leg[1].M_expz = zep_L-heightControlNow_L;                     
		M_leg[2].M_expz = 0-heightControlNow_L;
		M_leg[3].M_expz = zep_R-heightControlNow_R;
		M_leg[4].M_expz = 0-heightControlNow_R;
		newIK();

   }

  for(t=Ts*faai;t<=Ts;t+=0.025)
		{		
		sigma = 2 * pi * (t - faai * Ts) / (faai * Ts);
		float xep_b = (xfinal - xpre) * ((sigma - sin(sigma)) / (2 * pi)) + xpre;
		float xep_z = (xpre - xfinal) * ((sigma - sin(sigma)) / (2 * pi)) + xfinal;
		float zep_L = zHeight_L * (1 - cos(sigma)) / 2; 
		float zep_R = zHeight_R * (1 - cos(sigma)) / 2;
		M_leg[1].M_expx = xep_z*leg_Mode[1] ;
		M_leg[2].M_expx = xep_b*leg_Mode[2] ;
		M_leg[3].M_expx = xep_z*leg_Mode[3] ;
		M_leg[4].M_expx = xep_b*leg_Mode[4] ;
		M_leg[1].M_expz = 0-heightControlNow_L;
		M_leg[2].M_expz = zep_L-heightControlNow_L;
		M_leg[3].M_expz = 0-heightControlNow_R;
		M_leg[4].M_expz = zep_R-heightControlNow_R;
  	newIK();
     } 
}

//d.慢速步态。调整了占空比 (faai) 和周期 (M_T)，用于精细移动或转弯。
//d1
void trot_slow(){

	float Ts = M_T;//Ts:一个行走周期
	float faai = 0.5;
	float t=0;
  float sigma=0;
	
	float xpre1=xpre-0.0205;
	float xpre2=xpre;
	float xpre3=xpre-0.017;
	float xpre4=xpre;
	float xfinal1=xfinal-0.0205;
	float xfinal2=xfinal;
	float xfinal3=xfinal-0.017;
	float xfinal4=xfinal;
	float zHeight1=zHeight;
	float zHeight2=zHeight;
	float zHeight3=zHeight;
	float zHeight4=zHeight;
	float heightControlNow1=heightControlNow;
	float heightControlNow2=heightControlNow-0.016;
	float heightControlNow3=heightControlNow-0.01;
	float heightControlNow4=heightControlNow;
	for (t = 0; t <= Ts*faai; t += 0.025)
	{
		sigma = 2 * pi * t / (faai * Ts);//sigma范围化成0到2pi
		float xep_b1 = (xfinal1 - xpre1) * ((sigma - sin(sigma)) / (2 * pi)) + xpre1;    
		float xep_z2 = (xpre2 - xfinal2) * ((sigma - sin(sigma)) / (2 * pi)) + xfinal2; 
		float xep_b3 = (xfinal3 - xpre3) * ((sigma - sin(sigma)) / (2 * pi)) + xpre3;    
		float xep_z4 = (xpre4 - xfinal4) * ((sigma - sin(sigma)) / (2 * pi)) + xfinal4; 
		float zep1 = zHeight1 * (1 - cos(sigma)) / 2; 
		float zep2 = zHeight2 * (1 - cos(sigma)) / 2; 
		float zep3 = zHeight3 * (1 - cos(sigma)) / 2; 
		float zep4 = zHeight4 * (1 - cos(sigma)) / 2; 
		M_leg[1].M_expx = xep_b1*leg_Mode[1] ;  //     xep_b摆动腿，xep_z支撑腿
		M_leg[2].M_expx = xep_z2*leg_Mode[2] ;
		M_leg[3].M_expx = xep_b3*leg_Mode[3] ;
		M_leg[4].M_expx = xep_z4*leg_Mode[4] ;
		M_leg[1].M_expz = zep1-heightControlNow1;                     
		M_leg[2].M_expz = 0-heightControlNow2;
		M_leg[3].M_expz = zep3-heightControlNow3;
		M_leg[4].M_expz = 0-heightControlNow4;	
		newIK();	
   }
  for(t=Ts*faai;t<=Ts;t+=0.025)
		{		
		sigma = 2 * pi * (t - faai * Ts) / (faai * Ts);
		float xep_b2 = (xfinal2 - xpre2) * ((sigma - sin(sigma)) / (2 * pi)) + xpre2;    
		float xep_z1 = (xpre1 - xfinal1) * ((sigma - sin(sigma)) / (2 * pi)) + xfinal1; 
		float xep_b4 = (xfinal4 - xpre4) * ((sigma - sin(sigma)) / (2 * pi)) + xpre4;    
		float xep_z3 = (xpre3 - xfinal3) * ((sigma - sin(sigma)) / (2 * pi)) + xfinal3; 
		float zep1 = zHeight1 * (1 - cos(sigma)) / 2; 
		float zep2 = zHeight2 * (1 - cos(sigma)) / 2; 
		float zep3 = zHeight3 * (1 - cos(sigma)) / 2; 
		float zep4 = zHeight4 * (1 - cos(sigma)) / 2; 			
		M_leg[1].M_expx = xep_z1*leg_Mode[1] ;
		M_leg[2].M_expx = xep_b2*leg_Mode[2] ;
		M_leg[3].M_expx = xep_z3*leg_Mode[3] ;
		M_leg[4].M_expx = xep_b4*leg_Mode[4] ;
		M_leg[1].M_expz = 0-heightControlNow1;
		M_leg[2].M_expz = zep2-heightControlNow2;
		M_leg[3].M_expz = 0-heightControlNow3;
		M_leg[4].M_expz = zep4-heightControlNow4;
		newIK();
     } 
}
//d2
void left_slow(void){
		__kp[1]=60;
		__kd[1]=40;
		__kp[2]=60;
		__kd[2]=40;
		__kp[3]=58;
		__kd[3]=45;
		__kp[4]=58;
		__kd[4]=45;
		xpre=-0.01;
		xfinal=0.01;
		zHeight=0.045;
		heightControlNow=0.25;
		M_T=0.55;//1
		leg_Mode[1]=-0.8;
		leg_Mode[2]=-0.8;
		leg_Mode[3]=0.8;
		leg_Mode[4]=0.8;
        
	
	float Ts = M_T;
	float faai = 0.4;
	float t;
  float sigma;
	
	for (t = 0; t <= Ts*faai; t += 0.025)
	{
		sigma = 2 * pi * t / (faai * Ts);
		float xep_b = (xfinal - xpre) * ((sigma - sin(sigma)) / (2 * pi)) + xpre;    
		float xep_z = (xpre - xfinal) * ((sigma - sin(sigma)) / (2 * pi)) + xfinal;  
		float zep = zHeight * (1 - cos(sigma)) / 2;  
		M_leg[1].M_expx = xep_b*leg_Mode[1] ;       
		M_leg[2].M_expx = xep_z*leg_Mode[2] ;
		M_leg[3].M_expx = xep_b*leg_Mode[3] ;
		M_leg[4].M_expx = xep_z*leg_Mode[4] ;
		M_leg[1].M_expz = zep*fabsf(leg_Mode[1])-heightControlNow;                     
		M_leg[2].M_expz = 0-heightControlNow;
		M_leg[3].M_expz = zep*fabsf(leg_Mode[3])-heightControlNow;
		M_leg[4].M_expz = 0-heightControlNow;
		M_leg[1].M_expx-=0.015;
		M_leg[2].M_expx-=0.015;
		M_leg[3].M_expx-=0.015;
		M_leg[4].M_expx-=0.015;
		newIK();

   }

  for(t=Ts*faai;t<=Ts;t+=0.025)
		{		
		sigma = 2 * pi * (t - faai * Ts) / (faai * Ts);
		float xep_b = (xfinal - xpre) * ((sigma - sin(sigma)) / (2 * pi)) + xpre;
		float xep_z = (xpre - xfinal) * ((sigma - sin(sigma)) / (2 * pi)) + xfinal;
		float zep = zHeight * (1 - cos(sigma)) / 2;
		M_leg[1].M_expx = xep_z*leg_Mode[1] ;
		M_leg[2].M_expx = xep_b*leg_Mode[2] ;
		M_leg[3].M_expx = xep_z*leg_Mode[3] ;
		M_leg[4].M_expx = xep_b*leg_Mode[4] ;
		M_leg[1].M_expz = 0-heightControlNow;
		M_leg[2].M_expz = zep*fabsf(leg_Mode[2])-heightControlNow;
		M_leg[3].M_expz = 0-heightControlNow;
		M_leg[4].M_expz = zep*fabsf(leg_Mode[4])-heightControlNow;
		M_leg[1].M_expx-=0.015;
		M_leg[2].M_expx-=0.015;
		M_leg[3].M_expx-=0.015;
		M_leg[4].M_expx-=0.015;
		newIK();
		}

}
//d3
void right_slow(){
        __kp[1]=55;
		__kd[1]=40;
		__kp[2]=55;
		__kd[2]=40;
		__kp[3]=62;
		__kd[3]=40;
		__kp[4]=62;
		__kd[4]=40;
		xpre=-0.01;
		xfinal=0.01;
		zHeight=0.045;
		heightControlNow=0.25;
		M_T=0.55;//1
		leg_Mode[1]=0.8;
		leg_Mode[2]=0.8;
		leg_Mode[3]=-0.8;
		leg_Mode[4]=-0.8;				
	float Ts = M_T;
	float faai = 0.4;
	float t;
  float sigma;
	for (t = 0; t <= Ts*faai; t += 0.025)
	{
		sigma = 2 * pi * t / (faai * Ts);
		float xep_b = (xfinal - xpre) * ((sigma - sin(sigma)) / (2 * pi)) + xpre;    
		float xep_z = (xpre - xfinal) * ((sigma - sin(sigma)) / (2 * pi)) + xfinal;  
		float zep = zHeight * (1 - cos(sigma)) / 2;  
		M_leg[1].M_expx = xep_b*leg_Mode[1] ;       
		M_leg[2].M_expx = xep_z*leg_Mode[2] ;
		M_leg[3].M_expx = xep_b*leg_Mode[3] ;
		M_leg[4].M_expx = xep_z*leg_Mode[4] ;
		M_leg[1].M_expz = zep*fabsf(leg_Mode[1])-heightControlNow;                     
		M_leg[2].M_expz = 0-heightControlNow;
		M_leg[3].M_expz = zep*fabsf(leg_Mode[3])-heightControlNow;
		M_leg[4].M_expz = 0-heightControlNow;
		M_leg[1].M_expx-=0.015;
		M_leg[2].M_expx-=0.015;
		M_leg[3].M_expx-=0.015;
		M_leg[4].M_expx-=0.015;
		newIK();  
   }
  for(t=Ts*faai;t<=Ts;t+=0.025)
		{		
		sigma = 2 * pi * (t - faai * Ts) / (faai * Ts);
		float xep_b = (xfinal - xpre) * ((sigma - sin(sigma)) / (2 * pi)) + xpre;
		float xep_z = (xpre - xfinal) * ((sigma - sin(sigma)) / (2 * pi)) + xfinal;
		float zep = zHeight * (1 - cos(sigma)) / 2;
		M_leg[1].M_expx = xep_z*leg_Mode[1] ;
		M_leg[2].M_expx = xep_b*leg_Mode[2] ;
		M_leg[3].M_expx = xep_z*leg_Mode[3] ;
		M_leg[4].M_expx = xep_b*leg_Mode[4] ;
		M_leg[1].M_expz = 0-heightControlNow;
		M_leg[2].M_expz = zep*fabsf(leg_Mode[2])-heightControlNow;
		M_leg[3].M_expz = 0-heightControlNow;
		M_leg[4].M_expz = zep*fabsf(leg_Mode[4])-heightControlNow;
		M_leg[1].M_expx-=0.015;
		M_leg[2].M_expx-=0.015;
		M_leg[3].M_expx-=0.015;
		M_leg[4].M_expx-=0.015;
		newIK();
		}
}



//e.高抬腿。大幅增加 zHeight，用于跨越障碍物。
void high_knee_walk(){
	__kp[1]=55;
		__kd[1]=15;//45
		__kp[2]=55 ;
		__kd[2]=15;//65
		__kp[3]=55;//51
		__kd[3]=15;//+detla_kp1;
		__kp[4]=55;
		__kd[4]=15;//+detla_kp1 ;
		xpre=-0.12;//-0.015
		xfinal=0.12;//0.05  //0.05 0.05 
		zHeight=0.12;
		heightControlNow=0.32;
		M_T=1.2;//0.4//0.75
		float error1=pathArray[11].next_angle-rx_text.yaw;
	if(fabs(error1)<=2.0)
		{
		k1=1;
		k2=1;
		}
		else if(error1>2.0)
		{
		k2=0.7;
		k1=1.3;
		}
		else if(error1<-2.0)
		{
		k2=1.3;
		k1=0.7;
		} 	
		leg_Mode[1]=k1;
		leg_Mode[2]=k1;
		leg_Mode[3]=k2;
		leg_Mode[4]=k2;	
		trot();

}


//f.低姿匍匐。降低机身高度，改变步态频率，用于钻过低矮障碍。
void pufu(int flag)
{
		__kp[1]=55;
		__kd[1]=35;//45
		__kp[2]=55 ;
		__kd[2]=35;//65
		__kp[3]=54;//51
		__kd[3]=35;//+detla_kp1;
		__kp[4]=54;
		__kd[4]=35;//+detla_kp1 ;
		xpre=-0.05;//-0.015
		xfinal=0.05;//0.05  //0.05 0.05 
		zHeight=0.02;
		heightControlNow=0.23;//0.25//0.23
		M_T=0.5;//0.4//0.75//0.55
		leg_Mode[1]=(1+detlam2);
		leg_Mode[2]=(1+detlam2);
		leg_Mode[3]=(1+detlam1);
		leg_Mode[4]=(1+detlam1);
		for(int i=0;i<flag;i++){
			trot();
		}
}


//g.上下坡步态。针对上坡或下坡调整重心前后位置（通过 xpre/xfinal 偏置）。
void trot_walkUP()
{
	float Ts = M_T;//Ts:一个行走周期
	float faai = 0.5;
	float t=0;
  float sigma=0;
	float xpre1=xpre-0.0315-0.03;
	float xpre2=xpre-0.03;
	float xpre3=xpre-0.017-0.03;
	float xpre4=xpre-0.03;
	float xfinal1=xfinal-0.0315-0.05;
	float xfinal2=xfinal-0.05;
	float xfinal3=xfinal-0.017-0.05;
	float xfinal4=xfinal-0.05;
	float zHeight1=zHeight+0.05;
	float zHeight2=zHeight+0.05;
	float zHeight3=zHeight+0.05;
	float zHeight4=zHeight+0.05;
	float heightControlNow1=heightControlNow-0.0096;
	float heightControlNow2=heightControlNow-0.0096;
	float heightControlNow3=heightControlNow-0.01;
	float heightControlNow4=heightControlNow;
	for (t = 0; t <= Ts*faai; t += 0.025)
	{
		sigma = 2 * pi * t / (faai * Ts);//sigma范围化成0到2pi
		float xep_b1 = (xfinal1 - xpre1) * ((sigma - sin(sigma)) / (2 * pi)) + xpre1;    
		float xep_z2 = (xpre2 - xfinal2) * ((sigma - sin(sigma)) / (2 * pi)) + xfinal2; 
		float xep_b3 = (xfinal3 - xpre3) * ((sigma - sin(sigma)) / (2 * pi)) + xpre3;    
		float xep_z4 = (xpre4 - xfinal4) * ((sigma - sin(sigma)) / (2 * pi)) + xfinal4; 
		float zep1 = zHeight1 * (1 - cos(sigma)) / 2; 
		float zep2 = zHeight2 * (1 - cos(sigma)) / 2; 
		float zep3 = zHeight3 * (1 - cos(sigma)) / 2; 
		float zep4 = zHeight4 * (1 - cos(sigma)) / 2; 
		M_leg[1].M_expx = xep_b1*leg_Mode[1] ;  //     xep_b摆动腿，xep_z支撑腿
		M_leg[2].M_expx = xep_z2*leg_Mode[2] ;
		M_leg[3].M_expx = xep_b3*leg_Mode[3] ;
		M_leg[4].M_expx = xep_z4*leg_Mode[4] ;
		M_leg[1].M_expz = zep1-heightControlNow1;                     
		M_leg[2].M_expz = 0-heightControlNow2;
		M_leg[3].M_expz = zep3-heightControlNow3;
		M_leg[4].M_expz = 0-heightControlNow4;
		newIK();
		
   }
  for(t=Ts*faai;t<=Ts;t+=0.025)
		{		
		sigma = 2 * pi * (t - faai * Ts) / (faai * Ts);
		float xep_b2 = (xfinal2 - xpre2) * ((sigma - sin(sigma)) / (2 * pi)) + xpre2;    
		float xep_z1 = (xpre1 - xfinal1) * ((sigma - sin(sigma)) / (2 * pi)) + xfinal1; 
		float xep_b4 = (xfinal4 - xpre4) * ((sigma - sin(sigma)) / (2 * pi)) + xpre4;    
		float xep_z3 = (xpre3 - xfinal3) * ((sigma - sin(sigma)) / (2 * pi)) + xfinal3; 
		float zep1 = zHeight1 * (1 - cos(sigma)) / 2; 
		float zep2 = zHeight2 * (1 - cos(sigma)) / 2; 
		float zep3 = zHeight3 * (1 - cos(sigma)) / 2; 
		float zep4 = zHeight4 * (1 - cos(sigma)) / 2; 
			
		M_leg[1].M_expx = xep_z1*leg_Mode[1] ;
		M_leg[2].M_expx = xep_b2*leg_Mode[2] ;
		M_leg[3].M_expx = xep_z3*leg_Mode[3] ;
		M_leg[4].M_expx = xep_b4*leg_Mode[4] ;
		M_leg[1].M_expz = 0-heightControlNow1;
		M_leg[2].M_expz = zep2-heightControlNow2;
		M_leg[3].M_expz = 0-heightControlNow3;
		M_leg[4].M_expz = zep4-heightControlNow4;
		newIK();
     } 
}
void trot_walkDOWN()
{

	float Ts = M_T;//Ts:一个行走周期
	float faai = 0.5;
	float t=0;
  float sigma=0;
	float xpre1=xpre-0.0315+0.03;
	float xpre2=xpre+0.03;
	float xpre3=xpre-0.017+0.03;
	float xpre4=xpre+0.03;
	float xfinal1=xfinal-0.0315+0.05;
	float xfinal2=xfinal+0.05;
	float xfinal3=xfinal-0.017+0.05;
	float xfinal4=xfinal+0.05;
	float zHeight1=zHeight+0.05;
	float zHeight2=zHeight+0.05;
	float zHeight3=zHeight+0.05;
	float zHeight4=zHeight+0.05;
	float heightControlNow1=heightControlNow-0.0096;
	float heightControlNow2=heightControlNow-0.0096;
	float heightControlNow3=heightControlNow-0.01;
	float heightControlNow4=heightControlNow;
	for (t = 0; t <= Ts*faai; t += 0.025)
	{
		sigma = 2 * pi * t / (faai * Ts);//sigma范围化成0到2pi
		float xep_b1 = (xfinal1 - xpre1) * ((sigma - sin(sigma)) / (2 * pi)) + xpre1;    
		float xep_z2 = (xpre2 - xfinal2) * ((sigma - sin(sigma)) / (2 * pi)) + xfinal2; 
		float xep_b3 = (xfinal3 - xpre3) * ((sigma - sin(sigma)) / (2 * pi)) + xpre3;    
		float xep_z4 = (xpre4 - xfinal4) * ((sigma - sin(sigma)) / (2 * pi)) + xfinal4; 
		float zep1 = zHeight1 * (1 - cos(sigma)) / 2; 
		float zep2 = zHeight2 * (1 - cos(sigma)) / 2; 
		float zep3 = zHeight3 * (1 - cos(sigma)) / 2; 
		float zep4 = zHeight4 * (1 - cos(sigma)) / 2; 
		M_leg[1].M_expx = xep_b1*leg_Mode[1] ;  //     xep_b摆动腿，xep_z支撑腿
		M_leg[2].M_expx = xep_z2*leg_Mode[2] ;
		M_leg[3].M_expx = xep_b3*leg_Mode[3] ;
		M_leg[4].M_expx = xep_z4*leg_Mode[4] ;
		M_leg[1].M_expz = zep1-heightControlNow1;                     
		M_leg[2].M_expz = 0-heightControlNow2;
		M_leg[3].M_expz = zep3-heightControlNow3;
		M_leg[4].M_expz = 0-heightControlNow4;
		newIK();
   }
  for(t=Ts*faai;t<=Ts;t+=0.025)
		{		
		sigma = 2 * pi * (t - faai * Ts) / (faai * Ts);
		float xep_b2 = (xfinal2 - xpre2) * ((sigma - sin(sigma)) / (2 * pi)) + xpre2;    
		float xep_z1 = (xpre1 - xfinal1) * ((sigma - sin(sigma)) / (2 * pi)) + xfinal1; 
		float xep_b4 = (xfinal4 - xpre4) * ((sigma - sin(sigma)) / (2 * pi)) + xpre4;    
		float xep_z3 = (xpre3 - xfinal3) * ((sigma - sin(sigma)) / (2 * pi)) + xfinal3; 
		float zep1 = zHeight1 * (1 - cos(sigma)) / 2; 
		float zep2 = zHeight2 * (1 - cos(sigma)) / 2; 
		float zep3 = zHeight3 * (1 - cos(sigma)) / 2; 
		float zep4 = zHeight4 * (1 - cos(sigma)) / 2; 
		M_leg[1].M_expx = xep_z1*leg_Mode[1] ;
		M_leg[2].M_expx = xep_b2*leg_Mode[2] ;
		M_leg[3].M_expx = xep_z3*leg_Mode[3] ;
		M_leg[4].M_expx = xep_b4*leg_Mode[4] ;
		M_leg[1].M_expz = 0-heightControlNow1;
		M_leg[2].M_expz = zep2-heightControlNow2;
		M_leg[3].M_expz = 0-heightControlNow3;
		M_leg[4].M_expz = zep4-heightControlNow4;
		newIK();
     } 
}


//else
void trot_testing()
{

	float Ts = M_T;//Ts:一个行走周期
	float faai = 0.5;
	float t=0;
  float sigma=0;
	
	float xpre1=xpre-0.0165;
	float xpre2=xpre;
	float xpre3=xpre-0.017;
	float xpre4=xpre;
	float xfinal1=xfinal-0.0165;
	float xfinal2=xfinal;
	float xfinal3=xfinal-0.017;
	float xfinal4=xfinal;
	float zHeight1=zHeight;
	float zHeight2=zHeight;
	float zHeight3=zHeight;
	float zHeight4=zHeight;
	float heightControlNow1=heightControlNow-0.0096;
	float heightControlNow2=heightControlNow-0.0096;
	float heightControlNow3=heightControlNow-0.01;
	float heightControlNow4=heightControlNow;
	
	
	for (t = 0; t <= Ts*faai; t += 0.025)
	{
		sigma = 2 * pi * t / (faai * Ts);
		float xep_b1 = (xfinal1 - xpre1) * ((sigma - sin(sigma)) / (2 * pi)) + xpre1; 
				float xep_b2 = (xfinal2 - xpre2) * ((sigma - sin(sigma)) / (2 * pi)) + xpre2;  
		float xep_b3 = (xfinal3 - xpre3) * ((sigma - sin(sigma)) / (2 * pi)) + xpre3; 
				float xep_b4 = (xfinal4 - xpre4) * ((sigma - sin(sigma)) / (2 * pi)) + xpre4;  
		float zep1 = zHeight1 * (1 - cos(sigma)) / 2; 
		float zep2 = zHeight2 * (1 - cos(sigma)) / 2; 
		float zep3 = zHeight3 * (1 - cos(sigma)) / 2; 
		float zep4 = zHeight4 * (1 - cos(sigma)) / 2; 
		M_leg[1].M_expx = xep_b1*leg_Mode[1] ;  //     xep_b摆动腿，xep_z支撑腿
		M_leg[2].M_expx = xep_b2*leg_Mode[2] ;//检查xpre1位置
		M_leg[3].M_expx = xep_b3*leg_Mode[3] ;
		M_leg[4].M_expx = xep_b4*leg_Mode[4] ;
		M_leg[1].M_expz = -heightControlNow1; //检查heightcrtlnow高度		
		M_leg[2].M_expz = 0-heightControlNow2;
		M_leg[3].M_expz = -heightControlNow3;
		M_leg[4].M_expz = 0-heightControlNow4;
		newIK();
		HAL_Delay(1500);
   }


  for(t=Ts*faai;t<=Ts;t+=0.025)
		{		
		sigma = 2 * pi * (t - faai * Ts) / (faai * Ts);
					float xep_z2 = (xpre2 - xfinal2) * ((sigma - sin(sigma)) / (2 * pi)) + xfinal2; 

		float xep_z1 = (xpre1 - xfinal1) * ((sigma - sin(sigma)) / (2 * pi)) + xfinal1; 
					float xep_z4 = (xpre4 - xfinal4) * ((sigma - sin(sigma)) / (2 * pi)) + xfinal4; 

		float xep_z3 = (xpre3 - xfinal3) * ((sigma - sin(sigma)) / (2 * pi)) + xfinal3; 
		float zep1 = zHeight1 * (1 - cos(sigma)) / 2; 
		float zep2 = zHeight2 * (1 - cos(sigma)) / 2; 
		float zep3 = zHeight3 * (1 - cos(sigma)) / 2; 
		float zep4 = zHeight4 * (1 - cos(sigma)) / 2; 
			
		M_leg[1].M_expx = xep_z1*leg_Mode[1] ;//检查xfinal位置
		M_leg[2].M_expx = xep_z2*leg_Mode[2] ;
		M_leg[3].M_expx = xep_z3*leg_Mode[3] ;
		M_leg[4].M_expx = xep_z4*leg_Mode[4] ;
		M_leg[1].M_expz = zep1-heightControlNow1;//在heightcontrol已经一样标准下检查zep
		M_leg[2].M_expz = zep2-heightControlNow2;
		M_leg[3].M_expz = zep3-heightControlNow3;
		M_leg[4].M_expz = zep4-heightControlNow4;
		newIK();
		HAL_Delay(1500);
     } 
}

void trot_testing_without_delay()
{

	float Ts = M_T;
	float faai = 0.5;
	float t=0;
  float sigma=0;
	
	float xpre1=xpre;
	float xpre2=xpre;
	float xpre3=xpre+0.02;
	float xpre4=xpre+0.02;
	float xfinal1=xfinal;
	float xfinal2=xfinal;
	float xfinal3=xfinal+0.02;
	float xfinal4=xfinal+0.02;
	float zHeight1=zHeight;
	float zHeight2=zHeight;
	float zHeight3=zHeight;
	float zHeight4=zHeight;
	float heightControlNow1=heightControlNow;
	float heightControlNow2=heightControlNow+0.01;
	float heightControlNow3=heightControlNow+0.0015;
	float heightControlNow4=heightControlNow;
	
	
	for (t = 0; t <= Ts*faai; t += 0.025)
	{
		sigma = 2 * pi * t / (faai * Ts);
		float xep_b1 = (xfinal1 - xpre1) * ((sigma - sin(sigma)) / (2 * pi)) + xpre1;  
				float xep_b2 = (xfinal2 - xpre2) * ((sigma - sin(sigma)) / (2 * pi)) + xpre2;  
		float xep_b3 = (xfinal3 - xpre3) * ((sigma - sin(sigma)) / (2 * pi)) + xpre3;  
				float xep_b4 = (xfinal4 - xpre4) * ((sigma - sin(sigma)) / (2 * pi)) + xpre4; 
		float zep1 = zHeight1 * (1 - cos(sigma)) / 2; 
		float zep2 = zHeight2 * (1 - cos(sigma)) / 2; 
		float zep3 = zHeight3 * (1 - cos(sigma)) / 2; 
		float zep4 = zHeight4 * (1 - cos(sigma)) / 2; 
		M_leg[1].M_expx = xep_b1*leg_Mode[1] ;  //     xep_b摆动腿，xep_z支撑腿
		M_leg[2].M_expx = xep_b2*leg_Mode[2] ;
		M_leg[3].M_expx = xep_b3*leg_Mode[3] ;
		M_leg[4].M_expx = xep_b4*leg_Mode[4] ;
		M_leg[1].M_expz = -heightControlNow1;                     
		M_leg[2].M_expz = 0-heightControlNow2;
		M_leg[3].M_expz = -heightControlNow3;
		M_leg[4].M_expz = 0-heightControlNow4;
		newIK();

   }


  for(t=Ts*faai;t<=Ts;t+=0.025)
		{		
		sigma = 2 * pi * (t - faai * Ts) / (faai * Ts);
					float xep_z2 = (xpre2 - xfinal2) * ((sigma - sin(sigma)) / (2 * pi)) + xfinal2; 
		float xep_z1 = (xpre1 - xfinal1) * ((sigma - sin(sigma)) / (2 * pi)) + xfinal1; 
					float xep_z4 = (xpre4 - xfinal4) * ((sigma - sin(sigma)) / (2 * pi)) + xfinal4; 
		float xep_z3 = (xpre3 - xfinal3) * ((sigma - sin(sigma)) / (2 * pi)) + xfinal3; 
		float zep1 = zHeight1 * (1 - cos(sigma)) / 2; 
		float zep2 = zHeight2 * (1 - cos(sigma)) / 2; 
		float zep3 = zHeight3 * (1 - cos(sigma)) / 2; 
		float zep4 = zHeight4 * (1 - cos(sigma)) / 2; 
			
		M_leg[1].M_expx = xep_z1*leg_Mode[1] ;
		M_leg[2].M_expx = xep_z2*leg_Mode[2] ;
		M_leg[3].M_expx = xep_z3*leg_Mode[3] ;
		M_leg[4].M_expx = xep_z4*leg_Mode[4] ;
		M_leg[1].M_expz = zep1-heightControlNow1;
		M_leg[2].M_expz = zep2-heightControlNow2;
		M_leg[3].M_expz = zep3-heightControlNow3;
		M_leg[4].M_expz = zep4-heightControlNow4;
		newIK();
     } 
}

void trot_testing_running()
{

	float Ts = M_T;//Ts:一个行走周期
	float faai = 0.5;
	float t=0;
  float sigma=0;
	
	float xpre1=xpre;
	float xpre2=xpre;
	float xpre3=xpre+0.02;
	float xpre4=xpre+0.02;
	float xfinal1=xfinal;
	float xfinal2=xfinal;
	float xfinal3=xfinal+0.02;
	float xfinal4=xfinal+0.02;
	float zHeight1=zHeight;
	float zHeight2=zHeight;
	float zHeight3=zHeight;
	float zHeight4=zHeight;
	float heightControlNow1=heightControlNow;
	float heightControlNow2=heightControlNow+0.01;
	float heightControlNow3=heightControlNow+0.0015;
	float heightControlNow4=heightControlNow;
  
	for (t = 0; t <= Ts*faai; t += 0.025)
		{		
		
			sigma = 2 * pi * t / (faai * Ts);
		float xep_b2 = (xfinal2 - xpre2) * ((sigma - sin(sigma)) / (2 * pi)) + xpre2;  
		float xep_z1 = (xpre1 - xfinal1) * ((sigma - sin(sigma)) / (2 * pi)) + xfinal1; 
					float xep_z4 = (xpre4 - xfinal4) * ((sigma - sin(sigma)) / (2 * pi)) + xfinal4; 
		float xep_b3 = (xfinal3 - xpre3) * ((sigma - sin(sigma)) / (2 * pi)) + xpre3;   
		float zep1 = zHeight1 * (1 - cos(sigma)) / 2; 
		float zep2 = zHeight2 * (1 - cos(sigma)) / 2; 
		float zep3 = zHeight3 * (1 - cos(sigma)) / 2; 
		float zep4 = zHeight4 * (1 - cos(sigma)) / 2; 			
		M_leg[1].M_expx = xep_z1*leg_Mode[1] ;
		M_leg[2].M_expx = xep_b2*leg_Mode[2] ;
		M_leg[3].M_expx = xep_b3*leg_Mode[3] ;
		M_leg[4].M_expx = xep_z4*leg_Mode[4] ;
		M_leg[1].M_expz = -heightControlNow1;
		M_leg[2].M_expz = zep2-heightControlNow2;
		M_leg[3].M_expz = zep3-heightControlNow3;
		M_leg[4].M_expz = -heightControlNow4;
		newIK();
     }
		
		 for(t=Ts*faai;t<=Ts;t+=0.025)
	{
		sigma = 2 * pi * (t - faai * Ts) / (faai * Ts);
		float xep_b1 = (xfinal1 - xpre1) * ((sigma - sin(sigma)) / (2 * pi)) + xpre1;    
		float xep_z3 = (xpre3 - xfinal3) * ((sigma - sin(sigma)) / (2 * pi)) + xfinal3; 
		float xep_z2 = (xpre2 - xfinal2) * ((sigma - sin(sigma)) / (2 * pi)) + xfinal2; 
				float xep_b4 = (xfinal4 - xpre4) * ((sigma - sin(sigma)) / (2 * pi)) + xpre4;   

		float zep1 = zHeight1 * (1 - cos(sigma)) / 2; 
		float zep2 = zHeight2 * (1 - cos(sigma)) / 2; 
		float zep3 = zHeight3 * (1 - cos(sigma)) / 2; 
		float zep4 = zHeight4 * (1 - cos(sigma)) / 2; 
		M_leg[1].M_expx = xep_b1*leg_Mode[1] ;  
		M_leg[2].M_expx = xep_z2*leg_Mode[2] ;
		M_leg[3].M_expx = xep_z3*leg_Mode[3] ;
		M_leg[4].M_expx = xep_b4*leg_Mode[4] ;
		M_leg[1].M_expz = zep1-heightControlNow1;                     
		M_leg[2].M_expz = 0-heightControlNow2;
		M_leg[3].M_expz = -heightControlNow3;
		M_leg[4].M_expz = zep4-heightControlNow4;
		newIK();

   }
}


void ramp_walk()
{
		float k1=1,k2=1;
		float error=0;
		__kp[1]=55;
		__kd[1]=15;//45
		__kp[2]=55 ;
		__kd[2]=15;//65
		__kp[3]=55;//51
		__kd[3]=15;//+detla_kp1;
		__kp[4]=55;
		__kd[4]=15;//+detla_kp1 ;
		xpre=-0.050;//-0.015
		xfinal=0.05;//0.05  //0.05 0.05 
		zHeight=0.03;
		heightControlNow=0.25;
		M_T=0.5;//0.4//0.75//0.25
		error=0.0-rx_text.yaw;
	if(fabs(error)<=2.0)
		{
		k1=1;
		k2=1;
		}
		else if(error>2.0)
		{
		k2=0.95;
		k1=1.05;
		}
		else if(error<-2.0)
		{
		k2=1.05;
		k1=0.95;
		}
		k1=1.05;
		k2=0.95;
		
		if(rc.ch4<-100)
		{
		k1=-k1;
		k2=-k2;
		}
		else if(rc.ch1<-100)
		{
			k1=-0.75;
			k2=0.75;
		}
		else if(rc.ch1>100)
		{
			k1=0.75;
			k2=-0.75;
		
		}
 		leg_Mode[1]=(1+detlam2)*k1;
		leg_Mode[2]=(1+detlam2)*k1;
		leg_Mode[3]=(1+detlam1)*k2;
		leg_Mode[4]=(1+detlam1)*k2;
		trot();
}


void ramp_walk_2_0(float target_angle)
{
        float k1=1,k2=1;
				float k=0.0;
        float error=0;
        __kp[1]=55;
        __kd[1]=15;//45
        __kp[2]=55 ;
        __kd[2]=15;//65
        __kp[3]=55;//51
        __kd[3]=15;//+detla_kp1;
        __kp[4]=55;
        __kd[4]=15;//+detla_kp1 ;
        xpre=-0.040;//-0.015
        xfinal=0.04;//0.05  //0.05 0.05 
        zHeight=0.02;
        heightControlNow=0.25;
        M_T=0.3;//0.4//0.75//0.25
					error=pathArray[places_have_reached-1].next_angle+target_angle-rx_text.yaw;
					k=error/30.0;
					if(k<-0.5){
						k=-0.5;
					}
					else if(k>0.5){
						k=0.5;
					}
            if(fabs(error)<=2.0)
            {
            k1=1;
            k2=1;
            }
            else if(error>2.0)
            {
            k2=1-k;;
            k1=1+k;;
            }
            else if(error<-2.0)
            {
            k2=1+k;;
            k1=1-k;;
            }
            leg_Mode[1]=k1;
            leg_Mode[2]=k1;
            leg_Mode[3]=k2;
            leg_Mode[4]=k2;
            trot();
}


void trot_walk_right_or_left()
{
	float Ts = M_T;//Ts:一个行走周期
	float faai = 0.5;
	float t=0;
  float sigma=0;
	float xpre1=xpre-0.0315;
	float xpre2=xpre;
	float xpre3=xpre-0.017;
	float xpre4=xpre;
	float xfinal1=xfinal-0.0315;
	float xfinal2=xfinal;
	float xfinal3=xfinal-0.017;
	float xfinal4=xfinal;
	float zHeight1=zHeight+0.05;
	float zHeight2=zHeight+0.05;
	float zHeight3=zHeight+0.05;
	float zHeight4=zHeight+0.05;
	float heightControlNow1=heightControlNow-0.0096-0.02;
	float heightControlNow2=heightControlNow-0.0096-0.02;
	float heightControlNow3=heightControlNow-0.01;
	float heightControlNow4=heightControlNow;
	for (t = 0; t <= Ts*faai; t += 0.025)
	{
		sigma = 2 * pi * t / (faai * Ts);//sigma范围化成0到2pi
		float xep_b1 = 0;    
		float xep_z2 = 0; 
		float xep_b3 = 0;    
		float xep_z4 = 0;		
		float zep1 = zHeight1 * (1 - cos(sigma)) / 2; 
		float zep2 = zHeight2 * (1 - cos(sigma)) / 2; 
		float zep3 = zHeight3 * (1 - cos(sigma)) / 2; 
		float zep4 = zHeight4 * (1 - cos(sigma)) / 2; 
		M_leg[1].M_expx = xep_b1*leg_Mode[1] ;  //     xep_b摆动腿，xep_z支撑腿
		M_leg[2].M_expx = xep_z2*leg_Mode[2] ;
		M_leg[3].M_expx = xep_b3*leg_Mode[3] ;
		M_leg[4].M_expx = xep_z4*leg_Mode[4] ;
		M_leg[1].M_expz = zep1-heightControlNow1;                     
		M_leg[2].M_expz = 0-heightControlNow2;
		M_leg[3].M_expz = zep3-heightControlNow3;
		M_leg[4].M_expz = 0-heightControlNow4;
		newIK();
   }
  for(t=Ts*faai;t<=Ts;t+=0.025)
		{		
		sigma = 2 * pi * (t - faai * Ts) / (faai * Ts);
		float xep_b2 = 0;    
		float xep_z1 = 0; 
		float xep_b4 = 0;    
		float xep_z3 = 0; 
		float zep1 = zHeight1 * (1 - cos(sigma)) / 2; 
		float zep2 = zHeight2 * (1 - cos(sigma)) / 2; 
		float zep3 = zHeight3 * (1 - cos(sigma)) / 2; 
		float zep4 = zHeight4 * (1 - cos(sigma)) / 2; 
		M_leg[1].M_expx = xep_z1*leg_Mode[1] ;
		M_leg[2].M_expx = xep_b2*leg_Mode[2] ;
		M_leg[3].M_expx = xep_z3*leg_Mode[3] ;
		M_leg[4].M_expx = xep_b4*leg_Mode[4] ;
		M_leg[1].M_expz = 0-heightControlNow1;
		M_leg[2].M_expz = zep2-heightControlNow2;
		M_leg[3].M_expz = 0-heightControlNow3;
		M_leg[4].M_expz = zep4-heightControlNow4;
		newIK();
     } 
}


void trot_JingSuBiHuan()
{
	float Ts = M_T;//Ts:一个行走周期
	float faai = 0.5;
	float t=0;
  float sigma=0;
	float xpre1=xpre-0.0315;
	float xpre2=xpre;
	float xpre3=xpre-0.017-0.02;
	float xpre4=xpre;
	float xfinal1=xfinal-0.0315;
	float xfinal2=xfinal;
	float xfinal3=xfinal-0.017;
	float xfinal4=xfinal;
	float zHeight1=zHeight;
	float zHeight2=zHeight;
	float zHeight3=zHeight;
	float zHeight4=zHeight;
	float heightControlNow1=heightControlNow-0.0096;
	float heightControlNow2=heightControlNow-0.0096;
	float heightControlNow3=heightControlNow-0.01;
	float heightControlNow4=heightControlNow;
	for (t = 0; t <= Ts*faai; t += 0.025)
	{
		sigma = 2 * pi * t / (faai * Ts);//sigma范围化成0到2pi
		float xep_b1 = (xfinal1 - xpre1) * ((sigma - sin(sigma)) / (2 * pi)) + xpre1;    
		float xep_z2 = (xpre2 - xfinal2) * ((sigma - sin(sigma)) / (2 * pi)) + xfinal2; 
		float xep_b3 = (xfinal3 - xpre3) * ((sigma - sin(sigma)) / (2 * pi)) + xpre3;    
		float xep_z4 = (xpre4 - xfinal4) * ((sigma - sin(sigma)) / (2 * pi)) + xfinal4; 
		float zep1 = zHeight1 * (1 - cos(sigma)) / 2; 
		float zep2 = zHeight2 * (1 - cos(sigma)) / 2; 
		float zep3 = zHeight3 * (1 - cos(sigma)) / 2; 
		float zep4 = zHeight4 * (1 - cos(sigma)) / 2; 
		M_leg[1].M_expx = xep_b1*leg_Mode[1] ;  //     xep_b摆动腿，xep_z支撑腿
		M_leg[2].M_expx = xep_z2*leg_Mode[2] ;
		M_leg[3].M_expx = xep_b3*leg_Mode[3] ;
		M_leg[4].M_expx = xep_z4*leg_Mode[4] ;
		M_leg[1].M_expz = zep1-heightControlNow1;                     
		M_leg[2].M_expz = 0-heightControlNow2;
		M_leg[3].M_expz = zep3-heightControlNow3;
		M_leg[4].M_expz = 0-heightControlNow4;
		newIK();
   }
  for(t=Ts*faai;t<=Ts;t+=0.025)
		{		
		sigma = 2 * pi * (t - faai * Ts) / (faai * Ts);
		float xep_b2 = (xfinal2 - xpre2) * ((sigma - sin(sigma)) / (2 * pi)) + xpre2;    
		float xep_z1 = (xpre1 - xfinal1) * ((sigma - sin(sigma)) / (2 * pi)) + xfinal1; 
		float xep_b4 = (xfinal4 - xpre4) * ((sigma - sin(sigma)) / (2 * pi)) + xpre4;    
		float xep_z3 = (xpre3 - xfinal3) * ((sigma - sin(sigma)) / (2 * pi)) + xfinal3; 
		float zep1 = zHeight1 * (1 - cos(sigma)) / 2; 
		float zep2 = zHeight2 * (1 - cos(sigma)) / 2; 
		float zep3 = zHeight3 * (1 - cos(sigma)) / 2; 
		float zep4 = zHeight4 * (1 - cos(sigma)) / 2; 
		M_leg[1].M_expx = xep_z1*leg_Mode[1] ;
		M_leg[2].M_expx = xep_b2*leg_Mode[2] ;
		M_leg[3].M_expx = xep_z3*leg_Mode[3] ;
		M_leg[4].M_expx = xep_b4*leg_Mode[4] ;
		M_leg[1].M_expz = 0-heightControlNow1;
		M_leg[2].M_expz = zep2-heightControlNow2;
		M_leg[3].M_expz = 0-heightControlNow3;
		M_leg[4].M_expz = zep4-heightControlNow4;
		newIK();
     } 
}


void trot_walkUP_DuanQiao()
{
	float Ts = M_T;//Ts:一个行走周期
	float faai = 0.5;
	float t=0;
  float sigma=0;	
	float xpre1=xpre-0.0315-0.07;
	float xpre2=xpre-0.03;
	float xpre3=xpre-0.017-0.07;
	float xpre4=xpre-0.03;
	float xfinal1=xfinal-0.0315-0.07;
	float xfinal2=xfinal-0.05;
	float xfinal3=xfinal-0.017-0.07;
	float xfinal4=xfinal-0.05;
	float zHeight1=zHeight;
	float zHeight2=zHeight;
	float zHeight3=zHeight;
	float zHeight4=zHeight;
	float heightControlNow1=heightControlNow-0.0096;
	float heightControlNow2=heightControlNow-0.0096+0.05;
	float heightControlNow3=heightControlNow-0.01+0.05;
	float heightControlNow4=heightControlNow;	
	for (t = 0; t <= Ts*faai; t += 0.025)
	{
		sigma = 2 * pi * t / (faai * Ts);//sigma范围化成0到2pi
		float xep_b1 = (xfinal1 - xpre1) * ((sigma - sin(sigma)) / (2 * pi)) + xpre1;    
		float xep_z2 = (xpre2 - xfinal2) * ((sigma - sin(sigma)) / (2 * pi)) + xfinal2; 
		float xep_b3 = (xfinal3 - xpre3) * ((sigma - sin(sigma)) / (2 * pi)) + xpre3;    
		float xep_z4 = (xpre4 - xfinal4) * ((sigma - sin(sigma)) / (2 * pi)) + xfinal4; 
		float zep1 = zHeight1 * (1 - cos(sigma)) / 2; 
		float zep2 = zHeight2 * (1 - cos(sigma)) / 2; 
		float zep3 = zHeight3 * (1 - cos(sigma)) / 2; 
		float zep4 = zHeight4 * (1 - cos(sigma)) / 2; 
		M_leg[1].M_expx = xep_b1*leg_Mode[1] ;  //     xep_b摆动腿，xep_z支撑腿
		M_leg[2].M_expx = xep_z2*leg_Mode[2] ;
		M_leg[3].M_expx = xep_b3*leg_Mode[3] ;
		M_leg[4].M_expx = xep_z4*leg_Mode[4] ;
		M_leg[1].M_expz = zep1-heightControlNow1;                     
		M_leg[2].M_expz = 0-heightControlNow2;
		M_leg[3].M_expz = zep3-heightControlNow3;
		M_leg[4].M_expz = 0-heightControlNow4;
		newIK();		
   }
  for(t=Ts*faai;t<=Ts;t+=0.025)
		{		
		sigma = 2 * pi * (t - faai * Ts) / (faai * Ts);
		float xep_b2 = (xfinal2 - xpre2) * ((sigma - sin(sigma)) / (2 * pi)) + xpre2;    
		float xep_z1 = (xpre1 - xfinal1) * ((sigma - sin(sigma)) / (2 * pi)) + xfinal1; 
		float xep_b4 = (xfinal4 - xpre4) * ((sigma - sin(sigma)) / (2 * pi)) + xpre4;    
		float xep_z3 = (xpre3 - xfinal3) * ((sigma - sin(sigma)) / (2 * pi)) + xfinal3; 
		float zep1 = zHeight1 * (1 - cos(sigma)) / 2; 
		float zep2 = zHeight2 * (1 - cos(sigma)) / 2; 
		float zep3 = zHeight3 * (1 - cos(sigma)) / 2; 
		float zep4 = zHeight4 * (1 - cos(sigma)) / 2; 
		M_leg[1].M_expx = xep_z1*leg_Mode[1] ;
		M_leg[2].M_expx = xep_b2*leg_Mode[2] ;
		M_leg[3].M_expx = xep_z3*leg_Mode[3] ;
		M_leg[4].M_expx = xep_b4*leg_Mode[4] ;
		M_leg[1].M_expz = 0-heightControlNow1;
		M_leg[2].M_expz = zep2-heightControlNow2;
		M_leg[3].M_expz = 0-heightControlNow3;
		M_leg[4].M_expz = zep4-heightControlNow4;
		newIK();
     } 
}

void high_leg_prepare()
{
	float Ts = M_T;//Ts:一个行走周期
	float faai = 0.5;
	float t=0;
  float sigma=0;
			sigma = 2 * pi * t / (faai * Ts);//sigma范围化成0到2pi
		float xep_b = (xfinal - xpre) * ((sigma - sin(sigma)) / (2 * pi)) + xpre;    
		float xep_z = (xpre - xfinal) * ((sigma - sin(sigma)) / (2 * pi)) + xfinal;  
		float zep = zHeight * (1 - cos(sigma)) / 2;  
		M_leg[1].M_expx = xep_b*leg_Mode[1] ;  //     xep_b摆动腿，xep_z支撑腿
		M_leg[2].M_expx = xep_z*leg_Mode[2] ;
		M_leg[3].M_expx = xep_b*leg_Mode[3] ;
		M_leg[4].M_expx = xep_z*leg_Mode[4] ;
		M_leg[1].M_expz = zep-heightControlNow;                     
		M_leg[2].M_expz = 0-heightControlNow;
		M_leg[3].M_expz = zep-heightControlNow;
		M_leg[4].M_expz = 0-heightControlNow;
		newIK();
}

//*******************************************************************************//
//*****************************4.动作原语与特技***********************************//
//a.跳跃。通过瞬间改变腿部期望位置和PID刚度 (__kp 设为极大) 来产生爆发力。区分大跳和小跳。
void big_jump(){
	__kp[1]=12;
		__kd[1]=100;
		__kp[2]=12;
		__kd[2]=100;
		__kp[3]=12;
		__kd[3]=100;
		__kp[4]=12;
		__kd[4]=100;
for(int legnumber =1;legnumber<2;legnumber++)	
		{
		 M_leg[legnumber].M_expx=-0.04;
		 M_leg[legnumber].M_expz=-0.14;	
		}
		for(int legnumber =2;legnumber<3;legnumber++)	
		{
		 M_leg[legnumber].M_expx=-0.053;
		 M_leg[legnumber].M_expz=-0.135;	
		}
		for(int legnumber =3;legnumber<4;legnumber++)	
		{
		 M_leg[legnumber].M_expx=-0.055;
		 M_leg[legnumber].M_expz=-0.14;	
		}
		for(int legnumber =4;legnumber<5;legnumber++)	
		{
		 M_leg[legnumber].M_expx=-0.04;
		 M_leg[legnumber].M_expz=-0.14;	
		}
		__kp[1]=17;
		__kd[1]=30;
		__kp[2]=17;
		__kd[2]=30;
		__kp[3]=17;
		__kd[3]=30;
		__kp[4]=17;
		__kd[4]=30;
		newIK();
		HAL_Delay(1500);
		M_T=0.2;//0.2
		hurdle();
		HAL_Delay(500);
		heightControlNow=0.25;
    trotDirectionState=0;
}

void small_jump(){
	__kp[1]=12;
		__kd[1]=100;
		__kp[2]=12;
		__kd[2]=100;
		__kp[3]=12;
		__kd[3]=100;
		__kp[4]=12;
		__kd[4]=100;
		for(int legnumber =1;legnumber<2;legnumber++)	
		{
		 M_leg[legnumber].M_expx=-0.06;
		 M_leg[legnumber].M_expz=-0.15;	
		}
		for(int legnumber =2;legnumber<3;legnumber++)	
		{
		 M_leg[legnumber].M_expx=-0.07;
		 M_leg[legnumber].M_expz=-0.15;	
		}
		for(int legnumber =3;legnumber<4;legnumber++)	
		{
		 M_leg[legnumber].M_expx=-0.07;
		 M_leg[legnumber].M_expz=-0.15;	
		}
		for(int legnumber =4;legnumber<5;legnumber++)	
		{
		 M_leg[legnumber].M_expx=-0.06;
		 M_leg[legnumber].M_expz=-0.15;	
		}
		__kp[1]=17;
		__kd[1]=30;
		__kp[2]=17;
		__kd[2]=30;
		__kp[3]=17;
		__kd[3]=30;
		__kp[4]=17;
		__kd[4]=30;
		newIK();
		HAL_Delay(500);
		M_T=0.2;//0.2
		louti2();
		HAL_Delay(500);
		heightControlNow=0.25;
}


//b.横向跳跃。调整 M_leg 的侧向参数实现侧跳。
void jump_right(){
	
		__kp[1]=70;//12
		__kd[1]=5;
		__kp[2]=70;//12
		__kd[2]=5;
		__kp[3]=70;//14
		__kd[3]=5;
		__kp[4]=70;
		__kd[4]=5;
		 M_leg[1].M_expx=0;
		 M_leg[1].M_expz=-0.24;
		 M_leg[2].M_expx=0;
		 M_leg[2].M_expz=-0.24;
		 M_leg[3].M_expx=0;
		 M_leg[3].M_expz=-0.22;
		 M_leg[4].M_expx=0;
		 M_leg[4].M_expz=-0.22;
		newIK();
	HAL_Delay(500);
	M_leg[1].M_expx=0;
		 M_leg[1].M_expz=-0.27;
		 M_leg[2].M_expx=0;
		 M_leg[2].M_expz=-0.27;
		 M_leg[3].M_expx=0;
		 M_leg[3].M_expz=-0.25;
		 M_leg[4].M_expx=0;
		 M_leg[4].M_expz=-0.25;
		newIK();
	HAL_Delay(10);
}
void jump_left(){
	
		__kp[1]=70;//12
		__kd[1]=5;
		__kp[2]=70;//12
		__kd[2]=5;
		__kp[3]=70;//14
		__kd[3]=5;
		__kp[4]=70;
		__kd[4]=5;
		 M_leg[3].M_expx=0+0.01;
		 M_leg[3].M_expz=-0.28;
		 M_leg[4].M_expx=0+0.01;
		 M_leg[4].M_expz=-0.28;
		 M_leg[1].M_expx=0-0.01;
		 M_leg[1].M_expz=-0.19;
		 M_leg[2].M_expx=0-0.01;
		 M_leg[2].M_expz=-0.19;

		newIK();
	HAL_Delay(500);
		 M_leg[3].M_expx=0+0.01;
		 M_leg[3].M_expz=-0.31;
		 M_leg[4].M_expx=0+0.01;
		 M_leg[4].M_expz=-0.31;
		 M_leg[1].M_expx=0-0.01;
		 M_leg[1].M_expz=-0.22;
		 M_leg[2].M_expx=0-0.01;
		 M_leg[2].M_expz=-0.22;
		
		newIK();
	HAL_Delay(10);
}


//c.绕杆。根据当前角度误差调整左右腿步幅 (leg_Mode)，实现绕S弯或圆周运动。
void raogan(int flag)
{
		float error1=0;
		float error2=0;
		float error3=0;
		float k1=0;
		float k2=0;
		__kp[1]=55;
		__kd[1]=35;//45
		__kp[2]=55 ;
		__kd[2]=35;//65
		__kp[3]=54;//51
		__kd[3]=35;//+detla_kp1;
		__kp[4]=54;
		__kd[4]=35;//+detla_kp1 ;
		xpre=-0.04;//-0.015
		xfinal=0.04;//0.05  //0.05 0.05 
		zHeight=0.02;
		heightControlNow=0.24;//0.25
		M_T=0.5;
		error1=rx_text.yaw-0.0;
		error2=rx_text.yaw-(-45.0);
		error3=rx_text.yaw-(45.0);
		if(fabs(error1)<=2.0 && flag==1)
		{
		k1=1;
		k2=1;
		}
		else if(error1>2.0 && flag==1)
		{
		k2=0.4;
		k1=1.6;
		}
		else if(error1<-2.0 && flag==1)
		{
		k2=1.6;
		k1=0.4;
		} 
		if(fabs(error2)<=2.0 && flag==2)
		{
		k1=1;
		k2=1;
		}
		else if(error2>2.0 && flag==2)
		{
		k2=0.4;
		k1=1.6;
		}
		else if(error2<-2.0 && flag==2)
		{
		k2=1.6;
		k1=0.4;
		} 
		if(fabs(error3)<=2.0 && flag==3)
		{
		k1=1;
		k2=1;
		}
		else if(error3>2.0 && flag==3)
		{
		k2=0.4;
		k1=1.6;
		}
		else if(error3<-2.0 && flag==3)
		{
		k2=1.6;
		k1=0.4;
		} 
 		leg_Mode[1]=(1+detlam2)*k1;
		leg_Mode[2]=(1+detlam2)*k1;
		leg_Mode[3]=(1+detlam1)*k2;
		leg_Mode[4]=(1+detlam1)*k2;
		trot();
}

void high_speed_raogan(int flag)
{
		float error1=0;
		float error2=0;
		float error3=0;
		float k1=0;
		float k2=0;
		__kp[1]=55;
		__kd[1]=35;//45
		__kp[2]=55 ;
		__kd[2]=35;//65
		__kp[3]=55;//51
		__kd[3]=35;//+detla_kp1;
		__kp[4]=55;
		__kd[4]=35;//+detla_kp1 ;
		xpre=-0.055;//-0.015
		xfinal=0.055;//0.05  //0.05 0.05 
		zHeight=0.02;
		heightControlNow=0.25;//0.25
		M_T=0.5;
		error1=gyro_angle-0.0;
		error2=gyro_angle-(-45.0);
		error3=gyro_angle-(45.0);
		if(fabs(error1)<=2.0 && flag==1)
		{
		k1=1;
		k2=1;
		}
		else if(error1>2.0 && flag==1)
		{
		k2=0.4;
		k1=1.6;
		}
		else if(error1<-2.0 && flag==1)
		{
		k2=1.6;
		k1=0.4;
		} 
		if(fabs(error2)<=2.0 && flag==2)
		{
		k1=1;
		k2=1;
		}
		else if(error2>2.0 && flag==2)
		{
		k2=0.4;
		k1=1.6;
		}
		else if(error2<-2.0 && flag==2)
		{
		k2=1.6;
		k1=0.4;
		} 
		if(fabs(error3)<=2.0 && flag==3)
		{
		k1=1;
		k2=1;
		}
		else if(error3>2.0 && flag==3)
		{
		k2=0.4;
		k1=1.6;
		}
		else if(error3<-2.0 && flag==3)
		{
		k2=1.6;
		k1=0.4;
		} 
 		leg_Mode[1]=(1+detlam2)*k1;
		leg_Mode[2]=(1+detlam2)*k1;
		leg_Mode[3]=(1+detlam1)*k2;
		leg_Mode[4]=(1+detlam1)*k2;
		trot();
}


//d.固定绕杆。硬编码的步数和转向序列，用于比赛中的固定绕杆项目。
void slalom(void){
		__kp[1]=55;
		__kd[1]=35;//45
		__kp[2]=55 ;
		__kd[2]=35;//65
		__kp[3]=55;//51
		__kd[3]=35;//+detla_kp1;
		__kp[4]=55;
		__kd[4]=35;//+detla_kp1 ;
		xpre=-0.06;//-0.015
		xfinal=0.06;//0.05  //0.05 0.05 
		zHeight=0.06;
		heightControlNow=0.25;
		M_T=1.5;
		leg_Mode[3]=2+detlam1;
		leg_Mode[4]=2+detlam1;
 		leg_Mode[1]=0.35+detlam2;
		leg_Mode[2]=0.35+detlam2;
		for(int i=0;i<8;i++){
		trot();
		}
		leg_Mode[3]=0.35+detlam1;
		leg_Mode[4]=0.35+detlam1;
 		leg_Mode[1]=2+detlam2;
		leg_Mode[2]=2+detlam2;
		for(int i=0;i<9;i++){
			trot();
		}
		leg_Mode[3]=2+detlam1;
		leg_Mode[4]=2+detlam1;
 		leg_Mode[1]=0.35+detlam2;
		leg_Mode[2]=0.35+detlam2;
		for(int i=0;i<8;i++){
		trot();
		}
		leg_Mode[3]=0.35+detlam1;
		leg_Mode[4]=0.35+detlam1;
 		leg_Mode[1]=2+detlam2;
		leg_Mode[2]=2+detlam2;
		for(int i=0;i<9;i++){
		trot();
		}
		leg_Mode[3]=2+detlam1;
		leg_Mode[4]=2+detlam1;
 		leg_Mode[1]=0.35+detlam2;
		leg_Mode[2]=0.35+detlam2;
		for(int i=0;i<9;i++){
		trot();
		}
	}

//e.断桥后退。在断桥上执行后退步态。
void back_on_DuanQiao(int steps){
		__kp[1]=10;
		__kd[1]=35;//45
		__kp[2]=10 ;
		__kd[2]=35;//65
		__kp[3]=10;//51
		__kd[3]=35;//+detla_kp1;
		__kp[4]=10;
		__kd[4]=35;//+detla_kp1 ;
		xpre=-0.02;//-0.015
		xfinal=0.02;//0.05  //0.05 0.05 
		zHeight=0.07;
		heightControlNow=0.25;
		M_T=1.5;
 		leg_Mode[1]=-1+detlam2;
		leg_Mode[2]=-1+detlam2;
		leg_Mode[3]=-1+detlam1;
		leg_Mode[4]=-1+detlam1;
		for(int i=0;i<steps;i++){
			trot_walkUP_DuanQiao();
		}
}

//f.定点转向。一侧腿前划，一侧腿后划，实现原地转向。
void Pivot_turn_left(){//左转一圈
		__kp[1]=55;
		__kd[1]=35;//45
		__kp[2]=55 ;
		__kd[2]=35;//65
		__kp[3]=55;//51
		__kd[3]=35;//+detla_kp1;
		__kp[4]=55;
		__kd[4]=35;//+detla_kp1 ;
		xpre=-0.06;//-0.015
		xfinal=0.06;//0.05  //0.05 0.05 
		zHeight=0.06;
		heightControlNow=0.25;
		M_T=1.5;//0.4//0.75
		leg_Mode[3]=1+detlam1;
		leg_Mode[4]=1+detlam1;
 		leg_Mode[1]=-1+detlam2;
		leg_Mode[2]=-1+detlam2;
		for(int i=0;i<8;i++){
			trot();
		}
}

void Pivot_turn_right(){//右转一圈
		__kp[1]=55;
		__kd[1]=35;//45
		__kp[2]=55 ;
		__kd[2]=35;//65
		__kp[3]=55;//51
		__kd[3]=35;//+detla_kp1;
		__kp[4]=55;
		__kd[4]=35;//+detla_kp1 ;
		xpre=-0.06;//-0.015
		xfinal=0.06;//0.05  //0.05 0.05 
		zHeight=0.06;
		heightControlNow=0.25;
		M_T=1.5;//0.4//0.75
		leg_Mode[3]=1+detlam1;
		leg_Mode[4]=1+detlam1;
 		leg_Mode[1]=-1+detlam2;
		leg_Mode[2]=-1+detlam2;
		for(int i=0;i<8;i++){
			trot();
		}
}

//else
void shakeng_with_Gyro(int flag)
{
		float error1=0;
		float k1=0;
		float k2=0;
		__kp[1]=55;
		__kd[1]=35;//45
		__kp[2]=55 ;
		__kd[2]=35;//65
		__kp[3]=54;//51
		__kd[3]=35;//+detla_kp1;
		__kp[4]=54;
		__kd[4]=35;//+detla_kp1 ;
		xpre=-0.07;//-0.015
		xfinal=0.07;//0.05  //0.05 0.05 
		zHeight=0.1;
		heightControlNow=0.24;//0.25//0.32
		M_T=0.6;//0.4//0.75//0.55
		error1=gyro_angle-0.0;
		if(fabs(error1)<=2.0 && flag==1)
		{
		k1=1;
		k2=1;
		}
		else if(error1>2.0 && flag==1)
		{
		k2=0.4;
		k1=1.6;
		}
		else if(error1<-2.0 && flag==1)
		{
		k2=1.6;
		k1=0.4;
		} 
 		leg_Mode[1]=(1+detlam2)*k1;
		leg_Mode[2]=(1+detlam2)*k1;
		leg_Mode[3]=(1+detlam1)*k2;
		leg_Mode[4]=(1+detlam1)*k2;
		trot();
}

void shakeng_high_knee(int step){
		__kp[1]=15;
		__kd[1]=15;//45
		__kp[2]=15 ;
		__kd[2]=15;//65
		__kp[3]=15;//51
		__kd[3]=15;//+detla_kp1;
		__kp[4]=15;
		__kd[4]=15;//+detla_kp1 ;
		xpre=-0.12;//-0.015
		xfinal=0.12;//0.05  //0.05 0.05 
		zHeight=0.02;
		heightControlNow=0.32;
		M_T=0.5;//0.4//0.75
 		leg_Mode[1]=1;
		leg_Mode[2]=1;
		leg_Mode[3]=1;
		leg_Mode[4]=1;
		
		high_leg_prepare();
		HAL_Delay(500);
		for(int i=0;i<step;i++){
			high_knee_walk();
		}
}

void sandBox_all(){//沙坑完整
	big_jump();
	int i=0;
	shakeng_high_knee(5);
	HAL_Delay(500);
	for(i=0;i<4;i++){
		small_jump();
	}
}

void jump_best()
{
		__kp[1]=12;
		__kd[1]=100;
		__kp[2]=12;
		__kd[2]=100;
		__kp[3]=12;
		__kd[3]=100;
		__kp[4]=12;
		__kd[4]=100;
		for(int legnumber =1;legnumber<2;legnumber++)	
		{
		 M_leg[legnumber].M_expx=-0.06;
		 M_leg[legnumber].M_expz=-0.13;	
		}
		for(int legnumber =2;legnumber<3;legnumber++)	
		{
		 M_leg[legnumber].M_expx=-0.07;
		 M_leg[legnumber].M_expz=-0.13;	
		}
		for(int legnumber =3;legnumber<4;legnumber++)	
		{
		 M_leg[legnumber].M_expx=-0.07;
		 M_leg[legnumber].M_expz=-0.13;	
		}
		for(int legnumber =4;legnumber<5;legnumber++)	
		{
		 M_leg[legnumber].M_expx=-0.06;
		 M_leg[legnumber].M_expz=-0.13;	
		}
		__kp[1]=17;
		__kd[1]=30;
		__kp[2]=17;
		__kd[2]=30;
		__kp[3]=17;
		__kd[3]=30;
		__kp[4]=17;
		__kd[4]=30;
		newIK();
		HAL_Delay(1000);
		M_T=0.2;//0.2
		hurdle();//1.已将腾空保持末位的kp,kd改为落地缓冲  //2.可以在贝塞尔曲线阶段加微量延时，使过栏时后脚处于一个比较偏前上的位置（这样同时也要改一下贝塞尔曲线的末尾速度，使过栏时刚好通过最宽裕的位置）
		HAL_Delay(500);
		heightControlNow=0.25;
      trotDirectionState=0;
}


void jump_middle()
{
	__kp[1]=12;
		__kd[1]=100;
		__kp[2]=12;
		__kd[2]=100;
		__kp[3]=12;
		__kd[3]=100;
		__kp[4]=12;
		__kd[4]=100;
		for(int legnumber =1;legnumber<2;legnumber++)	
		{
		 M_leg[legnumber].M_expx=-0.06;
		 M_leg[legnumber].M_expz=-0.14;	
		}
		for(int legnumber =2;legnumber<3;legnumber++)	
		{
		 M_leg[legnumber].M_expx=-0.07;
		 M_leg[legnumber].M_expz=-0.14;	
		}
		for(int legnumber =3;legnumber<4;legnumber++)	
		{
		 M_leg[legnumber].M_expx=-0.07;
		 M_leg[legnumber].M_expz=-0.14;	
		}
		for(int legnumber =4;legnumber<5;legnumber++)	
		{
		 M_leg[legnumber].M_expx=-0.06;
		 M_leg[legnumber].M_expz=-0.14;	
		}
		__kp[1]=17;
		__kd[1]=30;
		__kp[2]=17;
		__kd[2]=20;
		__kp[3]=17;
		__kd[3]=20;
		__kp[4]=17;
		__kd[4]=30;
		newIK();
		HAL_Delay(500);
		M_T=0.2;//0.2
		louti_middle();
		HAL_Delay(500);
		heightControlNow=0.25;
      trotDirectionState=0;
}



void jump_small()
{
	
		__kp[1]=12;
		__kd[1]=100;
		__kp[2]=12;
		__kd[2]=100;
		__kp[3]=12;
		__kd[3]=100;
		__kp[4]=12;
		__kd[4]=100;
		for(int legnumber =1;legnumber<2;legnumber++)	
		{
		 M_leg[legnumber].M_expx=-0.06;
		 M_leg[legnumber].M_expz=-0.15;	
		}
		for(int legnumber =2;legnumber<3;legnumber++)	
		{
		 M_leg[legnumber].M_expx=-0.07;
		 M_leg[legnumber].M_expz=-0.15;	
		}
		for(int legnumber =3;legnumber<4;legnumber++)	
		{
		 M_leg[legnumber].M_expx=-0.07;
		 M_leg[legnumber].M_expz=-0.15;	
		}
		for(int legnumber =4;legnumber<5;legnumber++)	
		{
		 M_leg[legnumber].M_expx=-0.06;
		 M_leg[legnumber].M_expz=-0.15;	
		}
		__kp[1]=17;
		__kd[1]=30;
		__kp[2]=17;
		__kd[2]=20;
		__kp[3]=17;
		__kd[3]=20;
		__kp[4]=17;
		__kd[4]=30;
		newIK();
		HAL_Delay(500);
		M_T=0.2;//0.2
		louti2();
		HAL_Delay(500);
		heightControlNow=0.25;
      trotDirectionState=0;
}
void real_ramp_jump(){
	__kp[1]=12;
		__kd[1]=100;
		__kp[2]=12;
		__kd[2]=100;
		__kp[3]=12;
		__kd[3]=100;
		__kp[4]=12;
		__kd[4]=100;
for(int legnumber =1;legnumber<2;legnumber++)	
		{
		 M_leg[legnumber].M_expx=-0.04;
		 M_leg[legnumber].M_expz=-0.14;	
		}
		for(int legnumber =2;legnumber<3;legnumber++)	
		{
		 M_leg[legnumber].M_expx=-0.053;
		 M_leg[legnumber].M_expz=-0.135;	
		}
		for(int legnumber =3;legnumber<4;legnumber++)	
		{
		 M_leg[legnumber].M_expx=-0.055;
		 M_leg[legnumber].M_expz=-0.14;	
		}
		for(int legnumber =4;legnumber<5;legnumber++)	
		{
		 M_leg[legnumber].M_expx=-0.04;
		 M_leg[legnumber].M_expz=-0.14;	
		}
		__kp[1]=17;
		__kd[1]=30;
		__kp[2]=17;
		__kd[2]=30;
		__kp[3]=17;
		__kd[3]=30;
		__kp[4]=17;
		__kd[4]=30;
		newIK();
		HAL_Delay(1500);
		M_T=0.2;//0.2
		ramp_hurdle();
		HAL_Delay(500);
		heightControlNow=0.25;
}

void dizipufu_all(){
	pufu(8);
	rotate_in_place_to_targetAngle(35);
	pufu(6);
	rotate_in_place_to_targetAngle(0);
	pufu(10);
}
//*******************************************************************************//
//*****************************5.辅助计算函数************************************//
double distance_point_to_vector(double ax, double ay, double bx, double by, double px, double py) {
    // 向量 AB 的分量
    double vector_ab_x = bx - ax;
    double vector_ab_y = by - ay;

    // 向量 AP 的分量
    double vector_ap_x = px - ax;
    double vector_ap_y = py - ay;

    // 叉乘 |AP × AB|
    double cross_product = vector_ap_x * vector_ab_y - vector_ap_y * vector_ab_x;

    // 向量 AB 的模长
    double length_ab = sqrt(vector_ab_x * vector_ab_x + vector_ab_y * vector_ab_y);

    // 距离 = |AP × AB| / |AB|含正负号，不是距离
    return cross_product / length_ab;
}

double distance_to_perpendicular_line(double x1, double y1, double x2, double y2) {//到垂线距离
    double dx = x2 - x1;
    double dy = y2 - y1;
    // 直线方程为: dx * x + dy * y + c = 0
    double c = -(dx * x2 + dy * y2);
    // 使用点到直线 Ax + By + C = 0 的距离公式
    double numerator = fabs(dx * x1 + dy * y1 + c);
    double denominator = sqrt(dx * dx + dy * dy);

    if (denominator == 0) {
        // 如果两点重合，返回 0 或者报错
        return 0.0; // 或者可以返回 -1 表示错误
    }
    return numerator / denominator;
}






