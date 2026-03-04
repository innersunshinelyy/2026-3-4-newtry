#include "newIK.h"
#include "math.h"
#include "Yobotics.h"

///
/// legNumber--order x,y,z--position after newIK you should get angle[0] and angle[1] to control 
///
struct M_ANGLE M_angle[5];
//float __kp[5];
//float __kd[5];
extern float __kp[5];
extern float __kd[5];
struct moment_force MomentForce[5]={0};
struct canshu M_leg[5];
	extern float __kpp[5];
	extern float __kdd[5];

/**
  * @brief  逆运动学解算核心函数
  * @param  无（通过读取全局结构体 M_leg[legnumber].M_expx/z 获取输入）
  * @retval 无（通过修改全局结构体 M_angle[legnumber] 输出角度，并通过 CAN 发送）
  * @用法及调用要求  
  * 1. 在步态生成 (trot_run) 或特殊动作 (qiaoqiao_jump2) 计算出期望足端位置后立即调用。
  * 2. 函数内部遍历四条腿 (legnumber 1-4)。
  * @其它  
  * **核心算法**：利用 atan 和 acos（基于余弦定理）计算出大腿侧摆角 (gamma) 和大腿-小腿弯曲角 (faai) 等目标角度。解算完成后，直接调用 Cal_Yobotics_Data/Data2 发送指令，构成闭环。
  */
void newIK()
{
//HAL_Delay(50);
	for (int legnumber=1;legnumber<5;legnumber++)
	{
	
		double gamma=atan(M_leg[legnumber].M_expx/(-M_leg[legnumber].M_expz));
		double faai =acos((L1*L1+M_leg[legnumber].M_expx*M_leg[legnumber].M_expx+M_leg[legnumber].M_expz*M_leg[legnumber].M_expz-L2*L2)
			/2/sqrt(M_leg[legnumber].M_expx*M_leg[legnumber].M_expx+M_leg[legnumber].M_expz*M_leg[legnumber].M_expz)/L1);
		M_angle[legnumber].angle_0=gamma+faai;
		M_angle[legnumber].angle_1=-gamma+faai;
		//printf("0 %f 1 %f\n",M_angle[legnumber].angle_0,M_angle[legnumber].angle_1);
	}
		//printf("0 %f 1 %f    ",M_angle[1].angle_0,M_angle[1].angle_1);
//		Cal_Yobotics_Data ( M_angle[1].angle_0,0,__kp[1],__kd[1],MomentForce[1].force_0,1,1); //force_0为内侧电机
//		Cal_Yobotics_Data ( M_angle[1].angle_1,0,__kp[1],__kd[1],MomentForce[1].force_1,1,2);
//		Cal_Yobotics_Data2(-M_angle[4].angle_0,0,__kp[4],__kd[4],MomentForce[4].force_0,2,1); //force_0为内侧电机
//		Cal_Yobotics_Data2(-M_angle[4].angle_1,0,__kp[4],__kd[4],MomentForce[4].force_1,2,2);
//		
//		Cal_Yobotics_Data2(-M_angle[3].angle_0,0,__kp[3],__kd[3],MomentForce[3].force_0,1,1); //force_0为内侧电机
//		Cal_Yobotics_Data2(-M_angle[3].angle_1,0,__kp[3],__kd[3],MomentForce[3].force_1,1,2);
//		Cal_Yobotics_Data ( M_angle[2].angle_0,0,__kp[2],__kd[2],MomentForce[2].force_0,2,1); //force_0为内侧电机
//		Cal_Yobotics_Data ( M_angle[2].angle_1,0,__kp[2],__kd[2],MomentForce[2].force_1,2,2);
	////////////////////////////////////////////////////////////////////////////////////////////////////////////学院派
//		Cal_Yobotics_Data ( M_angle[1].angle_0+0.0,0,__kp[1],__kd[1],MomentForce[1].force_0,1,1); //force_0为内侧电机
//		Cal_Yobotics_Data ( M_angle[1].angle_1+0.95,0,__kp[1],__kd[1],MomentForce[1].force_1,1,2);
//		
//		Cal_Yobotics_Data ( M_angle[2].angle_0-0.02,0,__kp[2],__kd[2],MomentForce[2].force_0,2,1); //force_0为内侧电机
//		Cal_Yobotics_Data ( M_angle[2].angle_1-0.04,0,__kp[2],__kd[2],MomentForce[2].force_1,2,2);
//	
//		Cal_Yobotics_Data2(-M_angle[3].angle_0+0.06,0,__kp[3],__kd[3],MomentForce[3].force_0,1,1); //force_0为内侧电机
//		Cal_Yobotics_Data2(-M_angle[3].angle_1,0,__kp[3],__kd[3],MomentForce[3].force_1,1,2);
//		
//		Cal_Yobotics_Data2(-M_angle[4].angle_0-0.0,0,__kp[4],__kd[4],MomentForce[4].force_0,2,1);
//		Cal_Yobotics_Data2(-M_angle[4].angle_1+0.17,0,__kp[4],__kd[4],MomentForce[4].force_1,2,2); //force_0为内侧电机
//	////////////////////////////////////////////////////////////////////////////////////////////////////////////////力大砖飞派
			Cal_Yobotics_Data ( M_angle[1].angle_0,0,__kp[1],__kd[1],MomentForce[1].force_0,1,1); //force_0为内侧电机
		Cal_Yobotics_Data ( M_angle[1].angle_1,0,__kp[1],__kd[1],MomentForce[1].force_1,1,2);
		
		Cal_Yobotics_Data ( M_angle[2].angle_0,0,__kp[2],__kd[2],MomentForce[2].force_0,2,1); //force_0为内侧电机
		Cal_Yobotics_Data ( M_angle[2].angle_1,0,__kp[2],__kd[2],MomentForce[2].force_1,2,2);
	
		Cal_Yobotics_Data2(-M_angle[3].angle_0,0,__kp[3],__kd[3],MomentForce[3].force_0,1,1); //force_0为内侧电机
		Cal_Yobotics_Data2(-M_angle[3].angle_1,0,__kp[3],__kd[3],MomentForce[3].force_1,1,2);
		
		Cal_Yobotics_Data2(-M_angle[4].angle_0,0,__kp[4],__kd[4],MomentForce[4].force_0,2,1);
		Cal_Yobotics_Data2(-M_angle[4].angle_1,0,__kp[4],__kd[4],MomentForce[4].force_1,2,2); //force_0为内侧电机

	
	
	
	
	
//		Cal_Yobotics_Data2(-M_angle[4].angle_0,0,__kp[4],__kd[4],MomentForce[4].force_1,2,2);
//		Cal_Yobotics_Data2(-M_angle[4].angle_1,0,__kp[4],__kd[4],MomentForce[4].force_0,2,1); //force_0为内侧电机
//		
//		Cal_Yobotics_Data ( M_angle[1].angle_0,0,__kp[1],__kd[1],MomentForce[1].force_0,1,1); //force_0为内侧电机
//		Cal_Yobotics_Data ( M_angle[1].angle_1,0,__kp[1],__kd[1],MomentForce[1].force_1,1,2);
//		
//		Cal_Yobotics_Data2(M_angle[3].angle_1,0,__kp[3],__kd[3],MomentForce[3].force_0,1,1); //force_0为内侧电机
//		Cal_Yobotics_Data2(M_angle[3].angle_0,0,__kp[3],__kd[3],MomentForce[3].force_1,1,2);
//		
//		Cal_Yobotics_Data ( -M_angle[2].angle_1,0,__kp[2],__kd[2],MomentForce[2].force_0,2,1); //force_0为内侧电机
//		Cal_Yobotics_Data ( -M_angle[2].angle_0,0,__kp[2],__kd[2],MomentForce[2].force_1,2,2);
		
}

	void TempNewIK()
{
//HAL_Delay(50);
	for (int legnumber=1;legnumber<5;legnumber++)
	{
	
		double gamma=atan(M_leg[legnumber].M_expx/(-M_leg[legnumber].M_expz));
		double faai =acos((L1*L1+M_leg[legnumber].M_expx*M_leg[legnumber].M_expx+M_leg[legnumber].M_expz*M_leg[legnumber].M_expz-L2*L2)
			/2/sqrt(M_leg[legnumber].M_expx*M_leg[legnumber].M_expx+M_leg[legnumber].M_expz*M_leg[legnumber].M_expz)/L1);
		M_angle[legnumber].angle_0=gamma+faai;
		M_angle[legnumber].angle_1=-gamma+faai;
		//printf("0 %f 1 %f\n",M_angle[legnumber].angle_0,M_angle[legnumber].angle_1);
	}
		//printf("0 %f 1 %f    ",M_angle[1].angle_0,M_angle[1].angle_1);
//		Cal_Yobotics_Data ( M_angle[1].angle_0,0,__kp[1],__kd[1],MomentForce[1].force_0,1,1); //force_0为内侧电机
//		Cal_Yobotics_Data ( M_angle[1].angle_1,0,__kp[1],__kd[1],MomentForce[1].force_1,1,2);
//		Cal_Yobotics_Data2(-M_angle[4].angle_0,0,__kp[4],__kd[4],MomentForce[4].force_0,2,1); //force_0为内侧电机
//		Cal_Yobotics_Data2(-M_angle[4].angle_1,0,__kp[4],__kd[4],MomentForce[4].force_1,2,2);
//		
//		Cal_Yobotics_Data2(-M_angle[3].angle_0,0,__kp[3],__kd[3],MomentForce[3].force_0,1,1); //force_0为内侧电机
//		Cal_Yobotics_Data2(-M_angle[3].angle_1,0,__kp[3],__kd[3],MomentForce[3].force_1,1,2);
//		Cal_Yobotics_Data ( M_angle[2].angle_0,0,__kp[2],__kd[2],MomentForce[2].force_0,2,1); //force_0为内侧电机
//		Cal_Yobotics_Data ( M_angle[2].angle_1,0,__kp[2],__kd[2],MomentForce[2].force_1,2,2);
		Cal_Yobotics_Data ( M_angle[1].angle_0,0,__kpp[1],__kdd[1],MomentForce[1].force_0,1,1); //force_0为内侧电机
		Cal_Yobotics_Data ( M_angle[1].angle_1,0,__kpp[1],__kdd[1],MomentForce[1].force_1,1,2);
		
		Cal_Yobotics_Data ( M_angle[2].angle_0,0,__kpp[2],__kdd[2],MomentForce[2].force_0,2,1); //force_0为内侧电机
		Cal_Yobotics_Data ( M_angle[2].angle_1,0,__kpp[2],__kdd[2],MomentForce[2].force_1,2,2);
	
		Cal_Yobotics_Data2(-M_angle[3].angle_0,0,__kpp[3],__kdd[3],MomentForce[3].force_0,1,1); //force_0为内侧电机
		Cal_Yobotics_Data2(-M_angle[3].angle_1,0,__kpp[3],__kdd[3],MomentForce[3].force_1,1,2);
		
		Cal_Yobotics_Data2(-M_angle[4].angle_0,0,__kpp[4],__kdd[4],MomentForce[4].force_0,2,1);
		Cal_Yobotics_Data2(-M_angle[4].angle_1,0,__kpp[4],__kdd[4],MomentForce[4].force_1,2,2); //force_0为内侧电机
	
	
	
	
	
	
//		Cal_Yobotics_Data2(-M_angle[4].angle_0,0,__kp[4],__kd[4],MomentForce[4].force_1,2,2);
//		Cal_Yobotics_Data2(-M_angle[4].angle_1,0,__kp[4],__kd[4],MomentForce[4].force_0,2,1); //force_0为内侧电机
//		
//		Cal_Yobotics_Data ( M_angle[1].angle_0,0,__kp[1],__kd[1],MomentForce[1].force_0,1,1); //force_0为内侧电机
//		Cal_Yobotics_Data ( M_angle[1].angle_1,0,__kp[1],__kd[1],MomentForce[1].force_1,1,2);
//		
//		Cal_Yobotics_Data2(M_angle[3].angle_1,0,__kp[3],__kd[3],MomentForce[3].force_0,1,1); //force_0为内侧电机
//		Cal_Yobotics_Data2(M_angle[3].angle_0,0,__kp[3],__kd[3],MomentForce[3].force_1,1,2);
//		
//		Cal_Yobotics_Data ( -M_angle[2].angle_1,0,__kp[2],__kd[2],MomentForce[2].force_0,2,1); //force_0为内侧电机
//		Cal_Yobotics_Data ( -M_angle[2].angle_0,0,__kp[2],__kd[2],MomentForce[2].force_1,2,2);
		
}
