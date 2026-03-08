#include "pid.h"
#include "mytypr.h"
#include <math.h>
//#include "cmsis_os.h"

#define ABS(x)		((x>0)? (x): (-x)) 

void abs_limit(float *a, float ABS_MAX){
    if(*a > ABS_MAX)
        *a = ABS_MAX;
    if(*a < -ABS_MAX)
        *a = -ABS_MAX;
}
/*参数初始化--------------------------------------------------------------*/
/**
  * @brief  PID参数初始化（内部静态函数）
  * @param  pid: PID结构体指针。
  * @param  mode: PID模式（位置式/增量式）。
  * @param  maxout: 最大输出值。
  * @param  intergral_limit: 积分项限幅。
  * @param  kp, ki, kd: PID增益。
  * @retval 无
  * @用法及调用要求  
  * 被 PID_struct_init 调用，用于初始化结构体内的所有参数。
  * @其它  
  * 这是一个内部函数，不对外暴露。
  */
static void pid_param_init(
    pid_t *pid, 
    uint32_t mode,
    uint32_t maxout,
    uint32_t intergral_limit,
    float 	kp, 
    float 	ki, 
    float 	kd)
{
    
    pid->IntegralLimit = intergral_limit;
    pid->MaxOutput = maxout;
    pid->pid_mode = mode;
    
    pid->p = kp;
    pid->i = ki;
    pid->d = kd;
    
}
/*中途更改参数设定(调试)------------------------------------------------------------*/
/**
  * @brief  PID参数重置（内部静态函数）
  * @param  pid: PID结构体指针。
  * @param  kp, ki, kd: 新的PID增益。
  * @retval 无
  * @用法及调用要求  
  * 用于调试时中途更改参数设定。
  * @其它  
  * 这是一个内部函数，不对外暴露。
  */
static void pid_reset(pid_t	*pid, float kp, float ki, float kd)
{
    pid->p = kp;
    pid->i = ki;
    pid->d = kd;
}

/**
  * @brief  PID核心计算函数
  * @param  pid: PID结构体指针。
  * @param  get: 当前反馈值（实际位置/速度）。
  * @param  set: 目标设定值。
  * @retval float：计算出的控制输出值（如电流、力矩）。
  * @用法及调用要求  
  * 在主控制环路中周期性调用，接收反馈和目标，计算出控制量。
  * @其它  
  * 包含位置式 (POSITION_PID) 和增量式 (DELTA_PID) 两种模式的实现，并使用 abs_limit 对积分项和最终输出进行限幅。
  */
float pid_calc(pid_t* pid, float get, float set){
    pid->get[NOW] = get;
    pid->set[NOW] = set;
    pid->err[NOW] = set - get;	//set - measure
    if (pid->max_err != 0 && ABS(pid->err[NOW]) >  pid->max_err  )
		return 0;
	if (pid->deadband != 0 && ABS(pid->err[NOW]) < pid->deadband)
		return 0;
    
    if(pid->pid_mode == POSITION_PID) //位置式p
    {
        pid->pout = pid->p * pid->err[NOW];
        pid->iout += pid->i * pid->err[NOW];
        pid->dout = pid->d * (pid->err[NOW] - pid->err[LAST] );
        abs_limit(&(pid->iout), pid->IntegralLimit);
        pid->pos_out = pid->pout + pid->iout + pid->dout;
        abs_limit(&(pid->pos_out), pid->MaxOutput);
        pid->last_pos_out = pid->pos_out;	//update last time 
    }
    else if(pid->pid_mode == DELTA_PID)//增量式P
    {
        pid->pout = pid->p * (pid->err[NOW] - pid->err[LAST]);
        pid->iout = pid->i * pid->err[NOW];
        pid->dout = pid->d * (pid->err[NOW] - 2*pid->err[LAST] + pid->err[LLAST]);
        
        abs_limit(&(pid->iout), pid->IntegralLimit);
        pid->delta_u = pid->pout + pid->iout + pid->dout;
        pid->delta_out = pid->last_delta_out + pid->delta_u;
        abs_limit(&(pid->delta_out), pid->MaxOutput);
        pid->last_delta_out = pid->delta_out;	//update last time
    }
    
    pid->err[LLAST] = pid->err[LAST];
    pid->err[LAST] = pid->err[NOW];
    pid->get[LLAST] = pid->get[LAST];
    pid->get[LAST] = pid->get[NOW];
    pid->set[LLAST] = pid->set[LAST];
    pid->set[LAST] = pid->set[NOW];
    return pid->pid_mode==POSITION_PID ? pid->pos_out : pid->delta_out;
//	
}

/**
    *@bref. special calculate position PID @attention @use @gyro data!!
    *@param[in] set： target
    *@param[in] real	measure
    */
float pid_sp_calc(pid_t* pid, float get, float set, float gyro){
    pid->get[NOW] = get;
    pid->set[NOW] = set;
    pid->err[NOW] = set - get;	//set - measure
    
    
    if(pid->pid_mode == POSITION_PID) //位置式p
    {
        pid->pout = pid->p * pid->err[NOW];
				if(fabs(pid->i) >= 0.001f)
					pid->iout += pid->i * pid->err[NOW];
				else
					pid->iout = 0;
        pid->dout = -pid->d * gyro/100.0f;	
        abs_limit(&(pid->iout), pid->IntegralLimit);
        pid->pos_out = pid->pout + pid->iout + pid->dout;
        abs_limit(&(pid->pos_out), pid->MaxOutput);
        pid->last_pos_out = pid->pos_out;	//update last time 
    }
    else if(pid->pid_mode == DELTA_PID)//增量式P
    {
        pid->pout = pid->p * (pid->err[NOW] - pid->err[LAST]);
        pid->iout = pid->i * pid->err[NOW];
        pid->dout = pid->d * (pid->err[NOW] - 2*pid->err[LAST] + pid->err[LLAST]);
        
        abs_limit(&(pid->iout), pid->IntegralLimit);
        pid->delta_u = pid->pout + pid->iout + pid->dout;
        pid->delta_out = pid->last_delta_out + pid->delta_u;
        abs_limit(&(pid->delta_out), pid->MaxOutput);
        pid->last_delta_out = pid->delta_out;	//update last time
    }
    
    pid->err[LLAST] = pid->err[LAST];
    pid->err[LAST] = pid->err[NOW];
    pid->get[LLAST] = pid->get[LAST];
    pid->get[LAST] = pid->get[NOW];
    pid->set[LLAST] = pid->set[LAST];
    pid->set[LAST] = pid->set[NOW];
    return pid->pid_mode==POSITION_PID ? pid->pos_out : pid->delta_out;
//	
}
/*pid总体初始化-----------------------------------------------------------------*/
/**
  * @brief  PID结构体初始化（外部调用接口）
  * @param  pid: PID结构体指针。
  * @param  mode: PID模式（位置式/增量式）。
  * @param  maxout: 最大输出值。
  * @param  intergral_limit: 积分项限幅。
  * @param  kp, ki, kd: PID增益。
  * @retval 无
  * @用法及调用要求  
  * 在程序启动阶段被调用，对所有需要PID控制的电机或控制环路进行参数配置。
  * @其它  
  * 内部调用了 pid_param_init。
  */
void PID_struct_init(
    pid_t* pid,
    uint32_t mode,
    uint32_t maxout,
    uint32_t intergral_limit,
    
    float 	kp, 
    float 	ki, 
    float 	kd)
{
    /*init function pointer*/
    pid->f_param_init = pid_param_init;
    pid->f_pid_reset = pid_reset;
//	pid->f_cal_pid = pid_calc;	
//	pid->f_cal_sp_pid = pid_sp_calc;	//addition
		
    /*init pid param */
    pid->f_param_init(pid, mode, maxout, intergral_limit, kp, ki, kd);
	
}


//pid_t pid_rol = {0};
//pid_t pid_pit = {0};
//pid_t pid_yaw = {0};
//pid_t pid_yaw_omg = {0};//角速度环
//pid_t pid_pit_omg = {0};//角速度环
//pid_t pid_yaw_alfa = {0};		//angle acce

//pid_t pid_chassis_angle={0};
//pid_t pid_poke = {0};
//pid_t pid_poke_omg = {0};
//pid_t pid_imu_tmp;
//pid_t pid_x;
//pid_t pid_cali_bby;
//pid_t pid_cali_bbp;

pid_t pid_omg;
pid_t pid_pos;
pid_t pid_spd[4];


void pid_test_init(){
	

	//为了解决上位机调参的时候第一次赋值的时候清零其他参数， 应该提前把参数表填充一下！

}
