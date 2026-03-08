#include "newTrot.h"
#include "newIK.h"
#include "bound.h"
#include "stm32g4xx_hal.h"


	double x0 ,z0,x1,x2,x3,z1,z2,z3;   //qi dian zuo biao 
	//x的
double x01,x02,x03,x04;//x的起点坐标
double x11,x12,x13,x14;//x的起始速度坐标
double x21,x22,x23,x24;//x的结束速度坐标
double x31,x32,x33,x34;//x的结束坐标
//z的
double z01,z02,z03,z04;//起点坐标
double z11,z12,z13,z14;//起始速度坐标
double z21,z22,z23,z24;//结束速度坐标
double z31,z32,z33,z34;//结束坐标
	double x_start,z_start;
	extern double M_T;
	extern float __kpp[5];
	extern float __kdd[5];
	
double x_start,z_start,x_start_L,x_start_R,z_start_L,z_start_R;
double x3L,z3L,x3R,z3R;
/**
  * @brief  特殊动作：跳跃/边界（Bound）轨迹生成
  * @param  无
  * @retval 无（通过修改全局结构体 M_leg[legnumber].M_expx/z 输出足端期望位置）
  * @用法及调用要求  
  * 1. 在遥控器开关触发特殊动作模式时调用（例如：跳跃）。
  * 2. 函数内部包含一个时间步进循环，用于计算贝塞尔曲线上的点。
  * @其它  
  * **核心算法**：使用三阶贝塞尔曲线公式 $P(t) = P_0(1-t)^3 + 3P_1 t(1-t)^2 + 3P_2 t^2(1-t) + P_3 t^3$ 计算足端 X 和 Z 坐标，实现平滑的跳跃/边界动作。循环内部调用 newIK() 完成角度转换和指令发送。
  */
void qiaoqiao_jump2()
{
        __kp[1]=12;
		__kd[1]=70;
		__kp[2]=12;
		__kd[2]=70;
		__kp[3]=12;
		__kd[3]=70;
		__kp[4]=12;
		__kd[4]=70;
		for(int legnumber =1;legnumber<5;legnumber++)	
		{
		 M_leg[legnumber].M_expx=-0.07;
		 M_leg[legnumber].M_expz=-0.15;	
		}
		newIK();
		newIK();
		newIK();
		HAL_Delay(950);
		M_T=0.2;
		//蹲下 
		x0= -0.23 ;  //   //qi dian zuo biao 
        z0= -0.27;
		x1= -0.25;  //qi shi su du fang xiang 
		z1= -0.27;
		x2= -0.3;   //jie shu su du fang xiang		
		z2= 0.01;
		x3= -0.05;  //jie shu zuo biao
		z3= -0.27 ;//0.27
		x_start=-0.05;	
		z_start=-0.27;	 //0.25
			__kp[1]=45;//40//44
		__kd[1]=4;
		__kp[2]=42.5;//43
		__kd[2]=4.2;
		__kp[3]=35;//6·//34;//28
		__kd[3]=10;
		__kp[4]=26.5	;//28.5
		__kd[4]=10;

	boundStep1();
		__kp[1]=28;
		__kd[1]=5;
		__kp[2]=25;
		__kd[2]=5;
		__kp[3]=25;
		__kd[3]=5;
		__kp[4]=28;
		__kd[4]=5;

	boundStep2();

	__kp[1]=6.5;
		__kd[1]=9;//6
		__kp[2]=6.5;
		__kd[2]=7;
		__kp[3]=6.5;
		__kd[3]=6;
		__kp[4]=5.5;
		__kd[4]=9;//5
	for(int legnumber=1;legnumber<5;legnumber++)
	{
		M_leg[legnumber].M_expx=x3;
		M_leg[legnumber].M_expz=z3;
	}
	
	newIK();
	
	HAL_Delay(450);
	__kp[1]=12;
	__kd[1]=100;
	__kp[2]=12;
	__kd[2]=100;
	__kp[3]=12;
	__kd[3]=100;
	__kp[4]=12;
	__kd[4]=100;
	for(int legnumber=1;legnumber<5;legnumber++)
	{
		M_leg[legnumber].M_expx=x_start;
		M_leg[legnumber].M_expz=z_start;
	}
	
	newIK();
	HAL_Delay(450);


}




	void boundStep1()
{

//		 M_leg[1].M_expx=x0+0.04;
//		 M_leg[1].M_expz=z0+0.02;
//		 M_leg[2].M_expx=x0+0.02;
//		 M_leg[2].M_expz=z0+0.02;
//		 M_leg[3].M_expx=x0+0.04;
//		 M_leg[3].M_expz=z0+0.0;
//		 M_leg[4].M_expx=x0+0.06;
//		 M_leg[4].M_expz=z0+0.01;
	
		 M_leg[1].M_expx=x01-0.02;
		 M_leg[1].M_expz=z01;
		 M_leg[2].M_expx=x02;
		 M_leg[2].M_expz=z02-0.02;
		 M_leg[3].M_expx=x03;
		 M_leg[3].M_expz=z03;
		 M_leg[4].M_expx=x04-0.02;
		 M_leg[4].M_expz=z04;
	
//		for(int legnumber =1;legnumber<5;legnumber++)	
//		{
//		 M_leg[legnumber].M_expx=x0;
//		 M_leg[legnumber].M_expz=z0;	
//		}
		TempNewIK();
		HAL_Delay(15);
}

void gaolan()
{
//	高栏
//x0= -0.21 ;  //   //qi dian zuo biao 
//z0= -0.31;
//x1= -0.25;  //qi shi su du fang xiang 
//z1= -0.27;
//x2= -0.3;   //jie shu su du fang xiang		
//z2= 0.01;
//x3= 0.05;  //jie shu zuo biao
//z3= -0.2 ;
//x_start=-0.05;	
//z_start=-0.15;
	
		//bei sai er qv xian
x0= -0.20 ;  //   //qi dian zuo biao 
z0= -0.27;
x1= -0.25;  //qi shi su du fang xiang 
z1= -0.27;
x2= -0.3;   //jie shu su du fang xiang		
z2= 0.01;
x3= 0.0;  //jie shu zuo biao
z3= -0.25 ;
x_start=0.0;	
z_start=-0.25;
x01=x0+0.03,x02=x0-0.05,x03=x0-0.05,x04=x0+0.03;//x的起点坐标
x11=x1,x12=x1,x13=x1,x14=x1;//x的起始速度坐标
x21=x2,x22=x2,x23=x2,x24=x2;//x的结束速度坐标
x31=x3,x32=x3,x33=x3,x34=x3;//x的结束坐标
//z的
z01=z0-0.06,z02=z0-0.06,z03=z0-0.06,z04=z0-0.06;//起点坐标
z11=z1,z12=z1,z13=z1,z14=z1;//起始速度坐标
z21=z2,z22=z2,z23=z2,z24=z2;//结束速度坐标
z31=z3,z32=z3,z33=z3,z34=z3;//结束坐标
	//printf("bound1\n");
////////	电压23.7                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   
////////		__kp[1]=45.5;//40//44
////////		__kd[1]=4;
////////		__kp[2]=43;//43
////////		__kd[2]=4.2;
////////		__kp[3]=35;//6·//34;//28
////////		__kd[3]=10;
////////		__kp[4]=26.5	;//28.5
////////		__kd[4]=10;	电压23.7
		__kp[1]=62;//40//44//45             上电压24.1，下电压24.4,1腿：49,2；2腿：48,4.2；3腿：35,10；4腿：26.5,10，场地上左偏一度（先边走边矫正走15步）
		__kd[1]=4;                        //蹬腿受腿的初始位置影响大，可以先走几步或者手动摆正，然后调参数  
		__kp[2]=51;//43//42.5              
		__kd[2]=4.2;
		__kp[3]=35;//6·//34;//28//35        下电源24.6V；上电源24.5V ； 1腿为51,4.3； 2腿为48.5,4.5 ； 3腿为34.5,12  4腿为24.5,10 ；场地上左偏移4度
		__kd[3]=10;
		__kp[4]=31.5;//28.5    //26.5
		__kd[4]=6;
	boundStep1();
	 //HAL_Delay(10000);
	//printf("bound2\n");
		__kp[1]=23;
		__kd[1]=7;
		__kp[2]=22.5;
		__kd[2]=7;
		__kp[3]=20;
		__kd[3]=7;
		__kp[4]=20;
		__kd[4]=7;

	boundStep2();
		//printf("bound3\n");
		__kp[1]=17;
		__kd[1]=8	;
		__kp[2]=17;
		__kd[2]=8;
		__kp[3]=17;
		__kd[3]=8;
		__kp[4]=17;
		__kd[4]=8;		

	boundStep3();

}



void boundStep2()
{	

	int first_nearest_point=1;
	
//	for(double t=0;t<=(M_T*0.5);t+=0.002)//M_T越大，贝塞尔曲线越平滑（实际轨迹）
//  {
//		for(int legnumber =1;legnumber<5;legnumber++)	
//		{
//		
//		M_leg[legnumber].M_expx=x0*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))+
//			3*x1*t/(M_T*0.5)*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))+
//			3*x2*(t/(M_T*0.5))*(t/(M_T*0.5))*(1-t/(M_T*0.5))+x3*(t/(M_T*0.5))*(t/(M_T*0.5))*(t/(M_T*0.5));
//		M_leg[legnumber].M_expz=z0*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))+
//			3*z1*t/(M_T*0.5)*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))+
//			3*z2*(t/(M_T*0.5))*(t/(M_T*0.5))*(1-t/(M_T*0.5))+z3*(t/(M_T*0.5))*(t/(M_T*0.5))*(t/(M_T*0.5));
//		}
//		newIK();

//	}
	for(double t=0;t<=(M_T*0.5);t+=0.003)//M_T越大，贝塞尔曲线越平滑（实际轨迹）//t+=0.002
  {
		for(int legnumber =1;legnumber<2;legnumber++)	
		{
		
		M_leg[legnumber].M_expx=x01*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))+
			3*x11*t/(M_T*0.5)*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))+
			3*x21*(t/(M_T*0.5))*(t/(M_T*0.5))*(1-t/(M_T*0.5))+x31*(t/(M_T*0.5))*(t/(M_T*0.5))*(t/(M_T*0.5));
		M_leg[legnumber].M_expz=z01*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))+
			3*z11*t/(M_T*0.5)*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))+
			3*z21*(t/(M_T*0.5))*(t/(M_T*0.5))*(1-t/(M_T*0.5))+z31*(t/(M_T*0.5))*(t/(M_T*0.5))*(t/(M_T*0.5));
		}
		for(int legnumber =2;legnumber<3;legnumber++)	
		{
		
		M_leg[legnumber].M_expx=x02*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))+
			3*x12*t/(M_T*0.5)*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))+
			3*x22*(t/(M_T*0.5))*(t/(M_T*0.5))*(1-t/(M_T*0.5))+x32*(t/(M_T*0.5))*(t/(M_T*0.5))*(t/(M_T*0.5));
		M_leg[legnumber].M_expz=z02*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))+
			3*z12*t/(M_T*0.5)*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))+
			3*z22*(t/(M_T*0.5))*(t/(M_T*0.5))*(1-t/(M_T*0.5))+z32*(t/(M_T*0.5))*(t/(M_T*0.5))*(t/(M_T*0.5));
		}
		for(int legnumber =3;legnumber<4;legnumber++)	
		{
		
		M_leg[legnumber].M_expx=x03*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))+
			3*x13*t/(M_T*0.5)*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))+
			3*x23*(t/(M_T*0.5))*(t/(M_T*0.5))*(1-t/(M_T*0.5))+x33*(t/(M_T*0.5))*(t/(M_T*0.5))*(t/(M_T*0.5));
		M_leg[legnumber].M_expz=z03*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))+
			3*z13*t/(M_T*0.5)*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))+
			3*z23*(t/(M_T*0.5))*(t/(M_T*0.5))*(1-t/(M_T*0.5))+z33*(t/(M_T*0.5))*(t/(M_T*0.5))*(t/(M_T*0.5));
		}
		for(int legnumber =4;legnumber<5;legnumber++)	
		{
		
		M_leg[legnumber].M_expx=x04*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))+
			3*x14*t/(M_T*0.5)*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))+
			3*x24*(t/(M_T*0.5))*(t/(M_T*0.5))*(1-t/(M_T*0.5))+x34*(t/(M_T*0.5))*(t/(M_T*0.5))*(t/(M_T*0.5));
		M_leg[legnumber].M_expz=z04*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))+
			3*z14*t/(M_T*0.5)*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))+
			3*z24*(t/(M_T*0.5))*(t/(M_T*0.5))*(1-t/(M_T*0.5))+z34*(t/(M_T*0.5))*(t/(M_T*0.5))*(t/(M_T*0.5));
		}
		if((M_leg[1].M_expx+M_leg[2].M_expx+M_leg[3].M_expx+M_leg[4].M_expx>0.16)&&first_nearest_point==1){
			first_nearest_point=0;
			HAL_Delay(150);
		}
		newIK();
		//HAL_Delay(15);
	}
}
void boundStep3()//LuoDiHuanChong
{
		__kp[1]=6.5;
		__kd[1]=7;//6
		__kp[2]=6.5;
		__kd[2]=7;
		__kp[3]=6.5;
		__kd[3]=6;
		__kp[4]=5.5;
		__kd[4]=5;//5//腾空时在终点保持静止的Kp，Kd参数
//	__kp[1]=15;
//	__kd[1]=100;
//	__kp[2]=15;
//	__kd[2]=100;
//	__kp[3]=15;
//	__kd[3]=100;
//	__kp[4]=15;
//	__kd[4]=100;//腾空时在终点保持静止的Kp，Kd参数
	
	
//	for(int legnumber=1;legnumber<5;legnumber++)
//	{
//		M_leg[legnumber].M_expx=x3;
//		M_leg[legnumber].M_expz=z3;
//	}
	for(int legnumber=1;legnumber<5;legnumber++)
	{
		M_leg[legnumber].M_expx=x_start;
		M_leg[legnumber].M_expz=z_start;
	}
	newIK();
	HAL_Delay(450);//腾空时在终点保持静止的时长（需要保持足够长时间，等到跨过了再改变位置）//450
	__kp[1]=15;
	__kd[1]=100;
	__kp[2]=15;
	__kd[2]=100;
	__kp[3]=15;
	__kd[3]=100;
	__kp[4]=15;
	__kd[4]=100;//真正的落地缓冲，Kd很大，不会让速度产生突变
	for(int legnumber=1;legnumber<5;legnumber++)
	{
		M_leg[legnumber].M_expx=x_start;
		M_leg[legnumber].M_expz=z_start;
	}
	
	newIK();
//	HAL_Delay(450);//缓冲保持时长
	HAL_Delay(450);//缓冲保持时长

}

void bench_testing(){//架上调试跳跃动作（完整）
	for(int legnumber =1;legnumber<2;legnumber++)	
		{
		 M_leg[legnumber].M_expx=-0.055;
		 M_leg[legnumber].M_expz=-0.13;	
		}
		for(int legnumber =2;legnumber<3;legnumber++)	
		{
		 M_leg[legnumber].M_expx=-0.053;
		 M_leg[legnumber].M_expz=-0.135;	
		}
		for(int legnumber =3;legnumber<4;legnumber++)	
		{
		 M_leg[legnumber].M_expx=-0.06;
		 M_leg[legnumber].M_expz=-0.1275;	
		}
		for(int legnumber =4;legnumber<5;legnumber++)	
		{
		 M_leg[legnumber].M_expx=-0.052;
		 M_leg[legnumber].M_expz=-0.13;	
		}
		__kp[1]=8;
		__kd[1]=30;
		__kp[2]=8;
		__kd[2]=30;
		__kp[3]=8;
		__kd[3]=30;
		__kp[4]=8;
		__kd[4]=30;
		newIK();//进行下蹲
		HAL_Delay(3000);
		HAL_Delay(1500);//静止蓄力
		M_T=0.2;//0.2
		
//x0= -0.23 ;  //   //qi dian zuo biao 起点坐标
//z0= -0.27;
//x1= -0.25;  //qi shi su du fang xiang 起始速度方向
//z1= -0.27;
//x2= 0.5;   //jie shu su du fang xiang		结束速度方向
//z2= -0.02;
//x3= 0.02;  //jie shu zuo biao 结束坐标
//z3= -0.25 ;
//x_start=0.0;	//？
//z_start=-0.25;

x0= -0.23 ;  //   //qi dian zuo biao 起点坐标
z0= -0.32;
x1= -0.23;  //qi shi su du fang xiang 起始速度方向
z1= -0.25;
x2= 0.3;   //jie shu su du fang xiang		结束速度方向
z2= -0.05;
x3= 0.2;  //jie shu zuo biao 结束坐标
z3= -0.25 ;
x_start=0.0;	//？
z_start=-0.25;

//x的
double x01=x0+0.03,x02=x0,x03=x0,x04=x0+0.03;//x的起点坐标
double x11=x1,x12=x1,x13=x1,x14=x1;//x的起始速度坐标
double x21=x2,x22=x2,x23=x2,x24=x2;//x的结束速度坐标
double x31=x3,x32=x3,x33=x3,x34=x3;//x的结束坐标
//z的
double z01=z0,z02=z0,z03=z0,z04=z0;//起点坐标
double z11=z1,z12=z1,z13=z1,z14=z1;//起始速度坐标
double z21=z2,z22=z2,z23=z2,z24=z2;//结束速度坐标
double z31=z3,z32=z3,z33=z3,z34=z3;//结束坐标

	//boundStep1();
		__kp[1]=8;
		__kd[1]=30;
		__kp[2]=8;
		__kd[2]=30;
		__kp[3]=8;
		__kd[3]=30;
		__kp[4]=8;
		__kd[4]=30;
		 M_leg[1].M_expx=x01;
		 M_leg[1].M_expz=z01;
		 M_leg[2].M_expx=x02;
		 M_leg[2].M_expz=z02;
		 M_leg[3].M_expx=x03;
		 M_leg[3].M_expz=z03;
		 M_leg[4].M_expx=x04;
		 M_leg[4].M_expz=z04;
		newIK();
		HAL_Delay(15);
	 HAL_Delay(3000);
	//printf("bound2\n");
		__kp[1]=23;
		__kd[1]=7;
		__kp[2]=23;
		__kd[2]=7;
		__kp[3]=20;
		__kd[3]=7;
		__kp[4]=20;
		__kd[4]=7;

	//boundStep2();
	for(double t=0;t<=(M_T*0.5);t+=0.002)//M_T越大，贝塞尔曲线越平滑（实际轨迹）
  {
		for(int legnumber =1;legnumber<2;legnumber++)	
		{
		
		M_leg[legnumber].M_expx=x01*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))+
			3*x11*t/(M_T*0.5)*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))+
			3*x21*(t/(M_T*0.5))*(t/(M_T*0.5))*(1-t/(M_T*0.5))+x31*(t/(M_T*0.5))*(t/(M_T*0.5))*(t/(M_T*0.5));
		M_leg[legnumber].M_expz=z01*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))+
			3*z11*t/(M_T*0.5)*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))+
			3*z21*(t/(M_T*0.5))*(t/(M_T*0.5))*(1-t/(M_T*0.5))+z31*(t/(M_T*0.5))*(t/(M_T*0.5))*(t/(M_T*0.5));
		}
		for(int legnumber =2;legnumber<3;legnumber++)	
		{
		
		M_leg[legnumber].M_expx=x02*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))+
			3*x12*t/(M_T*0.5)*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))+
			3*x22*(t/(M_T*0.5))*(t/(M_T*0.5))*(1-t/(M_T*0.5))+x32*(t/(M_T*0.5))*(t/(M_T*0.5))*(t/(M_T*0.5));
		M_leg[legnumber].M_expz=z02*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))+
			3*z12*t/(M_T*0.5)*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))+
			3*z22*(t/(M_T*0.5))*(t/(M_T*0.5))*(1-t/(M_T*0.5))+z32*(t/(M_T*0.5))*(t/(M_T*0.5))*(t/(M_T*0.5));
		}
		for(int legnumber =3;legnumber<4;legnumber++)	
		{
		
		M_leg[legnumber].M_expx=x03*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))+
			3*x13*t/(M_T*0.5)*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))+
			3*x23*(t/(M_T*0.5))*(t/(M_T*0.5))*(1-t/(M_T*0.5))+x33*(t/(M_T*0.5))*(t/(M_T*0.5))*(t/(M_T*0.5));
		M_leg[legnumber].M_expz=z03*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))+
			3*z13*t/(M_T*0.5)*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))+
			3*z23*(t/(M_T*0.5))*(t/(M_T*0.5))*(1-t/(M_T*0.5))+z33*(t/(M_T*0.5))*(t/(M_T*0.5))*(t/(M_T*0.5));
		}
		for(int legnumber =4;legnumber<5;legnumber++)	
		{
		
		M_leg[legnumber].M_expx=x04*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))+
			3*x14*t/(M_T*0.5)*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))+
			3*x24*(t/(M_T*0.5))*(t/(M_T*0.5))*(1-t/(M_T*0.5))+x34*(t/(M_T*0.5))*(t/(M_T*0.5))*(t/(M_T*0.5));
		M_leg[legnumber].M_expz=z04*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))+
			3*z14*t/(M_T*0.5)*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))+
			3*z24*(t/(M_T*0.5))*(t/(M_T*0.5))*(1-t/(M_T*0.5))+z34*(t/(M_T*0.5))*(t/(M_T*0.5))*(t/(M_T*0.5));
		}
		
		newIK();
		HAL_Delay(500);
	}
	

		

	//boundStep3();
	__kp[1]=6.5;
		__kd[1]=7;//6
		__kp[2]=6.5;
		__kd[2]=7;
		__kp[3]=6.5;
		__kd[3]=6;
		__kp[4]=5.5;
		__kd[4]=5;//5//腾空时在终点保持静止的Kp，Kd参数
	for(int legnumber=1;legnumber<5;legnumber++)
	{
		M_leg[legnumber].M_expx=x3;
		M_leg[legnumber].M_expz=z3;
	}
	newIK();
	HAL_Delay(450);//腾空时在终点保持静止的时长（需要保持足够长时间，等到跨过了再改变位置）
	__kp[1]=15;
	__kd[1]=100;
	__kp[2]=15;
	__kd[2]=100;
	__kp[3]=15;
	__kd[3]=100;
	__kp[4]=15;
	__kd[4]=100;//真正的落地缓冲，Kd很大，不会让速度产生突变
	for(int legnumber=1;legnumber<5;legnumber++)
	{
		M_leg[legnumber].M_expx=x_start;
		M_leg[legnumber].M_expz=z_start;
	}
	
	newIK();
	HAL_Delay(450);//缓冲保持时长
}

void ramp_big_jump(){//斜坡大跳（落地修正）
	for(int legnumber =1;legnumber<2;legnumber++)	
		{
		 M_leg[legnumber].M_expx=-0.055;
		 M_leg[legnumber].M_expz=-0.13;	
		}
		for(int legnumber =2;legnumber<3;legnumber++)	
		{
		 M_leg[legnumber].M_expx=-0.053;
		 M_leg[legnumber].M_expz=-0.135;	
		}
		for(int legnumber =3;legnumber<4;legnumber++)	
		{
		 M_leg[legnumber].M_expx=-0.06;
		 M_leg[legnumber].M_expz=-0.1275;	
		}
		for(int legnumber =4;legnumber<5;legnumber++)	
		{
		 M_leg[legnumber].M_expx=-0.052;
		 M_leg[legnumber].M_expz=-0.13;	
		}
		__kp[1]=8;
		__kd[1]=30;
		__kp[2]=8;
		__kd[2]=30;
		__kp[3]=8;
		__kd[3]=30;
		__kp[4]=8;
		__kd[4]=30;
		newIK();//进行下蹲
		HAL_Delay(3000);
		HAL_Delay(1500);//静止蓄力
		M_T=0.2;//0.2
		
//x0= -0.23 ;  //   //qi dian zuo biao 起点坐标
//z0= -0.27;
//x1= -0.25;  //qi shi su du fang xiang 起始速度方向
//z1= -0.27;
//x2= 0.5;   //jie shu su du fang xiang		结束速度方向
//z2= -0.02;
//x3= 0.02;  //jie shu zuo biao 结束坐标
//z3= -0.25 ;
//x_start=0.0;	//？
//z_start=-0.25;

x0= -0.23 ;  //   //qi dian zuo biao 起点坐标
z0= -0.32;
x1= -0.23;  //qi shi su du fang xiang 起始速度方向
z1= -0.25;
x2= 0.3;   //jie shu su du fang xiang		结束速度方向
z2= -0.05;
x3= 0.2;  //jie shu zuo biao 结束坐标
z3= -0.25 ;
x_start=0.0;	//？
z_start=-0.25;

//x的
double x01=x0+0.03,x02=x0,x03=x0,x04=x0+0.03;//x的起点坐标
double x11=x1,x12=x1,x13=x1,x14=x1;//x的起始速度坐标
double x21=x2,x22=x2,x23=x2,x24=x2;//x的结束速度坐标
double x31=x3,x32=x3,x33=x3,x34=x3;//x的结束坐标
//z的
double z01=z0,z02=z0,z03=z0,z04=z0;//起点坐标
double z11=z1,z12=z1,z13=z1,z14=z1;//起始速度坐标
double z21=z2,z22=z2,z23=z2,z24=z2;//结束速度坐标
double z31=z3+0.15,z32=z3+0.15,z33=z3,z34=z3;//结束坐标

	//boundStep1();
		__kp[1]=8;
		__kd[1]=30;
		__kp[2]=8;
		__kd[2]=30;
		__kp[3]=8;
		__kd[3]=30;
		__kp[4]=8;
		__kd[4]=30;
		 M_leg[1].M_expx=x01;
		 M_leg[1].M_expz=z01;
		 M_leg[2].M_expx=x02;
		 M_leg[2].M_expz=z02;
		 M_leg[3].M_expx=x03;
		 M_leg[3].M_expz=z03;
		 M_leg[4].M_expx=x04;
		 M_leg[4].M_expz=z04;
		newIK();
		//TempNewIK();
		HAL_Delay(15);
	 HAL_Delay(3000);
	//printf("bound2\n");
		__kp[1]=23;
		__kd[1]=7;
		__kp[2]=23;
		__kd[2]=7;
		__kp[3]=20;
		__kd[3]=7;
		__kp[4]=20;
		__kd[4]=7;

	//boundStep2();
	for(double t=0;t<=(M_T*0.5);t+=0.002)//M_T越大，贝塞尔曲线越平滑（实际轨迹）
  {
		for(int legnumber =1;legnumber<2;legnumber++)	
		{
		
		M_leg[legnumber].M_expx=x01*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))+
			3*x11*t/(M_T*0.5)*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))+
			3*x21*(t/(M_T*0.5))*(t/(M_T*0.5))*(1-t/(M_T*0.5))+x31*(t/(M_T*0.5))*(t/(M_T*0.5))*(t/(M_T*0.5));
		M_leg[legnumber].M_expz=z01*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))+
			3*z11*t/(M_T*0.5)*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))+
			3*z21*(t/(M_T*0.5))*(t/(M_T*0.5))*(1-t/(M_T*0.5))+z31*(t/(M_T*0.5))*(t/(M_T*0.5))*(t/(M_T*0.5));
		}
		for(int legnumber =2;legnumber<3;legnumber++)	
		{
		
		M_leg[legnumber].M_expx=x02*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))+
			3*x12*t/(M_T*0.5)*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))+
			3*x22*(t/(M_T*0.5))*(t/(M_T*0.5))*(1-t/(M_T*0.5))+x32*(t/(M_T*0.5))*(t/(M_T*0.5))*(t/(M_T*0.5));
		M_leg[legnumber].M_expz=z02*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))+
			3*z12*t/(M_T*0.5)*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))+
			3*z22*(t/(M_T*0.5))*(t/(M_T*0.5))*(1-t/(M_T*0.5))+z32*(t/(M_T*0.5))*(t/(M_T*0.5))*(t/(M_T*0.5));
		}
		for(int legnumber =3;legnumber<4;legnumber++)	
		{
		
		M_leg[legnumber].M_expx=x03*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))+
			3*x13*t/(M_T*0.5)*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))+
			3*x23*(t/(M_T*0.5))*(t/(M_T*0.5))*(1-t/(M_T*0.5))+x33*(t/(M_T*0.5))*(t/(M_T*0.5))*(t/(M_T*0.5));
		M_leg[legnumber].M_expz=z03*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))+
			3*z13*t/(M_T*0.5)*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))+
			3*z23*(t/(M_T*0.5))*(t/(M_T*0.5))*(1-t/(M_T*0.5))+z33*(t/(M_T*0.5))*(t/(M_T*0.5))*(t/(M_T*0.5));
		}
		for(int legnumber =4;legnumber<5;legnumber++)	
		{
		
		M_leg[legnumber].M_expx=x04*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))+
			3*x14*t/(M_T*0.5)*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))+
			3*x24*(t/(M_T*0.5))*(t/(M_T*0.5))*(1-t/(M_T*0.5))+x34*(t/(M_T*0.5))*(t/(M_T*0.5))*(t/(M_T*0.5));
		M_leg[legnumber].M_expz=z04*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))+
			3*z14*t/(M_T*0.5)*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))+
			3*z24*(t/(M_T*0.5))*(t/(M_T*0.5))*(1-t/(M_T*0.5))+z34*(t/(M_T*0.5))*(t/(M_T*0.5))*(t/(M_T*0.5));
		}
		
		newIK();
		HAL_Delay(500);
	}
	

		

	//boundStep3();
	__kp[1]=6.5;
		__kd[1]=7;//6
		__kp[2]=6.5;
		__kd[2]=7;
		__kp[3]=6.5;
		__kd[3]=6;
		__kp[4]=5.5;
		__kd[4]=5;//5//腾空时在终点保持静止的Kp，Kd参数
	for(int legnumber=1;legnumber<5;legnumber++)
	{
		M_leg[legnumber].M_expx=x3;
		M_leg[legnumber].M_expz=z3;
	}
	newIK();
	HAL_Delay(450);//腾空时在终点保持静止的时长（需要保持足够长时间，等到跨过了再改变位置）
	__kp[1]=15;
	__kd[1]=100;
	__kp[2]=15;
	__kd[2]=100;
	__kp[3]=15;
	__kd[3]=100;
	__kp[4]=15;
	__kd[4]=100;//真正的落地缓冲，Kd很大，不会让速度产生突变
	for(int legnumber=1;legnumber<5;legnumber++)
	{
		M_leg[legnumber].M_expx=x_start;
		M_leg[legnumber].M_expz=z_start;
	}
	
	newIK();
	HAL_Delay(450);//缓冲保持时长
}

void jump_to_right_or_left(){//跳跃横移调整
	for(int legnumber =1;legnumber<2;legnumber++)	
		{
		 M_leg[legnumber].M_expx=0;
		 M_leg[legnumber].M_expz=-0.18;	
		}
		for(int legnumber =2;legnumber<3;legnumber++)	
		{
		 M_leg[legnumber].M_expx=0;
		 M_leg[legnumber].M_expz=-0.18;	
		}
		for(int legnumber =3;legnumber<4;legnumber++)	
		{
		 M_leg[legnumber].M_expx=0;
		 M_leg[legnumber].M_expz=-0.25;	
		}
		for(int legnumber =4;legnumber<5;legnumber++)	
		{
		 M_leg[legnumber].M_expx=0;
		 M_leg[legnumber].M_expz=-0.25;	
		}
		__kp[1]=8;
		__kd[1]=30;
		__kp[2]=8;
		__kd[2]=30;
		__kp[3]=8;
		__kd[3]=30;
		__kp[4]=8;
		__kd[4]=30;
		newIK();//进行下蹲
		HAL_Delay(3000);
		HAL_Delay(1500);//静止蓄力
		M_T=0.2;//0.2
		
//x0= -0.23 ;  //   //qi dian zuo biao 起点坐标
//z0= -0.27;
//x1= -0.25;  //qi shi su du fang xiang 起始速度方向
//z1= -0.27;
//x2= 0.5;   //jie shu su du fang xiang		结束速度方向
//z2= -0.02;
//x3= 0.02;  //jie shu zuo biao 结束坐标
//z3= -0.25 ;
//x_start=0.0;	//？
//z_start=-0.25;

x0= -0.23 ;  //   //qi dian zuo biao 起点坐标
z0= -0.32;
x1= -0.23;  //qi shi su du fang xiang 起始速度方向
z1= -0.25;
x2= 0.3;   //jie shu su du fang xiang		结束速度方向
z2= -0.05;
x3= 0.2;  //jie shu zuo biao 结束坐标
z3= -0.25 ;
x_start=0.0;	//？
z_start=-0.25;

//x的
double x01=x0+0.03,x02=x0,x03=x0,x04=x0+0.03;//x的起点坐标
double x11=x1,x12=x1,x13=x1,x14=x1;//x的起始速度坐标
double x21=x2,x22=x2,x23=x2,x24=x2;//x的结束速度坐标
double x31=x3,x32=x3,x33=x3,x34=x3;//x的结束坐标
//z的
double z01=z0,z02=z0,z03=z0,z04=z0;//起点坐标
double z11=z1,z12=z1,z13=z1,z14=z1;//起始速度坐标
double z21=z2,z22=z2,z23=z2,z24=z2;//结束速度坐标
double z31=z3+0.15,z32=z3+0.15,z33=z3,z34=z3;//结束坐标

	//boundStep1();
		__kp[1]=8;
		__kd[1]=30;
		__kp[2]=8;
		__kd[2]=30;
		__kp[3]=8;
		__kd[3]=30;
		__kp[4]=8;
		__kd[4]=30;
		 M_leg[1].M_expx=x01;
		 M_leg[1].M_expz=z01;
		 M_leg[2].M_expx=x02;
		 M_leg[2].M_expz=z02;
		 M_leg[3].M_expx=x03;
		 M_leg[3].M_expz=z03;
		 M_leg[4].M_expx=x04;
		 M_leg[4].M_expz=z04;
		newIK();
		HAL_Delay(15);
	 HAL_Delay(3000);
	//printf("bound2\n");
		__kp[1]=23;
		__kd[1]=7;
		__kp[2]=23;
		__kd[2]=7;
		__kp[3]=20;
		__kd[3]=7;
		__kp[4]=20;
		__kd[4]=7;

	//boundStep2();
	for(double t=0;t<=(M_T*0.5);t+=0.002)//M_T越大，贝塞尔曲线越平滑（实际轨迹）
  {
		for(int legnumber =1;legnumber<2;legnumber++)	
		{
		
		M_leg[legnumber].M_expx=x01*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))+
			3*x11*t/(M_T*0.5)*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))+
			3*x21*(t/(M_T*0.5))*(t/(M_T*0.5))*(1-t/(M_T*0.5))+x31*(t/(M_T*0.5))*(t/(M_T*0.5))*(t/(M_T*0.5));
		M_leg[legnumber].M_expz=z01*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))+
			3*z11*t/(M_T*0.5)*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))+
			3*z21*(t/(M_T*0.5))*(t/(M_T*0.5))*(1-t/(M_T*0.5))+z31*(t/(M_T*0.5))*(t/(M_T*0.5))*(t/(M_T*0.5));
		}
		for(int legnumber =2;legnumber<3;legnumber++)	
		{
		
		M_leg[legnumber].M_expx=x02*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))+
			3*x12*t/(M_T*0.5)*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))+
			3*x22*(t/(M_T*0.5))*(t/(M_T*0.5))*(1-t/(M_T*0.5))+x32*(t/(M_T*0.5))*(t/(M_T*0.5))*(t/(M_T*0.5));
		M_leg[legnumber].M_expz=z02*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))+
			3*z12*t/(M_T*0.5)*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))+
			3*z22*(t/(M_T*0.5))*(t/(M_T*0.5))*(1-t/(M_T*0.5))+z32*(t/(M_T*0.5))*(t/(M_T*0.5))*(t/(M_T*0.5));
		}
		for(int legnumber =3;legnumber<4;legnumber++)	
		{
		
		M_leg[legnumber].M_expx=x03*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))+
			3*x13*t/(M_T*0.5)*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))+
			3*x23*(t/(M_T*0.5))*(t/(M_T*0.5))*(1-t/(M_T*0.5))+x33*(t/(M_T*0.5))*(t/(M_T*0.5))*(t/(M_T*0.5));
		M_leg[legnumber].M_expz=z03*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))+
			3*z13*t/(M_T*0.5)*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))+
			3*z23*(t/(M_T*0.5))*(t/(M_T*0.5))*(1-t/(M_T*0.5))+z33*(t/(M_T*0.5))*(t/(M_T*0.5))*(t/(M_T*0.5));
		}
		for(int legnumber =4;legnumber<5;legnumber++)	
		{
		
		M_leg[legnumber].M_expx=x04*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))+
			3*x14*t/(M_T*0.5)*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))+
			3*x24*(t/(M_T*0.5))*(t/(M_T*0.5))*(1-t/(M_T*0.5))+x34*(t/(M_T*0.5))*(t/(M_T*0.5))*(t/(M_T*0.5));
		M_leg[legnumber].M_expz=z04*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))+
			3*z14*t/(M_T*0.5)*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))+
			3*z24*(t/(M_T*0.5))*(t/(M_T*0.5))*(1-t/(M_T*0.5))+z34*(t/(M_T*0.5))*(t/(M_T*0.5))*(t/(M_T*0.5));
		}
		
		newIK();
		HAL_Delay(500);
	}
	

		

	//boundStep3();
	__kp[1]=6.5;
		__kd[1]=7;//6
		__kp[2]=6.5;
		__kd[2]=7;
		__kp[3]=6.5;
		__kd[3]=6;
		__kp[4]=5.5;
		__kd[4]=5;//5//腾空时在终点保持静止的Kp，Kd参数
	for(int legnumber=1;legnumber<5;legnumber++)
	{
		M_leg[legnumber].M_expx=x3;
		M_leg[legnumber].M_expz=z3;
	}
	newIK();
	HAL_Delay(450);//腾空时在终点保持静止的时长（需要保持足够长时间，等到跨过了再改变位置）
	__kp[1]=15;
	__kd[1]=100;
	__kp[2]=15;
	__kd[2]=100;
	__kp[3]=15;
	__kd[3]=100;
	__kp[4]=15;
	__kd[4]=100;//真正的落地缓冲，Kd很大，不会让速度产生突变
	for(int legnumber=1;legnumber<5;legnumber++)
	{
		M_leg[legnumber].M_expx=x_start;
		M_leg[legnumber].M_expz=z_start;
	}
	
	newIK();
	HAL_Delay(450);//缓冲保持时长
}

void hurdle()
{
//	高栏
//x0= -0.21 ;  //   //qi dian zuo biao 
//z0= -0.31;
//x1= -0.25;  //qi shi su du fang xiang 
//z1= -0.27;
//x2= -0.3;   //jie shu su du fang xiang		
//z2= 0.01;
//x3= 0.05;  //jie shu zuo biao
//z3= -0.2 ;
//x_start=-0.05;	
//z_start=-0.15;
	
		//bei sai er qv xian
//x0= -0.23 ;  //   //qi dian zuo biao 
//z0= -0.27;
//x1= -0.25;  //qi shi su du fang xiang 
//z1= -0.27;
//x2= -0.3;   //jie shu su du fang xiang		
//z2= 0.01;
//x3= 0.0;  //jie shu zuo biao
//z3= -0.25 ;
//x_start=0.0;	
//z_start=-0.25;
//	//printf("bound1\n");
//////////	电压23.7
//////////		__kp[1]=45.5;//40//44
//////////		__kd[1]=4;
//////////		__kp[2]=43;//43
//////////		__kd[2]=4.2;
//////////		__kp[3]=35;//6·//34;//28
//////////		__kd[3]=10;
//////////		__kp[4]=26.5	;//28.5
//////////		__kd[4]=10;	电压23.7
//		__kp[1]=30;//40//44//45             上电压24.1，下电压24.4,1腿：49,2；2腿：48,4.2；3腿：35,10；4腿：26.5,10，场地上左偏一度（先边走边矫正走15步）
//		__kd[1]=40;                        //蹬腿受腿的初始位置影响大，可以先走几步或者手动摆正，然后调参数  
//		__kp[2]=30;//43//42.5              
//		__kd[2]=40;
//		__kp[3]=30;//6·//34;//28//35        下电源24.6V；上电源24.5V ； 1腿为51,4.3； 2腿为48.5,4.5 ； 3腿为34.5,12  4腿为24.5,10 ；场地上左偏移4度
//		__kd[3]=40;
//		__kp[4]=30;//28.5    //      26.5//①四条腿均57,40：空中左低右高，落地左侧着地侧翻，落点右偏40厘米
//		__kd[4]=40;
//	boundStep1();//蹬地起跳
//	 //HAL_Delay(10000);
//	//printf("bound2\n");
//		__kp[1]=23;
//		__kd[1]=7;
//		__kp[2]=23;
//		__kd[2]=7;
//		__kp[3]=23;
//		__kd[3]=7;
//		__kp[4]=23;
//		__kd[4]=7;

//	boundStep2();
//		//printf("bound3\n");
//		__kp[1]=17;
//		__kd[1]=8	;
//		__kp[2]=17;
//		__kd[2]=8;
//		__kp[3]=17;
//		__kd[3]=8;
//		__kp[4]=17;
//		__kd[4]=8;		

//	boundStep3();
x0= -0.23 ;  //   //qi dian zuo biao 起点坐标
z0= -0.32;
x1= -0.23;  //qi shi su du fang xiang 起始速度方向
z1= -0.25;
x2= 0.3;   //jie shu su du fang xiang		结束速度方向
z2= -0.05;
x3= 0.2;  //jie shu zuo biao 结束坐标
z3= -0.25 ;
x_start=0.0;	//？
z_start=-0.25;

////x的
//x01=x0,x02=x0,x03=x0+0.04,x04=x0+0.02;//x的起点坐标
//x11=x1,x12=x1,x13=x1,x14=x1;//x的起始速度坐标
//x21=x2,x22=x2,x23=x2,x24=x2;//x的结束速度坐标
//x31=x3,x32=x3,x33=x3+0.04,x34=x3+0.02;//x的结束坐标
////z的
//z01=z0,z02=z0,z03=z0,z04=z0;//起点坐标
//z11=z1,z12=z1,z13=z1,z14=z1;//起始速度坐标
//z21=z2,z22=z2,z23=z2,z24=z2;//结束速度坐标
//z31=z3,z32=z3,z33=z3,z34=z3;//结束坐标
//x的
x01=x0+0.03,x02=x0-0.05,x03=x0-0.05,x04=x0+0.03;//x的起点坐标
x11=x1,x12=x1,x13=x1,x14=x1;//x的起始速度坐标
x21=x2,x22=x2,x23=x2,x24=x2;//x的结束速度坐标
x31=x3,x32=x3,x33=x3,x34=x3;//x的结束坐标
//z的
z01=z0-0.06,z02=z0-0.06,z03=z0-0.06,z04=z0-0.06;//起点坐标
z11=z1,z12=z1,z13=z1,z14=z1;//起始速度坐标
z21=z2,z22=z2,z23=z2,z24=z2;//结束速度坐标
z31=z3,z32=z3,z33=z3,z34=z3;//结束坐标
	//printf("bound1\n");
////////	电压23.7
////////		__kp[1]=45.5;//40//44
////////		__kd[1]=4;
////////		__kp[2]=43;//43
////////		__kd[2]=4.2;
////////		__kp[3]=35;//6·//34;//28
////////		__kd[3]=10;
////////		__kp[4]=26.5	;//28.5
////////		__kd[4]=10;	电压23.7



//		__kpp[1]=57;//40//44//45             上电压24.1，下电压24.4,1腿：49,2；2腿：48,4.2；3腿：35,10；4腿：26.5,10，场地上左偏一度（先边走边矫正走15步）
//		__kdd[1]=4;                        //蹬腿受腿的初始位置影响大，可以先走几步或者手动摆正，然后调参数  
//		__kpp[2]=42;//43//42.5              
//		__kdd[2]=4.2;
//		__kpp[3]=39 ;//6·//34;//28//35        下电源24.6V；上电源24.5V ； 1腿为51,4.3； 2腿为48.5,4.5 ； 3腿为34.5,12  4腿为24.5,10 ；场地上左偏移4度
//		__kdd[3]=10;
//		__kpp[4]=30.5 ;//28.5    //26.5
//		__kdd[4]=10;
	boundStep1();
	 //HAL_Delay(10000);
	//printf("bound2\n");
		__kp[1]=23;
		__kd[1]=7;
		__kp[2]=23;
		__kd[2]=7;
		__kp[3]=20;
		__kd[3]=7;
		__kp[4]=20;
		__kd[4]=7;

	boundStep2();
		//printf("bound3\n");
		__kp[1]=17;
		__kd[1]=8	;
		__kp[2]=17;
		__kd[2]=8;
		__kp[3]=17;
		__kd[3]=8;
		__kp[4]=17;
		__kd[4]=8;		

	boundStep3();

}

void ramp_hurdle()
{
//	高栏
//x0= -0.21 ;  //   //qi dian zuo biao 
//z0= -0.31;
//x1= -0.25;  //qi shi su du fang xiang 
//z1= -0.27;
//x2= -0.3;   //jie shu su du fang xiang		
//z2= 0.01;
//x3= 0.05;  //jie shu zuo biao
//z3= -0.2 ;
//x_start=-0.05;	
//z_start=-0.15;
	
		//bei sai er qv xian
//x0= -0.23 ;  //   //qi dian zuo biao 
//z0= -0.27;
//x1= -0.25;  //qi shi su du fang xiang 
//z1= -0.27;
//x2= -0.3;   //jie shu su du fang xiang		
//z2= 0.01;
//x3= 0.0;  //jie shu zuo biao
//z3= -0.25 ;
//x_start=0.0;	
//z_start=-0.25;
//	//printf("bound1\n");
//////////	电压23.7
//////////		__kp[1]=45.5;//40//44
//////////		__kd[1]=4;
//////////		__kp[2]=43;//43
//////////		__kd[2]=4.2;
//////////		__kp[3]=35;//6·//34;//28
//////////		__kd[3]=10;
//////////		__kp[4]=26.5	;//28.5
//////////		__kd[4]=10;	电压23.7
//		__kp[1]=30;//40//44//45             上电压24.1，下电压24.4,1腿：49,2；2腿：48,4.2；3腿：35,10；4腿：26.5,10，场地上左偏一度（先边走边矫正走15步）
//		__kd[1]=40;                        //蹬腿受腿的初始位置影响大，可以先走几步或者手动摆正，然后调参数  
//		__kp[2]=30;//43//42.5              
//		__kd[2]=40;
//		__kp[3]=30;//6·//34;//28//35        下电源24.6V；上电源24.5V ； 1腿为51,4.3； 2腿为48.5,4.5 ； 3腿为34.5,12  4腿为24.5,10 ；场地上左偏移4度
//		__kd[3]=40;
//		__kp[4]=30;//28.5    //      26.5//①四条腿均57,40：空中左低右高，落地左侧着地侧翻，落点右偏40厘米
//		__kd[4]=40;
//	boundStep1();//蹬地起跳
//	 //HAL_Delay(10000);
//	//printf("bound2\n");
//		__kp[1]=23;
//		__kd[1]=7;
//		__kp[2]=23;
//		__kd[2]=7;
//		__kp[3]=23;
//		__kd[3]=7;
//		__kp[4]=23;
//		__kd[4]=7;

//	boundStep2();
//		//printf("bound3\n");
//		__kp[1]=17;
//		__kd[1]=8	;
//		__kp[2]=17;
//		__kd[2]=8;
//		__kp[3]=17;
//		__kd[3]=8;
//		__kp[4]=17;
//		__kd[4]=8;		

//	boundStep3();
x0= -0.23 ;  //   //qi dian zuo biao 起点坐标
z0= -0.32;
x1= -0.23;  //qi shi su du fang xiang 起始速度方向
z1= -0.25;
x2= 0.3;   //jie shu su du fang xiang		结束速度方向
z2= -0.05;
x3= 0.2;  //jie shu zuo biao 结束坐标
z3= -0.25 ;
x_start=0.0;	//？
z_start=-0.25;

////x的
//x01=x0,x02=x0,x03=x0+0.04,x04=x0+0.02;//x的起点坐标
//x11=x1,x12=x1,x13=x1,x14=x1;//x的起始速度坐标
//x21=x2,x22=x2,x23=x2,x24=x2;//x的结束速度坐标
//x31=x3,x32=x3,x33=x3+0.04,x34=x3+0.02;//x的结束坐标
////z的
//z01=z0,z02=z0,z03=z0,z04=z0;//起点坐标
//z11=z1,z12=z1,z13=z1,z14=z1;//起始速度坐标
//z21=z2,z22=z2,z23=z2,z24=z2;//结束速度坐标
//z31=z3,z32=z3,z33=z3,z34=z3;//结束坐标
//x的
x01=x0+0.03,x02=x0-0.05,x03=x0-0.05,x04=x0+0.03;//x的起点坐标
x11=x1,x12=x1,x13=x1,x14=x1;//x的起始速度坐标
x21=x2,x22=x2,x23=x2,x24=x2;//x的结束速度坐标
x31=x3,x32=x3,x33=x3,x34=x3;//x的结束坐标
//z的
z01=z0-0.01,z02=z0-0.01,z03=z0-0.01,z04=z0-0.01;//起点坐标
z11=z1,z12=z1,z13=z1,z14=z1;//起始速度坐标
z21=z2,z22=z2,z23=z2,z24=z2;//结束速度坐标
z31=z3+0.1,z32=z3+0.1,z33=z3,z34=z3;//结束坐标//恰好为0.059
	//printf("bound1\n");
////////	电压23.7
////////		__kp[1]=45.5;//40//44
////////		__kd[1]=4;
////////		__kp[2]=43;//43
////////		__kd[2]=4.2;
////////		__kp[3]=35;//6·//34;//28
////////		__kd[3]=10;
////////		__kp[4]=26.5	;//28.5
////////		__kd[4]=10;	电压23.7



//		__kpp[1]=57;//40//44//45             上电压24.1，下电压24.4,1腿：49,2；2腿：48,4.2；3腿：35,10；4腿：26.5,10，场地上左偏一度（先边走边矫正走15步）
//		__kdd[1]=4;                        //蹬腿受腿的初始位置影响大，可以先走几步或者手动摆正，然后调参数  
//		__kpp[2]=42;//43//42.5              
//		__kdd[2]=4.2;
//		__kpp[3]=39 ;//6·//34;//28//35        下电源24.6V；上电源24.5V ； 1腿为51,4.3； 2腿为48.5,4.5 ； 3腿为34.5,12  4腿为24.5,10 ；场地上左偏移4度
//		__kdd[3]=10;
//		__kpp[4]=30.5 ;//28.5    //26.5
//		__kdd[4]=10;
	boundStep1();
	 //HAL_Delay(10000);
	//printf("bound2\n");
		__kp[1]=23;
		__kd[1]=7;
		__kp[2]=23;
		__kd[2]=7;
		__kp[3]=20;
		__kd[3]=7;
		__kp[4]=20;
		__kd[4]=7;

	boundStep2();
		//printf("bound3\n");
		__kp[1]=17;
		__kd[1]=8	;
		__kp[2]=17;
		__kd[2]=8;
		__kp[3]=17;
		__kd[3]=8;
		__kp[4]=17;
		__kd[4]=8;		

	boundStep3();

}

	void incline10JumpUp()
{
//	斜坡跳跃
//x0= -0.21 ;  //   //qi dian zuo biao 
//z0= -0.31;
//x1= -0.25;  //qi shi su du fang xiang 
//z1= -0.27;
//x2= -0.3;   //jie shu su du fang xiang		
//z2= 0.01;
//x3= 0.05;  //jie shu zuo biao
//z3= -0.2 ;
//x_start=-0.05;	
//z_start=-0.15;
	
		//bei sai er qv xian
x0= -0.23 ;  //   //qi dian zuo biao 
z0= -0.27;
x1= -0.25;  //qi shi su du fang xiang 
z1= -0.27;
x2= -0.3;   //jie shu su du fang xiang		
z2= 0.01;
x3L= 0.02;  //jie shu zuo biao
z3L= -0.25 ;
x3R= 0.0;  //jie shu zuo biao
z3R= -0.196 ;
x_start_L=0.02;	
z_start_L=-0.25;
x_start_R=0.0;	
z_start_R=-0.196;
	//printf("bound1\n");
////////	电压23.7
////////		__kp[1]=45.5;//40//44
////////		__kd[1]=4;
////////		__kp[2]=43;//43
////////		__kd[2]=4.2;
////////		__kp[3]=35;//6·//34;//28
////////		__kd[3]=10;
////////		__kp[4]=26.5	;//28.5
////////		__kd[4]=10;	电压23.7
		__kp[1]=57;//40//44//45             上电压24.1，下电压24.4,1腿：49,2；2腿：48,4.2；3腿：35,10；4腿：26.5,10，场地上左偏一度（先边走边矫正走15步）
		__kd[1]=8;                        //蹬腿受腿的初始位置影响大，可以先走几步或者手动摆正，然后调参数  
		__kp[2]=50;//43//42.5              
		__kd[2]=10;
		__kp[3]=24;//6·//34;//28//35        下电源24.6V；上电源24.5V ； 1腿为51,4.3； 2腿为48.5,4.5 ； 3腿为34.5,12  4腿为24.5,10 ；场地上左偏移4度
		__kd[3]=10;
		__kp[4]=33;//28.5    //26.5
		__kd[4]=10;
	incline10JumpUpStep1();//蹬地起跳
	 //HAL_Delay(10000);
	//printf("bound2\n");
		__kp[1]=23;
		__kd[1]=7;
		__kp[2]=22.5;
		__kd[2]=7;
		__kp[3]=20;
		__kd[3]=7;
		__kp[4]=20;
		__kd[4]=7;

	incline10JumpUpStep2();
		//printf("bound3\n");
		__kp[1]=17;
		__kd[1]=8	;
		__kp[2]=17;
		__kd[2]=8;
		__kp[3]=17;
		__kd[3]=8;
		__kp[4]=17;
		__kd[4]=8;		

	incline10JumpUpStep3();

}

void incline10JumpUpStep1(){
			 M_leg[1].M_expx=x0;
		 M_leg[1].M_expz=z0+0.02;
//         M_leg[2].M_expx=-0.285;
//		 M_leg[2].M_expz=z0;
		
		for(int legnumber =1;legnumber<5;legnumber++)	
		{
		 M_leg[legnumber].M_expx=x0;
		 M_leg[legnumber].M_expz=z0;	
		}
		newIK();
		HAL_Delay(20);
}

void incline10JumpUpStep2()
{	

	
	
	for(double t=0;t<=(M_T*0.5);t+=0.002)
  {
		for(int legnumber =1;legnumber<3;legnumber++)	
		{
		
		M_leg[legnumber].M_expx=x0*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))+
			3*x1*t/(M_T*0.5)*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))+
			3*x2*(t/(M_T*0.5))*(t/(M_T*0.5))*(1-t/(M_T*0.5))+x3L*(t/(M_T*0.5))*(t/(M_T*0.5))*(t/(M_T*0.5));
		M_leg[legnumber].M_expz=z0*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))+
			3*z1*t/(M_T*0.5)*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))+
			3*z2*(t/(M_T*0.5))*(t/(M_T*0.5))*(1-t/(M_T*0.5))+z3L*(t/(M_T*0.5))*(t/(M_T*0.5))*(t/(M_T*0.5));
		}
		
		for(int legnumber =3;legnumber<5;legnumber++)	
		{
		
		M_leg[legnumber].M_expx=x0*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))+
			3*x1*t/(M_T*0.5)*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))+
			3*x2*(t/(M_T*0.5))*(t/(M_T*0.5))*(1-t/(M_T*0.5))+x3R*(t/(M_T*0.5))*(t/(M_T*0.5))*(t/(M_T*0.5));
		M_leg[legnumber].M_expz=z0*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))+
			3*z1*t/(M_T*0.5)*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))+
			3*z2*(t/(M_T*0.5))*(t/(M_T*0.5))*(1-t/(M_T*0.5))+z3R*(t/(M_T*0.5))*(t/(M_T*0.5))*(t/(M_T*0.5));
		}
		
		newIK();

	}
}


	
void incline10JumpUpStep3()//LuoDiHuanChong
{
		__kp[1]=6.5;
		__kd[1]=7;//6
		__kp[2]=6.5;
		__kd[2]=7;
		__kp[3]=6.5;
		__kd[3]=6;
		__kp[4]=5.5;
		__kd[4]=5;//5
//	for(int legnumber=1;legnumber<5;legnumber++)
//	{
//		M_leg[legnumber].M_expx=x3;
//		M_leg[legnumber].M_expz=z3;
//		//printf("x3=%f,z3=%f\n\n\n",M_leg[legnumber].M_expx,M_leg[legnumber].M_expz);
//	}
	for(int legnumber =1;legnumber<3;legnumber++)	
		{
				M_leg[legnumber].M_expx=x3L;
				M_leg[legnumber].M_expz=z3L;

		}
		
		for(int legnumber =3;legnumber<5;legnumber++)	
		{
				M_leg[legnumber].M_expx=x3R;
				M_leg[legnumber].M_expz=z3R;

		}
	newIK();
	HAL_Delay(450);
	__kp[1]=15;
	__kd[1]=100;
	__kp[2]=15;
	__kd[2]=100;
	__kp[3]=15;
	__kd[3]=100;
	__kp[4]=15;
	__kd[4]=100;
//	for(int legnumber=1;legnumber<5;legnumber++)
//	{
//		M_leg[legnumber].M_expx=x_start;
//		M_leg[legnumber].M_expz=z_start;
//		//printf("x3=%f,z3=%f\n\n\n",M_leg[legnumber].M_expx,M_leg[legnumber].M_expz);
//	}
		for(int legnumber =1;legnumber<3;legnumber++)	
		{
				M_leg[legnumber].M_expx=x_start_L;
				M_leg[legnumber].M_expz=z_start_L;

		}
		
		for(int legnumber =3;legnumber<5;legnumber++)	
		{
				M_leg[legnumber].M_expx=x_start_R;
				M_leg[legnumber].M_expz=z_start_R;

		}
	
	newIK();
	HAL_Delay(450);
	/*
	while(1)
	{
		newIK();
		HAL_Delay(500);
	}
	*/
}

void louti_middle()//前腿高 后腿低
{	
	//前腿在顶层  后腿在2层
	x0= -0.17 ;  //   //qi dian zuo biao 
	z0= -0.3;
	x1= -0.15;  //qi shi su du fang xiang 
	z1= -0.27;
	x2= -0.3;   //jie shu su du fang xiang		
	z2= 0.01;
	x3= 0.06;  //jie shu zuo biao
	z3= -0.30 ;
	x_start=0.04;	
	z_start=-0.25;
	//HAL_Delay(5000);
		/*__kp[1]=19;
		__kd[1]=7;
		__kp[2]=23;
		__kd[2]=6;
		__kp[3]=23;
		__kd[3]=6;
		__kp[4]=19;
		__kd[4]=7;*/
		__kp[1]=14;
		__kd[1]=14;
		__kp[2]=17;
		__kd[2]=14;
		__kp[3]=17;
		__kd[3]=14;
		__kp[4]=14;
		__kd[4]=14;
	loutiStep1();
	//printf("bound2\n");
	M_T=0.1;
		__kp[1]=19;
		__kd[1]=5;
		__kp[2]=19;
		__kd[2]=5;
		__kp[3]=15;
		__kd[3]=5;
		__kp[4]=18;
		__kd[4]=5;

	loutiStep2();
		//printf("bound3\n");
		__kp[1]=12;
		__kd[1]=5;
		__kp[2]=12;
		__kd[2]=5;
		__kp[3]=12;
		__kd[3]=5;
		__kp[4]=12;
		__kd[4]=5;
	
	loutiStep4();
}



void louti2()//前腿高 后腿低
{	
	//前腿在顶层  后腿在2层
	x0= -0.13 ;  //   //qi dian zuo biao 
	z0= -0.3;
	x1= -0.25;  //qi shi su du fang xiang 
	z1= -0.27;
	x2= -0.3;   //jie shu su du fang xiang		//-0.3
	z2= 0.15;//0.15
	x3= 0.0;  //jie shu zuo biao
	z3= -0.30 ;
	x_start=-0.075;	//0.0//-0.05
	z_start=-0.175;//-0.25//-0.2
	//HAL_Delay(5000);
		/*__kp[1]=19;
		__kd[1]=7;
		__kp[2]=23;
		__kd[2]=6;
		__kp[3]=23;
		__kd[3]=6;
		__kp[4]=19;
		__kd[4]=7;*/
		__kp[1]=12;//12
		__kd[1]=17;
		__kp[2]=12;//12
		__kd[2]=17;
		__kp[3]=14;//14
		__kd[3]=17;
		__kp[4]=12;
		__kd[4]=17;
	loutiStep1();
	//printf("bound2\n");
	M_T=0.1;
		__kp[1]=19;
		__kd[1]=5;
		__kp[2]=19;
		__kd[2]=5;
		__kp[3]=15;
		__kd[3]=5;
		__kp[4]=18;
		__kd[4]=5;

	loutiStep2();
		//printf("bound3\n");
		__kp[1]=12;
		__kd[1]=5;
		__kp[2]=12;
		__kd[2]=5;
		__kp[3]=12;
		__kd[3]=5;
		__kp[4]=12;
		__kd[4]=5;
	
	loutiStep4();
}
void louti26()//前腿高 后腿低
{	
	//前腿在顶层  后腿在2层
	x0= -0.13 ;  //   //qi dian zuo biao 
	z0= -0.3;
	x1= -0.2;  //qi shi su du fang xiang 
	z1= -0.27;
	x2= -0.3;   //jie shu su du fang xiang		//-0.3
	z2= 0.15;//0.15
	x3= 0.0;  //jie shu zuo biao
	z3= -0.30 ;
	x_start=-0.075;	//0.0//-0.05
	z_start=-0.175;//-0.25//-0.2
	//HAL_Delay(5000);
		/*__kp[1]=19;
		__kd[1]=7;
		__kp[2]=23;
		__kd[2]=6;
		__kp[3]=23;
		__kd[3]=6;
		__kp[4]=19;
		__kd[4]=7;*/
		__kp[1]=12;//12
		__kd[1]=17;
		__kp[2]=12;//12
		__kd[2]=17;
		__kp[3]=14;//14
		__kd[3]=17;
		__kp[4]=12;
		__kd[4]=17;
	loutiStep1();
	//printf("bound2\n");
	M_T=0.1;
		__kp[1]=19;
		__kd[1]=5;
		__kp[2]=19;
		__kd[2]=5;
		__kp[3]=15;
		__kd[3]=5;
		__kp[4]=18;
		__kd[4]=5;

	loutiStep2();
		//printf("bound3\n");
		__kp[1]=12;
		__kd[1]=5;
		__kp[2]=12;
		__kd[2]=5;
		__kp[3]=12;
		__kd[3]=5;
		__kp[4]=12;
		__kd[4]=5;
	
	loutiStep4();
}
void louti261()//前腿高 后腿低
{	
	//前腿在顶层  后腿在2层
	x0= -0.13 ;  //   //qi dian zuo biao 
	z0= -0.3;
	x1= -0.23;  //qi shi su du fang xiang 
	z1= -0.27;
	x2= -0.3;   //jie shu su du fang xiang		//-0.3
	z2= 0.15;//0.15
	x3= 0.0;  //jie shu zuo biao
	z3= -0.30 ;
	x_start=-0.075;	//0.0//-0.05
	z_start=-0.175;//-0.25//-0.2
	//HAL_Delay(5000);
		/*__kp[1]=19;
		__kd[1]=7;
		__kp[2]=23;
		__kd[2]=6;
		__kp[3]=23;
		__kd[3]=6;
		__kp[4]=19;
		__kd[4]=7;*/
		__kp[1]=12;//12
		__kd[1]=17;
		__kp[2]=12;//12
		__kd[2]=17;
		__kp[3]=14;//14
		__kd[3]=17;
		__kp[4]=12;
		__kd[4]=17;
	loutiStep1();
	//printf("bound2\n");
	M_T=0.1;
		__kp[1]=19;
		__kd[1]=5;
		__kp[2]=19;
		__kd[2]=5;
		__kp[3]=15;
		__kd[3]=5;
		__kp[4]=18;
		__kd[4]=5;

	loutiStep2();
		//printf("bound3\n");
		__kp[1]=12;
		__kd[1]=5;
		__kp[2]=12;
		__kd[2]=5;
		__kp[3]=12;
		__kd[3]=5;
		__kp[4]=12;
		__kd[4]=5;
	
	loutiStep4();
}

void loutiStep1()
{


//		for(int legnumber =1;legnumber<5;legnumber++)	
//		{
//		 M_leg[legnumber].M_expx=x0;
//		 M_leg[legnumber].M_expz=z0;	
//		}
				 M_leg[1].M_expx=x0;
				 M_leg[1].M_expz=z0+0.04;
				 M_leg[2].M_expx=x0;
		     M_leg[2].M_expz=z0-0.04;
				 M_leg[3].M_expx=x0;
		     M_leg[3].M_expz=z0-0.04;
				 M_leg[4].M_expx=x0;
		     M_leg[4].M_expz=z0+0.04;
		newIK();
		HAL_Delay(30);
}

void loutiStep2()
{	

	
	
	for(double t=0;t<(M_T*0.5);t+=0.002)
  {
		for(int legnumber =1;legnumber<5;legnumber++)	
		{
		
		M_leg[legnumber].M_expx=x0*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))+
			3*x1*t/(M_T*0.5)*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))+
			3*x2*(t/(M_T*0.5))*(t/(M_T*0.5))*(1-t/(M_T*0.5))+x3*(t/(M_T*0.5))*(t/(M_T*0.5))*(t/(M_T*0.5));
		M_leg[legnumber].M_expz=z0*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))+
			3*z1*t/(M_T*0.5)*(1-t/(M_T*0.5))*(1-t/(M_T*0.5))+
			3*z2*(t/(M_T*0.5))*(t/(M_T*0.5))*(1-t/(M_T*0.5))+z3*(t/(M_T*0.5))*(t/(M_T*0.5))*(t/(M_T*0.5));
		}
	newIK();
		M_leg[1].M_expx+=0.02;
		M_leg[4].M_expx+=0.02;
	}
}

void loutiStep4()//落地缓冲
{
		__kp[1]=12;//11
		__kd[1]=10;
		__kp[2]=8;
		__kd[2]=10;
		__kp[3]=8;
		__kd[3]=10;
		__kp[4]=10;//8
		__kd[4]=12;
	for(int legnumber=1;legnumber<5;legnumber++)
	{
		M_leg[legnumber].M_expx=x3;
		M_leg[legnumber].M_expz=z3;
	}
	M_leg[2].M_expx-=0.05;
	M_leg[3].M_expx-=0.05;
	M_leg[1].M_expz-=0.05;//原本没有这2行
	M_leg[4].M_expz-=0.05;//原本没有这2行
	newIK();
	HAL_Delay(150);
	__kp[1]=15;
	__kd[1]=100;
	__kp[2]=15;
	__kd[2]=100;
	__kp[3]=15;
	__kd[3]=100;
	__kp[4]=15;
	__kd[4]=100;

		M_leg[1].M_expx=x_start;//下次往后来一点，不要太前//0
		M_leg[1].M_expz=z_start;
		M_leg[4].M_expx=x_start;//0
		M_leg[4].M_expz=z_start;
		M_leg[2].M_expx=-x_start-0.08;//下次修改加一些，防止每次跳跃结束往前送一段//-0.08
		M_leg[2].M_expz=z_start;
		M_leg[3].M_expx=-x_start-0.08;//-0.08
		M_leg[3].M_expz=z_start;
	
	newIK();
	//HAL_Delay(1050);
}




