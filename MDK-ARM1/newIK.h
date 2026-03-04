#ifndef NEWIK_H
#define NEWIK_H

struct M_ANGLE
{
double angle_0;//out towards
double angle_1;
};
void newIK();
void TempNewIK();
#define L1 0.15
#define L2 0.288


extern float __kp[5];
extern float __kd[5];
extern struct M_ANGLE M_angle[5];

struct canshu
{
float M_expx;
float M_expz;
float M_realx;
float M_realz;
float turnMode;
};


struct moment_force//存放输出力矩的结构体
{
    float force_0;
    float force_1;
};
extern struct moment_force MomentForce[5];
extern  struct canshu M_leg[5];

#endif
