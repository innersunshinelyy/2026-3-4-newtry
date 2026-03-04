#ifndef NEWTROT_H
#define NEWTROT_H

void dogInit(double x,double y,double z);
void trot();

void M_statechange();
void JingSuSai(void);
void Shuang();
void LouTi();
void Shuang111();
void  Shuang222();
void louti_new();
void jinsusai1();
void jingsusai2 ();
void jingsusai3();
void zhangaisai();
void zhangaisai1();
void zhangaisai2();
void zhangaisai3();
void zhangaisai4();
void ceshi();
void printMotionMenu();

void right_slow();
void left_slow();
void jump_change();
void baizheng();
void shuangbu_walk(void);
void zuozhuan(void);
void youzhuan(void);
void louti_jump(void);
void lou(void);

void bound_forward();
void slalom(void);
void Pivot_turn_left();
void Pivot_turn_right();
void Diagonal_forward();
void Low_posture();
void slope1();

void M_statechange();
void ramp();
void ramp_trot();
void bound_forward();
void slalom(void);
void Pivot_turn_left();
void Pivot_turn_right();

void TempTrot();
void left_slow(void);
void right_slow(void);
void jump_best();
void ramp_walk();
void pufu();
void jump_small();
void raogan(int flag);
void shakeng_with_Gyro(int flag);
void PID_walk(double target_direction);
void dash_with_gyro();
void high_speed_raogan(int flag);
void radarcontrol(float x,float y);

void rotate_in_place_by22_5_left(int number);
void rotate_in_place_by22_5_right(int number);
void move_forward_for_steps(int number);
void rotate_in_place_to_targetAngle(float angle);
void rotate_in_place_to_targetAngle1(float angle);
void move_forward_for_small_steps(int number);

typedef struct {
    double x;
    double y;
    double pitch_now;
		double distance_square_to_next_point;
		int next_state;
		double next_angle;
} path;
void from_a2b();
void move_to_targetDistance();
void rotate_in_place_to_targetAngle_by_radar();
double distance_point_to_vector(double ax, double ay, double bx, double by, double px, double py);
void rotate_in_place_to_targetAngle_by_radar_for_line();
void move_to_certain_line();
void from_a2b_v2();
double distance_to_perpendicular_line(double x1, double y1, double x2, double y2);
void from_a2b_v3();
void rotate_in_place_to_targetAngle_by_radar_for_line_v3();
void move_to_certain_line_v3();

void trot_slow();
void trot_testing();
void trot_testing_without_delay();
void trot_testing_running();
void rotate_in_place_to_targetAngle_by_radar_for_line_v3_plus();

void big_jump();
void small_jump();
void sandBox_all();
void broken_bridge_all();

void from_a2b_v3_dizipufu();
void rotate_in_place_to_targetAngle_by_radar_for_line_v3_dizipufu();
void move_to_certain_line_v3_dizipufu();
void control_angle1(float target_angle);

void high_leg_prepare();
void control_angle(float target_angle);
void control_angle_pufu(float target_angle);
	
void JingSuBiHuan_1_0();
void trot_JingSuBiHuan();
void turn_45_right_dizipufu();
void turn_45_left_dizipufu();
void dizipufu_all();
void ramp_walk_2_0(float target_angle);

void high_knee_walk();
void shakeng_high_knee(int step);
void trot_walkUP();
void walkUP_with_radar(float target_angle);
void trot_walkDOWN();
void walkDOWN_with_radar(float target_angle);
void jump_middle();

void trot_walk_right_or_left();
void real_ramp_jump();
void rotate_in_place_to_targetAngle_for_high_knee(float angle);
void jump_right();
void jump_left();
void yueye_walk();

void rotate_in_place_to_targetAngle_by_radar_for_line_v3_yueye();
void trot_for_yueye();
void move_to_certain_line_v3_for_yueye();

void control_position(float target_angle);
void trot_walk_AUTO_UPDOWN();
void AUTO_walkUPDOWN_with_radar(float target_angle);
void AUTO_walkUPDOWN_withOUT_radar();

void walkUP_withOUT_radar();
void control_position_2(float target_angle);
void back_on_DuanQiao(int steps);
void trot_walkUP_DuanQiao();
void trot_run();
#endif
