#ifndef _comm_8010_task_H
#define _comm_8010_task_H

#include "stm32f4xx.h"
#include "User.h"
#include "motor_task.h"

void coom_8010_task(void *parm);
void go_8010_test_tesk1(int id,float T,float Pos,float W,float K_P,float K_W);
void go_8010_init(int id,float torque);
int go_8010_stop(int id);
float tamp_task(float input,float step,float ref_angle);
void ref_8010_angle(float get_angle[]);
void get_8010_angle(float receive_angle[]);
void go_8010_init_task(int GO_ID,float speed);
int judge_duzhuan_OK(uint8_t GO_ID,uint8_t modle_type);



#define Single_mode 1
#define Both_mode   0
#endif 
