/*****************************************************
	  �������д��2024/4/-
		
		��ȥ������һ�������Ҫȥbsp_can�ļ��������ո�ʽ��
ȥcommon�ĽǶ�ת�����뷢�͸�ʽ��ȥ��Ӧ�����ļ��������ĳ�ʼ��
		��ȥ����е����̧��������Ĵ����߼�����һ�£���������������
	  
		��ͳһmanipulator��slide��chassis��������߼���
��������ĵ��ID���ٶ���ǶȻ�ѡ���ȫ����ͷ�ļ�ͳһ�޸�
ʣ�½����������д���

		Ϊͳһ���õ�ʱ�������ɣ�
*****************************************************/
#include "motor_task.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "comm_task.h"
#include "modeswitch_task.h"
#include "clamp.h"
#include "motor_8010.h"
#include "stm32f4xx.h"
#include "comm_8010_task.h"
int16_t test_angle1=2500;
int16_t small_test1=100;
UBaseType_t Motor_stack_surplus;
extern TaskHandle_t can_msg_send_Task_Handle;
extern TaskHandle_t comm_8010_task_Handle;
extern int16_t go_8010_current;

extern float  send_current_8010_1;
extern float  send_current_8010_2;
extern float send_ref_angle_8010_1;
extern float send_ref_angle_8010_2;
extern float send_fdb_angle_8010_1;
extern float send_fdb_angle_8010_2;


motor_t Motor[MOTOR_NUMBER];
Machine_Angle_T Machine_angle[Machine_NUMBER];

uint8_t Motor_init_state=INIT_NEVER;
uint8_t All_Offset_Angle_init_state=0;

uint32_t Servo_Test;
float test_pid_speed_out=0;
float test_pid=0;
int32_t test_angle;
float test_ref[14];
float l_test_ref[14];
uint32_t go_8010_init_time=0;


motor_angle_debug motor_debug[3]=
{
	{0,0,0,0},
	{0,0,0,0},
	{0,0,0,0}
};//offset target normal mode 


//�����ʼ�����
void Motor_base_init()
{
	motor_t Motor_Struct;
	//����ճ����ֻ�ø�һ�����־���������Ӧ�����������ǰǿ������
	{// CHASSIS INIT
    Motor_Struct.MOTOR_NAME = CHASSIS_FR;
    Motor_Struct.MOTOR_TYPE = M3508;
    Motor_Struct.Brushless.CAN_ID = 1;
    Motor_Struct.Brushless.ESC_ID = 1;
    Motor_Struct.Brushless.speed = 0; // ��ǶȲ����ٶ�
    Motor_Struct.Brushless.state = 0;
    Motor_Struct.Brushless.offset_angle_init_flag = 0;
		Motor_Struct.Brushless.offset_angle_init_speed = 0;

    memcpy(&Motor[Motor_Struct.MOTOR_NAME], &Motor_Struct, sizeof(Motor_Struct));
    memset(&Motor_Struct, 0, sizeof(Motor_Struct));
	}

	{// UPRISE INIT
    Motor_Struct.MOTOR_NAME = UPRAISE;
    Motor_Struct.MOTOR_TYPE = M3508;
    Motor_Struct.Brushless.CAN_ID = 1;
    Motor_Struct.Brushless.ESC_ID = 5;
		Motor_Struct.Brushless.Speed_or_Angle_flag=1;//�ó�ʼ��ʱ�ܹ����ٶȻ���ת
		Motor_Struct.Brushless.Angle_to_Speed_mode=0;//���л�
    Motor_Struct.Brushless.speed = 1500; // ��ǶȲ����ٶ�
    Motor_Struct.Brushless.state = 0;
    Motor_Struct.Brushless.offset_angle_init_flag = 0;
		Motor_Struct.Brushless.offset_angle_init_speed = -700;
		

    memcpy(&Motor[Motor_Struct.MOTOR_NAME], &Motor_Struct, sizeof(Motor_Struct));
    memset(&Motor_Struct, 0, sizeof(Motor_Struct));
	}
	{// UPRISE_CLAMP INIT
    Motor_Struct.MOTOR_NAME = UPRAISE_CLAMP;
    Motor_Struct.MOTOR_TYPE = M3508;
    Motor_Struct.Brushless.CAN_ID = 2;
    Motor_Struct.Brushless.ESC_ID = 6;
		Motor_Struct.Brushless.Speed_or_Angle_flag=1;//�ó�ʼ��ʱ�ܹ����ٶȻ���ת
		Motor_Struct.Brushless.Angle_to_Speed_mode=0;//���л�
    Motor_Struct.Brushless.speed = 1500; // ��ǶȲ����ٶ�
    Motor_Struct.Brushless.state = 0;
    Motor_Struct.Brushless.offset_angle_init_flag = 0;
		Motor_Struct.Brushless.offset_angle_init_speed = 700;
		

    memcpy(&Motor[Motor_Struct.MOTOR_NAME], &Motor_Struct, sizeof(Motor_Struct));
    memset(&Motor_Struct, 0, sizeof(Motor_Struct));
	}

{// CLAMP_SILD INIT
    Motor_Struct.MOTOR_NAME = CLAMP_SILD;
    Motor_Struct.MOTOR_TYPE = M2006;
    Motor_Struct.Brushless.CAN_ID = 2;
    Motor_Struct.Brushless.ESC_ID = 5;
		Motor_Struct.Brushless.Speed_or_Angle_flag=1;
    Motor_Struct.Brushless.speed = 0; // ��ǶȲ����ٶ�
    Motor_Struct.Brushless.state = 0;
    Motor_Struct.Brushless.offset_angle_init_flag = 0;
		Motor_Struct.Brushless.offset_angle_init_speed = -1500;

  

    memcpy(&Motor[Motor_Struct.MOTOR_NAME], &Motor_Struct, sizeof(Motor_Struct));
    memset(&Motor_Struct, 0, sizeof(Motor_Struct));
	}

	{// clamp_pitch_and_roll_l INIT
    Motor_Struct.MOTOR_NAME = clamp_pitch_and_roll_l;
    Motor_Struct.MOTOR_TYPE = M2006;
    Motor_Struct.Brushless.CAN_ID = 2;
    Motor_Struct.Brushless.ESC_ID = 4;
		Motor_Struct.Brushless.Speed_or_Angle_flag=1;
    Motor_Struct.Brushless.speed = 0; // ��ǶȲ����ٶ�
    Motor_Struct.Brushless.state = 0;
    Motor_Struct.Brushless.offset_angle_init_flag = 0;
		Motor_Struct.Brushless.offset_angle_init_speed = -800;

  

    memcpy(&Motor[Motor_Struct.MOTOR_NAME], &Motor_Struct, sizeof(Motor_Struct));
    memset(&Motor_Struct, 0, sizeof(Motor_Struct));
	}

	{// clamp_pitch_and_roll_r
    Motor_Struct.MOTOR_NAME = clamp_pitch_and_roll_r;
    Motor_Struct.MOTOR_TYPE = M2006;
    Motor_Struct.Brushless.CAN_ID = 2;
    Motor_Struct.Brushless.ESC_ID = 7;
	Motor_Struct.Brushless.Speed_or_Angle_flag=1;
    Motor_Struct.Brushless.speed = 0; // ��ǶȲ����ٶ�
    Motor_Struct.Brushless.state = 0;
    Motor_Struct.Brushless.offset_angle_init_flag = 0;
		Motor_Struct.Brushless.offset_angle_init_speed = 800;

    memcpy(&Motor[Motor_Struct.MOTOR_NAME], &Motor_Struct, sizeof(Motor_Struct));
    memset(&Motor_Struct, 0, sizeof(Motor_Struct));
	}



	{// joint_3_yaw
    Motor_Struct.MOTOR_NAME = joint_3_yaw;
    Motor_Struct.MOTOR_TYPE = M2006;
    Motor_Struct.Brushless.CAN_ID = 2;
    Motor_Struct.Brushless.ESC_ID = 3;
		Motor_Struct.Brushless.Speed_or_Angle_flag=1;
    Motor_Struct.Brushless.speed = 0; // ��ǶȲ����ٶ�
    Motor_Struct.Brushless.state = 0;
    Motor_Struct.Brushless.offset_angle_init_flag = 0;
		Motor_Struct.Brushless.offset_angle_init_speed = -1500;

    memcpy(&Motor[Motor_Struct.MOTOR_NAME], &Motor_Struct, sizeof(Motor_Struct));
    memset(&Motor_Struct, 0, sizeof(Motor_Struct));
	}

	{// joint_1
    Motor_Struct.MOTOR_NAME = joint_1;
    Motor_Struct.MOTOR_TYPE = go_8010;
		Motor_Struct.Brushless.GO_ID=GO_8010_1;
    Motor_Struct.Brushless.speed = 0; // ��ǶȲ����ٶ�
    Motor_Struct.Brushless.state = 0;
    Motor_Struct.Brushless.offset_angle_init_flag = 0;
		Motor_Struct.Brushless.offset_angle_init_speed = 120;

    memcpy(&Motor[Motor_Struct.MOTOR_NAME], &Motor_Struct, sizeof(Motor_Struct));
    memset(&Motor_Struct, 0, sizeof(Motor_Struct));
	}

	{// joint_2 INIT
    Motor_Struct.MOTOR_NAME = joint_2;
    Motor_Struct.MOTOR_TYPE = go_8010; 
		Motor_Struct.Brushless.GO_ID=GO_8010_2; 
    Motor_Struct.Brushless.speed = 0; // ��ǶȲ����ٶ�
    Motor_Struct.Brushless.state = 0;
    Motor_Struct.Brushless.offset_angle_init_flag = 0;
		Motor_Struct.Brushless.offset_angle_init_speed = -1000;

    memcpy(&Motor[Motor_Struct.MOTOR_NAME], &Motor_Struct, sizeof(Motor_Struct));
    memset(&Motor_Struct, 0, sizeof(Motor_Struct));
	}
	

	{// P_T_Pitch INIT
    Motor_Struct.MOTOR_NAME = P_T_Pitch;
    Motor_Struct.MOTOR_TYPE = SERVO;
    Motor_Struct.Servo.TIM = TIM4;
    Motor_Struct.Servo.Compare = 4; // PD15
    Motor_Struct.Servo.Rotation_range = 270;

    memcpy(&Motor[Motor_Struct.MOTOR_NAME], &Motor_Struct, sizeof(Motor_Struct));
    memset(&Motor_Struct, 0, sizeof(Motor_Struct));
	}
	
	{// P_T_ROLL INIT
    Motor_Struct.MOTOR_NAME = P_T_Yaw;
    Motor_Struct.MOTOR_TYPE = SERVO;
    Motor_Struct.Servo.TIM = TIM4;
    Motor_Struct.Servo.Compare = 2; // PA2
    Motor_Struct.Servo.Rotation_range = 270;

    memcpy(&Motor[Motor_Struct.MOTOR_NAME], &Motor_Struct, sizeof(Motor_Struct));
    memset(&Motor_Struct, 0, sizeof(Motor_Struct));
	}
	
	{// exchang_pitch_and_roll_l INIT
    Motor_Struct.MOTOR_NAME = exchang_pitch_and_roll_l;
    Motor_Struct.MOTOR_TYPE = M2006;
    Motor_Struct.Brushless.CAN_ID = 2;
    Motor_Struct.Brushless.ESC_ID = 1;
		Motor_Struct.Brushless.Speed_or_Angle_flag=1;
    Motor_Struct.Brushless.speed = 0; // ��ǶȲ����ٶ�
    Motor_Struct.Brushless.state = 0;
    Motor_Struct.Brushless.offset_angle_init_flag = 0;
		Motor_Struct.Brushless.offset_angle_init_speed = 800;

    memcpy(&Motor[Motor_Struct.MOTOR_NAME], &Motor_Struct, sizeof(Motor_Struct));
    memset(&Motor_Struct, 0, sizeof(Motor_Struct));
	}
	{// SLIDE_CLAMP INIT
    Motor_Struct.MOTOR_NAME = exchang_pitch_and_roll_r;
    Motor_Struct.MOTOR_TYPE = M2006;
    Motor_Struct.Brushless.CAN_ID = 2;
    Motor_Struct.Brushless.ESC_ID = 2;
		Motor_Struct.Brushless.Speed_or_Angle_flag=1;
    Motor_Struct.Brushless.speed = 0; // ��ǶȲ����ٶ�
    Motor_Struct.Brushless.state = 0;
    Motor_Struct.Brushless.offset_angle_init_flag = 0;
		Motor_Struct.Brushless.offset_angle_init_speed = -800;

    memcpy(&Motor[Motor_Struct.MOTOR_NAME], &Motor_Struct, sizeof(Motor_Struct));
    memset(&Motor_Struct, 0, sizeof(Motor_Struct));
	}
	
	
	Motor_pid_init(DONE);
	
	
	Motor_base_init_copy(CHASSIS_FR,CHASSIS_BR);//���ø���

	Machine_angle_init();
	Motor_angle_init_test();
	//Motor_base_init_reversal(SLIDE_R);//��תSLIDE_R����
//	Motor[SLIDE_R].MOTOR_TURN_FLAG=M_TURN;
}

//��ʼ��pid
void Motor_pid_init(INIT_STATUS init_status) 
{
	pid_motor_parameter_t PID_Motor_parameter_Struct;

	// CHASSIS_FR
	{
    PID_Motor_parameter_Struct.MOTOR_NAME = CHASSIS_FR;
    PID_Motor_parameter_Struct.angle.p = 60.0;
    PID_Motor_parameter_Struct.angle.i = 0.01;
    PID_Motor_parameter_Struct.angle.d = 0.1;
    PID_Motor_parameter_Struct.angle.max_out = 30000;
    PID_Motor_parameter_Struct.angle.integral_limit = 30000;

    PID_Motor_parameter_Struct.speed.p = 60.0;
    PID_Motor_parameter_Struct.speed.i = 0.01;
    PID_Motor_parameter_Struct.speed.d = 0.1;
    PID_Motor_parameter_Struct.speed.max_out = 30000;
    PID_Motor_parameter_Struct.speed.integral_limit = 30000;

    Motor_PID_Struct_Init(&Motor[CHASSIS_FR], PID_Motor_parameter_Struct, init_status);
	}

	// UPRISE
	{
    PID_Motor_parameter_Struct.MOTOR_NAME = UPRAISE;
    PID_Motor_parameter_Struct.angle.p = 30;
    PID_Motor_parameter_Struct.angle.i = 0.2;
    PID_Motor_parameter_Struct.angle.d = 0;
    PID_Motor_parameter_Struct.angle.max_out = 5000;
    PID_Motor_parameter_Struct.angle.integral_limit = 500;

    PID_Motor_parameter_Struct.speed.p = 20.0;
    PID_Motor_parameter_Struct.speed.i = 0.001;
    PID_Motor_parameter_Struct.speed.d = 0;
    PID_Motor_parameter_Struct.speed.max_out = 4000;
    PID_Motor_parameter_Struct.speed.integral_limit = 500;

    Motor_PID_Struct_Init(&Motor[UPRAISE], PID_Motor_parameter_Struct, init_status);
	}
	
	// UPRISE_CLAMP
	{
    PID_Motor_parameter_Struct.MOTOR_NAME = UPRAISE_CLAMP;
    PID_Motor_parameter_Struct.angle.p = 30;
    PID_Motor_parameter_Struct.angle.i = 0.2;
    PID_Motor_parameter_Struct.angle.d = 0;
    PID_Motor_parameter_Struct.angle.max_out =5000;
    PID_Motor_parameter_Struct.angle.integral_limit = 500;

    PID_Motor_parameter_Struct.speed.p = 20.0;
    PID_Motor_parameter_Struct.speed.i = 0.001;
    PID_Motor_parameter_Struct.speed.d = 0;
    PID_Motor_parameter_Struct.speed.max_out = 4000;
    PID_Motor_parameter_Struct.speed.integral_limit = 500;

    Motor_PID_Struct_Init(&Motor[UPRAISE_CLAMP], PID_Motor_parameter_Struct, init_status);
	}

	// CLAMP_YAW
	{
    PID_Motor_parameter_Struct.MOTOR_NAME = CLAMP_YAW;
    PID_Motor_parameter_Struct.angle.p = 0;
    PID_Motor_parameter_Struct.angle.i = 0;
    PID_Motor_parameter_Struct.angle.d = 0;
    PID_Motor_parameter_Struct.angle.max_out = 0;
    PID_Motor_parameter_Struct.angle.integral_limit = 0;

    PID_Motor_parameter_Struct.speed.p = 60.0;
    PID_Motor_parameter_Struct.speed.i = 0.01;
    PID_Motor_parameter_Struct.speed.d = 0.1;
    PID_Motor_parameter_Struct.speed.max_out = 1000;
    PID_Motor_parameter_Struct.speed.integral_limit = 1000;

    Motor_PID_Struct_Init(&Motor[CLAMP_YAW], PID_Motor_parameter_Struct, init_status);
	}
        //clamp_pitch_and_roll_l
	{
    PID_Motor_parameter_Struct.MOTOR_NAME = clamp_pitch_and_roll_l;
    PID_Motor_parameter_Struct.angle.p = 30;
    PID_Motor_parameter_Struct.angle.i = 0.2;
    PID_Motor_parameter_Struct.angle.d = 0;
    PID_Motor_parameter_Struct.angle.max_out = 2000;
    PID_Motor_parameter_Struct.angle.integral_limit = 500;

    PID_Motor_parameter_Struct.speed.p = 20.0;
    PID_Motor_parameter_Struct.speed.i = 0.01;
    PID_Motor_parameter_Struct.speed.d = 0.;
    PID_Motor_parameter_Struct.speed.max_out = 4000;
    PID_Motor_parameter_Struct.speed.integral_limit = 1500;

    Motor_PID_Struct_Init(&Motor[clamp_pitch_and_roll_l], PID_Motor_parameter_Struct, init_status);
	}

	// CLAMP_SILD
	{
    PID_Motor_parameter_Struct.MOTOR_NAME = CLAMP_SILD;
    PID_Motor_parameter_Struct.angle.p = 15.5;
    PID_Motor_parameter_Struct.angle.i = 0.1;
    PID_Motor_parameter_Struct.angle.d = 0;
    PID_Motor_parameter_Struct.angle.max_out = 4000;
    PID_Motor_parameter_Struct.angle.integral_limit = 500;

    PID_Motor_parameter_Struct.speed.p = 16;
    PID_Motor_parameter_Struct.speed.i = 0.01;
    PID_Motor_parameter_Struct.speed.d = 0;
    PID_Motor_parameter_Struct.speed.max_out = 4000;
    PID_Motor_parameter_Struct.speed.integral_limit = 500;

    Motor_PID_Struct_Init(&Motor[CLAMP_SILD], PID_Motor_parameter_Struct, init_status);
	}


	// joint_3_yaw
	{
    PID_Motor_parameter_Struct.MOTOR_NAME = joint_3_yaw;
    PID_Motor_parameter_Struct.angle.p = 30;
    PID_Motor_parameter_Struct.angle.i = 0.2;
    PID_Motor_parameter_Struct.angle.d = 0;
    PID_Motor_parameter_Struct.angle.max_out = 4400;
    PID_Motor_parameter_Struct.angle.integral_limit = 500;

    PID_Motor_parameter_Struct.speed.p = 20.0;
    PID_Motor_parameter_Struct.speed.i = 0.01;
    PID_Motor_parameter_Struct.speed.d = 0.;
    PID_Motor_parameter_Struct.speed.max_out = 3000;
    PID_Motor_parameter_Struct.speed.integral_limit = 1500;

    Motor_PID_Struct_Init(&Motor[joint_3_yaw], PID_Motor_parameter_Struct, init_status);
	}
	
	// joint_1
	{
    PID_Motor_parameter_Struct.MOTOR_NAME = joint_1;
    PID_Motor_parameter_Struct.angle.p =1;
    PID_Motor_parameter_Struct.angle.i = 0;
    PID_Motor_parameter_Struct.angle.d =10;
    PID_Motor_parameter_Struct.angle.max_out =10;
    PID_Motor_parameter_Struct.angle.integral_limit = 0.6;

    PID_Motor_parameter_Struct.speed.p = 0.01;
    PID_Motor_parameter_Struct.speed.i = 0;
    PID_Motor_parameter_Struct.speed.d = 0.1;
    PID_Motor_parameter_Struct.speed.max_out = 0.05;
    PID_Motor_parameter_Struct.speed.integral_limit = 0.6;

    Motor_PID_Struct_Init(&Motor[joint_1], PID_Motor_parameter_Struct, init_status);
	}

	// joint_2
	{
    PID_Motor_parameter_Struct.MOTOR_NAME = joint_2;
    PID_Motor_parameter_Struct.angle.p = 0.1;
    PID_Motor_parameter_Struct.angle.i = 0;
    PID_Motor_parameter_Struct.angle.d = 50;
    PID_Motor_parameter_Struct.angle.max_out =0.1;
    PID_Motor_parameter_Struct.angle.integral_limit = 0.6;

    PID_Motor_parameter_Struct.speed.p = 1;
    PID_Motor_parameter_Struct.speed.i = 0;
    PID_Motor_parameter_Struct.speed.d = 0;
    PID_Motor_parameter_Struct.speed.max_out = 0.2;
    PID_Motor_parameter_Struct.speed.integral_limit = 0.6;

    Motor_PID_Struct_Init(&Motor[joint_2], PID_Motor_parameter_Struct, init_status);
	}
       //clamp_pitch_and_roll_r
	{
    PID_Motor_parameter_Struct.MOTOR_NAME = clamp_pitch_and_roll_r;
    PID_Motor_parameter_Struct.angle.p = 30;
    PID_Motor_parameter_Struct.angle.i = 0.2;
    PID_Motor_parameter_Struct.angle.d = 0;
    PID_Motor_parameter_Struct.angle.max_out = 2000;
    PID_Motor_parameter_Struct.angle.integral_limit = 500;

    PID_Motor_parameter_Struct.speed.p = 20.0;
    PID_Motor_parameter_Struct.speed.i = 0.01;
    PID_Motor_parameter_Struct.speed.d = 0.1;
    PID_Motor_parameter_Struct.speed.max_out = 4000;
    PID_Motor_parameter_Struct.speed.integral_limit = 1500;

    Motor_PID_Struct_Init(&Motor[clamp_pitch_and_roll_r], PID_Motor_parameter_Struct, init_status);
	}
	// exchang_pitch_and_roll_l
	{
    PID_Motor_parameter_Struct.MOTOR_NAME = exchang_pitch_and_roll_l;
    PID_Motor_parameter_Struct.angle.p = 30;
    PID_Motor_parameter_Struct.angle.i = 0.2;
    PID_Motor_parameter_Struct.angle.d = 0;
    PID_Motor_parameter_Struct.angle.max_out = 4000;
    PID_Motor_parameter_Struct.angle.integral_limit = 500;

    PID_Motor_parameter_Struct.speed.p = 20;
    PID_Motor_parameter_Struct.speed.i = 0.01;
    PID_Motor_parameter_Struct.speed.d = 0.1;
    PID_Motor_parameter_Struct.speed.max_out = 4000;
    PID_Motor_parameter_Struct.speed.integral_limit = 500;

    Motor_PID_Struct_Init(&Motor[exchang_pitch_and_roll_l], PID_Motor_parameter_Struct, init_status);
	}
		// exchang_pitch_and_roll_r
	{
    PID_Motor_parameter_Struct.MOTOR_NAME = exchang_pitch_and_roll_r;
    PID_Motor_parameter_Struct.angle.p = 30;
    PID_Motor_parameter_Struct.angle.i = 0.2;
    PID_Motor_parameter_Struct.angle.d = 0;
    PID_Motor_parameter_Struct.angle.max_out = 4000;
    PID_Motor_parameter_Struct.angle.integral_limit = 500;

    PID_Motor_parameter_Struct.speed.p = 20.0;
    PID_Motor_parameter_Struct.speed.i = 0.01;
    PID_Motor_parameter_Struct.speed.d = 0.1;
    PID_Motor_parameter_Struct.speed.max_out = 4000;
    PID_Motor_parameter_Struct.speed.integral_limit = 500;

    Motor_PID_Struct_Init(&Motor[exchang_pitch_and_roll_r], PID_Motor_parameter_Struct, init_status);
	}
}

//�����Ƕȳ�ʼ��
void Machine_angle_init()
{
	
	{
		Machine_angle[Upraise_clamp].Machine_angle_l.normal_angle=0;//max805  23  
		Machine_angle[Upraise].Machine_angle_l.normal_angle=0;	//max2305	
		Machine_angle[Clamp_sild].Machine_angle_l.normal_angle=0;
		Machine_angle[Clamp_pitch].Machine_angle_l.normal_angle=135;
		Machine_angle[Clamp_roll].Machine_angle_l.normal_angle=0;
		Machine_angle[Joint_1].Machine_angle_l.normal_angle=-1.239*6.33;//-0.261
		Machine_angle[Joint_2].Machine_angle_l.normal_angle= 0.535*6.33;//2.355*6.33
		Machine_angle[Actuator_yaw].Machine_angle_l.normal_angle=350/3+90;
		Machine_angle[Actuator_roll].Machine_angle_l.normal_angle=0;
		Machine_angle[Actuator_pitch].Machine_angle_l.normal_angle=-40;
	
		Machine_angle[P_T_pitch].Machine_angle_l.normal_angle=220;
		Machine_angle[P_T_yaw].Machine_angle_l.normal_angle=168;
		
		
		
	}
	{
		Machine_angle[Upraise_clamp].Machine_angle_l.exchange_angle=0;
		Machine_angle[Upraise].Machine_angle_l.exchange_angle=500;
		Machine_angle[Clamp_sild].Machine_angle_l.exchange_angle=4;		
		Machine_angle[Clamp_pitch].Machine_angle_l.exchange_angle=0;
		Machine_angle[Clamp_roll].Machine_angle_l.exchange_angle=0;
		Machine_angle[Clamp_yaw].Machine_angle_l.exchange_angle=0;
		Machine_angle[Joint_1].Machine_angle_l.exchange_angle=0.769*6.33;
		Machine_angle[Joint_2].Machine_angle_l.exchange_angle=2.218*6.33;
		Machine_angle[Actuator_yaw].Machine_angle_l.exchange_angle=-90.6;
		Machine_angle[Actuator_roll].Machine_angle_l.exchange_angle=0;
		Machine_angle[Actuator_pitch].Machine_angle_l.exchange_angle=0;
		
		Machine_angle[P_T_pitch].Machine_angle_l.exchange_angle=-54;
		Machine_angle[P_T_yaw].Machine_angle_l.exchange_angle=0;
		
	}
	
	{
		Machine_angle[Upraise_clamp].Machine_angle_l.store_angle=22;
		Machine_angle[Upraise].Machine_angle_l.store_angle=700;         //366
		Machine_angle[Clamp_sild].Machine_angle_l.store_angle=4;
		Machine_angle[Clamp_pitch].Machine_angle_l.store_angle=0;
		Machine_angle[Clamp_roll].Machine_angle_l.store_angle=0;        
		Machine_angle[Clamp_yaw].Machine_angle_l.store_angle=0;
		Machine_angle[Joint_1].Machine_angle_l.store_angle=-3.053884;    //-1.935
		Machine_angle[Joint_2].Machine_angle_l.store_angle=3.5314;     //2.36
		Machine_angle[Actuator_yaw].Machine_angle_l.store_angle=-122;
		Machine_angle[Actuator_roll].Machine_angle_l.store_angle=0;
		Machine_angle[Actuator_pitch].Machine_angle_l.store_angle=-5;
        
		Machine_angle[P_T_pitch].Machine_angle_l.store_angle=0;
		Machine_angle[P_T_yaw].Machine_angle_l.store_angle=0;		
		
		
	}
	
	{
		Machine_angle[Upraise_clamp].Machine_angle_l.store_angle2=0;
		Machine_angle[Upraise].Machine_angle_l.store_angle2=2505;         //366
		Machine_angle[Clamp_sild].Machine_angle_l.store_angle2=4;
		Machine_angle[Clamp_pitch].Machine_angle_l.store_angle2=0;
		Machine_angle[Clamp_roll].Machine_angle_l.store_angle2=0;        
		Machine_angle[Clamp_yaw].Machine_angle_l.store_angle2=0;
		Machine_angle[Joint_1].Machine_angle_l.store_angle2=-8.04743958;    //-8.4221   
		Machine_angle[Joint_2].Machine_angle_l.store_angle2=-0.932180405;     //-0.545  
		Machine_angle[Actuator_yaw].Machine_angle_l.store_angle2=-25/3;
		Machine_angle[Actuator_roll].Machine_angle_l.store_angle2=0;
		Machine_angle[Actuator_pitch].Machine_angle_l.store_angle2=-10;
        
		Machine_angle[P_T_pitch].Machine_angle_l.store_angle2=0;
		Machine_angle[P_T_yaw].Machine_angle_l.store_angle2=80;		
		
		
	}
	{
		Machine_angle[Upraise_clamp].Machine_angle_l.store_angle3=9.2;
		Machine_angle[Upraise].Machine_angle_l.store_angle3=0;         //366
		Machine_angle[Clamp_sild].Machine_angle_l.store_angle3=0;
		Machine_angle[Clamp_pitch].Machine_angle_l.store_angle3=0;
		Machine_angle[Clamp_roll].Machine_angle_l.store_angle3=0;        
		Machine_angle[Clamp_yaw].Machine_angle_l.store_angle3=0;
		Machine_angle[Joint_1].Machine_angle_l.store_angle3=-1.5458588;    //-8.4221   
		Machine_angle[Joint_2].Machine_angle_l.store_angle3=3.5838575-3.3126999;     //-0.545  
		Machine_angle[Actuator_yaw].Machine_angle_l.store_angle3=-331/3;
		
		Machine_angle[Actuator_roll].Machine_angle_l.store_angle3=0;
		Machine_angle[Actuator_pitch].Machine_angle_l.store_angle3=0;
        
		Machine_angle[P_T_pitch].Machine_angle_l.store_angle3=61;
		Machine_angle[P_T_yaw].Machine_angle_l.store_angle3=-133;		
		
		
	}
	
	{
		Machine_angle[Upraise_clamp].Machine_angle_l.exchange_pick_angle=22;
		Machine_angle[Upraise].Machine_angle_l.exchange_pick_angle=700;
		Machine_angle[Clamp_sild].Machine_angle_l.exchange_pick_angle=4;
		Machine_angle[Clamp_pitch].Machine_angle_l.exchange_pick_angle=0;
		Machine_angle[Clamp_roll].Machine_angle_l.exchange_pick_angle=0;
		Machine_angle[Clamp_yaw].Machine_angle_l.exchange_pick_angle=0;
		Machine_angle[Joint_1].Machine_angle_l.exchange_pick_angle=-3.053884;
		Machine_angle[Joint_2].Machine_angle_l.exchange_pick_angle=3.5314;
		Machine_angle[Actuator_yaw].Machine_angle_l.exchange_pick_angle=-122;
		Machine_angle[Actuator_roll].Machine_angle_l.exchange_pick_angle=0;
		Machine_angle[Actuator_pitch].Machine_angle_l.exchange_pick_angle=-5;
		
	    Machine_angle[P_T_pitch].Machine_angle_l.exchange_pick_angle=0;
		Machine_angle[P_T_yaw].Machine_angle_l.exchange_pick_angle=0;				
		
	}
	{
		Machine_angle[Upraise_clamp].Machine_angle_l.exchange_pick_angle_2=200;
		Machine_angle[Upraise].Machine_angle_l.exchange_pick_angle_2=50;
		Machine_angle[Clamp_sild].Machine_angle_l.exchange_pick_angle_2=0;
		Machine_angle[Clamp_pitch].Machine_angle_l.exchange_pick_angle_2=0;
		Machine_angle[Clamp_roll].Machine_angle_l.exchange_pick_angle_2=0;
		Machine_angle[Clamp_yaw].Machine_angle_l.exchange_pick_angle_2=0;
		Machine_angle[Joint_1].Machine_angle_l.exchange_pick_angle_2=0;
		Machine_angle[Joint_2].Machine_angle_l.exchange_pick_angle_2=0;
		Machine_angle[Actuator_yaw].Machine_angle_l.exchange_pick_angle_2=0;
		Machine_angle[Actuator_roll].Machine_angle_l.exchange_pick_angle_2=90;
		Machine_angle[Actuator_pitch].Machine_angle_l.exchange_pick_angle_2=90;
		
	    Machine_angle[P_T_pitch].Machine_angle_l.exchange_pick_angle_2=0;
		Machine_angle[P_T_yaw].Machine_angle_l.exchange_pick_angle_2=0;			
		
	}
	{
		Machine_angle[Upraise_clamp].Machine_angle_l.bigisland_straight_angle=22;
		Machine_angle[Upraise].Machine_angle_l.bigisland_straight_angle=280;		
		Machine_angle[Clamp_sild].Machine_angle_l.bigisland_straight_angle=0;    
		Machine_angle[Clamp_pitch].Machine_angle_l.bigisland_straight_angle=0;
		Machine_angle[Clamp_roll].Machine_angle_l.bigisland_straight_angle=0;
		Machine_angle[Clamp_yaw].Machine_angle_l.bigisland_straight_angle=0;
		Machine_angle[Joint_1].Machine_angle_l.bigisland_straight_angle=4.34961;
		Machine_angle[Joint_2].Machine_angle_l.bigisland_straight_angle=13.64908;
		Machine_angle[Actuator_yaw].Machine_angle_l.bigisland_straight_angle=-237/3;
		Machine_angle[Actuator_roll].Machine_angle_l.bigisland_straight_angle=0;
		Machine_angle[Actuator_pitch].Machine_angle_l.bigisland_straight_angle=-10;
     
 		Machine_angle[P_T_pitch].Machine_angle_l.bigisland_straight_angle=0;
		Machine_angle[P_T_yaw].Machine_angle_l.bigisland_straight_angle=0;				
		
	}
	{
		Machine_angle[Upraise_clamp].Machine_angle_l.bigisland_slanted_angle=22;
		Machine_angle[Upraise].Machine_angle_l.bigisland_slanted_angle=330;		
		Machine_angle[Clamp_sild].Machine_angle_l.bigisland_slanted_angle=0;
		Machine_angle[Clamp_pitch].Machine_angle_l.bigisland_slanted_angle=0;
		Machine_angle[Clamp_roll].Machine_angle_l.bigisland_slanted_angle=0;
		Machine_angle[Clamp_yaw].Machine_angle_l.bigisland_slanted_angle=0;
		Machine_angle[Joint_1].Machine_angle_l.bigisland_slanted_angle=3.5682;
		Machine_angle[Joint_2].Machine_angle_l.bigisland_slanted_angle=16.0990;
		Machine_angle[Actuator_yaw].Machine_angle_l.bigisland_slanted_angle=-239/3;
		Machine_angle[Actuator_roll].Machine_angle_l.bigisland_slanted_angle=0;
		Machine_angle[Actuator_pitch].Machine_angle_l.bigisland_slanted_angle=-10;
		
 		Machine_angle[P_T_pitch].Machine_angle_l.bigisland_slanted_angle=0;
		Machine_angle[P_T_yaw].Machine_angle_l.bigisland_slanted_angle=0;				
		
		
	}
	{
		Machine_angle[Upraise_clamp].Machine_angle_l.smallisland_angle=10;
		Machine_angle[Upraise].Machine_angle_l.smallisland_angle=1200;
		Machine_angle[Clamp_sild].Machine_angle_l.smallisland_angle=20;
		Machine_angle[Clamp_pitch].Machine_angle_l.smallisland_angle=-95;
		Machine_angle[Clamp_roll].Machine_angle_l.smallisland_angle=0;
		Machine_angle[Clamp_yaw].Machine_angle_l.smallisland_angle=0;
		Machine_angle[Joint_1].Machine_angle_l.smallisland_angle=0.663*6.33;
		Machine_angle[Joint_2].Machine_angle_l.smallisland_angle=0.477*6.33;
		Machine_angle[Actuator_yaw].Machine_angle_l.smallisland_angle=0;
		Machine_angle[Actuator_roll].Machine_angle_l.smallisland_angle=0;
		Machine_angle[Actuator_pitch].Machine_angle_l.smallisland_angle=0;
		
	    Machine_angle[P_T_pitch].Machine_angle_l.smallisland_angle=61;
		Machine_angle[P_T_yaw].Machine_angle_l.smallisland_angle=-133;				
	}
	{
		Machine_angle[Upraise_clamp].Machine_angle_l.smallisland_angle_2=200;
		Machine_angle[Upraise].Machine_angle_l.smallisland_angle_2=10;
		Machine_angle[Clamp_sild].Machine_angle_l.smallisland_angle_2=0;
		Machine_angle[Clamp_pitch].Machine_angle_l.smallisland_angle_2=0;
		Machine_angle[Clamp_roll].Machine_angle_l.smallisland_angle_2=0;
		Machine_angle[Clamp_yaw].Machine_angle_l.smallisland_angle_2=0;
		Machine_angle[Joint_1].Machine_angle_l.smallisland_angle_2=0;
		Machine_angle[Joint_2].Machine_angle_l.smallisland_angle_2=0;
		Machine_angle[Actuator_yaw].Machine_angle_l.smallisland_angle_2=0;
		Machine_angle[Actuator_roll].Machine_angle_l.smallisland_angle_2=90;
		Machine_angle[Actuator_pitch].Machine_angle_l.smallisland_angle_2=90;
		
        Machine_angle[P_T_pitch].Machine_angle_l.smallisland_angle_2=0;
		Machine_angle[P_T_yaw].Machine_angle_l.smallisland_angle_2=0;			
		
	}
	{
		Machine_angle[Upraise_clamp].Machine_angle_l.ground_angle=22;     //200
		Machine_angle[Upraise].Machine_angle_l.ground_angle=700;             //9
		Machine_angle[Clamp_sild].Machine_angle_l.ground_angle=4;
		Machine_angle[Clamp_pitch].Machine_angle_l.ground_angle=0;
		Machine_angle[Clamp_roll].Machine_angle_l.ground_angle=0;
		Machine_angle[Clamp_yaw].Machine_angle_l.ground_angle=0;
		Machine_angle[Joint_1].Machine_angle_l.ground_angle=-8.4221;
		Machine_angle[Joint_2].Machine_angle_l.ground_angle=-0.545;
		Machine_angle[Actuator_yaw].Machine_angle_l.ground_angle=-12;
		Machine_angle[Actuator_roll].Machine_angle_l.ground_angle=0;        //90
		Machine_angle[Actuator_pitch].Machine_angle_l.ground_angle=-80;       //90
		
        Machine_angle[P_T_pitch].Machine_angle_l.ground_angle=0;
		Machine_angle[P_T_yaw].Machine_angle_l.ground_angle=0;			
	}
	{
		Machine_angle[Upraise_clamp].Machine_angle_l.check_angle=200;
		Machine_angle[Upraise].Machine_angle_l.check_angle=8;		
		Machine_angle[Clamp_sild].Machine_angle_l.check_angle=0;
		Machine_angle[Clamp_pitch].Machine_angle_l.check_angle=0;
		Machine_angle[Clamp_roll].Machine_angle_l.check_angle=0;
		Machine_angle[Clamp_yaw].Machine_angle_l.check_angle=0;
		Machine_angle[Joint_1].Machine_angle_l.check_angle=0;
		Machine_angle[Joint_2].Machine_angle_l.check_angle=0;
		Machine_angle[Actuator_yaw].Machine_angle_l.check_angle=0;
		Machine_angle[Actuator_roll].Machine_angle_l.check_angle=90;
		Machine_angle[Actuator_pitch].Machine_angle_l.check_angle=90;
		
    Machine_angle[P_T_pitch].Machine_angle_l.check_angle=0;
		Machine_angle[P_T_yaw].Machine_angle_l.check_angle=0;			
	}
	
}

//�����Ƕ�ӳ�䵽mode_angle��
void Motor_angle_init_test()
{
	//one����˵������һ��Ϊ�����Ƕȣ��ڶ�����Ŀ�������������Ǳ���
	//couple����˵����ǰ����Ϊ�����Ƕȣ����������븳ֵ�ĵ�������ĵ���ֱ���ǰ���������ĸ�ֵ���������Ǳ���
	config_full_mapping_one(Upraise,UPRAISE,1);
	config_full_mapping_one(Upraise_clamp,UPRAISE_CLAMP,-35);
	config_full_mapping_couple(Clamp_pitch,Clamp_roll,clamp_pitch_and_roll_l,2.3,-2.3,1);
	config_full_mapping_couple(Clamp_pitch,Clamp_roll,clamp_pitch_and_roll_r,-2.3,-2.3,1);	
	config_full_mapping_one(Clamp_yaw,CLAMP_YAW,1);
	config_full_mapping_one(Clamp_sild,CLAMP_SILD,45);
	config_full_mapping_one(Joint_1,joint_1,1);
	config_full_mapping_one(Joint_2,joint_2,1);
	config_full_mapping_one(Actuator_yaw,joint_3_yaw,3);
	config_full_mapping_couple(Actuator_pitch,Actuator_roll,exchang_pitch_and_roll_l,2,-2,1);
	config_full_mapping_couple(Actuator_pitch,Actuator_roll,exchang_pitch_and_roll_r,-2,-2,1);
	
	
	config_full_mapping_one(P_T_pitch,P_T_Pitch,1);
	config_full_mapping_one(P_T_yaw,P_T_Yaw,1);
}







void Motor_task(void *parm)
{
	uint32_t Signal;
	BaseType_t STAUS;
	uint8_t ID;
	
	go_8010_init_time=osKernelSysTick();
	while(1)
	{
		STAUS = xTaskNotifyWait((uint32_t) NULL, 
								(uint32_t) INFO_GET_MOTOR_SIGNAL, 
								(uint32_t *)&Signal, 
								(TickType_t) portMAX_DELAY );
		if(STAUS ==pdTRUE)
		{

			
			if(Motor_init_state==INIT_NEVER)//δ���ò�������в�������
			{
				Motor_base_init();
				Air_Pump_Init();
				Servo_angle_init();
				Motor_init_state=INIT_DONE;
			}
			for(ID=0;ID<MOTOR_NUMBER;ID++)//�������
			{
				
				if(Whether_Brushless_Motor(Motor[ID])==0)
				{
					//TIM_SetCompare4(TIM4,1000);
					Motor_Servo_handler(ID);
				}
				
			}
			
			debug_angle_calucate(&motor_debug[0]);
			debug_angle_calucate(&motor_debug[1]);
			debug_angle_calucate(&motor_debug[2]);
			if((Signal & INFO_GET_MOTOR_SIGNAL) && chassis_mode != CHASSIS_RELEASE)
			{
				/******************��ʽ��ʼ*****************************/
				if(All_Offset_Angle_init_state==0)
				{

			    
					All_Offset_Angle_init_state=Motor_offset_angle_init();
					
					//��8010-task���ͳ�ʼ��֪ͨ
					xTaskGenericNotify((TaskHandle_t) comm_8010_task_Handle, 
													 (uint32_t) GO_8010_INIT_SIGNAL, 
													 (eNotifyAction) eSetBits, 
													 (uint32_t *)NULL );
				 
				}//�ж�
			
				else//��ת����
				{
					clamp_angle_handle();
					for(ID=7;ID<MOTOR_NUMBER;ID++)//��ʽ����
					{
					if((Motor[ID].MOTOR_NAME>CHASSIS_BR) && (Whether_Brushless_Motor(Motor[ID])))//����û�ģ�����������м���
						{
			
							/******************�ٶȽǶȻ��л�����*****************************/
							if(Motor[ID].MOTOR_TYPE!=go_8010)
						{
							Motor[ID].Brushless.Speed_or_Angle_flag=ANGLE_MODE;
							if(Motor[ID].Brushless.Angle_to_Speed_mode && All_Offset_Angle_init_state)
							{
								Motor[ID].Brushless.Speed_or_Angle_flag=SPEED_MODE;
								if(Motor[ID].Brushless.angle_ref-Motor[ID].Brushless.angle_fdb > 100)
								{
									Motor[ID].Brushless.spd_ref=  Motor[ID].Brushless.speed;
								}
								else if(Motor[ID].Brushless.angle_ref-Motor[ID].Brushless.angle_fdb < 100)
								{
									Motor[ID].Brushless.spd_ref= -Motor[ID].Brushless.speed;
								}
								else Motor[ID].Brushless.Speed_or_Angle_flag=ANGLE_MODE; 
							}
						}
						else
						{
							//Motor[ID].Brushless.Speed_or_Angle_flag=SPEED_MODE;
						}
						
							Motor_pid_clac(ID);
							Motor_current_into_CAN(ID);
						xTaskGenericNotify((TaskHandle_t) comm_8010_task_Handle, 
							 (uint32_t) INFO_SEND_MOTOR_SIGNAL, 
							 (eNotifyAction) eSetBits, 
							 (uint32_t *)NULL );


						
						}//����ȥ�����ŷ���ˢ���ȥ������
					}//ѭ������
				}//�ж�����
				
				
				/******************�������*****************************/
			}
				xTaskGenericNotify((TaskHandle_t) can_msg_send_Task_Handle, 
													 (uint32_t) INFO_SEND_MOTOR_SIGNAL, 
													 (eNotifyAction) eSetBits, 
													 (uint32_t *)NULL );


				
				Motor_stack_surplus = uxTaskGetStackHighWaterMark(NULL);    					
		}
	}
}


void Motor_base_init_copy(uint8_t low,uint8_t hight)
{
	uint8_t ID;
	uint8_t count=0;
	for(ID=low+1;ID<=hight;ID++)
	{
		memcpy(&Motor[ID],&Motor[low],sizeof(Motor[low]));
		if(Whether_Brushless_Motor(Motor[ID]))
		{
			count++;
			Motor[ID].MOTOR_NAME=(motor_name_status)ID;//����
			Motor[ID].Brushless.ESC_ID=Motor[low].Brushless.ESC_ID+count;//�ĵ��ID
		}
	}//����
	
}

//���ٶ�ȡ�������ұ�������ģʽ�Ƕ�
void Motor_base_init_reversal(uint8_t ID)
{
	uint32_t first_mode_angle_adress;
	int16_t reversal_angle;
	uint8_t i;
	first_mode_angle_adress=(uint32_t)&Motor[ID].Angle.normal_angle;
	
	Motor[ID].Brushless.offset_angle_init_speed=-Motor[ID].Brushless.offset_angle_init_speed;
	for(i=0;i<MODE_ANGLE_NUMBER;i++)
	{
		reversal_angle=*(int16_t*)first_mode_angle_adress;
		*(int16_t*)first_mode_angle_adress=-reversal_angle;
		first_mode_angle_adress++;
		first_mode_angle_adress++;
	}
	
}

void Motor_PID_Struct_Init(motor_t *Motor_recieve,pid_motor_parameter_t parameter_Struct,INIT_STATUS init_status)
{
	PID_Struct_Init(&Motor_recieve->Brushless.angle_pid,parameter_Struct.angle.p,parameter_Struct.angle.i,parameter_Struct.angle.d,
		parameter_Struct.angle.max_out,parameter_Struct.angle.integral_limit,init_status);
	PID_Struct_Init(&Motor_recieve->Brushless.speed_pid,parameter_Struct.speed.p,parameter_Struct.speed.i,parameter_Struct.speed.d,
		parameter_Struct.speed.max_out,parameter_Struct.speed.integral_limit,init_status);
}

uint8_t test_ID=0;
uint8_t Motor_offset_angle_init(void)
{
	uint8_t ID;
	uint8_t all_init_state;
	all_init_state=1;
	
	for(ID=7;ID<MOTOR_NUMBER;ID++)//��ת��ʼ��
	{
		if(Motor[ID].MOTOR_NAME>CHASSIS_BR && Whether_Brushless_Motor(Motor[ID]))
		{
			if(Motor[ID].Brushless.offset_angle_init_flag==0)//����û�ģ�����������м���
			{
				if(Motor[ID].MOTOR_TYPE==M3508||Motor[ID].MOTOR_TYPE==M2006)
				{
					Motor[ID].Brushless.spd_ref=Motor[ID].Brushless.offset_angle_init_speed;
				}
				else if(Motor[ID].MOTOR_TYPE==M6020) 
				{
					if((Motor[ID].Brushless.ecd_fdb - 1679) > 0)
						Motor[ID].Brushless.spd_ref=-Motor[ID].Brushless.offset_angle_init_speed;
					else if((Motor[ID].Brushless.ecd_fdb - 1679) < 0)
						Motor[ID].Brushless.spd_ref=Motor[ID].Brushless.offset_angle_init_speed;
					else Motor[ID].Angle.offset_angle=Motor[ID].Brushless.angle_fdb;
				}
			
	

			
			//GO_8010���������ʼ��			
			if(Motor[ID].MOTOR_TYPE!=go_8010)
			{
				if((fabs(Motor[ID].Brushless.spd_ref) - fabs(Motor[ID].Brushless.spd_fdb)) > 0.6*fabs(Motor[ID].Brushless.spd_ref))//�ж��Ƿ��ת��ռ��Խ������Խ���̣�
				{
					Motor[ID].Brushless.err_count++;
					if(Motor[ID].Brushless.err_count>200)
					{//�ﵽ��ת
						Motor[ID].Brushless.offset_angle_init_flag = 1;
						Motor[ID].Angle.offset_angle=Motor[ID].Brushless.angle_fdb;
						Motor[ID].Brushless.spd_ref=0;
						Motor[ID].Brushless.err_count=0;
					}
				}
			}
			
			
			
//				else
//				{
//					Motor[ID].Brushless.spd_ref=0;
//					Motor[ID].Brushless.err_count=0;
//				}
			}//δ��ת�����
			//motor_8010_speed_get_limit(ID);
			Motor_pid_clac(ID);
			Motor_current_into_CAN(ID);
			if(Motor[ID].Brushless.offset_angle_init_flag==0) all_init_state=0;//��������ʼ��δ���
		}//ѡ�п��Ƶ��
	}//ѭ��

	return all_init_state;
}
uint32_t TIM_out;
void Motor_Servo_handler(uint8_t ID)
{
	
	TIM_out=((2000/Motor[ID].Servo.Rotation_range)*Motor[ID].Servo.angle_ref)+500;
	switch(Motor[ID].Servo.Compare)
	{
		case 1:
		{
			TIM_SetCompare1(Motor[ID].Servo.TIM,TIM_out);
		}break;
		case 2:
		{
			TIM_SetCompare2(Motor[ID].Servo.TIM,TIM_out);
		}break;
		case 3:
		{
			TIM_SetCompare3(Motor[ID].Servo.TIM,TIM_out);
		}break;
		case 4:
		{
			TIM_SetCompare4(Motor[ID].Servo.TIM,TIM_out);
		}break;
	}
}

void Motor_pid_clac(uint8_t ID)
{
	if(Motor[ID].Brushless.Speed_or_Angle_flag==ANGLE_MODE)
	{
		
		pid_calc(&Motor[ID].Brushless.angle_pid,Motor[ID].Brushless.angle_fdb,Motor[ID].Brushless.angle_ref);
		Motor[ID].Brushless.spd_ref=Motor[ID].Brushless.angle_pid.out;
		pid_calc(&Motor[ID].Brushless.speed_pid,Motor[ID].Brushless.spd_fdb,Motor[ID].Brushless.spd_ref);
		Motor[ID].Brushless.current_send=Motor[ID].Brushless.speed_pid.out;
	}
	else if(Motor[ID].Brushless.Speed_or_Angle_flag==SPEED_MODE)
	{
		Motor[ID].Brushless.current_send=pid_calc(&Motor[ID].Brushless.speed_pid,Motor[ID].Brushless.spd_fdb,Motor[ID].Brushless.spd_ref);
	}
	
	if(Motor[ID].MOTOR_TYPE==go_8010)
	{
		
		pid_calc(&Motor[ID].Brushless.angle_pid,Motor[ID].Brushless.angle_fdb,Motor[ID].Brushless.angle_ref);
		Motor[ID].Brushless.current_send=Motor[ID].Brushless.angle_pid.out;
		
//	  test_ref[ID] = pid_calc(&Motor[ID].Brushless.angle_pid,Motor[ID].Brushless.angle_fdb,Motor[ID].Brushless.angle_ref);
//		pid_calc(&Motor[ID].Brushless.speed_pid,Motor[ID].Brushless.spd_fdb,test_ref[ID]);
//		Motor[ID].Brushless.current_send=Motor[ID].Brushless.angle_pid.out;
//		test_angle=0.1*Motor[ID].Brushless.speed_pid.out;
	}
 
 
//	if(Motor[ID].MOTOR_TURN_FLAG)
//	{
//		Motor[ID].Brushless.current_send=-Motor[ID].Brushless.current_send;
//	}
}

void Motor_current_into_CAN(uint8_t ID)
{
	switch(Motor[ID].Brushless.CAN_ID)
	{
		case 1:
		{
			memcpy(&CAN1_current[Motor[ID].Brushless.ESC_ID],&Motor[ID].Brushless.current_send,sizeof(Motor[ID].Brushless.current_send));
		}break;
		case 2:
		{
			memcpy(&CAN2_current[Motor[ID].Brushless.ESC_ID],&Motor[ID].Brushless.current_send,sizeof(Motor[ID].Brushless.current_send));
		}break;
	}
//	
		if(Motor[ID].MOTOR_TYPE==go_8010)
		{
			switch(Motor[ID].Brushless.GO_ID)
		{
			case GO_8010_1:
			{
			memcpy(&send_current_8010_1,&Motor[ID].Brushless.current_send,sizeof(Motor[ID].Brushless.current_send));			
			memcpy(&send_ref_angle_8010_1,&Motor[ID].Brushless.angle_ref,sizeof(Motor[ID].Brushless.angle_ref));
			memcpy(&send_fdb_angle_8010_1,&Motor[ID].Brushless.angle_fdb,sizeof(Motor[ID].Brushless.angle_fdb));	
				
			}break;
			case GO_8010_2:
			{
				memcpy(&send_current_8010_2,&Motor[ID].Brushless.current_send,sizeof(Motor[ID].Brushless.current_send));
				memcpy(&send_ref_angle_8010_2,&Motor[ID].Brushless.angle_ref,sizeof(Motor[ID].Brushless.angle_ref));
			}break;
		}
		
	}
	
	
}

uint8_t Whether_Brushless_Motor(motor_t Motor)
{
	switch(Motor.MOTOR_TYPE)
	{
		case M6020:
		case M3508:
		case M2006:
		case go_8010:
			return 1;
	default:
	return 0;
	}
}


//��ȡ�Ƕ��ֶ�ָ��
float* get_angle_field(float* mode_angle,int16_t count)
{
	return (float*)((uint32_t)mode_angle+(sizeof(Motor_angle_t)/MODE_ANGLE_NUMBER)*count);//����count��ȡ��Ӧ��ģʽ�Ƕ�
}


//˫�Ƕ�ӳ��
void config_full_mapping_couple(int16_t Machine_ID_l,int16_t Machine_ID,int16_t Motor_ID,float Machine_l_ratio,float Machine_ratio,int16_t total_ratio)
{
	float*Machine_l_scr=&Machine_angle[Machine_ID_l].Machine_angle_l.normal_angle;
	float*Machine_scr=&Machine_angle[Machine_ID].Machine_angle_l.normal_angle;
	float*Motor_scr=&Motor[Motor_ID].Angle.normal_angle;
	//�����2��ԭ������intit_angle�ȵ������ֵ������ֵ
	for(int i=0;i<MODE_ANGLE_NUMBER-2;i++)
	{
		 float*Machine_l_dest=get_angle_field(Machine_l_scr,i);
		 float*Machine_dest=get_angle_field(Machine_scr,i);
		 float*Motor_dest=get_angle_field(Motor_scr,i);

		*Motor_dest=(*Machine_dest)*Machine_ratio+(*Machine_l_dest)*Machine_l_ratio;
	}
}

//���Ƕ�ӳ��
void config_full_mapping_one(int16_t Machine_ID,int16_t Motor_ID,int16_t total_ratio)
{
	float*Machine_scr=&Machine_angle[Machine_ID].Machine_angle_l.normal_angle;
	float*Motor_scr=&Motor[Motor_ID].Angle.normal_angle;
	
		for(int i=0;i<MODE_ANGLE_NUMBER-2;i++)
	{
	
		float* Machine_dest=get_angle_field(Machine_scr,i);
		float* Motor_dest=get_angle_field(Motor_scr,i);
		
		*Motor_dest=*Machine_dest*total_ratio;
	}
}

void Air_Pump_Init (void)
{	
   PUMP_OFF	
}
uint8_t Servo_angle_init(void)
{
		Motor[P_T_Pitch].Servo.angle_ref=220;//135��׼   //NORMAL
		Motor[P_T_Yaw].Servo.angle_ref=168;	
}


void debug_angle_calucate(motor_angle_debug* angle_debug)
{
	angle_debug->mode_angle=angle_debug->targrt_angle-angle_debug->offset_angle-angle_debug->normal_angle;
}


