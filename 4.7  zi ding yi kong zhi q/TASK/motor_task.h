#ifndef _moto
#define _moto

#include "pid.h"
#include "fuzzy_pid.h"
#include "string.h"

#include "User.h"
/**************电机数目定义*************/
#define MOTOR_NUMBER 17
#define MODE_ANGLE_NUMBER 14
#define Machine_NUMBER    11
#define MOTOR_MAX_ID exchang_pitch_and_roll_r
/*************************************/
#define ON 1
#define OFF 0
#define ANGLE_MODE 2
#define SPEED_MODE 1

#ifdef BACK_DRIVE
#define NORMAL_LOOK 255
#define MODE_LOOK -193
#else
#define NORMAL_LOOK 62
#define MODE_LOOK 0
#endif

#define GO_8010_1   2
#define GO_8010_2   3
/**************电机名定义*************/
typedef enum
{
//以下为MOTOR_NAME 逐级递增方便增改
  CHASSIS_FR=0,
  CHASSIS_FL ,
  CHASSIS_BL ,
  CHASSIS_BR ,


  CLAMP_YAW ,
  CLAMP_ROLL ,
  EXCHANGE_ROLL ,
	GIMBAL_YAW,
	
	CLAMP_PITCH,
	CLAMP_SILD,
  UPRAISE,
	UPRAISE_CLAMP,
  joint_3_yaw ,
  joint_1 ,
  joint_2 ,
	exchang_pitch_and_roll_l,
	exchang_pitch_and_roll_r,

	
	
}motor_name_status;

typedef enum
{
  M6020=0 ,
  M3508 ,
  M2006 ,
	go_8010,
  SERVO ,
}motor_type_status;

//typedef enum
//{
//  M_UN_TURN=0  ,
//	M_TURN  ,
//}motor_turn_flag_status;

typedef struct
{
	uint8_t state;//初始化完成？ 删
	uint8_t offset_angle_init_flag;//初始化标志
	int16_t offset_angle_init_speed;//堵转初始化速度
	uint16_t err_count;

	uint8_t CAN_ID;
	uint8_t ESC_ID;//电调ID
	uint8_t GO_ID;
	
	uint8_t Angle_to_Speed_mode;//是否在角度差过大时切换到速度环
	uint8_t Speed_or_Angle_flag;//选择速度或角度环
	int16_t speed;//角度差过大直接给多少速度
	
	int32_t ecd_fdb;
	float torque_fbd;
	float torque_ref;
  float angle_fdb;//角度环get
	float l_angle_fdb;
	float angle_ref;//角度环目标
  float spd_fdb;//速度环get
  float spd_ref;//速度环目标
	float current_read;//电流
	
	int16_t current_send;//电流
	pid_t angle_pid;
	pid_t speed_pid;
}Brushless_motor_t;

typedef struct
{
	TIM_TypeDef* TIM;
	uint8_t Compare;
	int16_t angle_ref;
	
	uint16_t Rotation_range;//旋转范围
}Servo_motor_t;

typedef struct
{
  float normal_angle;
	float exchange_angle;
	float store_angle;
	float exchange_pick_angle;
	float exchange_pick_angle_2;
  float bigisland_straight_angle;
	float bigisland_slanted_angle;	
	float smallisland_angle;
	float smallisland_angle_2;
	float ground_angle;
	float check_angle;
	float mode_angle;
	
	float init_angle;//初始化得到的基准
	float offset_angle;//进模式角度相对变化用基准
}Motor_angle_t;

typedef struct
{
	motor_name_status MOTOR_NAME;//等于其宏定义，debug看名字
	motor_type_status MOTOR_TYPE;//电机类型
//	motor_turn_flag_status MOTOR_TURN_FLAG;
	
  Brushless_motor_t Brushless;
	Servo_motor_t Servo;
	Motor_angle_t Angle;
	
}motor_t;


typedef struct
{
	motor_name_status MOTOR_NAME;
	pid_parameter_t speed;
	pid_parameter_t angle;
}pid_motor_parameter_t;


typedef enum
{
	  Upraise=0,
		Upraise_clamp,
	
    Clamp_pitch,
	  Clamp_roll,
	  Clamp_yaw,
		Clamp_sild,
	
	  Joint_1,
	  Joint_2,
	  Actuator_yaw,
	  Actuator_roll,
	  Actuator_pitch,
}Machine_Angle_N;

typedef struct
{
	Motor_angle_t Machine_angle_l;
}Machine_Angle_T;
extern motor_t Motor[MOTOR_NUMBER];

void Motor_task(void *parm);

void Motor_base_init(void);
void Motor_base_init_copy(uint8_t low,uint8_t hight);
void Motor_base_init_reversal(uint8_t ID);
void Motor_pid_init(INIT_STATUS init_status);
void Motor_angle_init(void);
void Motor_PID_Struct_Init(motor_t *Motor_recieve,pid_motor_parameter_t parameter_Struct,INIT_STATUS init_status);
uint8_t Motor_offset_angle_init(void);
void Motor_Servo_handler(uint8_t ID);
void Motor_pid_clac(uint8_t ID);
void Motor_current_into_CAN(uint8_t ID);
uint8_t Whether_Brushless_Motor(motor_t Motor);
uint8_t motor_8010_speed_get_limit(uint8_t ID);
void config_full_mapping_couple(int16_t Machine_ID_l,int16_t Machine_ID,int16_t Motor_ID,int16_t Machine_l_ratio,int16_t Machine_ratio,int16_t total_ratio);
void config_full_mapping_one(int16_t Machine_ID,int16_t Motor_ID,int16_t total_ratio);
void Motor_angle_init_test(void);
void Machine_angle_init(void);
float* get_angle_field(float* mode_angle,int16_t count);

void Air_Pump_Init (void);




#endif


