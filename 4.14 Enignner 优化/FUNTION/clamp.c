/*****************************************************
	  此文件用作motor_task的从属文件，用于存放夹取用的
堵转初始化与动作等函数以避免motor_task文件太长而翻找
麻烦
*****************************************************/
#include "STM32_TIM_BASE.h"

#include "clamp.h"
#include "motor_task.h"
#include "modeswitch_task.h"
#include "remote_ctrl.h"
#include "controller.h"
#include "keyboard.h"
int16_t test1=-2500;
int16_t test2=20;
int16_t test3=0;

int16_t smalltest1=-250;
int16_t smalltest2=180;
int16_t smalltest3=0;

Motor_angle_t Mode;

uint32_t clamp_record_time;
uint8_t action_one_state=ACTIONING;
uint8_t action_two_state=ACTIONING;
uint8_t action_three_state=ACTIONING;
uint8_t have_box_number=0;

uint8_t current_limit_intit_flag=0;
Motor_current_limit_t Motor_current_limit[MOTOR_NUMBER]={0};

void clamp_angle_handle()
{
	uint8_t ID;
	
//	if(current_limit_intit_flag==0)
//		current_limit_intit_flag=Motor_current_limit_init();
//	Motor_current_limit_reset();
	
	switch(chassis_mode)
	{
		case CHASSIS_NORMAL_MODE:
		{
			normal_clmap_handler();
			clamp_action=CLAMP_UN_CMD;
		}break;
		case CHASSIS_CLAMP_BIG_ISLAND_STRAIGHT_MODE:
		{
			big_island_stright_clamp_handler();
		}break;
		case CHASSIS_CLAMP_BIG_ISLAND_SLANTED_MODE:
		{
			big_island_slanted_clamp_handler();
		}break;
		case CHASSIS_CLAMP_SMALL_ISLAND_MODE:
		{
			small_island_clamp_handler();
		}break;
		case CHASSIS_GROUND_MODE:
		{
			clamp_ground_handler();
		}break;
		case CHASSIS_EXCHANGE_MODE:
		case CHASSIS_ANGLE_MODE:
		{
			exchange_handler();
		}break;
		case CHASSIS_CHECK_MODE:
		{
			check_handler();
		}
		default:
		{
			clamp_action=CLAMP_UN_CMD;
			PUMP_OFF;
		}
		break;
	}
	
	//Motor_current_limit_handler();
	
	for(ID=0;ID<MOTOR_NUMBER;ID++)//
	{
	if(Motor[ID].MOTOR_NAME>CHASSIS_BR && Whether_Brushless_Motor(Motor[ID]))//底盘没改，不在这里进行计算
		{
			if(Motor[ID].MOTOR_TYPE!=go_8010)
				Motor[ID].Brushless.angle_ref= Motor[ID].Angle.offset_angle + Motor[ID].Angle.normal_angle + Motor[ID].Angle.mode_angle;
			else
				Motor[ID].Brushless.angle_ref= Motor[ID].Angle.offset_angle + Motor[ID].Angle.normal_angle + Motor[ID].Angle.mode_angle;
		}
	else if(Whether_Brushless_Motor(Motor[ID])==0)
		{
			Motor[ID].Servo.angle_ref=Motor[ID].Angle.normal_angle + Motor[ID].Angle.mode_angle;
		}
	}

	Motor[joint_1].Brushless.angle_ref+=(model_out_angle[1]-kb_adrust[JOINT_1].angle);
	Motor[joint_2].Brushless.angle_ref+=(model_out_angle[2]+kb_adrust[JOINT_2].angle);
	Motor[exchang_pitch_and_roll_l].Brushless.angle_ref+=2*(-(kb_adrust[ADRUST_ROLL].angle+model_out_angle[5])+(kb_adrust[ADRUST_PITCH].angle+model_out_angle[4]));
	Motor[exchang_pitch_and_roll_r].Brushless.angle_ref+=2*(-(kb_adrust[ADRUST_ROLL].angle+model_out_angle[5])-(kb_adrust[ADRUST_PITCH].angle+model_out_angle[4]));
	
	Motor[P_T_Pitch].Servo.angle_ref+=kb_adrust[P_T_PITCH].angle;
	Motor[P_T_Yaw].Servo.angle_ref+=kb_adrust[P_T_YAW].angle;
	
	
	//Motor[joint_3_yaw].Brushless.angle_ref+=kb_adrust[ADRUST_YAW].angle;
	//Motor[UPRAISE].Brushless.angle_ref+=kb_adrust[ADRUST_Z].angle;

	//Motor[joint_3_yaw].Brushless.angle_ref+=kb_slide_clamp_del.angle;
	/******************限制伸出*******************************/
//	if(Motor[SLIDE_L].Brushless.angle_ref<Motor[SLIDE_L].Angle.offset_angle-SLIDE_MAX_CHANGE)
//		Motor[SLIDE_L].Brushless.angle_ref=Motor[SLIDE_L].Angle.offset_angle-SLIDE_MAX_CHANGE;
//	if(Motor[SLIDE_R].Brushless.angle_ref>Motor[SLIDE_R].Angle.offset_angle+SLIDE_MAX_CHANGE)
//		Motor[SLIDE_R].Brushless.angle_ref=Motor[SLIDE_R].Angle.offset_angle+SLIDE_MAX_CHANGE;
}

void normal_clmap_handler(void)
{
	uint8_t ID;
	
	for(ID=CLAMP_YAW;ID<=MOTOR_MAX_ID;ID++)
		Motor[ID].Angle.mode_angle=0;
	
	PUMP_ON
	Exchange_ON
	CLAMP_ON
	
	if(have_box_number==0)
		PUMP_OFF
	else 
		PUMP_ON
}

int16_t test_final=0;
void big_island_stright_clamp_handler(void)
{
	switch(clamp_action)
	{
		case CLAMP_UN_CMD:
		{
			Motor_change_mode_angle(&Mode.bigisland_straight_angle,clamp_pitch_and_roll_l,MOTOR_MAX_ID);//只将夹取相关电机置对应模式角度
			clamp_record_time= HAL_GetTick();
			action_one_state=ACTIONING;
			action_two_state=ACTIONING;
			PUMP_ON
			CLAMP_ON
		}break;
		case ACTION_ONE:
		{
			if(action_one_state==ACTIONING)
			{
					if((HAL_GetTick()-clamp_record_time)>200&&(HAL_GetTick()-clamp_record_time)<250)
				{
					Motor_uprise_clamp_angle_change(3);
					Motor[UPRAISE].Angle.mode_angle=-100;
				
				}
				else if((HAL_GetTick()-clamp_record_time)>1500)
				{
					if(have_box_number<MAX_BOX_NUMBER)
						have_box_number++;
					action_one_state=ACTION_DONE;
				}
			}
			else
//				Motor_current_limit[SLIDE_CLAMP].flag=1;//使能限流
				clamp_record_time= HAL_GetTick();
		}break;
		case ACTION_TWO:
		{
			if(action_two_state==ACTIONING)
				action_two_state=store_handler();
			else
				clamp_action=CLAMP_UN_CMD;
		}break;
	}
}

uint8_t store_big_island_slanted_second_box=0;
void big_island_slanted_clamp_handler(void)
{
  switch(clamp_action)
  {
    case CLAMP_UN_CMD:
    {
      Motor_change_mode_angle(&Mode.bigisland_slanted_angle,UPRAISE,MOTOR_MAX_ID);
      clamp_record_time= HAL_GetTick();
      action_one_state=ACTIONING;
      action_two_state=ACTIONING;
			PUMP_ON
			CLAMP_ON
    }break;
    case ACTION_ONE:
    {
			if(action_one_state==ACTIONING)
			{
				if((HAL_GetTick()-clamp_record_time)==200)
				{
					//Motor_uprise_angle_change(7000/542*48*2);//1500
			//		Motor[CLAMP_PITCH].Angle.mode_angle-=20;
				}
				else if((HAL_GetTick()-clamp_record_time)>2000)
				{
//					Motor[SLIDE_CLAMP].Angle.mode_angle+=500;
					if(have_box_number<MAX_BOX_NUMBER)
						have_box_number++;
					action_one_state=ACTION_DONE;
				}
			}
			else
			{	
				Motor_current_limit[joint_3_yaw].flag=1;//使能限流
        clamp_record_time= HAL_GetTick();
    
			}
			}break;
   case ACTION_TWO:
		{
			if(action_two_state==ACTIONING)
				action_two_state=store_handler();
			else
				clamp_action=CLAMP_UN_CMD;
		}break;
  }
}
uint32_t time_test=0;
void small_island_clamp_handler(void)
{
	
  switch(clamp_action)
  {
    case CLAMP_UN_CMD:
    {
      Motor_change_mode_angle(&Mode.smallisland_angle,clamp_pitch_and_roll_l,MOTOR_MAX_ID);
      clamp_record_time= HAL_GetTick();
      action_one_state=ACTIONING;
      action_two_state=ACTIONING;
	  action_three_state=ACTIONING;
			PUMP_ON
			CLAMP_ON
		//Air_Pump_Clamp_ON
		    Exchange_ON;
			//ALL_ON
    }break;
    case ACTION_ONE:
    {
      if(action_one_state==ACTIONING)
      {
				if((HAL_GetTick()-clamp_record_time)>200&&(HAL_GetTick()-clamp_record_time<250))
				{
					Motor_uprise_clamp_angle_change(-9.5);
				}
				else if((HAL_GetTick()-clamp_record_time)>1000&&(HAL_GetTick()-clamp_record_time<1100))
				{
					Motor_uprise_clamp_angle_change(14);
					
					
				}
				else if((HAL_GetTick()-clamp_record_time)>1300)
				{
					action_one_state=ACTION_DONE;
				}
						
				
      }
      else
        clamp_record_time= HAL_GetTick();
    }break;
    case ACTION_TWO:
    {
      if(action_two_state==ACTIONING)
	  {
		  if((HAL_GetTick()-clamp_record_time)>200&&(HAL_GetTick()-clamp_record_time<250))
			{
				Motor[clamp_pitch_and_roll_l].Angle.mode_angle=0;
			    Motor[clamp_pitch_and_roll_r].Angle.mode_angle=0;

				
				Motor[CLAMP_SILD].Angle.mode_angle=2*45;
			}

		else if((HAL_GetTick()-clamp_record_time)>1500)
			{
				
					action_two_state=ACTION_DONE;
			}
		
	  }
	else
	clamp_record_time= HAL_GetTick();
    }break;
	case ACTION_THREE:
    {
      if(action_three_state==ACTIONING)
        action_three_state=store_handler();
			else
		clamp_action=CLAMP_UN_CMD;
    }break;
  }
}

int16_t test_198=0;
void clamp_ground_handler(void)
{
  switch(clamp_action)
  {
    case CLAMP_UN_CMD:
    {
      Motor_change_mode_angle(&Mode.ground_angle,UPRAISE,MOTOR_MAX_ID);
      clamp_record_time= HAL_GetTick();
      action_one_state=ACTIONING;
      action_two_state=ACTIONING;
			PUMP_ON
			CLAMP_ON
    }break;
    case ACTION_ONE:
    {
      if(action_one_state==ACTIONING)
      {
				if((HAL_GetTick()-clamp_record_time)==200)
				{
					//Motor_uprise_angle_change(-1500);
					test_198++;
				}
				if((HAL_GetTick()-clamp_record_time)==2000)
				{
					//Motor_uprise_angle_change(1500);
					test_198++;
				}
				else if((HAL_GetTick()-clamp_record_time)>=3500)
				{
					action_one_state=ACTION_DONE;
				}
      }
      else
        clamp_record_time= HAL_GetTick();
    }break;
    case ACTION_TWO:
    {
      if(action_two_state==ACTIONING)
        action_two_state=store_handler();
			else
				clamp_action=CLAMP_UN_CMD;
    }break;
  }
}

uint8_t ready_to_del_box_number_flag=1;
void exchange_handler(void)
{

  switch(exchange_action)
  {
    case EXCHANGE_UN_CMD:
    {
			PUMP_ON
			//EXCHANGE_ON
      Motor_change_mode_angle(&Mode.exchange_angle,UPRAISE,MOTOR_MAX_ID);
      clamp_record_time= HAL_GetTick();
			action_one_state=ACTIONING;
      action_two_state=ACTIONING;
			ready_to_del_box_number_flag=1;
    }break;
    case PICK_ACTION:
    {
        if(action_two_state==ACTIONING)
					action_two_state=pick_handler();
				else
				{
					exchange_action=EXCHANGE_UN_CMD;
					clamp_record_time= HAL_GetTick();
				}
    }break;
    case RELESE_BOX_ACTION:
    {
		Exchange_OFF
			if(ready_to_del_box_number_flag)
				have_box_number--;
  
		}break;
  }
}

uint8_t store_handler(void)
{
	uint8_t finish_flag=0;
	uint32_t store_action_times;
	store_action_times=HAL_GetTick()-clamp_record_time;
	

			if(chassis_mode==CHASSIS_CLAMP_BIG_ISLAND_SLANTED_MODE||chassis_mode==CHASSIS_CLAMP_BIG_ISLAND_STRAIGHT_MODE)
			{
				if(store_action_times==10)
					Motor_change_mode_angle(&Mode.store_angle,UPRAISE,MOTOR_MAX_ID);
				if(store_action_times==2500)
				{
					
			//		Motor[CLAMP_PITCH].Angle.mode_angle-=5;

				
		
					//Motor_uprise_angle_change(-2400);
				}
				if(store_action_times==3500)
				{
				//	CLAMP_OFF
				}
				if(store_action_times==5500)
				{
					//Motor_uprise_angle_change(3000);
				}
				if(store_action_times>=6500)
				{
					finish_flag=0;
				}
			}			
			else if(chassis_mode==CHASSIS_CLAMP_SMALL_ISLAND_MODE)
		 {
			 if(store_action_times>10&&store_action_times<60)
			 {
				 Motor_change_mode_angle(&Mode.store_angle,clamp_pitch_and_roll_l,MOTOR_MAX_ID);
				 
			 }
			 
		   else if(store_action_times>1000&&store_action_times<1100)
			{
		         CLAMP_OFF
			}				 
		if(store_action_times>=1500)
			{

					finish_flag=1;
			}
		}

	return finish_flag;
}

uint8_t pick_handler(void)
{
	uint8_t finish_flag=0;
	uint32_t pick_action_times;
	pick_action_times=HAL_GetTick()-clamp_record_time;
   Exchange_ON
	
	
	for(int i=0;i<=8;i++)
	{
		kb_adrust[i].angle=0;
	}
	if(pick_action_times>10&&pick_action_times<60)
	{
		 
		 
		
		
				 Motor_change_mode_angle(&Mode.exchange_pick_angle,clamp_pitch_and_roll_l,MOTOR_MAX_ID);
				 
	}
	else if(pick_action_times>1000&&pick_action_times<1100)
	{
		  
			Motor[joint_1].Angle.mode_angle=-1.538*6.33;
			Motor[joint_2].Angle.mode_angle=0.769*6.33;
			Motor[joint_3_yaw].Angle.mode_angle=-110;
		
	}		
			 
	else if(pick_action_times>2000&&pick_action_times<2300)
	{
		      CLAMP_OFF
	}				 
		if(pick_action_times>=2500)
	{
		
					if(have_box_number>0)have_box_number--;
					finish_flag=1;
	}
	return finish_flag;
}
void check_handler(void)
{
	Motor_change_mode_angle(&Mode.check_angle,UPRAISE,MOTOR_MAX_ID);
}

//通过地址操作写指定范围电机mode_angle为当前模式角度
void Motor_change_mode_angle(float* Mode_now,uint8_t low_ID,uint8_t hight_ID)
{
	uint8_t ID;
	uint32_t mode_angle_adress;
	uint32_t now_angle_adress;
	uint32_t target_adress;
	mode_angle_adress=(uint32_t)&Mode.mode_angle;
	now_angle_adress=(uint32_t)Mode_now;
	for(ID=low_ID;ID<=hight_ID;ID++)
	{
			target_adress=((uint32_t)&Motor[ID].Angle.mode_angle + now_angle_adress - mode_angle_adress);
			Motor[ID].Angle.mode_angle=*(float*)target_adress;
	}
}

void Motor_uprise_clamp_angle_change(float change_displacement)
{
	float change_angle;
	change_angle=change_displacement*UPRISE_DISPLACEMENT_TO_ANGLE;
	Motor[UPRAISE_CLAMP].Angle.mode_angle=change_angle;
	//Motor[UPRISE_R].Angle.mode_angle+=change_angle;
}

void Motor_slide_angle_change(int16_t change_displacement)
{
	int16_t change_angle;
	change_angle=change_displacement*SLIDE_DISPLACEMENT_TO_ANGLE;
	
//	Motor[SLIDE_L].Angle.mode_angle+=change_angle;
//	Motor[SLIDE_R].Angle.mode_angle-=change_angle;
}

//static uint8_t Motor_current_limit_init(void)
//{
//	uint8_t ID;
//	//Motor_current_limit[joint_3_yaw].special=5+00;
////	Motor_current_limit[SLIDE_L].special=1000;
////	Motor_current_limit[SLIDE_R].special=1000;
//	for(ID=0;ID<MOTOR_NUMBER;ID++)
//	{
//		if(Whether_Brushless_Motor(Motor[ID]))
//		{
//			Motor_current_limit[ID].normal=Motor[ID].Brushless.speed_pid.maxout;
//		}
//	}
//	
//	return 1;
//}

//static void Motor_current_limit_reset(void)
//{
//	uint8_t ID;
//	for(ID=0;ID<MOTOR_NUMBER;ID++)
//	{
//		if(Whether_Brushless_Motor(Motor[ID]))
//		{
//			Motor_current_limit[ID].flag=0;
//	
//		}
//	}
//}

//static void Motor_current_limit_handler(void)
//{
//	uint8_t ID;
//	for(ID=0;ID<MOTOR_NUMBER;ID++)
//	{
//		if(Whether_Brushless_Motor(Motor[ID]))
//		{
//			if(Motor_current_limit[ID].flag)
//				Motor[ID].Brushless.speed_pid.maxout=Motor_current_limit[ID].special;
//			else
//				Motor[ID].Brushless.speed_pid.maxout=Motor_current_limit[ID].normal;
//		}
//	}
//}













