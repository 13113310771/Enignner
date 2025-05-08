#include "modeswitch_task.h"
#include "STM32_TIM_BASE.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "remote_ctrl.h"
#include "keyboard.h"
#include "comm_task.h"
#include "chassis_task.h"
#include "controller.h"

#include "pc_tx_data.h"
#include "judge_tx_data.h"

#include "bsp_can.h"
#include "motor_8010.h"
/**********************************************************************************
*******��Դ��������߼���20.12.19��ĳ���޸ģ����н��鼰��������ϵQQ��3155460945��******
***********************************************************************************
//1.���Ȼ�ȡȫ��״̬��Ȼ�󾭲���ѡ������ģʽ
//2.����ģʽ������������ģʽ�Ĺ��ܣ���������ȡ���ϰ�����ˡ���Ԯ���һ����ģʽ
//3.�����жϽ��빤��ģʽ������ģʽ��ͨ���鿴���̵�ģʽ
*********************21.1.31��ʼȫ���޸��˼�ȡģʽ�Ŀ����߼�**************************

***********************************************************************************/

UBaseType_t mode_switch_stack_surplus;

extern TaskHandle_t info_get_Task_Handle;

global_status global_mode; 
global_status last_global_mode = RELEASE_CTRL;

chassis_status chassis_mode;
chassis_status last_chassis_mode = CHASSIS_RELEASE;

rescue_status rescue_mode;
rescue_status last_rescue_mode = RESCUE_INIT;

clamp_status clamp_mode;
clamp_status last_clamp_mode = CLAMP_INIT;

barrier_carry_status barrier_carry_mode;
barrier_carry_status last_barrier_carry_mode = BARRIER_CARRY_INIT;

supply_status supply_mode;
supply_status last_supply_mode;

gimbal_status gimbal_mode;

/*С/����Դ��ģʽ����Ĳ���ģʽ����Ҫ�����ϴε�ģʽ*/
small_island_mode_t small_island_mode;
big_island_mode_t   big_island_mode;

void mode_switch_task(void *parm)
{
	uint32_t mode_switch_wake_time = osKernelSysTick();
	while(1)
	{
		
		//go8010_task();
		get_last_mode();           //��ȡ���������ϴε�ģʽ
		get_main_mode();
		get_chassis_mode();
		remote_ctrl_clamp_hook();
  	keyboard_clamp_hook();
		
//    /*����ң������*/
//    send_rc_data1();
//    send_rc_data2();
//    send_rc_data3();
    
    xTaskGenericNotify( (TaskHandle_t) info_get_Task_Handle, 
                        (uint32_t) MODE_SWITCH_INFO_SIGNAL, 
                        (eNotifyAction) eSetBits, 
                        (uint32_t *)NULL );
    
    mode_switch_stack_surplus = uxTaskGetStackHighWaterMark(NULL);
    
    vTaskDelayUntil(&mode_switch_wake_time, 6);//������ʱ����
  }
}
/*************************************************************
*****1.�����ж�״̬��û�б仯
*****2.״̬���ͱ仯ʱ�ٸ�ֵ�ϴ�״̬
*****3.�ϴ�״̬�͵�ǰ״̬���ֲ�һ��������ǰ״̬�л�ʱ�ٸ����ϴ�״̬

********��˼·ֻ�ڹ��̴�����ģ�����������������¿�ܵ�˼·
***************************************************************/
uint8_t	global_change_state = 1;
	/*�м�仯ģʽ����*/
uint8_t	global_middle_change_mode;
uint8_t chassis_middle_change_mode;
uint8_t rescue_middle_change_mode;
uint8_t clamp_middle_change_mode ;
uint8_t barrier_carry_middle_change_mode;
uint8_t supply_middle_change_mode;

void get_last_mode(void)
{	
	if(global_change_state)//ֻ����һ�Σ���ʼ���ͺ�
	{		
		global_middle_change_mode        = global_mode;
		last_global_mode                 = (global_status)global_middle_change_mode;
		
		chassis_middle_change_mode       = chassis_mode;
		last_chassis_mode                = (chassis_status)chassis_middle_change_mode;
		
		rescue_middle_change_mode        = rescue_mode;
		last_rescue_mode                 = (rescue_status)rescue_middle_change_mode;
		
		clamp_middle_change_mode         = clamp_mode;
		last_clamp_mode                  = (clamp_status)clamp_middle_change_mode;
		
    	barrier_carry_middle_change_mode = barrier_carry_mode;
		last_barrier_carry_mode          = (barrier_carry_status)barrier_carry_middle_change_mode;
		
		supply_middle_change_mode        = supply_mode;
		last_supply_mode                 = (supply_status)supply_middle_change_mode;
		
		global_change_state = 0;
	}
	if((global_status)global_middle_change_mode != global_mode )                      //��ȡ��һ��ȫ��״̬
	{
		last_global_mode                = (global_status)global_middle_change_mode;
		global_middle_change_mode       = global_mode;
	}
	if((chassis_status)chassis_middle_change_mode !=chassis_mode )                    //��ȡ��һ�ε���״̬
	{
		last_chassis_mode               = (chassis_status)chassis_middle_change_mode;
		chassis_middle_change_mode      = chassis_mode;
		
		/*����̧�� ����ģʽ�仯ʱ ̧���ı䶯�����õ������ƺ������ٶ�*/
//		upraise_angle_ctrl_state[0] = 0;
//		upraise_angle_ctrl_state[1] = 0;
//		upraise.updown_flag        = RAISE; //̧����־
	}
	if(rescue_middle_change_mode != rescue_mode)                                      //��ȡ��һ�ξ�Ԯ״̬
	{
		last_rescue_mode                 = (rescue_status)rescue_middle_change_mode;
		rescue_middle_change_mode        = rescue_mode;
	}
	if((clamp_status)clamp_middle_change_mode != clamp_mode)                          //��ȡ��һ�μ�ȡ״̬
	{
		last_clamp_mode                  = (clamp_status)clamp_middle_change_mode;
		clamp_middle_change_mode         = clamp_mode;
	}
	if((barrier_carry_status)barrier_carry_middle_change_mode != barrier_carry_mode)   //��ȡ��һ�ΰ����ϰ���״̬
	{
		last_barrier_carry_mode          = (barrier_carry_status)barrier_carry_middle_change_mode;
		barrier_carry_middle_change_mode = barrier_carry_mode;
	}
	if((supply_status)supply_middle_change_mode != supply_mode)                        //��ȡ��һ�β���״̬
	{
		last_supply_mode                 = (supply_status)supply_middle_change_mode;
		supply_middle_change_mode        = supply_mode;
	}
}

void get_main_mode(void)
{   
  switch(rc.sw2)
  {
    case RC_UP:
    {
    	global_mode = MANUAL_CTRL;   			//����ģʽ
    }
    break;
    case RC_MI:
    {
    	global_mode = ENGINEER_CTRL;           //����ģʽ����ȡ�һ��ȣ�
    }break;
    case RC_DN:
    {
    	global_mode = RELEASE_CTRL;						//�ϵ�״̬
    }
    break;
    default:
    {
		global_mode = RELEASE_CTRL;
    }break;
  }
}
//extern uint8_t rescue_over;
uint8_t start_big_island_mode_state = 0;//Ϊ�˿��Ƹս������Դ������ʱ��̧ͷ�鿴��Ч��ӽ���ģʽ��־λ

void get_chassis_mode(void)
{
  switch(global_mode)
  {
    case RELEASE_CTRL:
		{
    	chassis_mode = CHASSIS_RELEASE;
			PUMP_OFF
		}
    break;  
		
		/*����������ڴ���ӷ���ģʽ*/
    case MANUAL_CTRL:
    {
			if((glb_sw.last_iw > IW_UP) && (rc.iw <= IW_UP) &&  rc.sw1 == RC_UP)
			{
				chassis_mode = CHASSIS_BARRIER_CARRY_MODE;
			}
			else if((glb_sw.last_iw > IW_UP) && (rc.iw <= IW_UP) &&  rc.sw1 == RC_DN)
			{
				chassis_mode = CHASSIS_CHECK_MODE;
			}
				else if(RC_IW_UP_NOMAL && rc.sw1 == RC_MI)        
            {
			    chassis_mode = CHASSIS_URGENT_MEASURE ;
			}	
			else if(chassis_mode != CHASSIS_BARRIER_CARRY_MODE && chassis_mode != CHASSIS_CHECK_MODE && chassis_mode != CHASSIS_URGENT_MEASURE)
			{
        		chassis_mode = CHASSIS_NORMAL_MODE;
			}
			
			if((glb_sw.last_iw < IW_DN) && (rc.iw >= IW_DN) && chassis_mode == CHASSIS_BARRIER_CARRY_MODE)
			{
				chassis_mode = CHASSIS_NORMAL_MODE;
			}
			if((glb_sw.last_iw < IW_DN) && (rc.iw >= IW_DN) && chassis_mode == CHASSIS_URGENT_MEASURE)
			{
				chassis_mode = CHASSIS_NORMAL_MODE;
			}
			else if((glb_sw.last_iw < IW_DN) && (rc.iw >= IW_DN) && chassis_mode == CHASSIS_CHECK_MODE)
			{
				chassis_mode = CHASSIS_NORMAL_MODE;				
			}
    }break;
    
	/***********************************************************
	***************global_mode = ENGINEER_CTRL******************
	******************��ģʽ���л�����ģʽ˵��*********************
	**** @1.rc.sw1 = RC_UP && glb_sw.last_iw���� �л�����ֱ����Դ��ģʽ
	**** @1.rc.sw1 = RC_UP && glb_sw.last_iw���� �л�����б����Դ��ģʽ

	**** @4.rc.sw1 = RC_MI && glb_sw.last_iw���� �л����һ�ģʽ
	**** @5.rc.sw1 = RC_MI && glb_sw.last_iw���� �л�����¼ģʽ

	**** @6.rc.sw1 = RC_DN && glb_sw.last_iw���� �л������С��Դ����ȡģʽ
	**** @7.rc.sw1 = RC_DN && glb_sw.last_iw���� �л����е���ģʽ
	****************************************************************/
    case ENGINEER_CTRL:
    {
		/*************����ģʽ����ң���л�ģʽ**************/
		if(RC_IW_UP && rc.sw1 == RC_UP)
    {          
       chassis_mode = CHASSIS_CLAMP_BIG_ISLAND_STRAIGHT_MODE;    //��ֱ����Դ��ģʽ   
		}
		else if(RC_IW_DN && rc.sw1 == RC_UP)
		{
			chassis_mode = CHASSIS_CLAMP_BIG_ISLAND_SLANTED_MODE;  	  //��б����Դ����ȡģʽ			
    }
		else if(RC_IW_UP && rc.sw1 == RC_MI)
		{
			chassis_mode = CHASSIS_EXCHANGE_MODE;     				  //�һ�ģʽ	
//				start_exexchange_cmd_state = 1;
		}
		else if(RC_IW_DN && rc.sw1 == RC_MI)
		{
			chassis_mode = CHASSIS_ANGLE_MODE;     						//ǰ��С��Դ����ȡģʽ
		}

		else if(RC_IW_UP && rc.sw1 == RC_DN)
		{
			chassis_mode = CHASSIS_CLAMP_SMALL_ISLAND_MODE;    		  //���С��Դ����ȡģʽ
			
			start_big_island_mode_state = 1;
		}

		else if(RC_IW_DN && rc.sw1 == RC_DN)
     	{        
        	chassis_mode = CHASSIS_GROUND_MODE;		        	 	  //��ȡ����ģʽ
      	}				
    }break;    			
    default:
    {
		chassis_mode = CHASSIS_RELEASE;
    }break;
  }
}

