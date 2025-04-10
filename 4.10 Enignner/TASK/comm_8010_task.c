#include "STM32_TIM_BASE.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "bsp_can.h"
#include "pid.h"
#include "motor_8010.h"
#include "comm_8010_task.h"
#include "comm_task.h"
#include "modeswitch_task.h"
#include "comm_8010_task.h"
#include "motor_task.h"

float send_current_8010_1 = 0;
float send_current_8010_2 = 0;

float send_ref_angle_8010_1 = 0;
float send_ref_angle_8010_2 = 0;

float send_fdb_angle_8010_1 = 0;
float send_fdb_angle_8010_2 = 0;
float angle_test = 0;
float speed_test = 0;

uint8_t motor8010_init_flag=0;
fp32 motor8010_init_angle=0;
fp32 test_8010_angle=0;
fp32	test_test=0;
uint8_t chazhi=0;

float target_angle[2];
float receive_angle[2];
float tete=1;

extern TaskHandle_t motor_Task_Handle;

void coom_8010_task(void *parm)
{
	uint32_t Signal;
	BaseType_t STAUS;
	uint32_t comm_8010_time = osKernelSysTick();

	while (1)
	{
		STAUS = xTaskNotifyWait((uint32_t)NULL, // 等待前清零指定任务通知值的比特位（旧值对应bit清0）
								(uint32_t)INFO_SEND_MOTOR_SIGNAL | MODE_SWITCH_MSG_SIGNAL|GO_8010_INIT_SIGNAL,
								(uint32_t *)&Signal,		// 用来取出通知值（如果不需要取出，可设为NULL）
								(TickType_t)portMAX_DELAY); // 设置阻塞时间
		if (STAUS == pdTRUE)
		{
			if (Signal == MODE_SWITCH_MSG_SIGNAL) 
			{
				
			go_8010_test_tesk1(GO_8010_1,0,0,0,0,0);
			vTaskDelayUntil(&comm_8010_time, 8);
			go_8010_test_tesk1(GO_8010_2,0,0,0,0,0);
			vTaskDelayUntil(&comm_8010_time, 8);	
			}
			if (Signal == INFO_SEND_MOTOR_SIGNAL) //??????
			{				
					ref_8010_angle(target_angle);
					get_8010_angle(receive_angle);
							
					go_8010_test_tesk1(GO_8010_1,0,tamp_task(receive_angle[0],tete,target_angle[0]),0,0.8,0.05);
					vTaskDelayUntil(&comm_8010_time, 8);
					go_8010_test_tesk1(GO_8010_2,0,tamp_task(receive_angle[1],tete,target_angle[1]),0,0.4,0.05);
					vTaskDelayUntil(&comm_8010_time, 8);
				
				
				
				
				
			}
			if(Signal == GO_8010_INIT_SIGNAL)
		{
			if(!judge_duzhuan_OK(GO_8010_1,Single_mode))
			{
				 go_8010_init_task(GO_8010_1,0.5);
			}	
			else if(judge_duzhuan_OK(GO_8010_1,Single_mode))
				{
				vTaskDelayUntil(&comm_8010_time, 10	);//这个延时是为了不让1的stop的值被刷掉
				go_8010_init_task(GO_8010_2,-0.2);
				
				}

			
		}
			vTaskDelayUntil(&comm_8010_time, 2); // 绝对延时函数
		}
		
		
		
	}
}

void go_8010_test_tesk1(int id, float T, float Pos, float W, float K_P, float K_W)
{
	GO_M8010_send_data(id, T, Pos, W, K_P, K_W); // 计算发数
	DMA_Cmd(DMA2_Stream1, ENABLE);
	DMA_Cmd(DMA2_Stream6, ENABLE);
}


float tamp_task(float input,float step,float ref_angle)
{
	if(fabs(ref_angle-input)<step)
		return ref_angle;
	else 
	{
		if(input<ref_angle)
			input+=step;

		else if(input>ref_angle)
			input-=step;
	}
	
	return input;
}

void go_8010_init(int id,float torque)
{
	GO_M8010_send_data(id, torque, 0, 0, 0, 0);
	DMA_Cmd(DMA2_Stream1, ENABLE);
	DMA_Cmd(DMA2_Stream6, ENABLE);
	

}
int go_8010_stop(int id)
{
	GO_M8010_send_data(id, 0, 0, 0, 0, 0);
	DMA_Cmd(DMA2_Stream1, ENABLE);
	DMA_Cmd(DMA2_Stream6, ENABLE);
	return 1;
}



void ref_8010_angle(float get_angle[])
{
	for(int ID=0;ID<MOTOR_NUMBER;ID++)
	{
		switch(Motor[ID].Brushless.GO_ID)
		{
			case GO_8010_1:get_angle[0]=Motor[ID].Brushless.angle_ref;break;
			
			case GO_8010_2:get_angle[1]=Motor[ID].Brushless.angle_ref;break;
			
			
		}
	}
}

void get_8010_angle(float receive_angle[])
{
	for(int ID=0;ID<MOTOR_NUMBER;ID++)
	{
		switch(Motor[ID].Brushless.GO_ID)
		{
			case GO_8010_1:receive_angle[0]=Motor[ID].Brushless.angle_fdb;break;
			
			case GO_8010_2:receive_angle[1]=Motor[ID].Brushless.angle_fdb;break;
			
		}
	}
}




void go_8010_init_task(int GO_ID,float speed)
{
	 go_8010_init(GO_ID,speed);
	for(int ID=0;ID<MOTOR_NUMBER;ID++)
	{
		 if(Motor[ID].Brushless.GO_ID==GO_ID)
		{
			 if(Motor[ID].Brushless.offset_angle_init_flag == 0)
			{
			   
			   if(Motor[ID].Brushless.angle_fdb-Motor[ID].Brushless.l_angle_fdb==0)
				{
					Motor[ID].Brushless.err_count++;
	          if(Motor[ID].Brushless.err_count>300)
					{
	     	    Motor[ID].Brushless.offset_angle_init_flag = 1;
	     	    Motor[ID].Angle.offset_angle=Motor[ID].Brushless.angle_fdb;		
	    
					}						
				}
			}
      else if(Motor[ID].Brushless.offset_angle_init_flag == 1)
			{
				go_8010_stop(GO_ID);
			}				
		}
	}
}



int judge_duzhuan_OK(uint8_t GO_ID,uint8_t modle_type)
{
	uint16_t judge_single=0;
	uint16_t judge_both1=0;
	uint16_t judge_both2=0;
	if(modle_type==Single_mode)
	{
		for(int ID=0;ID<MOTOR_NUMBER;ID++)
		{
				if(Motor[ID].Brushless.GO_ID==GO_ID)
			{
					if(Motor[ID].Brushless.offset_angle_init_flag==1)
				{
				  judge_single=1;
					return 1;
				}
				else return 0;
			}
	
		}	
	}
	else if(modle_type==Both_mode)
	{
		for(int ID=0;ID<MOTOR_NUMBER;ID++)
		{
				if(Motor[ID].Brushless.GO_ID==GO_8010_1)
			{
					if(Motor[ID].Brushless.offset_angle_init_flag==1)
				{
				  judge_both1=1;
				}
			}
			if(Motor[ID].Brushless.GO_ID==GO_8010_2)
			{
					if(Motor[ID].Brushless.offset_angle_init_flag==1)
				{
				  judge_both2=1;
				}
			}
	
		}

			if(judge_both1&&judge_both2)
			{
				return 1;
			}
			else
			{
				return 0;
			}
	}
}


