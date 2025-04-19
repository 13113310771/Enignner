#include "STM32_TIM_BASE.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "bsp_can.h"
#include "pid.h"
#include "motor_task.h"
#include "motor_8010.h"
#include "comm_8010_task"
void coom_8010_task(void *parm)
{
	uint32_t Signal;
	BaseType_t STAUS;
	
	while(1)
	{
	
		if(STAUS ==pdTRUE)
		{
			
		}
		
		
	}	
}