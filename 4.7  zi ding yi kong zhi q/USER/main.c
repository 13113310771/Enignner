/**
  ******************************************************************************
  * 函数库  ： 基于STM32F4标准库 
  * 芯片型号： STM32F427IIH
  * 代码版本： 第一代框架
  * 完成日期： 2019-11-9
  ******************************************************************************
  *                          RM . 电控之歌
  *
  *                  一年备赛两茫茫，写程序，到天亮。
  *                      万行代码，Bug何处藏。
  *                  机械每天新想法，天天改，日日忙。
  *
  *                  视觉调试又怎样，朝令改，夕断肠。
  *                      相顾无言，惟有泪千行。
  *                  每晚灯火阑珊处，夜难寐，继续肝。
  ******************************************************************************
**/

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
/*freertos*/
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
/*config*/
#include "delay.h"
#include "gpio.h"
#include "can.h"
#include "usart.h"
#include "spi.h"
#include "tim.h"
#include "STM32_TIM_BASE.h"
/*bsp*/
#include "bsp_flash.h"
#include "judge_rx_data.h"
#include "judge_tx_data.h"
#include "pc_rx_data.h"
#include "pc_tx_data.h"
/*task*/
#include "start_task.h"
//#include "gimbal_task.h"
#include "imu_task.h"
#include "detect_task.h"

#include "chassis_task.h"

#include "stdlib.h"
/** @addtogroup Template_Project
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
void flash_cali(void);
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program10
  * @param  None
  * @retval None
  */
int main(void)
{ 
	
	
			
  /*功能参数初始*/
  chassis_param_init();
//	rescue_param_init();
	detect_param_init();

	/*通信参数初始*/
  judgement_rx_param_init();
  judgement_tx_param_init();
  pc_rx_param_init();
  pc_tx_param_init();
	
	/*配置*/
	SysTick_Init(180);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//优先级分组4：全为抢占优先级
	TIM_BASE_Init(10-1,9000-1);//90 000 000/9 000 = 10 000/10 = 1000HZ
	GPIO_INIT();
	TIM5_DEVICE(20000-1,90-1); //舵机  自动重装载值 20000  1M/20000 = 50HZ
//	TIM4_ENCODER_Init();
	TIM4_DEVICE(20000-1,90-1); //舵机  PD13 14 15  13夹取yaw 14 夹取pitch
	TIM2_DEVICE(20000-1,90-1); //舵机  12相机 PA2兑换pitch
	CAN1_DEVICE(CAN_Mode_Normal, CAN_SJW_1tq, CAN_BS1_3tq, CAN_BS2_5tq, 5);
	CAN2_DEVICE(CAN_Mode_Normal, CAN_SJW_4tq, CAN_BS1_3tq, CAN_BS2_5tq, 5);
	USART1_DEVICE();
	USART3_DEVICE();
	USART6_DEVICE();
	UART8_DEVICE();
	UART7_DEVICE();
	SPI_DEVICE();

	
	/*从内部FLASH读出校准数据*/
//	flash_cali();//（云台校准）

  /*创建start_task任务*/
	TASK_START();
	/*开启任务调度*/
	vTaskStartScheduler();//自动创建一个空闲任务（优先级为0）
  /* Infinite loop */

  while (1)
  {	
  }
}

void flash_cali(void)
{
  BSP_FLASH_READ();
  if(cali_param.cali_state != CALI_DONE)
  {
    for( ; ; );
  }
  else
  {
//    gimbal.pit_center_offset = cali_param.pitch_offset;
//    gimbal.yaw_center_offset = cali_param.yaw_offset;
  }
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
