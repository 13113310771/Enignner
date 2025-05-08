#include "judge_task.h"
#include "STM32_TIM_BASE.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "comm_task.h"
#include "detect_task.h"
#include "data_packet.h"
#include "judge_rx_data.h"
#include "judge_tx_data.h"
#include <string.h> 
#include "dma.h"

extern TaskHandle_t judge_rx_Task_Handle;
extern uint8_t judge_rxbuf_test[256];
UBaseType_t judge_tx_stack_surplus;
UBaseType_t judge_rx_stack_surplus;
uint8_t DATA[113] = {"AwakeLion!!!"};//该数组用来储存机器人交互的数据，可用户自行修改


void judge_tx_task(void *parm)
{
	uint32_t judge_wake_time = osKernelSysTick();
	static uint8_t i;
  while(1)
  {
		/*车间通信*/
//			judgement_client_packet_pack(DATA);
//			send_packed_fifo_data(&judge_txdata_fifo, DN_REG_ID);
		 
		  i++;
		/*首先发送不需要变化的 发送成功后将不在刷新这些数据*/
			if(i < 5)
			{
				judgement_client_graphics_draw_pack(CONSTANT);//显示常量
				send_packed_fifo_data(&judge_txdata_fifo, DN_REG_ID);
			}
			else if(i < 10)
			{
				/*显示实时工程当前模式*/
				judgement_client_graphics_draw_pack(CHASSIS_MODE);//显示底盘模式
				send_packed_fifo_data(&judge_txdata_fifo, DN_REG_ID);
			}
			else if(i < 15)
			{
				/*实时显示夹取箱数*/
				judgement_client_graphics_draw_pack(PICK_BOX);//显示夹取箱数
				send_packed_fifo_data(&judge_txdata_fifo, DN_REG_ID);
			}
			else if(i < 20)
			{
				judgement_client_graphics_draw_pack(AUXILIARY_LINE);//显示空接
				send_packed_fifo_data(&judge_txdata_fifo, DN_REG_ID);				
			}
			else if(i < 25)
			{
				judgement_client_graphics_draw_pack(MODE_STATE);//显示吸盘状态
				send_packed_fifo_data(&judge_txdata_fifo, DN_REG_ID);								
			}
			if(i > 20)
					i = 6;	
    
    judge_tx_stack_surplus = uxTaskGetStackHighWaterMark(NULL);    
			
    vTaskDelayUntil(&judge_wake_time, 100);
  }
}


//fifo_register(&test_fifo,judge_rxbuf_test,39,NULL,NULL);
void judge_rx_task(void *parm)
{
  uint32_t Signal;
	BaseType_t STAUS;
  while(1)
  {

		
      unpack_fifo_data(&judge_unpack_obj, DN_REG_ID,NORMAL);//同样再通过指针取址的方法把FIFO里的数据拿出来放进一个数组里
			unpack_fifo_data(&control_unpack_obj, DN_REG_ID,CONTROLLER);
  }
}

void DMA1_Stream1_IRQHandler(void)
{
	if(DMA_GetFlagStatus(DMA1_Stream1,DMA_FLAG_TCIF1) != RESET 
		 && DMA_GetITStatus(DMA1_Stream1,DMA_IT_TCIF1) != RESET)
	{
		dma_buffer_to_unpack_buffer(&judge_rx_obj,UART_DMA_FULL_IT,JUDGE_MAX_LEN);
		DMA_ClearFlag(DMA1_Stream1, DMA_FLAG_TCIF1);
	}
	
		if(DMA_GetFlagStatus(DMA1_Stream1,DMA_FLAG_HTIF1) != RESET 
		 && DMA_GetITStatus(DMA1_Stream1,DMA_IT_HTIF1) != RESET)
	{
		
		dma_buffer_to_unpack_buffer(&judge_rx_obj,UART_DMA_HALF_IT,JUDGE_MAX_LEN);
		DMA_ClearFlag(DMA1_Stream1, DMA_FLAG_HTIF1);
	}
	
	
}

void DMA1_Stream6_IRQHandler(void)
{
	if(DMA_GetFlagStatus(DMA1_Stream6,DMA_FLAG_TCIF6) != RESET 
		 && DMA_GetITStatus(DMA1_Stream6,DMA_IT_TCIF6) != RESET)
	{
		dma_buffer_to_unpack_buffer(&control_rx_obj,UART_DMA_FULL_IT,JUDGE_MAX_LEN);
		DMA_ClearFlag(DMA1_Stream6, DMA_FLAG_TCIF6);
	}
	
//		if(DMA_GetFlagStatus(DMA1_Stream6,DMA_FLAG_HTIF6) != RESET 
//		 && DMA_GetITStatus(DMA1_Stream6,DMA_IT_HTIF6) != RESET)
//	{
//		dma_buffer_to_unpack_buffer(&control_rx_obj,UART_DMA_HALF_IT,JUDGE_MAX_LEN);
//		DMA_ClearFlag(DMA1_Stream6, DMA_FLAG_HTIF6);
//	}
	
	
}

void UART8_IRQHandler(void)
{
		if(USART_GetFlagStatus(UART8,USART_FLAG_IDLE) != RESET 
		 && USART_GetITStatus(UART8,USART_IT_IDLE) != RESET)
	{
		dma_buffer_to_unpack_buffer(&control_rx_obj, UART_IDLE_IT,JUDGE_MAX_LEN);
		USART_ReceiveData(UART8);
		USART_ClearFlag(UART8, USART_FLAG_IDLE);//清除空闲中断标志位
	}
}

void USART3_IRQHandler(void)
{
	 BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	
	if(USART_GetFlagStatus(USART3,USART_FLAG_IDLE) != RESET 
		 && USART_GetITStatus(USART3,USART_IT_IDLE) != RESET)
	{
		dma_buffer_to_unpack_buffer(&judge_rx_obj, UART_IDLE_IT,JUDGE_MAX_LEN);
		USART_ReceiveData(USART3);
		USART_ClearFlag(USART3, USART_FLAG_IDLE);//清除空闲中断标志位
	}
	
	
	
	if(USART_GetITStatus(USART3,USART_IT_RXNE) != RESET)
	{
		USART_ReceiveData(USART3);
    //DMA_ClearITPendingBit(USART3, USART_IT_RXNE);//清除空闲中断标志位
    USART_ClearITPendingBit(USART3,USART_IT_RXNE);
    err_detector_hook(JUDGE_SYS_OFFLINE);
    
    if(judge_rx_Task_Handle != NULL) //避免任务没来得及创建就发送信号量，导致卡在断言机制中
    {
      xTaskNotifyFromISR((TaskHandle_t) judge_rx_Task_Handle, 
                         (uint32_t) JUDGE_UART_IDLE_SIGNAL,
                         (eNotifyAction) eSetBits,
                         (BaseType_t *)&xHigherPriorityTaskWoken);
      /*进行上下文切换*/
      if(xHigherPriorityTaskWoken != pdFALSE)
        portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
    }
	}
}

/*USART6 中断函数*/
void USART6_IRQHandler(void)
{	
	
	if(USART_GetFlagStatus(USART6,USART_FLAG_IDLE) != RESET 
		 && USART_GetITStatus(USART6,USART_IT_IDLE) != RESET)
	{
		USART_ReceiveData(USART6);
		USART_ITConfig(USART6,USART_IT_IDLE,DISABLE);	//关闭空闲中断，防止处理数据时再有数据进来
		DMA_Cmd(DMA2_Stream1,DISABLE);	
		DMA_SetCurrDataCounter(DMA2_Stream1,16);
		DMA_Cmd(DMA2_Stream1,ENABLE);
		USART_ClearITPendingBit(USART6,USART_IT_IDLE);
		USART_ClearFlag(USART6, USART_FLAG_IDLE);//清除空闲中断标志位
	}
		if(USART_GetFlagStatus(USART6,USART_IT_ORE_RX) != RESET)
	{
		
		USART_ReceiveData(USART6);
		USART_ClearFlag(USART6, USART_IT_ORE_RX);
	}
	/*串口发送完成中断*/
		if(USART_GetITStatus(USART6,USART_IT_TC) != RESET)
		{
			
			USART_ClearFlag(USART6, USART_FLAG_TC);//清除空闲中断标志位
			USART_ITConfig(USART6,USART_IT_TC,DISABLE);		
		}
		

}


