#include "info_get_task.h"
#include "STM32_TIM_BASE.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "comm_task.h"
#include "keyboard.h"
#include "remote_ctrl.h"
#include "bsp_can.h"

#include "chassis_task.h"


#include "modeswitch_task.h"
#include "imu_task.h"
#include "sys_config.h"
#include "pc_rx_data.h"
#include "stdlib.h"
#include "math.h"

#include "controller.h"
#include "motor_task.h"
#include "motor_8010.h"

UBaseType_t info_stack_surplus;

extern TaskHandle_t can_msg_send_Task_Handle;
extern TaskHandle_t chassis_Task_Handle;
extern TaskHandle_t gimbal_Task_Handle;
extern TaskHandle_t clamp_Task_Handle;
extern TaskHandle_t upraise_Task_Handle;
extern TaskHandle_t barrier_carry_Task_Handle;
extern TaskHandle_t supply_Task_Handle;
extern TaskHandle_t slide_Task_Handle;
extern GO_Motorfield motor_recevie;
extern TaskHandle_t motor_Task_Handle;
extern TaskHandle_t comm_8010_task_Handle;
int16_t test_8010_pid;
int16_t test_8010_pid2;

extern float send_current_8010_1;
extern float send_current_8010_2;
float test_ecd_to_angle = 0;

void info_get_task(void *parm)
{
	uint32_t Signal;
	BaseType_t STAUS;
	while (1)
	{
		/*此函数用于获取通知值和清除通知值的指定位值*/
		STAUS = xTaskNotifyWait((uint32_t)NULL,					   // 等待前清零指定任务通知值的比特位
								(uint32_t)MODE_SWITCH_INFO_SIGNAL, // 等待成功后清零指定的任务通知值比特位
								(uint32_t *)&Signal,			   // 取出通知值
								(TickType_t)portMAX_DELAY);		   // 设置阻塞时间
		if (STAUS == pdTRUE)
		{
			if (Signal & MODE_SWITCH_INFO_SIGNAL)
			{
				
				taskENTER_CRITICAL();
				Get_Motor_info();
				get_chassis_info();
				taskEXIT_CRITICAL();
				get_global_last_info();
				if (global_mode == RELEASE_CTRL)
				{
					memset(&CAN1_current, 0, sizeof(CAN1_current));
					memset(&CAN2_current, 0, sizeof(CAN2_current));
					memset(&send_current_8010_1, 0, sizeof(send_current_8010_1));
					memset(&send_current_8010_2, 0, sizeof(send_current_8010_2));
					xTaskGenericNotify((TaskHandle_t)can_msg_send_Task_Handle,
									   (uint32_t)MODE_SWITCH_MSG_SIGNAL,
									   (eNotifyAction)eSetBits,
									   (uint32_t *)NULL);

					xTaskGenericNotify((TaskHandle_t)comm_8010_task_Handle,
									   (uint32_t)MODE_SWITCH_MSG_SIGNAL,
									   (eNotifyAction)eSetBits,
									   (uint32_t *)NULL);
				}
				else
				{ // 通知数据处理函数运行


					xTaskGenericNotify((TaskHandle_t)chassis_Task_Handle,
									   (uint32_t)INFO_GET_CHASSIS_SIGNAL,
									   (eNotifyAction)eSetBits,
									   (uint32_t *)NULL);

					xTaskGenericNotify((TaskHandle_t)motor_Task_Handle,
									   (uint32_t)INFO_GET_MOTOR_SIGNAL,
									   (eNotifyAction)eSetBits,
									   (uint32_t *)NULL);
				}

				info_stack_surplus = uxTaskGetStackHighWaterMark(NULL);
			}
		}
	}
}

static void get_chassis_info(void)
{
	/* ??????????? */
	/* get remote and keyboard chassis control information */
	keyboard_chassis_hook();
	remote_ctrl_chassis_hook();
	/* get chassis structure configuration parameter */
	get_structure_param();
}

static void get_structure_param(void)
{
	if ((pc_recv_mesg.structure_data.chassis_config == CUSTOM_CONFIG) && (pc_recv_mesg.structure_data.wheel_perimeter != 0) && (pc_recv_mesg.structure_data.wheel_base != 0) && (pc_recv_mesg.structure_data.wheel_track != 0))
	{
		glb_struct.chassis_config = CUSTOM_CONFIG;
		glb_struct.wheel_perimeter = pc_recv_mesg.structure_data.wheel_perimeter;
		glb_struct.wheel_base = pc_recv_mesg.structure_data.wheel_base;
		glb_struct.wheel_track = pc_recv_mesg.structure_data.wheel_track;
	}
	else
	{
		glb_struct.chassis_config = DEFAULT_CONFIG;
		glb_struct.wheel_perimeter = PERIMETER;
		glb_struct.wheel_base = WHEELBASE;
		glb_struct.wheel_track = WHEELTRACK;
	}
	if ((pc_recv_mesg.structure_data.gimbal_config == CUSTOM_CONFIG) && (abs(pc_recv_mesg.structure_data.gimbal_x_offset) < pc_recv_mesg.structure_data.wheel_base / 2) && (abs(pc_recv_mesg.structure_data.gimbal_y_offset) < pc_recv_mesg.structure_data.wheel_track / 2))
	{
		glb_struct.gimbal_config = CUSTOM_CONFIG;
		glb_struct.gimbal_x_offset = pc_recv_mesg.structure_data.gimbal_x_offset;
		glb_struct.gimbal_y_offset = pc_recv_mesg.structure_data.gimbal_y_offset;
	}
	else
	{
		glb_struct.gimbal_config = DEFAULT_CONFIG;
		glb_struct.gimbal_x_offset = GIMBAL_X_OFFSET;
		glb_struct.gimbal_y_offset = GIMBAL_Y_OFFSET;
	}
}
static void CAN_ecd_to_angle(moto_measure_t *ptr, motor_t *Motor)
{
	switch (Motor->MOTOR_TYPE)
	{
	case M6020:
		ptr->total_angle = ptr->total_ecd / (ENCODER_ANGLE_RATIO); // 6020电机1比1直接转
		break;
	case M3508:
		ptr->total_angle = ptr->total_ecd / (ENCODER_ANGLE_RATIO * DECELE_RATIO_3508); // 3508电机
		break;
	case M2006:
		ptr->total_angle = ptr->total_ecd / (ENCODER_ANGLE_RATIO * DECELE_RATIO_2006); // 2006电机

		break;
	}
}

static void Get_Motor_info(void)
{
	uint8_t ID;
	for (ID = 0; ID < MOTOR_NUMBER; ID++)
	{
		if (Whether_Brushless_Motor(Motor[ID]))
		{
			uint8_t esc_ID;
			esc_ID = Motor[ID].Brushless.ESC_ID - 1;

			switch (Motor[ID].Brushless.CAN_ID)
			{
			case 1:
			{
				CAN_ecd_to_angle(&Motor_CAN1_data[esc_ID], &Motor[ID]);
				Motor[ID].Brushless.ecd_fdb = Motor_CAN1_data[esc_ID].ecd;
				Motor[ID].Brushless.spd_fdb = Motor_CAN1_data[esc_ID].speed_rpm;
				Motor[ID].Brushless.angle_fdb = Motor_CAN1_data[esc_ID].total_angle;
				Motor[ID].Brushless.current_read = Motor_CAN1_data[esc_ID].given_current;
			}
			break;
			case 2:
			{
				CAN_ecd_to_angle(&Motor_CAN2_data[esc_ID], &Motor[ID]);
				Motor[ID].Brushless.ecd_fdb = Motor_CAN2_data[esc_ID].ecd;
				Motor[ID].Brushless.spd_fdb = Motor_CAN2_data[esc_ID].speed_rpm;
				Motor[ID].Brushless.angle_fdb = Motor_CAN2_data[esc_ID].total_angle;
				Motor[ID].Brushless.current_read = Motor_CAN2_data[esc_ID].given_current;
			}
			break;
			}

			if (Motor[ID].MOTOR_TYPE == go_8010)
			{
				go8010_receive();
				// ecd_to_angle_8010(&motor_recevie);
				switch (Motor[ID].Brushless.GO_ID)
				{
				case GO_8010_1:
				{
					if (motor_recevie.id == GO_8010_1)
					{

						Motor[ID].Brushless.torque_fbd = motor_recevie.T;
						Motor[ID].Brushless.spd_fdb = motor_recevie.W;
							Motor[ID].Brushless.l_angle_fdb=Motor[ID].Brushless.angle_fdb;
						Motor[ID].Brushless.angle_fdb = motor_recevie.Pos;
						// test_ecd_to_angle=(((motor_recevie.Pos)*180.00f)/PI)/6.33;
						
					}
				}
				break;
				case GO_8010_2:
				{
					if (motor_recevie.id == GO_8010_2)
					{
						Motor[ID].Brushless.torque_fbd = motor_recevie.T;
						Motor[ID].Brushless.spd_fdb = motor_recevie.W;
			      Motor[ID].Brushless.l_angle_fdb=Motor[ID].Brushless.angle_fdb;
						Motor[ID].Brushless.angle_fdb = motor_recevie.Pos;

						// test_8010_pid=Motor[13].Brushless.angle_fdb;
						// test_8010_pid2=Motor[13].Brushless.angle_ref;
					}
				}
				break;
				}
			}
		}
	}
	for (ID = 0; ID <= CHASSIS_BR; ID++)
	{ // 底盘删
		if (chassis_mode != CHASSIS_EXCHANGE_MODE && chassis_mode != CHASSIS_ANGLE_MODE)
		{
			Motor_CAN1_data[ID].round_cnt = 0;
			Motor_CAN1_data[ID].total_ecd = Motor_CAN1_data[ID].round_cnt * 8192 + Motor_CAN1_data[ID].ecd - Motor_CAN1_data[ID].offset_ecd;
		}

		chassis.wheel_angle_fdb[ID] = Motor_CAN1_data[ID].total_angle;
		chassis.wheel_spd_fdb[ID] = Motor_CAN1_data[ID].speed_rpm;
		chassis.current[ID] = Motor_CAN1_data[ID].given_current;
	}
}

/*这个函数目前没有用到，其目的为获取相对角度，让电机转劣弧*/
static int16_t get_relative_pos(int16_t raw_ecd, int16_t center_offset) // 得到相对位置
{
	int16_t tmp = 0;
	if (center_offset >= 4096) //????????????
	{
		if (raw_ecd - center_offset > -4096)
			tmp = raw_ecd - center_offset; //????? = ????
		else
			tmp = raw_ecd + 8192 - center_offset; //????? = ????
	}
	else
	{
		if (raw_ecd - center_offset > 4096)
			tmp = raw_ecd - 8192 - center_offset; //????? = ????
		else
			tmp = raw_ecd - center_offset; //????? = ????
	}
	return tmp;
}

uint8_t rc_change_state = 1;
/*?м?仯???????*/
uint8_t rc_middle_change_sw1;
uint8_t rc_middle_change_sw2;
/*此函数用于获取波轮上一次的位置*/
static void get_global_last_info(void)
{
	if (rc_change_state)
	{
		rc_middle_change_sw1 = rc.sw1;
		glb_sw.last_sw1 = rc_middle_change_sw1;
		glb_sw.last_last_sw1 = glb_sw.last_sw1;

		rc_middle_change_sw2 = rc.sw2;
		glb_sw.last_sw2 = rc_middle_change_sw2;

		rc_change_state = 0;
	}
	if (rc_middle_change_sw1 != rc.sw1)
	{
		glb_sw.last_last_sw1 = glb_sw.last_sw1;
		glb_sw.last_sw1 = rc_middle_change_sw1;
		rc_middle_change_sw1 = rc.sw1;
	}
	if (rc_middle_change_sw2 != rc.sw2)
	{
		glb_sw.last_sw2 = rc_middle_change_sw2;
		rc_middle_change_sw2 = rc.sw2;
	}

	glb_sw.last_iw = rc.iw;
}
