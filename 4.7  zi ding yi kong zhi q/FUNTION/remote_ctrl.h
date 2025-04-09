#ifndef _remote_ctrl_H
#define _remote_ctrl_H

#include "stm32f4xx.h"
#include "rc.h"


/*�л�ģʽ����*/                                                          /*IW_DN 1354 IW_UP 694*/                    
/*��rc.sw1��һ��ʱ���л��ɾ�Ԯ���������ϰ������*/

#define RC_IW_DN           ((rc.sw2 == RC_MI) && (glb_sw.last_iw < IW_DN) && (rc.iw >= IW_DN)) 
#define RC_IW_UP           ((rc.sw2 == RC_MI) && (glb_sw.last_iw > IW_UP) && (rc.iw <= IW_UP))

#define RC_SW1_DN						((glb_sw.last_sw1 == RC_MI) && (rc.sw1 == RC_DN))
#define RC_SW1_UP						((glb_sw.last_sw1 == RC_MI) && (rc.sw1 == RC_UP))




typedef enum
{
	CLAMP_UN_CMD = 0, //
	ACTION_ONE, //ȷ�϶���
	ACTION_TWO,//�����
	ACTION_THREE,
}clamp_action_status;

typedef enum
{
	EXCHANGE_UN_CMD  = 0, //
	PICK_ACTION, 
	RELESE_BOX_ACTION,
}exchange_action_status;

enum
{
  RC_UP = 1,
  RC_MI = 3,
  RC_DN = 2,
	IW_UP = 694,		//	min 364		middle 1024	����
	IW_DN = 1354,		//	max 1684
};

typedef enum
{
	CLOCKWISE_ADJUST = 0, //˳ʱ�����״̬
	ONTICLOCKWISE_ADJUST, //��ʱ�����״̬
}adjusement_t;

typedef struct
{
	/*��¼���˺Ͳ�����һ��״̬*/
  uint8_t last_sw1;
	uint8_t last_last_sw1;//��ȡʱ����Ϊ����Ҫ����࣬���������ϴε��ж�
	
  uint8_t last_sw2;
  uint16_t last_iw;
} sw_record_t;

typedef struct
{
	/*���̷���*/
  float vx; //ǰ��
  float vy; //����
  float vw; //ת��
  /*��̨����*/
  float pit_v;
  float yaw_v;
} rc_ctrl_t;


extern rc_ctrl_t rm;
extern sw_record_t glb_sw;
extern adjusement_t adjustment_cmd;
extern clamp_action_status clamp_action;
extern exchange_action_status exchange_action;

extern int32_t adjustment_slide_v;
extern int32_t adjustment_upraise_v;
extern int32_t adjustment_pit_v;

void remote_ctrl(rc_info_t *rc,uint8_t *dbus_buf);
void remote_ctrl_chassis_hook(void);
void remote_ctrl_gimbal_hook(void);
//void remote_ctrl_shoot_hook(void);

void remote_ctrl_supply_hook(void);
void remote_ctrl_rescue_hook(void);
void remote_ctrl_clamp_hook(void);
void remote_ctrl_barrier_carry_hook(void);

#endif


