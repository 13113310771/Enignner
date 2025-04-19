#ifndef _user_H
#define _user_H

#define CLAMP_ON   	 	 GPIO_SetBits(GPIOE,GPIO_Pin_5);      //夹取电磁阀
#define CLAMP_OFF   	 		GPIO_ResetBits(GPIOE,GPIO_Pin_5);

#define Air_Pump_Clamp_OFF   	 	       GPIO_ResetBits(GPIOC,GPIO_Pin_2);
#define Air_Pump_Clamp_ON   	 				GPIO_SetBits(GPIOC,GPIO_Pin_2);   //夹取气泵

#define Air_Pump_Exchange_OFF   	 	  GPIO_ResetBits(GPIOF,GPIO_Pin_1);       //兑换气泵
#define Air_Pump_Exchange_ON   	 			 GPIO_SetBits(GPIOF,GPIO_Pin_1);

#define Exchange_ON       GPIO_SetBits(GPIOF,GPIO_Pin_0);    //兑换电磁阀
#define Exchange_OFF     GPIO_ResetBits(GPIOF,GPIO_Pin_0);

#define PUMP_ON   	 	 {Air_Pump_Clamp_ON  Air_Pump_Exchange_ON}
#define PUMP_OFF   	 		{Air_Pump_Clamp_OFF  Air_Pump_Exchange_OFF}

//ɾ
#define MANIPULATOR_6020_ID 0
#define SLIDE_NUMBER 4

#define	ADRUST_X 0
#define	ADRUST_Y 1
#define	ADRUST_Z 2
#define	ADRUST_YAW 3
#define	ADRUST_PITCH 4
#define	ADRUST_ROLL 5
#define JOINT_1     6
#define JOINT_2     7
#define P_T_PITCH	8
#define P_T_YAW		9

//#define BACK_DRIVE 

typedef enum
{
	SPEED_MODE=1,
	ANGLE_MODE,
}PID_Mode_t;




#endif
