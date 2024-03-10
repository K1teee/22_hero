#include "stm32f4xx.h"    
#include "RC_task.h"
#include "remote_control.h"
#include "gimbal.h"
#include "math.h"
#include "Vision_Task.h"        
#include "can_comm.h"

#include "can_receive.h"

extern shoot_task_t rc_shoot;

extern shoot_t shoot;


int vision_switch_flag=0;//�Ӿ����صı�ʶ��������ʵ�ֲ�һ�¿����Ӿ����ٲ�һ�¿����Ӿ��Ĺ��� 
static int deadline_judge(uint8_t a);
float rc_cali(int max,int min, int RC_actual, int RC_max, int RC_mid, int RC_min,int deadline);
int F_flag=0;//�ж�F�Ƿ���
int frie_first_flag=0;//�жϵ�һ�η���
int Fric_Switch_Flag = 0;
int Shoot_Num = 0;
int MOUSE_pre_left_cnt=0;	
//��ʱ����
int Chassis_Spin_Delay_Cnt					= 0;
int Gimbal_Reverse_Bottom_Delay_Cnt			= 0;
int Chassis_Reverse_Bottom_Delay_Cnt		= 0;
int Gimbal_Precision_Mode_Delay_Cnt			= 0;
int Chassis_TurnAround_Delay_Cnt			= 0;
int Fric_Switch_Delay_Cnt					= 0;
int Pitch_Calibration_Delay_Cnt				= 0;

KEY_CONTROL KEY_MODE=KEY_OFF;
extern int Sent_dataC;
//�޸��ⲿģʽ����
extern int Gimbal_Precision_Mode;
extern int Last_Gimbal_Precision_Mode;
extern int Gimbal_Precision_Activated_Flag;
extern int Gimbal_Precision_Inactivated_Flag;


void control_mode_judge(void)
{
	if(rc_ctrl.rc.ch[0]!=0||rc_ctrl.rc.ch[1]!=0||rc_ctrl.rc.ch[2]!=0||rc_ctrl.rc.ch[3]!=0||rc_ctrl.rc.ch[4]!=0)
		KEY_MODE=KEY_OFF;
	if(KEY_board||MOUSE_x||MOUSE_y||MOUSE_z)
		KEY_MODE=KEY_ON;
}
int shoot_true=0,shoot_true_cnt=0;
//void remote_control_data(void)
//{

//	
//	if(switch_is_up(SW_R))//���
//	{
//		
//		
//	}

//	if(switch_is_up(SW_L) ) //С����ģʽ
//	{
//		
//	}

//	if(switch_is_mid(SW_L) ) //�涯
//	{
//		
//		
//	}
//	if(switch_is_down(SW_L)) //����״̬
//	{
//		
//		
//	}
//	
//	
//	
//}
int R_flag=0,R_cnt=0;

void key_control_data(void)
{	

	if(R_flag==1)
		R_cnt++;
	if(R_cnt>=1000) {R_flag=0;R_cnt=0;}
		rc_sent.x_speed=0;
		rc_sent.y_speed=0;
	//Shift������
	if(KEY_board & KEY_PRESSED_OFFSET_SHIFT)
	{
		if(KEY_board & KEY_PRESSED_OFFSET_W)
			rc_sent.x_speed=50;
		if(KEY_board & KEY_PRESSED_OFFSET_S)
			rc_sent.x_speed=-50;
		if(KEY_board & KEY_PRESSED_OFFSET_A)
			rc_sent.y_speed=-50;
		if(KEY_board & KEY_PRESSED_OFFSET_D)
			rc_sent.y_speed=50;
		rc_sent.yaw.target_angle=limits_change(KEY_YAW_ANGLE_MAXX_RUN,KEY_YAW_ANGLE_MINN_RUN,MOUSE_x,KEY_MAXX,KEY_MINN);
		if (Gimbal_Precision_Mode == 0)
			rc_sent.pitch.target_angle=limits_change(KEY_PITCH_ANGLE_MAXX_RUN,KEY_PITCH_ANGLE_MINN_RUN,MOUSE_y,KEY_MAXX,KEY_MINN);
		else rc_sent.pitch.target_angle=limits_change(KEY_PITCH_ANGLE_MAXX_RUN,KEY_PITCH_ANGLE_MINN_RUN,MOUSE_z,KEY_MAXX,KEY_MINN);
	}
	else
	{
		if(KEY_board & KEY_PRESSED_OFFSET_W)
			rc_sent.x_speed=KEY_X_SPEED_MAXX;
		if(KEY_board & KEY_PRESSED_OFFSET_S)
			rc_sent.x_speed=KEY_X_SPEED_MINN;
		if(KEY_board & KEY_PRESSED_OFFSET_A)
			rc_sent.y_speed=KEY_Y_SPEED_MINN;
		if(KEY_board & KEY_PRESSED_OFFSET_D)
			rc_sent.y_speed=KEY_Y_SPEED_MAXX;
		rc_sent.yaw.target_angle=limits_change(KEY_YAW_ANGLE_MAXX_ON,KEY_YAW_ANGLE_MINN_ON,MOUSE_x,KEY_MAXX,KEY_MINN);
		if (Gimbal_Precision_Mode == 0)
			rc_sent.pitch.target_angle=limits_change(KEY_PITCH_ANGLE_MAXX_ON,KEY_PITCH_ANGLE_MINN_ON,MOUSE_y,KEY_MAXX,KEY_MINN);
		else rc_sent.pitch.target_angle=limits_change(KEY_PITCH_ANGLE_MAXX_ON,KEY_PITCH_ANGLE_MINN_ON,MOUSE_z,KEY_MAXX,KEY_MINN);
	}
	
	//Ctrl+R�������涯����
	if(KEY_board & KEY_PRESSED_OFFSET_R)
	{
		if (KEY_board & KEY_PRESSED_OFFSET_CTRL)
		{
			if (Chassis_Spin_Delay_Cnt == 0)
			{
				Chassis_Spin_Delay_Cnt = 100;
				
				CAN_COMM_T_MODE(0,3,3);//�涯ģʽ
			}
		}
	}
	//R��С����ģʽ����
	if(KEY_board & KEY_PRESSED_OFFSET_R)
	{
		if (Chassis_Spin_Delay_Cnt == 0)
		{
			Chassis_Spin_Delay_Cnt = 100;
			
			CAN_COMM_T_MODE(0,2,3);//С����ģʽ
		}
	}
	

	//Ħ���ֿ���
	if (MOUSE_pre_right)
	{
		if (Fric_Switch_Delay_Cnt==0)
		{
			if (Fric_Switch_Flag == 0) Fric_Switch_Flag=1;
			else if (Fric_Switch_Flag == 1) Fric_Switch_Flag=0;
			
			shoot.fric_speedset = -SHOOT_FRIC_HIGH_SPEED*Fric_Switch_Flag;
		   
			
			Fric_Switch_Delay_Cnt = 100;
		}
	}
	//���估�����ӳ�
	if( MOUSE_pre_left==1   ) 
    {			
		if (Fric_Switch_Flag)
		{
			rc_shoot.trigger.target_angle= 1 ;//shoot_num == 1
			rc_shoot.trigger.last_shoot_flag= 1 ;
			shoot_true=0 ;
			Shoot_Num++ ;
		}
		else rc_shoot.trigger.target_angle=0;
		
		trigger_task(rc_shoot.trigger.target_angle);
	}
	if(MOUSE_pre_left==0) 
	{
		MOUSE_pre_left_cnt++;
		if(MOUSE_pre_left_cnt>=100)
			{rc_shoot.trigger.last_shoot_flag=0;MOUSE_pre_left_cnt=0;}
	}
    
	
	

	if(KEY_PRESSED_OFFSET_B&KEY_board)
	{
		if (Gimbal_Precision_Mode_Delay_Cnt == 0) 
		{
			//ģʽ�л�
			if (Gimbal_Precision_Mode==1) Gimbal_Precision_Mode = 0;
			else Gimbal_Precision_Mode = 1;
			
			//��ʱ����
			Gimbal_Precision_Mode_Delay_Cnt = 100;
		}
	}
//	if(KEY_PRESSED_OFFSET_F&KEY_board)
//	{
//		if (Pitch_Calibration_Delay_Cnt == 0) 
//		{
//			//����У׼
//			Gimbal_Calibration_Target_Times++;
//			//��ʱ����
//			Pitch_Calibration_Delay_Cnt = 1000;
//		}
//	}
//	extern int Relay_Set_State;
//	if (KEY_board&KEY_PRESSED_OFFSET_CTRL)
//	{
//		if (KEY_board&KEY_PRESSED_OFFSET_G)	//Ctrl+G	��ع���ģʽ
//			Relay_Set_State = 2;
//	}
	

	//��ʱ��
	if (Chassis_Spin_Delay_Cnt > 0)				Chassis_Spin_Delay_Cnt--;
	if (Gimbal_Precision_Mode_Delay_Cnt > 0)	Gimbal_Precision_Mode_Delay_Cnt--;
	if (Gimbal_Reverse_Bottom_Delay_Cnt > 0)	Gimbal_Reverse_Bottom_Delay_Cnt--; 
	if (Chassis_Reverse_Bottom_Delay_Cnt > 0)	Chassis_Reverse_Bottom_Delay_Cnt--;
	if (Chassis_TurnAround_Delay_Cnt>0)			Chassis_TurnAround_Delay_Cnt--;
	if (Fric_Switch_Delay_Cnt>0)				Fric_Switch_Delay_Cnt--;
	if (Pitch_Calibration_Delay_Cnt>0)			Pitch_Calibration_Delay_Cnt--;
}
/**
	* @brief       �����жϺ���
	* @param[a]		��Ҫ�жϵ���ֵ
	* @retvel      1�������޷�����0�����޷���
*/
static int deadline_judge(uint8_t a)
{
	if(abs(a-RC_MIDD)<=DEADLINE) return 1;
	else return 0;
}

/**
	* @brief       			ң����ӳ�䵽ʵ���ٶȻ���ʵ�ʽǶ�
	* @param[max]		    ��Ҫӳ������ֵ
	* @param[min]		    ��Ҫӳ�����Сֵ
	* @param[RCactual]	ң����ͨ��ʵ��ֵ
	* @param[RCmax]		  ң����ͨ�����ֵ
	* @param[RCmid]		  ң����ͨ����ֵ
	* @param[RCmin]		  ң����ͨ����Сֵ
	* @param[deadline]	ң��������
	* @retvel      1�������޷�����0�����޷���
*/
float rc_cali(int max,int min, int RC_actual, int RC_max, int RC_mid, int RC_min, int deadline)
{
	float value;
	if((RC_actual-RC_mid)>=deadline)
	{
		value=(float)(RC_actual-RC_mid-deadline)/(RC_max-RC_min);
		value*=max;
	}
	else if((RC_actual-RC_mid)<-deadline)
	{
		value=(float)(RC_actual-RC_mid+deadline)/(RC_max-RC_min);
		value*=max;
	}
	else
		value=0;
	return value;
}




