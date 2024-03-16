#include "mode.h"
#include "remote_control.h"
#include "gimbal.h"
#include "can_comm.h"
#include "bsp_imu.h"
#include "usbd_cdc_if.h"
#include "Vision_Task.h"
#include "RC_task.h"

extern shoot_t shoot;

extern shoot_task_t rc_shoot;

extern tray_mode_t tray_mode;

extern MODE_t Mode;
extern gimbal_y_t gimbal_y;
extern gimbal_p_t gimbal_p;
extern gimbal_move_t gimbal_move;
extern TIM_HandleTypeDef htim2;

extern VISION_GET_t vision_sent;

extern RC_ctrl_t rc_data;

int speed_g1,speed_g2,speed_g3;
int mode_s;

MODE_t Mode;
V_MODE_t V_Mode;

extern VISION_MODE vision_mode;

int lever_flag = 2,lever_num;
 void mode_select(RC_ctrl_t *rc_data)
 {
	//右边为s0
	 if(rc_data->rc.s[0] == 2 )
	 {
		 Mode = ZERO_FORCE;
	 }
	 if(rc_data->rc.s[0] == 3 && rc_data->rc.s[1] == 3)
	 {
		 Mode = FOLLOW;
	 }
	if(rc_data->rc.s[0] == 3 && rc_data->rc.s[1] == 2)
	{
		Mode = TOP_ANGLE;
	}
	if(rc_data->rc.s[0] == 3 && rc_data->rc.s[1] == 1)
	{
		Mode = HANGING;
		
	}
	if(rc_data->rc.s[0] == 1 && rc_data->rc.s[1] == 1)
	{
		vision_mode = VISION_ON;
	}
	if(rc_data->rc.s[0] != 1 && rc_data->rc.s[1] != 1)
	{
		vision_mode = VISION_OFF;
	}
	
 }
 
 
 void mode_control(void)
 {
	
	if(Mode == TOP_ANGLE)
		
		{
			gimbal_rc_to_set(&gimbal_move);//遥控器数据传给云台电机
			gimbal_ctrl_loop(&gimbal_move);//云台电机闭环控制
			CAN_CMD_GIMBAL(gimbal_move.motor_gimbal[1].give_current,gimbal_move.motor_gimbal[2].give_current,gimbal_move.motor_gimbal[3].give_current);
			

		}
	if(Mode == FOLLOW)
		
		{
			gimbal_rc_to_set(&gimbal_move);//遥控器数据传给云台电机
			gimbal_ctrl_loop(&gimbal_move);//云台电机闭环控制
			CAN_CMD_GIMBAL(gimbal_move.motor_gimbal[1].give_current,gimbal_move.motor_gimbal[2].give_current,gimbal_move.motor_gimbal[3].give_current);
		
		}
	if(Mode == ZERO_FORCE)
		{
		
			CAN_CMD_GIMBAL(0,0,0);
			CAN_COMM_XYZ_IMU(0,0,0,0);
		}
		if(Mode == HANGING)
		{
		
			gimbal_rc_to_set(&gimbal_move);//遥控器数据传给云台电机
			gimbal_ctrl_loop(&gimbal_move);//云台电机闭环控制
			CAN_CMD_GIMBAL(gimbal_move.motor_gimbal[1].give_current,gimbal_move.motor_gimbal[2].give_current,gimbal_move.motor_gimbal[3].give_current);
			CAN_COMM_XYZ_IMU(0,0,rc_data.rc.ch[0],gimbal_y.IMU_actualangle);
		}
//		if(Mode == VISION_ON)
		
//		{
//			Vision_Task();
//		
//		}
	
 }
 int num_flag = 0,IMU_cnt = 0,start_flag = 0,MS_Count = 0,S_Count = 0;
 extern int lose_control;
 extern float sent_tray;

 void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)//任务调度
{
	
	if (htim == &htim2)
    {	
				MS_Count++;
//			if(lose_control>10)//防止遥控器失联
//				lose_task();
//				lose_control++;
			if(IMU_cnt > 3)
				{
					start_flag = 1;
				}
			else
				{
					CAN_COMM_XYZ_IMU(0,0,0,0);
				}
				
				INS_task();	
			
			if(KEY_MODE == KEY_OFF)
				{
					if(MS_Count % 5 == 0)
					{	
						CANTX_XYZ_IMU(&rc_data,gimbal_y.IMU_actualangle*100.0f);
						CANTX_MODE(rc_data.rc.s[0],rc_data.rc.s[1]);//发送遥控器数据，底盘选择模式
					}
					if(MS_Count % 7 == 2)
						{
								
							mode_select(&rc_data);
								
							CAN_COMM_T(rc_data.rc.ch[4]);
								
							mode_control();
								
							UPPER_COMM_TX();//给上位机发送数据
								
						}
					

					}	//发送遥控器数据
				else
				{
					if(MS_Count % 7 == 0)
						key_control_data();
					if(start_flag == 1)
						{	
							
							CAN_COMM_XYZ_IMU(rc_sent.x_speed,rc_sent.y_speed,rc_sent.yaw.target_angle,gimbal_y.IMU_actualangle*100.0f);//X，Y，Z，IMU_Y;//底盘xyz，imu数据发送
							
							CANTX_MODE(tray_mode.mode_s1,tray_mode.mode_s2);

							
							if(MS_Count % 7 == 2)
							{
								
								gimbal_task();
								
								shoot_task();
								
								CAN_CMD_GIMBAL(gimbal_p.motor_speed_pid[0].out,shoot.motor_gimbal[left_f].give_current,shoot.motor_gimbal[right_f].give_current);

								CAN_COMM_T(sent_tray);
				
								
								
								UPPER_COMM_TX();//给上位机发送数据
								
							}

						}
				}
			
			
				
		
		if(MS_Count % 20 == 0)
			{
				 control_mode_judge();
	
			}
	
		//计数
		if(MS_Count >= 1000)
		{
			MS_Count = 0;
			S_Count++;
			if(start_flag == 0)
				IMU_cnt++;
		}
			
	}
		
}
