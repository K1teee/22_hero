#include "mode.h"
#include "remote_control.h"
#include "gimbal.h"
#include "can_comm.h"
#include "bsp_imu.h"
#include "usbd_cdc_if.h"
#include "Vision_Task.h"


extern MODE_t Mode;
extern gimbal_y_t gimbal_y;
extern gimbal_p_t gimbal_p;
extern gimbal_move_t gimbal_move;
extern TIM_HandleTypeDef htim2;

extern VISION_GET_t vision_sent;

extern RC_ctrl_t rc_data;
int num_flag = 0,IMU_cnt = 0,start_flag = 0,MS_Count;
int speed_g1,speed_g2,speed_g3;
int mode_s;

MODE_t Mode;
V_MODE_t V_Mode;

 void mode_select(RC_ctrl_t *rc_data)
 {
	
	 if(rc_data->rc.s[0] == 2 )
	 {
		 Mode = ZERO_FORCE;
		V_Mode = UPPER_OFF;
	 }
	 if(rc_data->rc.s[0] == 3 && rc_data->rc.s[1] == 3)
	 {
		 Mode = FOLLOW;
		V_Mode = UPPER_OFF;
	 }
	if(rc_data->rc.s[0] == 2 && rc_data->rc.s[1] == 3)//左0右1
	{
		Mode = TOP_ANGLE;
		V_Mode = UPPER_OFF;
	}
	if(rc_data->rc.s[0] == 1 && rc_data->rc.s[1] == 3)
	{
		Mode = HANGING;
		V_Mode = UPPER_OFF;
	}
	if(rc_data->rc.s[1] == 1)
	{
		V_Mode = UPPER_ON;
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
		if(V_Mode == UPPER_ON)
		{
			gimbal_move.angleset_pitch = vision_sent.pitch.target_angle;
			gimbal_ctrl_loop(&gimbal_move);//云台电机闭环控制
			CAN_CMD_GIMBAL(gimbal_move.motor_gimbal[1].give_current,gimbal_move.motor_gimbal[2].give_current,gimbal_move.motor_gimbal[3].give_current);
			
			CANTX_YAW_TARGET((int16_t)vision_sent.yaw.target_angle);
			
		}
	
 }
 
 void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)//任务调度
{
	
	if (htim == &htim2)
    {	
			MS_Count++;
		if(IMU_cnt > 3)
			{
				start_flag = 1;
			}
		if(start_flag == 1)
			{	
				INS_task();
				CANTX_XYZ_IMU(&rc_data,&gimbal_y,gimbal_y.IMU_actualangle);
			}
		if(MS_Count >= 1000)
			{
				MS_Count = 0;
				
				if(start_flag == 0)
					IMU_cnt++;
			}
		
			
			
			
//		mode_select(&rc_data);
//		CANTX_T_MODE(&rc_data);//发送遥控器数据，底盘选择模式
//		mode_control();
//		UPPER_COMM_TX();//给上位机发送数据
		


			
	}
		
}
