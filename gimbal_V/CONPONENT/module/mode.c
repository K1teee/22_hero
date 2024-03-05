#include "mode.h"
#include "remote_control.h"



//extern gimbal_y_t gimbal_y;
//extern gimbal_p_t gimbal_p;
//extern gimbal_move_t gimbal_move;

//extern RC_ctrl_t rc_data;
//int num_flag = 0,IMU_cnt = 0,start_flag = 0,MS_Count;
//int speed_g1,speed_g2,speed_g3;
//int mode_s;
MODE_t Mode;

 void mode_select(RC_ctrl_t *rc_data)
 {
	
	 if(rc_data->rc.s[0] == 2)
		Mode = ZERO_FORCE;
	 if(rc_data->rc.s[1] == 3 && rc_data->rc.s[0] != 2)
		Mode = FOLLOW;
	if(rc_data->rc.s[1] == 2 && rc_data->rc.s[0] != 2)
		Mode = TOP_ANGLE;
	if(rc_data->rc.s[1] == 1 && rc_data->rc.s[0] != 2)
		Mode = HANGING;
	

 }
 
 void mode_control(void)
 {
//	if(Mode == TOP_ANGLE)
//			
//	if(Mode == FOLLOW)
		
 }
// void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//	
//	if (htim == &htim2)
//    {	
//			MS_Count++;
//		if(IMU_cnt > 3)
//			{
//				start_flag = 1;
//			}
//		if(start_flag == 1)
//			{	
//				INS_task();
//				CANTX_XYZ_IMU(&rc_data,&gimbal_y,gimbal_y.IMU_actualangle);
//			}
//		if(MS_Count >= 1000)
//			{
//				MS_Count = 0;
//				
//				if(start_flag == 0)
//					IMU_cnt++;
//			}
//		mode_select(&rc_data);
//		CANTX_T_MODE(&rc_data);

//				

//				;
////		gimbal_rc_to_set(&gimbal_move);//遥控器数据传给云台电机
////		gimbal_ctrl_loop(&gimbal_move);//云台电机闭环控制
////		
//			

////       CAN_CMD_GIMBAL(gimbal_move.motor_gimbal[1].give_current,gimbal_move.motor_gimbal[2].give_current,gimbal_move.motor_gimbal[3].give_current);
////      num_flag = mode_select(&rc_data);
////     CAN_COMM_num(num_flag);
////      gimbal_rc_to_set(&gimbal_move);//遥控器数据传给云台电机
////      
////      if(num_flag%2 == 0)//有力模式
////      {
////        gimbal_ctrl_loop(&gimbal_move);//云台电机闭环控制
////        CAN_CMD_GIMBAL(gimbal_move.motor_gimbal[1].give_current,gimbal_move.motor_gimbal[2].give_current,gimbal_move.motor_gimbal[3].give_current);
////      }
////      else
////      {
////        CAN_CMD_GIMBAL(0,0,0);
////      }

//		
//	}
//		
//}