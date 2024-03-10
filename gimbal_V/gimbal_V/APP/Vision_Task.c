#include "Vision_Task.h"
#include "gimbal.h"
#include "can_comm.h"

VISION_GET_t vision_sent;
extern gimbal_move_t gimbal_move;
extern chassis_tx_t chassis_tx;
VISION_MODE vision_mode;

void Vision_Task()
{
	chassis_tx.yaw_target = vision_sent.yaw.target_angle;
	gimbal_move.angleset_pitch = vision_sent.pitch.target_angle;

	//	gimbal_move.fric_speedset = 6000.0f;
	
	chassis_tx.x_target =  vision_sent.chassis.target_x;
	chassis_tx.y_target =  vision_sent.chassis.target_y;
	chassis_tx.z_target =  vision_sent.chassis.target_z;
	
	gimbal_ctrl_loop(&gimbal_move);//��̨����ջ�����
	CAN_CMD_GIMBAL(gimbal_move.motor_gimbal[1].give_current,gimbal_move.motor_gimbal[2].give_current,gimbal_move.motor_gimbal[3].give_current);
	CANTX_YAW_TARGET(chassis_tx.yaw_target);		
}



