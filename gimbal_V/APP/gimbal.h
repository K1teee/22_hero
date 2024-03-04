#ifndef __GIMBAL_H
#define __GIMBAL_H

#include "struct_typedef.h"
#include "CAN_receive.h"
#include "pid.h"
#include "remote_control.h"


//遥控器通道值换算到角度的比例
#define CH1_TO_YAW_RATIO 0.54545454f
#define CH2_TO_PITCH_RATIO 2.46515151f

//角度环输出死区
#define DEADBAND_ANGLEPID_OUT 30.0f
//pitch的角度环输出死区
#define DEADBAND_p_ANGLEPID_OUT 100.0f

//速度环kpkikd
#define PITCH_SPEED_KP 10.0f
#define PITCH_SPEED_KI 0.0f
#define PITCH_SPEED_KD 20.0f
#define PITCH_SPEED_PID_MAX_OUT 12000.0f
#define PITCH_SPEED_PID_MAX_IOUT 6000.0f

#define FRIC_SPEED_PID_MAX_OUT 16000.0f

#define YAW_SPEED_KP 50.0f
#define YAW_SPEED_KI 2.8f
#define YAW_SPEED_KD 28.0f
#define YAW_SPEED_PID_MAX_OUT 25000.0f
#define YAW_SPEED_PID_MAX_IOUT 10000.0f

//角度环kpkikd
#define PITCH_ANGLE_KP 4.0f
#define PITCH_ANGLE_KI 0.0f
#define PITCH_ANGLE_KD 0.0f
#define PITCH_ANGLE_PID_MAX_IOUT 1500.0f
#define PITCH_ANGLE_PID_MAX_OUT 1500.0f

#define YAW_ANGLE_KP 0.6f
#define YAW_ANGLE_KI 0.1f
#define YAW_ANGLE_KD 0.35f
#define YAW_ANGLE_PID_MAX_IOUT 0.2f
#define YAW_ANGLE_PID_MAX_OUT 60.0f

typedef struct
{
  const motor_measure_t *gimbal_motor_measure;//云台电机反馈参数
  int16_t give_current;
  int16_t give_voltage;
} gimbal_motor_t;

typedef struct
{
  const RC_ctrl_t *gimbal_RC;               //云台使用的遥控器指针
  
  gimbal_motor_t motor_gimbal[5];          //云台电机数据
  
	pid_type_def motor_speed_pid[4];             //云台电机速度pid
 
	pid_type_def motor_angle_pid[4];          //云台电机角度pid

  fp32 angleset_yaw;
  fp32 angleset_pitch;
  fp32 angleset_shoot;

  fp32 angle_yaw;
  fp32 angle_pitch;
  fp32 angle_shoot;
  
  fp32 speed_yaw;
  fp32 speed_pitch;
  fp32 speed_shoot;
	
  fp32 offset_P;//pitch零点
  fp32 offset_y;//yaw零点

  fp32 fric_speedset;
} gimbal_move_t;


typedef struct
{
	fp32 IMU_actualangle;
	pid_type_def imu_pid[4];

}gimbal_y_t;

typedef struct
{
	fp32 IMU_actualangle;
	pid_type_def imu_pid[4];
	
	
}gimbal_p_t;

extern void gimbal_init(gimbal_move_t *gimbal_init);

extern void gimbal_ctrl_loop(gimbal_move_t *gimbal_crtl_loop);
extern void gimbal_rc_to_set(gimbal_move_t *gimbal_rc_set);

extern gimbal_move_t gimbal_move;

extern void gimbal_rc_to_ctrl_vector(gimbal_move_t *gimbal_rc_to_ctrl_vector,fp32 *FRIC,fp32 *PITCH_SET);

extern fp32 deadband_anglepid_out(fp32 anglepid_out);


extern void IMU_trans(void);

#endif
