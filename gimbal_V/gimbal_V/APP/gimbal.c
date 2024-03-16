#include "gimbal.h"
#include "stdio.h"
#include "remote_control.h"
#include "chassis_task.h"
#include "pid.h"
#include "main.h"
#include "can_comm.h"
#include "CAN_receive.h"
#include "Vision_Task.h"

shoot_task_t rc_shoot;

shoot_t shoot;


gimbal_p_t gimbal_p;

extern RC_GET_t rc_sent;
extern tray_mode_t tray_mode;

extern VISION_MODE vision_mode;

int Gimbal_Precision_Mode = 0;

int Vision_Precision_Mode = 0;

gimbal_move_t gimbal_move;
fp32 PITCH_ch,watch_pitch_set,FRIC_ch;
int flag_fric = 2,num = 1;
fp32 pit_x,pit_y,speed_a;//0.51,0.51,10000.0f

float speed_add = 4.0f,gim_flag;
fp32 PITCH_set = 0,FRIC_set = 0;
//void gimbal_rc_to_ctrl_vector(gimbal_move_t *gimbal_rc_to_ctrl_vector,fp32 *FRIC,fp32 *PITCH_SET)
//{
//	
//	
//	
//	PITCH_ch = gimbal_rc_to_ctrl_vector->gimbal_RC->rc.ch[1];
//	FRIC_ch = gimbal_rc_to_ctrl_vector->gimbal_RC->rc.ch[4];
//	
////	if(vision_mode == VISION_ON)
////    {
////		if(PITCH_ch > 330.0f)//摇杆往上拨是摇杆增
////		{
////			PITCH_set = gimbal_p.IMU_actualangle - speed_add;
////		
////		}
////		if(PITCH_ch < -330.0f)
////		{
////			PITCH_set = gimbal_p.IMU_actualangle + speed_add;
////			
////		}
////		if(PITCH_set-gimbal_p.IMU_actualangle>180)
////			PITCH_set-=360.0f;
////		if(PITCH_set-gimbal_p.IMU_actualangle<-180)
////			PITCH_set+=360.0f;
////		
////	}
////	if(vision_mode == VISION_OFF)
////    {
//			if(PITCH_ch > 330.0f)
//		{
//			
//			PITCH_set +=0.3f ;
//			
//			if(PITCH_set-gimbal_rc_to_ctrl_vector->motor_gimbal[1].gimbal_motor_measure->real_angle>100.0f)
//				PITCH_set = gimbal_rc_to_ctrl_vector->motor_gimbal[1].gimbal_motor_measure->real_angle;
//		}
//		if(PITCH_ch < -330.0f)
//		{

//			PITCH_set -= 0.3f ;
//			
//			if(PITCH_set-gimbal_rc_to_ctrl_vector->motor_gimbal[1].gimbal_motor_measure->real_angle<-100.0f)
//				PITCH_set = gimbal_rc_to_ctrl_vector->motor_gimbal[1].gimbal_motor_measure->real_angle;
//		}
//		gim_flag = 1;
////	}
//	//摩擦轮控制

//    if(flag_fric == 0 && FRIC_ch < -200.0f)
//    {
//        num++;
//		flag_fric = 1;
//    }
//	if(FRIC_ch > -200.0f)
//	{
//		flag_fric = 0;
//	}
//    if(num%2 != 0)
//      {  FRIC_set = 0.0f;
//        
//      }
//    if(num%2 == 0)
//      {  FRIC_set = 6000.0f;
//        
//      }
//	 
//	  
//	  
//     watch_pitch_set = PITCH_set;
//    *PITCH_SET = PITCH_set;
//    *FRIC = FRIC_set;
//	 
//}

float speed_add;
void gimbal_rc_to_ctrl_vector(gimbal_move_t *gimbal_rc_to_ctrl_vector,fp32 *FRIC,fp32 *PITCH_SET)
{
	
	static fp32 PITCH_set,FRIC_set;
	
	PITCH_ch = gimbal_rc_to_ctrl_vector->gimbal_RC->rc.ch[1];
	FRIC_ch = gimbal_rc_to_ctrl_vector->gimbal_RC->rc.ch[4];
	

    if(PITCH_ch > 330.0f)
	{
		
		PITCH_set +=4.0f ;
		
		
		if(PITCH_set>1415.0f)
			PITCH_set = 1410.0f;
//		if(PITCH_set-gimbal_rc_to_ctrl_vector->motor_gimbal[1].gimbal_motor_measure->real_angle>50.0f)
//			PITCH_set = gimbal_rc_to_ctrl_vector->motor_gimbal[1].gimbal_motor_measure->real_angle;
		
	}
	if(PITCH_ch < -330.0f)
	{

		PITCH_set -= 4.0f ;
		if(PITCH_set<6.0f)
			PITCH_set = 6.0f;
		
		
//		if(PITCH_set-gimbal_rc_to_ctrl_vector->motor_gimbal[1].gimbal_motor_measure->real_angle<-50.0f)
//			PITCH_set = gimbal_rc_to_ctrl_vector->motor_gimbal[1].gimbal_motor_measure->real_angle;
	}
	//摩擦轮控制

    if(flag_fric == 0 && FRIC_ch < -200.0f)
    {
        num++;
		flag_fric = 1;
    }
	if(FRIC_ch > -200.0f)
	{
		flag_fric = 0;
	}
    if(num%2 != 0)
      {  FRIC_set = 0.0f;
        
      }
    if(num%2 == 0)
      {  FRIC_set = 6000.0f;
        
      }
	 
	  
	  
     watch_pitch_set = PITCH_set;
    *PITCH_SET = PITCH_set;
    *FRIC = FRIC_set;
	 
}
fp32 deadband_p_anglepid_out(fp32 anglepid_out)
{
    if(fabs(anglepid_out)>DEADBAND_p_ANGLEPID_OUT)
		return 0;
    else
        return anglepid_out;
}

float fric_w,pit_w;
void gimbal_rc_to_set(gimbal_move_t *gimbal_rc_set)
{
    
    fp32 PITCH_SET,FRIC_SET;
    gimbal_rc_to_ctrl_vector(gimbal_rc_set,&FRIC_SET,&PITCH_SET);//遥控器数值传递包含数值映射
	
	fric_w = FRIC_SET;
	pit_w = PITCH_SET;
    gimbal_rc_set->fric_speedset = FRIC_SET;
    gimbal_rc_set->angleset_pitch = PITCH_SET;
	
}

fp32 angle_target,speed_x;
//void gimbal_ctrl_loop(gimbal_move_t *gimbal_ctrl_loop)
//{
//    //YAW轴和pitch轴的角度与速度更新
//    
//    gimbal_ctrl_loop->imu_angle_pitch = gimbal_p.IMU_actualangle;
//    gimbal_ctrl_loop->imu_speed_pitch = gimbal_p.IMU_actualspeed;
//	
//	gimbal_ctrl_loop->angle_pitch = gimbal_ctrl_loop->motor_gimbal[1].gimbal_motor_measure->real_angle;
//    gimbal_ctrl_loop->speed_pitch = gimbal_ctrl_loop->motor_gimbal[1].gimbal_motor_measure->speed_rpm;

//   //pitch轴pid控制
////    if(vision_mode == VISION_ON)
////	{
////	PID_angleloop_calc(&gimbal_ctrl_loop->imu_angle_pid[0], gimbal_ctrl_loop->imu_angle_pitch,pit_w);
////    PID_calc(&gimbal_ctrl_loop->imu_speed_pid[0], gimbal_ctrl_loop->imu_speed_pitch, gimbal_ctrl_loop->imu_angle_pid[0].out);
////	 gimbal_ctrl_loop->motor_gimbal[1].give_current = gimbal_ctrl_loop->imu_speed_pid[0].out;
////	}
//	
////	if(vision_mode == VISION_OFF)
////	{
//	PID_angleloop_calc(&gimbal_ctrl_loop->motor_angle_pid[1], gimbal_ctrl_loop->angle_pitch, pit_w);
////    PID_calc(&gimbal_ctrl_loop->motor_speed_pid[1], gimbal_ctrl_loop->speed_pitch, gimbal_ctrl_loop->motor_angle_pid[1].out);
//	 gimbal_ctrl_loop->motor_gimbal[1].give_current  = gimbal_ctrl_loop->motor_speed_pid[0].out;
////	}
//	
//	
//    //FRIC速度环控制
//    
//	PID_calc(&gimbal_ctrl_loop->motor_speed_pid[2],gimbal_ctrl_loop->motor_gimbal[2].gimbal_motor_measure->speed_rpm, -gimbal_ctrl_loop->fric_speedset);
//    PID_calc(&gimbal_ctrl_loop->motor_speed_pid[3], gimbal_ctrl_loop->motor_gimbal[3].gimbal_motor_measure->speed_rpm, gimbal_ctrl_loop->fric_speedset);
//    
//    //赋值电流值
//    
//	for(int i = 2;i<4;i++)
//    {
//        gimbal_ctrl_loop->motor_gimbal[i].give_current =(int16_t) gimbal_ctrl_loop->motor_speed_pid[i].out;

//    }
//}

void gimbal_ctrl_loop(gimbal_move_t *gimbal_ctrl_loop)
{
    //YAW轴和pitch轴的角度与速度更新
    
    gimbal_ctrl_loop->angle_pitch = gimbal_ctrl_loop->motor_gimbal[1].gimbal_motor_measure->real_angle;
    gimbal_ctrl_loop->speed_pitch = gimbal_ctrl_loop->motor_gimbal[1].gimbal_motor_measure->speed_rpm;

   //pitch轴pid控制
    
	PID_angleloop_calc(&gimbal_ctrl_loop->motor_angle_pid[1], gimbal_ctrl_loop->angle_pitch, gimbal_ctrl_loop->angleset_pitch);
    PID_calc(&gimbal_ctrl_loop->motor_speed_pid[1], gimbal_ctrl_loop->speed_pitch, gimbal_ctrl_loop->motor_angle_pid[1].out);
	
	//限位部分防止pitch部分损坏
	if(gimbal_ctrl_loop->angleset_pitch > 1415.0f)
	{
		gimbal_ctrl_loop->angleset_pitch = 1415.0f;
		gimbal_ctrl_loop->motor_speed_pid[1].out = 0.0f;
	}
	
	if(gimbal_ctrl_loop->angleset_pitch < 6.0f)
	{
		gimbal_ctrl_loop->angleset_pitch = 6.0f;
		gimbal_ctrl_loop->motor_speed_pid[1].out = 0.0f;
	}
	
	
	
    //FRIC速度环控制
    
	PID_calc(&gimbal_ctrl_loop->motor_speed_pid[2],gimbal_ctrl_loop->motor_gimbal[2].gimbal_motor_measure->speed_rpm, -gimbal_ctrl_loop->fric_speedset);
    PID_calc(&gimbal_ctrl_loop->motor_speed_pid[3], gimbal_ctrl_loop->motor_gimbal[3].gimbal_motor_measure->speed_rpm, gimbal_ctrl_loop->fric_speedset);
    
    //赋值电流值
    
	for(int i = 0;i<4;i++)
    {
        gimbal_ctrl_loop->motor_gimbal[i].give_current =(int16_t) gimbal_ctrl_loop->motor_speed_pid[i].out;

    }
}


void gimbal_init(gimbal_move_t *gimbal_init)
{
    
    gimbal_init->gimbal_RC = get_remote_control_point();//获取遥控器数据指针

    //速度环
   
    fp32 PITCH_SPEED_PID[4] = {PITCH_SPEED_KP,PITCH_SPEED_KI,PITCH_SPEED_KD};
	
//	 fp32 IMU_SPEED_PID[4] = {IMU_SPEED_KP,IMU_SPEED_KI,IMU_SPEED_KD};
   
    //角度环
    //编码器
    fp32 PITCH_ANGLE_PID[4] = {PITCH_ANGLE_KP,PITCH_ANGLE_KI,PITCH_ANGLE_KD};
	
	//imu
//	fp32 IMU_ANGLE_PID[4] = {P_IMU_ANGLE_KP,P_IMU_ANGLE_KI,P_IMU_ANGLE_KD};
	
   
    for (int i = 0; i < 4; i++)//获取云台电机数据,1为PITCH，2为SHOOT
    {
        gimbal_init->motor_gimbal[i].gimbal_motor_measure = get_gimbal_motor_measure_point(i);
    }
	//gimbaltask，shoot中的獲取电机指针
	gimbal_p.motor_gimbal[0].gimbal_motor_measure = get_gimbal_motor_measure_point(1);
	shoot.motor_gimbal[0].gimbal_motor_measure = get_gimbal_motor_measure_point(2);
    shoot.motor_gimbal[1].gimbal_motor_measure = get_gimbal_motor_measure_point(3);
    //PITCH速度环
	
   //编码器
	PID_init(&gimbal_init->motor_speed_pid[1],  PITCH_SPEED_PID,0.2f ,PITCH_SPEED_PID_MAX_OUT,PITCH_SPEED_PID_MAX_IOUT);

//	PID_init(&gimbal_init->imu_speed_pid[0],  IMU_SPEED_PID,0.2f ,IMU_SPEED_PID_MAX_OUT,IMU_SPEED_PID_MAX_IOUT);
	 
	//imu，gimbaltask任務中的初始化
	PID_init(&gimbal_p.motor_speed_pid[0],  PITCH_SPEED_PID,0.2f ,PITCH_SPEED_PID_MAX_OUT,PITCH_SPEED_PID_MAX_IOUT);
	
    //PITCH角度环
	
	//编码器
     PID_angleloop_init(&gimbal_init->motor_angle_pid[1],  PITCH_ANGLE_PID, 0.009f,PITCH_ANGLE_PID_MAX_OUT, PITCH_ANGLE_PID_MAX_IOUT);
    
//	//imu
//	 PID_angleloop_init(&gimbal_init->imu_angle_pid[0],  IMU_ANGLE_PID, 0.07f,P_IMU_ANGLE_PID_MAX_OUT, P_IMU_ANGLE_PID_MAX_IOUT);
	//gimbaltask中的imu初始化
	PID_angleloop_init(&gimbal_p.motor_angle_pid[0],  PITCH_ANGLE_PID, 0.07f,PITCH_ANGLE_PID_MAX_OUT, PITCH_ANGLE_PID_MAX_IOUT);

	
	//FRIC1 2的速度pid初始化
	PID_init(&gimbal_init->motor_speed_pid[2],  PITCH_SPEED_PID,0.01f ,FRIC_SPEED_PID_MAX_OUT,PITCH_SPEED_PID_MAX_IOUT);
	PID_init(&gimbal_init->motor_speed_pid[3],  PITCH_SPEED_PID,0.01f ,FRIC_SPEED_PID_MAX_OUT,PITCH_SPEED_PID_MAX_IOUT);
	//shoot任務中的初始化
	PID_init(&shoot.motor_speed_pid[0],  PITCH_SPEED_PID,0.01f ,FRIC_SPEED_PID_MAX_OUT,PITCH_SPEED_PID_MAX_IOUT);
	PID_init(&shoot.motor_speed_pid[1],  PITCH_SPEED_PID,0.01f ,FRIC_SPEED_PID_MAX_OUT,PITCH_SPEED_PID_MAX_IOUT);
	
	
}

//void gimbal_init(gimbal_move_t *gimbal_init)
//{
//    
//    gimbal_init->gimbal_RC = get_remote_control_point();//获取遥控器数据指针

//    //速度环
//   
//    fp32 PITCH_SPEED_PID[4] = {PITCH_SPEED_KP,PITCH_SPEED_KI,PITCH_SPEED_KD};
//    // fp32 SHOOT_SPEED_PID[4] = {0.0f};
//    //角度环
//    
//    fp32 PITCH_ANGLE_PID[4] = {PITCH_ANGLE_KP,PITCH_ANGLE_KI,PITCH_ANGLE_KD};
//    // fp32 SHOOT_ANGLE_PID[4] = {0.0f};

//    for (int i = 0; i < 4; i++)//获取云台电机数据,1为PITCH，2为SHOOT
//    {
//        gimbal_init->motor_gimbal[i].gimbal_motor_measure = get_gimbal_motor_measure_point(i);
//    }
//	

//    
//    //PITCH速度环
//   
//	PID_init(&gimbal_init->motor_speed_pid[1],  PITCH_SPEED_PID,0.01f ,PITCH_SPEED_PID_MAX_OUT,PITCH_SPEED_PID_MAX_IOUT);
//	 
//    //PITCH角度环
//	
//    PID_angleloop_init(&gimbal_init->motor_angle_pid[1],  PITCH_ANGLE_PID, 0.009f,PITCH_ANGLE_PID_MAX_OUT, PITCH_ANGLE_PID_MAX_IOUT);
//    
//	//FRIC1 2的速度pid初始化
//	PID_init(&gimbal_init->motor_speed_pid[2],  PITCH_SPEED_PID,0.01f ,FRIC_SPEED_PID_MAX_OUT,PITCH_SPEED_PID_MAX_IOUT);
//	PID_init(&gimbal_init->motor_speed_pid[3],  PITCH_SPEED_PID,0.01f ,FRIC_SPEED_PID_MAX_OUT,PITCH_SPEED_PID_MAX_IOUT);

//}
float PITCH_set_g,add_speed_p;
void gimbal_task(void)
{
	 
	rc_sent.pitch.target_angle = rc_sent.pitch.target_angle*880.0f;
	 if(rc_sent.pitch.target_angle > 10.0)
	{
		
		
			PITCH_set_g -= 10.0f ;
		if(PITCH_set_g<6.0f)
			PITCH_set_g = 6.0f;
		
	}
	if(rc_sent.pitch.target_angle < -10.0)
	{

		
		
		PITCH_set_g +=10.0f ;
		
		
		if(PITCH_set_g>1415.0f)
			PITCH_set_g = 1410.0f;
	
	}
	
	 gimbal_p.angleset_pitch = PITCH_set_g;
	
//	if(vision_mode == VISION_ON)
//	{
//		gimbal_p.angle_pitch = gimbal_p.IMU_actualangle;
//		gimbal_p.speed_pitch = gimbal_p.IMU_actualspeed;
//		
//		//pitch轴pid控制
//		PID_angleloop_calc(&gimbal_p.imu_angle_pid[0], gimbal_p.angle_pitch, gimbal_p.angleset_pitch);
//		gimbal_p.imu_angle_pid[0].out = deadband_p_anglepid_out(gimbal_p.imu_angle_pid[0].out);
//		PID_calc(&gimbal_p.imu_speed_pid[0], gimbal_p.speed_pitch, gimbal_p.imu_angle_pid[0].out);
//		
//		gimbal_p.motor_gimbal[0].give_current =(int16_t) gimbal_p.imu_speed_pid[0].out;
//	}
//	if(vision_mode == VISION_OFF)
//	{
		gimbal_p.angle_pitch = gimbal_p.motor_gimbal[0].gimbal_motor_measure->real_angle;
		gimbal_p.speed_pitch = gimbal_p.motor_gimbal[0].gimbal_motor_measure->speed_rpm;
		
		//pitch轴pid控制
		
		PID_angleloop_calc(&gimbal_p.motor_angle_pid[0], gimbal_p.angle_pitch, gimbal_p.angleset_pitch);
//		gimbal_p.motor_angle_pid[1].out = deadband_p_anglepid_out(gimbal_p.motor_angle_pid[1].out);
		
		PID_calc(&gimbal_p.motor_speed_pid[0], gimbal_p.speed_pitch, gimbal_p.motor_angle_pid[0].out);
		
			if(gimbal_p.angleset_pitch > 1415.0f)
		{
			gimbal_p.angleset_pitch = 1415.0f;
			gimbal_p.motor_speed_pid[0].out = 0.0f;
		}
		
		if(gimbal_p.angleset_pitch < 6.0f)
		{
			gimbal_p.angleset_pitch = 6.0f;
			gimbal_p.motor_speed_pid[0].out = 0.0f;
		}
		
		
		gimbal_p.motor_gimbal[0].give_current =(int16_t) gimbal_p.motor_speed_pid[0].out;
	
//	}
	//pit电流发送，yaw目标值发送
		

		rc_sent.yaw.target_angle = rc_sent.yaw.target_angle/3.0f*660.0f*0.8f;
		
		
		
	

	
}
float sent_tray;
void shoot_task(void)
{
	
	shoot.fric_speedset = rc_shoot.left_fric.target_speed; 
	
	PID_calc(&shoot.motor_speed_pid[left_f],shoot.motor_gimbal[left_f].gimbal_motor_measure->speed_rpm, -rc_shoot.left_fric.target_speed);//left
	PID_calc(&shoot.motor_speed_pid[right_f], shoot.motor_gimbal[right_f].gimbal_motor_measure->speed_rpm, rc_shoot.left_fric.target_speed);//riget
    
	
	shoot.motor_gimbal[left_f].give_current =(int16_t) shoot.motor_speed_pid[left_f].out;
	shoot.motor_gimbal[right_f].give_current =(int16_t) shoot.motor_speed_pid[right_f].out;
	
	sent_tray = tray_mode.tray*550;
	
	

}



