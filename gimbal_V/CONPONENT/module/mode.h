#ifndef __MODE_H
#define __MODE_H
#include "remote_control.h"

typedef enum
{
  
    CHASSIS_FOLLOW_GIMBAL = 0, //云台随动
	TOP_ANGLE,       //小陀螺
	
	ZERO_FORCE,   //无力状态
} MODE_t;

extern void mode_select(RC_ctrl_t *rc_data);

#endif
