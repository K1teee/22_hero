#ifndef __MODE_H
#define __MODE_H
#include "remote_control.h"

typedef enum
{
  
    CHASSIS_FOLLOW_GIMBAL = 0, //��̨�涯
	TOP_ANGLE,       //С����
	
	ZERO_FORCE,   //����״̬
} MODE_t;

extern void mode_select(RC_ctrl_t *rc_data);

#endif
