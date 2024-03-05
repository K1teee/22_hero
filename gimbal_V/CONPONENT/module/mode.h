#ifndef __MODE_H
#define __MODE_H
#include "remote_control.h"

typedef enum
{
  
    HANGING = 0, //云台随动
	FOLLOW,       //小陀螺
	TOP_ANGLE,   //无力状态
	ZERO_FORCE,
} MODE_t;

extern void mode_select(RC_ctrl_t *rc_data);

#endif
