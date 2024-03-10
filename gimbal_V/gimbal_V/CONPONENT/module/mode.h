#ifndef __MODE_H
#define __MODE_H
#include "remote_control.h"

typedef enum
{
  
    HANGING = 0, //��̨�涯
	FOLLOW,       //С����
	TOP_ANGLE,   //����״̬
	ZERO_FORCE,
	
} MODE_t;


typedef enum
{
	UPPER_ON,
    UPPER_OFF,
} V_MODE_t;

extern void mode_select(RC_ctrl_t *rc_data);

#endif
