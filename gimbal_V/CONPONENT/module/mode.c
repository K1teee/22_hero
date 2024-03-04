#include "mode.h"
#include "remote_control.h"

MODE_t Mode;

 void mode_select(RC_ctrl_t *rc_data)
 {
	if(rc_data->rc.s[1] == 3)
		Mode = CHASSIS_FOLLOW_GIMBAL;
	if(rc_data->rc.s[1] == 1)
		Mode = TOP_ANGLE;
	if(rc_data->rc.s[1] == 2)
		Mode = ZERO_FORCE;
 
 }