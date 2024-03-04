#ifndef __CAN_COMM_H
#define __CAN_COMM_H
#include "struct_typedef.h"
#include "remote_control.h"
#include "gimbal.h"

extern void CAN_COMM_XYZ(int16_t X,int16_t Y,int16_t Z,int16_t IMU_Y);
extern void CAN_COMM_T_MODE(int16_t TRAY,int16_t MODE);

extern void CANTX_IMU_UPPER(int16_t YAW,int16_t PIT);


extern void CANTX_XYZ_IMU(RC_ctrl_t *rc_data,gimbal_y_t *gimbal_y,int16_t IMU_Y);
extern void CANTX_T_MODE(RC_ctrl_t *rc_data,int16_t MODE);

extern void CAN_COMM_UNLOAD(int16_t unload,int16_t num);
extern void CAN_C_TO_105_UNLOAD(RC_ctrl_t *rc_data,int16_t num);
//extern int mode_select(RC_ctrl_t *rc_data);

extern void CAN_C_TO_105_INA(RC_ctrl_t *rc_data);
extern void CAN_COMM_INABILITY(int16_t INA);

extern void CAN_COMM_num(int16_t num);




#endif
