#include "can_comm.h"
#include "CAN_receive.h"
#include "main.h"
#include "remote_control.h"
#include "gimbal.h"

extern RC_ctrl_t rc_data;

extern CAN_HandleTypeDef hcan2;
extern CAN_HandleTypeDef hcan1;
chassis_tx_t chassis_tx;

static CAN_TxHeaderTypeDef  XYZ_tx_message;
static CAN_TxHeaderTypeDef  TY_tx_message;
static CAN_TxHeaderTypeDef  UNLOAD_tx_message;
static CAN_TxHeaderTypeDef  INABILITY_tx_message;
static CAN_TxHeaderTypeDef  NUM_tx_message;
static CAN_TxHeaderTypeDef  IMU_tx_message;
static CAN_TxHeaderTypeDef  MODE_tx_message;
static CAN_TxHeaderTypeDef  YAW_TAARGET_tx_message;


static uint8_t  XYZ_can_send_data[8];
static uint8_t  TY_can_send_data[8];
static uint8_t  IMU_can_send_data[8];
static uint8_t  MODE_can_send_data[8];
static uint8_t  YAW_TARGET_can_send_data[8];




void CAN_COMM_XYZ_IMU(int16_t X,int16_t Y,int16_t Z,int16_t IMU_Y)//X，Y，Z，IMU_Y
{
    uint32_t send_mail_box;
    XYZ_tx_message.StdId = 0x001;
    XYZ_tx_message.IDE = CAN_ID_STD;
    XYZ_tx_message.RTR = CAN_RTR_DATA;
    XYZ_tx_message.DLC = 0x08;
    XYZ_can_send_data[0] = X >> 8;
    XYZ_can_send_data[1] = X;
    XYZ_can_send_data[2] = Y >> 8;
    XYZ_can_send_data[3] = Y;
    XYZ_can_send_data[4] = Z >> 8;
    XYZ_can_send_data[5] = Z;
    XYZ_can_send_data[6] = IMU_Y >> 8;
    XYZ_can_send_data[7] = IMU_Y;
	
    HAL_CAN_AddTxMessage(&hcan2, &XYZ_tx_message, XYZ_can_send_data, &send_mail_box);
}

void CAN_COMM_T_MODE(int16_t TRAY,int16_t s1,int16_t s2)//拨弹盘，模式选择
{
    uint32_t send_mail_box;
    TY_tx_message.StdId = 0x002;
    TY_tx_message.IDE = CAN_ID_STD;
    TY_tx_message.RTR = CAN_RTR_DATA;
    TY_tx_message.DLC = 0x08;
    TY_can_send_data[0] = TRAY >> 8;
    TY_can_send_data[1] = TRAY;
    TY_can_send_data[2] = s1;
    TY_can_send_data[3] = s2;
    TY_can_send_data[4] = 0;
    TY_can_send_data[5] = 0;
    TY_can_send_data[6] = 0;
    TY_can_send_data[7] = 0;
	
    HAL_CAN_AddTxMessage(&hcan2, &TY_tx_message, TY_can_send_data, &send_mail_box);

}




int watch_can_comm = 2;


int see_imu_1,see_imu_2;
//void CANTX_IMU_UPPER(int16_t YAW,int16_t PIT)//上位机通信
//{
//    uint32_t send_mail_box;
//    NUM_tx_message.StdId = 0x011;
//    NUM_tx_message.IDE = CAN_ID_STD;
//    NUM_tx_message.RTR = CAN_RTR_DATA;
//    NUM_tx_message.DLC = 0x08;
//	NUM_tx_message.TransmitGlobalTime=DISABLE;
//    IMU_can_send_data[0] = YAW << 8;
//    IMU_can_send_data[1] = YAW;
//    IMU_can_send_data[2] = PIT <<8;
//    IMU_can_send_data[3] = PIT;
//    IMU_can_send_data[4] = 0;
//    IMU_can_send_data[5] = 0;
//    IMU_can_send_data[6] = 0;
//    IMU_can_send_data[7] = 0;
//	
//	see_imu_1 = IMU_can_send_data[0];
//	see_imu_2 = IMU_can_send_data[1];
//    watch_can_comm = HAL_CAN_AddTxMessage(&hcan1, &IMU_tx_message, IMU_can_send_data, &send_mail_box);

//}



fp32 x,y,z;
int16_t num_IMU ;
void CANTX_XYZ_IMU(RC_ctrl_t *rc_data,gimbal_y_t *gimbal_y,int16_t IMU_Y)
{
	
	
	x = -rc_data->rc.ch[3];
	y = -rc_data->rc.ch[2];
	z = rc_data->rc.ch[0];

	num_IMU = (int16_t)IMU_Y;
	CAN_COMM_XYZ_IMU(x,y,z,num_IMU);
}

fp32 yaw,tray,s1,s2;

void CANTX_T_MODE(RC_ctrl_t *rc_data)
{
	
	
	
	tray = rc_data->rc.ch[4];
	s1 = rc_data->rc.s[0];
	s2 = rc_data->rc.s[1];
	CAN_COMM_T_MODE(tray,s1,s2);
}


void CANTX_YAW_TARGET(int16_t YAW_TARGET)
{
	 uint32_t send_mail_box;
    YAW_TAARGET_tx_message.StdId = 0x003;
    YAW_TAARGET_tx_message.IDE = CAN_ID_STD;
    YAW_TAARGET_tx_message.RTR = CAN_RTR_DATA;
    YAW_TAARGET_tx_message.DLC = 0x08;
    YAW_TARGET_can_send_data[0] = YAW_TARGET >> 8;
    YAW_TARGET_can_send_data[1] = YAW_TARGET;
    YAW_TARGET_can_send_data[2] = 0;
    YAW_TARGET_can_send_data[3] = 0;
    YAW_TARGET_can_send_data[4] = 0;
    YAW_TARGET_can_send_data[5] = 0;
    YAW_TARGET_can_send_data[6] = 0;
    YAW_TARGET_can_send_data[7] = 0;
	
    HAL_CAN_AddTxMessage(&hcan2, &YAW_TAARGET_tx_message, YAW_TARGET_can_send_data, &send_mail_box);



}


