
#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H
#include "struct_typedef.h"
#include "bsp_rc.h"

#define RX_BUF 36u

#define RC_LENGTH 18u

#define RC_CH_MIDDLE_VALUE     ((uint16_t)1024)



typedef __packed struct
{
        __packed struct
        {
                int16_t ch[5];
                char s[2];
        } rc;
        __packed struct
        {
                int16_t x;
                int16_t y;
                int16_t z;
                uint8_t press_l;
                uint8_t press_r;
        } mouse;
        __packed struct
        {
                uint16_t v;
        } key;

} RC_ctrl_t;


//遥控器数据初始化
extern void rc_init(void);

//获取遥控器数据的指针
extern const RC_ctrl_t *get_remote_control_point(void);
 
#endif
