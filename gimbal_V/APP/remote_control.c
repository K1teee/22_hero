
#include "remote_control.h"
#include "main.h"

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;



static void rc_dataresol(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);

//遥控器控制变量
RC_ctrl_t rc_data;

//接收原始数据，为18个字节，给了36个字节长度，防止DMA传输越界
static uint8_t sbus_rx_buf[2][RX_BUF];

void rc_init(void)
{
    RC_Init(sbus_rx_buf[0], sbus_rx_buf[1], RX_BUF);
}


const RC_ctrl_t *get_remote_control_point(void)
{
    return &rc_data;
}

static void rc_dataresol(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{
    if (sbus_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }
	
    rc_ctrl->rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;        //右摇杆左右（-+），左右平移
    rc_ctrl->rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //右摇杆下上（-+），前进后退
    rc_ctrl->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) |          //左摇杆左右自旋（-+）
                         (sbus_buf[4] << 10)) &0x07ff;
    rc_ctrl->rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //左摇杆下上（-+）
    rc_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);                  //左拨杆
    rc_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                       //右拨杆
    rc_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);                    //鼠标x轴坐标
    rc_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);                    //鼠标y轴坐标
    rc_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                  //鼠标z轴坐标
    rc_ctrl->mouse.press_l = sbus_buf[12];                                  //鼠标左键按下
    rc_ctrl->mouse.press_r = sbus_buf[13];                                  //鼠标右键按下
    rc_ctrl->key.v = sbus_buf[14] | (sbus_buf[15] << 8);                    //键盘v
    rc_ctrl->rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);                 //NULL
	
    
	rc_ctrl->rc.ch[0] -= RC_CH_MIDDLE_VALUE ;//摇杆初始位置设为0；原始值-1024；
    rc_ctrl->rc.ch[1] -= RC_CH_MIDDLE_VALUE ;
    rc_ctrl->rc.ch[2] -= RC_CH_MIDDLE_VALUE ;
    rc_ctrl->rc.ch[3] -= RC_CH_MIDDLE_VALUE ;
    rc_ctrl->rc.ch[4] -= RC_CH_MIDDLE_VALUE ;
}

//串口中断
void USART3_IRQHandler(void)
{
    if(huart3.Instance->SR & UART_FLAG_RXNE)//接收到数据
    {
        __HAL_UART_CLEAR_PEFLAG(&huart3);
		
    }
    else if(USART3->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart3);

        if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
           

           
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = RX_BUF - hdma_usart3_rx.Instance->NDTR;

            
            //重新设定数据长度
            hdma_usart3_rx.Instance->NDTR = RX_BUF;

            //设定缓冲区1
            hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;
            
           
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if(this_time_rx_len == RC_LENGTH)
            {
                rc_dataresol(sbus_rx_buf[0], &rc_data);
            }
        }
        else
        {
            
            
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = RX_BUF - hdma_usart3_rx.Instance->NDTR;

            
            //重新设定数据长度
            hdma_usart3_rx.Instance->NDTR = RX_BUF;

            
            //设定缓冲区0
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);
            
            
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if(this_time_rx_len == RC_LENGTH)
            {
                //处理遥控器数据
                rc_dataresol(sbus_rx_buf[1], &rc_data);
            }
        }
    }

}

