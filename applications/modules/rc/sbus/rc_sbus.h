 /*
 * Change Logs:
 * Date            Author          Notes
 * 2023-09-24      ChuShicheng     first version
 *                 ZhengWanshun
 *                 Yangshuo
 *                 ChenSihan
 */
#ifndef _RC_SBUS_H
#define _RC_SBUS_H

#include <rtthread.h>
#include "rm_config.h"

#define SBUS_RX_BUF_NUM 36u
#define SBUS_FRAME_SIZE 18u

/**
  * @brief 遥控器拨杆值
  */
enum {
    RC_UP = RC_UP_VALUE  ,
    RC_MI = RC_MID_VALUE ,
    RC_DN = RC_DN_VALUE  ,
};

typedef  struct
{
     struct
    {
        int16_t ch[4];
        char s[2];
    }  __attribute__((__packed__)) rc;
     struct
    {
        int16_t x;
        int16_t y;
        int16_t z;
        uint8_t press_l;
        uint8_t press_r;
    }  __attribute__((__packed__))mouse;
     struct
    {
        uint16_t v;
    }  __attribute__((__packed__))key;

}  __attribute__((__packed__))RC_ctrl_t;


typedef struct
{   /* 摇杆最终值为：-784~783 */
    int16_t ch1;   //右侧左右
    int16_t ch2;   //右侧上下
    int16_t ch3;   //左侧上下
    int16_t ch4;   //左侧左右
    /* FS-i6x旋钮为线性，左右最终值为：-784~783 */
    int16_t ch5;   //左侧线性旋钮
    int16_t ch6;   //右侧线性旋钮
    /* 遥控器的拨杆数据，上(中)下最终值分别为：240、（0）、15 */
    uint8_t sw1;   //SWA，二档
    uint8_t sw2;   //SWB，二档
    uint8_t sw3;   //SWC，三档
    uint8_t sw4;   //SWD，二档
} rc_obj_t;

/**
 * @brief 初始化sbus_rc
 *
 * @return rc_obj_t* 指向NOW和LAST两次数据的数组起始地址
 */
RC_ctrl_t *sbus_rc_init(void);

#endif /* _RC_SBUS_H */
