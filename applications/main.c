/*
 * Copyright (c) 2025,北京合鲸科技发展有限公司
 *  All Rights Reserved.
 * 
 * @FilePath     : main.c
 * @Version      : 1.0
 * @Description  : 
 * 
 * @Author       : 岳凯(1585202329@qq.com)
 * @Date         : 2025-01-13 23:02:26
 * 
 * @LastEditors  : 岳凯(1585202329@qq.com)
 * @LastEditTime : 2025-02-09 12:09:56
 */
/*
 * Copyright (c) 2006-2025, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-01-13     RT-Thread    first version
 */

#include <rtthread.h>

#define DBG_TAG "main"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>
#include <vconsole.h>

#include <drv_dwt.h>

int main(void)
{
    // 使用USB作为串口 废弃 必须正常启动才能使用 在出错的时候限制比较大
//    rt_device_t dev = rt_device_find("vcom");
    // 借助库在USB初始化完成后设置为终端串口
//    vconsole_switch(dev);
    // PID计算需要时间戳 必须要初始化dwt
    dwt_init(168);
    int count = 1;

    while (count++)
    {
        rt_thread_delay(1000);

    }

    return RT_EOK;
}
