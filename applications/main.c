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


int main(void)
{
    // 使用USB作为串口
    rt_device_t dev = rt_device_find("vcom");
    // 借助库在USB初始化完成后设置为终端串口
    vconsole_switch(dev);
    int count = 1;

    while (count++)
    {
        rt_thread_delay(1000);
    }

    return RT_EOK;
}
