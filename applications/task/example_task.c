/*
* Change Logs:
* Date            Author          Notes
* 2023-09-05      ChuShicheng     first version
*/

#include "example_task.h"
#include "rm_config.h"
#include "rm_algorithm.h"
#include "rm_module.h"
#include "rm_task.h"
#include "drv_can.h"
#include "dji_motor.h"
#define DBG_TAG   "rm.task"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

static struct chassis_controller_t{
    pid_obj_t *speed_pid;
}chassis_controller;

static struct gimbal_controller_t{
    pid_obj_t *speed_pid;
    pid_obj_t *angle_pid;
}gimbal_controlelr;

static dji_motor_object_t *chassis_motor;
static dji_motor_object_t *gimbal_motor;

// CAN设备信号量
static struct rt_semaphore can1_rx_sem;
static struct rt_semaphore can2_rx_sem;

// CAN设备句柄
static rt_device_t can_chassis_dev = RT_NULL;
static rt_device_t can_gimbal_dev = RT_NULL;

// 新增全局变量，分别用于左右电机目标速度
static rt_int16_t target_speed_left = 0;
static rt_int16_t target_speed_right = 0;

static rt_int16_t chassis_control(dji_motor_measure_t measure){
    static rt_int16_t set = 0;
    // 使用左电机目标速度
    set = pid_calculate(chassis_controller.speed_pid, measure.speed_rpm, target_speed_left);
    return set;
}

static rt_int16_t gimbal_control(dji_motor_measure_t measure){
    static rt_int16_t set = 0;
    // 使用右电机目标速度
    set = pid_calculate(gimbal_controlelr.speed_pid, measure.speed_rpm, target_speed_right);
    return set;
}

/* 接收数据回调函数 */
rt_err_t can1_rx_call(rt_device_t dev, rt_size_t size) {
    // 释放CAN1接收信号量
    rt_sem_release(&can1_rx_sem);
    return RT_EOK;
}

rt_err_t can2_rx_call(rt_device_t dev, rt_size_t size) {
    // 释放CAN2接收信号量
    rt_sem_release(&can2_rx_sem);
    return RT_EOK;
}

#define CAN_BAUD            CAN1MBaud //CAN总线波特率

/* CAN 设备初始化函数 */
static rt_err_t can_init(void)
{
    rt_err_t result = RT_EOK;

    /* 查找CAN设备 */
    can_chassis_dev = rt_device_find(CAN_CHASSIS);
    if (!can_chassis_dev) {
        LOG_E("Cannot find %s device!", CAN_CHASSIS);
        return -RT_ERROR;
    }

    // 初始化CAN1接收信号量
    rt_sem_init(&can1_rx_sem, "can1_rx_sem", 0, RT_IPC_FLAG_FIFO);

     rt_err_t err = rt_device_open(can_chassis_dev, (RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_INT_TX));
     if( err != RT_EOK)
     {
         LOG_E("canfestival open device %s failed, err = %d", CAN_CHASSIS);
         return;
     }
     /* 设置 CAN 通信的波特率*/
     err = rt_device_control(can_chassis_dev, RT_CAN_CMD_SET_BAUD, (void *)CAN_BAUD);
     if( err != RT_EOK)
     {
         LOG_E("canfestival set baud failed, err = %d", err);
         return;
     }
     /* 设置接收回调函数 */
     err = rt_device_set_rx_indicate(can_chassis_dev, can1_rx_call);

     if( err != RT_EOK)
     {
         LOG_E("canfestival set rx indicate failed, err = %d", err);
         return;
     }

     LOG_I("CAN devices initialized successfully!");
     return RT_EOK;
}

// 接收CAN消息的线程入口函数
static void can_receive_thread_entry(void *parameter) {
    struct rt_can_msg rxmsg;
    uint8_t i;

    while (1) {
        // 等待CAN1接收信号量
        if (rt_sem_take(&can1_rx_sem, RT_WAITING_FOREVER) == RT_EOK) {
            // 从CAN1读取消息
            if (rt_device_read(can_chassis_dev, 0, &rxmsg, sizeof(rxmsg)) == sizeof(rxmsg)) {
                // 处理CAN1消息
                // LOG_I("CAN1 Received ID: 0x%X", rxmsg.id);
                // LOG_I("Data: ");
                // for (i = 0; i < rxmsg.ide ? 8 : 8; i++) { // CAN标准帧和扩展帧均为8字节
                //     LOG_I("%02X ", rxmsg.data[i]);
                // }
                // LOG_I("\n");

                // 调用DJI电机接收回调函数
                dji_motor_rx_callback(can_chassis_dev, rxmsg.id, rxmsg.data);
            }
        }
    }
}



// 添加PWM设备句柄
static struct rt_device_pwm *pwm_example_dev;
static struct rt_device_pwm *pwm_rgb_dev;  // RGB LED的PWM设备


// 在文件开头定义
#define PWM_PERIOD 10000  // PWM周期为10us
#define MAX_DUTY   10000  // 最大占空比值
#define MIN_DUTY   0      // 最小占空比值
#define STEP       100    // 步进值

// RGB LED状态控制
static uint32_t rgb_duty[3] = {MAX_DUTY, 0, 0};  // B,G,R通道占空比,初始为蓝色
static uint8_t current_led = 0;  // 当前正在变化的LED (0:B, 1:G, 2:R)
static uint8_t phase = 0;        // 变化阶段 (0-5)

static rt_timer_t rgb_timer; // 定时器句柄

// PWM初始化函数
static void pwm_init(void)
{
    // 查找PWM设备
    pwm_example_dev = (struct rt_device_pwm *)rt_device_find("pwm8");
    pwm_rgb_dev = (struct rt_device_pwm *)rt_device_find("pwm5");
    
    if (pwm_example_dev == RT_NULL || pwm_rgb_dev == RT_NULL)
    {
        LOG_E("Can't find pwm device!");
        return;
    }

    // 舵机PWM初始化(20ms周期)
    rt_pwm_set(pwm_example_dev, 3, 20000000, 780000);
    rt_pwm_enable(pwm_example_dev, 3);

    // RGB LED PWM初始化(RGB对应通道1,2,3)
    for(int i = 1; i <= 3; i++)
    {
        rt_pwm_set(pwm_rgb_dev, i, 10000, 0);  // 周期10us,初始占空比0
        rt_pwm_enable(pwm_rgb_dev, i);
    }
}

// 定时器回调函数，用于控制RGB LED变化
static void rgb_timer_callback(void *parameter)
{
    // 根据不同阶段改变LED亮度
    switch(phase)
    {
        case 0:  // 蓝->青(B->G)
            rgb_duty[1] += STEP;  // G上升
            if(rgb_duty[1] >= MAX_DUTY) phase = 1;
            break;
        
        case 1:  // 青->绿(B->0)
            rgb_duty[0] -= STEP;  // B下降
            if(rgb_duty[0] <= MIN_DUTY) phase = 2;
            break;
        
        case 2:  // 绿->黄(G->R)
            rgb_duty[2] += STEP;  // R上升
            if(rgb_duty[2] >= MAX_DUTY) phase = 3;
            break;
        
        case 3:  // 黄->红(G->0)
            rgb_duty[1] -= STEP;  // G下降
            if(rgb_duty[1] <= MIN_DUTY) phase = 4;
            break;
        
        case 4:  // 红->紫(R->B)
            rgb_duty[0] += STEP;  // B上升
            if(rgb_duty[0] >= MAX_DUTY) phase = 5;
            break;
        
        case 5:  // 紫->蓝(R->0)
            rgb_duty[2] -= STEP;  // R下降
            if(rgb_duty[2] <= MIN_DUTY) phase = 0;
            break;
    }

    // 更新PWM占空比
    for(int i = 0; i < 3; i++)
    {
        rt_pwm_set(pwm_rgb_dev, i+1, PWM_PERIOD, rgb_duty[i]);
    }
}



static rt_int16_t target_speed = 0;  // 声明目标速度变量

/* MSH命令回调函数 */
static void chassis_speed(int argc, char**argv)
{
    if (argc != 2)
    {
        rt_kprintf("Usage: chassis_speed [speed]\n");
        rt_kprintf("Example: chassis_speed 1000\n");
        return;
    }
    
    target_speed = atoi(argv[1]);

    rt_kprintf("Set chassis speed to: %d\n", target_speed);
}
MSH_CMD_EXPORT(chassis_speed, set chassis motor speed);

// 声明本地变量用来接收 chassis_cmd 数据
static struct chassis_cmd_msg local_chassis_cmd;
static subscriber_t *sub_chassis_cmd;

static struct gimbal_cmd_msg local_gim_cmd;
static subscriber_t *sub_gim_cmd;


static void example_init()
{
    /* 初始化 CAN 设备 */
    rt_err_t result;

    // 初始化CAN设备
    result = can_init();
    if (result != RT_EOK) {
        LOG_E("CAN initialization failed!");
        return;
    }
    
    sub_chassis_cmd = sub_register("chassis_cmd", sizeof(struct chassis_cmd_msg));
    if (sub_chassis_cmd == RT_NULL)
    {
        LOG_E("Failed to register chassis_cmd subscriber!");
    }

    sub_gim_cmd = sub_register("gim_cmd", sizeof(struct gimbal_cmd_msg));
    if (sub_gim_cmd == RT_NULL)
    {
        LOG_E("Failed to register gim_cmd subscriber!");
    }

   pid_config_t chassis_speed_config = {
           .Kp = 10, // 4.5
           .Ki = 0,  // 0
           .Kd = 0,  // 0
           .IntegralLimit = 3000,
           .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
           .MaxOut = 12000,
   };
   pid_config_t gimbal_speed_config = {
           .Kp = 50,  // 50
           .Ki = 200, // 200
           .Kd = 0,
           .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
           .IntegralLimit = 3000,
           .MaxOut = 20000,
   };
   chassis_controller.speed_pid = pid_register(&chassis_speed_config);
   gimbal_controlelr.speed_pid = pid_register(&gimbal_speed_config);

   motor_config_t chassis_motor_config = {
           .motor_type = M3508,
           .can_name = CAN_CHASSIS,
           .rx_id = 0x201,
           .controller = &chassis_controller,
   };
   motor_config_t gimbal_motor_config = {
           .motor_type = GM6020,
           .can_name = CAN_CHASSIS,
           .rx_id = 0x206,
           .controller = &gimbal_controlelr,
   };
   chassis_motor = dji_motor_register(&chassis_motor_config, chassis_control);
   gimbal_motor = dji_motor_register(&gimbal_motor_config, gimbal_control);

   dji_motor_enable(chassis_motor);
   dji_motor_enable(gimbal_motor);

    pwm_init();

    // 创建并启动定时器
    rgb_timer = rt_timer_create("rgb_timer",
                                rgb_timer_callback,
                                RT_NULL,
                                RT_TICK_PER_SECOND / 50, // 20ms周期
                                RT_TIMER_FLAG_PERIODIC);

    if (rgb_timer != RT_NULL)
    {
        rt_timer_start(rgb_timer);
    }
    else
    {
        LOG_E("Failed to create RGB timer!");
    }
}

int angle_to_pulse_width(float angle) {
    // 线性映射角度到脉宽(ms)，范围从0.5ms到2.5ms
    float pulse_width_ms = (angle + 135) * (2.0 / 270) + 0.5; // 计算脉宽(ms)
    return (int)(pulse_width_ms * 1000 * 1000); // 转换为纳秒(ns)
}



void example_thread_entry(void *argument)
{
    static float example_dt;
    static float example_start;
    static uint32_t pwm_duty = 780000;
    static uint8_t pwm_dir = 1;
    
    example_init();
    pwm_init();
    LOG_I("Example Task Start");

    // 创建CAN接收线程
    rt_thread_t thread = rt_thread_create("can_rx_thread",
                                        can_receive_thread_entry,
                                        RT_NULL,
                                        1024, 16, 10);
    if (thread != RT_NULL)
        rt_thread_startup(thread);
    else
        LOG_E("Failed to create CAN receive thread!");

        //  rt_pwm_set(pwm_example_dev, 3, 20000000, pwm_duty);
    for (;;)
    {
        // 从 chassis_cmd 话题中获取最新的遥控器转换数据
        if (sub_chassis_cmd != RT_NULL)
        {
            sub_get_msg(sub_chassis_cmd, &local_chassis_cmd);

            // 假设 chassis_cmd.vx 与遥控器左通道对应，vy 与右通道对应
            target_speed_left  = local_chassis_cmd.vx;
            target_speed_right = local_chassis_cmd.vy;
        }

        // 新增：订阅 gim_cmd 数据，映射 yaw 到舵机PWM输出
       if (sub_gim_cmd != RT_NULL)
       {
        sub_get_msg(sub_gim_cmd, &local_gim_cmd);
        // 映射公式：new_pulse = ((pitch + 135) * 2000000) / 270 + 500000
        // 对应范围：pitch -135 ~ 135 映射到脉宽 500000 ~ 2500000 (单位: ns)
        printf("pitch: %f\n", local_gim_cmd.pitch);
        // int new_pulse = ((local_gim_cmd.pitch + 135) * 2000000) / 270 + 500000;
        // printf("new_pulse: %d\n", new_pulse);
        // // 限制脉宽在 [500000, 2500000] 范围内
        // if (new_pulse < 500000)
        // {
        //     new_pulse = 500000;
        // }
        // else if (new_pulse > 2500000)
        // {
        //     new_pulse = 2500000;
        // }
        rt_pwm_set(pwm_example_dev, 3, 20 * 1000 * 1000, (int)angle_to_pulse_width(local_gim_cmd.pitch));
       }
        

        rt_thread_delay(1); // 适时让出 CPU
    }
}
