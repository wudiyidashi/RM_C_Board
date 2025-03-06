
# RM_C_BOARD

## 介绍
RM_C_Board 是一个基于 RT-Thread 实时操作系统实现的 RM_C 板相关功能的项目。该项目旨在利用 RT-Thread 的强大功能和丰富的组件，为 RM_C 板提供高效、稳定的开发环境，帮助开发者快速搭建各种应用。

## 软件架构
RM_C_Board 的软件架构采用了分层设计，主要分为以下几个层次：

### 硬件抽象层（HAL）
这一层主要负责与硬件设备进行交互，包括各种传感器、执行器等。通过对硬件进行抽象封装，提供统一的接口给上层软件使用，提高了代码的可移植性和可维护性。例如，在 `libraries/STM32F4xx_HAL_Driver` 目录下，包含了 STM32F4 系列芯片的 HAL 驱动库，方便对芯片的各种外设进行操作。

### RT-Thread 内核层
RT-Thread 作为项目的核心，提供了任务管理、内存管理、设备驱动框架、文件系统等基础功能。开发者可以利用 RT-Thread 的这些功能，轻松实现多任务并发处理、资源管理等操作。例如，在 `rt-thread` 目录下，包含了 RT-Thread 的内核代码和各种组件。

### 应用层
应用层基于 RT-Thread 内核和硬件抽象层开发，包含以下核心模块：

#### 主要模块
1. **算法模块** (algorithm/)
   - 提供常用算法的封装，包括 PID 控制、卡尔曼滤波等。
   - **PID 控制器**：
     - 支持多种优化功能，如积分限幅、微分滤波等。
     - 示例代码：
       ```c
       #include "pid.h"

       pid_config_t pid_config = INIT_PID_CONFIG(1.0, 0.1, 0.01, 10.0, 100.0, PID_Integral_Limit);
       pid_obj_t *pid = pid_register(&pid_config);

       float output = pid_calculate(pid, measure_value, setpoint);
       ```
   - **卡尔曼滤波器**：
     - 支持动态调整矩阵维度和数值，适用于多传感器融合场景。
     - 示例代码：
       ```c
       KalmanFilter_t kf;
       Kalman_Filter_Init(&kf, 3, 0, 3);
       Kalman_Filter_Update(&kf);
       ```

2. **外设模块** (modules/)
   - 封装了常用外设的驱动和接口，包括电机、磁力计、IMU 等。
   - **电机模块**：
     - 支持 DJI 智能电机（如 6020、3508、2006）。
     - 示例代码：
       ```c
       dji_motor_object_t motor = {
           .control = custom_motor_control
       };
       ```
   - **磁力计模块**：
     - 提供 IST8310 磁力计的驱动。
     - 示例代码：
       ```c
       mag.mag_init("i2c1");
       float data[3];
       mag.mag_read(data);
       ```
   - **IMU 模块**：
     - 支持 BMI088 IMU，提供角速度和加速度读取功能。
     - 示例代码：
       ```c
       struct imu_ops imu = {
           .imu_init = imu_init,
           .gyro_read = gyro_read,
           .accel_read = accel_read
       };
       ```

3. **任务模块** (task/)
   - 提供常用任务的封装，如底盘控制任务、云台控制任务等。
   - **底盘控制任务**：
     - 支持多种控制模式（速度控制、角度控制等）。
     - 示例代码：
       ```c
       struct chassis_cmd_msg cmd = {
           .vx = 1.0,
           .vy = 0.0,
           .vw = 0.5
       };
       ```

4. **消息模块** (modules/msg/)
   - 实现发布-订阅模式，用于模块间的数据传递。
   - 示例代码：
     ```c
     // 发布者
     publisher_t *pub = pub_register("topic_name", sizeof(data_type));
     pub_push_msg(pub, &data);

     // 订阅者
     subscriber_t *sub = sub_register("topic_name", sizeof(data_type));
     sub_get_msg(sub, &data);
     ```

#### 模块交互
以下是一个典型的模块交互示例：
```c
#include "modules/motor/DJI_motor/dji_motor.h"
#include "modules/spi.h"
#include "algorithm/pid.h"

int main(void)
{
    // 初始化 SPI 设备
    rt_device_t spi_dev = rt_device_find("spi1");

    // 注册 DJI 电机控制回调
    dji_motor_object_t motor = {
        .control = custom_motor_control
    };

    // 初始化 PID 控制器
    pid_config_t pid_config = INIT_PID_CONFIG(1.0, 0.1, 0.01, 10.0, 100.0, PID_Integral_Limit);
    pid_obj_t *pid = pid_register(&pid_config);

    // 控制逻辑
    float measure = 0.0, setpoint = 1.0;
    float output = pid_calculate(pid, measure, setpoint);
}
```
### 贡献指南
当新增模块时，请遵循以下规范：

1. 在 applications/ 下创建独立子目录。
2. 每个模块应包含：
   - 模块头文件（.h）声明公开接口。
   - Kconfig 配置项。
   - README 模块说明文档。
3. 驱动模块必须实现 RT-Thread 设备驱动框架接口。