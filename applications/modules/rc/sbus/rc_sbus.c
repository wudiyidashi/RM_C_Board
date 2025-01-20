 /*
 * Change Logs:
 * Date            Author          Notes
 * 2023-09-24      ChuShicheng     first version
 *                 ZhengWanshun
 *                 YangShuo
 *                 ChenSihan
 */
#include "rc_sbus.h"
#include "rm_config.h"
#include <stm32f4xx.h>

#define DBG_TAG           "rc.sbus"
#define DBG_LVL DBG_INFO
#include <rtdevice.h>
#include <rtdbg.h>
static rt_device_t serial = RT_NULL;

#define NOW 0
#define LAST 1
#define abs(x) ((x > 0) ? x : -x)

/* C板预留的遥控器接口（具备取反电路）为 uart3 */
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;

//接收原始数据，为18个字节，给了36个字节长度，防止DMA传输越界
static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];
static RC_ctrl_t rc_ctrl[2];   // [0]:当前数据NOW,[1]:上一次的数据LAST
// TODO: 目前遥控器发送端关闭并不会检测为丢失，只有接收端异常才会判断为离线，
//       后续需要修改判断条件，预期效果是发送端关闭后判断为离线
static rt_timer_t rc_timer;  // 定时器，用于判断遥控器是否在线

/**
 * @brief 遥控器sbus数据解析
 *
 * @param rc_obj 指向sbus_rc实例的指针
 */
static rt_err_t sbus_rc_decode(uint8_t *buff){

   
    /* 通道值解析 */
    // 范围 -660~660
    // 顺序 0: 右侧左右 1: 右侧上下 2: 左侧左右 3: 左侧上下
    rc_ctrl[NOW].rc.ch[0] = (buff[0] | (buff[1] << 8)) & 0x07FF;                 //!< Channel 0
    rc_ctrl[NOW].rc.ch[1] = ((buff[1] >> 3) | (buff[2] << 5)) & 0x07FF;         //!< Channel 1
    rc_ctrl[NOW].rc.ch[2] = ((buff[2] >> 6) | (buff[3] << 2) | 
                            (buff[4] << 10)) & 0x07FF;                            //!< Channel 2
    rc_ctrl[NOW].rc.ch[3] = ((buff[4] >> 1) | (buff[5] << 7)) & 0x07FF;         //!< Channel 3

    /* 拨杆值解析 */
    rc_ctrl[NOW].rc.s[0] = ((buff[5] >> 4) & 0x0003);                           //!< Switch left
    rc_ctrl[NOW].rc.s[1] = ((buff[5] >> 4) & 0x000C) >> 2;                      //!< Switch right

    /* 鼠标数据解析 */
    rc_ctrl[NOW].mouse.x = buff[6] | (buff[7] << 8);                            //!< Mouse X axis
    rc_ctrl[NOW].mouse.y = buff[8] | (buff[9] << 8);                            //!< Mouse Y axis
    rc_ctrl[NOW].mouse.z = buff[10] | (buff[11] << 8);                          //!< Mouse Z axis
    rc_ctrl[NOW].mouse.press_l = buff[12];                                       //!< Mouse Left Press
    rc_ctrl[NOW].mouse.press_r = buff[13];                                       //!< Mouse Right Press

    /* 键盘数据解析 */
    rc_ctrl[NOW].key.v = buff[14] | (buff[15] << 8);                            //!< KeyBoard value

    /* 通道值偏移处理 */
    for(uint8_t i = 0; i < 4; i++)
    {
        rc_ctrl[NOW].rc.ch[i] -= 1024;
        if(rc_ctrl[NOW].rc.ch[i] <= 10 && rc_ctrl[NOW].rc.ch[i] >= -10) 
            rc_ctrl[NOW].rc.ch[i] = 0;
    }

    /* 调试输出 */
    rt_kprintf("ch:%d %d %d %d\n", 
               rc_ctrl[NOW].rc.ch[0], rc_ctrl[NOW].rc.ch[1],
               rc_ctrl[NOW].rc.ch[2], rc_ctrl[NOW].rc.ch[3]);
    rt_kprintf("sw:%d %d\n", rc_ctrl[NOW].rc.s[0], rc_ctrl[NOW].rc.s[1]);

    rc_ctrl[LAST] = rc_ctrl[NOW];
     return RT_EOK;
}

/**
 * @brief 遥控器定时器超时回调函数
 */
static void rc_lost_callback(void *paramete)
{
    LOG_W("Sbus RC lost!");
}

/**
 * @brief 串口 DMA 双缓冲初始化
 * @param rx1_buf 缓冲区1
 * @param rx2_buf 缓冲区2
 * @param dma_buf_num DMA缓冲区大小
 */
static void rc_doub_dma_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
    //使能DMA串口接收
    SET_BIT(huart3.Instance->CR3, USART_CR3_DMAR);

    //使能空闲中断
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);

    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart3_rx);
    while(hdma_usart3_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart3_rx);
    }

    hdma_usart3_rx.Instance->PAR = (uint32_t) & (USART3->DR);
    //内存缓冲区1
    hdma_usart3_rx.Instance->M0AR = (uint32_t)(rx1_buf);
    //内存缓冲区2
    hdma_usart3_rx.Instance->M1AR = (uint32_t)(rx2_buf);
    //数据长度
    hdma_usart3_rx.Instance->NDTR = dma_buf_num;
    //使能双缓冲区
    SET_BIT(hdma_usart3_rx.Instance->CR, DMA_SxCR_DBM);

    //使能DMA
    __HAL_DMA_ENABLE(&hdma_usart3_rx);
}

//void USART3_IRQHandler(void)
//{
//    if(huart3.Instance->SR & UART_FLAG_RXNE)
//    {
//        __HAL_UART_CLEAR_PEFLAG(&huart3);
//    }
//    else if(USART3->SR & UART_FLAG_IDLE)
//    {
//        static uint16_t this_time_rx_len = 0;
//
//        __HAL_UART_CLEAR_PEFLAG(&huart3);
//
//        if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET)
//        {
//            /* Current memory buffer used is Memory 0 */
//            //失效DMA
//            __HAL_DMA_DISABLE(&hdma_usart3_rx);
//
//            //get receive data length, length = set_data_length - remain_length
//            //获取接收数据长度,长度 = 设定长度 - 剩余长度
//            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;
//
//            //重新设定数据长度
//            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;
//
//            //设定缓冲区1
//            hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;
//
//            //使能DMA
//            __HAL_DMA_ENABLE(&hdma_usart3_rx);
//
//            if(this_time_rx_len == SBUS_FRAME_SIZE)
//            {
//                //处理遥控器数据
//                sbus_rc_decode(sbus_rx_buf[0]);
//                rt_timer_start(rc_timer);
//            }
//        }
//        else
//        {
//            /* Current memory buffer used is Memory 1 */
//            //失效DMA
//            __HAL_DMA_DISABLE(&hdma_usart3_rx);
//
//            //get receive data length, length = set_data_length - remain_length
//            //获取接收数据长度,长度 = 设定长度 - 剩余长度
//            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;
//
//            //重新设定数据长度
//            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;
//
//            //设定缓冲区0
//            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);
//
//            //使能DMA
//            __HAL_DMA_ENABLE(&hdma_usart3_rx);
//
//            if(this_time_rx_len == SBUS_FRAME_SIZE)
//            {
//                //处理遥控器数据
//                sbus_rc_decode(sbus_rx_buf[1]);
//                rt_timer_start(rc_timer);
//            }
//        }
//    }
//}

/* 串口接收回调函数 */
static rt_size_t rx_cb_func(rt_device_t dev, rt_size_t size)
{
    static rt_uint8_t temp_buf[SBUS_RX_BUF_NUM];  // 临时缓冲区
    static rt_uint16_t buf_index = 0;             // 缓冲区索引

    /* 从串口读取数据到临时缓冲区 */
    rt_uint8_t buf[SBUS_RX_BUF_NUM];
    rt_device_read(dev, 0, buf, size);
    
    /* 将数据存入临时缓冲区 */
    for(rt_uint16_t i = 0; i < size && buf_index < SBUS_FRAME_SIZE; i++)
    {
        temp_buf[buf_index++] = buf[i];
    }

    /* 当积累了足够的数据时进行处理 */
    if(buf_index >= SBUS_FRAME_SIZE)
    {
        /* 16 进制打印数据 */
        rt_kprintf("Received data: ");
        for(rt_uint16_t i = 0; i < SBUS_FRAME_SIZE; i++)
        {
            rt_kprintf("%02X ", temp_buf[i]);
        }
        rt_kprintf("\n");

        /* 处理完整的一帧数据 */
        sbus_rc_decode(temp_buf);
        
        /* 重置缓冲区索引 */
        buf_index = 0;
    }
    return size;
}

/**
 * @brief 初始化sbus_rc
 *
 * @return rc_obj_t* 指向NOW和LAST两次数据的数组起始地址
 */
RC_ctrl_t *sbus_rc_init(void)
{
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;  // 获取默认配置

    /* 查找并打开 uart3 设备，使用 DMA 或中断模式 */
    serial = rt_device_find("uart3");
    if (!serial)
    {
        LOG_E("Can not find uart3 device.");
        return RT_NULL;
    }

    /* 配置串口参数 */
    config.baud_rate = 100000;        // 波特率
    config.data_bits = DATA_BITS_9;   // 9位数据位
    config.stop_bits = STOP_BITS_2;   // 2位停止位
    config.parity = PARITY_EVEN;      // 偶校验
    
    /* 应用配置 */
    if (rt_device_control(serial, RT_DEVICE_CTRL_CONFIG, &config) != RT_EOK)
    {
        LOG_E("UART3 config failed.");
        return RT_NULL;
    }

    /* 以 DMA RX 模式打开串口 */
    if (rt_device_open(serial, RT_DEVICE_FLAG_DMA_RX) != RT_EOK)
    {
        LOG_E("Open uart3 failed.");
        return RT_NULL;
    }

    /* 注册接收回调函数 */
    rt_device_set_rx_indicate(serial, rx_cb_func);

    // /* DMA controller clock enable */
    // __HAL_RCC_DMA1_CLK_ENABLE();
    // /* DMA1_Stream1_IRQn interrupt configuration */
    // HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
    // HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

    // huart3.Instance = USART3;
    // huart3.Init.BaudRate = 100000;
    // huart3.Init.WordLength = UART_WORDLENGTH_9B;
    // huart3.Init.StopBits = UART_STOPBITS_2;
    // huart3.Init.Parity = UART_PARITY_EVEN;
    // huart3.Init.Mode = UART_MODE_TX_RX;
    // huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    // huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    // HAL_UART_Init(&huart3);

    // rc_doub_dma_init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);

    // // 遥控器离线检测定时器相关
    // rc_timer = rt_timer_create("rc_sbus",
    //                          rc_lost_callback,
    //                          RT_NULL, 20,
    //                          RT_TIMER_FLAG_PERIODIC);
    // rt_timer_start(rc_timer);

    return rc_ctrl;
}

