/**
 * @Copyright (c) 2025,北京合鲸科技发展有限公司
 * @ All Rights Reserved.
 * @
 * @FilePath     : spi.h
 * @Version      : 1.0
 * @Description  : 
 * @
 * @Author       : 岳凯(1585202329@qq.com)
 * @Date         : 2025-01-18 23:52:36
 * @
 * @LastEditors  : 岳凯(1585202329@qq.com)
 * @LastEditTime : 2025-01-19 00:00:45
 */
/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-01-18     msi       the first version
 */
#ifndef APPLICATIONS_MODULES_SPI_H_
#define APPLICATIONS_MODULES_SPI_H_

#include <rtthread.h>
#include <drv_spi.h>
#define SPI_DIR_READ  0x80
#define SPI_DIR_WRITE 0x00
/**
 * This function write a 8 bit reg.
 *
 * @param device the SPI device attached to SPI bus
 *
 * @param reg Register address
 *
 * @param val The value to be written
 *
 * @return RT_EOK if write successfully.
 */
rt_inline rt_err_t spi_write_reg8(rt_device_t spi_device, uint8_t reg, uint8_t val)
{
    uint8_t buffer[2];
    rt_size_t w_byte;
    buffer[0] = SPI_DIR_WRITE | reg;
    buffer[1] = val;
    w_byte = rt_spi_transfer((struct rt_spi_device*)spi_device, buffer, NULL, 2);
    return (w_byte == 2) ? RT_EOK : RT_ERROR;
}
/**
 * This function read a 8 bit reg.
 *
 * @param device the SPI device attached to SPI bus
 *
 * @param reg Register address
 *
 * @param buffer Buffer of read data
 *
 * @return RT_EOK if read successfully.
 */
rt_inline rt_err_t spi_read_reg8(rt_device_t spi_device, uint8_t reg, uint8_t* buffer)
{
    uint8_t reg_addr;
    reg_addr = SPI_DIR_READ | reg;
    return rt_spi_send_then_recv((struct rt_spi_device*)spi_device, (void*)reg_addr, 1, (void*)buffer, 1);
}
/**
 * This function read multiple contiguous 8 bit regs.
 *
 * @param device the SPI device attached to SPI bus
 *
 * @param reg Start register address
 *
 * @param buffer Buffer of read data
 *
 * @param len The number of read registers
 *
 * @return RT_EOK if read successfully.
 */
rt_inline rt_err_t spi_read_multi_reg8(rt_device_t spi_device, uint8_t reg, uint8_t* buffer, uint8_t len)
{
    uint8_t reg_addr;
    reg_addr = SPI_DIR_READ | reg;
    return rt_spi_send_then_recv((struct rt_spi_device*)spi_device, (void*)reg_addr, 1, (void*)buffer, len);
}


#endif /* APPLICATIONS_MODULES_SPI_H_ */
