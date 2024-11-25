/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-11-06     SummerGift   first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "rm_module.h"
#include "rm_config.h"

/* defined the LED_G pin: PF14 */
#define LED0_PIN    GET_PIN(F, 14)

int main(void)
{
    /* set LED0 pin mode to output */
    rt_pin_mode(LED0_PIN, PIN_MODE_OUTPUT);

    dwt_init(CPU_FREQUENCY);

    while (1)
    {
        rt_pin_write(LED0_PIN, PIN_HIGH);
        rt_thread_mdelay(500);
        rt_pin_write(LED0_PIN, PIN_LOW);
        rt_thread_mdelay(500);
    }
}

/*!
 * @brief can 设备初始化函数，仅调用一次
 * @return RT_EOK or RT_ERROR
 */
static int can_device_init(void){
    rt_device_t can_dev = RT_NULL;

#ifdef BSP_USING_CAN1
    can_dev = rt_device_find("can1");
    if (!can_dev)
    {
        rt_kprintf("find can device failed!\n");
        return RT_ERROR;
    }
    /* 设置 CAN 设备接收回调处理 */
    rt_device_set_rx_indicate(can_dev, can1_rx_call);
    /* 设置 CAN 通信的波特率为 1Mbit/s*/
    rt_device_control(can_dev, RT_CAN_CMD_SET_BAUD, (void *)CAN1MBaud);
    /* 启动对应 CAN 设备 */
    rt_device_open(can_dev, RT_DEVICE_FLAG_INT_TX | RT_DEVICE_FLAG_INT_RX);
#endif /* BSP_USING_CAN1 */

#ifdef BSP_USING_CAN2
    can_dev = rt_device_find("can2");
    if (!can_dev)
    {
        rt_kprintf("find can device failed!\n");
        return RT_ERROR;
    }
    /* 设置 CAN 设备接收回调处理 */
    rt_device_set_rx_indicate(can_dev, can2_rx_call);
    /* 设置 CAN 通信的波特率为 1Mbit/s*/
    rt_device_control(can_dev, RT_CAN_CMD_SET_BAUD, (void *)CAN1MBaud);
    /* 启动对应 CAN 设备 */
    rt_device_open(can_dev, RT_DEVICE_FLAG_INT_TX | RT_DEVICE_FLAG_INT_RX);
#endif /* BSP_USING_CAN2 */

    return RT_EOK;
}
INIT_COMPONENT_EXPORT(can_device_init);

/*!
 * @brief i2c 设备初始化函数，仅调用一次
 * @return RT_EOK or RT_ERROR
 */
static int i2c_device_init(void){
    rt_device_t i2c_dev = RT_NULL;

#ifdef BSP_USING_I2C1
    i2c_dev = rt_device_find("i2c1");

    RT_ASSERT(i2c_dev != NULL);

    rt_device_open(i2c_dev, RT_DEVICE_OFLAG_RDWR);
#endif /* BSP_USING_CAN1 */

    return RT_EOK;
}
INIT_COMPONENT_EXPORT(i2c_device_init);
