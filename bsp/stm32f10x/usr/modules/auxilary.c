/**
 ****************************************************************************
 * @Warning Without permission from the author,Not for commercial use
 * @File    .c
 * @Author  xiaowine@cee0.com
 * @date
 * @version V1.0
 *************************************************
 * @brief   标注系统信息
 ****************************************************************************
 * @attention
 * Powered By Xiaowine
 * <h2><center>&copy;  Copyright(C) cee0.com 2015-2019</center></h2>
 * All rights reserved
 *
 **/
#include <rtthread.h>
#include "auxilary.h"
#include "local_status.h"
#include "ble_key.h"

#define CONFIG_DEBUG
#ifdef CONFIG_DEBUG
#ifndef auxilary_log
#define auxilary_log(N, ...) rt_kprintf("##[auxilary %s:%4d] " N "\r\n", __FUNCTION__, __LINE__, ##__VA_ARGS__)
#endif /* auxilary_log(...) */
#else
#define auxilary_log(...)
#endif /* ! CONFIG_DEBUG */

#define SAMPLE_UART_NAME "uart4"

/* 串口设备句柄 */
static rt_device_t serial;

static rt_uint8_t txBuff[RT_SERIAL_RB_BUFSZ + 1] = {0xa7, 0xf3, 0xaa, 0x04, 0x06};
static rt_uint8_t rxBuff[RT_SERIAL_RB_BUFSZ + 1] = {0xa7, 0xf3, 0xaa, 0x04, 0x06};
const rt_uint8_t protocolHeader[2]               = {0xff, 0xa5};
static rt_uint8_t rxCount                        = 0;

extern local_reg_st l_sys;

static void recOperation(rt_uint8_t *serialDataIn)
{
    switch (*(serialDataIn + 2))
    {
        case CMD_IDEL:
            break;
        case CMD_KEY:
            break;
        case CMD_LED:
            break;
        case CMD_REG_UP:
            break;
        case CMD_REG_DOWN:
            break;
        case CMD_AUX_DIAI:
            l_sys.j25AuxiliaryBoardDI = (*(serialDataIn + 4) << 8) | *(serialDataIn + 5);
            l_sys.j25AuxiliaryBoardAI = (*(serialDataIn + 6) << 8) | *(serialDataIn + 7);
            break;
        case CMD_AUX_DO:
            break;
        default:
            break;
    }
}
static rt_uint8_t getCheckSum(rt_uint8_t *data)
{
    rt_uint8_t checkSum = 0;
    rt_uint8_t i;
    for (i = 0; i < (*(data + 3) + 4); i++)
    {
        checkSum += *(data + i);
    }
    return checkSum;
}
static void receiveProtocol(void)
{
    static rt_uint8_t rxStep = 0;

again:
    if (rxStep == 0)
    {
        if (rxCount < 2)
            goto rxContinue;
        if (rt_memcmp(rxBuff, (void *)protocolHeader, 2) == 0)
            rxStep = 1;
        else
        {
            rt_memcpy(rxBuff, rxBuff + 1, rxCount - 1);
            rxCount--;
            goto again;
        }
    }

    if (rxStep == 1)
    {
        if (rxCount < 4)
            goto rxContinue;
        if (rxBuff[3] <= 15)
            rxStep = 2;
        else
        {
            rt_memcpy(rxBuff, rxBuff + 2, rxCount - 2);
            rxCount -= 2;
            rxStep = 0;
            goto again;
        }
    }

    if (rxStep == 2)
    {
        rt_uint8_t checkSum, checkIndex, len;
        if (rxCount < rxBuff[3] + 5)
            goto rxContinue;
        checkSum   = getCheckSum(rxBuff);
        checkIndex = rxBuff[3] + 4;
        if (checkSum == rxBuff[checkIndex])
        {
            recOperation(rxBuff);
            len    = rxBuff[3] + 5;
            rxStep = 0;
            rt_memcpy(rxBuff, rxBuff + len, rxCount - len);
            rxCount -= len;
        }
        else
        {
            rt_memcpy(rxBuff, rxBuff + 4, rxCount - 4);
            rxCount -= 4;
            rxStep = 0;
            goto again;
        }
    }
rxContinue:
    return;
}

/* 接收数据回调函数 */
static rt_err_t uartByteReceived(rt_device_t dev, rt_size_t size)
{
    rt_uint32_t rx_length;
    /* 从串口读取数据*/
    rx_length = rt_device_read(dev, 0, rxBuff + rxCount, size);
    rxCount += rx_length;
    receiveProtocol();
    return RT_EOK;
}
static rt_uint8_t auxilaryDataRepare(rt_uint8_t *buff)
{
    rt_uint8_t len = 0;
    rt_memcpy(buff, protocolHeader, 2);
    *(buff + 2) = CMD_AUX_DO;
    *(buff + 3) = 2;
    *(buff + 4) = l_sys.j25AuxiliaryBoardDO >> 8;
    *(buff + 5) = l_sys.j25AuxiliaryBoardDO & 0xff;
    *(buff + 6) = getCheckSum(buff);
    len         = *(buff + 3) + 5;
    return len;
}

static void serial_thread_entry(void *parameter)
{
    rt_kprintf("****************************start auxilaryStart thread**************************\r\n");
    rt_uint8_t len = 0;
    while (1)
    {
        rt_thread_delay(100);
        len = auxilaryDataRepare(txBuff);
        rt_device_write(serial, 0, txBuff, len);
    }
}

int auxilaryStart(void)
{
    rt_err_t ret = RT_EOK;
    char uart_name[RT_NAME_MAX];
    char str[] = "hello RT-Thread!\r\n";

    rt_strncpy(uart_name, SAMPLE_UART_NAME, RT_NAME_MAX);

    /* 查找串口设备 */
    serial = rt_device_find(uart_name);
    if (!serial)
    {
        rt_kprintf("find %s failed!\n", uart_name);
        return RT_ERROR;
    }

    /* 以 DMA 接收及轮询发送方式打开串口设备 */
    rt_device_open(serial, RT_DEVICE_FLAG_DMA_RX);
    /* 设置接收回调函数 */
    rt_device_set_rx_indicate(serial, uartByteReceived);
    /* 发送字符串 */
    rt_device_write(serial, 0, str, (sizeof(str) - 1));

    /* 创建 serial 线程 */
    rt_thread_t thread = rt_thread_create("Auserial", serial_thread_entry, RT_NULL, 512, 13, 10);
    /* 创建成功则启动线程 */
    if (thread != RT_NULL)
    {
        rt_thread_startup(thread);
    }
    else
    {
        rt_kprintf("serial thread_startup failed!\n");
        ret = RT_ERROR;
    }

    return ret;
}
INIT_APP_EXPORT(auxilaryStart);
