/**
 ****************************************************************************
 * @Warning Without permission from the author,Not for commercial use
 * @File    ledkey_opt.c
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

#include "ledkey_opt.h"
#include "local_status.h"
#include "auxilary.h"
#include "req_execution.h"
#include "global_var.h"
#include "mb_event_cpad.h"
extern local_reg_st l_sys;
extern sys_reg_st g_sys;
#define SAMPLE_UART_NAME "uart3"
/* 串口接收消息结构*/
struct rx_msg
{
    rt_device_t dev;
    rt_size_t size;
};
/* 串口设备句柄 */
static rt_device_t serial;
/* 消息队列控制块 */
static struct rt_messagequeue rx_mq;

static unsigned char ledkeyRecOK = 0;
static unsigned char txBuff[RT_SERIAL_RB_BUFSZ + 1] = {0xa7, 0xf3, 0xaa, 0x04, 0x06};
static unsigned char rxBuff[RT_SERIAL_RB_BUFSZ + 1] = {0};
extern const rt_uint8_t protocolHeader[2];
static rt_uint8_t rxCount = 0;

#define PARA_ADDR_START 64
#define PARA_NUM 22
#define STATE_ADDR_START 500
#define STATE_NUM 8

static rt_uint8_t regMap[PARA_NUM + STATE_NUM][14] = {0};
/**********************key led*********************************************************/
_TKS_FLAGA_type keyState[5];
volatile _TKS_FLAGA_type keyTrg[5];
_USR_FLAGA_type ledState[7];
_BEEP_STATE beepState = {0, 0, 0};

/*************************function******************************************************/
static void receiveProtocol(void);

/* 接收数据回调函数 */
static rt_err_t uart_input(rt_device_t dev, rt_size_t size)
{
    struct rx_msg msg;
    rt_err_t result = RT_EOK;
    rt_size_t rx_length;
    msg.dev  = dev;
    msg.size = size;

    /* 从串口读取数据*/
    rx_length = rt_device_read(dev, 0, rxBuff + rxCount, size);
    rxCount += rx_length;
    receiveProtocol();

    if (ledkeyRecOK)
    {
        result = rt_mq_send(&rx_mq, &msg, sizeof(msg));
        if (result == -RT_EFULL)
        {
            /* 消息队列满 */
            rt_kprintf("message queue full！\n");
        }
    }
    return result;
}
static void keyRecOperation(_TKS_FLAGA_type *keyState)
{
    static rt_uint8_t k_count[4] = {0};

    for (rt_uint8_t i = 0; i < rxBuff[3]; i++)
    {
        (keyState + i)->byte = rxBuff[4 + i];

        keyTrg[i].byte = (keyState + i)->byte & ((keyState + i)->byte ^ k_count[i]);
        k_count[i]     = (keyState + i)->byte;
    }
    if (boilingKeyTrg)
    {
        l_sys.j25WaterTempreture = BOILINGTEM;
    }
    if (normalKeyTrg)
    {
        l_sys.j25WaterTempreture = NORMALTEM;
    }
    if (teaKeyTrg)
    {
        l_sys.j25WaterTempreture = TEATEM;
    }
    if (cleanKeyTrg)
    {
        if (l_sys.j25AutomaticCleanState == 7)
        {
            l_sys.j25AutomaticCleanState = 0;
        }
    }
    if (milkKeyTrg)
    {
        l_sys.j25WaterTempreture = MILKTEM;
    }
    if (chlidKeyTrg)
    {
    }
    if (fetchKeyTrg)
    {
    }

    if (boilingKeyRestainTrg)
    {
    }

    if (normalKeyRestainTrg)
    {
    }

    if (teaKeyRestainTrg)
    {
    }

    if (cleanKeyRestainTrg)
    {
    }

    if (milkKeyRestainTrg)
    {
    }

    if (chlidKeyRestainTrg)
    {
        if (l_sys.j25ChildLockState == 0)
        {
            l_sys.j25ChildLockState = g_sys.config.ComPara.j25ChildLockTime * 2;
        }
    }

    if (fetchKeyRestainTrg)
    {
    }   
    if (BLEON)
    {
    }
    if (BLEONTrg)
    {
        rt_memset(regMap, 0, sizeof(regMap));
        rt_kprintf("BLEON\n");
    }

}
/***
 *
 *
 *
 **/
static void recOperation(rt_uint8_t *serialDataIn)
{
    switch (*(serialDataIn + 2))
    {
        case CMD_IDEL:
            break;
        case CMD_KEY:
            keyRecOperation(keyState);
            break;
        case CMD_LED:
            break;
        case CMD_REG_UP:
            break;
        case CMD_REG_DOWN: {
            rt_uint8_t length = *(serialDataIn + 3) - 2;
            rt_uint8_t row, index;
            rt_uint16_t address = (*(serialDataIn + 4) << 8) + *(serialDataIn + 5);
            rt_kprintf("length:%d address:%d \n", length, address);
            cpad_eMBRegHoldingCB(serialDataIn + 6, address, length / 2, CPAD_MB_REG_MULTIPLE_WRITE);
            if (address > STATE_ADDR_START)
            {
                row = (address - STATE_ADDR_START) / 6 + PARA_NUM;
            }
            else
            {
                row = (address - PARA_ADDR_START) / 6;
            }
            index = ((address - PARA_ADDR_START) % 6) * 2;

            rt_kprintf("((length > (12 - index)) ? (12 - index) : length):%d row:%d \n",
                       (length > (12 - index)) ? (12 - index) : length, row);
            rt_memcpy(&regMap[row][index + 2], serialDataIn + 6, (length > (12 - index)) ? (12 - index) : length);
            if ((12 - index) < length)
            {
                row++;
                rt_kprintf("(length + index - 12):%d row:%d \n", length + index - 12, row);
                rt_memcpy(&regMap[row][2], serialDataIn + 12 - index, length + index - 12);
            }
            break;
        }
        case CMD_AUX_DIAI:
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
        {
            rxStep = 1;
        }
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
            ledkeyRecOK = 1;
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
 
 

void ledSendOperation(void)
{
    if (l_sys.j25WaterMakeState)
    {
        makeWaterBlueState = STATE_LED_ON;
    }
    else
    {
        makeWaterBlueState = STATE_LED_OFF;
    }
    if (l_sys.j25AutomaticCleanState)
    {
        cleanBlueState = STATE_LED_ON;
    }
    else
    {
        cleanBlueState = STATE_LED_OFF;
    }
    boilingKeyState = STATE_LED_OFF;
    normalKeyState  = STATE_LED_OFF;
    teaKeyState     = STATE_LED_OFF;
    milkKeyState    = STATE_LED_OFF;
    switch (l_sys.j25WaterTempreture)
    {
        case IDELTEM:
            normalKeyState = STATE_LED_ON;
            break;
        case BOILINGTEM:
            boilingKeyState = STATE_LED_ON;
            break;
        case NORMALTEM:
            normalKeyState = STATE_LED_ON;
            break;
        case TEATEM:
            teaKeyState = STATE_LED_ON;
            break;
        case MILKTEM:
            milkKeyState = STATE_LED_ON;
            break;
        default:
            break;
    }
    if (l_sys.j25ChildLockState)
    {
        childLockState = STATE_LED_ON;
    }
    else
    {
        childLockState = STATE_LED_OFF;
    }
    if (l_sys.j25AutomaticCleanState == 7)
    {
        cleanKeyState = STATE_LED_ON;
    }
    else
    {
        cleanKeyState = STATE_LED_OFF;
    }
    if (g_sys.status.alarm_bitmap[1] & 0xff80)
    {
        fetchKeyState = STATE_LED_ON;
    }
    else
    {
        fetchKeyState = STATE_LED_OFF;
    }

    makeWaterGreenState = STATE_LED_OFF;
    loopBlueState       = STATE_LED_OFF;
    loopGreenState      = STATE_LED_OFF;
    cleanGreenState     = STATE_LED_OFF;
    loopRedState        = STATE_LED_OFF;

    beepState.byte = 0;
}
rt_uint8_t *getRegData(void)
{
    rt_uint8_t temp[12];
    for (rt_uint8_t i = 0; i < 30; i++)
    {
        rt_uint16_t address = (i < PARA_NUM) ? (i * 6 + PARA_ADDR_START) : ((i - PARA_NUM) * 6 + STATE_ADDR_START);
        cpad_eMBRegHoldingCB(temp, address, 6, CPAD_MB_REG_READ);
        if (rt_memcmp(&regMap[i][2], temp, 12) != 0)
        {
            regMap[i][0] = (address >> 8);
            regMap[i][1] = address & 0xff;
            rt_memcpy(&regMap[i][2], temp, 12);
            rt_kprintf("index:%04d\n", address - ((i < PARA_NUM) ? PARA_ADDR_START : 0));
            return &regMap[i][0];
        }
    }
    return RT_NULL;
}
static rt_uint8_t dataRepare(rt_uint8_t *buff)
{
    rt_uint8_t *regPoint = RT_NULL;
    if (BLEON)
    {
        regPoint = getRegData();
    }
    rt_memcpy(buff, protocolHeader, 2);
    if (regPoint)
    {
        buff[2] = CMD_REG_UP;
        buff[3] = 14;
        rt_memcpy(&buff[4], regPoint, 14);
    }
    else
    {
        rt_uint8_t len = 0;
        ledSendOperation();
        *(buff + 2) = CMD_LED;
        *(buff + 3) = 8;
        *(buff + 4) = beepState.byte;
        for (len = 0; len < (*(buff + 3) - 1); len++)
        {
            *(buff + 5 + len) = ledState[len].byte;
        }
        *(buff + 4 + *(buff + 3)) = getCheckSum(buff);
    }

    return (*(buff + 3) + 5);
}
static void serial_thread_entry(void *parameter)
{
    struct rx_msg msg;
    rt_err_t result;
    rt_uint32_t rx_length;

    rt_kprintf("*************start ledkey thread***********\r\n");

    while (1)
    {
        rt_memset(&msg, 0, sizeof(msg));
        /* 从消息队列中读取消息*/
        result = rt_mq_recv(&rx_mq, &msg, sizeof(msg), 1000);
        if (result == RT_EOK)
        {
            if (ledkeyRecOK)
            {
                rx_length = dataRepare(txBuff);
                rt_device_write(serial, 0, txBuff, rx_length);
                ledkeyRecOK = 0;
            }
        }
    }
}

int ledKeyStart(void)
{
    rt_err_t ret = RT_EOK;
    char uart_name[RT_NAME_MAX];
    static char msg_pool[256];
    char str[] = "hello RT-Thread!\r\n";

    rt_strncpy(uart_name, SAMPLE_UART_NAME, RT_NAME_MAX);

    /* 查找串口设备 */
    serial = rt_device_find(uart_name);
    if (!serial)
    {
        rt_kprintf("find %s failed!\n", uart_name);
        return RT_ERROR;
    }

    /* 初始化消息队列 */
    rt_mq_init(&rx_mq, "rx_mq", msg_pool, /* 存放消息的缓冲区 */
               sizeof(struct rx_msg),     /* 一条消息的最大长度 */
               sizeof(msg_pool),          /* 存放消息的缓冲区大小 */
               RT_IPC_FLAG_FIFO);         /* 如果有多个线程等待，按照先来先得到的方法分配消息 */

    /* 以 DMA 接收及轮询发送方式打开串口设备 */
    rt_device_open(serial, RT_DEVICE_FLAG_DMA_RX);
    /* 设置接收回调函数 */
    rt_device_set_rx_indicate(serial, uart_input);
    /* 发送字符串 */
    rt_device_write(serial, 0, str, (sizeof(str) - 1));

    /* 创建 serial 线程 */
    rt_thread_t thread = rt_thread_create("serial", serial_thread_entry, RT_NULL, 1024, 25, 10);
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
