/**
 ****************************************************************************
 * @Warning Without permission from the author,Not for commercial use
 * @File    ledkey_opt.c
 * @Author  xiaowine@cee0.com
 * @date
 * @version V1.0
 *************************************************
 * @brief   ��עϵͳ��Ϣ
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
#define SAMPLE_UART_NAME "uart3"
/* ���ڽ�����Ϣ�ṹ*/
struct rx_msg
{
    rt_device_t dev;
    rt_size_t size;
};
/* �����豸��� */
static rt_device_t serial;
/* ��Ϣ���п��ƿ� */
static struct rt_messagequeue rx_mq;

unsigned char recOK                                 = 0;
static unsigned char txBuff[RT_SERIAL_RB_BUFSZ + 1] = {0xa7, 0xf3, 0xaa, 0x04, 0x06};
static unsigned char rxBuff[RT_SERIAL_RB_BUFSZ + 1] = {0};
extern const rt_uint8_t protocolHeader[2];
static rt_uint8_t rxCount = 0;
/**********************key led*********************************************************/
_TKS_FLAGA_type keyState[2];
volatile _TKS_FLAGA_type keyTrg[2];

#define KEY1 keyTrg[0].bits.b2
#define KEY2 keyTrg[0].bits.b3
#define KEY3 keyTrg[0].bits.b1
#define KEY4 keyTrg[0].bits.b0

#define KEY1Restain keyTrg[1].bits.b2
#define KEY2Restain keyTrg[1].bits.b3
#define KEY3Restain keyTrg[1].bits.b1
#define KEY4Restain keyTrg[1].bits.b0

_USR_FLAGA_type ledState[5];
#define led1State ledState[0].s4bits.s0
#define led2State ledState[0].s4bits.s1
#define led3State ledState[1].s4bits.s0
#define led4State ledState[1].s4bits.s1
#define led5State ledState[2].s4bits.s0
#define led6State ledState[2].s4bits.s1
#define led7State ledState[3].s4bits.s0
#define led8State ledState[3].s4bits.s1
#define led9State ledState[4].s4bits.s0

unsigned char beepCount = 0;

/*************************function******************************************************/

/* �������ݻص����� */
static rt_err_t uart_input(rt_device_t dev, rt_size_t size)
{
    struct rx_msg msg;
    rt_err_t result;
    msg.dev  = dev;
    msg.size = size;

    result = rt_mq_send(&rx_mq, &msg, sizeof(msg));
    if (result == -RT_EFULL)
    {
        /* ��Ϣ������ */
        rt_kprintf("message queue full��\n");
    }
    return result;
}
static void keyRecOperation(_TKS_FLAGA_type *keyState)
{
    static rt_uint8_t k_count[2] = {0};
    keyTrg[0].byte               = keyState->byte & (keyState->byte ^ k_count[0]);
    k_count[0]                   = keyState->byte;
    keyTrg[1].byte               = (keyState + 1)->byte & ((keyState + 1)->byte ^ k_count[1]);
    k_count[1]                   = (keyState + 1)->byte;
    if (KEY1)
    {
        rt_kprintf("key1\n");
    }
    if (KEY2)
    {
    }
    if (KEY3)
    {
        // RAM_Write_Reg(86, g_sys.config.ComPara.u16ColdWater_Mode, 1);
        rt_kprintf("key3\n");
    }
    if (KEY4)
    {
        rt_kprintf("key4\n");
    }

    if (KEY1Restain)
    {
        // RAM_Write_Reg(EE_EXITWATER, g_sys.config.ComPara.u16ExitWater_Mode, 1);
        rt_kprintf("KEY1Restain\n");
    }
    if (KEY2Restain)
    {
        rt_kprintf("KEY2Restain\n");
    }
    if (KEY3Restain)
    {
        rt_kprintf("KEY3Restain\n");
    }

    if (KEY4Restain)
    {
        // write_reg_map(POWER_ON_ADDR, g_sys.config.ComPara.u16Power_Mode);
        rt_kprintf("KEY4Restain\n");
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
        case CMD_REG_DOWN:
            break;
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
void receiveProtocol(void)
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
            recOK = 1;
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
/****
 *
 *
 *
 *
 **/
#include "req_execution.h"
#include "local_status.h"
#include "global_var.h"
extern local_reg_st l_sys;
extern sys_reg_st g_sys;
/**
 *
 *
 *
 **/
#include "sys_status.h"
void ledSendOperation(void)
{
    led1State = STATE_LED_FLASH_1HZ;

    led1State = STATE_LED_OFF;

    led2State = STATE_LED_OFF;

    led3State = STATE_LED_FLASH_1HZ;

    led4State = STATE_LED_FLASH_1HZ;

    led5State = STATE_LED_ON;

    led6State = STATE_LED_FLASH_2HZ;

    led7State = STATE_LED_FLASH_0_5HZ;

    led8State = STATE_LED_OFF;

    led9State = STATE_LED_ON;

    txBuff[0] = 0xa7;
    txBuff[1] = 0xf6;
    txBuff[2] = ledState[0].byte;
    txBuff[3] = ledState[1].byte;
    txBuff[4] = ledState[2].byte;
    txBuff[5] = ledState[3].byte;
    txBuff[6] = ledState[4].byte;
    txBuff[7] = beepCount & 0x0f;
    beepCount = 0;

    txBuff[8] = txBuff[0];
    txBuff[8] += txBuff[1];
    txBuff[8] += txBuff[2];
    txBuff[8] += txBuff[3];
    txBuff[8] += txBuff[4];
    txBuff[8] += txBuff[5];
    txBuff[8] += txBuff[6];
    txBuff[8] += txBuff[7];
}
static rt_uint8_t dataRepare(rt_uint8_t *buff)
{
    rt_uint8_t len = 0;
    keyRecOperation(keyState);
    ledSendOperation();
    rt_memcpy(buff, protocolHeader, 2);
    *(buff + 2) = CMD_LED;
    *(buff + 3) = 2;
    *(buff + 4) = l_sys.j25AuxiliaryBoardDO >> 8;
    *(buff + 5) = l_sys.j25AuxiliaryBoardDO & 0xff;
    *(buff + 6) = getCheckSum(buff);
    len         = *(buff + 3) + 5;
    return len;
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
        /* ����Ϣ�����ж�ȡ��Ϣ*/
        result = rt_mq_recv(&rx_mq, &msg, sizeof(msg), 1000);
        if (result == RT_EOK)
        {
            /* �Ӵ��ڶ�ȡ����*/
            rx_length = rt_device_read(msg.dev, 0, rxBuff, msg.size);
            rxCount += rx_length;
            receiveProtocol();

            if (recOK)
            {
                rx_length = dataRepare(txBuff);

                rt_device_write(serial, 0, txBuff, rx_length);
                recOK = 0;
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

    /* ���Ҵ����豸 */
    serial = rt_device_find(uart_name);
    if (!serial)
    {
        rt_kprintf("find %s failed!\n", uart_name);
        return RT_ERROR;
    }

    /* ��ʼ����Ϣ���� */
    rt_mq_init(&rx_mq, "rx_mq", msg_pool, /* �����Ϣ�Ļ����� */
               sizeof(struct rx_msg),     /* һ����Ϣ����󳤶� */
               sizeof(msg_pool),          /* �����Ϣ�Ļ�������С */
               RT_IPC_FLAG_FIFO);         /* ����ж���̵߳ȴ������������ȵõ��ķ���������Ϣ */

    /* �� DMA ���ռ���ѯ���ͷ�ʽ�򿪴����豸 */
    rt_device_open(serial, RT_DEVICE_FLAG_DMA_RX);
    /* ���ý��ջص����� */
    rt_device_set_rx_indicate(serial, uart_input);
    /* �����ַ��� */
    rt_device_write(serial, 0, str, (sizeof(str) - 1));

    /* ���� serial �߳� */
    rt_thread_t thread = rt_thread_create("serial", serial_thread_entry, RT_NULL, 1024, 25, 10);
    /* �����ɹ��������߳� */
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
