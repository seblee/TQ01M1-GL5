/**
 ****************************************************************************
 * @Warning Without permission from the author,Not for commercial use
 * @File    ble_key.c
 * @Author  xiaowine@cee0.com
 * @date
 * @version V1.0
 *************************************************
 ****************************************************************************
 * @attention
 * Powered By Xiaowine
 * <h2><center>&copy;  Copyright(C) cee0.com 2015-2019</center></h2>
 * All rights reserved
 *
 **/

#include <rtthread.h>
#include <rtdevice.h>
#include "ble_key.h"
#include "i2c_utils.h"
#include "mb_event_cpad.h"
#include "sys_status.h"
#include "req_execution.h"
#include "local_status.h"
#include "global_var.h"
/*************************define**log****************************************************/
#define CONFIG_DEBUG
#ifdef CONFIG_DEBUG
#ifndef bleKeyLog
#define bleKeyLog(N, ...) rt_kprintf("[%d][bleKey:%04d]" N "\r\n", rt_tick_get(), __LINE__, ##__VA_ARGS__)
#endif /* bleKeyLog(...) */
#else
#define bleKeyLog(...)
#endif /* ! CONFIG_DEBUG */
/*************************I2C******************************************************/
#define I2C_ADDRESS 0x7e
#ifdef I2C_TOOLS_USE_SW_I2C
#define SDA_PORT_NUM 61
#define SCL_PORT_NUM 62
#else
#define I2C_DEVICE_NAME "i2c1"
#endif

#define BUFFMAX 22

rt_uint8_t recOK                     = 0;
static rt_uint8_t tx_buffer[BUFFMAX] = {1, 2, 3, 4, 5, 0, 7, 8, 9, 20};
static rt_uint8_t rx_buffer[BUFFMAX] = {20, 9, 8, 7, 6, 5, 4, 3, 2, 1};
const rt_uint8_t protocolHeader[2]   = {0xff, 0xa5};

#define PARA_ADDR_START 64
#define PARA_NUM 22
#define STATE_ADDR_START 500
#define STATE_NUM 8

static rt_uint8_t regMap[PARA_NUM + STATE_NUM][14] = {0};

static rt_uint16_t segmentValue[3]    = {0};
static rt_uint16_t segmentValueBak[3] = {0xffff};
#define WATERTEMPRETURE segmentValue[2]
/*******************************************************************************/
extern local_reg_st l_sys;
extern sys_reg_st g_sys;
/*************************Variable declaration******************************************************/
_TKS_FLAGA_type bleKeyFlag = {0};
#define PARACHANGEFLAG bleKeyFlag.bits.b0
#define CLILDLOCKWARNFLAG bleKeyFlag.bits.b0

_TKS_FLAGA_type keyState[KEYBYTENUM];
_TKS_FLAGA_type keyUpState[KEYBYTENUM];
volatile _TKS_FLAGA_type keyTrg[KEYBYTENUM];
volatile _TKS_FLAGA_type keyUpTrg[KEYBYTENUM];

#define SHOTBEEPMASK 0x7e
#define LONGBEEPMASK 0x02
rt_uint8_t keyBeepMask[2]    = {SHOTBEEPMASK, LONGBEEPMASK};
rt_uint8_t keyBeepMaskBak[2] = {0};

_USR_FLAGA_type ledState[7];

_BEEP_STATE beepState     = {0, 0, 0};
rt_uint8_t beepShortCount = 0;
rt_uint8_t beepLongCount  = 0;
/****************************************************************************/
rt_uint8_t getCheckSum(rt_uint8_t *data);
void keyRecOperation(_TKS_FLAGA_type *keyState);
void operateRxData(rt_uint8_t *rxData);
rt_uint8_t *getRegData(void);
static void caculateLed(void);
static void keyProcess(void);
static void waterOutOpt(uint8_t outWaterval);
/****************************************************************************/

void keyRecOperation(_TKS_FLAGA_type *keyState)
{
    static rt_uint8_t k_count[KEYBYTENUM]   = {0};
    static rt_uint8_t k_Upcount[KEYBYTENUM] = {0};

    for (rt_uint8_t i = 0; i < KEYBYTENUM; i++)
    {
        keyTrg[i].byte = (keyState + i)->byte & ((keyState + i)->byte ^ k_count[i]);
        k_count[i]     = (keyState + i)->byte;
    }

    for (rt_uint8_t i = 0; i < KEYBYTENUM; i++)
    {
        keyUpState[i].byte = ~(keyState + i)->byte;
        keyUpTrg[i].byte   = keyUpState[i].byte & (keyUpState[i].byte ^ k_Upcount[i]);
        k_Upcount[i]       = keyUpState[i].byte;
    }
    if (l_sys.OutWater_Flag == WATER_NO)
    {
        //短按
        if (boilingKeyTrg)  //开水
        {
            l_sys.LedKey.WaterTemperature = BOILINGTEM;
            l_sys.LedKey.WaterTemperature |= KEY_TEMP;
        }
        if (normalKeyTrg)  //常温水
        {
            l_sys.LedKey.WaterTemperature = NORMALTEM;
            l_sys.LedKey.WaterTemperature |= KEY_TEMP;
        }
        if (teaKeyTrg)  //泡茶
        {
            l_sys.LedKey.WaterTemperature = TEATEM;
            l_sys.LedKey.WaterTemperature |= KEY_TEMP;
        }
        if (milkKeyTrg)  //泡奶
        {
            l_sys.LedKey.WaterTemperature = MILKTEM;
            l_sys.LedKey.WaterTemperature |= KEY_TEMP;
        }
    }
    if (cleanKeyTrg)
    {
        if (l_sys.j25AutomaticCleanState == 7)
        {
            l_sys.j25AutomaticCleanState = 0;
        }
    }

    if (chlidKeyTrg)  //童锁
    {
    }

    if (fetchKeyTrg)  //出水
    {
        if ((sys_get_remap_status(WORK_MODE_STS_REG_NO, OUTWATER_STS_BPOS) == TRUE))  // Water out
            waterOutOpt(FALSE);
        else
        {
            if ((l_sys.LedKey.WaterTemperature != NORMALTEM) && (l_sys.LedKey.ChildLock == FALSE) &&
                (l_sys.LedKey.OutWater == FALSE))
            {
                beepShortCount    = 3;
                CLILDLOCKWARNFLAG = 1;
            }
        }
    }

    //长按
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
        if (l_sys.j25AutomaticCleanState == 0)
        {
            l_sys.j25AutomaticCleanState = 1;
        }
    }

    if (milkKeyRestainTrg)
    {
    }

    if (chlidKeyRestainTrg)
    {
        if ((l_sys.LedKey.WaterTemperature & 0xff) != NORMALTEM)
        {
            l_sys.LedKey.ChildLock = TRUE;
            rt_kprintf("l_sys.LedKey.ChildLock = TRUE\n");
        }
    }
    if (fetchKeyRestainTrg)
    {
        if (WaterOut_level() == TRUE)
        {
            if (l_sys.LedKey.WaterTemperature == NORMALTEM)  //常温水，直接出
            {
                waterOutOpt(TRUE);
            }
            else
            {
                if (l_sys.LedKey.ChildLock == TRUE)  //需要解锁
                {
                    waterOutOpt(TRUE);
                }
                else
                {
                    waterOutOpt(TRUE);
                }
            }
        }
    }
    if (fetchKeyRestain)
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
    if (BLEINITDONE)
    {
    }
    if (BLEINITDONETrg)
    {
        rt_kprintf("BLEINITDONETrg\n");
    }
    //按键逻辑处理
    keyProcess();
}
void operateRxData(rt_uint8_t *rxData)
{
    if ((*(rxData + 3) + 5) > BUFFMAX)
    {
        return;
    }
    if (getCheckSum(rxData) == *(rxData + *(rxData + 3) + 4))
    {
        switch (*(rxData + 2))
        {
            case CMD_IDEL:
                break;
            case CMD_KEY: {
                keyState[0].byte = *(rxData + 4);
                keyState[1].byte = *(rxData + 5);
                keyState[2].byte = *(rxData + 6);
                keyRecOperation(keyState);
                break;
            }
            case CMD_LED:
                break;
            case CMD_REG_UP:
                rt_kprintf("i2c_read I2C_REG_UP \n");
                break;
            case CMD_REG_DOWN: {
                rt_uint8_t length = *(rxData + 3) - 2;
                rt_uint8_t row, index;
                rt_uint16_t address = (*(rxData + 4) << 8) + *(rxData + 5);
                rt_kprintf("length:%d address:%d \n", length, address);
                cpad_eMBRegHoldingCB(rxData + 6, address, length / 2, CPAD_MB_REG_MULTIPLE_WRITE);
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
                rt_memcpy(&regMap[row][index + 2], rxData + 6, (length > (12 - index)) ? (12 - index) : length);
                if ((12 - index) < length)
                {
                    row++;
                    rt_kprintf("(length + index - 12):%d row:%d \n", length + index - 12, row);
                    rt_memcpy(&regMap[row][2], rxData + 12 - index, length + index - 12);
                }
                break;
            }
            case CMD_AUX_DIAI:
                break;
            case CMD_AUX_DO:
                break;
            case CMD_PARA:
                break;
            default:
                break;
        }
    }
    else
    {
        for (char i = 0; i < 20; i++)
        {
            rt_kprintf("%02x ", *(rxData + i));
        }
        rt_kprintf("\r\n");
        rt_kprintf("***checkSum err \n");
    }
}

rt_uint8_t getCheckSum(rt_uint8_t *data)
{
    rt_uint8_t checkSum = 0;
    if ((*(data + 3) + 4) > (BUFFMAX - 1))
    {
        return 0;
    }
    for (rt_uint8_t i = 0; i < (*(data + 3) + 4); i++)
    {
        checkSum += *(data + i);
    }
    return checkSum;
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

static void caculateLed(void)
{
    if (CLILDLOCKWARNFLAG)
    {
        childLockState    = STATE_LED_FLASH_3_T;
        CLILDLOCKWARNFLAG = 0;
    }
    else
    {
        if (l_sys.LedKey.ChildLock)
        {
            childLockState = STATE_LED_ON;
        }
        else
        {
            childLockState = STATE_LED_OFF;
        }
    }
    boilingKeyState = STATE_LED_OFF;
    teaKeyState     = STATE_LED_OFF;
    milkKeyState    = STATE_LED_OFF;
    normalKeyState  = STATE_LED_OFF;
    switch (l_sys.LedKey.WaterTemperature & 0x00ff)
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
            normalKeyState = STATE_LED_ON;
            break;
    }
    if (g_sys.status.alarm_bitmap[1] & 0xff80)
    {
        fetchKeyState = STATE_LED_ON;
    }
    else
    {
        fetchKeyState = STATE_LED_OFF;
    }
    if (l_sys.j25AutomaticCleanState == 7)
    {
        cleanKeyState = STATE_LED_ON;
    }
    else
    {
        cleanKeyState = STATE_LED_OFF;
    }
    makeWaterGreenState = STATE_LED_OFF;
    loopBlueState       = STATE_LED_OFF;
    loopGreenState      = STATE_LED_OFF;
    if (l_sys.j25WaterMakeState)
    {
        makeWaterBlueState = STATE_LED_ON;
    }
    else
    {
        makeWaterBlueState = STATE_LED_OFF;
    }
    loopRedState = STATE_LED_OFF;
    if (l_sys.j25AutomaticCleanState)
    {
        cleanBlueState = STATE_LED_ON;
    }
    else
    {
        cleanBlueState = STATE_LED_OFF;
    }
    cleanGreenState = STATE_LED_OFF;

    beepState.byte = 0;
    BEEPSHORT      = beepShortCount;
    BEEPLONG       = beepLongCount;
    beepLongCount  = 0;
    beepShortCount = 0;
}

//按键处理
static void keyProcess(void)
{
    extern local_reg_st l_sys;
    uint8_t u8Temp;
    //出水温度处理
    if (l_sys.LedKey.WaterTemperature & KEY_TEMP)
    {
        rt_uint16_t u16HotWater_TempBak = g_sys.config.ComPara.u16HotWater_Temp;
        u8Temp                          = l_sys.LedKey.WaterTemperature & (~KEY_TEMP);
        l_sys.LedKey.WaterTemperature &= (~KEY_TEMP);
        switch (u8Temp)
        {
            case BOILINGTEM: {
                g_sys.config.ComPara.u16HotWater_Temp = 99;
                keyBeepMask[0] |= 0x01;
                keyBeepMask[1] |= 0x01;
                break;
            }
            case TEATEM: {
                g_sys.config.ComPara.u16HotWater_Temp = 85;
                keyBeepMask[0] |= 0x01;
                keyBeepMask[1] |= 0x01;
                break;
            }
            case MILKTEM: {
                g_sys.config.ComPara.u16HotWater_Temp = 45;
                keyBeepMask[0] |= 0x01;
                keyBeepMask[1] |= 0x01;
                break;
            }
            case NORMALTEM: {
                g_sys.config.ComPara.u16HotWater_Temp = 25;
                keyBeepMask[0] &= ~0x01;
                keyBeepMask[1] &= ~0x01;
                break;
            }
            default: {
                g_sys.config.ComPara.u16HotWater_Temp = 25;
                keyBeepMask[0] &= ~0x01;
                keyBeepMask[1] &= ~0x01;
                break;
            }
        }
        if (u16HotWater_TempBak != g_sys.config.ComPara.u16HotWater_Temp)
        {
            RAM_Write_Reg(EE_HOTTEMP, g_sys.config.ComPara.u16HotWater_Temp, 1);
        }
    }
    if ((l_sys.LedKey.WaterTemperature == NORMALTEM) || (l_sys.LedKey.ChildLock) || (l_sys.LedKey.OutWater))
    {
        keyBeepMask[0] |= 0x02;
    }
    else
    {
        keyBeepMask[0] &= ~0x02;
    }

    return;
}
/**
 * @brief
 * @param  outWaterval      My Param doc
 */
static void waterOutOpt(uint8_t outWaterval)
{
    //出水处理
    if (outWaterval == TRUE)
    {
        g_sys.config.ComPara.u16Water_Mode = WATER_HEAT;
        g_sys.config.ComPara.u16Water_Flow = 5000;
        l_sys.LedKey.OutWater              = 1;
    }
    else
    {
        g_sys.config.ComPara.u16Water_Mode = 0;
        g_sys.config.ComPara.u16Water_Flow = 0;
        l_sys.LedKey.OutWater              = 0;
    }
}

/**
 * @brief
 * @param  buff             data buff
 * @return rt_uint8_t
 */
static rt_uint8_t dataRepare(rt_uint8_t *buff)
{
    rt_uint8_t *regPoint = RT_NULL;
    rt_memcpy(tx_buffer, protocolHeader, 2);

    if (BLEON && BLEINITDONE)
    {
        regPoint = getRegData();
    }

    if (regPoint)
    {
        buff[2] = CMD_REG_UP;
        buff[3] = 14;
        rt_memcpy(&buff[4], regPoint, 14);
        *(buff + 4 + *(buff + 3)) = getCheckSum(buff);
        goto repareExit;
    }
    if (rt_memcmp(segmentValue, segmentValueBak, 6))
    {
        *(buff + 2) = CMD_SEGMENT;
        *(buff + 3) = 6;
        rt_memcpy(buff + 4, segmentValue, 6);
        *(buff + 4 + *(buff + 3)) = getCheckSum(buff);
        rt_memcpy(segmentValueBak, segmentValue, 6);
        goto repareExit;
    }
    if ((!PARAOK) || rt_memcmp(keyBeepMask, keyBeepMaskBak, 2))
    {
        bleKeyLog("keyBeepMask [0]:%02x [1]:%02x", keyBeepMask[0], keyBeepMask[1]);
        *(buff + 2) = CMD_PARA;
        *(buff + 3) = 2;
        rt_memcpy((buff + 4), keyBeepMask, 2);
        *(buff + 4 + *(buff + 3)) = getCheckSum(buff);
        rt_memcpy(keyBeepMaskBak, keyBeepMask, 2);
        goto repareExit;
    }

    {
        rt_uint8_t len = 0;
        caculateLed();
        *(buff + 2) = CMD_LED;
        *(buff + 3) = 8;
        *(buff + 4) = beepState.byte;
        for (len = 0; len < (*(buff + 3) - 1); len++)
        {
            *(buff + 5 + len) = ledState[len].byte;
        }
        *(buff + 4 + *(buff + 3)) = getCheckSum(buff);
        goto repareExit;
    }

repareExit:
    return (*(buff + 3) + 5);
}
void ledDataInit(void)
{
    switch (l_sys.j25WaterTempreture)
    {
        case 99: {
            l_sys.LedKey.WaterTemperature = BOILINGTEM;
            break;
        }
        case 85: {
            l_sys.LedKey.WaterTemperature = TEATEM;
            break;
        }
        case 45: {
            l_sys.LedKey.WaterTemperature = MILKTEM;
            break;
        }
        case 25:
        default: {
            l_sys.LedKey.WaterTemperature = NORMALTEM;
            break;
        }
    }
    if ((l_sys.LedKey.WaterTemperature & 0xff) != NORMALTEM)
    {
        keyBeepMask[0] = SHOTBEEPMASK | 0x01;
        keyBeepMask[1] = LONGBEEPMASK | 0x01;
    }
}

static void i2c_thread_entry(void *para)
{
    static rt_uint8_t len = 0;
    rt_kprintf("*************start ledkey thread***********\r\n");
    rt_thread_delay(1000);
    ledDataInit();
    while (1)
    {
        /* 调用I2C设备接口传输数据 */
        if (len == 0)
        {
            len = dataRepare(tx_buffer);
        }
        if (i2c_write(I2C_ADDRESS, tx_buffer, len) == 1)
        {
            len = 0;
        }
        else
        {
            rt_thread_delay(rt_tick_from_millisecond(1000));
            rt_kprintf("i2c_write err \n");
            continue;
        }
        rt_thread_delay(rt_tick_from_millisecond(50));
        if (i2c_read(I2C_ADDRESS, 0, rx_buffer, 20) == 20)
        {
            operateRxData(rx_buffer);
        }
        else
        {
            rt_thread_delay(rt_tick_from_millisecond(1000));
            rt_kprintf("i2c_read err \n");
            continue;
        }
        rt_thread_delay(rt_tick_from_millisecond(50));
    }
}

int i2cBleThreadInit(void)
{
#ifdef I2C_TOOLS_USE_SW_I2C
    if (i2c_init(SDA_PORT_NUM, SCL_PORT_NUM))
    {
        rt_kprintf("[i2c] failed to find bus with sda=%d scl=%d\n", SDA_PORT_NUM, SCL_PORT_NUM);
        return RT_ERROR;
    }
#else
    char name[RT_NAME_MAX];
    rt_strncpy(name, I2C_DEVICE_NAME, RT_NAME_MAX);
    if (i2c_init(name))
    {
        rt_kprintf("[i2c] failed to find bus %s\n", name);
        return RT_ERROR;
    }

#endif
    /* 创建 i2c 线程 */
    rt_thread_t thread = rt_thread_create("bleKey", i2c_thread_entry, RT_NULL, 512, 8, 10);
    /* 创建成功则启动线程 */
    if (thread != RT_NULL)
    {
        rt_thread_startup(thread);
    }
    else
    {
        rt_kprintf("thread_create i2c err \n");
        return RT_ERROR;
    }
    return RT_EOK;
}
INIT_APP_EXPORT(i2cBleThreadInit);
