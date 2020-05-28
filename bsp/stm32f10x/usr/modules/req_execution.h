#ifndef __REQ_EXE_H__
#define __REQ_EXE_H__
#include "stdint.h"

enum
{
    COMPRESSOR_FSM_STATE_IDLE = 0,
    COMPRESSOR_FSM_STATE_INIT,
    COMPRESSOR_FSM_STATE_STARTUP,
    COMPRESSOR_FSM_STATE_NORMAL,
    COMPRESSOR_FSM_STATE_SHUTING,
    COMPRESSOR_FSM_STATE_STOP,
};

//风机状态机
enum
{
    FSM_FAN_IDLE = 0,
    FSM_FAN_INIT,
    FSM_FAN_START_UP,
    FSM_FAN_NORM,
    FSM_FAN_SHUT
};

/************************************************************************/

//出水模式
enum
{
    WATER_NO         = 0x00,
    WATER_NORMAL_ICE = 0x01,
    WATER_HEAT       = 0x02,
    WATER_ICE        = 0x04,  //冰水
    WATER_HEATPOT    = 0x08,  //热灌
};

//水源模式
enum
{
    WATER_AIR          = 0x00,
    WATER_FLUSH        = 0x01,  //
    WATER_FILL         = 0x02,  //注水
    WATER_Disinfection = 0x03,  //消毒
};

//水位
enum
{
    S_L  = 0x01,
    S_M  = 0x02,
    S_U  = 0x04,
    D_L  = 0x08,  //制水
    D_M  = 0x10,  //满水
    D_U  = 0x20,
    D_ML = 0x40,  //制冷水位,中水位
};

//水位
enum
{
    FILL_ZERO = 0,  //缺水
    FILL_L    = 1,  //低水
    FILL_ML   = 2,  //制冷水位,中水位
    FILL_M    = 3,  //满水
};
#define COUNT3S 6u
#define COUNT9M 1080u
#define COUNT20M 2400u
#define COUNT30M 3600u
#define COUNT3H 21600u
#define COUNT7D 1209600u
#define COUNT30D 5184000u
//水位
enum
{
    FLOATBALLIDEL = 0,  //浮球IDEL
    FLOATBALLL    = 1,  //浮球下
    FLOATBALLM    = 2,  //浮球中
    FLOATBALLH    = 4,  //浮球上
};

typedef enum
{
    COLLECTIDEL = 0,
    COLLECTHALF = 1,
    COLLECTFULL = 2,
} CollectState;
typedef enum
{
    TRANSCHAMBERIDEL   = 0,
    TRANSCHAMBERLOOP   = 1,
    TRANSCHAMBERINJECT = 2,
    TRANSCHAMBEREMPTY  = 3,
} ChamberState_t;
typedef enum
{
    DRINKTANKIDEL   = 0,
    DRINKTANKLOOP   = 1,
    DRINKTANKINJECT = 2,
    DRINKTANKEMPTY  = 3,
} DrinkTankState_t;

#define AUXILIARYDOZERO (uint16_t)0x0000
#define AUXILIARYDO_IN1OUT2EV3 (uint16_t)(1 << 8)
#define AUXILIARYDO_EV3 (uint16_t)(1 << 9)
#define AUXILIARYDO_EV4 (uint16_t)(1 << 10)
#define AUXILIARYDO_EV5 (uint16_t)(1 << 11)
#define AUXILIARYDO_SRCUV (uint16_t)(1 << 12)
#define AUXILIARYDO_PUMP1 (uint16_t)(1 << 13)
#define AUXILIARYDO_PUMP2 (uint16_t)(1 << 14)

#define AUXILIARYDI_EVFAUCET (uint16_t)0x0001
#define AUXILIARYDI_PUREKEY (uint16_t)0x0002

//出水状态
enum
{
    HEATER_IDLE = 0,
    HEATER_SEND,
    WATER_READ,
    WATER_OUT,
};

//定时保存时间
#define FIXED_SAVETIME 900
enum
{
    FAN_MODE_FIX = 0,        //定速模式
    FAN_MODE_PRESS_DIFF,     //压差模式
    FAN_MODE_AVR_RETURN,     //回风平均
    FAN_MODE_AVR_SUPPLY,     //送风平均
    FAN_MODE_TEMP_DIFF,      //温差平均
    FAN_MODE_MAX_RETURN,     //回风热点
    FAN_MODE_MAX_SUPPLY,     //送风热点
    FAN_MODE_TEMP_MAX_DIFF,  //温差热点
    FAM_MODE_INV_COMP,       //变频跟踪
};

//时间
#define INTERAL_TIME 24 * 3600 * 2  //间隔24小时
#define CLOSE_TIME 5

void hum_capacity_calc(void);
void req_execution(int16_t target_req_temp, int16_t target_req_hum);
void req_bitmap_op(uint8_t component_bpos, uint8_t action);
void Close_DIS_PWR(uint8_t u8Type);
void UV_req_exe(uint8_t u8Type);
uint8_t Sys_Get_Storage_Signal(void);

#define hotWaterKey KEY1

#endif  //__REQ_EXE_H__
