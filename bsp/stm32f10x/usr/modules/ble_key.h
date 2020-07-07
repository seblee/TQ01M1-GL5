#ifndef __BLE_KEY_H
#define __BLE_KEY_H
/**
 ****************************************************************************
 * @Warning Without permission from the author,Not for commercial use
 * @File
 * @Author  xiaowine@cee0.com
 * @date
 * @version V1.0
 *************************************************
 * @brief   -->>
 ****************************************************************************
 * @attention
 * Powered By Xiaowine
 * <h2><center>&copy;  Copyright(C) cee0.com 2015-2019</center></h2>
 * All rights reserved
 *
 **/

#include <rtthread.h>

typedef struct
{
    unsigned char b0 : 1;
    unsigned char b1 : 1;
    unsigned char b2 : 1;
    unsigned char b3 : 1;
    unsigned char b4 : 1;
    unsigned char b5 : 1;
    unsigned char b6 : 1;
    unsigned char b7 : 1;
} _FLAG_bits;

typedef union
{
    _FLAG_bits bits;
    unsigned char byte;
} _TKS_FLAGA_type;

typedef struct
{
    unsigned char s0 : 2;
    unsigned char s1 : 2;
    unsigned char s2 : 2;
    unsigned char s3 : 2;
} _STATE_2bits;

typedef struct
{
    unsigned char s0 : 4;
    unsigned char s1 : 4;
} _STATE_4bits;

typedef union
{
    _STATE_2bits s2bits;
    _STATE_4bits s4bits;
    unsigned char byte;
} _USR_FLAGA_type;
typedef struct
{
    unsigned char mode : 1;
    unsigned char bits3 : 3;
    unsigned char bits4 : 4;
} _BEEP_bits;
typedef union
{
    _BEEP_bits bits;
    unsigned char byte;
} _BEEP_STATE;
enum
{
    STATE_LED_OFF=0,
    STATE_LED_ON=1,
    STATE_LED_FLASH_2HZ=2,
    STATE_LED_FLASH_1HZ=3,
    STATE_LED_FLASH_0_5HZ=4,
    STATE_LED_FLASH_2_T=5,  //闪烁两下
};
enum
{
    IDELTEM,
    BOILINGTEM,
    NORMALTEM,
    TEATEM,
    MILKTEM,
    KEY_TEMP = 0x1000,
};

enum
{
    CMD_IDEL     = 0,
    CMD_KEY      = 1,
    CMD_LED      = 2,
    CMD_REG_UP   = 3,
    CMD_REG_DOWN = 4,
    CMD_AUX_DIAI = 5,
    CMD_AUX_DO   = 6,
    CMD_PARA     = 7,
    CMD_SEGMENT  = 8,
};
#define KEYBYTENUM 3
extern _TKS_FLAGA_type keyState[KEYBYTENUM];
extern _TKS_FLAGA_type keyUpState[KEYBYTENUM];
extern volatile _TKS_FLAGA_type keyTrg[KEYBYTENUM];
extern volatile _TKS_FLAGA_type keyUpTrg[KEYBYTENUM];

#define BLEON keyState[0].bits.b0
#define BLEONTrg keyTrg[0].bits.b0
#define PARAOK keyState[0].bits.b1
#define PARAOKTrg keyTrg[0].bits.b1

#define KEY0 keyState[1].bits.b4
#define KEY1 keyState[1].bits.b5
#define KEY2 keyState[1].bits.b3
#define KEY3 keyState[1].bits.b6
#define KEY4 keyState[1].bits.b2
#define KEY5 keyState[1].bits.b0
#define KEY6 keyState[1].bits.b1

#define KEY0Restain keyState[2].bits.b4
#define KEY1Restain keyState[2].bits.b5
#define KEY2Restain keyState[2].bits.b3
#define KEY3Restain keyState[2].bits.b6
#define KEY4Restain keyState[2].bits.b2
#define KEY5Restain keyState[2].bits.b0
#define KEY6Restain keyState[2].bits.b1

#define KEY0Trg keyTrg[1].bits.b4
#define KEY1Trg keyTrg[1].bits.b5
#define KEY2Trg keyTrg[1].bits.b3
#define KEY3Trg keyTrg[1].bits.b6
#define KEY4Trg keyTrg[1].bits.b2
#define KEY5Trg keyTrg[1].bits.b0
#define KEY6Trg keyTrg[1].bits.b1

#define KEY0RestainTrg keyTrg[2].bits.b4
#define KEY1RestainTrg keyTrg[2].bits.b5
#define KEY2RestainTrg keyTrg[2].bits.b3
#define KEY3RestainTrg keyTrg[2].bits.b6
#define KEY4RestainTrg keyTrg[2].bits.b2
#define KEY5RestainTrg keyTrg[2].bits.b0
#define KEY6RestainTrg keyTrg[2].bits.b1

#define boilingKey KEY0
#define normalKey KEY1
#define teaKey KEY2
#define cleanKey KEY3
#define milkKey KEY4
#define chlidKey KEY5
#define fetchKey KEY6

#define boilingKeyRestain KEY0Restain
#define normalKeyRestain KEY1Restain
#define teaKeyRestain KEY2Restain
#define cleanKeyRestain KEY3Restain
#define milkKeyRestain KEY4Restain
#define chlidKeyRestain KEY5Restain
#define fetchKeyRestain KEY6Restain

#define boilingKeyTrg KEY0Trg
#define normalKeyTrg KEY1Trg
#define teaKeyTrg KEY2Trg
#define cleanKeyTrg KEY3Trg
#define milkKeyTrg KEY4Trg
#define chlidKeyTrg KEY5Trg
#define fetchKeyTrg KEY6Trg

#define boilingKeyRestainTrg KEY0RestainTrg
#define normalKeyRestainTrg KEY1RestainTrg
#define teaKeyRestainTrg KEY2RestainTrg
#define cleanKeyRestainTrg KEY3RestainTrg
#define milkKeyRestainTrg KEY4RestainTrg
#define chlidKeyRestainTrg KEY5RestainTrg
#define fetchKeyRestainTrg KEY6RestainTrg

extern _USR_FLAGA_type ledState[7];
#define led00State ledState[0].s4bits.s0
#define led01State ledState[0].s4bits.s1
#define led02State ledState[1].s4bits.s0
#define led03State ledState[1].s4bits.s1
#define led04State ledState[2].s4bits.s0
#define led05State ledState[2].s4bits.s1
#define led06State ledState[3].s4bits.s0
#define led07State ledState[3].s4bits.s1
#define led08State ledState[4].s4bits.s0
#define led09State ledState[4].s4bits.s1
#define led10State ledState[5].s4bits.s0
#define led11State ledState[5].s4bits.s1
#define led12State ledState[6].s4bits.s0
#define led13State ledState[6].s4bits.s1

#define boilingKeyState led00State
#define normalKeyState led01State
#define teaKeyState led02State
#define cleanKeyState led03State
#define milkKeyState led04State
#define childLockState led05State
#define fetchKeyState led06State
#define makeWaterGreenState led07State
#define loopBlueState led08State
#define loopGreenState led09State
#define cleanGreenState led10State
#define makeWaterBlueState led11State
#define loopRedState led12State
#define cleanBlueState led13State
extern _BEEP_STATE beepState;
#define BEEPMODE beepState.bits.mode
#define BEEPLONG beepState.bits.bits3
#define BEEPSHORT beepState.bits.bits4
#endif
