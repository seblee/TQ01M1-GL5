#ifndef __LEDKEY_OPT_H
#define __LEDKEY_OPT_H
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
    STATE_LED_OFF,
    STATE_LED_ON,
    STATE_LED_FLASH_2HZ,
    STATE_LED_FLASH_1HZ,
    STATE_LED_FLASH_0_5HZ,

};

enum
{
    IDELTEM,
    BOILINGTEM,
    NORMALTEM,
    TEATEM,
    MILKTEM,
};

extern _TKS_FLAGA_type keyState[4];
extern volatile _TKS_FLAGA_type keyTrg[4];

#define KEY0 keyState[1].bits.b2
#define KEY1 keyState[0].bits.b0
#define KEY2 keyState[1].bits.b1
#define KEY3 keyState[0].bits.b2
#define KEY4 keyState[1].bits.b0
#define KEY5 keyState[1].bits.b3
#define KEY6 keyState[0].bits.b1

#define KEY0Restain keyState[3].bits.b2
#define KEY1Restain keyState[2].bits.b0
#define KEY2Restain keyState[3].bits.b1
#define KEY3Restain keyState[2].bits.b2
#define KEY4Restain keyState[3].bits.b0
#define KEY5Restain keyState[3].bits.b3
#define KEY6Restain keyState[2].bits.b1

#define KEY0Trg keyTrg[1].bits.b2
#define KEY1Trg keyTrg[0].bits.b0
#define KEY2Trg keyTrg[1].bits.b1
#define KEY3Trg keyTrg[0].bits.b2
#define KEY4Trg keyTrg[1].bits.b0
#define KEY5Trg keyTrg[1].bits.b3
#define KEY6Trg keyTrg[0].bits.b1

#define KEY0RestainTrg keyTrg[3].bits.b2
#define KEY1RestainTrg keyTrg[2].bits.b0
#define KEY2RestainTrg keyTrg[3].bits.b1
#define KEY3RestainTrg keyTrg[2].bits.b2
#define KEY4RestainTrg keyTrg[3].bits.b0
#define KEY5RestainTrg keyTrg[3].bits.b3
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
#define led0State ledState[0].s4bits.s0
#define led1State ledState[0].s4bits.s1
#define led2State ledState[1].s4bits.s0
#define led3State ledState[1].s4bits.s1
#define led4State ledState[2].s4bits.s0
#define led5State ledState[2].s4bits.s1
#define led6State ledState[3].s4bits.s0
#define led7State ledState[3].s4bits.s1
#define led8State ledState[4].s4bits.s0
#define led9State ledState[4].s4bits.s1
#define led10State ledState[5].s4bits.s0
#define led11State ledState[5].s4bits.s1
#define led12State ledState[6].s4bits.s0
#define led13State ledState[6].s4bits.s1

#define childLockState led0State
#define boilingKeyState led1State
#define teaKeyState led2State
#define milkKeyState led3State
#define normalKeyState led4State
#define fetchKeyState led5State
#define cleanKeyState led6State
#define makeWaterGreenState led7State
#define loopBlueState led8State
#define loopGreenState led9State
#define cleanGreenState led10State
#define makeWaterBlueState led11State
#define loopRedState led12State
#define cleanBlueState led13State

extern _BEEP_STATE beepState;
#define BEEPMODE beepState.bits.mode
#define BEEPLONG beepState.bits.bits3
#define BEEPSHORT beepState.bits.bits4

#endif
