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

enum
{
    STATE_LED_OFF,
    STATE_LED_ON,
    STATE_LED_FLASH_2HZ,
    STATE_LED_FLASH_1HZ,
    STATE_LED_FLASH_0_5HZ,
};
extern _TKS_FLAGA_type keyState[4];
extern volatile _TKS_FLAGA_type keyTrg[4];

#define KEY1 keyState[0].bits.b0
#define KEY2 keyState[0].bits.b1
#define KEY3 keyState[0].bits.b2
#define KEY4 keyState[0].bits.b3

#define KEY1Restain keyState[2].bits.b0
#define KEY2Restain keyState[2].bits.b1
#define KEY3Restain keyState[2].bits.b2
#define KEY4Restain keyState[2].bits.b3

#define KEY1Trg keyTrg[0].bits.b0
#define KEY2Trg keyTrg[0].bits.b1
#define KEY3Trg keyTrg[0].bits.b2
#define KEY4Trg keyTrg[0].bits.b3

#define KEY1RestainTrg keyTrg[2].bits.b0
#define KEY2RestainTrg keyTrg[2].bits.b1
#define KEY3RestainTrg keyTrg[2].bits.b2
#define KEY4RestainTrg keyTrg[2].bits.b3

extern _USR_FLAGA_type ledState[5];
#define led1State ledState[0].s4bits.s0
#define led2State ledState[0].s4bits.s1
#define led3State ledState[1].s4bits.s0
#define led4State ledState[1].s4bits.s1
#define led5State ledState[2].s4bits.s0
#define led6State ledState[2].s4bits.s1
#define led7State ledState[3].s4bits.s0
#define led8State ledState[3].s4bits.s1
#define led9State ledState[4].s4bits.s0

#endif
