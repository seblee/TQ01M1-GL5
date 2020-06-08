#include <rtthread.h>

#include "calc.h"
#include "sys_conf.h"
#include "local_status.h"
#include "req_execution.h"
#include "sys_status.h"
#include "dio_bsp.h"
#include "rtc_bsp.h"
#include <stdlib.h>
#include <math.h>
#include "global_var.h"
#include "pwm_bsp.h"
#include "usart_bsp.h"

#include "ledkey_opt.h"

#define CONFIG_DEBUG
#ifdef CONFIG_DEBUG
#ifndef req_exe_log
#define req_exe_log(N, ...) rt_kprintf("[%d][req_exe:%04d]" N "\r\n", rt_tick_get(), __LINE__, ##__VA_ARGS__)
#endif /* req_exe_log(...) */
#else
#define req_exe_log(...)
#endif /* ! CONFIG_DEBUG */

extern local_reg_st l_sys;
extern sys_reg_st g_sys;

/*******DO State*********/
static rt_uint8_t PUMP1OutPut = 0;
#define PUMP1DRINKTANK (rt_uint8_t)(1 << 0)
#define PUMP1LIFEWATEROUT (rt_uint8_t)(1 << 1)

static rt_uint8_t PUMP3OutPut = 0;
#define PUMP3COLLECT (rt_uint8_t)(1 << 0)
#define PUMP3PUREOUT (rt_uint8_t)(1 << 1)
#define PUMP3DRINKTANK (rt_uint8_t)(1 << 2)

static rt_uint8_t EV1OutPut = 0;
#define EV1COLLECT (rt_uint8_t)(1 << 0)
#define EV1DRINKTANK (rt_uint8_t)(1 << 1)

static rt_uint8_t EV2OutPut = 0;
#define EV2DRINKTANK (rt_uint8_t)(1 << 0)
#define EV2PUREWATEROUT (rt_uint8_t)(1 << 1)

static rt_uint8_t EV3OutPut = 0;
#define EV3COLLECT (rt_uint8_t)(1 << 0)
#define EV3DRINKTANK (rt_uint8_t)(1 << 1)
#define EV3TRANSFORM (rt_uint8_t)(1 << 2)

void j25OutPutDO(void)
{
    // PUMP1OutPut
    if (PUMP1OutPut)
    {
        l_sys.j25AuxiliaryBoardDO |= AUXILIARYDO_PUMP1;
    }
    else
    {
        l_sys.j25AuxiliaryBoardDO &= ~AUXILIARYDO_PUMP1;
    }
    // PUMP3OutPut
    if (PUMP3OutPut)
    {
        req_bitmap_op(DO_PUMP3_BPOS, 1);
    }
    else
    {
        req_bitmap_op(DO_PUMP3_BPOS, 0);
    }
    // EV1OutPut
    if (EV1OutPut)
    {
        req_bitmap_op(DO_EV1_BPOS, 1);
    }
    else
    {
        req_bitmap_op(DO_EV1_BPOS, 0);
    }
    // EV2OutPut
    if (EV2OutPut)
    {
        req_bitmap_op(DO_EV2_BPOS, 1);
    }
    else
    {
        req_bitmap_op(DO_EV2_BPOS, 0);
    }
    // EV3OutPut
    if (EV3OutPut)
    {
        l_sys.j25AuxiliaryBoardDO |= AUXILIARYDO_EV3;
    }
    else
    {
        l_sys.j25AuxiliaryBoardDO &= ~AUXILIARYDO_EV3;
    }
}

enum
{
    COMPRESSOR_SIG_HOLD = 0,
    COMPRESSOR_SIG_ON,
    COMPRESSOR_SIG_OFF,
    COMPRESSOR_SIG_ERR,
};

enum
{
    FAN_SIG_IDLE = 0,
    FAN_SIG_START,
    FAN_SIG_STOP
};

enum
{
    FAN_TPYE_AC = 0,
    FAN_TPYE_EC
};

//需求比特位操作函数
void req_bitmap_op(uint8_t component_bpos, uint8_t action)
{
    uint8_t byte_offset, bit_offset;

    byte_offset = component_bpos >> 4;
    bit_offset  = component_bpos & 0x0f;

    if (action == 0)
    {
        l_sys.bitmap[byte_offset][BITMAP_REQ] &= ~(0x0001 << bit_offset);
    }
    else
    {
        l_sys.bitmap[byte_offset][BITMAP_REQ] |= (0x0001 << bit_offset);
    }
}

//需求模拟输出操作函数
static void req_ao_op(uint8_t component_bpos, int16_t value)
{
    if ((g_sys.config.ComPara.u16Manual_Test_En == 0) && (g_sys.config.ComPara.u16Test_Mode_Type == 0))
    {  // normal mode
        switch (component_bpos)
        {
            case (AO_EC_FAN): {
                if (1)
                {
                    l_sys.ao_list[component_bpos][BITMAP_REQ] = value;
                }
                break;
            }
            case (AO_EC_FAN2): {
                break;
            }
            case (AO_EC_COMPRESSOR): {
                if (1)
                {
                    l_sys.ao_list[component_bpos][BITMAP_REQ] = value;
                }
                break;
            }
            default: {
                l_sys.ao_list[component_bpos][BITMAP_REQ] = value;
                break;
            }
        }
    }
    else
    {  // dianose or test mode, output directly
        l_sys.ao_list[component_bpos][BITMAP_REQ] = value;
    }
}

//模拟输出跟踪函数，向设置目标，按照步长参数进行变化；
static int16_t analog_step_follower(int16_t target, uint16_t dev_type)
{
    uint16_t current_output;
    int16_t ret_val;
    int16_t delta;

    switch (dev_type)
    {
        case (AO_EC_FAN):
        case (AO_EC_FAN2): {
            current_output = g_sys.status.aout[dev_type];
            delta          = target - current_output;
            if (abs(delta) > g_sys.config.fan.adjust_step)
            {
                if (delta > 0)
                    ret_val = current_output + g_sys.config.fan.adjust_step;
                else
                    ret_val = current_output - g_sys.config.fan.adjust_step;
            }
            else if (delta == 0)
            {
                ret_val = current_output;
            }
            else
            {
                ret_val = current_output + delta;
            }
            break;
        }
        default: {
            ret_val = target;
            break;
        }
    }
    return ret_val;
}

void Close_DIS_PWR(uint8_t u8Type)
{
    if (u8Type)
    {
        req_bitmap_op(DO_PWR_CTRL_BPOS, 1);  //关闭显示电源
    }
    else
    {
        req_bitmap_op(DO_PWR_CTRL_BPOS, 0);  //关闭显示电源
    }
    return;
}

//风机档位输出
static void Fan_Fsm_Out(uint8_t Fan_Gear)
{
    switch (Fan_Gear)
    {
        case FAN_GEAR_START:
            req_bitmap_op(DO_FAN_LOW_BPOS, 1);
            req_bitmap_op(DO_F24_BPOS, 1);
            break;
        case FAN_GEAR_NO:
            req_bitmap_op(DO_FAN_LOW_BPOS, 0);
            req_bitmap_op(DO_F24_BPOS, 0);
            break;
        default:
            req_bitmap_op(DO_FAN_LOW_BPOS, 0);
            req_bitmap_op(DO_F24_BPOS, 0);
            break;
    }
}

/**
 * @brief 	fan output control state FSM execution
 * @param  none
 * @retval none
 */
//风机状态机执行函数
static void fan_fsm_exe(uint8_t fan_signal)
{
    //		rt_kprintf("FAN_FSM_STATE = %d,fan_signal = %d\n",l_sys.l_fsm_state[FAN_FSM_STATE],fan_signal);
    if (get_alarm_bitmap(ACL_E7))  //风机告警
    {
        l_sys.l_fsm_state[FAN_FSM_STATE] = FSM_FAN_IDLE;
        l_sys.Fan.Fan_Gear               = FAN_GEAR_NO;  //
        l_sys.comp_timeout[DO_FAN_BPOS]  = 0;            // reset timeout counter
    }
    switch (l_sys.l_fsm_state[FAN_FSM_STATE])
    {
        case (FSM_FAN_IDLE): {
            if (fan_signal == FAN_SIG_START)
            {
                l_sys.l_fsm_state[FAN_FSM_STATE] = FSM_FAN_INIT;
                l_sys.comp_timeout[DO_FAN_BPOS] =
                    g_sys.config.ComPara.u16Start_Delay;  // assign startup delay to timeout counter
            }
            else
            {
                l_sys.l_fsm_state[FAN_FSM_STATE] = FSM_FAN_IDLE;
                l_sys.comp_timeout[DO_FAN_BPOS]  = l_sys.comp_timeout[DO_FAN_BPOS];  // remains timeout counter
            }
            l_sys.Fan.Fan_Gear = FAN_GEAR_NO;  //无输出						//disable fan output
            break;
        }
        case (FSM_FAN_INIT): {
            if (fan_signal != FAN_SIG_START)
            {
                l_sys.l_fsm_state[FAN_FSM_STATE] = FSM_FAN_IDLE;
                l_sys.Fan.Fan_Gear               = FAN_GEAR_NO;                      //无输出
                l_sys.comp_timeout[DO_FAN_BPOS]  = l_sys.comp_timeout[DO_FAN_BPOS];  // reset timeout counter
            }
            else
            {
                if (l_sys.comp_timeout[DO_FAN_BPOS] == 0)
                {
                    l_sys.l_fsm_state[FAN_FSM_STATE] = FSM_FAN_START_UP;
                    //									  l_sys.comp_timeout[DO_FAN_BPOS] =
                    // g_sys.config.fan.cold_start_delay;
                    l_sys.comp_timeout[DO_FAN_BPOS] = 0;
                    l_sys.Fan.Fan_Gear              = FAN_GEAR_START;  //
                }
                else  // wait until startup delay elapses
                {
                    l_sys.l_fsm_state[FAN_FSM_STATE] = FSM_FAN_INIT;
                    l_sys.Fan.Fan_Gear               = FAN_GEAR_NO;  //无输出
                }
            }
            break;
        }
        case (FSM_FAN_START_UP): {
            if (l_sys.comp_timeout[DO_FAN_BPOS] == 0)
            {
                l_sys.l_fsm_state[FAN_FSM_STATE] = FSM_FAN_NORM;
            }
            else
            {
                l_sys.l_fsm_state[FAN_FSM_STATE] = FSM_FAN_START_UP;
            }
            l_sys.Fan.Fan_Gear              = FAN_GEAR_START;                   //
            l_sys.comp_timeout[DO_FAN_BPOS] = l_sys.comp_timeout[DO_FAN_BPOS];  // remain timeout counter
            break;
        }
        case (FSM_FAN_NORM): {
            if ((fan_signal == FAN_SIG_STOP) && (l_sys.comp_timeout[DO_FAN_BPOS] == 0))
            {
                l_sys.l_fsm_state[FAN_FSM_STATE] = FSM_FAN_SHUT;
                l_sys.comp_timeout[DO_FAN_BPOS] =
                    g_sys.config.ComPara.u16Fan_Stop_Delay;  // assign startup delay to timeout counter
            }
            else
            {
                l_sys.l_fsm_state[FAN_FSM_STATE] = FSM_FAN_NORM;
                l_sys.comp_timeout[DO_FAN_BPOS]  = l_sys.comp_timeout[DO_FAN_BPOS];  // reset timeout counter
            }
            l_sys.Fan.Fan_Gear = FAN_GEAR_START;  //

            break;
        }
        case (FSM_FAN_SHUT): {
            if (fan_signal == FAN_SIG_START)
            {
                l_sys.l_fsm_state[FAN_FSM_STATE] = FSM_FAN_NORM;
                l_sys.Fan.Fan_Gear               = FAN_GEAR_START;                   //
                l_sys.comp_timeout[DO_FAN_BPOS]  = l_sys.comp_timeout[DO_FAN_BPOS];  // reset timeout counter
            }
            else
            {
                if (l_sys.comp_timeout[DO_FAN_BPOS] == 0)
                {
                    l_sys.l_fsm_state[FAN_FSM_STATE] = FSM_FAN_IDLE;
                    l_sys.Fan.Fan_Gear =
                        FAN_GEAR_NO;                                                    //																			//enable
                                                                                        // fan output
                    l_sys.comp_timeout[DO_FAN_BPOS] = l_sys.comp_timeout[DO_FAN_BPOS];  // reset timeout counter
                }
                else  // wait until startup delay elapses
                {
                    l_sys.l_fsm_state[FAN_FSM_STATE] = FSM_FAN_SHUT;
                    l_sys.comp_timeout[DO_FAN_BPOS]  = l_sys.comp_timeout[DO_FAN_BPOS];  // remain timeout counter
                }
            }
            break;
        }
        default: {
            l_sys.l_fsm_state[FAN_FSM_STATE] = FSM_FAN_IDLE;
            l_sys.Fan.Fan_Gear               = FAN_GEAR_NO;  //
            l_sys.comp_timeout[DO_FAN_BPOS]  = 0;            // reset timeout counter
            break;
        }
    }

    //		if(g_sys.config.fan.type == FAN_TPYE_EC)
    {
        if ((sys_get_pwr_sts() != 0) && (l_sys.Fan.Fan_Gear != FAN_GEAR_NO))
        {
            l_sys.Fan.Fan_Gear = FAN_GEAR_LOW;  //风机低挡
        }
    }
    //		rt_kprintf("Fan_Gear = %d\n",l_sys.Fan.Fan_Gear);
    Fan_Fsm_Out(l_sys.Fan.Fan_Gear);  //风机DO输出
}

#define EC_FAN_NOLOAD_DELAY 20
static void ec_fan_output(int16_t req_temp, int16_t req_hum, uint8_t fan_sig)
{
    uint16_t status_sense;
    uint16_t require;
    uint16_t fan_mode;
    uint16_t u16Fan_Speed;
    // static uint16_t ec_fan_shut_delay = 0;

    fan_mode = g_sys.config.fan.mode;
    //#ifdef SYS_M_V50
    if ((g_sys.config.ComPara.LN.u16LN_Mode) || (g_sys.config.ComPara.LN.u16LN_Enable))  //低噪音,低速制水
    {
        u16Fan_Speed = g_sys.config.ComPara.LN.u16LN_Fan;
    }
    else
    {
        u16Fan_Speed = g_sys.config.fan.set_speed;
    }
    //#else
    //		u16Fan_Speed = g_sys.config.fan.set_speed;
    //#endif
    require = 0;

    {
        if (l_sys.Fan.Fan_Gear == FAN_GEAR_NO)  //未开风机
        {
            require = 0;
        }
        else if (g_sys.config.fan.type == FAN_TPYE_EC)
        {
            if (l_sys.l_fsm_state[FAN_FSM_STATE] == FSM_FAN_START_UP)
            {
                require = u16Fan_Speed;
            }
            else
            {
                // fan mode in invertor compressor mode
                if (fan_mode == FAM_MODE_INV_COMP)  //变速
                {
                    require = analog_step_follower(req_temp, AO_EC_FAN);
                    // ec_fan_shut_delay = EC_FAN_NOLOAD_DELAY;
                }
                else if (fan_mode == FAN_MODE_FIX)  //定速
                {
                    require = u16Fan_Speed;
                }
                else
                {
                    if ((g_sys.config.fan.noload_down != 0) && (status_sense == 0))
                    {
                        require = u16Fan_Speed;
                    }
                    else
                    {
                        require = u16Fan_Speed;
                    }
                }
            }
            require = lim_min_max(g_sys.config.fan.min_speed, g_sys.config.fan.max_speed, require);
        }
        else
        {
            require = u16Fan_Speed;
        }
    }
    req_ao_op(AO_EC_FAN, require);
    req_ao_op(AO_EC_FAN2, require);
}

/**************************************/
//风机状态机信号产生电路
static uint8_t fan_signal_gen(void)
{
    uint8_t fan_signal;

    fan_signal = 0;
    if ((sys_get_pwr_sts() == 1) && (l_sys.Fan_Close == 0x00))
    {
        fan_signal = FAN_SIG_START;
    }
    else if ((sys_get_pwr_sts() == 0) || (l_sys.Fan_Close != 0x00))
    {
        //			if((g_sys.status.ComSta.u16Dout_bitmap&(~((0x0001<<DO_FAN_BPOS)|(0x0001<<DO_ALARM_BPOS)|(0x0001<<DO_RH1_BPOS)
        //				|(0x0001<<DO_LAMP_BPOS)|(0x0001<<DO_HWP_BPOS)|(0x0001<<DO_PWP_BPOS)))) == 0)
        if ((g_sys.status.ComSta.u16Dout_bitmap[0] & ((0x0001 << DO_COMP1_BPOS) | (0x0001 << DO_COMP2_BPOS))) == 0)
        {
            fan_signal = FAN_SIG_STOP;
        }
        else
        {
            fan_signal = FAN_SIG_IDLE;
        }
    }
    else
    {
        fan_signal = FAN_SIG_IDLE;
    }
    //		rt_kprintf("fan_signal=%x,Fan_Close=%d\n",fan_signal,l_sys.Fan_Close);
    return fan_signal;
}

//制水
void fan_req_exe(int16_t target_req_temp, int16_t target_req_hum)
{
    uint8_t fan_signal;
    fan_signal = fan_signal_gen();                               //风机状态机信号产生
    fan_fsm_exe(fan_signal);                                     //风机状态机执行
    ec_fan_output(target_req_temp, target_req_hum, fan_signal);  //风机模拟输出控制
}

static uint16_t compressor_signal_gen(int16_t req_temp, int16_t req_hum, uint8_t *comp1_sig, uint8_t *comp2_sig)
{
    uint8_t comp1_alarm_flag, comp2_alarm_flag;
    uint16_t compressor_count;

    if ((g_sys.config.general.alarm_bypass_en == 0) &&
        ((l_sys.bitmap[0][BITMAP_REQ] & (0x0001 << DO_COMP1_BPOS)) != 0) &&
        ((l_sys.bitmap[0][BITMAP_ALARM] & (0x0001 << DO_COMP1_BPOS)) == 0) &&
        ((l_sys.bitmap[0][BITMAP_MASK] & (0x0001 << DO_COMP1_BPOS)) != 0))
        comp1_alarm_flag = 1;
    else
        comp1_alarm_flag = 0;

    if ((g_sys.config.general.alarm_bypass_en == 0) &&
        ((l_sys.bitmap[0][BITMAP_REQ] & (0x0001 << DO_COMP2_BPOS)) != 0) &&
        ((l_sys.bitmap[0][BITMAP_ALARM] & (0x0001 << DO_COMP2_BPOS)) == 0) &&
        ((l_sys.bitmap[0][BITMAP_MASK] & (0x0001 << DO_COMP2_BPOS)) != 0))
        comp2_alarm_flag = 1;
    else
        comp2_alarm_flag = 0;

    compressor_count = devinfo_get_compressor_cnt();

    if (sys_get_do_sts(DO_FAN_BPOS) == 0)  // fan disabled, emergency shutdown
    {
        *comp1_sig                                = COMPRESSOR_SIG_ERR;
        *comp2_sig                                = COMPRESSOR_SIG_ERR;
        l_sys.l_fsm_state[COMPRESS_SIG_FSM_STATE] = 0;
        return 0;
    }

    if ((sys_get_pwr_sts() == 0) || ((l_sys.Comp_Close[0] != 0x00) && (l_sys.Comp_Close[1] != 0x00)))
    {
        *comp1_sig                                = COMPRESSOR_SIG_OFF;
        *comp2_sig                                = COMPRESSOR_SIG_OFF;
        l_sys.l_fsm_state[COMPRESS_SIG_FSM_STATE] = 0;
        return 0;
    }

    if ((sys_get_remap_status(WORK_MODE_STS_REG_NO, FAN_STS_BPOS) != 0))
    {
        if (compressor_count == 1)  // one compressor configured
        {
            if (get_alarm_bitmap(ACL_E1) || get_alarm_bitmap(ACL_E2))  //告警
            {
                *comp1_sig = COMPRESSOR_SIG_OFF;
                *comp2_sig = COMPRESSOR_SIG_OFF;
            }
            else
            {
                *comp1_sig = COMPRESSOR_SIG_ON;
                *comp2_sig = COMPRESSOR_SIG_OFF;
            }
            l_sys.l_fsm_state[COMPRESS_SIG_FSM_STATE] = 0;
        }
        else if (compressor_count == 2)  // two compressors configured
        {
            switch (l_sys.l_fsm_state[COMPRESS_SIG_FSM_STATE])
            {
                case (0): {
                    *comp1_sig = COMPRESSOR_SIG_OFF;
                    *comp2_sig = COMPRESSOR_SIG_OFF;
                    if ((get_alarm_bitmap(ACL_E1) || get_alarm_bitmap(ACL_E2)) ||  //水满告警
                        ((comp1_alarm_flag & comp2_alarm_flag) != 0) ||            //告警
                        ((sys_get_remap_status(WORK_MODE_STS_REG_NO, DEFROST1_STS_BPOS) != 0) &&
                         (sys_get_remap_status(WORK_MODE_STS_REG_NO, DEFROST2_STS_BPOS) != 0)))  //除霜
                    {
                        l_sys.l_fsm_state[COMPRESS_SIG_FSM_STATE] = 0;
                    }
                    else
                    {
                        l_sys.l_fsm_state[COMPRESS_SIG_FSM_STATE] = 1;
                    }
                    break;
                }
                case (1): {
                    *comp1_sig = COMPRESSOR_SIG_ON;
                    *comp2_sig = COMPRESSOR_SIG_ON;

                    if ((get_alarm_bitmap(ACL_E1) || get_alarm_bitmap(ACL_E2)) ||  //水满告警
                        ((comp1_alarm_flag & comp2_alarm_flag) != 0) ||            //告警
                        ((sys_get_remap_status(WORK_MODE_STS_REG_NO, DEFROST1_STS_BPOS) != 0) &&
                         (sys_get_remap_status(WORK_MODE_STS_REG_NO, DEFROST2_STS_BPOS) != 0)))  //除霜
                    {
                        l_sys.l_fsm_state[COMPRESS_SIG_FSM_STATE] = 0;
                    }
                    else if ((comp1_alarm_flag == 1) ||
                             (sys_get_remap_status(WORK_MODE_STS_REG_NO, DEFROST1_STS_BPOS) != 0))  // alarm alternation
                    {
                        l_sys.l_fsm_state[COMPRESS_SIG_FSM_STATE] = 1;
                        *comp1_sig                                = COMPRESSOR_SIG_OFF;
                    }
                    else if ((comp2_alarm_flag == 1) ||
                             (sys_get_remap_status(WORK_MODE_STS_REG_NO, DEFROST2_STS_BPOS) != 0))  // alarm alternation
                    {
                        l_sys.l_fsm_state[COMPRESS_SIG_FSM_STATE] = 1;
                        *comp2_sig                                = COMPRESSOR_SIG_OFF;
                    }
                    else
                    {
                        l_sys.l_fsm_state[COMPRESS_SIG_FSM_STATE] = 1;
                    }
                    break;
                }
                default: {
                    *comp1_sig                                = COMPRESSOR_SIG_OFF;
                    *comp2_sig                                = COMPRESSOR_SIG_OFF;
                    l_sys.l_fsm_state[COMPRESS_SIG_FSM_STATE] = 0;
                    break;
                }
            }
        }
        else
        {
            *comp1_sig                                = COMPRESSOR_SIG_OFF;
            *comp2_sig                                = COMPRESSOR_SIG_OFF;
            l_sys.l_fsm_state[COMPRESS_SIG_FSM_STATE] = 0;
        }
    }
    else
    {
        *comp1_sig                                = COMPRESSOR_SIG_OFF;
        *comp2_sig                                = COMPRESSOR_SIG_OFF;
        l_sys.l_fsm_state[COMPRESS_SIG_FSM_STATE] = 0;
    }
    return 1;
}

void compressor_alarm_signal_gen(uint8_t *comp1_sig, uint8_t *comp2_sig)
{
    if ((get_alarm_bitmap_mask(DO_COMP1_BPOS) == 1) && (get_alarm_bitmap_op(DO_COMP1_BPOS) == 0))
    {
        *comp1_sig = COMPRESSOR_SIG_OFF;
        if ((l_sys.l_fsm_state[COMPRESS1_FSM_STATE] != COMPRESSOR_FSM_STATE_STOP) &&
            (l_sys.l_fsm_state[COMPRESS1_FSM_STATE] != COMPRESSOR_FSM_STATE_IDLE))
        {
            l_sys.l_fsm_state[COMPRESS1_FSM_STATE] = COMPRESSOR_FSM_STATE_STOP;
        }
    }

    if ((get_alarm_bitmap_mask(DO_COMP2_BPOS) == 1) && (get_alarm_bitmap_op(DO_COMP2_BPOS) == 0))
    {
        *comp2_sig = COMPRESSOR_SIG_OFF;
        if ((l_sys.l_fsm_state[COMPRESS2_FSM_STATE] != COMPRESSOR_FSM_STATE_STOP) &&
            (l_sys.l_fsm_state[COMPRESS2_FSM_STATE] != COMPRESSOR_FSM_STATE_IDLE))
        {
            l_sys.l_fsm_state[COMPRESS2_FSM_STATE] = COMPRESSOR_FSM_STATE_STOP;
        }
    }
}

//压缩机状态机函数
static void compressor_fsm(uint8_t compressor_id, uint8_t signal)
{
    uint16_t compress_fsm_state;

    uint8_t l_fsm_state_id;
    uint8_t do_bpos;

    if (compressor_id == 0)
    {
        l_fsm_state_id = COMPRESS1_FSM_STATE;
        do_bpos        = DO_COMP1_BPOS;
    }
    else
    {
        l_fsm_state_id = COMPRESS2_FSM_STATE;
        do_bpos        = DO_COMP2_BPOS;
    }

    compress_fsm_state = l_sys.l_fsm_state[l_fsm_state_id];

    //		rt_kprintf("compressor_id=%d,signal=%d,,compress_fsm_state=%d,l_sys.Comp_Close=%d,l_sys.comp_startup_interval=%d\n",compressor_id,signal,compress_fsm_state,l_sys.Comp_Close,l_sys.comp_startup_interval);

    switch (compress_fsm_state)
    {
        case (COMPRESSOR_FSM_STATE_IDLE): {
            if ((signal == COMPRESSOR_SIG_ON) && (l_sys.comp_timeout[do_bpos] == 0))
            {
                l_sys.l_fsm_state[l_fsm_state_id] = COMPRESSOR_FSM_STATE_INIT;
                l_sys.comp_timeout[do_bpos]       = g_sys.config.compressor.startup_delay;

                req_bitmap_op(do_bpos, 0);
            }
            else
            {
                l_sys.l_fsm_state[l_fsm_state_id] = COMPRESSOR_FSM_STATE_IDLE;
                l_sys.comp_timeout[do_bpos]       = l_sys.comp_timeout[do_bpos];
                req_bitmap_op(do_bpos, 0);
            }
            break;
        }
        case (COMPRESSOR_FSM_STATE_INIT): {
            if (signal != COMPRESSOR_SIG_ON)
            {
                l_sys.l_fsm_state[l_fsm_state_id] = COMPRESSOR_FSM_STATE_IDLE;
                l_sys.comp_timeout[do_bpos]       = 0;
                req_bitmap_op(do_bpos, 0);
            }
            else if ((signal == COMPRESSOR_SIG_ON) && (l_sys.comp_timeout[do_bpos] == 0))
            {
                if (l_sys.comp_startup_interval == 0)
                {
                    l_sys.l_fsm_state[l_fsm_state_id] = COMPRESSOR_FSM_STATE_STARTUP;
                    l_sys.comp_timeout[do_bpos]       = g_sys.config.compressor.startup_lowpress_shield;
                    req_bitmap_op(do_bpos, 1);
                    l_sys.comp_startup_interval = g_sys.config.ComPara.u16Comp_Interval;
                }
                else
                {
                    l_sys.l_fsm_state[l_fsm_state_id] = COMPRESSOR_FSM_STATE_INIT;
                    l_sys.comp_timeout[do_bpos]       = l_sys.comp_timeout[do_bpos];
                    req_bitmap_op(do_bpos, 0);
                }
            }
            else
            {
                l_sys.l_fsm_state[l_fsm_state_id] = COMPRESSOR_FSM_STATE_INIT;
                l_sys.comp_timeout[do_bpos]       = l_sys.comp_timeout[do_bpos];
                req_bitmap_op(do_bpos, 0);
            }
            break;
        }
        case (COMPRESSOR_FSM_STATE_STARTUP): {
            if (signal == COMPRESSOR_SIG_ERR)
            {
                l_sys.l_fsm_state[l_fsm_state_id] = COMPRESSOR_FSM_STATE_STOP;
            }
            else if (l_sys.comp_timeout[do_bpos] == 0)
            {
                l_sys.l_fsm_state[l_fsm_state_id] = COMPRESSOR_FSM_STATE_NORMAL;
                l_sys.comp_timeout[do_bpos] =
                    (g_sys.config.compressor.min_runtime > g_sys.config.compressor.startup_lowpress_shield)
                        ? (g_sys.config.compressor.min_runtime - g_sys.config.compressor.startup_lowpress_shield)
                        : 0;
                req_bitmap_op(do_bpos, 1);
            }
            else
            {
                l_sys.l_fsm_state[l_fsm_state_id] = COMPRESSOR_FSM_STATE_STARTUP;
                l_sys.comp_timeout[do_bpos]       = l_sys.comp_timeout[do_bpos];
                req_bitmap_op(do_bpos, 1);
            }
            break;
        }
        case (COMPRESSOR_FSM_STATE_NORMAL): {
            if (signal == COMPRESSOR_SIG_ERR)
            {
                l_sys.l_fsm_state[l_fsm_state_id] = COMPRESSOR_FSM_STATE_STOP;
            }
            else if ((signal == COMPRESSOR_SIG_OFF) && (l_sys.comp_timeout[do_bpos] == 0))
            {
                l_sys.l_fsm_state[l_fsm_state_id] = COMPRESSOR_FSM_STATE_SHUTING;
                l_sys.comp_timeout[do_bpos]       = g_sys.config.compressor.stop_delay;
                req_bitmap_op(do_bpos, 1);
            }
            else
            {
                l_sys.l_fsm_state[l_fsm_state_id] = COMPRESSOR_FSM_STATE_NORMAL;
                l_sys.comp_timeout[do_bpos]       = l_sys.comp_timeout[do_bpos];
                req_bitmap_op(do_bpos, 1);
            }
            break;
        }
        case (COMPRESSOR_FSM_STATE_SHUTING): {
            if (signal == COMPRESSOR_SIG_ERR)
            {
                l_sys.l_fsm_state[l_fsm_state_id] = COMPRESSOR_FSM_STATE_STOP;
            }
            else if ((signal == COMPRESSOR_SIG_OFF) && (l_sys.comp_timeout[do_bpos] == 0))
            {
                l_sys.l_fsm_state[l_fsm_state_id] = COMPRESSOR_FSM_STATE_STOP;
            }
            else if (signal == COMPRESSOR_SIG_ON)
            {
                l_sys.l_fsm_state[l_fsm_state_id] = COMPRESSOR_FSM_STATE_NORMAL;
                l_sys.comp_timeout[do_bpos]       = 0;
                req_bitmap_op(do_bpos, 1);
            }
            else
            {
                l_sys.l_fsm_state[l_fsm_state_id] = COMPRESSOR_FSM_STATE_SHUTING;
                l_sys.comp_timeout[do_bpos]       = l_sys.comp_timeout[do_bpos];
                req_bitmap_op(do_bpos, 1);
            }
            break;
        }
        case (COMPRESSOR_FSM_STATE_STOP): {
            //						l_sys.l_fsm_state[l_fsm_state_id] = COMPRESSOR_FSM_STATE_IDLE;
            //						l_sys.comp_timeout[do_bpos] = g_sys.config.compressor.min_stoptime;
            //						req_bitmap_op(do_bpos,0);
            if (l_sys.comp_stop_interval == 0)
            {
                l_sys.l_fsm_state[l_fsm_state_id] = COMPRESSOR_FSM_STATE_IDLE;
                //									l_sys.comp_timeout[do_bpos] =
                // g_sys.config.compressor.min_stoptime;//取消最短停机时间
                l_sys.comp_timeout[do_bpos] = g_sys.config.ComPara.u16Start_Delay;  //启动延时
                req_bitmap_op(do_bpos, 0);
                l_sys.comp_stop_interval = g_sys.config.ComPara.u16Comp_Interval;
            }
            else
            {
                l_sys.l_fsm_state[l_fsm_state_id] = COMPRESSOR_FSM_STATE_STOP;
            }
            break;
        }
    }
}

// compressor requirement execution
static void compressor_req_exe(int16_t req_temp, int16_t req_hum)
{
    uint8_t comp1_sig, comp2_sig;
    compressor_signal_gen(req_temp, req_hum, &comp1_sig, &comp2_sig);  // FSM signal generation
    compressor_alarm_signal_gen(&comp1_sig, &comp2_sig);               // compressors alarm

    compressor_fsm(0, comp1_sig);  // compressor 1 FSM execution
    compressor_fsm(1, comp2_sig);  // compressor 2 FSM execution
}

#define POWERTIME 5400
#define THINTERVAL 180
#define CF_DELAY 60
enum
{
    FC_WL = 0x01,
    FC_PT = 0x02,
    FC_TH = 0x04,
    FC_WS = 0x08,
    FC_LW = 0x10,
    FC_AL = 0x80,  //告警
};

uint8_t Sys_Get_Storage_Signal(void)
{
    uint8_t ret;

    if (g_sys.config.ComPara.u16Storage == 1)
    {
        ret = TRUE;
    }
    else
    {
        ret = FALSE;
    }
    return ret;
}

//显示器控制
uint8_t Close_DIS_Enable(void)
{
    //禁止重启
    if ((g_sys.config.Platform.Restart_Enable & 0x8000) && (sys_get_di_sts(DI_HI_PRESS2_BPOS)))
    {
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}

//显示器控制
void Restart_DIS_exe(void)
{
    static uint8_t u8CloseTime[2]  = {0};
    static uint16_t u16NeterrDelay = 0;
    static uint32_t u32RSInteral   = 0;
    static uint8_t u8Restart       = 0;
    uint16_t u16MaxTimes;
    uint16_t u16HoldTime;
    uint32_t u32InteralTime;

    u16MaxTimes = g_sys.config.Platform.Restart_Enable & 0x0FFF;
    //		u16MaxTimes=3;
    u32InteralTime = (g_sys.config.Platform.Restart_Delay >> 8) * 3600 * 2;
    u16HoldTime    = (g_sys.config.Platform.Restart_Delay & 0x00FF) * 60 * 2;
    // 		rt_kprintf("u8CloseTime[1]=%d,u32RSInteral=%d\n", u8CloseTime[1],u32RSInteral);
    //		u16HoldTime=100;

    //禁止重启
    if (Close_DIS_Enable() == FALSE)
    {
        u8CloseTime[1]           = 0;
        u32RSInteral             = 0;
        l_sys.u16Uart_Timeout[1] = 0;
        u16NeterrDelay           = 0;
        u8Restart                = 0;
        l_sys.u8RSInteral_Neterr = FALSE;
        Close_DIS_PWR(0);  //开启
        return;
    }
    else
    {
        //超过次数
        if (u8CloseTime[1] >= u16MaxTimes)
        {
            u32RSInteral++;
            if (u32RSInteral >= u32InteralTime)  //间隔时间
            {
                u32RSInteral   = 0;
                u8CloseTime[1] = 0;
            }

            l_sys.u8RSInteral_Neterr = TRUE;  //网络异常
            l_sys.u16Uart_Timeout[1] = 0;
            u16NeterrDelay           = 0;
            u8Restart                = 0;
            Close_DIS_PWR(0);
            return;
        }
        else
        {
            // APP可能退出，通信异常
            l_sys.u16Uart_Timeout[1]++;
            if (l_sys.u16Uart_Timeout[1] >= u16HoldTime)
            {
                l_sys.u16Uart_Timeout[1] = u16HoldTime;
                u8Restart |= 0x01;
            }

            if (g_sys.config.Platform.Net_ERR != 0)  //网络异常
            {
                u16NeterrDelay++;
                if (u16NeterrDelay >= u16HoldTime)
                {
                    u16NeterrDelay = u16HoldTime;
                    u8Restart |= 0x03;
                }
            }

            l_sys.u8RSInteral_Neterr = FALSE;
        }
        //需要重启
        if (u8Restart)
        {
            u8CloseTime[0]++;
            if (u8CloseTime[0] <= CLOSE_TIME)
            {
                Close_DIS_PWR(1);  //关闭XX秒
            }
            else
            {
                u8CloseTime[0] = 0;
                u8CloseTime[1]++;  //次数
                l_sys.u16Uart_Timeout[1]      = 0;
                u16NeterrDelay                = 0;
                g_sys.config.Platform.Net_ERR = 0;
                u8Restart                     = 0;
            }
        }
        else
        {
            Close_DIS_PWR(0);
        }
    }
    // 		rt_kprintf("u8CloseTime[0]=%x,u8CloseTime[1]=%x,u16Uart_Timeout[1]=%x,u8Restarterr=%d,DI_HI_PRESS2=%d\n",
    // u8CloseTime[0],u8CloseTime[1],l_sys.u16Uart_Timeout[1],u8Restart, sys_get_di_sts(DI_HI_PRESS2_BPOS));

    return;
}
uint8_t j25GetFloatBall1(void)
{
    uint8_t Water_level = 0;
    if (sys_get_di_sts(DI_FLOAT1_H_BPOS) == 0)
    {
        Water_level |= FLOATBALLH;
    }

    if (sys_get_di_sts(DI_FLOAT1_L_BPOS) == 0)
    {
        Water_level |= FLOATBALLL;
    }

    return Water_level;
}
uint8_t j25GetFloatBall2(void)
{
    uint8_t Water_level = 0;
    if (sys_get_di_sts(DI_FLOAT2_H_BPOS) == 0)
    {
        Water_level |= FLOATBALLH;
    }
    if (sys_get_di_sts(DI_FLOAT2_L_BPOS) == 0)
    {
        Water_level |= FLOATBALLL;
    }
    return Water_level;
}
uint8_t j25GetFloatBall3(void)
{
    uint8_t Water_level = 0;
    if (sys_get_di_sts(DI_FLOAT3_H_BPOS) == 0)
    {
        Water_level |= FLOATBALLH;
    }
    if (sys_get_di_sts(DI_FLOAT3_M_BPOS) == 0)
    {
        Water_level |= FLOATBALLM;
    }
    if (sys_get_di_sts(DI_FLOAT3_L_BPOS) == 0)
    {
        Water_level |= FLOATBALLL;
    }

    return Water_level;
}

void j25WaterCollectStateSet(CollectState state)
{
    l_sys.j25WaterCollectState = state;
}
void j25WaterCollect(void)
{
    static uint16_t collectHalfCounter = 0;
    uint8_t waterLevel2                = j25GetFloatBall2();
    switch (l_sys.j25WaterCollectState)
    {
        case COLLECTIDEL:

            l_sys.j25WaterCollectTime = 0;
            EV1OutPut &= ~EV1COLLECT;
            PUMP3OutPut &= ~PUMP3COLLECT;
            collectHalfCounter = 0;
            break;
        case COLLECTHALF:
            if ((l_sys.j25WaterCollectTime < 0xffff) && (waterLevel2 & FLOATBALLH))
            {
                l_sys.j25WaterCollectTime++;
            }
            if (waterLevel2 & FLOATBALLL)
            {
                collectHalfCounter = 0;
                EV1OutPut |= EV1COLLECT;
                PUMP3OutPut |= PUMP3COLLECT;
            }
            else
            {
                if (collectHalfCounter < COUNT3S)
                {
                    collectHalfCounter++;
                    EV1OutPut |= EV1COLLECT;
                    PUMP3OutPut |= PUMP3COLLECT;
                }
                else
                {
                    EV1OutPut &= ~EV1COLLECT;
                    PUMP3OutPut &= ~PUMP3COLLECT;
                    l_sys.j25WaterCollectState = COLLECTIDEL;
                }
            }
            break;
        case COLLECTFULL:
            if ((l_sys.j25WaterCollectTime < 0xffff) && (waterLevel2 & FLOATBALLH))
            {
                l_sys.j25WaterCollectTime++;
            }
            collectHalfCounter = 0;
            if (waterLevel2 & FLOATBALLH)
            {
                EV1OutPut |= EV1COLLECT;
                PUMP3OutPut |= PUMP3COLLECT;
            }
            if ((waterLevel2 & FLOATBALLL) == 0)
            {
                EV1OutPut &= ~EV1COLLECT;
                PUMP3OutPut &= ~PUMP3COLLECT;
            }
            break;
        default:
            break;
    }
}

void j25DrinkTankStateSet(DrinkTankState_t state)
{
    l_sys.j25DrinkTankState = state;
}
void j25DrinkTank(void)
{
    static rt_uint32_t j25DrinkTankLoopCount    = 0;
    static rt_uint16_t j25DrinkTankEmptyCounter = 0;
    uint8_t waterLevel2                         = j25GetFloatBall2();
    uint8_t waterLevel3                         = j25GetFloatBall3();
    switch (l_sys.j25DrinkTankState)
    {
        case DRINKTANKIDEL:
            break;
        case DRINKTANKLOOP:
            if (j25DrinkTankLoopCount < COUNT3H)
            {
                j25DrinkTankLoopCount++;
                EV2OutPut &= ~EV2DRINKTANK;
                PUMP3OutPut &= ~PUMP3DRINKTANK;
            }
            else if (j25DrinkTankLoopCount < (COUNT3H + COUNT20M))
            {
                j25DrinkTankLoopCount++;
                EV2OutPut |= EV2DRINKTANK;
                PUMP3OutPut |= PUMP3DRINKTANK;
            }
            else
            {
                j25DrinkTankLoopCount = 0;
            }
            break;
        case DRINKTANKINJECT:

            if (waterLevel3 & FLOATBALLH)
            {
                l_sys.j25DrinkTankState = DRINKTANKIDEL;
            }
            else
            {
                if (waterLevel2 & FLOATBALLH)
                {
                    req_bitmap_op(DO_IN1OUT2EV1_BPOS, 0);
                    EV3OutPut &= ~EV3DRINKTANK;
                    EV1OutPut |= EV1DRINKTANK;
                    PUMP3OutPut |= PUMP3DRINKTANK;
                }
                if (!(waterLevel2 & FLOATBALLL))
                {
                    req_bitmap_op(DO_IN1OUT2EV1_BPOS, 1);
                    EV3OutPut |= EV3DRINKTANK;
                    EV1OutPut &= ~EV1DRINKTANK;
                    PUMP3OutPut &= ~PUMP3DRINKTANK;
                }
            }

            break;
        case DRINKTANKEMPTY:
            if (waterLevel3 & FLOATBALLL)
            {
                l_sys.j25AuxiliaryBoardDO |= AUXILIARYDO_EV4;
                PUMP1OutPut |= PUMP1DRINKTANK;
                j25DrinkTankEmptyCounter = 0;
            }
            else
            {
                if (j25DrinkTankEmptyCounter < COUNT3S)
                {
                    l_sys.j25AuxiliaryBoardDO |= AUXILIARYDO_EV4;
                    PUMP1OutPut |= PUMP1DRINKTANK;
                    j25DrinkTankEmptyCounter++;
                }
                else
                {
                    l_sys.j25DrinkTankState = DRINKTANKIDEL;
                    PUMP1OutPut &= ~PUMP1DRINKTANK;
                    l_sys.j25AuxiliaryBoardDO &= ~AUXILIARYDO_EV4;
                }
            }
            break;
        default:
            l_sys.j25DrinkTankState = DRINKTANKIDEL;
            break;
    }
}

void j25TransformChamberStateSet(ChamberState_t state)
{
    l_sys.j25TransformChamberState = state;
}

void j25TransformChamber(void)
{
    static uint16_t transformChamberEmptyCount       = 0;
    static rt_uint32_t transformChamberPeriodicCount = 0;
    rt_uint32_t T5                                   = COUNT24H;

    /**********
     * Periodic Transform Chamber Empty
     */
    if (transformChamberPeriodicCount < T5)
    {
        transformChamberPeriodicCount++;
    }
    else
    {
        if (l_sys.j25TransformChamberState == TRANSCHAMBERIDEL)
        {
            transformChamberPeriodicCount  = 0;
            l_sys.j25TransformChamberState = TRANSCHAMBEREMPTY;
        }
    }

    switch (l_sys.j25TransformChamberState)
    {
        case TRANSCHAMBERIDEL:
            EV3OutPut &= ~EV3TRANSFORM;
            l_sys.j25AuxiliaryBoardDO &= ~(AUXILIARYDO_PUMP2 | AUXILIARYDO_IN1OUT2EV3);
            break;
        case TRANSCHAMBERLOOP:
            EV3OutPut &= ~EV3TRANSFORM;
            l_sys.j25AuxiliaryBoardDO &= ~AUXILIARYDO_IN1OUT2EV3;
            l_sys.j25AuxiliaryBoardDO |= AUXILIARYDO_PUMP2;
            /**
             * 启动水泵2
             * 电磁阀3关闭
             * 一二阀3失电
             */
            break;
        case TRANSCHAMBERINJECT:
            l_sys.j25AuxiliaryBoardDO &= ~AUXILIARYDO_PUMP2;
            EV3OutPut |= EV3TRANSFORM;
            l_sys.j25AuxiliaryBoardDO |= AUXILIARYDO_IN1OUT2EV3;
            /**
             * 电磁阀3打开
             * 一二阀3得电
             * 关闭水泵2
             */
            break;
        case TRANSCHAMBEREMPTY:
            if (j25GetFloatBall1() & FLOATBALLL)
            {
                EV3OutPut &= ~EV3TRANSFORM;
                l_sys.j25AuxiliaryBoardDO |= (AUXILIARYDO_PUMP2 | AUXILIARYDO_IN1OUT2EV3);
                /**
                 * 一二阀3得电
                 * 水泵2启动
                 * 电磁阀3关闭
                 */
                transformChamberEmptyCount = 0;
            }
            else
            {
                if (transformChamberEmptyCount < COUNT3S)
                {
                    transformChamberEmptyCount++;
                }
                else
                {
                    EV3OutPut &= ~EV3TRANSFORM;
                    l_sys.j25AuxiliaryBoardDO &= ~(AUXILIARYDO_PUMP2 | AUXILIARYDO_IN1OUT2EV3);
                    /**
                     * 一二阀3失电
                     * 水泵2停机
                     */
                    l_sys.j25TransformChamberState = TRANSCHAMBERIDEL;
                }
            }
            break;
        default:
            l_sys.j25TransformChamberState = TRANSCHAMBERIDEL;
            break;
    }
}

void j25CompressorWork(FunctionalState state)
{
    if (state)
    {
        l_sys.Fan_Close &= ~FC_WL;
        l_sys.Comp_Close[0] &= ~FC_WL;
        l_sys.Comp_Close[1] &= ~FC_WL;
    }
    else
    {
        l_sys.Fan_Close |= FC_WL;
        l_sys.Comp_Close[0] |= FC_WL;
        l_sys.Comp_Close[1] |= FC_WL;
    }
}

void j25WaterMakeLogic(void)
{
    static uint16_t waterMakeCount = 0;
    uint16_t T3                    = COUNT30M;

    rt_uint8_t ball1 = j25GetFloatBall1();
    rt_uint8_t ball2 = j25GetFloatBall2();
    rt_uint8_t ball3 = j25GetFloatBall3();

    if ((j25GetFloatBall3() & FLOATBALLM) == 0)  //浮球3中不满
    {
        l_sys.j25WaterMakeState = 1;
    }

    if (l_sys.j25WaterMakeState == 1)
    {
        if (j25GetFloatBall3() & FLOATBALLH)  //浮球3上满
        {
            l_sys.j25WaterMakeState = 0;
            j25WaterCollectStateSet(COLLECTIDEL);
            j25TransformChamberStateSet(TRANSCHAMBERIDEL);
            j25CompressorWork(ENABLE);
        }
        else
        {
            if (j25GetFloatBall1() & FLOATBALLL)
            {
                j25TransformChamberStateSet(TRANSCHAMBERLOOP);
            }
            else
            {
                j25TransformChamberStateSet(TRANSCHAMBERINJECT);
            }
            j25CompressorWork(ENABLE);
            j25WaterCollectStateSet(COLLECTFULL);
        }
        waterMakeCount = 0;
    }
    else
    {
        if (waterMakeCount < T3)
        {
            waterMakeCount++;
        }
        if (waterMakeCount == T3)
        {
            j25CompressorWork(DISABLE);
            j25TransformChamberStateSet(TRANSCHAMBEREMPTY);
            j25WaterCollectStateSet(COLLECTHALF);
            waterMakeCount++;
        }
    }
}

void j25AutomaticClean(void)
{
    static rt_uint32_t _7DayCount = 0, _30DayDCount = 0;
    if (l_sys.j25AutomaticCleanState == 0)
    {
        rt_uint8_t waterLevel3 = j25GetFloatBall3();
        if (waterLevel3 & FLOATBALLH)
        {
            if (_7DayCount < COUNT7D)
            {
                _7DayCount++;
            }
            else
            {
                _7DayCount                   = 0;
                l_sys.j25AutomaticCleanState = 1;
            }
        }
        else
        {
            _7DayCount = 0;
        }
        if (_30DayDCount < COUNT30D)
        {
            _30DayDCount++;
        }
        else
        {
            _30DayDCount                 = 0;
            l_sys.j25AutomaticCleanState = 1;
        }
    }
    else
    {
        _7DayCount++;
    }

    if (l_sys.j25AutomaticCleanState == 1)
    {
        j25TransformChamberStateSet(TRANSCHAMBEREMPTY);
        j25WaterCollectStateSet(COLLECTHALF);
        j25DrinkTankStateSet(DRINKTANKEMPTY);
        l_sys.j25AutomaticCleanState = 2;
    }
    if (l_sys.j25AutomaticCleanState == 2)
    {
        rt_uint8_t waterlevel1 = j25GetFloatBall1();
        if (waterlevel1 & FLOATBALLH)
        {
            l_sys.j25AutomaticCleanState = 3;
        }
        else
        {
            j25TransformChamberStateSet(TRANSCHAMBERINJECT);
        }
    }
    if (l_sys.j25AutomaticCleanState == 3)
    {
        j25TransformChamberStateSet(TRANSCHAMBERIDEL);
        j25DrinkTankStateSet(DRINKTANKINJECT);
        l_sys.j25AutomaticCleanState = 5;
        _7DayCount                   = 0;
    }
    if ((l_sys.j25AutomaticCleanState == 5) && (_7DayCount > COUNT9M))
    {
        l_sys.j25AutomaticCleanState = 6;
        _7DayCount                   = 0;
        j25CompressorWork(ENABLE);
    }
    if ((l_sys.j25AutomaticCleanState == 6) && (_7DayCount > COUNT30M))
    {
        l_sys.j25AutomaticCleanState = 7;
        j25CompressorWork(DISABLE);
        j25TransformChamberStateSet(TRANSCHAMBEREMPTY);
        j25WaterCollectStateSet(COLLECTHALF);
        j25DrinkTankStateSet(DRINKTANKEMPTY);
    }
}

void j25LifeWaterOut(void)
{
    if ((j25GetFloatBall3() & FLOATBALLL) && (l_sys.j25AuxiliaryBoardDI & AUXILIARYDI_EVFAUCET))
    {
        PUMP1OutPut |= PUMP1LIFEWATEROUT;
        l_sys.j25AuxiliaryBoardDO |= AUXILIARYDO_EV5;
    }
    else
    {
        PUMP1OutPut &= ~PUMP1LIFEWATEROUT;
        l_sys.j25AuxiliaryBoardDO &= ~AUXILIARYDO_EV5;
    }
}
void j25PureWaterOut(void)
{
    static uint16_t pureWaterOutCount = 0;
    static uint16_t T8                = 20;
    if ((getKeyRestain) && ((l_sys.j25WaterTempreture == NORMALTEM) || (l_sys.j25WaterTempreture == IDELTEM)))
    {
        l_sys.j25PureWaterOutState = 1;
    }
    else
    {
        l_sys.j25PureWaterOutState = 0;
    }

    if (l_sys.j25PureWaterOutState)
    {
        EV2OutPut |= EV2PUREWATEROUT;
        PUMP3OutPut |= PUMP3PUREOUT;
        if (pureWaterOutCount > T8)
        {
            req_bitmap_op(DO_IN1OUT2EV2_BPOS, 1);
        }
        else
        {
            pureWaterOutCount++;
        }
    }
    else
    {
        pureWaterOutCount = 0;
        EV2OutPut &= ~EV2PUREWATEROUT;
        PUMP3OutPut &= ~PUMP3PUREOUT;
        req_bitmap_op(DO_IN1OUT2EV2_BPOS, 0);
    }
}

void hotWaterOut(void)
{
    static uint8_t hotWaterOutState = 0;
    static uint8_t u8HeatNum        = 0;
    static uint8_t u8CloseNum       = 0;
    uint8_t u8Temp                  = 0;
    req_exe_log("WaterTempreture:%d,childLockState:%d", l_sys.j25WaterTempreture, l_sys.j25ChildLockState);
    if ((getKeyRestain) &&
        ((l_sys.j25WaterTempreture == BOILINGTEM) || (l_sys.j25WaterTempreture == TEATEM) ||
         (l_sys.j25WaterTempreture == MILKTEM)) &&
        (l_sys.j25ChildLockState))
    {
        hotWaterOutState = 1;
        l_sys.j25ChildLockState++;
    }
    else
    {
        if (hotWaterOutState == 1)
            hotWaterOutState = 0;
    }

    if (hotWaterOutState == 1)
    {
        u8CloseNum = 0;
        //串口通信
        if ((l_sys.OutWater_OK == HEATER_SEND) && (u8HeatNum >= 3))
        {
            l_sys.OutWater_OK = WATER_READ;
            if (Heat_Send(HEAT_READPARA, 0, 0, 0))
            {
                g_ComStat[UART_HEAT] = SEND_Over;  //发送完成
            }
        }
        else
        {
            if (u8HeatNum < WRITEHEAT_MAX)  //
            {
                u8HeatNum++;
                l_sys.OutWater_OK = HEATER_SEND;
                switch (l_sys.j25WaterTempreture)
                {
                    case IDELTEM:
                        u8Temp = 25;  //常温
                        break;
                    case BOILINGTEM:
                        u8Temp = 99;  // 开水 break;
                    case NORMALTEM:
                        u8Temp = 25;  //常温
                        break;
                    case TEATEM:
                        u8Temp = 65;  // 65℃
                        break;
                    case MILKTEM:
                        u8Temp = 45;  // 45℃
                        break;
                    default:
                        u8Temp = 25;  //常温
                        break;
                }
                if (Heat_Send(HEAT_WRITEPARA, OPEN_HEAT, u8Temp, 5000))
                {
                    l_sys.OutWater_Flag = WATER_HEAT;  //出水中
                }
            }
        }
    }
    else if (hotWaterOutState == 0)
    {
        u8HeatNum = 0;
        if (u8CloseNum < CLOSEHEAT_MAX)  //关闭出水
        {
            u8CloseNum++;
            u8Temp = 0;
            if (Heat_Send(HEAT_WRITEPARA, CLOSE_HEAT, u8Temp, 5000))
            {
            }
        }
    }
    if (l_sys.j25ChildLockState)
        l_sys.j25ChildLockState--;
}

void j25UVCtrl(void)
{
    static rt_uint8_t UVState = 0;
    if (UVState == 0)
    {
        UVState = 1;
        req_bitmap_op(DO_PUREUV_BPOS, 1);
        l_sys.j25AuxiliaryBoardDO |= AUXILIARYDO_SRCUV;
    }
}

//总体需求执行逻辑
void req_execution(int16_t target_req_temp, int16_t target_req_hum)
{
    //风机控制
    fan_req_exe(target_req_temp, target_req_hum);
    //压缩机控制
    compressor_req_exe(target_req_temp, target_req_hum);
    //显示屏控制
    Restart_DIS_exe();

    j25WaterMakeLogic();

    // j25AutomaticClean();

    j25WaterCollect();

    j25TransformChamber();

    j25LifeWaterOut();

    j25PureWaterOut();

    hotWaterOut();

    j25UVCtrl();

    j25OutPutDO();
}
