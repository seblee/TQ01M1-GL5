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

#include "ble_key.h"

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

static rt_uint8_t EV1OutPut   = 0;
static rt_uint8_t EV2OutPut   = 0;
static rt_uint8_t EV3OutPut   = 0;
static rt_uint8_t EV4OutPut   = 0;
static rt_uint8_t EV5OutPut   = 0;
static rt_uint8_t UVOutPut    = 0;
static rt_uint8_t PUMP1OutPut = 0;
static rt_uint8_t PUMP2OutPut = 0;
static rt_uint8_t PUMP3OutPut = 0;

#define PUMP_TO_DRINK_FLAG 0X01
#define WATEROUT_FLAG 0X02
#define WATERLOOP_FLAG 0X04
#define EMPTY_DRINKTANK_FLAG 0X08

void j25OutPutDO(void)
{
    // EV1OutPut
    if (EV1OutPut)
    {
        l_sys.j25AuxiliaryBoardDO0 |= AUXILIARYDO0_EV1;
    }
    else
    {
        l_sys.j25AuxiliaryBoardDO0 &= ~AUXILIARYDO0_EV1;
    }
    // EV2OutPut
    if (EV2OutPut)
    {
        l_sys.j25AuxiliaryBoardDO0 |= AUXILIARYDO0_EV2;
    }
    else
    {
        l_sys.j25AuxiliaryBoardDO0 &= ~AUXILIARYDO0_EV2;
    }
    // EV3OutPut
    if (EV3OutPut)
    {
        l_sys.j25AuxiliaryBoardDO0 |= AUXILIARYDO0_EV3;
    }
    else
    {
        l_sys.j25AuxiliaryBoardDO0 &= ~AUXILIARYDO0_EV3;
    }
    // EV4OutPut
    if (EV4OutPut)
    {
        l_sys.j25AuxiliaryBoardDO0 |= AUXILIARYDO0_EV4;
    }
    else
    {
        l_sys.j25AuxiliaryBoardDO0 &= ~AUXILIARYDO0_EV4;
    }
    // EV5OutPut
    if (EV5OutPut)
    {
        l_sys.j25AuxiliaryBoardDO0 |= AUXILIARYDO0_EV5;
    }
    else
    {
        l_sys.j25AuxiliaryBoardDO0 &= ~AUXILIARYDO0_EV5;
    }
    // UVOutPut
    if (UVOutPut)
    {
        l_sys.j25AuxiliaryBoardDO0 |= AUXILIARYDO0_UV;
    }
    else
    {
        l_sys.j25AuxiliaryBoardDO0 &= ~AUXILIARYDO0_UV;
    }
    // PUMP1OutPut
    if (PUMP1OutPut)
    {
        l_sys.j25AuxiliaryBoardDO0 |= AUXILIARYDO0_PUMP1;
    }
    else
    {
        l_sys.j25AuxiliaryBoardDO0 &= ~AUXILIARYDO0_PUMP1;
    }
    // PUMP2OutPut
    if (PUMP2OutPut)
    {
        l_sys.j25AuxiliaryBoardDO0 |= AUXILIARYDO0_PUMP2;
    }
    else
    {
        l_sys.j25AuxiliaryBoardDO0 &= ~AUXILIARYDO0_PUMP2;
    }
    // PUMP3OutPut
    if (PUMP3OutPut)
    {
        l_sys.j25AuxiliaryBoardDO1 |= AUXILIARYDO1_PUMP3;
    }
    else
    {
        l_sys.j25AuxiliaryBoardDO1 &= ~AUXILIARYDO1_PUMP3;
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
            if (g_sys.status.ComSta.u16TH[0].Temp >= 300)
            {
                req_bitmap_op(DO_F24_BPOS, 1);
            }

            if (g_sys.status.ComSta.u16TH[0].Temp <= 260)
            {
                req_bitmap_op(DO_F24_BPOS, 0);
            }

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
/**
 * @brief   饮水箱低水位判断
 * @return TRUE:达到低水位 FALSE:未达到低水位
 */
uint8_t WaterOut_level(void)
{
    rt_uint8_t ballLevel = 0;
    ballLevel            = j25GetFloatBall3();
    if (ballLevel & FLOATBALLL)
    {
        return TRUE;
    }
    else
    {
        return FALSE;
    }
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

    if (l_sys.j25AuxiliaryBoardDI & (1 << 4))  // ball high
    {
        Water_level |= FLOATBALLH;
    }
    if (l_sys.j25AuxiliaryBoardDI & (1 << 6))  // ball high
    {
        Water_level |= FLOATBALLM;
    }
    if (l_sys.j25AuxiliaryBoardDI & (1 << 3))  // ball high
    {
        Water_level |= FLOATBALLL;
    }
    return Water_level;
}

/**
 * @brief   抽水到转化腔
 */
void j25InjectTransformChamber(void)
{
    rt_uint8_t ballLevel = 0;

    ballLevel = j25GetFloatBall3();
    if (ballLevel & FLOATBALLH)
    {
        /**
         * @brief   关闭水泵3
         */
        PUMP3OutPut = 0;
    }
    else
    {
        /**
         * @brief   启动水泵3
         */
        PUMP3OutPut = 1;
    }
}
/**
 * @brief   浓缩水槽排水
 */
void j25ConcentrateSinkEmpty(void)
{
    rt_uint8_t ballLevel = 0;

    ballLevel = j25GetFloatBall1();
    if (ballLevel & FLOATBALLH)
    {
        /**
         * @brief   启动水泵2
         */
        PUMP2OutPut = 1;
    }
    if ((ballLevel & FLOATBALLL) == 0)
    {
        /**
         * @brief   关闭水泵2
         */
        PUMP2OutPut = 0;
    }
}
/**
 * @brief   抽水到饮水箱
 */
void j25PumpingWaterToDrinkTank(void)
{
    rt_uint8_t ballLevel = 0;

    ballLevel = j25GetFloatBall2();
    if ((ballLevel & FLOATBALLH) && ((l_sys.OutWater_Flag & WATER_NORMAL_ICE) == 0) &&
        ((l_sys.j25AutomaticCleanState == 0) || (l_sys.j25AutomaticCleanState == 7)))
    {
        /**
         * @brief   启动
         * @brief   水泵1
         * @brief   电磁阀2
         * @brief   电磁阀3
         * @brief   紫外灯
         */
        PUMP1OutPut |= PUMP_TO_DRINK_FLAG;
        EV2OutPut |= PUMP_TO_DRINK_FLAG;
        EV3OutPut |= PUMP_TO_DRINK_FLAG;
        UVOutPut |= PUMP_TO_DRINK_FLAG;
        l_sys.j25PumpingWaterToDrinkTankState = 1;
    }
    if (((ballLevel & FLOATBALLL) == 0) || (l_sys.OutWater_Flag & WATER_NORMAL_ICE) ||
        ((l_sys.j25AutomaticCleanState != 0) && (l_sys.j25AutomaticCleanState != 7)))
    {
        /**
         * @brief   关闭
         * @brief   水泵1
         * @brief   电磁阀2
         * @brief   电磁阀3
         * @brief   紫外灯
         */
        PUMP1OutPut &= ~PUMP_TO_DRINK_FLAG;
        EV2OutPut &= ~PUMP_TO_DRINK_FLAG;
        EV3OutPut &= ~PUMP_TO_DRINK_FLAG;
        UVOutPut &= ~PUMP_TO_DRINK_FLAG;
        l_sys.j25PumpingWaterToDrinkTankState = 0;
    }
}
/**
 * @brief   饮用水出水
 */
void j25WaterOut(void)
{
    rt_uint8_t ballLevel = 0;

    ballLevel = j25GetFloatBall3();
    if ((ballLevel & FLOATBALLL) && (l_sys.LedKey.OutWater) &&
        ((l_sys.j25AutomaticCleanState == 0) || (l_sys.j25AutomaticCleanState == 7)))
    {
        /**
         * @brief   启动电磁阀1
         * @brief   启动水泵1
         * @brief   紫外灯
         * @brief   启动电磁阀5
         */
        EV1OutPut |= WATEROUT_FLAG;
        PUMP1OutPut |= WATEROUT_FLAG;
        UVOutPut |= WATEROUT_FLAG;
        EV5OutPut |= WATEROUT_FLAG;
        l_sys.OutWater_Flag |= WATER_NORMAL_ICE;  //出水中
    }
    else
    {
        if ((!(ballLevel & FLOATBALLL)) || ((l_sys.j25AutomaticCleanState != 0) && (l_sys.j25AutomaticCleanState != 7)))
        {
            l_sys.LedKey.OutWater = 0;
        }
        EV1OutPut &= ~WATEROUT_FLAG;
        PUMP1OutPut &= ~WATEROUT_FLAG;
        UVOutPut &= ~WATEROUT_FLAG;
        EV5OutPut &= ~WATEROUT_FLAG;
        l_sys.OutWater_Flag &= ~WATER_NORMAL_ICE;  //出水中
    }
}
/**
 * @brief   饮水箱循环
 */
void j25DrinkWaterTankLoop(void)
{
    static rt_uint32_t j25DrinkTankLoopCount = 0;
    rt_uint8_t ballLevel                     = 0;

    ballLevel = j25GetFloatBall3();
    j25DrinkTankLoopCount++;
    if (j25DrinkTankLoopCount < (3 * 60 * 60 * 2))  // interval 3H
    {
        l_sys.j25LoopState = 1;
    }
    else if (j25DrinkTankLoopCount < ((3 * 60 * 60 * 2) + (20 * 60 * 2)))  // loop 20 minute
    {
        if ((ballLevel & FLOATBALLL) && (l_sys.j25PumpingWaterToDrinkTankState == 0) &&
            ((l_sys.OutWater_Flag & WATER_NORMAL_ICE) == 0) &&
            ((l_sys.j25AutomaticCleanState == 0) || (l_sys.j25AutomaticCleanState == 7)))
        {
            l_sys.j25LoopState = 2;
            /**
             * @brief   启动电磁阀1
             * @brief   启动水泵1
             * @brief   紫外灯
             * @brief   启动电磁阀3
             */
            EV1OutPut |= WATERLOOP_FLAG;
            PUMP1OutPut |= WATERLOOP_FLAG;
            UVOutPut |= WATERLOOP_FLAG;
            EV3OutPut |= WATERLOOP_FLAG;
        }
        else
        {
            l_sys.j25LoopState = 3;
            EV1OutPut &= ~WATERLOOP_FLAG;
            PUMP1OutPut &= ~WATERLOOP_FLAG;
            UVOutPut &= ~WATERLOOP_FLAG;
            EV3OutPut &= ~WATERLOOP_FLAG;
        }
    }
    else  // clear 0
    {
        l_sys.j25LoopState    = 0;
        j25DrinkTankLoopCount = 0;
        EV1OutPut &= ~WATERLOOP_FLAG;
        PUMP1OutPut &= ~WATERLOOP_FLAG;
        UVOutPut &= ~WATERLOOP_FLAG;
        EV3OutPut &= ~WATERLOOP_FLAG;
    }
}
/**
 * @brief   饮水箱排水
 */
void j25DrinkWaterTankEmpty(void)
{
    rt_uint8_t ballLevel = j25GetFloatBall3();
    /**
     * @brief   启动电磁阀1
     * @brief   启动水泵1
     * @brief   启动电磁阀4
     */
    if (l_sys.j25AutomaticCleanState == 1)
    {
        if ((ballLevel & FLOATBALLL))
        {
            l_sys.j25AutomaticCleanState = 2;
            EV1OutPut |= EMPTY_DRINKTANK_FLAG;
            PUMP1OutPut |= EMPTY_DRINKTANK_FLAG;
            EV4OutPut |= EMPTY_DRINKTANK_FLAG;
        }
        else
        {
            l_sys.j25AutomaticCleanState = 7;
        }
    }
    else if (l_sys.j25AutomaticCleanState == 2)
    {
        if ((ballLevel & FLOATBALLL))
        {
            EV1OutPut |= EMPTY_DRINKTANK_FLAG;
            PUMP1OutPut |= EMPTY_DRINKTANK_FLAG;
            EV4OutPut |= EMPTY_DRINKTANK_FLAG;
        }
        else
        {
            l_sys.j25AutomaticCleanState = 7;
            EV1OutPut &= ~EMPTY_DRINKTANK_FLAG;
            PUMP1OutPut &= ~EMPTY_DRINKTANK_FLAG;
            EV4OutPut &= ~EMPTY_DRINKTANK_FLAG;
        }
    }
}
/**
 * @brief   compressor & fan state set
 * @param  state            My Param doc
 */
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
    // rt_uint8_t ballLevel        = j25GetFloatBall3();
    static rt_uint16_t humCount = 0;

    static _TKS_FLAGA_type MakeLogicFlag = {0};
#define HUM_START_FLAG MakeLogicFlag.bits.b0
#define HUM_STOP_FLAG MakeLogicFlag.bits.b1
#define ALARM_STOP_FLAG MakeLogicFlag.bits.b2
    // #define WATERLEVEL_STOP_FLAG MakeLogicFlag.bits.b3

    if (g_sys.status.ComSta.u16TH[0].Hum > g_sys.config.ComPara.u16Start_Humidity)
    {
        if (humCount < 10)
        {
            humCount++;
        }
        else
        {
            HUM_START_FLAG = 1;
        }
    }
    else if (g_sys.status.ComSta.u16TH[0].Hum > g_sys.config.ComPara.u16Stop_Humidity)
    {
        humCount       = 0;
        HUM_START_FLAG = 0;
        HUM_STOP_FLAG  = 0;
    }
    else
    {
        if (humCount < 10)
        {
            humCount++;
        }
        else
        {
            HUM_STOP_FLAG = 1;
        }
    }

    if ((get_alarm_bitmap(ACL_SYS01_EXHAUST_HI)) || (get_alarm_bitmap(ACL_J25_HI_TEM)))
    {
        ALARM_STOP_FLAG = 1;
    }
    else
    {
        ALARM_STOP_FLAG = 0;
    }
    // if (ballLevel & FLOATBALLM)
    // {
    //     WATERLEVEL_STOP_FLAG = 1;
    // }
    // else
    // {
    //     WATERLEVEL_STOP_FLAG = 0;
    // }

    if (HUM_START_FLAG)
    {
        l_sys.j25WaterMakeState = 1;
    }

    if (ALARM_STOP_FLAG || HUM_STOP_FLAG)  //|| WATERLEVEL_STOP_FLAG)
    {
        l_sys.j25WaterMakeState = 0;
    }

    if (l_sys.j25WaterMakeState == 1)
    {
        j25CompressorWork(ENABLE);
    }
    else
    {
        j25CompressorWork(DISABLE);
    }
}

void j25HotWaterOut(void)
{
    static uint8_t hotWaterOutState = 0;
    static uint8_t u8HeatNum        = 0;
    static uint8_t u8CloseNum       = 0;
    uint8_t u8Temp                  = 0;
    if ((fetchKeyRestain) &&
        ((l_sys.j25WaterTempreture == BOILINGTEM) || (l_sys.j25WaterTempreture == TEATEM) ||
         (l_sys.j25WaterTempreture == MILKTEM)) &&
        (l_sys.j25ChildLockState))
    {
        hotWaterOutState        = 1;
        l_sys.j25ChildLockState = g_sys.config.ComPara.j25ChildLockTime * 2;  // T3
    }
    if (!fetchKeyRestain)
        hotWaterOutState = 0;

    if (hotWaterOutState == 1)
    {
        l_sys.j25AutomaticCleanCount7D = 0;
        u8CloseNum                     = 0;
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
                        u8Temp = g_sys.config.ComPara.j25BoilingTempreture;  // boiling
                        break;
                    case NORMALTEM:
                        u8Temp = 25;  //常温
                        break;
                    case TEATEM:
                        u8Temp = g_sys.config.ComPara.j25TeaTempreture;  // 65℃
                        break;
                    case MILKTEM:
                        u8Temp = g_sys.config.ComPara.j25MilkTempreture;  // 45℃
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
        l_sys.j25AuxiliaryBoardDO0 |= AUXILIARYDO0_UV;
    }
}

void reqCounter500ms(void)
{
    static rt_uint8_t childLockCounter = 0;
    if (l_sys.LedKey.ChildLock)
    {
        if (childLockCounter == 0)
        {
            childLockCounter = g_sys.config.ComPara.j25ChildLockTime * 2;
        }
        if (childLockCounter > 0)
        {
            childLockCounter--;
            if (childLockCounter == 0)
            {
                l_sys.LedKey.ChildLock = FALSE;
            }
        }
    }
    else
    {
        childLockCounter = 0;
    }
}

//总体需求执行逻辑
void req_execution(int16_t target_req_temp, int16_t target_req_hum)
{
    rt_uint8_t ball[3] = {0};
    //风机控制
    fan_req_exe(target_req_temp, target_req_hum);
    //压缩机控制
    compressor_req_exe(target_req_temp, target_req_hum);
    //显示屏控制
    // Restart_DIS_exe();

    j25InjectTransformChamber();
    j25ConcentrateSinkEmpty();

    j25DrinkWaterTankLoop();
    j25PumpingWaterToDrinkTank();
    j25WaterOut();
    j25DrinkWaterTankEmpty();

    j25WaterMakeLogic();

    j25OutPutDO();
    ball[0] = j25GetFloatBall1();
    ball[1] = j25GetFloatBall2();
    ball[2] = j25GetFloatBall3();

    rt_kprintf("ball:%02X %02X %02X empty:%d,waterOut:%02x,pumpState:%02x,loop:%d,BITMAP:%04X,AUXDI:%04X\n", ball[0],
               ball[1], ball[2], l_sys.j25AutomaticCleanState, l_sys.OutWater_Flag,
               l_sys.j25PumpingWaterToDrinkTankState, l_sys.j25LoopState, g_sys.status.ComSta.u16Din_bitmap[0],
               l_sys.j25AuxiliaryBoardDI);
}
