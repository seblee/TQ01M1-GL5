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
#define CONFIG_DEBUG
#ifdef CONFIG_DEBUG
#ifndef req_exe_log
#define req_exe_log(N, ...) rt_kprintf("##[req_exe %s:%4d] " N "\r\n", __FUNCTION__, __LINE__, ##__VA_ARGS__)
#endif /* req_exe_log(...) */
#else
#define req_exe_log(...)
#endif /* ! CONFIG_DEBUG */
enum
{
    RUNING_STATUS_COOLING_BPOS = 0,
    RUNING_STATUS_HEATING_BPOS,
    RUNING_STATUS_HUMIDIFYING_BPOS,
    RUNING_STATUS_DEHUMING_BPOS,
};

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

extern local_reg_st l_sys;

typedef struct
{
    uint16_t time_out;
    uint16_t flush_delay_timer;
    uint16_t hum_fill_cnt;
    uint32_t hum_timer;
    uint32_t check_timer;
    uint8_t check_fill_flag;
    uint8_t check_drain_flag;
    uint8_t check_flag;
    uint16_t warm_time;
} hum_timer;

// static hum_timer hum_delay_timer;

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

    //			if(action == 0)
    //			{
    //					l_sys.bitmap[BITMAP_REQ] &= ~(0x0001<<component_bpos);
    //			}
    //			else
    //			{
    //					l_sys.bitmap[BITMAP_REQ] |= (0x0001<<component_bpos);
    //			}
}

//需求模拟输出操作函数
static void req_ao_op(uint8_t component_bpos, int16_t value)
{
    extern sys_reg_st g_sys;

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

// static void req_pwm_op(uint8_t component_bpos, int16_t value)
//{
//

//		l_sys.pwm_list[component_bpos][BITMAP_REQ] = value;
//}
//模拟输出跟踪函数，向设置目标，按照步长参数进行变化；
static int16_t analog_step_follower(int16_t target, uint16_t dev_type)
{
    extern sys_reg_st g_sys;
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
    extern sys_reg_st g_sys;

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
    extern sys_reg_st g_sys;

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
    extern sys_reg_st g_sys;

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
    extern sys_reg_st g_sys;

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
    extern sys_reg_st g_sys;

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
//外接水源
uint8_t Exit_Water(void)
{
    extern sys_reg_st g_sys;
    if (g_sys.config.ComPara.u16ExitWater_Mode == WATER_FLUSH)  //冲洗模式
    {
        return WATER_FLUSH;
    }
    else if (g_sys.config.ComPara.u16ExitWater_Mode == WATER_FILL)  //注水模式
    {
        return WATER_FILL;
    }
    else if (g_sys.config.ComPara.u16ExitWater_Mode == WATER_Disinfection)  //注水模式
    {
        return WATER_Disinfection;
    }
    else
    {
        return WATER_AIR;
    }
}

//外接水源
void External_Water_exe(uint8_t u8Type)
{
    //		uint16_t u16WL;
    extern sys_reg_st g_sys;

    if (Exit_Water() != WATER_AIR)  //外接水源
    {
        if (u8Type == TRUE)  //出水
        {
            req_bitmap_op(DO_FV_BPOS, 1);  //外接水源
        }
        else
        {
            req_bitmap_op(DO_FV_BPOS, 0);  //外接水源
        }
    }
    else
    {
        req_bitmap_op(DO_FV_BPOS, 0);
    }

    return;
}

//风机,压机启动条件
void Sys_Fan_CP_WL(void)
{
    extern sys_reg_st g_sys;

    uint16_t u16WL;
    uint16_t Test               = 0;
    static uint8_t u8Offset_CNT = 0;
    static uint8_t u8FCP_Start  = 0;
    static uint8_t u8WL_Start   = FALSE;

    if (sys_get_pwr_sts() == 1)
    {
        Test = 1;
        l_sys.PowerOn_Time++;
    }
    else
    {
        l_sys.PowerOn_Time = 0;
    }

    u16WL = Get_Water_level();
    //		rt_kprintf("u16WL=%x,Test=%x,,TH_Check_Interval=%d,din_bitmap[0]=%x\n",u16WL,Test,l_sys.TH_Check_Interval,g_sys.status.ComSta.u16Din_bitmap[0]);
    //		//TEST
    //		u16WL=0x13;
    //水位异常
    if ((u16WL & S_U) || (u16WL & D_U) || (u16WL & D_M))
    {
        u8WL_Start = FALSE;
        if (((l_sys.Cold_Water == TRUE) && (g_sys.config.ComPara.u16ColdWater_Mode == NORMAL_ICE)) &&
            ((u16WL & S_U) == 0) && ((u16WL & D_U) == 0))  //制冰水模式
        {
            Test |= 0x02;
            l_sys.Fan_Close &= ~FC_WL;
            l_sys.Comp_Close[0] &= ~FC_WL;
            l_sys.Comp_Close[1] &= ~FC_WL;
        }
        else
        {
            Test |= 0x04;
            l_sys.Fan_Close |= FC_WL;
            l_sys.Comp_Close[0] |= FC_WL;
            l_sys.Comp_Close[1] |= FC_WL;
        }
    }
    else
    {
        //#ifdef SYS_M_V50
        /*
        控制逻辑：静音模式使能，水位低于满水位开始低速制水；
                            静音模式禁止，夜间闲时(默认1:00-6:00),水位低于满水位开始高速制水;
                            其他时段（即白天），水位低于中水位时，开始低速制水;
                            如果闲时起始时间与结束时间一致，则水位低于满水位开始高速制水；
        */

        if (g_sys.config.ComPara.LN.u16LN_Enable)  //低噪音模式，继续制水
        {
            l_sys.Water_Full = FALSE;
        }
        else
        {
            if (g_sys.config.ComPara.LN.u16LN_Mode)  //其他时段（即白天），水位低于中水位时，开始低速制水
            {
                if (u16WL & D_ML)  //中水位上,
                {
                    if (u8WL_Start == TRUE)  //开始在制水，则继续
                    {
                        Test |= 0x08;
                        l_sys.Water_Full = FALSE;
                    }
                    else
                    {
                        if (l_sys.Cold_Water == TRUE)  //在制冰水,开启风机、压机
                        {
                            Test |= 0x08;
                            l_sys.Water_Full = FALSE;
                        }
                        else
                        {
                            Test |= 0x10;
                            l_sys.Water_Full = TRUE;
                        }
                    }
                }
                else  //中水位下,
                {
                    Test |= 0x40;
                    l_sys.Water_Full = FALSE;
                    u8WL_Start       = TRUE;  //中水位以下，之前一直在制水
                }
            }
            else  //夜间闲时,水位低于满水位开始高速制水;
            {
                Test |= 0x80;
                l_sys.Water_Full = FALSE;
                u8WL_Start       = TRUE;  //闲时，之前一直在制水
            }
        }

        //#else
        //      l_sys.Water_Full = FALSE;
        //#endif

        if (l_sys.Water_Full == TRUE)
        {
            Test |= 0x20;
            l_sys.Fan_Close |= FC_WL;
            l_sys.Comp_Close[0] |= FC_WL;
            l_sys.Comp_Close[1] |= FC_WL;
        }
        else
        {
            l_sys.Fan_Close &= ~FC_WL;
            l_sys.Comp_Close[0] &= ~FC_WL;
            l_sys.Comp_Close[1] &= ~FC_WL;
        }
    }

    //		//长时间未到中水位
    //		if(((u16WL&S_M)==0)&&(l_sys.PowerOn_Time>=POWERTIME))//90分钟小于中水位
    //		{
    //				Test|=0x08;
    //				l_sys.Fan_Close |= FC_PT;
    //				l_sys.Comp_Close |= FC_PT;
    //		}
    //		else
    //		{
    //				l_sys.Fan_Close &= ~FC_PT;
    //				l_sys.Comp_Close &= ~FC_PT;
    //		}

    u8Offset_CNT++;
    if (u8Offset_CNT >= CF_DELAY)
    {
        u8Offset_CNT = CF_DELAY;
    }
    if ((l_sys.TH_Check_Interval == 0) && (u8Offset_CNT >= CF_DELAY))  //上电延时判断温湿度
    {
        l_sys.TH_Check_Interval = g_sys.config.ComPara.u16TH_Interal * 60;  //温湿度判断间隔
        if (u8FCP_Start == 0)                                               //冷启动
        {
            if (((g_sys.status.ComSta.u16TH[0].Temp > g_sys.config.ComPara.u16Start_Temp[0]) &&
                 (g_sys.status.ComSta.u16TH[0].Temp < g_sys.config.ComPara.u16Start_Temp[1])) &&
                (g_sys.status.ComSta.u16TH[0].Hum > g_sys.config.ComPara.u16Start_Humidity))  //温湿度满足条件
            {
                Test |= 0x100;
                l_sys.Fan_Close &= ~FC_TH;
                l_sys.Comp_Close[0] &= ~FC_TH;
                l_sys.Comp_Close[1] &= ~FC_TH;
                u8FCP_Start = 1;  //启动
            }
            else
            {
                Test |= 0x200;
                l_sys.Fan_Close |= FC_TH;
                l_sys.Comp_Close[0] |= FC_TH;
                l_sys.Comp_Close[1] |= FC_TH;
            }
        }
        else
        {
            if (((g_sys.status.ComSta.u16TH[0].Temp < g_sys.config.ComPara.u16Stop_Temp[0]) ||
                 (g_sys.status.ComSta.u16TH[0].Temp >= g_sys.config.ComPara.u16Stop_Temp[1])) ||
                (g_sys.status.ComSta.u16TH[0].Hum < g_sys.config.ComPara.u16Stop_Humidity))  //温湿度不满足条件
            {
                Test |= 0x400;
                l_sys.Fan_Close |= FC_TH;
                l_sys.Comp_Close[0] |= FC_TH;
                l_sys.Comp_Close[1] |= FC_TH;
                u8FCP_Start = 0;
            }
        }
    }

    if (g_sys.config.ComPara.u16ExitWater_Mode != WATER_AIR)  //外接水源
    {
        Test |= 0x800;
        l_sys.Fan_Close |= FC_WS;
        l_sys.Comp_Close[0] |= FC_WS;
        l_sys.Comp_Close[1] |= FC_WS;
    }
    else
    {
        Test |= 0x1000;
        l_sys.Fan_Close &= ~FC_WS;
        l_sys.Comp_Close[0] &= ~FC_WS;
        l_sys.Comp_Close[1] &= ~FC_WS;
    }
    if ((sys_get_remap_status(WORK_MODE_STS_REG_NO, DEFROST1_STS_BPOS) != 0) ||
        (sys_get_remap_status(WORK_MODE_STS_REG_NO, DEFROST2_STS_BPOS) != 0))  //
    {
        l_sys.TH_Check_Interval = 0;  //除霜后，需要检测温湿度
        u8FCP_Start             = 0;
        Test |= 0x2000;
    }
    if (alarm_Off() == TRUE)  //关机告警
    {
        Test |= 0x4000;
        l_sys.Fan_Close |= FC_AL;
        l_sys.Comp_Close[0] |= FC_AL;
        l_sys.Comp_Close[1] |= FC_AL;
    }
    else
    {
        Test |= 0x8000;
        l_sys.Fan_Close &= ~FC_AL;
        l_sys.Comp_Close[0] &= ~FC_AL;
        l_sys.Comp_Close[1] &= ~FC_AL;
    }

    g_sys.status.ComSta.REQ_TEST[2] = Test;

    //		rt_kprintf("u16WL=%x,Test=%x,u16LN_Mode=%d,u8WL_Start=%x,Fan_Close=%x,Comp_Close[0]=%x\n",u16WL,Test,g_sys.config.ComPara.LN.u16LN_Mode,u8WL_Start,l_sys.Fan_Close,l_sys.Comp_Close[0]);

    return;
}

//流量计算
uint16_t PluseCalc_Water(uint16_t PluseCnt)
{
    extern sys_reg_st g_sys;

    uint16_t u16Water_Flow;
    float fWflow;

    //		if(PluseCnt<L200)//小于0.15L
    //		{
    //			fWflow=	(float)PluseCnt*L200_FACTOR;
    //		}
    //		else
    if (PluseCnt < L300)  //小于0.3L
    {
        fWflow = (float)PluseCnt * L300_FACTOR;
    }
    else if (PluseCnt < L500)
    {
        fWflow = (float)PluseCnt * L500_FACTOR;
    }
    else if (PluseCnt < L1000)
    {
        fWflow = (float)PluseCnt * L1000_FACTOR;
    }
    else if (PluseCnt < L1500)
    {
        fWflow = (float)PluseCnt * L1500_FACTOR;
    }
    else if (PluseCnt < L2000)
    {
        fWflow = (float)PluseCnt * L2000_FACTOR;
    }
    else
    {
        fWflow = (float)PluseCnt * L2000_FACTOR;
    }

    u16Water_Flow = (uint16_t)fWflow;

    return u16Water_Flow;
}
//按键出水检测
void WaterOut_Key(void)
{
    extern sys_reg_st g_sys;

    //冷水 1
    if ((sys_get_di_sts(DI_Cold_1_BPOS) == 1))
    {
        l_sys.OutWater_Key |= WATER_NORMAL_ICE;
        l_sys.OutWater_Delay[0] = WATER_MAXTIME;
    }
    else
    {
        l_sys.OutWater_Key &= ~WATER_NORMAL_ICE;
        l_sys.OutWater_Delay[0] = 0;
    }

    //冷水 2
    if ((sys_get_di_sts(DI_Cold_2_BPOS) == 1) && (g_sys.config.ComPara.u16Water_Ctrl & TWO_COLD))  //双路出冷水
    {
        l_sys.OutWater_Key |= WATER_ICE;
        l_sys.OutWater_Delay[2] = WATER_MAXTIME;
    }
    else
    {
        l_sys.OutWater_Key &= ~WATER_ICE;
        l_sys.OutWater_Delay[2] = 0;
    }

    //		//童锁
    //		if((sys_get_di_sts(DI_K3_BPOS)==1))
    //		{
    //				l_sys.ChildLock_Cnt[0]++;
    //		}
    //		else
    //		{
    //				l_sys.ChildLock_Cnt[0]=0;
    //		}

    if (l_sys.ChildLock_Cnt[0] >= ChildKey_Cnt)
    {
        l_sys.ChildLock_Cnt[0] = 0;
        l_sys.ChildLock_Key    = 1;
        l_sys.ChildLock_Cnt[1] = ChildKey_Lose;
    }
    //童锁使能
    if (l_sys.ChildLock_Key)
    {
        //热水
        if ((sys_get_di_sts(DI_Heat_BPOS) == 1))
        {
            l_sys.OutWater_Key |= WATER_HEAT;
            l_sys.OutWater_Delay[1] = WATER_MAXTIME;
        }
        else
        {
            l_sys.OutWater_Key &= ~WATER_HEAT;
            l_sys.OutWater_Delay[1] = 0;
        }
    }
    else
    {
        if ((sys_get_di_sts(DI_Heat_BPOS) == 1))  //无效
        {
        }
        else
        {
            l_sys.OutWater_Key &= ~WATER_HEAT;
            l_sys.OutWater_Delay[1] = 0;
        }
    }

    //童锁指示
    if (l_sys.ChildLock_Cnt[1])
    {
        req_bitmap_op(DO_LED_LOCK_BPOS, 1);  // LED指示,反向
    }
    else
    {
        req_bitmap_op(DO_LED_LOCK_BPOS, 0);  // LED指示
        l_sys.ChildLock_Key = 0;
    }

    return;
}
//饮水箱低水位判断
uint8_t WaterOut_level(void)
{
    extern sys_reg_st g_sys;

    uint16_t u16WL;

    //水位
    u16WL = Get_Water_level();
    if (Exit_Water() != WATER_AIR)  //外接水源
    {
        return TRUE;
    }
    else
    {
        if (!(u16WL & D_L))
        {
            return FALSE;
        }
        else
        {
            return TRUE;
        }
    }
}

// UV开关
// void UV_req_exe(uint8_t u8Type,uint8_t u8Delay)
//{
//		extern sys_reg_st		g_sys;
//
//    if (u8Type == TRUE)
//    {
//				l_sys.u16UV_Delay=g_sys.config.ComPara.u16UV_Delay*60;
//        if ((g_sys.config.ComPara.u16Sterilize_Mode&0x0F)== 1) //
//        {
//            req_bitmap_op(DO_PUREUV_BPOS, 1); //紫外灯常开
//        }
//        else if ((g_sys.config.ComPara.u16Sterilize_Mode&0x0F)== 2) //
//        {
//            req_bitmap_op(DO_UV24_BPOS, 1); //紫外灯常开
//        }
//        else if ((g_sys.config.ComPara.u16Sterilize_Mode&0x0F)== 4) //
//        {
//            req_bitmap_op(DO_PUREUV_BPOS, 1);  //紫外灯常开
//            req_bitmap_op(DO_UV24_BPOS, 1); //紫外灯常开
//        }
//        else
//        {
//            req_bitmap_op(DO_PUREUV_BPOS, 1); //紫外灯常开
//        }
//    }
//    else
//    {
//				if(g_sys.config.ComPara.u16UV_Delay==999)//常亮
//				{
//						return;
//				}
//				if (u8Delay == FALSE)//不延时
//				{
//						l_sys.u16UV_Delay=0;
//				}
//				if(l_sys.u16UV_Delay)//未到0
//				{
//						return;
//				}
//        if ((g_sys.config.ComPara.u16Sterilize_Mode&0x0F)== 1) //
//        {
//            req_bitmap_op(DO_PUREUV_BPOS, 0); //紫外灯常开
//        }
//        else if ((g_sys.config.ComPara.u16Sterilize_Mode&0x0F)== 2) //
//        {
//            req_bitmap_op(DO_UV24_BPOS, 0); //紫外灯常开
//        }
//        else if ((g_sys.config.ComPara.u16Sterilize_Mode&0x0F)== 4) //
//        {
//            req_bitmap_op(DO_PUREUV_BPOS, 0);  //紫外灯常开
//            req_bitmap_op(DO_UV24_BPOS, 0); //紫外灯常开
//        }
//        else
//        {
//            req_bitmap_op(DO_PUREUV_BPOS, 0); //紫外灯常开
//        }
//    }

//
//		//水位查看LED
//    if(!(g_sys.config.ComPara.u16Water_Ctrl & HMI_KEY)) //非按键出水,童锁
//		{
//        req_bitmap_op(DO_LED_LOCK_BPOS, 1); //
//		}
//		return;
//}
// UV开关
void UV_req_exe(uint8_t u8Type)
{
    extern sys_reg_st g_sys;

    if (u8Type == TRUE)
    {
        if (g_sys.config.ComPara.u16Sterilize_Mode == 1)  //
        {
            req_bitmap_op(DO_PUREUV_BPOS, 1);  //紫外灯常开
            req_bitmap_op(DO_UV24_BPOS, 1);    //紫外灯常开
        }
        else if (g_sys.config.ComPara.u16Sterilize_Mode == 2)  //
        {
            if (l_sys.Pwp_Open == TRUE)
            {
                req_bitmap_op(DO_PUREUV_BPOS, 1);  //紫外灯常开
                req_bitmap_op(DO_UV24_BPOS, 1);    //紫外灯常开
            }
            else
            {
                req_bitmap_op(DO_PUREUV_BPOS, 0);  //紫外灯关闭
                req_bitmap_op(DO_UV24_BPOS, 0);    //紫外灯关闭
            }
        }
        else if (g_sys.config.ComPara.u16Sterilize_Mode == 3)  //
        {
            l_sys.u16UV_Delay = g_sys.config.ComPara.u16UV_Delay * 60;
            if (l_sys.Pwp_Open == TRUE)
            {
                req_bitmap_op(DO_PUREUV_BPOS, 1);  //紫外灯常开
                req_bitmap_op(DO_UV24_BPOS, 1);    //紫外灯常开
            }
            else
            {
                if (l_sys.u16UV_Delay == 0)
                {
                    req_bitmap_op(DO_PUREUV_BPOS, 0);  //紫外灯关闭
                    req_bitmap_op(DO_UV24_BPOS, 0);    //紫外灯关闭
                }
            }
        }
        else
        {
            req_bitmap_op(DO_PUREUV_BPOS, 0);  //紫外灯常开
            req_bitmap_op(DO_UV24_BPOS, 0);    //紫外灯常开
        }
    }
    else
    {
        req_bitmap_op(DO_PUREUV_BPOS, 0);  //紫外灯常开
        req_bitmap_op(DO_UV24_BPOS, 0);    //紫外灯常开
    }

    //		//水位查看LED
    if (!(g_sys.config.ComPara.u16Water_Ctrl & HMI_KEY))  //非按键出水,童锁
    {
        req_bitmap_op(DO_LED_LOCK_BPOS, 1);  //
    }
    return;
}

//除霜
void Defrost_req_exe(void)
{
    extern sys_reg_st g_sys;

    int16_t i16NTC1;
    int16_t i16NTC2;

    if (Exit_Water() != WATER_AIR)  //外接水源
    {
        return;
    }

    i16NTC1 = (int16_t)g_sys.status.ComSta.u16Ain[AI_NTC1];
    i16NTC2 = (int16_t)g_sys.status.ComSta.u16Ain[AI_NTC2];

    // set Deforost status
    if (i16NTC1 < (int16_t)g_sys.config.ComPara.u16Start_Defrost_Temp)
    {
        sys_set_remap_status(WORK_MODE_STS_REG_NO, DEFROST1_STS_BPOS, 1);
    }
    else if (i16NTC1 > (int16_t)g_sys.config.ComPara.u16Stop_Defrost_Temp)
    {
        sys_set_remap_status(WORK_MODE_STS_REG_NO, DEFROST1_STS_BPOS, 0);
    }

    if (i16NTC2 < (int16_t)g_sys.config.ComPara.u16Start_Defrost_Temp)
    {
        sys_set_remap_status(WORK_MODE_STS_REG_NO, DEFROST2_STS_BPOS, 1);
    }
    else if (i16NTC2 > (int16_t)g_sys.config.ComPara.u16Stop_Defrost_Temp)
    {
        sys_set_remap_status(WORK_MODE_STS_REG_NO, DEFROST2_STS_BPOS, 0);
    }

    return;
}

//制冰水
void Cold_Water_exe(void)
{
    extern sys_reg_st g_sys;

    uint16_t u16WL;
    uint16_t u16Temp;
    int16_t i16Water_Temp;
    static uint8_t u8Coldwater = FALSE;

    if (Exit_Water() != WATER_AIR)  //外接水源
    {
        return;
    }
    if (!(g_sys.config.dev_mask.din[0] & D_ML))  // 3浮球
    {
        return;
    }
    u16Temp          = 0;
    l_sys.Cold_Water = FALSE;
    u16WL            = Get_Water_level();
    if (g_sys.config.ComPara.u16ColdWater_Mode == NORMAL_ICE)  //制冰水模式
    {
        u16Temp |= 0x01;
        i16Water_Temp = (int16_t)g_sys.status.ComSta.u16Ain[AI_NTC4];
        if ((u16WL & D_L) && (u16WL & D_ML) && (i16Water_Temp != ABNORMAL_VALUE))  //到达制冷水位,中水位
        {
            u16Temp |= 0x02;
            if (i16Water_Temp > g_sys.config.ComPara.u16ColdWater_StartTemp)  //开始制冰水
            {
                u16Temp |= 0x04;

                l_sys.Cold_Water = TRUE;
                if (u8Coldwater == FALSE)
                {
                    u8Coldwater         = TRUE;
                    l_sys.Cold_Delay[0] = COLD_START_DELAY;  //延时1M
                    l_sys.Cold_Delay[1] = COLD_FV_DELAY;     //延时1S
                }
                else
                {
                }
            }
            else if (i16Water_Temp < g_sys.config.ComPara.u16ColdWater_StopTemp)  //关闭
            {
                u16Temp |= 0x08;
                if (u8Coldwater == TRUE)
                {
                    u8Coldwater         = FALSE;
                    l_sys.Cold_Delay[0] = COLD_START_DELAY;  //延时1M
                    l_sys.Cold_Delay[1] = COLD_FV_DELAY;     //延时1S
                }
                else
                {
                }
            }
            else  //保持
                if (u8Coldwater == TRUE)
            {
                u16Temp |= 0x10;
                l_sys.Cold_Water = TRUE;
            }
        }
        else
        {
            u16Temp |= 0x20;
            if (u8Coldwater == TRUE)
            {
                u8Coldwater         = FALSE;
                l_sys.Cold_Delay[0] = COLD_START_DELAY;  //延时1M
                l_sys.Cold_Delay[1] = COLD_FV_DELAY;     //延时1S
            }
            else
            {
            }
        }

        //输出控制
        if (l_sys.Cold_Water == TRUE)  //制冰水
        {
            u16Temp |= 0x40;
            if (l_sys.Cold_Delay[0] == 0)  //延时开启
            {
                req_bitmap_op(DO_CV_BPOS, 1);  //制冰水
                if (l_sys.Cold_Delay[1])
                {
                    l_sys.Cold_Delay[1]--;
                }
                else
                {
#ifdef WV_TEST
                    req_bitmap_op(DO_WV_BPOS, 0);  //制冷
#else
                    req_bitmap_op(DO_WV_BPOS, 1);  //制冷
#endif
                }
            }
        }
        else
        {
            u16Temp |= 0x80;
            if (l_sys.Cold_Delay[0] == 0)
            {
#ifdef WV_TEST
                req_bitmap_op(DO_WV_BPOS, 1);  //制冷
#else
                req_bitmap_op(DO_WV_BPOS, 0);      //制冷
#endif
                if (l_sys.Cold_Delay[1])
                {
                    l_sys.Cold_Delay[1]--;
                }
                else
                {
                    req_bitmap_op(DO_CV_BPOS, 0);  //制冰水关闭
                }
            }
        }
    }
#ifdef BD_COLD
    else if (g_sys.config.ComPara.u16ColdWater_Mode == BD_ICE)  //冰胆模式
    {
        return;

        u16Temp |= 0x100;

        if ((u16WL & D_L) && (u16WL & D_ML) && (u8FCW == FALSE))  //到达制冷水位
        {
            u16Temp |= 0x200;
            u8FCW            = TRUE;
            l_sys.u16BD_Time = BD_TIME;
        }
        if (u8FCW == TRUE)
        {
            if (l_sys.u16BD_Time > 0)
            {
                u16Temp |= 0x400;
                req_bitmap_op(DO_WP_BPOS, 1);   //泵2
                req_bitmap_op(DO_EV4_BPOS, 1);  //阀4

                req_bitmap_op(DO_EV1_BPOS, 0);  //阀1

                if (g_sys.config.ComPara.u16CloseFrist == PUMP_FIRET)
                {
                    req_bitmap_op(DO_EV2_BPOS, 0);  //阀2
                }
                else
                {
                    if (l_sys.u8CloseDelay == 0)
                    {
                        req_bitmap_op(DO_EV2_BPOS, 0);  //阀2
                    }
                }
                req_bitmap_op(DO_EV3_BPOS, 0);  //阀3
            }
            else
            {
                if (g_sys.config.ComPara.u16CloseFrist == PUMP_FIRET)
                {
                    req_bitmap_op(DO_WP_BPOS, 0);  //泵2
                }
                else
                {
                    if (l_sys.u8CloseDelay == 0)
                    {
                        req_bitmap_op(DO_WP_BPOS, 0);  //泵2
                    }
                }
                req_bitmap_op(DO_EV4_BPOS, 0);  //阀4

                i16Water_Temp = (int16_t)g_sys.status.ComSta.u16Ain[AI_NTC3];
                if (i16Water_Temp == ABNORMAL_VALUE)  //温度异常
                {
                    u16Temp |= 0x800;
                    req_bitmap_op(DO_BD_BPOS, 0);      // BD
                    req_bitmap_op(DO_BD_FAN_BPOS, 0);  // BD_FAN
                }
                else
                {
                    if (i16Water_Temp > g_sys.config.ComPara.u16ColdWater_StartTemp)  //开始制冰水
                    {
                        u16Temp |= 0x1000;
                        u8Coldwater = TRUE;
                        req_bitmap_op(DO_BD_BPOS, 1);      // BD
                        req_bitmap_op(DO_BD_FAN_BPOS, 1);  // BD_FAN
                        l_sys.u16BD_FAN_Delay = BD_DELAY;
                    }
                    else if (i16Water_Temp < g_sys.config.ComPara.u16ColdWater_StopTemp)  //关闭
                    {
                        u16Temp |= 0x2000;
                        u8Coldwater = FALSE;
                        req_bitmap_op(DO_BD_BPOS, 0);  // BD
                        if (l_sys.u16BD_FAN_Delay == 0)
                        {
                            req_bitmap_op(DO_BD_FAN_BPOS, 0);  // BD_FAN
                        }
                    }
                    else  //保持
                        if (u8Coldwater == TRUE)
                    {
                        u16Temp |= 0x4000;
                        l_sys.Cold_Water = TRUE;
                        req_bitmap_op(DO_BD_BPOS, 1);      // BD
                        req_bitmap_op(DO_BD_FAN_BPOS, 1);  // BD_FAN
                        l_sys.u16BD_FAN_Delay = BD_DELAY;
                    }
                }
            }
        }
    }
#else

#endif
    else
    {
        u16Temp |= 0x8000;
#ifdef WV_TEST
        req_bitmap_op(DO_WV_BPOS, 1);  //制冷
#else
        req_bitmap_op(DO_WV_BPOS, 0);              //制冷
#endif
        req_bitmap_op(DO_CV_BPOS, 0);  //制冰水
    }
    g_sys.status.ComSta.REQ_TEST[3] = u16Temp;
    //    rt_kprintf("u16Temp=%x,i16Water_Temp=%d,Cold_Delay[0]=%d,u16ColdWater_StartTemp=%d,u16BD_Time=%d\n", u16Temp,
    //    i16Water_Temp, l_sys.Cold_Delay[0],g_sys.config.ComPara.u16ColdWater_StartTemp,l_sys.u16BD_Time);
    return;
}

//扇热风机
void Heat_Fan_exe(void)
{
    extern sys_reg_st g_sys;

    static uint8_t u8Heatfan = 0;
    int16_t i16Heat_Temp;

    if (g_sys.config.ComPara.u16ColdWater_Mode == BD_ICE)  //冰胆模式
    {
        return;
    }
    if (Exit_Water() != WATER_AIR)  //外接水源
    {
        return;
    }
    i16Heat_Temp = (int16_t)g_sys.status.ComSta.u16Ain[AI_NTC3];

    if ((g_sys.config.ComPara.u16Water_Ctrl & TWO_COLD) == 0)  //非双路出冷水
    {
        if ((i16Heat_Temp == ABNORMAL_VALUE) || (i16Heat_Temp > g_sys.config.ComPara.u16HeatFan_StartTemp))  //
        {
            u8Heatfan = TRUE;
            req_bitmap_op(DO_HEAT_FAN_BPOS, 1);  //扇热风机
        }
        else if ((i16Heat_Temp < g_sys.config.ComPara.u16HeatFan_StopTemp))  //关闭
        {
            u8Heatfan = FALSE;
            req_bitmap_op(DO_HEAT_FAN_BPOS, 0);  //
        }
        else  //保持
            if (u8Heatfan == TRUE)
        {
            req_bitmap_op(DO_HEAT_FAN_BPOS, 1);  //扇热风机
        }
    }
    return;
}

uint8_t Sys_Get_Storage_Signal(void)
{
    extern sys_reg_st g_sys;
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
    extern sys_reg_st g_sys;

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
    extern sys_reg_st g_sys;

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
uint8_t getFloatBall1(void)
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
uint8_t getFloatBall2(void)
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
uint8_t getFloatBall3(void)
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

void transformChamberLoop(FunctionalState state)
{
    if (state)
    {
        /**
         * 关闭阀3
         * 关闭电磁阀2
         */
    }
    else
    {
        /**
         * 打开阀3
         * 打开电磁阀2
         */
    }
}
void transformChamberInjection(FunctionalState state)
{
    if (state)
    {
        /**
         * 电磁阀3打开
         * 一二阀3得电
         */
    }
    else
    {
        /**
         * 电磁阀3关闭
         * 一二阀3失电
         */
    }
}
void transformChamberEmptyStateSet(FunctionalState state)
{
    static uint8_t transformChamberEmptyState = 0;
    if (state)
    {
        transformChamberEmptyState = 0;
    }
    else
    {
        transformChamberEmptyState = 1;
    }
    if (transformChamberEmptyState)
    {
    }
}
void transformChamberEmpty(void)
{
    static uint16_t transformChamberEmptyCount = 0;
    if (getFloatBall1() & FLOATBALLL)
    {
        /**
         * 一二阀3得电
         * 水泵2启动
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
            /**
             * 一二阀3失电
             * 水泵2停机
             */
        }
    }
}

void compressorWork(FunctionalState state)
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
void waterCollectStateSet(CollectState state)
{
    static CollectState waterCollectState;
    waterCollectState = state;
    if (waterCollectState)
    {
    }
}
void waterCollect(void)
{
    static CollectState waterCollectState = COLLECTIDEL;
    static uint16_t collectHalfCounter    = 0;
    switch (waterCollectState)
    {
        case COLLECTIDEL:
            /**
             * 关闭阀1
             * 关闭水泵3
             */
            break;
        case COLLECTHALF:
            if (getFloatBall2() & FLOATBALLL)
            {
                collectHalfCounter = 0;
                /**
                 * 打开阀1
                 * 打开水泵3
                 */
            }
            else
            {
                if (collectHalfCounter < COUNT3S)
                {
                    collectHalfCounter++;
                    /**
                     * 打开阀1
                     * 打开水泵3
                     */
                }
                else
                {
                    /**
                     * 关闭阀1
                     * 关闭水泵3
                     */
                    waterCollectState = COLLECTIDEL;
                }
            }
            break;
        case COLLECTFULL:
            if (getFloatBall2() & FLOATBALLH)
            {
                /**
                 * 打开阀1
                 * 打开水泵3
                 */
            }
            if (getFloatBall2() & FLOATBALLL)
            {
                /**
                 * 关闭阀1
                 * 关闭水泵3
                 */
            }

            break;
        default:
            break;
    }
}

// void automaticClean(void)
// {
//     static uint16_t automaticCleanCount = 0;
//     if (automaticCleanCount)
//         transformChamberEmpty(ENABLE);
//     waterCollectStateSet(COLLECTHALF);
// }

// void lifeWaterOut(void)
// {
//     if ((getFloatBall3() & 3) && (EVLongTou))
//     {
//         /**
//          * 水泵1启动
//          * 电磁阀5打开
//          */
//     }
//     else
//     {
//         /**
//          * 水泵1关闭
//          * 电磁阀5关闭
//          */
//     }
// }
// void pureWaterOut(void)
// {
//     static uint8_t pureWaterOutState  = 0;
//     static uint16_t pureWaterOutCount = 0;
//     if (pureWaterKey)
//     {
//         pureWaterOutState = 1;
//     }
//     else
//     {
//         pureWaterOutState = 0;
//     }

//     if (pureWaterOutState)
//     {
//         pureWaterOutCount++;
//         /**
//          * 电磁阀2打开
//          * 水泵3启动
//          */
//         if (pureWaterOutCount > T8)
//         {
//             /**
//              * 一二阀2得电
//              */
//         }
//     }
//     else
//     {
//         pureWaterOutCount = 0;
//         /**
//          * 电磁阀2关闭
//          * 水泵3停机
//          * 一二阀2断电
//          */
//     }
// }

// void hotWaterOut(void)
// {
//     static uint8_t hotWaterOutState = 0;
//     if (hotWaterKey)
//     {
//         hotWaterOutState = 1;
//     }
//     else
//     {
//         if (hotWaterOutState == 1)
//             hotWaterOutState = 0;
//     }

//     if (hotWaterOutState == 1)
//     {
//         /**
//          * 即热模块
//          */
//     }
//     else if (hotWaterOutState == 0)
//     {
//         hotWaterOutState = 2;
//         /**
//          * 即热模块
//          */
//     }
// }

void waterMakeLogic(void)
{
    static uint8_t waterMakeState  = 0;
    static uint16_t waterMakeCount = 0;
    uint16_t T3                    = 0;
    if ((getFloatBall3() & FLOATBALLM) == 0)  //浮球3中不满
    {
        waterMakeState = 1;
    }
    req_exe_log("waterMakeState:%d", waterMakeState);
    if (waterMakeState == 1)
    {
        if (getFloatBall3() & FLOATBALLH)  //浮球3上满
        {
            req_exe_log("FLOATBALLH");
            waterMakeState = 0;
            waterCollectStateSet(COLLECTIDEL);
            transformChamberLoop(DISABLE);
            transformChamberInjection(DISABLE);
            compressorWork(ENABLE);
        }
        else
        {
            req_exe_log("! FLOATBALLH");
            if (getFloatBall1() & FLOATBALLL)
            {
                transformChamberLoop(ENABLE);
                transformChamberInjection(DISABLE);
            }
            else
            {
                transformChamberLoop(DISABLE);
                transformChamberInjection(ENABLE);
            }
            compressorWork(ENABLE);
            waterCollectStateSet(COLLECTFULL);
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
            compressorWork(DISABLE);
            transformChamberEmptyStateSet(ENABLE);
            waterCollectStateSet(COLLECTHALF);
            waterMakeCount++;
        }
    }
}

//总体需求执行逻辑
void req_execution(int16_t target_req_temp, int16_t target_req_hum)
{
    //风机,压机启动条件
    Sys_Fan_CP_WL();
    //风机控制
    fan_req_exe(target_req_temp, target_req_hum);
    //压缩机控制
    compressor_req_exe(target_req_temp, target_req_hum);
    // UV开启
    UV_req_exe(TRUE);
    //制冰水
    Cold_Water_exe();
    //出水
    // WaterOut_req_exe();
    //除霜
    Defrost_req_exe();
    //显示屏控制
    Restart_DIS_exe();

    waterMakeLogic();
    return;
}
