#include <rtthread.h>
#include "sys_conf.h"
#include "calc.h"
#include "local_status.h"

#include "i2c_bsp.h"
#include "rtc_bsp.h"
#include "event_record.h"
#include "daq.h"
#include "sys_status.h"
#include "watchdog_bsp.h"

#include "req_execution.h"
#include "dio_bsp.h"
#include "led_bsp.h"
#include "usart_bsp.h"

static void sys_comp_cooldown(void);
static void run_time_process(void);
// static void team_tab_cooldown(void);
// static void team_timer(void);
// static void check_team_config(void);
static void sys_debug_timeout(void);
// static void analog_dummy_out(void);
// static void ec_fan_diff_req(void);
// static void ec_fan_suc_temp(void);
// extern void work_mode_manage(void);
static void temp_hum_calc(void); 
static uint16_t Set_Systime(void);

//掉电提示
void power_loss_delay(void)
{
    static uint16_t delay = 0;

    if (delay++ > 30)
    {
        delay = 11;
        sys_option_di_sts(DI_POWER_LOSS_BPOS, 0);
    }
}

/**
 * @brief 	output control module components cooldown
 * @param  none
 * @retval none
 */
void bkg_thread_entry(void *parameter)
{
    //初始化温湿度曲线记录
    extern sys_reg_st g_sys;
    rt_thread_delay(BKG_THREAD_DELAY);
    init_tem_hum_record();
    watchdog_init();

    while (1)
    {
        led_toggle();
         
        sys_comp_cooldown();
        run_time_process();
        
        temp_hum_calc();

      
        sys_running_mode_update();
        sys_debug_timeout();
        Set_Systime();
        Rtc_sts_update(&g_sys);  

        dog();
        rt_thread_delay(1000);
    }
}

static void temp_hum_calc(void)
{
    extern sys_reg_st g_sys;
    g_sys.status.sys_tem_hum.return_air_hum  = get_current_hum();
    g_sys.status.sys_tem_hum.return_air_temp = get_current_temp();

    // rt_kprintf("g_sys.status.sys_tem_hum.return_air_hum = %d\n", g_sys.status.sys_tem_hum.return_air_hum);
    // rt_kprintf("g_sys.status.sys_tem_hum.return_air_temp = %d\n", g_sys.status.sys_tem_hum.return_air_temp);
}
 

/**
 * @brief 	system components runtime counter
 * @param  none
 * @retval none
 */
//运行时间计算
static void time_calc(uint16_t *sec, uint16_t *h)
{
    uint16_t second;
    uint16_t hour;

    second = *sec;
    hour   = *h;
    if ((second & 0x0fff) >= 3600)
    {
        second = 0;
        if (second == 0)
        {
            hour++;
            *h = hour;
        }
        *sec = second;
    }
}

//流量计算
static void Flow_calc(uint16_t *u16ML, uint16_t *u16L)
{
    uint16_t ML;
    uint16_t L;
    uint16_t nL;

    ML = *u16ML;
    L  = *u16L;
    if (ML >= 1000)
    {
        nL = ML / 1000;
        L += nL;
        *u16L = L;
        ML %= 1000;
        *u16ML = ML;
    }

    return;
}

//运行时间
static void run_time_process(void)
{
    extern sys_reg_st g_sys;
    extern local_reg_st l_sys;
    //		time_t now;
    uint16_t i;
    static uint16_t u16Sec = 0;

    for (i = 0; i < DO_FILLTER_DUMMY_BPOS; i++)
    {
        if (i >= 16)  //大于16
        {
            if ((g_sys.status.dout_bitmap[1] & (0x0001 << (i - 16))) != 0)
            {
                g_sys.status.ComSta.u16Runtime[0][i]++;
                time_calc(&g_sys.status.ComSta.u16Runtime[0][i], &g_sys.status.ComSta.u16Runtime[1][i]);
            }
        }
        else
        {
            if ((g_sys.status.dout_bitmap[0] & (0x0001 << i)) != 0)
            {
                g_sys.status.ComSta.u16Runtime[0][i]++;
                time_calc(&g_sys.status.ComSta.u16Runtime[0][i], &g_sys.status.ComSta.u16Runtime[1][i]);
            }
        }
    }
    //过滤网运行时间累计
    if ((g_sys.status.dout_bitmap[0] & (0x0001 << DO_FAN_BPOS)) != 0)
    {
        g_sys.status.ComSta.u16Runtime[0][DO_FILLTER_DUMMY_BPOS]++;
        time_calc(&g_sys.status.ComSta.u16Runtime[0][DO_FILLTER_DUMMY_BPOS],
                  &g_sys.status.ComSta.u16Runtime[1][DO_FILLTER_DUMMY_BPOS]);
    }
    //滤芯运行时间累计
    if (g_sys.config.ComPara.u16FILTER_ELEMENT_Type != 0)  //时间计算
    {
        if ((sys_get_remap_status(WORK_MODE_STS_REG_NO, OUTWATER_STS_BPOS) == TRUE))  // Water out
        {
            for (i = DO_FILLTER_ELEMENT_DUMMY_BPOS_0; i <= DO_FILLTER_ELEMENT_DUMMY_MAX; i++)
            {
                g_sys.status.ComSta.u16Runtime[0][i]++;
                time_calc(&g_sys.status.ComSta.u16Runtime[0][i], &g_sys.status.ComSta.u16Runtime[1][i]);
            }
        }
        if (l_sys.OutWater_OK == WATER_OUT)
        {
            l_sys.OutWater_OK = HEATER_IDLE;
            //总流量
            g_sys.status.ComSta.u16Cumulative_Water[2] += g_sys.status.ComSta.u16Last_Water;
            Flow_calc(&g_sys.status.ComSta.u16Cumulative_Water[2], &g_sys.status.ComSta.u16Cumulative_Water[0]);
            Flow_calc(&g_sys.status.ComSta.u16Cumulative_Water[0], &g_sys.status.ComSta.u16Cumulative_Water[1]);
        }
    }
    else  //流量计算
    {
        if (l_sys.OutWater_OK == WATER_OUT)
        {
            l_sys.OutWater_OK = HEATER_IDLE;
            for (i = DO_FILLTER_ELEMENT_DUMMY_BPOS_0; i <= DO_FILLTER_ELEMENT_DUMMY_MAX; i++)
            {
                g_sys.status.ComSta.u16Runtime[0][i] += g_sys.status.ComSta.u16Last_Water;
                Flow_calc(&g_sys.status.ComSta.u16Runtime[0][i], &g_sys.status.ComSta.u16Runtime[1][i]);
            }
            //总流量
            g_sys.status.ComSta.u16Cumulative_Water[2] += g_sys.status.ComSta.u16Last_Water;                      // ml
            Flow_calc(&g_sys.status.ComSta.u16Cumulative_Water[2], &g_sys.status.ComSta.u16Cumulative_Water[0]);  // L
            Flow_calc(&g_sys.status.ComSta.u16Cumulative_Water[0], &g_sys.status.ComSta.u16Cumulative_Water[1]);
        }
    }

    // get_local_time(&now);
    // if ((now % FIXED_SAVETIME) == 0) //每15分钟保存一次
    u16Sec++;
    // rt_kprintf("adc_value=%d\n", u16Sec);
    if ((u16Sec % FIXED_SAVETIME) == 0)  //每15分钟保存一次
    {
        u16Sec = 0;
        I2C_EE_BufWrite((uint8_t *)&g_sys.status.ComSta.u16Runtime, STS_REG_EE1_ADDR,
                        sizeof(g_sys.status.ComSta.u16Runtime));  // when, fan is working update eeprom every minite
        //累计流量
        I2C_EE_BufWrite((uint8_t *)&g_sys.status.ComSta.u16Cumulative_Water[0],
                        STS_REG_EE1_ADDR + sizeof(g_sys.status.ComSta.u16Runtime), 6);  // u16Cumulative_Water,
    }
    return;
}

static void sys_comp_cooldown(void)
{
    extern local_reg_st l_sys;
    uint16_t i;

    for (i = 0; i < DO_MAX_CNT; i++)
    {
        if (l_sys.comp_timeout[i] > 0)
        {
            l_sys.comp_timeout[i]--;
        }
        else
        {
            l_sys.comp_timeout[i] = 0;
        }
    }

    if (l_sys.comp_startup_interval > 0)
    {
        l_sys.comp_startup_interval--;
    }
    else
    {
        l_sys.comp_startup_interval = 0;
    }

    if (l_sys.comp_stop_interval > 0)
    {
        l_sys.comp_stop_interval--;
    }
    else
    {
        l_sys.comp_stop_interval = 0;
    }

    if (l_sys.TH_Check_Delay > 0)
    {
        l_sys.TH_Check_Delay--;
        l_sys.TH_Check_Interval = 0;
    }
    else
    {
        l_sys.TH_Check_Delay = 0;
    }

    if (l_sys.TH_Check_Interval > 0)
    {
        l_sys.TH_Check_Interval--;
    }
    else
    {
        l_sys.TH_Check_Interval = 0;
    }

    if (l_sys.Set_Systime_Delay > 0)
    {
        l_sys.Set_Systime_Delay--;
    }
    else
    {
        l_sys.Set_Systime_Delay = 0;
    }

    //童锁
    if ((sys_get_di_sts(DI_Child_BPOS) == 1))
    {
        l_sys.ChildLock_Cnt[0]++;
    }
    else
    {
        l_sys.ChildLock_Cnt[0] = 0;
    }
    //童锁指示
    if (l_sys.ChildLock_Cnt[1])
    {
        l_sys.ChildLock_Cnt[1]--;
    }
    else
    {
        l_sys.ChildLock_Cnt[1] = 0;
    }

    //出水延时
    if (l_sys.OutWater_Delay[0])
    {
        l_sys.OutWater_Delay[0]--;
    }
    else
    {
        l_sys.OutWater_Delay[0] = 0;
    }

    //出水延时
    if (l_sys.OutWater_Delay[1])
    {
        l_sys.OutWater_Delay[1]--;
    }
    else
    {
        l_sys.OutWater_Delay[1] = 0;
    }

    if (l_sys.u16BD_Time)
    {
        l_sys.u16BD_Time--;
    }
    if (l_sys.u16BD_FAN_Delay)
    {
        l_sys.u16BD_FAN_Delay--;
    }

    //净化泵打开时间
    if (g_sys.config.ComPara.u16ExitWater_Mode == WATER_AIR)
    {
        if (l_sys.Pwp_Open == FALSE)
        {
            l_sys.Pwp_Open_Time = 0;
        }
        else
        {
            l_sys.Pwp_Open_Time++;
        }
    }
    else
    {
        l_sys.Pwp_Open_Time = 0;
    }

    // UV延迟
    if (l_sys.u16UV_Delay)
    {
        l_sys.u16UV_Delay--;
    }

    if (l_sys.Cold_Delay[0])  //延时开启制冰水
    {
        l_sys.Cold_Delay[0]--;
    }
    //注水延迟
    if (l_sys.u8ExitFill >= D_M)
    {
        l_sys.u16Fill_Delay[0] = 0;
        l_sys.u16Fill_Delay[1] = 0;
        l_sys.u16Fill_Delay[2] = 0;
    }
    else
    {
        if (l_sys.u8ExitFill < FILL_L)
        {
            l_sys.u16Fill_Delay[0]++;
        }
        else if (l_sys.u8ExitFill == FILL_L)
        {
            l_sys.u16Fill_Delay[1]++;
        }
        else if (l_sys.u8ExitFill == FILL_ML)
        {
            l_sys.u16Fill_Delay[2]++;
        }
    }
    return;
}

//通信设置系统时间
static uint16_t Set_Systime(void)
{
    extern local_reg_st l_sys;
    extern sys_reg_st g_sys;
    time_t now;
    rt_device_t device;

    if (l_sys.Set_Systime_Delay == 0)
    {
        l_sys.Set_Systime_Flag = 0;
    }
    if (l_sys.Set_Systime_Flag == 0x03)
    {
        now = g_sys.config.ComPara.u16Set_Time[1];
        now = (now << 16) | g_sys.config.ComPara.u16Set_Time[0];

        device = rt_device_find("rtc");

        if (device != RT_NULL)
        {
            rt_rtc_control(device, RT_DEVICE_CTRL_RTC_SET_TIME, &now);
            return 1;
        }
    }
    return 0;
}

static void sys_debug_timeout(void)
{
    extern local_reg_st l_sys;
    if (l_sys.debug_tiemout == DEBUG_TIMEOUT_NA)
    {
        return;
    }
    else if (l_sys.debug_tiemout > 0)
    {
        l_sys.debug_tiemout--;
    }
    else
    {
        l_sys.debug_flag    = DEBUG_OFF_FLAG;
        l_sys.debug_tiemout = 0;
    }
}
