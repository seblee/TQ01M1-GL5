#ifndef __ALAMRS_H__
#define __ALAMRS_H__

#include "sys_conf.h"

#define ACL_INACTIVE 0
#define ACL_PREACTIVE 1
#define ACL_ACTIVE 2
#define ACL_POSTACTIVE 3

#define ACL_ENABLE 0
#define ACL_SUPPRESS 1
#define ACL_DISABLE 2

// alarm acl  def
enum
{
    //其他
    ACL_E0 = 0,  //无出水告警
    ACL_E1,
    ACL_E2,
    ACL_E3,
    ACL_E4,  //
    ACL_E5,  //
    ACL_E6,
    ACL_E7,
    ACL_E8,   //紫外灯杀菌未开UV1
    ACL_E9,   //源水箱故障，复合滤芯堵塞
    ACL_E10,  //漏水
    ACL_E11,  //排气高温
    ACL_E12,  //排气高温锁死
    ACL_E13,  //滤网
    ACL_E14,  //滤芯1
    ACL_E15,  //滤芯2
    ACL_E16,  //滤芯3
    ACL_E17,  //注水故障
    ACL_E18,  //滤芯5
    ACL_E19,  //
    ACL_E20,
    ACL_E21,  //
    ACL_E22,
    ACL_E23,
    ACL_E24,
    ACL_E25,
    ACL_E26,
    ACL_E27,
    ACL_E28,
    ACL_E29,
    //异常
    ACL_TOTAL_NUM,

};
#define ACL_FAN01_OD ACL_E7  //风机故障
#define ACL_UV1_OT ACL_E8    //紫外灯1超时
#define ACL_WATER_LEAK ACL_E10
#define ACL_SYS01_EXHAUST_HI ACL_E11
#define ACL_SYS01_EXHAUST_HI_LOCK ACL_E12
#define ACL_FILTER_OT ACL_E13            //滤网
#define ACL_FILTER_ELEMENT_0_OT ACL_E14  //滤芯1
#define ACL_FILTER_ELEMENT_1_OT ACL_E15  //滤芯2
#define ACL_FILTER_ELEMENT_2_OT ACL_E16  //滤芯3
#define ACL_FILTER_ELEMENT_4_OT ACL_E18
#define ACL_UV2_OT ACL_E19

#define ACL_RS_NETERR ACL_E20
#define ACL_HI_PRESS1 ACL_E21
#define ACL_HI_PRESS2 ACL_E22
#define ACL_J25_HI_TEM ACL_E23
#define ACL_J25_BALL1 ACL_E24
#define ACL_J25_BALL2 ACL_E25
#define ACL_J25_BALL3 ACL_E26
#define ACL_J25_COLLECT_TIME_OUT ACL_E27
#define ACL_J25_UV_TIME_OUT ACL_E28
#define ACL_J25_UV_FEEDBACK_TIME_OUT ACL_E29

#define RT_MS 1000

// Alair,20161227
enum
{
    DEV_RETURN_SENSOR1_FAULT_BPOS = 0,
    DEV_RETURN_SENSOR2_FAULT_BPOS,
    DEV_RETURN_SENSOR3_FAULT_BPOS,
    DEV_RETURN_SENSOR4_FAULT_BPOS,
    DEV_SUPPLY_SENSOR1_FAULT_BPOS,
    DEV_SUPPLY_SENSOR2_FAULT_BPOS,
    //
    DEV_TEM_HUM_RESERVE1_FAULT_BPOS,
    DEV_TEM_HUM_RESERVE2_FAULT_BPOS,
    //
    DEV_NTC7_BPOS,
    DEV_NTC8_BPOS,
    DEV_CIOL_NTC1_FAULT_BPOS,
    DEV_CIOL_NTC2_FAULT_BPOS,
    DEV_RETUREN_NTC1_FAULT_BPOS,
    DEV_RETUREN_NTC2_FAULT_BPOS,
    DEV_SUPPLY_NTC1_FAULT_BPOS,
    DEV_SUPPLY_NTC2_FAULT_BPOS,
};

void alarm_acl_init(void);
void alarm_acl_exe(void);

void In_alarm_stats(void);

uint8_t get_alarm_bitmap(uint8_t alarm_id);

// uint8_t clear_alarm(void);
uint8_t clear_alarm(uint8_t alarm_id);
uint8_t get_alarm_bitmap_mask(uint8_t component_bpos);
uint8_t get_alarm_bitmap_op(uint8_t component_bpos);
uint8_t alarm_Off(void);

#endif  //__ALAMRS_H__
