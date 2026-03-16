/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       remote_control.h
  * @brief      遥控器处理，支持 DBUS (18字节) 和 SBUS (25字节) 协议自适应解析
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H
#include "stdint.h"
#include "bsp_rc.h"

/* ----------------------- RC Protocol Definitions --------------------------- */
#define SBUS_RX_BUF_NUM 36u         // 接收缓冲区大小，需大于最大帧长
#define DBUS_FRAME_LENGTH 18u       // DBUS帧长18字节 (DJI原始协议)
#define SBUS_FRAME_LENGTH 25u       // SBUS标准帧长25字节
#define SBUS_FRAME_HEADER 0x0Fu     // SBUS帧头

/* ----------------------- RC Value Definitions ------------------------------ */
#define RC_CH_VALUE_MIN ((uint16_t)364)
#define RC_CH_VALUE_OFFSET ((uint16_t)1024)
#define RC_CH_VALUE_MAX ((uint16_t)1684)

// 摇杆死区设置，可根据实际遥控器磨损情况调整
#define RC_CH_DEADBAND 10

/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP ((uint16_t)1)
#define RC_SW_MID ((uint16_t)3)
#define RC_SW_DOWN ((uint16_t)2)
#define switch_is_down(s) (s == RC_SW_DOWN)
#define switch_is_mid(s) (s == RC_SW_MID)
#define switch_is_up(s) (s == RC_SW_UP)

/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_PRESSED_OFFSET_W ((uint16_t)1 << 0)
#define KEY_PRESSED_OFFSET_S ((uint16_t)1 << 1)
#define KEY_PRESSED_OFFSET_A ((uint16_t)1 << 2)
#define KEY_PRESSED_OFFSET_D ((uint16_t)1 << 3)
#define KEY_PRESSED_OFFSET_SHIFT ((uint16_t)1 << 4)
#define KEY_PRESSED_OFFSET_CTRL ((uint16_t)1 << 5)
#define KEY_PRESSED_OFFSET_Q ((uint16_t)1 << 6)
#define KEY_PRESSED_OFFSET_E ((uint16_t)1 << 7)
#define KEY_PRESSED_OFFSET_R ((uint16_t)1 << 8)
#define KEY_PRESSED_OFFSET_F ((uint16_t)1 << 9)
#define KEY_PRESSED_OFFSET_G ((uint16_t)1 << 10)
#define KEY_PRESSED_OFFSET_Z ((uint16_t)1 << 11)
#define KEY_PRESSED_OFFSET_X ((uint16_t)1 << 12)
#define KEY_PRESSED_OFFSET_C ((uint16_t)1 << 13)
#define KEY_PRESSED_OFFSET_V ((uint16_t)1 << 14)
#define KEY_PRESSED_OFFSET_B ((uint16_t)1 << 15)

/* ----------------------- Data Struct ------------------------------------- */
typedef __packed struct
{
  // 1. 基础摇杆数据 (大疆和天地飞共用前四个通道，已处理死区)
  int16_t ch[4]; 

  // 2. 大疆 DR16 专用控制
  __packed struct {
    uint8_t s_left;   // 左三档开关
    uint8_t s_right;  // 右三档开关
    int16_t wheel;    // 左上角拨轮
  } dji;

  // 3. 天地飞 WFLY (SBUS) 专用控制
  __packed struct {
    uint8_t swA;      // 二档开关
    uint8_t swB;      // 三档开关
    uint8_t swC;      // 三档开关
    uint8_t swD;      // 二档开关
  } wfly;

  // 4. 键鼠数据 (仅连接大疆接收机时有效)
  __packed struct {
    int16_t x;
    int16_t y;
    int16_t z;
    uint8_t press_l;
    uint8_t press_r;
  } mouse;
  __packed struct {
    uint16_t v;
  } key;
  //增加有效数据的时间戳
  uint32_t update_time;

} RC_ctrl_t;

/* ----------------------- Internal Data ----------------------------------- */
extern void remote_control_init(void);
extern const RC_ctrl_t *get_remote_control_point(void);
//增加失控保护检测函数声明
extern void RC_offline_check(void);

#endif
