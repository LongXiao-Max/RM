/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       remote_control.c
  * @brief      遥控器处理，利用串口空闲中断+DMA实现 DBUS/SBUS 的自适应解析，
  * 内置死区处理与结构体解耦。
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "remote_control.h"
#include "main.h"

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;

// 独立的两套解析函数声明
static void dbus_decode(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);
static void sbus_decode(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);
static void apply_deadband(RC_ctrl_t *rc_ctrl);

RC_ctrl_t rc_ctrl;
static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];

void remote_control_init(void)
{
  RC_init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
}

const RC_ctrl_t *get_remote_control_point(void)
{
  return &rc_ctrl;
}

// 串口中断
void USART3_IRQHandler(void)
{
  if (huart3.Instance->SR & UART_FLAG_RXNE)
  {
    __HAL_UART_CLEAR_PEFLAG(&huart3);
  }
  else if (USART3->SR & UART_FLAG_IDLE)
  {
    static uint16_t this_time_rx_len = 0;
    __HAL_UART_CLEAR_PEFLAG(&huart3);

    if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET)
    {
      /* Current memory buffer used is Memory 0 */
      __HAL_DMA_DISABLE(&hdma_usart3_rx);
      this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;
      hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;
      hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;
      __HAL_DMA_ENABLE(&hdma_usart3_rx);

      if (this_time_rx_len == DBUS_FRAME_LENGTH) {
        dbus_decode(sbus_rx_buf[0], &rc_ctrl);
      } else if (this_time_rx_len == SBUS_FRAME_LENGTH) {
        sbus_decode(sbus_rx_buf[0], &rc_ctrl);
      }
    }
    else
    {
      /* Current memory buffer used is Memory 1 */
      __HAL_DMA_DISABLE(&hdma_usart3_rx);
      this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;
      hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;
      DMA1_Stream1->CR &= ~(DMA_SxCR_CT);
      __HAL_DMA_ENABLE(&hdma_usart3_rx);

      if (this_time_rx_len == DBUS_FRAME_LENGTH) {
        dbus_decode(sbus_rx_buf[1], &rc_ctrl);
      } else if (this_time_rx_len == SBUS_FRAME_LENGTH) {
        sbus_decode(sbus_rx_buf[1], &rc_ctrl);
      }
    }
  }
}

/**
 * @brief          加入基础摇杆死区，防止零点漂移
 */
static void apply_deadband(RC_ctrl_t *rc_ctrl)
{
    for (uint8_t i = 0; i < 4; i++) {
        if (rc_ctrl->ch[i] <= RC_CH_DEADBAND && rc_ctrl->ch[i] >= -RC_CH_DEADBAND) {
            rc_ctrl->ch[i] = 0;
        }
    }
}

/**
 * @brief          失控保护检测（如果超过100ms没收到数据，强制归零）
 */
void RC_offline_check(void)
{
    // HAL_GetTick() 返回当前系统运行的毫秒数
    if (HAL_GetTick() - rc_ctrl.update_time > 100) 
    {
        // 1. 摇杆全部归零
        rc_ctrl.ch[0] = 0;
        rc_ctrl.ch[1] = 0;
        rc_ctrl.ch[2] = 0;
        rc_ctrl.ch[3] = 0;

        // 2. 天地飞遥控器开关全部置为最下方的安全档位
        rc_ctrl.wfly.swA = RC_SW_DOWN;
        rc_ctrl.wfly.swB = RC_SW_DOWN;
        rc_ctrl.wfly.swC = RC_SW_DOWN;
        rc_ctrl.wfly.swD = RC_SW_DOWN;

        // 3. 大疆遥控器开关也置为安全档位
        rc_ctrl.dji.s_left = RC_SW_DOWN;
        rc_ctrl.dji.s_right = RC_SW_DOWN;
        rc_ctrl.dji.wheel = 0;

        // 4. 键鼠归零
        rc_ctrl.mouse.x = 0;
        rc_ctrl.mouse.y = 0;
        rc_ctrl.mouse.z = 0;
        rc_ctrl.mouse.press_l = 0;
        rc_ctrl.mouse.press_r = 0;
        rc_ctrl.key.v = 0;
    }
}

/**
 * @brief          DBUS协议解析 (DJI原始协议)
 */
static void dbus_decode(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{
  uint16_t wheel_raw;

  if (sbus_buf == NULL || rc_ctrl == NULL) return;

    rc_ctrl->update_time = HAL_GetTick();
    
  // 基础摇杆解析
  rc_ctrl->ch[0] = ((sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff) - RC_CH_VALUE_OFFSET;
  rc_ctrl->ch[1] = (((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff) - RC_CH_VALUE_OFFSET;
  rc_ctrl->ch[2] = (((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) | (sbus_buf[4] << 10)) & 0x07ff) - RC_CH_VALUE_OFFSET;
  rc_ctrl->ch[3] = (((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff) - RC_CH_VALUE_OFFSET;
  
  apply_deadband(rc_ctrl);

  // 专属控制
  rc_ctrl->dji.s_right = ((sbus_buf[5] >> 4) & 0x0003);
  rc_ctrl->dji.s_left  = ((sbus_buf[5] >> 4) & 0x000C) >> 2;
  
  // 还原原版拨轮解析逻辑：强制转换为16位无符号数拼接，避免高位符号扩展截断，再减去偏移量
  wheel_raw = sbus_buf[16] | (sbus_buf[17] << 8);
  rc_ctrl->dji.wheel = (int16_t)wheel_raw - RC_CH_VALUE_OFFSET;

  // 键鼠数据
  rc_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);
  rc_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);
  rc_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);
  rc_ctrl->mouse.press_l = sbus_buf[12];
  rc_ctrl->mouse.press_r = sbus_buf[13];
  rc_ctrl->key.v = sbus_buf[14] | (sbus_buf[15] << 8);
}

/**
 * @brief          SBUS协议解析 (天地飞 WFLY)
 */
static void sbus_decode(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{
  int16_t raw_ch4, raw_ch5, raw_ch6, raw_ch7;

  if (sbus_buf == NULL || rc_ctrl == NULL || sbus_buf[0] != SBUS_FRAME_HEADER) return;

  rc_ctrl->update_time = HAL_GetTick();
    
  // 基础摇杆解析：彻底对齐大疆的通道顺序与正反向逻辑
  
  // ch[0]: 右摇杆左右 (SBUS通道1) - 标准方向，向右670
  rc_ctrl->ch[0] = (((sbus_buf[1]       | (sbus_buf[2]  << 8)) & 0x07FF)) - RC_CH_VALUE_OFFSET;
  
  // ch[1]: 右摇杆上下 (SBUS通道2) - 1023翻转方向，向上670
  rc_ctrl->ch[1] = 1023 - (((sbus_buf[2]  >> 3 | (sbus_buf[3]  << 5)) & 0x07FF));
  
  // ch[2]: 左摇杆左右 (SBUS通道4) - 映射到ch[2]，恢复标准方向，向右670
  rc_ctrl->ch[2] = (((sbus_buf[5]  >> 1 | (sbus_buf[6]  << 7)) & 0x07FF)) - RC_CH_VALUE_OFFSET;
  
  // ch[3]: 左摇杆上下 (SBUS通道3) - 映射到ch[3]，1023翻转方向，向上670
  rc_ctrl->ch[3] = 1023 - (((sbus_buf[3]  >> 6 | (sbus_buf[4]  << 2) | (sbus_buf[5] << 10)) & 0x07FF));

  // 摇杆死区处理
  apply_deadband(rc_ctrl);

  // 提取开关通道原始值 (仅赋值)
  raw_ch4 = (((sbus_buf[6]  >> 4 | (sbus_buf[7]  << 4)) & 0x07FF)) - RC_CH_VALUE_OFFSET;
  raw_ch5 = (((sbus_buf[7]  >> 7 | (sbus_buf[8]  << 1) | (sbus_buf[9] << 9)) & 0x07FF)) - RC_CH_VALUE_OFFSET;
  raw_ch6 = (((sbus_buf[9]  >> 2 | (sbus_buf[10] << 6)) & 0x07FF)) - RC_CH_VALUE_OFFSET;
  raw_ch7 = (((sbus_buf[10] >> 5 | (sbus_buf[11] << 3)) & 0x07FF)) - RC_CH_VALUE_OFFSET;

  // 映射到 WFLY 专属开关状态 (二档开关)
  // 物理往上打(负值) = RC_SW_UP，往下打(正值) = RC_SW_DOWN
  rc_ctrl->wfly.swA = (raw_ch4 > 0) ? RC_SW_DOWN : RC_SW_UP; 
  rc_ctrl->wfly.swD = (raw_ch7 > 0) ? RC_SW_DOWN : RC_SW_UP; 

  // 独立解析 swB (纯靠通道5，三档开关)
  if (raw_ch5 > 300) {
      rc_ctrl->wfly.swB = RC_SW_DOWN;  // 物理往下打，原始值大
  } else if (raw_ch5 < -300) {
      rc_ctrl->wfly.swB = RC_SW_UP;    // 物理往上打，原始值小(负)
  } else {
      rc_ctrl->wfly.swB = RC_SW_MID;
  }

  // 独立解析 swC (纯靠通道6，三档开关)
  if (raw_ch6 > 300) {
      rc_ctrl->wfly.swC = RC_SW_DOWN;  // 物理往下打，原始值大
  } else if (raw_ch6 < -300) {
      rc_ctrl->wfly.swC = RC_SW_UP;    // 物理往上打，原始值小(负)
  } else {
      rc_ctrl->wfly.swC = RC_SW_MID;
  }

  // 清除 SBUS 中不存在的键鼠数据
  rc_ctrl->mouse.x = 0;
  rc_ctrl->mouse.y = 0;
  rc_ctrl->mouse.z = 0;
  rc_ctrl->mouse.press_l = 0;
  rc_ctrl->mouse.press_r = 0;
  rc_ctrl->key.v = 0;
}


