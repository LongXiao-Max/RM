#include "bsp_rc.h"
#include "main.h"

// 引用外部定义的串口和 DMA 句柄
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;

/**
 * @brief  初始化遥控器接收配置 (UART3 + DMA双缓存模式)
 * @param  rx1_buf: 内存缓冲区 1 地址
 * @param  rx2_buf: 内存缓冲区 2 地址
 * @param  dma_buf_num: 缓冲区长度 (通常为遥控器数据包大小)
 */
void RC_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
    // 1. 开启串口的 DMA 接收请求 (配置 USART3 寄存器)
    SET_BIT(huart3.Instance->CR3, USART_CR3_DMAR);

    // 2. 开启串口空闲中断 (IDLE Interrupt)，用于判断一帧数据接收完毕
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);

    // 3. 暂时关闭 DMA 以便进行寄存器配置
    __HAL_DMA_DISABLE(&hdma_usart3_rx);
    
    // 确认 DMA 已完全停止
    while(hdma_usart3_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart3_rx);
    }

    // 4. 设置 DMA 外设基地址 (指向 USART3 数据寄存器)
    hdma_usart3_rx.Instance->PAR = (uint32_t) & (USART3->DR);

    // 5. 设置双缓存地址
    hdma_usart3_rx.Instance->M0AR = (uint32_t)(rx1_buf); // 缓冲区 0
    hdma_usart3_rx.Instance->M1AR = (uint32_t)(rx2_buf); // 缓冲区 1

    // 6. 设置 DMA 传输数据量
    hdma_usart3_rx.Instance->NDTR = dma_buf_num;

    // 7. 开启 DMA 双缓冲模式 (Double Buffer Mode)
    // 可以在 DMA 填充一个缓存时，CPU 处理另一个，实现无缝接收
    SET_BIT(hdma_usart3_rx.Instance->CR, DMA_SxCR_DBM);

    // 8. 重新开启 DMA，开始接收数据
    __HAL_DMA_ENABLE(&hdma_usart3_rx);
}

