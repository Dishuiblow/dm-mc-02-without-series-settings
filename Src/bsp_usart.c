#include "bsp_usart.h"
#include "main.h"

extern UART_HandleTypeDef huart10;
extern DMA_HandleTypeDef hdma_usart10_rx;
extern DMA_HandleTypeDef hdma_usart10_tx;

void usart10_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
    // 使能DMA串口接收和发送请求
    SET_BIT(huart10.Instance->CR3, USART_CR3_DMAR);
    SET_BIT(huart10.Instance->CR3, USART_CR3_DMAT);

    // 使能空闲中断
    __HAL_UART_ENABLE_IT(&huart10, UART_IT_IDLE);

    // ==========================================
    // 接收 DMA 配置
    // ==========================================
    // 失效 DMA
    __HAL_DMA_DISABLE(&hdma_usart10_rx);
    
    // 等待 DMA 彻底关闭，注意H7需要强转为 DMA_Stream_TypeDef
    while(((DMA_Stream_TypeDef *)hdma_usart10_rx.Instance)->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart10_rx);
    }

    // 清除标志位，使用通用宏适配所有流
    __HAL_DMA_CLEAR_FLAG(&hdma_usart10_rx, __HAL_DMA_GET_TC_FLAG_INDEX(&hdma_usart10_rx));

    // STM32H7 串口的接收寄存器是 RDR
    ((DMA_Stream_TypeDef *)hdma_usart10_rx.Instance)->PAR = (uint32_t) & (USART10->RDR);
    // 内存缓冲区1
    ((DMA_Stream_TypeDef *)hdma_usart10_rx.Instance)->M0AR = (uint32_t)(rx1_buf);
    // 内存缓冲区2
    ((DMA_Stream_TypeDef *)hdma_usart10_rx.Instance)->M1AR = (uint32_t)(rx2_buf);
    // 数据长度
    __HAL_DMA_SET_COUNTER(&hdma_usart10_rx, dma_buf_num);

    // 使能双缓冲区 (Double Buffer Mode)
    SET_BIT(((DMA_Stream_TypeDef *)hdma_usart10_rx.Instance)->CR, DMA_SxCR_DBM);

    // 使能 DMA 接收
    __HAL_DMA_ENABLE(&hdma_usart10_rx);

    // ==========================================
    // 发送 DMA 初始化配置
    // ==========================================
    // 失效 DMA
    __HAL_DMA_DISABLE(&hdma_usart10_tx);

    while(((DMA_Stream_TypeDef *)hdma_usart10_tx.Instance)->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart10_tx);
    }

    // STM32H7 串口的发送寄存器是 TDR
    ((DMA_Stream_TypeDef *)hdma_usart10_tx.Instance)->PAR = (uint32_t) & (USART10->TDR);
}


void usart10_tx_dma_enable(uint8_t *data, uint16_t len)
{
    // 失效 DMA
    __HAL_DMA_DISABLE(&hdma_usart10_tx);

    // 等待 DMA 彻底关闭
    while(((DMA_Stream_TypeDef *)hdma_usart10_tx.Instance)->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart10_tx);
    }

    // 清除发送完成等标志位
    __HAL_DMA_CLEAR_FLAG(&hdma_usart10_tx, __HAL_DMA_GET_TC_FLAG_INDEX(&hdma_usart10_tx));

    // 设置内存源地址和发送长度
    ((DMA_Stream_TypeDef *)hdma_usart10_tx.Instance)->M0AR = (uint32_t)(data);
    __HAL_DMA_SET_COUNTER(&hdma_usart10_tx, len);

    // 使能 DMA 触发发送
    // 注意：如果是 D-Cache 启用的区域，外部在调用此函数前必须执行 SCB_CleanDCache_by_Addr
    __HAL_DMA_ENABLE(&hdma_usart10_tx);
}
