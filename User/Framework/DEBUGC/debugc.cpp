#include "debugc.h"
#include <cstring>
#include <cstdlib>
#include "usart.h"
#include "usartio.h"
#include "ITtask.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart6;

DebugC debugParam;

#define TX_BUF_SIZE 512
uint8_t send_buf[TX_BUF_SIZE];

char debugRvBuff[DEBUG_RVSIZE] = { 0 };
char debugBuff[DEBUG_RVSIZE] = { 0 };
char *pEnd;
int16_t start_flag = 0;

void DEBUGC_UartIdleCallback(UART_HandleTypeDef* huart);

/**
 * @brief  初始化调试串口（USART1）的 DMA 接收与空闲中断
 *
 * 该函数开启 USART1 的 UART 空闲中断，用于检测一帧数据结束，
 * 并启动 DMA 接收，将数据写入 debugRvBuff 缓冲区。
 *
 * @note 使用 HAL_UART_Receive_DMA 启动 DMA 接收模式。
 *
 * @retval None
 */
void DEBUGC_UartInit(void)
{
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE); ///< 使能 USART1 空闲中断
    HAL_UART_Receive_DMA(&huart1, (uint8_t*)debugRvBuff, DEBUG_RVSIZE); ///< 启动 DMA 接收
}

/**
 * @brief  通过 USART1 (DMA) 发送格式化字符串
 *
 * 使用可变参数格式化输出字符串，并通过 DMA 异步发送。
 * 适合实时调试输出，不会阻塞 CPU。
 *
 * @param  format 格式化字符串（printf 风格）
 * @param  ...    可变参数
 * @retval None
 */
void usart_printf(const char* format, ...)
{
    va_list args;
    uint32_t length;
    va_start(args, format); ///< 开始读取可变参数
    length = vsnprintf((char*)send_buf, TX_BUF_SIZE, (const char*)format, args); ///< 格式化字符串
    va_end(args);
    HAL_UART_Transmit_DMA(&huart1, (uint8_t*)send_buf, length); ///< 通过 USART1 DMA 发送
}

/**
 * @brief  通过 USART6 (DMA) 发送格式化字符串
 *
 * 与 usart_printf 相同，但输出口为 USART6。
 *
 * @param  format 格式化字符串（printf 风格）
 * @param  ...    可变参数
 * @retval None
 */
void usart_printf_3pin(const char* format, ...)
{
    va_list args;
    uint32_t length;
    va_start(args, format);
    length = vsnprintf((char*)send_buf, TX_BUF_SIZE, (const char*)format, args);
    va_end(args);
    HAL_UART_Transmit_DMA(&huart6, (uint8_t*)send_buf, length); ///< 通过 USART6 DMA 发送
}

/**
 * @brief  调试串口中断处理函数
 *
 * 检测 UART 空闲中断。
 *
 * @param  huart UART 句柄
 * @retval None
 */
void DEBUGC_UartIrqHandler(UART_HandleTypeDef* huart)
{
    if (huart->Instance == USART1)
    {
        if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) != RESET) ///< 检测空闲标志
        {
            __HAL_UART_CLEAR_IDLEFLAG(huart); ///< 清除空闲标志
        }
    }
}
