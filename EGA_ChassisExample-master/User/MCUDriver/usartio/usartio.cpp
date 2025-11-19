//
// Created by justinwcola on 22-11-18.
//

#include "usartio.hpp"
#include "usart.h"
#include "remotec.hpp"
#include "chassisc.hpp"

/**
 * 串口相关初始化函数，在主函数中调用
 */
void USARTIO_DriverInit()
{
    chassis.rc.open();
}

/**
 * HAL库DMA空闲中断回调函数
 * @param huart 串口指针
 * @param Size  数据大小
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size)
{

}

/**
 * HAL库DMA接收完成中断回调函数
 * @param huart 串口指针
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
    chassis.rc.rxCallback(huart);
}

/**
 * HAL库DMA发送完成中断回调函数
 * @param huart 串口指针
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart)
{

}

/**
 * @brief   串口类开启函数
 */
void cUsart::open()
{
    if (type == DMA_CPLT_IT)
        HAL_UART_Receive_DMA(local_huart, rx_buf, buf_size);
    else if (type == DMA_IDLE_IT)
        HAL_UARTEx_ReceiveToIdle_DMA(local_huart, rx_buf, buf_size);
}

/**
* @brief   串口类关闭函数
*/
void cUsart::close()
{
    HAL_UART_DMAStop(local_huart);
}

/**
* @brief   串口类回调函数
*/
void cUsart::rxCallback(UART_HandleTypeDef* huart)
{
    if (huart == local_huart)
    {
        close();
        rxUserCallback(); //修改用户回调即可
        open();
    }
}

void cUsart::txCallback(UART_HandleTypeDef* huart)
{
    if (huart == local_huart)
        txUserCallback(); //修改用户回调即可
}
