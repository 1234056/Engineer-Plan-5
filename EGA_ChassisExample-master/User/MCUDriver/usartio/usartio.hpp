//
// Created by justinwcola on 22-11-18.
//

#ifndef USARTIO_HPP
#define USARTIO_HPP

#include "usart.h"

typedef enum
{
    DMA_IDLE_IT = 0, //DMA空闲中断
    DMA_CPLT_IT = 1  //DMA接收完成中断
} eUsartType;

#ifdef __cplusplus

class cUsart
{
public:
    cUsart(UART_HandleTypeDef* local_huart, uint16_t buf_size, eUsartType type) :
        local_huart(local_huart), buf_size(buf_size), type(type)
    {
    }

    virtual void open();
    void         close();
    void         rxCallback(UART_HandleTypeDef* huart);
    void         txCallback(UART_HandleTypeDef* huart);

    virtual void rxUserCallback()
    {
    };

    virtual void txUserCallback()
    {
    };

    UART_HandleTypeDef* local_huart{};

protected:
    eUsartType type;
    uint16_t   buf_size;
    uint8_t    rx_buf[255] = {0};
};

#endif

void USARTIO_DriverInit();

#endif //USARTIO_HPP
