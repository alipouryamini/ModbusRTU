//Modbus RTU with HAL + DMA (circular RX)-->Writer : Ali pouryamini
/*
 * UARTCallback.c
 * IRQ wrappers for Modbus RTU with HAL + DMA (circular RX)
 */

#include "Modbus.h"
#include "stm32f1xx_hal.h"
#include "main.h"

/* external modbus handler defined in main */
extern modbusHandler_t modbusH;
volatile uint32_t uartErrorCount = 0;
volatile uint32_t t = 0;
/* IDLE wrapper (call from USART IRQ after HAL_UART_IRQHandler) */
void HAL_UART_IRQHandler_Custom(UART_HandleTypeDef *huart)
{
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))
    {
        __HAL_UART_CLEAR_IDLEFLAG(huart);
        Modbus_UART_IdleCallback(huart, &modbusH);
    }
}

/* TX Complete callback called in IRQ context by HAL */
//void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
//{
  //  if (huart->Instance == USART1)
  // {
        /* set a flag for task to handle DE reset */
   //     modbus_txDoneFlag = 1;
  //  }
//}

/*void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        // غیرفعال کردن DE بلافاصله پس از اتمام ارسال
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
        modbus_txBusy = 0;
        modbus_txDoneFlag = 0;
    }
}*/

/* Optional: RxCplt (not used for IDLE processing) */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    (void)huart;
}

/* UART error callback */
/*void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
    	uartErrorCount++;
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_PEFLAG(huart);
    }
}*/

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        if (__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE))
        {
           t=1; // خطای Overrun
        }
        if (__HAL_UART_GET_FLAG(huart, UART_FLAG_NE))
        {
           t=2; // خطای Noise
        }
        if (__HAL_UART_GET_FLAG(huart, UART_FLAG_FE))
        {
           t=3; // خطای Framing
        }
        if (__HAL_UART_GET_FLAG(huart, UART_FLAG_PE))
        {
           t=4; // خطای Parity
        }

        uartErrorCount++;
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_PEFLAG(huart);
    }
}

