//Modbus RTU with HAL + DMA (circular RX)-->Writer : Ali pouryamini

#ifndef MODBUS_CONFIG_H
#define MODBUS_CONFIG_H

// انتخاب استفاده از FreeRTOS native API (CMSIS V1 / xTaskCreate) یا CMSIS V2
#define MODBUS_USE_FREERTOS_NATIVE 1

// Max number of registers handled
#define MODBUS_MAX_REGISTERS 8

// Enable RTU via USART
#define MODBUS_USE_USART 1
#define MODBUS_RX_BUFFER_LEN 12

// Enable DMA RX idle detection support
#define MODBUS_UART_DMA_IDLE_DETECT 1

// UART handle type - adjust if you use different UART names
// The library expects you to pass &huart1 (or other) at initialization

#endif // MODBUS_CONFIG_H


