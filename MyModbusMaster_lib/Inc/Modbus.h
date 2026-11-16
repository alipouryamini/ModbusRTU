//Modbus RTU with HAL + DMA (circular RX)-->Writer : Ali pouryamini
#ifndef MODBUS_H
#define MODBUS_H

#include "stm32f4xx_hal.h"  // متناسب با MCU شما
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"


#define MODBUS_MASTER_RX_BUFFER_LEN 256
#define MODBUS_MASTER_TIMEOUT_MS 1000
#define MODBUS_MASTER_DMA_RX_BUFFER_LEN 256

typedef enum {
    MODBUS_MASTER_OK = 0,
    MODBUS_MASTER_ERR_BUSY,
    MODBUS_MASTER_ERR_TIMEOUT,
    MODBUS_MASTER_ERR_INVALID_RESPONSE,
    MODBUS_MASTER_ERR_CRC,
} ModbusMaster_StatusTypeDef;

typedef struct {
    UART_HandleTypeDef *huart;
    GPIO_TypeDef *DEPort;
    uint16_t DEPin;
    TIM_HandleTypeDef *htimTimeout;

    uint8_t slaveId;

    uint8_t dmaRxBuffer[MODBUS_MASTER_DMA_RX_BUFFER_LEN];
    uint8_t rxBuffer[MODBUS_MASTER_RX_BUFFER_LEN];
    uint16_t rxLength;
    uint16_t lastRxPos;
    uint8_t rxReady;

    uint8_t txBusy;

    SemaphoreHandle_t mutex;
    SemaphoreHandle_t rxSemaphore;

    TaskHandle_t taskHandle;
} ModbusMaster_HandleTypeDef;


int ModbusMaster_Init(ModbusMaster_HandleTypeDef *h, UART_HandleTypeDef *huart,
                      GPIO_TypeDef *DEPort, uint16_t DEPin, TIM_HandleTypeDef *htimTimeout,
                      uint8_t slaveId);

int ModbusMaster_StartTask(ModbusMaster_HandleTypeDef *h, const char *taskName, uint16_t stackSize, UBaseType_t priority);

ModbusMaster_StatusTypeDef ModbusMaster_ReadHoldingRegisters(ModbusMaster_HandleTypeDef *h,
                                                            uint16_t startAddr, uint16_t quantity,
                                                            uint16_t *regs);

void ModbusMaster_UART_TxCpltCallback(ModbusMaster_HandleTypeDef *h);
void ModbusMaster_UART_ErrorCallback(ModbusMaster_HandleTypeDef *h);
void ModbusMaster_UART_IdleCallback(ModbusMaster_HandleTypeDef *h);

void ModbusMaster_Task(void *pvParameters);

#endif // MODBUS_H
