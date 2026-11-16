//created byy ali pouryamini
#ifndef MODBUS_H
#define MODBUS_H

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"

#define MODBUS_RX_BUFFER_LEN 12
#define MODBUS_MAX_REGISTERS 8

typedef struct {
    UART_HandleTypeDef *huart;
    uint8_t slaveId;
    uint16_t *holdingRegisters;
    uint16_t holdingRegsSize;
    TaskHandle_t taskHandle;
} modbusHandler_t;

int ModbusInit(modbusHandler_t *h, UART_HandleTypeDef *huart, uint8_t slaveId, uint16_t *regs, uint16_t regsSize);
int StartModbusSlaveTask(modbusHandler_t *h, const char *name, uint16_t stackSize, UBaseType_t priority);

void Modbus_UART_IdleCallback(UART_HandleTypeDef *huart, modbusHandler_t *h);
void Modbus_UART_TxCpltCallback(UART_HandleTypeDef *huart, modbusHandler_t *h);
void Modbus_ProcessFrame(uint8_t *frame, uint16_t len, modbusHandler_t *h);

extern volatile uint8_t modbus_txBusy;

#endif // MODBUS_H
