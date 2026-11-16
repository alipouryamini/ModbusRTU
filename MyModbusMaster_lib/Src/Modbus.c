//Modbus RTU with HAL + DMA (circular RX)-->Writer : Ali pouryamini
#include "Modbus.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <string.h>

#ifndef __HAL_DMA_GET_COUNTER
#define __HAL_DMA_GET_COUNTER(__HANDLE__) ((__HANDLE__)->Instance->CNDTR)
#endif

// CRC16 مودباس
static uint16_t modbus_crc16(uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t pos = 0; pos < len; pos++) {
        crc ^= buf[pos];
        for (int i = 0; i < 8; i++)
            crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
    }
    return crc;
}

// فعال کردن حالت ارسال RS485
static void RS485_EnableTx(ModbusMaster_HandleTypeDef *h)
{
    HAL_GPIO_WritePin(h->DEPort, h->DEPin, GPIO_PIN_SET);
    // تاخیر کوتاه برای آماده شدن خط
    for (volatile int i=0; i<1000; i++) __NOP();
}

// فعال کردن حالت دریافت RS485
static void RS485_EnableRx(ModbusMaster_HandleTypeDef *h)
{
    HAL_GPIO_WritePin(h->DEPort, h->DEPin, GPIO_PIN_RESET);
}

// ارسال داده با DMA
static int ModbusMaster_Transmit(ModbusMaster_HandleTypeDef *h, uint8_t *data, uint16_t len)
{
    if (h->txBusy) return -1;
    h->txBusy = 1;
    RS485_EnableTx(h);
    if (HAL_UART_Transmit_DMA(h->huart, data, len) != HAL_OK) {
        h->txBusy = 0;
        RS485_EnableRx(h);
        return -2;
    }
    return 0;
}

// اجرای درخواست مودباس با مدیریت تایم‌اوت و دریافت پاسخ
static ModbusMaster_StatusTypeDef ModbusMaster_ExecuteRequest(ModbusMaster_HandleTypeDef *h,
                                                              uint8_t *request, uint16_t reqLen,
                                                              uint16_t expectedRespLen)
{
    if (xSemaphoreTake(h->mutex, pdMS_TO_TICKS(500)) != pdTRUE)
        return MODBUS_MASTER_ERR_BUSY;

    memset(h->rxBuffer, 0, MODBUS_MASTER_RX_BUFFER_LEN);
    h->rxLength = 0;
    h->lastRxPos = 0;
    h->rxReady = 0;

    // ارسال درخواست
    if (ModbusMaster_Transmit(h, request, reqLen) != 0) {
        xSemaphoreGive(h->mutex);
        return MODBUS_MASTER_ERR_BUSY;
    }

    // راه‌اندازی تایمر تایم‌اوت
    if (h->htimTimeout) {
        __HAL_TIM_SET_COUNTER(h->htimTimeout, 0);
        HAL_TIM_Base_Start_IT(h->htimTimeout);
    }

    // انتظار دریافت پاسخ یا تایم‌اوت
    if (xSemaphoreTake(h->rxSemaphore, pdMS_TO_TICKS(MODBUS_MASTER_TIMEOUT_MS)) != pdTRUE) {
        if (h->htimTimeout) {
            HAL_TIM_Base_Stop_IT(h->htimTimeout);
        }
        RS485_EnableRx(h);
        h->txBusy = 0;
        xSemaphoreGive(h->mutex);
        return MODBUS_MASTER_ERR_TIMEOUT;
    }

    if (h->htimTimeout) {
        HAL_TIM_Base_Stop_IT(h->htimTimeout);
    }

    RS485_EnableRx(h);
    h->txBusy = 0;

    // بررسی حداقل طول پاسخ (ممکن است بیشتر باشد)
    if (h->rxLength < expectedRespLen) {
        xSemaphoreGive(h->mutex);
        return MODBUS_MASTER_ERR_INVALID_RESPONSE;
    }

    // محاسبه CRC
    uint16_t crc_calc = modbus_crc16(h->rxBuffer, h->rxLength - 2);
    uint16_t crc_recv = h->rxBuffer[h->rxLength - 2] | (h->rxBuffer[h->rxLength - 1] << 8);

    if (crc_calc != crc_recv) {
        xSemaphoreGive(h->mutex);
        return MODBUS_MASTER_ERR_CRC;
    }

    xSemaphoreGive(h->mutex);
    return MODBUS_MASTER_OK;
}

// خواندن رجیسترهای نگه‌دارنده (0x03)
ModbusMaster_StatusTypeDef ModbusMaster_ReadHoldingRegisters(ModbusMaster_HandleTypeDef *h,
                                                            uint16_t startAddr, uint16_t quantity,
                                                            uint16_t *regs)
{
    if (quantity == 0 || quantity > 125) return MODBUS_MASTER_ERR_INVALID_RESPONSE;

    uint16_t reqLen = 8;
    uint8_t request[8];

    request[0] = h->slaveId;
    request[1] = 0x03;
    request[2] = (startAddr >> 8) & 0xFF;
    request[3] = startAddr & 0xFF;
    request[4] = (quantity >> 8) & 0xFF;
    request[5] = quantity & 0xFF;

    uint16_t crc = modbus_crc16(request, 6);
    request[6] = crc & 0xFF;
    request[7] = (crc >> 8) & 0xFF;

    uint16_t expectedRespLen = 5 + 2 * quantity;
    if (expectedRespLen > MODBUS_MASTER_RX_BUFFER_LEN) return MODBUS_MASTER_ERR_INVALID_RESPONSE;

    ModbusMaster_StatusTypeDef status = ModbusMaster_ExecuteRequest(h, request, reqLen, expectedRespLen);

    if (status != MODBUS_MASTER_OK) return status;

    if (h->rxBuffer[1] != 0x03) return MODBUS_MASTER_ERR_INVALID_RESPONSE;

    for (uint16_t i = 0; i < quantity; i++) {
        regs[i] = (h->rxBuffer[3 + 2 * i] << 8) | h->rxBuffer[4 + 2 * i];
    }

    return MODBUS_MASTER_OK;
}

// کال‌بک اتمام ارسال
void ModbusMaster_UART_TxCpltCallback(ModbusMaster_HandleTypeDef *h)
{
    h->txBusy = 0;
    RS485_EnableRx(h);
}

// کال‌بک خطا
void ModbusMaster_UART_ErrorCallback(ModbusMaster_HandleTypeDef *h)
{
    h->txBusy = 0;
    RS485_EnableRx(h);
}

// کال‌بک وقفه IDLE UART (باید در USARTx_IRQHandler فراخوانی شود)
void ModbusMaster_UART_IdleCallback(ModbusMaster_HandleTypeDef *h)
{
    HAL_UART_DMAStop(h->huart);
    uint16_t currPos = MODBUS_MASTER_DMA_RX_BUFFER_LEN - __HAL_DMA_GET_COUNTER(h->huart->hdmarx);

    if (currPos != h->lastRxPos) {
        uint16_t len = 0;
        if (currPos > h->lastRxPos) {
            len = currPos - h->lastRxPos;
            memcpy(&h->rxBuffer[h->rxLength], &h->dmaRxBuffer[h->lastRxPos], len);
            h->rxLength += len;
        } else {
            len = MODBUS_MASTER_DMA_RX_BUFFER_LEN - h->lastRxPos;
            memcpy(&h->rxBuffer[h->rxLength], &h->dmaRxBuffer[h->lastRxPos], len);
            h->rxLength += len;

            if (currPos > 0) {
                memcpy(&h->rxBuffer[h->rxLength], &h->dmaRxBuffer[0], currPos);
                h->rxLength += currPos;
            }
        }
        h->lastRxPos = currPos;

        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        h->rxReady = 1;
        xSemaphoreGiveFromISR(h->rxSemaphore, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }

    HAL_UART_Receive_DMA(h->huart, h->dmaRxBuffer, MODBUS_MASTER_DMA_RX_BUFFER_LEN);
}

// تسک مودباس مستر (حلقه ساده برای توسعه)
void ModbusMaster_Task(void *pvParameters)
{
    ModbusMaster_HandleTypeDef *h = (ModbusMaster_HandleTypeDef *)pvParameters;


    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// تعریف تابع ModbusMaster_Init
int ModbusMaster_Init(ModbusMaster_HandleTypeDef *h, UART_HandleTypeDef *huart,
                      GPIO_TypeDef *DEPort, uint16_t DEPin, TIM_HandleTypeDef *htimTimeout,
                      uint8_t slaveId)
{
    if (!h || !huart)
        return -1;

    memset(h, 0, sizeof(ModbusMaster_HandleTypeDef));
    h->huart = huart;
    h->DEPort = DEPort;
    h->DEPin = DEPin;
    h->slaveId = slaveId;
    h->htimTimeout = htimTimeout;
    h->txBusy = 0;
    h->rxReady = 0;
    h->rxLength = 0;
    h->lastRxPos = 0;

    h->mutex = xSemaphoreCreateMutex();
    if (h->mutex == NULL)
        return -2;

    h->rxSemaphore = xSemaphoreCreateBinary();
    if (h->rxSemaphore == NULL) {
        vSemaphoreDelete(h->mutex);
        return -3;
    }

    HAL_GPIO_WritePin(h->DEPort, h->DEPin, GPIO_PIN_RESET);

    if (HAL_UART_Receive_DMA(huart, h->dmaRxBuffer, MODBUS_MASTER_DMA_RX_BUFFER_LEN) != HAL_OK)
        return -4;

    return 0;
}

// تعریف تابع ModbusMaster_StartTask
int ModbusMaster_StartTask(ModbusMaster_HandleTypeDef *h, const char *taskName, uint16_t stackSize, UBaseType_t priority)
{
    if (!h)
        return -1;

    if (xTaskCreate(ModbusMaster_Task, taskName, stackSize / 4, (void *)h, priority, &h->taskHandle) != pdPASS)
        return -2;

    return 0;
}
