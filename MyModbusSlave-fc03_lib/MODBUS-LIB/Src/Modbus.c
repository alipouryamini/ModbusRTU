
//Modbus RTU with HAL + DMA (circular RX)-->Writer : Ali pouryamini

#include "Modbus.h"
#include "ModbusConfig.h"
#include "stm32f1xx_hal.h"
#include "string.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stream_buffer.h"
#include "main.h"

#ifndef __HAL_DMA_GET_COUNTER
#define __HAL_DMA_GET_COUNTER(__HANDLE__) ((__HANDLE__)->Instance->CNDTR)
#endif

#define RX_BUFFER_LEN MODBUS_RX_BUFFER_LEN
#define STREAM_BUFFER_SIZE 1300

#define RS485_DE_RE_GPIO_Port GPIOB
#define RS485_DE_RE_Pin GPIO_PIN_15

static uint8_t rxBuffer[RX_BUFFER_LEN];
static StreamBufferHandle_t sbh = NULL;
static modbusHandler_t *globalH = NULL;

/* TX staging buffer */
static uint8_t txBuffer[256];
static volatile uint16_t txLength = 0;
volatile uint8_t modbus_txBusy = 0;

/* Circular RX tracking */
static volatile uint16_t lastRxPos = 0;

extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern TIM_HandleTypeDef htim3;

/* Forward declarations */
static void ModbusTask(void *pvParameters);
static uint16_t modbus_crc16(uint8_t *buf, uint16_t len);
static void Modbus_SendResponse(modbusHandler_t *h, uint8_t *data, uint16_t len);

/* Delay microseconds */
static void delay_us(uint32_t us)
{
    volatile uint32_t count = us * 10; // تنظیم بر اساس فرکانس CPU
    while (count--) __NOP();
}

/* Initialization */
int ModbusInit(modbusHandler_t *h, UART_HandleTypeDef *huart, uint8_t slaveId,
               uint16_t *regs, uint16_t regsSize)
{
    if (h == NULL || huart == NULL || regs == NULL) return -1;

    memset(h, 0, sizeof(modbusHandler_t));
    h->huart = huart;
    h->slaveId = slaveId;
    h->holdingRegisters = regs;
    h->holdingRegsSize = regsSize;
    globalH = h;

    if (sbh == NULL) {
        sbh = xStreamBufferCreate(STREAM_BUFFER_SIZE, 1);
        if (sbh == NULL) return -2;
    }

    /* Ensure DE low (RX mode) */
    HAL_GPIO_WritePin(RS485_DE_RE_GPIO_Port, RS485_DE_RE_Pin, GPIO_PIN_RESET);

    /* Start circular DMA RX */
    if (HAL_UART_Receive_DMA(huart, rxBuffer, RX_BUFFER_LEN) != HAL_OK)
        return -3;

    /* last position starts at 0 */
    lastRxPos = 0;

    return 0;
}

/* Start task */
int StartModbusSlaveTask(modbusHandler_t *h, const char *name, uint16_t stackSize, UBaseType_t priority)
{
    if (h == NULL) return -1;
    if (xTaskCreate(ModbusTask, name, stackSize / 4, (void*)h, priority, &h->taskHandle) != pdPASS)
        return -2;
    return 0;
}

/* CRC16 calculation */
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

/* Process a single frame */
void Modbus_ProcessFrame(uint8_t *frame, uint16_t len, modbusHandler_t *h)
{
    if (len < 4 || frame[0] != h->slaveId) return;

    uint16_t crc_rx = (frame[len - 2]) | (frame[len - 1] << 8);
    if (modbus_crc16(frame, len - 2) != crc_rx) return;

    uint8_t func = frame[1];

    /* READ HOLDING REGISTERS (0x03) */
    if (func == 0x03)
    {
        uint16_t addr = (frame[2] << 8) | frame[3];
        uint16_t qty  = (frame[4] << 8) | frame[5];
        if (addr + qty > h->holdingRegsSize || qty == 0 || qty > 125) return;

        uint16_t idx = 0;
        txBuffer[idx++] = h->slaveId;
        txBuffer[idx++] = func;
        txBuffer[idx++] = qty * 2;

        for (uint16_t i = 0; i < qty; i++) {
            uint16_t val = h->holdingRegisters[addr + i];
            txBuffer[idx++] = val >> 8;
            txBuffer[idx++] = val & 0xFF;
        }

        uint16_t crc = modbus_crc16(txBuffer, idx);
        txBuffer[idx++] = crc & 0xFF;
        txBuffer[idx++] = (crc >> 8) & 0xFF;

        Modbus_SendResponse(h, txBuffer, idx);
        return;
    }

    /* WRITE MULTIPLE REGISTERS (0x10) */
    if (func == 0x10)
    {
        uint16_t addr = (frame[2] << 8) | frame[3];
        uint16_t qty  = (frame[4] << 8) | frame[5];
        uint8_t byteCount = frame[6];
        if (byteCount != qty * 2 || addr + qty > h->holdingRegsSize) return;

        for (uint16_t i = 0; i < qty; i++) {
            uint16_t val = (frame[7 + i * 2] << 8) | frame[8 + i * 2];
            h->holdingRegisters[addr + i] = val;
        }

        uint16_t idx = 0;
        txBuffer[idx++] = h->slaveId;
        txBuffer[idx++] = func;
        txBuffer[idx++] = frame[2];
        txBuffer[idx++] = frame[3];
        txBuffer[idx++] = frame[4];
        txBuffer[idx++] = frame[5];
        uint16_t crc = modbus_crc16(txBuffer, 6);
        txBuffer[idx++] = crc & 0xFF;
        txBuffer[idx++] = (crc >> 8) & 0xFF;

        Modbus_SendResponse(h, txBuffer, idx);
        return;
    }
}

/* IDLE callback for circular DMA RX — called from USART IRQ */
void Modbus_UART_IdleCallback(UART_HandleTypeDef *huart, modbusHandler_t *h)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    uint16_t currPos = RX_BUFFER_LEN - __HAL_DMA_GET_COUNTER(huart->hdmarx);
    uint16_t len = 0;

    if (currPos != lastRxPos)
    {
        if (currPos > lastRxPos)
        {
            len = currPos - lastRxPos;
            xStreamBufferSendFromISR(sbh, &rxBuffer[lastRxPos], len, &xHigherPriorityTaskWoken);
        }
        else
        {
            len = RX_BUFFER_LEN - lastRxPos;
            xStreamBufferSendFromISR(sbh, &rxBuffer[lastRxPos], len, &xHigherPriorityTaskWoken);
            if (currPos > 0)
            {
                xStreamBufferSendFromISR(sbh, &rxBuffer[0], currPos, &xHigherPriorityTaskWoken);
                len += currPos;
            }
        }
        lastRxPos = currPos;

        /* Reset and start frame timeout timer (TIM3) */
        __HAL_TIM_SET_COUNTER(&htim3, 0);
        HAL_TIM_Base_Start_IT(&htim3);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* TX complete callback */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        HAL_GPIO_WritePin(RS485_DE_RE_GPIO_Port, RS485_DE_RE_Pin, GPIO_PIN_RESET);
        modbus_txBusy = 0;
    }
}

/* Task: process received frames */
static void ModbusTask(void *pvParameters)
{
    modbusHandler_t *h = (modbusHandler_t*)pvParameters;
    uint8_t local[RX_BUFFER_LEN + 8];

    for (;;) {
        size_t len = xStreamBufferReceive(sbh, local, sizeof(local), portMAX_DELAY);
        if (len > 0) {
            Modbus_ProcessFrame(local, (uint16_t)len, h);
        }
    }
}

/* Helper: send response */
static void Modbus_SendResponse(modbusHandler_t *h, uint8_t *data, uint16_t len)
{
    if (h == NULL || data == NULL || len == 0) return;

    if (modbus_txBusy) return;

    if (len > sizeof(txBuffer)) return;
    memcpy(txBuffer, data, len);
    txLength = len;
    modbus_txBusy = 1;

    HAL_GPIO_WritePin(RS485_DE_RE_GPIO_Port, RS485_DE_RE_Pin, GPIO_PIN_SET);
    delay_us(50);

    HAL_UART_Transmit_DMA(h->huart, txBuffer, txLength);
}
