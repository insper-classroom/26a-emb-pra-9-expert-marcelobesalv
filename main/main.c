#include <FreeRTOS.h>
#include <queue.h>
#include <stdio.h>
#include <string.h>
#include <task.h>

#include "hardware/irq.h"
#include "hardware/uart.h"
#include "hc06.h"
#include "pico/stdlib.h"

#define HC06_NAME "LAB-EXPERT-BT"
#define HC06_PIN  "1234"

QueueHandle_t xQueueRX;
QueueHandle_t xQueueTX;

void uart_rx_handler() {
    uint8_t ch = uart_getc(HC06_UART_ID);
    xQueueSendFromISR(xQueueRX, &ch, 0);
}

void init_uart_hc06(void) {
    uart_init(HC06_UART_ID, HC06_BAUD_RATE);

    gpio_set_function(HC06_TX_PIN,
                      UART_FUNCSEL_NUM(HC06_UART_ID, HC06_TX_PIN));
    gpio_set_function(HC06_RX_PIN,
                      UART_FUNCSEL_NUM(HC06_UART_ID, HC06_RX_PIN));

    int __unused actual = uart_set_baudrate(HC06_UART_ID, HC06_BAUD_RATE);

    uart_set_hw_flow(HC06_UART_ID, false, false);
    uart_set_format(HC06_UART_ID, 8, 1, UART_PARITY_NONE);
}

void init_uart_irq(void) {
    uart_set_fifo_enabled(HC06_UART_ID, false);

    int UART_IRQ = HC06_UART_ID == uart0 ? UART0_IRQ : UART1_IRQ;

    irq_set_exclusive_handler(UART_IRQ, uart_rx_handler);
    irq_set_priority(UART_IRQ, 0x80);
    irq_set_enabled(UART_IRQ, true);

    uart_set_irq_enables(HC06_UART_ID, true, false);
}

static void tx_task(void *p) {
    uint8_t ch;
    while (true) {
        if (xQueueReceive(xQueueTX, &ch, portMAX_DELAY) == pdTRUE) {
            uart_putc_raw(HC06_UART_ID, ch);
        }
    }
}

static void serial_task(void *p) {
    uint8_t ch;
    while (true) {
        int c = getchar_timeout_us(0);
        if (c != PICO_ERROR_TIMEOUT) {
            ch = (uint8_t)c;
            xQueueSend(xQueueTX, &ch, 0);
        }

        while (xQueueReceive(xQueueRX, &ch, 0) == pdTRUE) {
            putchar_raw(ch);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

int main(void) {
    stdio_init_all();

    init_uart_hc06();
    // hc06_config(HC06_NAME, HC06_PIN);

    xQueueRX = xQueueCreate(256, sizeof(uint8_t));
    xQueueTX = xQueueCreate(256, sizeof(uint8_t));

    init_uart_irq();

    xTaskCreate(tx_task,     "TX",     512,  NULL, 2, NULL);
    xTaskCreate(serial_task, "Serial", 1024, NULL, 1, NULL);

    vTaskStartScheduler();
    while (true);
}
