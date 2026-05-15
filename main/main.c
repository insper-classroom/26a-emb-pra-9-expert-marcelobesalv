#include <FreeRTOS.h>
#include <queue.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <task.h>

#include "hardware/adc.h"
#include "hardware/i2c.h"
#include "hardware/irq.h"
#include "hardware/uart.h"
#include "hc06.h"
#include "pico/stdlib.h"
#include "ssd1306.h"

#define OLED_I2C      i2c1
#define OLED_SDA_PIN  2
#define OLED_SCL_PIN  3
#define OLED_ADDR     0x3C

#define HC06_NAME "LAB-EXPERT-BT"
#define HC06_PIN  "1234"

QueueHandle_t xQueueRX;
QueueHandle_t xQueueTX;

typedef struct adc {
    int axis;
    int val;
} adc_t;

QueueHandle_t xQueueADC;

int filtro(int v) {
    int f = (v - 2048) / 8;
    if (abs(f) > 50) return f;
    else return 0;
}

void adc_task(void *pvParameters) {
    adc_t data;
    for (;;) {
        adc_select_input(0);
        data.axis = 0;
        data.val = filtro((int)adc_read());
        if (data.val != 0)
            xQueueSend(xQueueADC, &data, 0);

        adc_select_input(1);
        data.axis = 1;
        data.val = filtro((int)adc_read());
        if (data.val != 0)
            xQueueSend(xQueueADC, &data, 0);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void uart_task(void *pvParameters) {
    adc_t data;
    for (;;) {
        if (xQueueReceive(xQueueADC, &data, portMAX_DELAY)) {
            char b0 = data.axis;
            char b1 = data.val & 0xFF;
            char b2 = (data.val >> 8) & 0xFF;
            char b3 = 0xFF;
            xQueueSend(xQueueTX, &b0, 0);
            xQueueSend(xQueueTX, &b1, 0);
            xQueueSend(xQueueTX, &b2, 0);
            xQueueSend(xQueueTX, &b3, 0);
        }
    }
}

ssd1306_t disp;

void oled_task(void *pvParameters) {
    ssd1306_clear(&disp);
    ssd1306_draw_string(&disp, 0, 0, 2, "Bluetooth");
    ssd1306_draw_string(&disp, 0, 20, 2, "PIN: 1234");
    ssd1306_show(&disp);
    vTaskDelete(NULL);
}


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
    adc_init();

    init_uart_hc06();
    // hc06_config(HC06_NAME, HC06_PIN);
    adc_gpio_init(26);
    adc_gpio_init(27);

    i2c_init(OLED_I2C, 400000);
    gpio_set_function(OLED_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(OLED_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(OLED_SDA_PIN);
    gpio_pull_up(OLED_SCL_PIN);
    ssd1306_init(&disp, 128, 64, OLED_ADDR, OLED_I2C);

    xQueueADC = xQueueCreate(32, sizeof(adc_t));

    xQueueRX = xQueueCreate(256, sizeof(uint8_t));
    xQueueTX = xQueueCreate(256, sizeof(uint8_t));

    init_uart_irq();

    xTaskCreate(adc_task,  "adc_task",  configMINIMAL_STACK_SIZE * 2, NULL, 1, NULL);
    xTaskCreate(uart_task, "uart_task", configMINIMAL_STACK_SIZE * 2, NULL, 1, NULL);

    xTaskCreate(tx_task,     "TX",     512,  NULL, 2, NULL);
    xTaskCreate(serial_task, "Serial", 1024, NULL, 1, NULL);
    xTaskCreate(oled_task,   "OLED",   512,  NULL, 1, NULL);

    vTaskStartScheduler();
    while (true);
}
