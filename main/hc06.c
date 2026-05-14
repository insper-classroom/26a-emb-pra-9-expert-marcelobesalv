#include <stdio.h>
#include <string.h>
#include "hc06.h"

#define TIMEOUT_US   1200
#define AT_WAIT_MS   500
#define CMD_WAIT_MS  1500
#define MAX_ATTEMPTS 5

static void hc06_limpar_rx(void) {
    while (uart_is_readable_within_us(HC06_UART_ID, TIMEOUT_US)) {
        uart_getc(HC06_UART_ID);
    }
}

static int hc06_ler_resposta(char *buf, int len) {
    int i = 0;
    while (i < len - 1 &&
           uart_is_readable_within_us(HC06_UART_ID, TIMEOUT_US)) {
        buf[i++] = uart_getc(HC06_UART_ID);
    }
    buf[i] = '\0';
    return i;
}

bool hc06_set_at_mode(int on) {
    gpio_init(HC06_ENABLE_PIN);
    gpio_set_dir(HC06_ENABLE_PIN, GPIO_OUT);
    gpio_put(HC06_ENABLE_PIN, on ? 1 : 0);
    sleep_ms(AT_WAIT_MS);
    return true;
}

bool hc06_check_connection(void) {
    char buf[32];
    hc06_limpar_rx();
    uart_puts(HC06_UART_ID, "AT");
    sleep_ms(AT_WAIT_MS);
    hc06_ler_resposta(buf, sizeof(buf));
    return strstr(buf, "OK") != NULL;
}

bool hc06_set_name(char name[]) {
    char cmd[64];
    char buf[32];
    snprintf(cmd, sizeof(cmd), "AT+NAME%s", name);
    hc06_limpar_rx();
    uart_puts(HC06_UART_ID, cmd);
    sleep_ms(CMD_WAIT_MS);
    hc06_ler_resposta(buf, sizeof(buf));
    return strstr(buf, "OKsetname") != NULL;
}

bool hc06_set_pin(char pin[]) {
    char cmd[32];
    char buf[32];
    snprintf(cmd, sizeof(cmd), "AT+PIN%s", pin);
    hc06_limpar_rx();
    uart_puts(HC06_UART_ID, cmd);
    sleep_ms(CMD_WAIT_MS);
    hc06_ler_resposta(buf, sizeof(buf));
    return strstr(buf, "OKsetPIN") != NULL;
}

bool hc06_set_baud_115200(void) {
    char buf[32];
    hc06_limpar_rx();
    uart_puts(HC06_UART_ID, "AT+BAUD8");
    sleep_ms(CMD_WAIT_MS);
    hc06_ler_resposta(buf, sizeof(buf));
    return strstr(buf, "OK115200") != NULL;
}

static bool hc06_tentar_baud(int baud) {
    uart_init(HC06_UART_ID, baud);
    gpio_set_function(HC06_RX_PIN,
                      UART_FUNCSEL_NUM(HC06_UART_ID, HC06_RX_PIN));
    gpio_set_function(HC06_TX_PIN,
                      UART_FUNCSEL_NUM(HC06_UART_ID, HC06_TX_PIN));
    uart_set_hw_flow(HC06_UART_ID, false, false);
    uart_set_format(HC06_UART_ID, 8, 1, UART_PARITY_NONE);

    for (int i = 0; i < MAX_ATTEMPTS; i++) {
        if (hc06_check_connection())
            return true;
    }
    return false;
}

bool hc06_config(char name[], char pin[]) {
    hc06_set_at_mode(1);

    printf("HC06: trying 9600...\n");
    bool found = hc06_tentar_baud(9600);

    if (!found) {
        printf("HC06: trying 115200...\n");
        found = hc06_tentar_baud(115200);
        if (!found) {
            printf("HC06: module not responding!\n");
            while (1);
        }
        printf("HC06: already at 115200\n");
    } else {
        printf("HC06: connected at 9600, upgrading baud...\n");
        if (!hc06_set_baud_115200()) {
            printf("HC06: baud upgrade failed!\n");
            while (1);
        }
        uart_init(HC06_UART_ID, 115200);
        gpio_set_function(HC06_RX_PIN,
                          UART_FUNCSEL_NUM(HC06_UART_ID, HC06_RX_PIN));
        gpio_set_function(HC06_TX_PIN,
                          UART_FUNCSEL_NUM(HC06_UART_ID, HC06_TX_PIN));
    }

    if (!hc06_set_name(name)) {
        printf("HC06: set name failed!\n");
        while (1);
    }
    printf("HC06: name set to %s\n", name);

    if (!hc06_set_pin(pin)) {
        printf("HC06: set pin failed!\n");
        while (1);
    }
    printf("HC06: pin set to %s\n", pin);

    hc06_set_at_mode(0);
    printf("HC06: config complete\n");
    return true;
}
