#include "FreeRTOS.h"
#include "ei_accelerometer.h"
#include "ei_analogsensor.h"
#include "ei_at_handlers.h"
#include "ei_classifier_porting.h"
#include "ei_device_raspberry_rp2xxx.h"
#include "ei_dht11sensor.h"
#include "ei_inertialsensor.h"
#include "ei_rp2xxx_internal_temperature.h"
#include "ei_run_impulse.h"
#include "ei_ultrasonicsensor.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "task.h"
#include <stdio.h>
#include <string.h>
#include <time.h>

// imu
#include <hardware/gpio.h>
#include <hardware/i2c.h>
#include <hardware/uart.h>
#include <pico/stdio.h>

// freertos
#include <FreeRTOS.h>
#include <queue.h>
#include <semphr.h>
#include <stdlib.h>
#include <task.h>

// // específico
#include "mpu6050.h"
// edited
// -- Adicione estas 3 linhas em main.cpp --
#include "edge-impulse-sdk/classifier/ei_model_types.h"
#include "edge-impulse-sdk/dsp/numpy.hpp"
#include "model-parameters/model_metadata.h"

using namespace ei;

extern "C" EI_IMPULSE_ERROR
run_classifier(ei::signal_t *signal, ei_impulse_result_t *result, bool debug);

static bool debug_nn = false;

const int MPU_ADDRESS = 0x68;
const int I2C_SDA_GPIO = 4;
const int I2C_SCL_GPIO = 5;

// LED RGB para indicar o movimento detectado (uma cor por movimento).
// I2C usa GPIO 4/5, então estes pinos estão livres.
const int LED_PIN_R = 16;
const int LED_PIN_G = 17;
const int LED_PIN_B = 18;

// Assume LED RGB de cátodo comum: gpio_put(pin, 1) acende.
// Para ânodo comum, inverta a lógica em set_rgb().
static void rgb_led_init()
{
    const int pins[] = { LED_PIN_R, LED_PIN_G, LED_PIN_B };
    for (int i = 0; i < 3; i++) {
        gpio_init(pins[i]);
        gpio_set_dir(pins[i], GPIO_OUT);
        gpio_put(pins[i], 0);
    }
}

static void set_rgb(bool r, bool g, bool b)
{
    gpio_put(LED_PIN_R, r);
    gpio_put(LED_PIN_G, g);
    gpio_put(LED_PIN_B, b);
}

static void mpu6050_init()
{
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(I2C_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_GPIO);
    gpio_pull_up(I2C_SCL_GPIO);

    // Two byte reset. First byte register, second byte data
    // There are a load more options to set up the device in different ways that could be added here
    uint8_t buf[] = { 0x6B, 0x00 };
    i2c_write_blocking(i2c_default, MPU_ADDRESS, buf, 2, false);
}

static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp)
{
    uint8_t buffer[14];

    // Read all data sequentially starting from acceleration registers (0x3B)
    // 0x3B-0x40: acceleration (6 bytes)
    // 0x41-0x42: temperature (2 bytes)
    // 0x43-0x48: gyro (6 bytes)
    uint8_t val = 0x3B;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 14, false);

    // Parse acceleration
    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    // Parse temperature
    *temp = buffer[6] << 8 | buffer[7];

    // Parse gyro
    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[8 + i * 2] << 8 | buffer[8 + (i * 2) + 1]);
    }
}

static void gesture_recognize_task(void *p)
{
    mpu6050_init();
    rgb_led_init();

    // Autoteste do LED no boot: acende R, G, B (1s cada) e avisa na serial.
    // Se voce ver as 3 cores aqui, os pinos/fiacao estao OK.
    ei_printf("LED self-test: RED\n");
    set_rgb(1, 0, 0);
    vTaskDelay(pdMS_TO_TICKS(1000));
    ei_printf("LED self-test: GREEN\n");
    set_rgb(0, 1, 0);
    vTaskDelay(pdMS_TO_TICKS(1000));
    ei_printf("LED self-test: BLUE\n");
    set_rgb(0, 0, 1);
    vTaskDelay(pdMS_TO_TICKS(1000));
    set_rgb(0, 0, 0);
    ei_printf("LED self-test: done\n");

    int16_t accelerometer[3], gyro[3], temp;

    while (true) {
        //        ei_printf("\nStarting inferencing in 2 seconds...\n");
        //        vTaskDelay(pdMS_TO_TICKS(2000));
        //        ei_printf("Sampling...\n");

        float buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = { 0 };

        for (size_t ix = 0; ix < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE; ix += 3) {
            mpu6050_read_raw(accelerometer, gyro, &temp);
            buffer[ix + 0] = accelerometer[0];
            buffer[ix + 1] = accelerometer[1];
            buffer[ix + 2] = accelerometer[2];

            vTaskDelay(pdMS_TO_TICKS(10));
        }

        // Prepara sinal
        ei::signal_t signal;
        int err = numpy::signal_from_buffer(buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
        if (err != 0) {
            ei_printf("Failed to create signal from buffer (%d)\n", err);
            break;
        }

        // Run the classifier
        ei_impulse_result_t result = { 0 };
        err = run_classifier(&signal, &result, debug_nn);
        if (err != EI_IMPULSE_OK) {
            ei_printf("ERR: Failed to run classifier (%d)\n", err);
            break;
        }

        // print the predictions
        ei_printf("Predictions ");
        ei_printf(
            "(DSP: %d ms., Classification: %d ms., Anomaly: %d ms.)",
            result.timing.dsp,
            result.timing.classification,
            result.timing.anomaly);
        ei_printf(": \n");

        // Acha o label de maior confiança enquanto imprime as predições.
        size_t best_ix = 0;
        float best_value = 0.0f;
        for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
            ei_printf(
                "teste    %s: %.5f\n",
                result.classification[ix].label,
                result.classification[ix].value);

            if (result.classification[ix].value > best_value) {
                best_value = result.classification[ix].value;
                best_ix = ix;
            }
        }

        // Acende uma cor por movimento detectado.
        const char *winner = result.classification[best_ix].label;
        if (strcmp(winner, "idle") == 0) {
            set_rgb(0, 1, 0); // verde 17
        } else if (strcmp(winner, "updown") == 0) {
            set_rgb(0, 0, 1); // azul 18
        } else if (strcmp(winner, "waving") == 0) {
            set_rgb(1, 0, 0); // vermelho 16
        } else {
            set_rgb(0, 0, 0); // desconhecido: apaga
        }
        ei_printf("=> movimento: %s\n", winner);

#if EI_CLASSIFIER_HAS_ANOMALY == 1
        ei_printf("    anomaly score: %.3f\n", result.anomaly);
#endif
    }
}

int main(void)
{
    stdio_init_all();

    xTaskCreate(gesture_recognize_task, "gesture_task 1", 8192, NULL, 1, NULL);
    vTaskStartScheduler();

    while (true)
        ;
}
