#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/uart.h"

#define ADC_MAX_VALUE 4095
#define OUTPUT_MAX_VALUE 255
#define DEAD_ZONE 100

const uint ADC_PIN_X = 26; // GPIO 26, que é o canal ADC 0
const uint ADC_PIN_Y = 27; // GPIO 27, que é o canal ADC 1

QueueHandle_t xQueueADC;

typedef struct {
    int axis;
    int val;
} adc_t;

int map_value(int x, int in_min, int in_max, int out_min, int out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int process_adc_reading(int adc_value) {
    int mapped = map_value(adc_value, 0, ADC_MAX_VALUE, -OUTPUT_MAX_VALUE, OUTPUT_MAX_VALUE);
    if (mapped > -DEAD_ZONE && mapped < DEAD_ZONE) {
        return 0;  // Zona morta
    }
    return mapped;
}

void x_task(void *params) {
    adc_t adc_reading = {.axis = 0};
    while (1) {
        adc_select_input(0);
        adc_reading.val = process_adc_reading(adc_read());
        xQueueSend(xQueueADC, &adc_reading, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void y_task(void *params) {
    adc_t adc_reading = {.axis = 1};
    while (1) {
        adc_select_input(1);
        int raw_value = adc_read();
        printf("Raw Y: %d\n", raw_value);  // Debug: imprimir valor bruto
        adc_reading.val = process_adc_reading(raw_value);
        printf("Processed Y: %d\n", adc_reading.val);  // Debug: imprimir valor processado
        xQueueSend(xQueueADC, &adc_reading, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}


void uart_task(void *params) {
    adc_t adc_reading;
    while (1) {
        if (xQueueReceive(xQueueADC, &adc_reading, portMAX_DELAY)) {
            uart_putc(uart0, adc_reading.axis);
            uart_putc(uart0, adc_reading.val & 0xFF);
            uart_putc(uart0, (adc_reading.val >> 8) & 0xFF);
        }
    }
}

int main() {
    stdio_init_all();
    adc_init();
    adc_gpio_init(ADC_PIN_X);
    adc_gpio_init(ADC_PIN_Y);

    uart_init(uart0, 115200);
    gpio_set_function(PICO_DEFAULT_UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(PICO_DEFAULT_UART_RX_PIN, GPIO_FUNC_UART);

    xQueueADC = xQueueCreate(10, sizeof(adc_t));

    xTaskCreate(x_task, "X Axis Task", 256, NULL, 1, NULL);
    xTaskCreate(y_task, "Y Axis Task", 256, NULL, 1, NULL);
    xTaskCreate(uart_task, "UART Task", 256, NULL, 1, NULL);

    vTaskStartScheduler();

    while (true);
    return 0;
}
