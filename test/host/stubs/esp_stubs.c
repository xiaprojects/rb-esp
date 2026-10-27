/*
 * Host fakes for ESP-IDF, FreeRTOS and the SC16IS750 driver.
 * Everything here is a no-op or a simple recorder; see host_stubs.h.
 */
#include <stdlib.h>
#include <string.h>
#include "host_stubs.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/stream_buffer.h"
#include "sc16is750.h"

/* ---- esp_timer ---------------------------------------------------------- */
static int64_t s_now_us = 0;

int64_t esp_timer_get_time(void) { return s_now_us; }
void host_stub_set_time_us(int64_t us) { s_now_us = us; }
void host_stub_advance_ms(uint32_t ms) { s_now_us += (int64_t)ms * 1000; }
uint32_t host_stub_now_ms(void) { return (uint32_t)(s_now_us / 1000); }

/* ---- gpio --------------------------------------------------------------- */
esp_err_t gpio_config(const gpio_config_t *cfg) { (void)cfg; return ESP_OK; }
esp_err_t gpio_install_isr_service(int flags) { (void)flags; return ESP_OK; }
esp_err_t gpio_isr_handler_add(gpio_num_t n, gpio_isr_t h, void *arg) { (void)n; (void)h; (void)arg; return ESP_OK; }
int gpio_get_level(gpio_num_t n) { (void)n; return 1; }

/* ---- FreeRTOS ----------------------------------------------------------- */
void vTaskDelay(TickType_t ticks) { (void)ticks; }
BaseType_t xTaskCreate(TaskFunction_t fn, const char *name, uint32_t stack, void *arg, UBaseType_t prio, TaskHandle_t *out)
{
    (void)fn; (void)name; (void)stack; (void)arg; (void)prio;
    if (out) *out = NULL;
    return pdPASS;
}
BaseType_t xTaskCreatePinnedToCore(TaskFunction_t fn, const char *name, uint32_t stack, void *arg, UBaseType_t prio, TaskHandle_t *out, BaseType_t core)
{
    (void)core;
    return xTaskCreate(fn, name, stack, arg, prio, out);
}
uint32_t ulTaskNotifyTake(BaseType_t clear, TickType_t wait) { (void)clear; (void)wait; return 0; }
void vTaskNotifyGiveFromISR(TaskHandle_t t, BaseType_t *woken) { (void)t; if (woken) *woken = pdFALSE; }

StreamBufferHandle_t xStreamBufferCreate(size_t size, size_t trigger) { (void)size; (void)trigger; return (StreamBufferHandle_t)1; }
size_t xStreamBufferSend(StreamBufferHandle_t h, const void *data, size_t len, TickType_t wait) { (void)h; (void)data; (void)wait; return len; }
size_t xStreamBufferReceive(StreamBufferHandle_t h, void *data, size_t len, TickType_t wait) { (void)h; (void)data; (void)len; (void)wait; return 0; }
size_t xStreamBufferBytesAvailable(StreamBufferHandle_t h) { (void)h; return 0; }

/* ---- SC16IS750 (I2C/SPI UART + GPIO expander) --------------------------- */
void    SC16IS750_init(SC16IS750_t *dev, uint8_t prtcl, uint8_t addr, int channels) { (void)dev; (void)prtcl; (void)addr; (void)channels; }
void    SC16IS750_begin(SC16IS750_t *dev, uint32_t baud_A, uint32_t baud_B, long crystal_freq) { (void)dev; (void)baud_A; (void)baud_B; (void)crystal_freq; }
uint8_t SC16IS750_GPIOGetPortState(SC16IS750_t *dev) { (void)dev; return 0xFF; /* pull-ups idle */ }
uint8_t SC16IS750_InterruptPendingTest(SC16IS750_t *dev, uint8_t channel) { (void)dev; (void)channel; return 0; }
int16_t SC16IS750_InterruptEventTest(SC16IS750_t *dev, uint8_t channel) { (void)dev; (void)channel; return 0; }
void    SC16IS750_SetPinInterrupt(SC16IS750_t *dev, uint8_t io_int_ena) { (void)dev; (void)io_int_ena; }
void    SC16IS750_InterruptControl(SC16IS750_t *dev, uint8_t channel, uint8_t int_ena) { (void)dev; (void)channel; (void)int_ena; }
void    SC16IS750_SetLine(SC16IS750_t *dev, uint8_t channel, uint8_t data_length, uint8_t parity_select, uint8_t stop_length) { (void)dev; (void)channel; (void)data_length; (void)parity_select; (void)stop_length; }
void    SC16IS750_GPIOSetPinMode(SC16IS750_t *dev, uint8_t pin_number, uint8_t i_o) { (void)dev; (void)pin_number; (void)i_o; }
void    SC16IS750_FIFOEnable(SC16IS750_t *dev, uint8_t channel, uint8_t fifo_enable) { (void)dev; (void)channel; (void)fifo_enable; }
void    SC16IS750_FIFOReset(SC16IS750_t *dev, uint8_t channel, uint8_t rx_fifo) { (void)dev; (void)channel; (void)rx_fifo; }
void    SC16IS750_FIFOSetTriggerLevel(SC16IS750_t *dev, uint8_t channel, uint8_t rx_fifo, uint8_t length) { (void)dev; (void)channel; (void)rx_fifo; (void)length; }
uint8_t SC16IS750_FIFOAvailableData(SC16IS750_t *dev, uint8_t channel) { (void)dev; (void)channel; return 0; }
int     SC16IS750_ReadByte(SC16IS750_t *dev, uint8_t channel) { (void)dev; (void)channel; return -1; }
void    SC16IS750_EnableRs485(SC16IS750_t *dev, uint8_t channel, uint8_t invert) { (void)dev; (void)channel; (void)invert; }
