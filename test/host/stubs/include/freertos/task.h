#pragma once
#include "freertos/FreeRTOS.h"
typedef void (*TaskFunction_t)(void *);
void vTaskDelay(TickType_t ticks);
BaseType_t xTaskCreate(TaskFunction_t fn, const char *name, uint32_t stack, void *arg, UBaseType_t prio, TaskHandle_t *out);
BaseType_t xTaskCreatePinnedToCore(TaskFunction_t fn, const char *name, uint32_t stack, void *arg, UBaseType_t prio, TaskHandle_t *out, BaseType_t core);
uint32_t ulTaskNotifyTake(BaseType_t clear, TickType_t wait);
void vTaskNotifyGiveFromISR(TaskHandle_t t, BaseType_t *woken);
