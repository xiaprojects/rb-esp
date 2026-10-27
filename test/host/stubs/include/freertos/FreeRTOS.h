/* Host-test stub: FreeRTOS primitives as no-ops (single-threaded tests). */
#pragma once
#include <stdint.h>
#include <stddef.h>
#include "esp_attr.h"
typedef int BaseType_t;
typedef unsigned int UBaseType_t;
typedef uint32_t TickType_t;
typedef void *TaskHandle_t;
#define pdFALSE 0
#define pdTRUE 1
#define pdPASS 1
#define portMAX_DELAY ((TickType_t)0xFFFFFFFFu)
#define portTICK_PERIOD_MS 1
#define pdMS_TO_TICKS(ms) ((TickType_t)(ms))
#define portYIELD_FROM_ISR(...) ((void)0)
#define tskNO_AFFINITY 0x7FFFFFFF
