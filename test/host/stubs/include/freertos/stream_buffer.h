#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/task.h" /* IDF headers pull task.h in transitively */
typedef void *StreamBufferHandle_t;
StreamBufferHandle_t xStreamBufferCreate(size_t size, size_t trigger);
size_t xStreamBufferSend(StreamBufferHandle_t h, const void *data, size_t len, TickType_t wait);
size_t xStreamBufferReceive(StreamBufferHandle_t h, void *data, size_t len, TickType_t wait);
size_t xStreamBufferBytesAvailable(StreamBufferHandle_t h);
