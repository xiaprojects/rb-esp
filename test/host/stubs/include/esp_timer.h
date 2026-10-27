/* Host-test stub: monotonic timer, controllable from tests via host_stubs.h */
#pragma once
#include <stdint.h>
int64_t esp_timer_get_time(void);
