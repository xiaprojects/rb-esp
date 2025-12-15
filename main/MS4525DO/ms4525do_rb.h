#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * MS4525DO-DS5AI001DP (1 PSI differential) helper for RB02-NavCore.
 *
 * - I2C 7-bit address: 0x28 (the caller gives 0x28, not shifted)
 * - Output: 4 bytes per read, no register address.
 *
 * Units:
 *  - Differential pressure output in Pa
 *  - Airspeed in km/h and km/h*10 for RB02 UI
 *
 * IMPORTANT assumption:
 *  The "001DP" variant is treated here as a +/-1 PSI (bidirectional) sensor.
 *  If your plumbing reverses sign, airspeed uses abs(dp) anyway.
 */

typedef enum {
  MS4525_STATUS_GOOD     = 0,
  MS4525_STATUS_STALE    = 1,
  MS4525_STATUS_FAULT    = 2,
  MS4525_STATUS_RESERVED = 3
} ms4525_status_t;

typedef struct {
  uint8_t addr_7bit;          // default 0x28
  float   p_min_pa;           // default -1 PSI in Pa
  float   p_max_pa;           // default +1 PSI in Pa
  bool    output_type_b;      // false = Type A (10-90%), true = Type B (5-95%)

  // simple 1st order IIR filter on dp (Pa)
  float   dp_filtered_pa;
  float   filter_alpha;       // 0..1 (0=very smooth, 1=no filtering)

  // cached last values
  float   last_dp_pa;
  float   last_v_ms;
  float   last_v_kmh;
  ms4525_status_t last_status;
} ms4525do_rb_t;

/**
 * Init for MS4525DO-DS5AI001DP (1 PSI differential).
 * filter_alpha:
 *  - 1.0  : no filter
 *  - 0.2  : quite smooth (recommended for UI)
 */
void ms4525do_rb_init_1psi(ms4525do_rb_t *dev, uint8_t addr_7bit, float filter_alpha, bool output_type_b);

/** Probe (one read). */
esp_err_t ms4525do_rb_probe(ms4525do_rb_t *dev);

/** Read sensor, update cached values (dp, airspeed). */
esp_err_t ms4525do_rb_update(ms4525do_rb_t *dev, float rho_air_kgm3);

/** Get last computed airspeed in km/h. */
static inline float ms4525do_rb_airspeed_kmh(const ms4525do_rb_t *dev) { return dev ? dev->last_v_kmh : 0.0f; }

/** Get last computed airspeed in km/h*10 (int), for RB02 UI "speed" variable. */
static inline int ms4525do_rb_airspeed_kmh10(const ms4525do_rb_t *dev)
{
  if (!dev) return 0;
  float v = dev->last_v_kmh;
  if (v < 0) v = -v;
  return (int)(v * 10.0f + 0.5f);
}

/** Optional: compute air density from static pressure + temperature (ideal gas). */
float ms4525do_rb_air_density_kgm3(float p_pa, float temp_c);


/* --- Convenience singleton-style API (so RB02.c does not need a global instance) --- */

/** Initialize MS4525DO-DS5AI001DP on addr 0x28 with sane defaults (filter=0.2, Type A). */
esp_err_t MS4525DO_Init(void);

/** Read sensor and update internal cached values. rho_air_kgm3: use 1.225f if unknown. */
esp_err_t MS4525DO_Update(float rho_air_kgm3);

/** Get last computed airspeed in km/h. */
float MS4525DO_AirspeedKmh(void);

/** Get last computed airspeed in km/h*10 (int) for RB02 UI. */
int MS4525DO_AirspeedKmh10(void);

#ifdef __cplusplus
}
#endif
