#include "ms4525do_rb.h"

#include <math.h>
#include <string.h>

#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "I2C_Driver.h"

/* --- Sensor specifics --- */
#define MS4525_COUNTS_MAX 16383.0f
#define MS4525_DEFAULT_ADDR 0x28

/* 1 PSI in Pa */
#define PSI_TO_PA 6894.757293168f

#ifndef MS4525DO_I2C_TIMEOUT_MS
#define MS4525DO_I2C_TIMEOUT_MS 20
#endif

/* Transfer function typical spans:
 *  - Output Type A: 10%..90% of counts
 *  - Output Type B: 5%..95% of counts
 */
static inline float output_min_counts(bool type_b) { return (type_b ? 0.05f : 0.10f) * MS4525_COUNTS_MAX; }
static inline float output_max_counts(bool type_b) { return (type_b ? 0.95f : 0.90f) * MS4525_COUNTS_MAX; }
static inline float output_span_counts(bool type_b) { return output_max_counts(type_b) - output_min_counts(type_b); }

/* Raw I2C read: MS4525DO has no register address, so read 4 bytes directly. */
static esp_err_t ms4525_read4(uint8_t addr_7bit, uint8_t out4[4])
{
  TickType_t to = pdMS_TO_TICKS(MS4525DO_I2C_TIMEOUT_MS);
  if (to == 0) to = 1; /* avoid 0-tick timeout if tick period > timeout */
  return i2c_master_read_from_device(I2C_MASTER_NUM, addr_7bit, out4, 4, to);
}

static esp_err_t ms4525_decode(const uint8_t buf[4],
                               uint16_t *pressure_counts,
                               uint16_t *temp_counts,
                               ms4525_status_t *status)
{
  if (!buf || !pressure_counts || !temp_counts || !status) return ESP_ERR_INVALID_ARG;

  *status = (ms4525_status_t)((buf[0] >> 6) & 0x03);

  uint16_t p = ((uint16_t)(buf[0] & 0x3F) << 8) | (uint16_t)buf[1];
  uint16_t t = ((uint16_t)buf[2] << 3) | (uint16_t)(buf[3] >> 5);

  *pressure_counts = p;
  *temp_counts = t;
  return ESP_OK;
}

static float pressure_counts_to_pa(const ms4525do_rb_t *dev, uint16_t counts)
{
  const float cmin = output_min_counts(dev->output_type_b);
  const float span = output_span_counts(dev->output_type_b);

  float c = (float)counts;
  if (c < cmin) c = cmin;
  if (c > (cmin + span)) c = cmin + span;

  float ratio = (c - cmin) / span; /* 0..1 */
  return dev->p_min_pa + ratio * (dev->p_max_pa - dev->p_min_pa);
}

static float airspeed_ms_from_dp(float dp_pa, float rho_air_kgm3)
{
  if (rho_air_kgm3 <= 0.0f) rho_air_kgm3 = 1.225f;
  float q = fabsf(dp_pa);
  return sqrtf((2.0f * q) / rho_air_kgm3);
}

void ms4525do_rb_init_1psi(ms4525do_rb_t *dev, uint8_t addr_7bit, float filter_alpha, bool output_type_b)
{
  if (!dev) return;
  memset(dev, 0, sizeof(*dev));

  dev->addr_7bit = (addr_7bit == 0) ? MS4525_DEFAULT_ADDR : addr_7bit;

  /* Assumption: +/-1 PSI differential */
  dev->p_min_pa = -PSI_TO_PA;
  dev->p_max_pa = +PSI_TO_PA;

  dev->output_type_b = output_type_b;

  if (filter_alpha < 0.0f) filter_alpha = 0.0f;
  if (filter_alpha > 1.0f) filter_alpha = 1.0f;
  dev->filter_alpha = filter_alpha;

  dev->dp_filtered_pa = 0.0f;
  dev->last_status = MS4525_STATUS_RESERVED;
}

esp_err_t ms4525do_rb_probe(ms4525do_rb_t *dev)
{
  if (!dev) return ESP_ERR_INVALID_ARG;
  uint8_t buf[4];
  return ms4525_read4(dev->addr_7bit, buf);
}

esp_err_t ms4525do_rb_update(ms4525do_rb_t *dev, float rho_air_kgm3)
{
  if (!dev) return ESP_ERR_INVALID_ARG;

  uint8_t buf[4];
  esp_err_t err = ms4525_read4(dev->addr_7bit, buf);
  if (err != ESP_OK) return err;

  uint16_t pc = 0, tc = 0;
  ms4525_status_t st = MS4525_STATUS_RESERVED;
  err = ms4525_decode(buf, &pc, &tc, &st);
  if (err != ESP_OK) return err;

  dev->last_status = st;

  /* For UI, we can ignore STALE but reject FAULT. */
  if (st == MS4525_STATUS_FAULT) return ESP_ERR_INVALID_STATE;

  float dp_pa = pressure_counts_to_pa(dev, pc);
  dev->last_dp_pa = dp_pa;

  /* Filter dp */
  if (dev->filter_alpha >= 1.0f) {
    dev->dp_filtered_pa = dp_pa;
  } else {
    dev->dp_filtered_pa = dev->dp_filtered_pa + dev->filter_alpha * (dp_pa - dev->dp_filtered_pa);
  }

  float v_ms = airspeed_ms_from_dp(dev->dp_filtered_pa, rho_air_kgm3);
  dev->last_v_ms = v_ms;
  dev->last_v_kmh = v_ms * 3.6f;

  return ESP_OK;
}

float ms4525do_rb_air_density_kgm3(float p_pa, float temp_c)
{
  const float R = 287.05f; /* J/(kg*K) */
  float T = temp_c + 273.15f;
  if (T <= 0.0f) T = 273.15f;
  if (p_pa <= 0.0f) p_pa = 101325.0f;
  return p_pa / (R * T);
}

/* --- Singleton-style convenience API --- */

static ms4525do_rb_t s_ms4525;

esp_err_t MS4525DO_Init(void)
{
  /* Defaults aligned with your previous snippet:
   *  - addr: 0x28
   *  - filter: 0.2 (smooth UI)
   *  - output type: A (10-90%)
   */
  ms4525do_rb_init_1psi(&s_ms4525, 0x28, 0.2f, false);
  return ms4525do_rb_probe(&s_ms4525);
}

esp_err_t MS4525DO_Update(float rho_air_kgm3)
{
  return ms4525do_rb_update(&s_ms4525, rho_air_kgm3);
}

float MS4525DO_AirspeedKmh(void)
{
  return ms4525do_rb_airspeed_kmh(&s_ms4525);
}

int MS4525DO_AirspeedKmh10(void)
{
  return ms4525do_rb_airspeed_kmh10(&s_ms4525);
}

