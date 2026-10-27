#include "CST820.h"

#define POINT_NUM_MAX       (1)

#define DATA_START_REG      (0x15)
#define CHIP_ID_REG         (0x01)
#define TOUCH_NUM           (0x02)
#define TOUCH_POSITION      (0x03)

static const char *TAG = "cst820";


static esp_err_t read_data(esp_lcd_touch_handle_t tp);
static bool get_xy(esp_lcd_touch_handle_t tp, uint16_t *x, uint16_t *y, uint16_t *strength, uint8_t *point_num, uint8_t max_point_num);
static esp_err_t del(esp_lcd_touch_handle_t tp);

static esp_err_t i2c_read_bytes(esp_lcd_touch_handle_t tp, uint16_t reg, uint8_t *data, uint8_t len);
static esp_err_t touch_cst820_i2c_write(esp_lcd_touch_handle_t tp, uint16_t reg, uint8_t *data, uint8_t len);

static esp_err_t reset(esp_lcd_touch_handle_t tp);
static esp_err_t read_id(esp_lcd_touch_handle_t tp);

esp_err_t esp_lcd_touch_new_i2c_cst820(const esp_lcd_panel_io_handle_t io, const esp_lcd_touch_config_t *config, esp_lcd_touch_handle_t *tp)
{
    ESP_RETURN_ON_FALSE(io, ESP_ERR_INVALID_ARG, TAG, "Invalid io");
    ESP_RETURN_ON_FALSE(config, ESP_ERR_INVALID_ARG, TAG, "Invalid config");
    ESP_RETURN_ON_FALSE(tp, ESP_ERR_INVALID_ARG, TAG, "Invalid touch handle");

    /* Prepare main structure */
    esp_err_t ret = ESP_OK;
    esp_lcd_touch_handle_t cst820 = calloc(1, sizeof(esp_lcd_touch_t));
    ESP_GOTO_ON_FALSE(cst820, ESP_ERR_NO_MEM, err, TAG, "Touch handle malloc failed");

    /* Communication interface */
    cst820->io = io;
    /* Only supported callbacks are set */
    cst820->read_data = read_data;
    cst820->get_xy = get_xy;
    cst820->del = del;
    /* Mutex */
    cst820->data.lock.owner = portMUX_FREE_VAL;
    /* Save config */
    memcpy(&cst820->config, config, sizeof(esp_lcd_touch_config_t));

    /* Prepare pin for touch interrupt */
    if (cst820->config.int_gpio_num != GPIO_NUM_NC) {
        const gpio_config_t int_gpio_config = {
            .mode = GPIO_MODE_INPUT,
            .intr_type = GPIO_INTR_NEGEDGE,
            .pin_bit_mask = BIT64(cst820->config.int_gpio_num)
        };
        ESP_GOTO_ON_ERROR(gpio_config(&int_gpio_config), err, TAG, "GPIO intr config failed");

        /* Register interrupt callback */
        if (cst820->config.interrupt_callback) {
            esp_lcd_touch_register_interrupt_callback(cst820, cst820->config.interrupt_callback);
        }
    }
    /* Prepare pin for touch controller reset */
    if (cst820->config.rst_gpio_num != GPIO_NUM_NC) {
        const gpio_config_t rst_gpio_config = {
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = BIT64(cst820->config.rst_gpio_num)
        };
        ESP_GOTO_ON_ERROR(gpio_config(&rst_gpio_config), err, TAG, "GPIO reset config failed");
    }


    /* Reset controller */
    ESP_GOTO_ON_ERROR(reset(cst820), err, TAG, "Reset failed");

    /* Read product id */
    ESP_GOTO_ON_ERROR(read_id(cst820), err, TAG, "Read version failed");
    *tp = cst820;

    return ESP_OK;
err:
    if (cst820) {
        del(cst820);
    }
    ESP_LOGE(TAG, "Initialization failed!");
    return ret;
}

static esp_err_t read_data(esp_lcd_touch_handle_t tp)
{
    esp_err_t err;
    uint8_t buf[41];
    uint8_t touch_cnt = 0;
    uint8_t clear = 0;
    uint8_t Over = 0xAB;
    size_t i = 0;
    uint8_t close = 1;

    assert(tp != NULL);
    // Waveshare official bugfix to the CST820 driver, the touch controller does not clear the touch data after reading, so we need to clear it manually.
    // uint8_t write_buf = 0x01;
    // i2c_master_write_to_device(0, DATA_START_REG, &write_buf, 1, 1000 / portTICK_PERIOD_MS);

    // touch_cst820_i2c_write(tp, 0xFE, &close, 1);

    err = i2c_read_bytes(tp, TOUCH_NUM, buf, 1);
    ESP_RETURN_ON_ERROR(err, TAG, "I2C read error!");
    // touch_cst820_i2c_write(tp, TOUCH_POSITION, &Over, 1);

    if ((buf[0] & 0x0F) == 0x00) {
        // touch_cst820_i2c_write(tp, TOUCH_NUM, &clear, 1);  
    } else {
        /* Count of touched points */
        touch_cnt = buf[0] & 0x0F;
        if (touch_cnt > 2 || touch_cnt == 0) {
            // touch_cst820_i2c_write(tp, TOUCH_NUM, &clear, 1);
            return ESP_OK;
        }

        /* Read all points */
        err = i2c_read_bytes(tp, TOUCH_POSITION, &buf[0], touch_cnt * 6);
        ESP_RETURN_ON_ERROR(err, TAG, "I2C read error!");
        // touch_cst820_i2c_write(tp, TOUCH_POSITION, &Over, 1);

        /* Clear all */
        // err = touch_cst820_i2c_write(tp, TOUCH_NUM, &clear, 1);

        taskENTER_CRITICAL(&tp->data.lock);

        /* Number of touched points */
        if(touch_cnt > CONFIG_ESP_LCD_TOUCH_MAX_POINTS)
            touch_cnt = CONFIG_ESP_LCD_TOUCH_MAX_POINTS;
        tp->data.points = (uint8_t)touch_cnt;
        
        /* Fill all coordinates */
        for (i = 0; i < touch_cnt; i++) {
            tp->data.coords[i].x = (uint16_t)(((uint16_t)(buf[(i * 6) ] & 0x0F) << 8) + (buf[(i * 6) + 1]));               
            tp->data.coords[i].y = (uint16_t)(((uint16_t)(buf[(i * 6) + 2] & 0x0F) << 8) + (buf[(i * 6) + 3])); 
            tp->data.coords[i].strength = 50;
        }

        taskEXIT_CRITICAL(&tp->data.lock);
    }

    return ESP_OK;
}

static bool get_xy(esp_lcd_touch_handle_t tp, uint16_t *x, uint16_t *y, uint16_t *strength, uint8_t *point_num, uint8_t max_point_num)
{
    portENTER_CRITICAL(&tp->data.lock);
    /* Count of points */
    *point_num = (tp->data.points > max_point_num ? max_point_num : tp->data.points);
    for (size_t i = 0; i < *point_num; i++) {
        x[i] = tp->data.coords[i].x;
        y[i] = tp->data.coords[i].y;

        if (strength) {
            strength[i] = tp->data.coords[i].strength;
        }
    }
    /* Invalidate */
    tp->data.points = 0;
    portEXIT_CRITICAL(&tp->data.lock);

    return (*point_num > 0);
}

static esp_err_t del(esp_lcd_touch_handle_t tp)
{
    /* Reset GPIO pin settings */
    if (tp->config.int_gpio_num != GPIO_NUM_NC) {
        gpio_reset_pin(tp->config.int_gpio_num);
    }
    if (tp->config.rst_gpio_num != GPIO_NUM_NC) {
        gpio_reset_pin(tp->config.rst_gpio_num);
    }
    /* Release memory */
    free(tp);

    return ESP_OK;
}

static esp_err_t reset(esp_lcd_touch_handle_t tp)
{

    Set_EXIO(TCA9554_EXIO2,false);
    vTaskDelay(pdMS_TO_TICKS(10));
    Set_EXIO(TCA9554_EXIO2,true);
    vTaskDelay(pdMS_TO_TICKS(50));
    return ESP_OK;
}

static esp_err_t read_id(esp_lcd_touch_handle_t tp)
{
    uint8_t id=0x10;
    uint8_t Over = 0x01;

    // touch_cst820_i2c_write(tp, DATA_START_REG, &Over, 1);

    ESP_RETURN_ON_ERROR(i2c_read_bytes(tp, CHIP_ID_REG, &id, 1), TAG, "I2C read failed");
    ESP_LOGI(TAG, "IC id: %d", id);
    return ESP_OK;
}


static esp_err_t i2c_read_bytes(esp_lcd_touch_handle_t tp, uint16_t reg, uint8_t *data, uint8_t len)
{
    assert(tp != NULL);
    assert(data != NULL);

    /* Read data */
    return esp_lcd_panel_io_rx_param(tp->io, reg, data, len);
}

static esp_err_t touch_cst820_i2c_write(esp_lcd_touch_handle_t tp, uint16_t reg, uint8_t* data, uint8_t len)
{
    assert(tp != NULL);

    // *INDENT-OFF*
    /* Write data */
    return esp_lcd_panel_io_tx_param(tp->io, reg, data, len);
    // *INDENT-ON*
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
esp_lcd_touch_handle_t tp = NULL;
void Touch_Init(void)
{
    
/********************* Touch *********************/

    esp_lcd_panel_io_handle_t tp_io_handle = NULL;
    esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_CST820_CONFIG();
    ESP_LOGI(TAG, "Initialize touch IO (I2C)");
    /* Touch IO handle */
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)I2C_MASTER_NUM, &tp_io_config, &tp_io_handle));
    esp_lcd_touch_config_t tp_cfg = {
        .x_max = EXAMPLE_LCD_V_RES,
        .y_max = EXAMPLE_LCD_H_RES,
        .rst_gpio_num = I2C_Touch_RST_IO,
        .int_gpio_num = I2C_Touch_INT_IO,
        .flags = {
            .swap_xy = 0,
            .mirror_x = 0,
            .mirror_y = 0,
        },
    };
    /* Initialize touch */
    ESP_LOGI(TAG, "Initialize touch controller CST820");
    ESP_ERROR_CHECK(esp_lcd_touch_new_i2c_cst820(tp_io_handle, &tp_cfg, &tp));
}