#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define I2C_PORT I2C_NUM_0
#define I2C_SDA_PIN GPIO_NUM_21
#define I2C_SCL_PIN GPIO_NUM_22
#define GREEN_LED_PIN GPIO_NUM_25
#define YELLOW_LED_PIN GPIO_NUM_26
#define RED_LED_PIN GPIO_NUM_27
#define BUZZER_PIN GPIO_NUM_18

#define MPU6050_ADDRESS 0x68
#define OLED_ADDRESS 0x3C
#define OLED_WIDTH 128
#define OLED_HEIGHT 64

#define WARNING_THRESHOLD 1.2f
#define ALERT_THRESHOLD 2.5f
#define ALERT_CLEAR_THRESHOLD 0.8f

#define SAMPLE_INTERVAL_MS 50
#define DISPLAY_INTERVAL_MS 200
#define ALERT_CONFIRM_MS 500
#define ALERT_CLEAR_MS 1000
#define SAMPLE_COUNT 5

typedef struct {
    float x;
    float y;
    float z;
    float vibration;
} motion_data_t;

typedef enum {
    STATE_NORMAL,
    STATE_WARNING,
    STATE_ALERT
} system_state_t;

static const char *TAG = "seismic";
static uint8_t display_buffer[OLED_WIDTH * OLED_HEIGHT / 8];
static float vibration_samples[SAMPLE_COUNT];
static int sample_index;
static int samples_stored;
static float sample_total;
static system_state_t current_state = STATE_NORMAL;
static bool high_timer_active;
static bool quiet_timer_active;
static int64_t high_vibration_start;
static int64_t quiet_start;

static const uint8_t font[37][5] = {
    {0x00, 0x00, 0x00, 0x00, 0x00},
    {0x7E, 0x11, 0x11, 0x11, 0x7E}, {0x7F, 0x49, 0x49, 0x49, 0x36},
    {0x3E, 0x41, 0x41, 0x41, 0x22}, {0x7F, 0x41, 0x41, 0x22, 0x1C},
    {0x7F, 0x49, 0x49, 0x49, 0x41}, {0x7F, 0x09, 0x09, 0x09, 0x01},
    {0x3E, 0x41, 0x49, 0x49, 0x7A}, {0x7F, 0x08, 0x08, 0x08, 0x7F},
    {0x00, 0x41, 0x7F, 0x41, 0x00}, {0x20, 0x40, 0x41, 0x3F, 0x01},
    {0x7F, 0x08, 0x14, 0x22, 0x41}, {0x7F, 0x40, 0x40, 0x40, 0x40},
    {0x7F, 0x02, 0x0C, 0x02, 0x7F}, {0x7F, 0x04, 0x08, 0x10, 0x7F},
    {0x3E, 0x41, 0x41, 0x41, 0x3E}, {0x7F, 0x09, 0x09, 0x09, 0x06},
    {0x3E, 0x41, 0x51, 0x21, 0x5E}, {0x7F, 0x09, 0x19, 0x29, 0x46},
    {0x46, 0x49, 0x49, 0x49, 0x31}, {0x01, 0x01, 0x7F, 0x01, 0x01},
    {0x3F, 0x40, 0x40, 0x40, 0x3F}, {0x1F, 0x20, 0x40, 0x20, 0x1F},
    {0x3F, 0x40, 0x38, 0x40, 0x3F}, {0x63, 0x14, 0x08, 0x14, 0x63},
    {0x07, 0x08, 0x70, 0x08, 0x07}, {0x61, 0x51, 0x49, 0x45, 0x43},
    {0x3E, 0x51, 0x49, 0x45, 0x3E}, {0x00, 0x42, 0x7F, 0x40, 0x00},
    {0x42, 0x61, 0x51, 0x49, 0x46}, {0x21, 0x41, 0x45, 0x4B, 0x31},
    {0x18, 0x14, 0x12, 0x7F, 0x10}, {0x27, 0x45, 0x45, 0x45, 0x39},
    {0x3C, 0x4A, 0x49, 0x49, 0x30}, {0x01, 0x71, 0x09, 0x05, 0x03},
    {0x36, 0x49, 0x49, 0x49, 0x36}, {0x06, 0x49, 0x49, 0x29, 0x1E}
};

static esp_err_t i2c_write(uint8_t address, const uint8_t *data, size_t length)
{
    return i2c_master_write_to_device(
        I2C_PORT, address, data, length, pdMS_TO_TICKS(100));
}

static esp_err_t i2c_read_registers(
    uint8_t address, uint8_t reg, uint8_t *data, size_t length)
{
    return i2c_master_write_read_device(
        I2C_PORT, address, &reg, 1, data, length, pdMS_TO_TICKS(100));
}

static esp_err_t mpu_write_register(uint8_t reg, uint8_t value)
{
    uint8_t data[] = {reg, value};
    return i2c_write(MPU6050_ADDRESS, data, sizeof(data));
}

static esp_err_t mpu_init(void)
{
    uint8_t identity;
    esp_err_t error = i2c_read_registers(
        MPU6050_ADDRESS, 0x75, &identity, sizeof(identity));
    if (error != ESP_OK) {
        return error;
    }
    if (identity != 0x68) {
        return ESP_ERR_NOT_FOUND;
    }

    error = mpu_write_register(0x6B, 0x00);
    if (error == ESP_OK) error = mpu_write_register(0x1A, 0x04);
    if (error == ESP_OK) error = mpu_write_register(0x1C, 0x08);
    return error;
}

static esp_err_t read_motion(motion_data_t *motion)
{
    uint8_t raw[6];
    esp_err_t error = i2c_read_registers(MPU6050_ADDRESS, 0x3B, raw, sizeof(raw));
    if (error != ESP_OK) {
        return error;
    }

    int16_t raw_x = (int16_t)((raw[0] << 8) | raw[1]);
    int16_t raw_y = (int16_t)((raw[2] << 8) | raw[3]);
    int16_t raw_z = (int16_t)((raw[4] << 8) | raw[5]);
    const float acceleration_scale = 9.80665f / 8192.0f;

    motion->x = raw_x * acceleration_scale;
    motion->y = raw_y * acceleration_scale;
    motion->z = raw_z * acceleration_scale;
    float total_acceleration = sqrtf(
        motion->x * motion->x +
        motion->y * motion->y +
        motion->z * motion->z);
    motion->vibration = fabsf(total_acceleration - 9.80665f);
    return ESP_OK;
}

static float average_vibration(float new_sample)
{
    if (samples_stored == SAMPLE_COUNT) {
        sample_total -= vibration_samples[sample_index];
    } else {
        samples_stored++;
    }

    vibration_samples[sample_index] = new_sample;
    sample_total += new_sample;
    sample_index = (sample_index + 1) % SAMPLE_COUNT;
    return sample_total / samples_stored;
}

static void update_state(float vibration, int64_t now_ms)
{
    if (current_state == STATE_ALERT) {
        if (vibration < ALERT_CLEAR_THRESHOLD) {
            if (!quiet_timer_active) {
                quiet_start = now_ms;
                quiet_timer_active = true;
            }
            if (now_ms - quiet_start >= ALERT_CLEAR_MS) {
                current_state = STATE_NORMAL;
                quiet_timer_active = false;
                high_timer_active = false;
            }
        } else {
            quiet_timer_active = false;
        }
        return;
    }

    if (vibration >= ALERT_THRESHOLD) {
        if (!high_timer_active) {
            high_vibration_start = now_ms;
            high_timer_active = true;
        }
        if (now_ms - high_vibration_start >= ALERT_CONFIRM_MS) {
            current_state = STATE_ALERT;
            quiet_timer_active = false;
            return;
        }
    } else {
        high_timer_active = false;
    }

    current_state = vibration >= WARNING_THRESHOLD
        ? STATE_WARNING
        : STATE_NORMAL;
}

static const char *state_name(system_state_t state)
{
    if (state == STATE_ALERT) return "ALERT";
    if (state == STATE_WARNING) return "WARNING";
    return "NORMAL";
}

static void update_outputs(void)
{
    gpio_set_level(GREEN_LED_PIN, current_state == STATE_NORMAL);
    gpio_set_level(YELLOW_LED_PIN, current_state == STATE_WARNING);
    gpio_set_level(RED_LED_PIN, current_state == STATE_ALERT);
    gpio_set_level(BUZZER_PIN, current_state == STATE_ALERT);
}

static const uint8_t *get_glyph(char character)
{
    static const uint8_t blank[5] = {0, 0, 0, 0, 0};
    static const uint8_t colon[5] = {0, 0x36, 0x36, 0, 0};
    static const uint8_t period[5] = {0, 0x60, 0x60, 0, 0};
    static const uint8_t dash[5] = {0x08, 0x08, 0x08, 0x08, 0x08};

    if (character >= 'a' && character <= 'z') {
        character = (char)(character - 'a' + 'A');
    }
    if (character >= 'A' && character <= 'Z') {
        return font[1 + character - 'A'];
    }
    if (character >= '0' && character <= '9') {
        return font[27 + character - '0'];
    }
    if (character == ':') return colon;
    if (character == '.') return period;
    if (character == '-') return dash;
    return blank;
}

static void draw_text(int x, int y, const char *text)
{
    for (size_t index = 0; text[index] != '\0'; index++) {
        const uint8_t *glyph = get_glyph(text[index]);
        for (int column = 0; column < 5; column++) {
            for (int row = 0; row < 7; row++) {
                if ((glyph[column] & (1U << row)) == 0) continue;
                int pixel_x = x + column;
                int pixel_y = y + row;
                if (pixel_x >= 0 && pixel_x < OLED_WIDTH &&
                    pixel_y >= 0 && pixel_y < OLED_HEIGHT) {
                    size_t offset = (size_t)(pixel_y / 8) * OLED_WIDTH + pixel_x;
                    display_buffer[offset] |= (uint8_t)(1U << (pixel_y % 8));
                }
            }
        }
        x += 6;
    }
}

static esp_err_t oled_refresh(void)
{
    const uint8_t set_window[] = {0x00, 0x21, 0, OLED_WIDTH - 1,
                                  0x22, 0, OLED_HEIGHT / 8 - 1};
    esp_err_t error = i2c_write(OLED_ADDRESS, set_window, sizeof(set_window));
    if (error != ESP_OK) return error;

    uint8_t packet[129];
    packet[0] = 0x40;
    for (size_t offset = 0; offset < sizeof(display_buffer); offset += 128) {
        memcpy(&packet[1], &display_buffer[offset], 128);
        error = i2c_write(OLED_ADDRESS, packet, sizeof(packet));
        if (error != ESP_OK) return error;
    }
    return ESP_OK;
}

static esp_err_t oled_init(void)
{
    const uint8_t commands[] = {
        0x00, 0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00,
        0x40, 0x8D, 0x14, 0x20, 0x00, 0xA1, 0xC8, 0xDA,
        0x12, 0x81, 0xCF, 0xD9, 0xF1, 0xDB, 0x40, 0xA4,
        0xA6, 0xAF
    };
    return i2c_write(OLED_ADDRESS, commands, sizeof(commands));
}

static void update_display(float vibration)
{
    memset(display_buffer, 0, sizeof(display_buffer));
    draw_text(0, 0, "SEISMIC MONITOR");
    draw_text(0, 16, "VIBRATION M/S2");

    int hundredths = (int)lroundf(vibration * 100.0f);
    char value[16];
    snprintf(value, sizeof(value), "%d.%02d", hundredths / 100, hundredths % 100);
    draw_text(0, 32, value);
    draw_text(0, 48, state_name(current_state));

    esp_err_t error = oled_refresh();
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "OLED update failed: %s", esp_err_to_name(error));
    }
}

static void configure_gpio(void)
{
    gpio_config_t output_config = {
        .pin_bit_mask = (1ULL << GREEN_LED_PIN) |
                        (1ULL << YELLOW_LED_PIN) |
                        (1ULL << RED_LED_PIN) |
                        (1ULL << BUZZER_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&output_config));
    update_outputs();
}

static void configure_i2c(void)
{
    i2c_config_t config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &config));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT, config.mode, 0, 0, 0));
}

void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(100));
    configure_gpio();
    configure_i2c();
    vTaskDelay(pdMS_TO_TICKS(50));

    int retries = 5;
    while (mpu_init() != ESP_OK && retries > 0) {
        ESP_LOGW(TAG, "Waiting for MPU6050... (%d retries left)", retries);
        vTaskDelay(pdMS_TO_TICKS(100));
        retries--;
    }
    if (retries == 0 && mpu_init() != ESP_OK) {
        ESP_LOGE(TAG, "MPU6050 not found. Check I2C wiring and address 0x68.");
        return;
    }

    retries = 5;
    while (oled_init() != ESP_OK && retries > 0) {
        ESP_LOGW(TAG, "Waiting for OLED... (%d retries left)", retries);
        vTaskDelay(pdMS_TO_TICKS(100));
        retries--;
    }
    if (retries == 0 && oled_init() != ESP_OK) {
        ESP_LOGE(TAG, "OLED not found. Check wiring and address 0x3C.");
        return;
    }

    ESP_LOGI(TAG, "Seismic monitor started at 115200 baud.");
    motion_data_t motion = {0};
    int64_t last_sample_ms = 0;
    int64_t last_display_ms = 0;

    while (true) {
        int64_t now_ms = esp_timer_get_time() / 1000;

        if (now_ms - last_sample_ms >= SAMPLE_INTERVAL_MS) {
            last_sample_ms = now_ms;
            if (read_motion(&motion) == ESP_OK) {
                motion.vibration = average_vibration(motion.vibration);
                update_state(motion.vibration, now_ms);
                update_outputs();

                int hundredths = (int)lroundf(motion.vibration * 100.0f);
                printf("Vibration: %d.%02d m/s^2, state: %s\n",
                       hundredths / 100, hundredths % 100,
                       state_name(current_state));
            } else {
                ESP_LOGW(TAG, "Could not read MPU6050 sample.");
            }
        }

        if (now_ms - last_display_ms >= DISPLAY_INTERVAL_MS) {
            last_display_ms = now_ms;
            update_display(motion.vibration);
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}