/**
 * Dreidel Game for M5StickC-PLUS (ESP-IDF C Implementation)
 * 8-bit style dreidel simulator with music and graphics
 * Press Button A to spin the dreidel!
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_random.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"
#include "driver/i2c.h"

static const char* TAG = "DREIDEL";

// Hardware pin definitions for M5StickC-PLUS
#define BUTTON_A_PIN    GPIO_NUM_37
#define BUZZER_PIN      GPIO_NUM_2

// I2C pins for AXP192 PMIC
#define I2C_SDA_PIN     GPIO_NUM_21
#define I2C_SCL_PIN     GPIO_NUM_22
#define I2C_PORT        I2C_NUM_0
#define AXP192_ADDR     0x34

// ST7789 LCD pins (M5StickC-PLUS corrected pinout)
#define LCD_MOSI        GPIO_NUM_15  // SDA
#define LCD_CLK         GPIO_NUM_13  // SCL
#define LCD_CS          GPIO_NUM_5   // CS
#define LCD_DC          GPIO_NUM_23  // DC/RS
#define LCD_RST         GPIO_NUM_18  // RST
#define LCD_BLK         GPIO_NUM_9   // Backlight - try GPIO_NUM_27 if this doesn't work

// LCD dimensions
#define LCD_WIDTH       135
#define LCD_HEIGHT      240
#define LCD_BUFFER_SIZE (LCD_WIDTH * LCD_HEIGHT * 2) // 16-bit color

// Game constants
#define BUTTON_DEBOUNCE_MS  3000
#define SPIN_DURATION_MS    8000
#define ANIMATION_DELAY_MS  300

// Colors (RGB565 format)
#define COLOR_BLACK     0x0000
#define COLOR_WHITE     0xFFFF
#define COLOR_RED       0xF800
#define COLOR_GREEN     0x07E0
#define COLOR_BLUE      0x001F
#define COLOR_YELLOW    0xFFE0
#define COLOR_CYAN      0x07FF
#define COLOR_MAGENTA   0xF81F

// SPI configuration (matching MicroPython)
#define SPI_HOST_ID     VSPI_HOST
#define SPI_CLOCK_SPEED 27000000   // 27 MHz like MicroPython

// ST7789 display offsets (M5StickC-PLUS specific)
#define COL_OFFSET      52
#define ROW_OFFSET      40

// PWM/LEDC configuration
#define LEDC_TIMER          LEDC_TIMER_0
#define LEDC_MODE           LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL        LEDC_CHANNEL_0
#define LEDC_DUTY_RES       LEDC_TIMER_13_BIT
#define LEDC_FREQUENCY      5000

// Note frequencies (4th octave)
typedef struct {
    char note;
    int frequency;
} note_freq_t;

static const note_freq_t note_frequencies[] = {
    {'C', 261}, {'D', 294}, {'E', 330}, {'F', 349},
    {'G', 392}, {'A', 440}, {'B', 494}, {'R', 0}  // R = REST
};

// Dreidel symbols and meanings
typedef struct {
    char symbol;
    const char* name;
    const char* meaning;
    const char* bmp_filename;
} dreidel_symbol_t;

static const dreidel_symbol_t dreidel_symbols[] = {
    {'N', "Nun",   "Nothing",  "nun.bmp"},
    {'G', "Gimel", "Get All",  "gimel.bmp"},
    {'H', "Hey",   "Half",     "hey.bmp"},
    {'S', "Shin",  "Put In",   "shin.bmp"}
};

// External embedded BMP data declarations
extern const uint8_t base_bmp_start[] asm("_binary_base_bmp_start");
extern const uint8_t base_bmp_end[]   asm("_binary_base_bmp_end");
extern const uint8_t gimel_bmp_start[] asm("_binary_gimel_bmp_start");
extern const uint8_t gimel_bmp_end[]   asm("_binary_gimel_bmp_end");
extern const uint8_t hey_bmp_start[] asm("_binary_hey_bmp_start");
extern const uint8_t hey_bmp_end[]   asm("_binary_hey_bmp_end");
extern const uint8_t intro_bmp_start[] asm("_binary_intro_bmp_start");
extern const uint8_t intro_bmp_end[]   asm("_binary_intro_bmp_end");
extern const uint8_t left_bmp_start[] asm("_binary_left_bmp_start");
extern const uint8_t left_bmp_end[]   asm("_binary_left_bmp_end");
extern const uint8_t nun_bmp_start[] asm("_binary_nun_bmp_start");
extern const uint8_t nun_bmp_end[]   asm("_binary_nun_bmp_end");
extern const uint8_t prompt_bmp_start[] asm("_binary_prompt_bmp_start");
extern const uint8_t prompt_bmp_end[]   asm("_binary_prompt_bmp_end");
extern const uint8_t right_bmp_start[] asm("_binary_right_bmp_start");
extern const uint8_t right_bmp_end[]   asm("_binary_right_bmp_end");
extern const uint8_t shin_bmp_start[] asm("_binary_shin_bmp_start");
extern const uint8_t shin_bmp_end[]   asm("_binary_shin_bmp_end");

// Game state
typedef enum {
    GAME_STATE_WELCOME,
    GAME_STATE_SPINNING,
    GAME_STATE_RESULT,
    GAME_STATE_WAITING
} game_state_t;

typedef struct {
    game_state_t state;
    uint32_t last_button_time;
    int current_result_index;
    bool spinning;
    SemaphoreHandle_t music_mutex;
    spi_device_handle_t spi;
    uint16_t *lcd_buffer;
} game_context_t;

static game_context_t game_ctx = {0};

// Function prototypes
static void init_hardware(void);
static bool init_i2c(void);
static bool init_axp192_pmic(void);
static void init_gpio(void);
static void init_pwm(void);
static bool init_lcd(void);
static void lcd_send_command(uint8_t cmd);
static void lcd_send_data(uint8_t *data, int len);
static void lcd_init_sequence(void);
static void lcd_load_bmp(const uint8_t* bmp_data, size_t bmp_size, int x, int y);
static void lcd_update_display(void);
static bool detect_button_press(void);
static void play_note(char note, int duration_ms);
static void play_dreidel_song(void);
static void spin_dreidel(void);
static void show_welcome_screen(void);
static void show_spinning_animation(void);
static void show_result(int symbol_index);
static void music_task(void *pvParameters);
static void animation_task(void *pvParameters);
static int get_note_frequency(char note);

/**
 * Initialize all hardware components
 */
static void init_hardware(void) {
    ESP_LOGI(TAG, "Initializing hardware...");

    // ESP hardware RNG is automatically initialized

    // Create mutex for music synchronization
    game_ctx.music_mutex = xSemaphoreCreateMutex();

    // Allocate LCD buffer
    game_ctx.lcd_buffer = heap_caps_malloc(LCD_BUFFER_SIZE, MALLOC_CAP_DMA);
    if (!game_ctx.lcd_buffer) {
        ESP_LOGE(TAG, "Failed to allocate LCD buffer");
        return;
    }

    // CRITICAL: Initialize I2C and PMIC first (powers LCD and buzzer)
    if (!init_i2c()) {
        ESP_LOGE(TAG, "Failed to initialize I2C");
        return;
    }

    if (!init_axp192_pmic()) {
        ESP_LOGE(TAG, "Failed to initialize AXP192 PMIC");
        return;
    }

    init_gpio();
    init_pwm();

    if (!init_lcd()) {
        ESP_LOGE(TAG, "Failed to initialize LCD - continuing without display");
        // Continue execution without LCD for debugging
    }

    ESP_LOGI(TAG, "Hardware initialization complete!");
}

/**
 * Initialize I2C for AXP192 PMIC communication
 */
static bool init_i2c(void) {
    ESP_LOGI(TAG, "Initializing I2C for AXP192 PMIC...");

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_SCL_PIN,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000,  // 400 kHz
    };

    esp_err_t ret = i2c_param_config(I2C_PORT, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C config failed: %s", esp_err_to_name(ret));
        return false;
    }

    ret = i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(ret));
        return false;
    }

    ESP_LOGI(TAG, "I2C initialized successfully");
    return true;
}

/**
 * Initialize AXP192 PMIC - CRITICAL for LCD and buzzer power
 */
static bool init_axp192_pmic(void) {
    ESP_LOGI(TAG, "Initializing AXP192 PMIC...");

    // Enable LDO2&3 and boost (EXTEN) - powers LCD and backlight
    uint8_t data = 0x4D;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AXP192_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x12, true);  // Register 0x12
    i2c_master_write_byte(cmd, data, true);  // Value 0x4D
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "AXP192 PMIC write failed: %s", esp_err_to_name(ret));
        return false;
    }

    // Wait for power rails to stabilize
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI(TAG, "AXP192 PMIC initialized - LCD and buzzer now powered!");
    return true;
}

/**
 * Initialize GPIO pins
 */
static void init_gpio(void) {
    // Configure button A (input-only pin, external pull-up)
    gpio_config_t button_config = {
        .pin_bit_mask = (1ULL << BUTTON_A_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,  // GPIO37 is input-only, no internal pull-up
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&button_config);


    // Configure LCD control pins (DC and RST only - backlight is via PMIC)
    gpio_config_t lcd_gpio_config = {
        .pin_bit_mask = (1ULL << LCD_DC) | (1ULL << LCD_RST),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&lcd_gpio_config);

    // Initialize LCD control pins
    gpio_set_level(LCD_RST, 1);
    gpio_set_level(LCD_DC, 0);

    ESP_LOGI(TAG, "LCD control pins initialized (backlight powered by PMIC)");

    ESP_LOGI(TAG, "GPIO initialized");
}

/**
 * Initialize PWM for buzzer
 */
static void init_pwm(void) {
    ESP_LOGI(TAG, "Initializing PWM for buzzer on pin %d", BUZZER_PIN);

    // Prepare and set configuration of timers
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz = LEDC_FREQUENCY,
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    esp_err_t ret = ledc_timer_config(&ledc_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure LEDC timer: %s", esp_err_to_name(ret));
        return;
    }

    // Prepare and set configuration of channel
    ledc_channel_config_t ledc_channel = {
        .channel    = LEDC_CHANNEL,
        .duty       = 0,
        .gpio_num   = BUZZER_PIN,
        .speed_mode = LEDC_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_TIMER
    };
    ret = ledc_channel_config(&ledc_channel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure LEDC channel: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "PWM initialized successfully");
}

/**
 * Initialize LCD display and SPI
 */
static bool init_lcd(void) {
    esp_err_t ret;

    // SPI bus configuration
    spi_bus_config_t buscfg = {
        .miso_io_num = -1,
        .mosi_io_num = LCD_MOSI,
        .sclk_io_num = LCD_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_BUFFER_SIZE,
    };

    // SPI device configuration (mode 0: polarity=0, phase=0 like MicroPython)
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = SPI_CLOCK_SPEED,
        .mode = 0,                     // SPI mode 0 (CPOL=0, CPHA=0)
        .spics_io_num = LCD_CS,
        .queue_size = 7,
        .flags = 0,
    };

    // Initialize SPI bus
    ESP_LOGI(TAG, "Initializing SPI bus on host %d", SPI_HOST_ID);
    ret = spi_bus_initialize(SPI_HOST_ID, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        return false;
    }

    // Attach device to SPI bus
    ESP_LOGI(TAG, "Adding SPI device with clock %d Hz", SPI_CLOCK_SPEED);
    ret = spi_bus_add_device(SPI_HOST_ID, &devcfg, &game_ctx.spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add SPI device: %s", esp_err_to_name(ret));
        return false;
    }

    // Reset LCD
    gpio_set_level(LCD_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(LCD_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(100));

    // Initialize LCD with command sequence
    lcd_init_sequence();

    ESP_LOGI(TAG, "LCD initialized successfully");
    return true;
}

/**
 * Send command to LCD
 */
static void lcd_send_command(uint8_t cmd) {
    if (!game_ctx.spi) {
        return; // SPI not initialized, skip LCD operations
    }

    esp_err_t ret;
    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &cmd,
    };

    gpio_set_level(LCD_DC, 0); // Command mode
    ret = spi_device_polling_transmit(game_ctx.spi, &t);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI command failed: %s", esp_err_to_name(ret));
    }
}

/**
 * Send data to LCD
 */
static void lcd_send_data(uint8_t *data, int len) {
    if (!game_ctx.spi || len == 0) return;

    esp_err_t ret;
    spi_transaction_t t = {
        .length = len * 8,
        .tx_buffer = data,
    };

    gpio_set_level(LCD_DC, 1); // Data mode
    ret = spi_device_polling_transmit(game_ctx.spi, &t);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI data failed: %s", esp_err_to_name(ret));
    }
}

/**
 * LCD initialization sequence for ST7789 (M5StickC-PLUS specific)
 */
static void lcd_init_sequence(void) {
    ESP_LOGI(TAG, "Starting ST7789 initialization sequence...");

    // Software reset
    lcd_send_command(0x01);  // SWRESET
    vTaskDelay(pdMS_TO_TICKS(150));

    // Sleep out
    lcd_send_command(0x11);  // SLPOUT
    vTaskDelay(pdMS_TO_TICKS(120));

    // Color mode - RGB565 (16-bit)
    lcd_send_command(0x3A);  // COLMOD
    uint8_t colmod_data = 0x55;  // RGB565
    lcd_send_data(&colmod_data, 1);

    // Memory access control - RGB order for correct colors
    lcd_send_command(0x36);  // MADCTL
    uint8_t madctl_data = 0x00;  // Use RGB order (bit 3 = 0)
    lcd_send_data(&madctl_data, 1);
    ESP_LOGI(TAG, "MADCTL set to 0x%02X (RGB order)", madctl_data);

    // Display inversion ON (required for M5StickC-PLUS)
    lcd_send_command(0x21);  // INVON

    // Normal display mode
    lcd_send_command(0x13);  // NORON
    vTaskDelay(pdMS_TO_TICKS(10));

    // Set drawing window with M5StickC-PLUS offsets
    // Column address set: x_start=52, x_end=52+135-1=186
    lcd_send_command(0x2A);  // CASET
    uint8_t caset_data[] = {
        0x00, COL_OFFSET,                    // x_start = 52
        0x00, COL_OFFSET + LCD_WIDTH - 1     // x_end = 186
    };
    lcd_send_data(caset_data, 4);

    // Row address set: y_start=40, y_end=40+240-1=279
    lcd_send_command(0x2B);  // RASET
    uint16_t y_end = ROW_OFFSET + LCD_HEIGHT - 1;  // 279
    uint8_t raset_data[] = {
        0x00, ROW_OFFSET,                    // y_start = 40
        (y_end >> 8) & 0xFF, y_end & 0xFF    // y_end = 279 (high byte, low byte)
    };
    lcd_send_data(raset_data, 4);

    // Display on
    lcd_send_command(0x29);  // DISPON
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI(TAG, "ST7789 initialization complete");
}


/**
 * BMP file structure definitions
 */
#pragma pack(push, 1)
typedef struct {
    uint16_t type;
    uint32_t size;
    uint16_t reserved1;
    uint16_t reserved2;
    uint32_t offset;
} bmp_header_t;

typedef struct {
    uint32_t size;
    int32_t width;
    int32_t height;
    uint16_t planes;
    uint16_t bits_per_pixel;
    uint32_t compression;
    uint32_t image_size;
    int32_t x_pixels_per_meter;
    int32_t y_pixels_per_meter;
    uint32_t colors_used;
    uint32_t colors_important;
} bmp_info_t;
#pragma pack(pop)

/**
 * Load BMP image into LCD buffer at specified position
 */
static void lcd_load_bmp(const uint8_t* bmp_data, size_t bmp_size, int x, int y) {
    if (!bmp_data || bmp_size < sizeof(bmp_header_t) + sizeof(bmp_info_t)) {
        ESP_LOGE(TAG, "Invalid BMP data");
        return;
    }

    bmp_header_t* header = (bmp_header_t*)bmp_data;
    bmp_info_t* info = (bmp_info_t*)(bmp_data + sizeof(bmp_header_t));

    // Verify BMP signature
    if (header->type != 0x4D42) { // "BM"
        ESP_LOGE(TAG, "Invalid BMP signature");
        return;
    }

    // Handle different BMP formats
    if (info->bits_per_pixel != 16 && info->bits_per_pixel != 8) {
        ESP_LOGE(TAG, "Only 8-bit and 16-bit BMPs supported, got %d bits", info->bits_per_pixel);
        return;
    }

    int bmp_width = info->width;
    int bmp_height = abs(info->height);
    bool top_down = info->height < 0;

    ESP_LOGI(TAG, "Loading BMP: %dx%d (%d-bit) at (%d,%d)", bmp_width, bmp_height, info->bits_per_pixel, x, y);

    if (info->bits_per_pixel == 16) {
        // 16-bit RGB565 BMP
        uint16_t* pixel_data = (uint16_t*)(bmp_data + header->offset);

        for (int row = 0; row < bmp_height; row++) {
            for (int col = 0; col < bmp_width; col++) {
                int src_row = top_down ? row : (bmp_height - 1 - row);
                int src_index = src_row * bmp_width + col;

                int dst_x = x + col;
                int dst_y = y + row;

                if (dst_x >= 0 && dst_x < LCD_WIDTH && dst_y >= 0 && dst_y < LCD_HEIGHT) {
                    // For 16-bit BMPs, apply byte swap like MicroPython
                    uint16_t color = pixel_data[src_index];
                    uint16_t swapped = ((color & 0xFF) << 8) | ((color >> 8) & 0xFF);
                    game_ctx.lcd_buffer[dst_y * LCD_WIDTH + dst_x] = swapped;
                }
            }
        }
    } else if (info->bits_per_pixel == 8) {
        // 8-bit indexed BMP with color palette
        uint32_t* palette = (uint32_t*)(bmp_data + sizeof(bmp_header_t) + sizeof(bmp_info_t));
        uint8_t* pixel_data = (uint8_t*)(bmp_data + header->offset);

        // Calculate row padding (BMP rows are padded to 4-byte boundaries)
        int row_size = ((bmp_width + 3) / 4) * 4;

        for (int row = 0; row < bmp_height; row++) {
            for (int col = 0; col < bmp_width; col++) {
                int src_row = top_down ? row : (bmp_height - 1 - row);
                int src_index = src_row * row_size + col;

                int dst_x = x + col;
                int dst_y = y + row;

                if (dst_x >= 0 && dst_x < LCD_WIDTH && dst_y >= 0 && dst_y < LCD_HEIGHT) {
                    uint8_t palette_index = pixel_data[src_index];
                    uint32_t bgra = palette[palette_index];

                    // Convert BGRA to RGB565 - swap R and B for correct colors
                    uint8_t r = (bgra >> 16) & 0xFF;  // Use B as R
                    uint8_t g = (bgra >> 8) & 0xFF;   // Keep G as G
                    uint8_t b = bgra & 0xFF;          // Use R as B

                    // Create RGB565 and byte swap like MicroPython does
                    uint16_t rgb565 = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
                    uint16_t swapped = ((rgb565 & 0xFF) << 8) | ((rgb565 >> 8) & 0xFF);
                    game_ctx.lcd_buffer[dst_y * LCD_WIDTH + dst_x] = swapped;
                }
            }
        }
    }
}


/**
 * Update display with current buffer
 */
static void lcd_update_display(void) {
    if (!game_ctx.spi || !game_ctx.lcd_buffer) {
        return; // LCD not initialized, skip display update
    }

    // Set column address with M5StickC-PLUS offsets
    lcd_send_command(0x2A);
    uint8_t caset_data[] = {
        0x00, COL_OFFSET,                    // x_start = 52
        0x00, COL_OFFSET + LCD_WIDTH - 1     // x_end = 186
    };
    lcd_send_data(caset_data, 4);

    // Set row address with M5StickC-PLUS offsets
    lcd_send_command(0x2B);
    uint16_t y_end = ROW_OFFSET + LCD_HEIGHT - 1;  // 279
    uint8_t raset_data[] = {
        0x00, ROW_OFFSET,                    // y_start = 40
        (y_end >> 8) & 0xFF, y_end & 0xFF    // y_end = 279 (high byte, low byte)
    };
    lcd_send_data(raset_data, 4);

    // Memory write
    lcd_send_command(0x2C);

    // Send framebuffer
    gpio_set_level(LCD_DC, 1); // Data mode

    // Send data in chunks to avoid SPI transaction size limits
    const int chunk_size = 4092; // Must be even number for 16-bit pixels
    uint8_t *buffer = (uint8_t*)game_ctx.lcd_buffer;

    for (int i = 0; i < LCD_BUFFER_SIZE; i += chunk_size) {
        int remaining = LCD_BUFFER_SIZE - i;
        int send_size = (remaining < chunk_size) ? remaining : chunk_size;

        spi_transaction_t t = {
            .length = send_size * 8,
            .tx_buffer = buffer + i,
        };

        esp_err_t ret = spi_device_polling_transmit(game_ctx.spi, &t);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "SPI display update failed: %s", esp_err_to_name(ret));
            return;
        }
    }
}

/**
 * Detect button press with debouncing
 */
static bool detect_button_press(void) {
    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;

    // Prevent too frequent button presses
    if (current_time - game_ctx.last_button_time < BUTTON_DEBOUNCE_MS) {
        return false;
    }

    // Check button A (active low)
    if (gpio_get_level(BUTTON_A_PIN) == 0) {
        game_ctx.last_button_time = current_time;
        vTaskDelay(pdMS_TO_TICKS(200)); // Debounce delay
        return true;
    }

    return false;
}

/**
 * Get frequency for a given note
 */
static int get_note_frequency(char note) {
    for (int i = 0; i < sizeof(note_frequencies) / sizeof(note_freq_t); i++) {
        if (note_frequencies[i].note == note) {
            return note_frequencies[i].frequency;
        }
    }
    return 0; // REST or unknown note
}

/**
 * Play a single note
 */
static void play_note(char note, int duration_ms) {
    int freq = get_note_frequency(note);

    if (freq > 0) {
        // Set frequency and start PWM
        ledc_set_freq(LEDC_MODE, LEDC_TIMER, freq);
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 1024); // ~12% duty cycle (quieter volume)
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
    } else {
        // Silence (REST)
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
    }

    vTaskDelay(pdMS_TO_TICKS(duration_ms));

    // Brief pause between notes
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
    vTaskDelay(pdMS_TO_TICKS(50));
}

/**
 * Play the dreidel song: "GGEGEGE EGGFED DFDFDFD DGFEDC"
 */
static void play_dreidel_song(void) {
    ESP_LOGI(TAG, "Playing dreidel song...");

    const char* phrases[] = {
        "GGEGEGE",   // First phrase
        "EGGFED",    // Second phrase
        "DFDFDFD",   // Third phrase
        "DGFEDC"     // Final phrase
    };

    int num_phrases = sizeof(phrases) / sizeof(phrases[0]);

    for (int i = 0; i < num_phrases; i++) {
        const char* phrase = phrases[i];
        int phrase_len = strlen(phrase);

        ESP_LOGI(TAG, "Playing phrase %d: %s", i + 1, phrase);

        for (int j = 0; j < phrase_len; j++) {
            play_note(phrase[j], 250); // 250ms per note
        }

        // Brief pause between phrases
        if (i < num_phrases - 1) {
            vTaskDelay(pdMS_TO_TICKS(200));
        }
    }

    ESP_LOGI(TAG, "Dreidel song complete!");
}

/**
 * Music task - plays music in background
 */
static void music_task(void *pvParameters) {
    if (xSemaphoreTake(game_ctx.music_mutex, portMAX_DELAY)) {
        play_dreidel_song();
        xSemaphoreGive(game_ctx.music_mutex);
    }
    vTaskDelete(NULL);
}

/**
 * Animation task - shows spinning animation with BMP switching
 */
static void animation_task(void *pvParameters) {
    uint32_t start_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    bool left_turn = false; // Start with right since we already showed left

    while ((xTaskGetTickCount() * portTICK_PERIOD_MS - start_time) < SPIN_DURATION_MS) {
        left_turn = !left_turn; // Toggle for next frame

        // Fast switching between left and right BMPs
        if (left_turn) {
            lcd_load_bmp(left_bmp_start, left_bmp_end - left_bmp_start, 0, 0);
        } else {
            lcd_load_bmp(right_bmp_start, right_bmp_end - right_bmp_start, 0, 0);
        }

        // Keep the base BMP at the bottom
        lcd_load_bmp(base_bmp_start, base_bmp_end - base_bmp_start, 0, 164);

        // Update display
        lcd_update_display();


        vTaskDelay(pdMS_TO_TICKS(80)); // Fast animation synchronized with music (80ms like MicroPython)
    }

    ESP_LOGI(TAG, "Animation complete!");

    vTaskDelete(NULL);
}

/**
 * Spin the dreidel with music and animation
 */
static void spin_dreidel(void) {
    ESP_LOGI(TAG, "Spinning dreidel...");
    game_ctx.spinning = true;

    // Generate random result (0-3 for 4 symbols)
    uint32_t random_val = esp_random();
    game_ctx.current_result_index = random_val % 4;
    ESP_LOGI(TAG, "Random value: %u, Selected index: %d (%s)", random_val,
             game_ctx.current_result_index, dreidel_symbols[game_ctx.current_result_index].name);

    // Create music and animation tasks to run concurrently
    xTaskCreate(music_task, "music_task", 4096, NULL, 5, NULL);
    xTaskCreate(animation_task, "animation_task", 2048, NULL, 5, NULL);

    // Wait for spinning duration
    vTaskDelay(pdMS_TO_TICKS(SPIN_DURATION_MS));

    game_ctx.spinning = false;
    ESP_LOGI(TAG, "Dreidel spin complete!");
}

/**
 * Show welcome screen
 */
static void show_welcome_screen(void) {
    ESP_LOGI(TAG, "Showing welcome screen on LCD");

    // Show intro image at top (135x164)
    ESP_LOGI(TAG, "Loading intro BMP (%d bytes)", intro_bmp_end - intro_bmp_start);
    lcd_load_bmp(intro_bmp_start, intro_bmp_end - intro_bmp_start, 0, 0);

    // Show initial prompt at bottom
    ESP_LOGI(TAG, "Loading prompt BMP (%d bytes)", prompt_bmp_end - prompt_bmp_start);
    lcd_load_bmp(prompt_bmp_start, prompt_bmp_end - prompt_bmp_start, 0, 164);

    // Update display
    ESP_LOGI(TAG, "Updating LCD display");
    lcd_update_display();

    // Brief display delay to show welcome screen
    vTaskDelay(pdMS_TO_TICKS(500));
}

/**
 * Show spinning animation on LCD (initial frame)
 */
static void show_spinning_animation(void) {
    ESP_LOGI(TAG, "Starting spinning animation on LCD");

    // Start spinning animation immediately (first frame - left)
    lcd_load_bmp(left_bmp_start, left_bmp_end - left_bmp_start, 0, 0);
    lcd_load_bmp(base_bmp_start, base_bmp_end - base_bmp_start, 0, 164);

    // Update display
    lcd_update_display();
}

/**
 * Show the dreidel result using BMP graphics
 */
static void show_result(int symbol_index) {
    const dreidel_symbol_t* symbol = &dreidel_symbols[symbol_index];

    ESP_LOGI(TAG, "Showing result: %s - %s", symbol->name, symbol->meaning);

    // Load the result image (contains symbol, name, and meaning)
    if (symbol_index == 0) { // Nun
        lcd_load_bmp(nun_bmp_start, nun_bmp_end - nun_bmp_start, 0, 0);
    } else if (symbol_index == 1) { // Gimel
        lcd_load_bmp(gimel_bmp_start, gimel_bmp_end - gimel_bmp_start, 0, 0);
    } else if (symbol_index == 2) { // Hey
        lcd_load_bmp(hey_bmp_start, hey_bmp_end - hey_bmp_start, 0, 0);
    } else if (symbol_index == 3) { // Shin
        lcd_load_bmp(shin_bmp_start, shin_bmp_end - shin_bmp_start, 0, 0);
    }

    // Load prompt image at bottom (always show prompt on results)
    lcd_load_bmp(prompt_bmp_start, prompt_bmp_end - prompt_bmp_start, 0, 164);

    // Update display
    lcd_update_display();
}

/**
 * Main application entry point
 */
void app_main(void) {
    ESP_LOGI(TAG, "Starting Dreidel Game!");

    // Initialize hardware
    init_hardware();

    // Initialize game state
    game_ctx.state = GAME_STATE_WELCOME;
    game_ctx.last_button_time = 0;
    game_ctx.current_result_index = 0;
    game_ctx.spinning = false;

    // Show welcome screen
    show_welcome_screen();
    game_ctx.state = GAME_STATE_WAITING;

    // Main game loop
    while (true) {
        switch (game_ctx.state) {
            case GAME_STATE_WAITING:
                if (detect_button_press()) {
                    ESP_LOGI(TAG, "Button pressed! Starting spin...");
                    game_ctx.state = GAME_STATE_SPINNING;
                    show_spinning_animation();
                    spin_dreidel();
                    game_ctx.state = GAME_STATE_RESULT;
                }
                break;

            case GAME_STATE_RESULT:
                show_result(game_ctx.current_result_index);
                game_ctx.state = GAME_STATE_WAITING;
                break;

            default:
                break;
        }

        vTaskDelay(pdMS_TO_TICKS(50)); // Small delay for responsiveness
    }
}
