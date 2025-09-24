#ifndef LCD_CONFIG_H
#define LCD_CONFIG_H
#ifdef WAVESHARE_S3_43
#include "esp_lcd_touch_gt911.h"
#define LCD_BCKL_ON_LEVEL 1
#define LCD_PIN_NUM_DE 5
#define LCD_PIN_NUM_VSYNC 3
#define LCD_PIN_NUM_HSYNC 46
#define LCD_PIN_NUM_CLK 7
#define LCD_PIN_NUM_D00 14
#define LCD_PIN_NUM_D01 38
#define LCD_PIN_NUM_D02 18
#define LCD_PIN_NUM_D03 17
#define LCD_PIN_NUM_D04 10
#define LCD_PIN_NUM_D05 39
#define LCD_PIN_NUM_D06 0
#define LCD_PIN_NUM_D07 45
#define LCD_PIN_NUM_D08 48
#define LCD_PIN_NUM_D09 47
#define LCD_PIN_NUM_D10 21
#define LCD_PIN_NUM_D11 1
#define LCD_PIN_NUM_D12 2
#define LCD_PIN_NUM_D13 42
#define LCD_PIN_NUM_D14 41
#define LCD_PIN_NUM_D15 40
#define LCD_PIN_NUM_BCKL -1
#define LCD_HSYNC_POLARITY 0
#define LCD_HSYNC_FRONT_PORCH 8
#define LCD_HSYNC_PULSE_WIDTH 4
#define LCD_HSYNC_BACK_PORCH 8
#define LCD_VSYNC_POLARITY 0
#define LCD_VSYNC_FRONT_PORCH 8
#define LCD_VSYNC_PULSE_WIDTH 4
#define LCD_VSYNC_BACK_PORCH 8
#define LCD_CLK_IDLE_HIGH 0
#define LCD_DE_IDLE_HIGH 0
#define LCD_BIT_DEPTH 16
//#define LCD_PANEL esp_lcd_new_panel_st7701
#define LCD_HRES 800
#define LCD_VRES 480
#define LCD_SWAP_HRES_VRES_TIMING
#define LCD_COLOR_SPACE ESP_LCD_COLOR_SPACE_BGR
#define LCD_SWAP_COLOR_BYTES true
#ifdef CONFIG_SPIRAM_MODE_QUAD
    #define LCD_PIXEL_CLOCK_HZ (6 * 1000 * 1000)
#else
    #define LCD_PIXEL_CLOCK_HZ (16 * 1000 * 1000)
#endif
#define LCD_TOUCH_I2C_HOST I2C_NUM_0
#define LCD_TOUCH_PIN_NUM_SCL           9
#define LCD_TOUCH_PIN_NUM_SDA           8

#define LCD_TOUCH_PANEL esp_lcd_touch_new_i2c_gt911
#define LCD_TOUCH_ADDRESS ESP_LCD_TOUCH_IO_I2C_GT911_ADDRESS
#define LCD_TOUCH_CMD_BITS 16
#define LCD_TOUCH_PARAM_BITS 0
#define LCD_TOUCH_DISABLE_CONTROL_PHASE
#define LCD_TOUCH_SPEED (400*1000)
#define LCD_TOUCH_HRES LCD_HRES
#define LCD_TOUCH_VRES LCD_VRES

#ifndef LEGACY_I2C
#define LCD_TOUCH_RESET \
    i2c_master_bus_handle_t bus; \
    ESP_ERROR_CHECK(i2c_master_get_bus_handle((i2c_port_num_t)0,&bus)); \
    i2c_master_dev_handle_t i2c=NULL; \
    i2c_device_config_t dev_cfg; \
    memset(&dev_cfg,0,sizeof(dev_cfg)); \
    dev_cfg.scl_speed_hz = 200*1000; \
    dev_cfg.device_address = 0x24; \
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7; \
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &dev_cfg,&i2c)); \
    uint8_t write_buf = 0x01; \
    ESP_ERROR_CHECK(i2c_master_transmit(i2c,&write_buf,1,1000 / portTICK_PERIOD_MS)); \
    ESP_ERROR_CHECK(i2c_master_bus_rm_device(i2c)); \
    dev_cfg.device_address = 0x38; \
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &dev_cfg,&i2c)); \
    write_buf = 0x2C; \
    ESP_ERROR_CHECK(i2c_master_transmit(i2c,&write_buf,1,1000 / portTICK_PERIOD_MS)); \
    esp_rom_delay_us(100 * 1000); \
    gpio_set_level((gpio_num_t)4, 0); \
    esp_rom_delay_us(100 * 1000); \
    write_buf = 0x2E; \
    ESP_ERROR_CHECK(i2c_master_transmit(i2c,&write_buf,1,1000 / portTICK_PERIOD_MS)); \
    esp_rom_delay_us(200 * 1000); \
    ESP_ERROR_CHECK(i2c_master_bus_rm_device(i2c))
#else
#define LCD_TOUCH_RESET \
    uint8_t out = 0x01;\
    ESP_ERROR_CHECK(i2c_master_write_to_device((i2c_port_t)0,0x24,&out,1,pdTICKS_TO_MS(1000)));\
    out = 0x2c;\
    ESP_ERROR_CHECK(i2c_master_write_to_device((i2c_port_t)0,0x38,&out,1,pdTICKS_TO_MS(1000)));\
    esp_rom_delay_us(100 * 1000);\
    gpio_set_level((gpio_num_t)4, 0);\
    out = 0x2E;\
    ESP_ERROR_CHECK(i2c_master_write_to_device((i2c_port_t)0,0x38,&out,1,pdTICKS_TO_MS(1000)));\
    esp_rom_delay_us(200 * 1000)
#endif
#endif

#ifdef MATOUCH_PARALLEL_43
#include "esp_lcd_touch_gt911.h"
#define LCD_PIN_NUM_DE 40
#define LCD_PIN_NUM_VSYNC 41
#define LCD_PIN_NUM_HSYNC 39
#define LCD_PIN_NUM_CLK 42
#define LCD_PIN_NUM_D00 8
#define LCD_PIN_NUM_D01 2
#define LCD_PIN_NUM_D02 46
#define LCD_PIN_NUM_D03 9
#define LCD_PIN_NUM_D04 1
#define LCD_PIN_NUM_D05 5
#define LCD_PIN_NUM_D06 6
#define LCD_PIN_NUM_D07 7
#define LCD_PIN_NUM_D08 15
#define LCD_PIN_NUM_D09 16
#define LCD_PIN_NUM_D10 4
#define LCD_PIN_NUM_D11 45
#define LCD_PIN_NUM_D12 48
#define LCD_PIN_NUM_D13 47
#define LCD_PIN_NUM_D14 21
#define LCD_PIN_NUM_D15 14
#define LCD_PIN_NUM_BCKL -1
#define LCD_HSYNC_POLARITY 0
#define LCD_HSYNC_FRONT_PORCH 8
#define LCD_HSYNC_PULSE_WIDTH 4
#define LCD_HSYNC_BACK_PORCH 8
#define LCD_VSYNC_POLARITY 0
#define LCD_VSYNC_FRONT_PORCH 8
#define LCD_VSYNC_PULSE_WIDTH 4
#define LCD_VSYNC_BACK_PORCH 8
#define LCD_CLK_IDLE_HIGH 1
#define LCD_DE_IDLE_HIGH 0
#define LCD_BOUNCE_HEIGHT 10
#define LCD_PSRAM_BUFFER
#define LCD_BIT_DEPTH 16
#define LCD_HRES 800
#define LCD_VRES 480
#define LCD_COLOR_SPACE ESP_LCD_COLOR_SPACE_BGR
#define LCD_SWAP_COLOR_BYTES 1
#ifdef CONFIG_SPIRAM_MODE_QUAD
    #define LCD_PIXEL_CLOCK_HZ (6 * 1000 * 1000)
#else
    #define LCD_PIXEL_CLOCK_HZ (16 * 1000 * 1000)
#endif
#define LCD_TOUCH_I2C_HOST I2C_NUM_0
#define LCD_TOUCH_PIN_NUM_SCL 18
#define LCD_TOUCH_PIN_NUM_SDA 17

#define LCD_TOUCH_PANEL esp_lcd_touch_new_i2c_gt911
#define LCD_TOUCH_ADDRESS ESP_LCD_TOUCH_IO_I2C_GT911_ADDRESS
#define LCD_TOUCH_CMD_BITS 16
#define LCD_TOUCH_PARAM_BITS 0
#define LCD_TOUCH_DISABLE_CONTROL_PHASE
#define LCD_TOUCH_SPEED (400*1000)
#define LCD_TOUCH_PIN_NUM_RST 38
#define LCD_TOUCH_HRES 480
#define LCD_TOUCH_VRES 272
#endif


#define LCD_BCKL_OFF_LEVEL !LCD_BCKL_ON_LEVEL
#ifndef LCD_WIDTH
#ifdef LCD_SWAP_XY
#if LCD_SWAP_XY
#define LCD_WIDTH LCD_VRES
#define LCD_HEIGHT LCD_HRES
#else
#define LCD_WIDTH LCD_HRES
#define LCD_HEIGHT LCD_VRES
#endif
#else
#define LCD_WIDTH LCD_HRES
#define LCD_HEIGHT LCD_VRES
#endif
#endif
#ifndef LCD_BIT_DEPTH
#define LCD_BIT_DEPTH 16
#endif
#ifndef LCD_X_ALIGN
#define LCD_X_ALIGN 1
#endif
#ifndef LCD_Y_ALIGN
#define LCD_Y_ALIGN 1
#endif
#ifndef LCD_DC_BIT_OFFSET
#define LCD_DC_BIT_OFFSET 0
#endif
#ifndef LCD_DIVISOR
#define LCD_DIVISOR 20
#endif
#endif // LCD_CONFIG_H