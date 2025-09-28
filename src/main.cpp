#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "esp_idf_version.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_rgb.h"
#include "esp_lcd_panel_vendor.h"
#include "driver/gpio.h"
#ifndef LEGACY_I2C
#include "driver/i2c_master.h"
#else
#include "driver/i2c.h"
#endif
#include "esp_lcd_touch.h"
#include "esp_spiffs.h"
#include "esp_vfs_fat.h"
#include "lcd_config.h"
#include "chess.h"
#include "gfx.hpp"
#include "uix.hpp"
#include "vlabel.hpp"
#include "vicon.hpp"
#define POWER_IMPLEMENTATION
#include "assets/power.hpp"
#define PIECE_IMPLEMENTATION
#include "assets/piece.hpp"
#define OPENSANS_REGULAR_IMPLEMENTATION
#include "assets/OpenSans_Regular.h"  // our font

using namespace gfx;
using namespace uix;

// fonts load from streams, so wrap our array in one
static const_buffer_stream font_stream(OpenSans_Regular,sizeof(OpenSans_Regular));

// manages the screens
static uix::display lcd;

static void i2c_initialize()
{
#ifndef LEGACY_I2C
    i2c_master_bus_config_t i2c_mst_config;
    memset(&i2c_mst_config,0,sizeof(i2c_mst_config));
    i2c_mst_config.clk_source = I2C_CLK_SRC_DEFAULT;
    i2c_mst_config.i2c_port = (i2c_port_num_t)LCD_TOUCH_I2C_HOST;
    i2c_mst_config.sda_io_num = (gpio_num_t)LCD_TOUCH_PIN_NUM_SDA;
    i2c_mst_config.scl_io_num = (gpio_num_t)LCD_TOUCH_PIN_NUM_SCL;
    i2c_mst_config.glitch_ignore_cnt = 7;
    i2c_mst_config.flags.enable_internal_pullup = 1;
    i2c_master_bus_handle_t bus;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus));
#else
    i2c_config_t i2c_config;
    memset(&i2c_config,0,sizeof(i2c_config));
    i2c_config.master.clk_speed = LCD_TOUCH_SPEED;
    i2c_config.mode = LCD_TOUCH_I2C_HOST;
    i2c_config.sda_io_num = LCD_TOUCH_PIN_NUM_SDA;
    i2c_config.scl_io_num = LCD_TOUCH_PIN_NUM_SCL;
    ESP_ERROR_CHECK(i2c_driver_install((i2c_port_t)LCD_TOUCH_I2C_HOST,I2C_MODE_MASTER,0,0,0));
    ESP_ERROR_CHECK(i2c_param_config((i2c_port_t)LCD_TOUCH_I2C_HOST,&i2c_config));    
#endif
}
static void spiffs_initialize(void) {
    esp_vfs_spiffs_conf_t conf;
    memset(&conf, 0, sizeof(conf));
    conf.base_path = "/spiffs";
    conf.partition_label = NULL;
    conf.max_files = 5;
    conf.format_if_mount_failed = 1;
    ESP_ERROR_CHECK(esp_vfs_spiffs_register(&conf));
}
// UIX calls this to send bitmaps to the display
static void uix_flush(const gfx::rect16& bounds, const void *bitmap, void *state)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t)state;
    
    // pass the draw buffer to the driver
    esp_lcd_panel_draw_bitmap(panel_handle, bounds.x1, bounds.y1, bounds.x2 + 1, bounds.y2 + 1, bitmap);
}
// LCD Panel API calls this
bool lcd_flush_complete(esp_lcd_panel_handle_t panel, const esp_lcd_rgb_panel_event_data_t *edata, void *user_ctx) {
    // let the display know the flush has finished
    lcd.flush_complete();
    return true;
}
static volatile bool lcd_is_vsync = 0;
// LCD Panel API calls this
bool lcd_vsync(esp_lcd_panel_handle_t panel, const esp_lcd_rgb_panel_event_data_t *edata, void *user_ctx) {
    lcd_is_vsync=1;
    return true;
}
static void lcd_initialize() {
    #if LCD_PIN_NUM_BCKL >= 0
    gpio_config_t bk_gpio_config;
    memset(&bk_gpio_config,0,sizeof(gpio_config_t));
    bk_gpio_config.mode = GPIO_MODE_OUTPUT;
    bk_gpio_config.pin_bit_mask = 1ULL << LCD_PIN_NUM_BCKL;
    ESP_ERROR_CHECK(gpio_config((gpio_num_t)LCD_PIN_NUM_BCKL,&bk_gpio_config));
    gpio_set_level((gpio_num_t)LCD_PIN_NUM_BCKL, LCD_BCKL_OFF_LEVEL);
    #endif
    
    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_rgb_panel_config_t panel_config;
    memset(&panel_config,0,sizeof(esp_lcd_rgb_panel_config_t));
     
        panel_config.data_width = 16; // RGB565 in parallel mode, thus 16bit in width
        //panel_config.dma_burst_size = 64;
        panel_config.num_fbs = 1,
        panel_config.clk_src = LCD_CLK_SRC_DEFAULT,
        panel_config.disp_gpio_num = -1,
        panel_config.pclk_gpio_num = LCD_PIN_NUM_CLK,
        panel_config.vsync_gpio_num = LCD_PIN_NUM_VSYNC,
        panel_config.hsync_gpio_num = LCD_PIN_NUM_HSYNC,
        panel_config.de_gpio_num = LCD_PIN_NUM_DE,
#if !defined(LCD_SWAP_COLOR_BYTES) || LCD_SWAP_COLOR_BYTES == false
        panel_config.data_gpio_nums[0]=LCD_PIN_NUM_D00;
        panel_config.data_gpio_nums[1]=LCD_PIN_NUM_D01;
        panel_config.data_gpio_nums[2]=LCD_PIN_NUM_D02;
        panel_config.data_gpio_nums[3]=LCD_PIN_NUM_D03;
        panel_config.data_gpio_nums[4]=LCD_PIN_NUM_D04;
        panel_config.data_gpio_nums[5]=LCD_PIN_NUM_D05;
        panel_config.data_gpio_nums[6]=LCD_PIN_NUM_D06;
        panel_config.data_gpio_nums[7]=LCD_PIN_NUM_D07;

        panel_config.data_gpio_nums[8]=LCD_PIN_NUM_D08;
        panel_config.data_gpio_nums[9]=LCD_PIN_NUM_D09;
        panel_config.data_gpio_nums[10]=LCD_PIN_NUM_D10;
        panel_config.data_gpio_nums[11]=LCD_PIN_NUM_D11;
        panel_config.data_gpio_nums[12]=LCD_PIN_NUM_D12;
        panel_config.data_gpio_nums[13]=LCD_PIN_NUM_D13;
        panel_config.data_gpio_nums[14]=LCD_PIN_NUM_D14;
        panel_config.data_gpio_nums[15]=LCD_PIN_NUM_D15;
#else
        panel_config.data_gpio_nums[0]=LCD_PIN_NUM_D08;
        panel_config.data_gpio_nums[1]=LCD_PIN_NUM_D09;
        panel_config.data_gpio_nums[2]=LCD_PIN_NUM_D10;
        panel_config.data_gpio_nums[3]=LCD_PIN_NUM_D11;
        panel_config.data_gpio_nums[4]=LCD_PIN_NUM_D12;
        panel_config.data_gpio_nums[5]=LCD_PIN_NUM_D13;
        panel_config.data_gpio_nums[6]=LCD_PIN_NUM_D14;
        panel_config.data_gpio_nums[7]=LCD_PIN_NUM_D15;

        panel_config.data_gpio_nums[8]=LCD_PIN_NUM_D00;
        panel_config.data_gpio_nums[9]=LCD_PIN_NUM_D01;
        panel_config.data_gpio_nums[10]=LCD_PIN_NUM_D02;
        panel_config.data_gpio_nums[11]=LCD_PIN_NUM_D03;
        panel_config.data_gpio_nums[12]=LCD_PIN_NUM_D04;
        panel_config.data_gpio_nums[13]=LCD_PIN_NUM_D05;
        panel_config.data_gpio_nums[14]=LCD_PIN_NUM_D06;
        panel_config.data_gpio_nums[15]=LCD_PIN_NUM_D07;
#endif
        memset(&panel_config.timings,0,sizeof(esp_lcd_rgb_timing_t));
        
        panel_config.timings.pclk_hz = LCD_PIXEL_CLOCK_HZ;
        panel_config.timings.h_res = LCD_HRES;
        panel_config.timings.v_res = LCD_VRES;
        panel_config.timings.hsync_back_porch = LCD_HSYNC_BACK_PORCH;
        panel_config.timings.hsync_front_porch = LCD_HSYNC_FRONT_PORCH;
        panel_config.timings.hsync_pulse_width = LCD_HSYNC_PULSE_WIDTH;
        panel_config.timings.vsync_back_porch = LCD_VSYNC_BACK_PORCH;
        panel_config.timings.vsync_front_porch = LCD_VSYNC_FRONT_PORCH;
        panel_config.timings.vsync_pulse_width = LCD_VSYNC_PULSE_WIDTH;
        panel_config.timings.flags.pclk_active_neg = true;
        panel_config.timings.flags.hsync_idle_low = false;
        panel_config.timings.flags.pclk_idle_high = LCD_CLK_IDLE_HIGH;
        panel_config.timings.flags.de_idle_high = LCD_DE_IDLE_HIGH;
        panel_config.timings.flags.vsync_idle_low = false;
        panel_config.flags.bb_invalidate_cache = true;
        panel_config.flags.disp_active_low = false;
        panel_config.flags.double_fb = false;
        panel_config.flags.no_fb = false;
        panel_config.flags.refresh_on_demand = false;
        panel_config.flags.fb_in_psram = true; // allocate frame buffer in PSRAM
        //panel_config.sram_trans_align = 4;
        //panel_config.psram_trans_align = 64;
        panel_config.num_fbs = 2;
        panel_config.bounce_buffer_size_px = LCD_HRES*(LCD_VRES/LCD_DIVISOR);
        ESP_ERROR_CHECK(esp_lcd_new_rgb_panel(&panel_config, &panel_handle));
        esp_lcd_rgb_panel_event_callbacks_t cbs;
        memset(&cbs,0,sizeof(cbs));
        cbs.on_color_trans_done = lcd_flush_complete;
        cbs.on_vsync = lcd_vsync;
        esp_lcd_rgb_panel_register_event_callbacks(panel_handle,&cbs,NULL);
        ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
        ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
        #if LCD_PIN_NUM_BCKL >= 0
        gpio_set_level((gpio_num_t)LCD_PIN_NUM_BCKL, LCD_BCKL_ON_LEVEL);
        #endif
        void *buf1 = NULL, *buf2=NULL;
        // it's recommended to allocate the draw buffer from internal memory, for better performance
        size_t draw_buffer_sz = LCD_HRES * (LCD_VRES/LCD_DIVISOR) * sizeof(uint16_t);
        buf1 = heap_caps_malloc(draw_buffer_sz, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        assert(buf1);
        buf2 = heap_caps_malloc(draw_buffer_sz, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        assert(buf2);
        lcd.buffer_size(draw_buffer_sz);
        lcd.buffer1((uint8_t*)buf1);
        // 2nd buffer is for DMA performance, such that we can write one buffer while the other is transferring
        lcd.buffer2((uint8_t*)buf2);
        lcd.on_flush_callback(uix_flush,panel_handle);
        
}
// UIX calls this
static void uix_touch(point16* out_locations,
                                           size_t* in_out_locations_size,
                                           void* state)
{
    esp_lcd_touch_handle_t handle = (esp_lcd_touch_handle_t)state;
    if(ESP_OK==esp_lcd_touch_read_data(handle)) {
        uint16_t x[5],y[5],s[5];
        uint8_t count;
        if(esp_lcd_touch_get_coordinates(handle,x,y,s,&count,5)) {
            if(count>*in_out_locations_size) {
                count = *in_out_locations_size;
            }
            *in_out_locations_size = count;
            for(size_t i = 0;i<(size_t)count;++i) {
                // the panel may have a different res than the screen
                x[i]=x[i]*LCD_HRES/LCD_TOUCH_HRES;
                y[i]=y[i]*LCD_VRES/LCD_TOUCH_VRES;
                out_locations[i]=point16(x[i],y[i]);
            }
            return;
        }
    }
    *in_out_locations_size = 0;
}

static void touch_initialize() {
    esp_lcd_panel_io_i2c_config_t tio_cfg;
    esp_lcd_panel_io_handle_t tio_handle;
    esp_lcd_touch_config_t tp_cfg;
    i2c_master_bus_handle_t i2c_handle;
    esp_lcd_touch_handle_t touch_handle;
    memset(&tio_cfg,0,sizeof(tio_cfg));
    tio_cfg.dev_addr = ESP_LCD_TOUCH_IO_I2C_GT911_ADDRESS;
    tio_cfg.control_phase_bytes = 1;
    tio_cfg.dc_bit_offset = 0;
    tio_cfg.lcd_cmd_bits = LCD_TOUCH_CMD_BITS;
    tio_cfg.lcd_param_bits = LCD_TOUCH_PARAM_BITS;
#ifdef LCD_TOUCH_DISABLE_CONTROL_PHASE
    tio_cfg.flags.disable_control_phase = 1;
#endif
    tio_cfg.flags.dc_low_on_data = 0;
    tio_cfg.on_color_trans_done = NULL;
#ifndef LEGACY_I2C
    tio_cfg.scl_speed_hz = LCD_TOUCH_SPEED;
#endif
    tio_cfg.user_ctx = NULL;
#ifndef LEGACY_I2C
    ESP_ERROR_CHECK(i2c_master_get_bus_handle(LCD_TOUCH_I2C_HOST,&i2c_handle)) ;
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(i2c_handle, &tio_cfg,&tio_handle));
#else
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c_v1(LCD_TOUCH_I2C_HOST,&tio_cfg,&tio_handle));
#endif
    memset(&tp_cfg,0,sizeof(tp_cfg));
    tp_cfg.x_max = LCD_TOUCH_HRES;
    tp_cfg.y_max = LCD_TOUCH_VRES;
#ifdef LCD_TOUCH_PIN_NUM_RST
    tp_cfg.rst_gpio_num = (gpio_num_t)LCD_TOUCH_PIN_NUM_RST;
#else
    tp_cfg.rst_gpio_num = (gpio_num_t)-1;
#endif
#ifdef LCD_TOUCH_PIN_NUM_INT
    tp_cfg.rst_gpio_num = (gpio_num_t)LCD_TOUCH_PIN_NUM_INT;
#else
    tp_cfg.int_gpio_num = (gpio_num_t)-1;
#endif
    tp_cfg.levels.reset = 0;
    tp_cfg.levels.interrupt = 0;
    tp_cfg.flags.swap_xy = 0;
    tp_cfg.flags.mirror_x = 0;
    tp_cfg.flags.mirror_y = 0;
#ifdef LCD_TOUCH_RESET
    LCD_TOUCH_RESET;
#endif
    // this fails sometimes, will reboot, and then works?
    ESP_ERROR_CHECK(LCD_TOUCH_PANEL(tio_handle,&tp_cfg,&touch_handle));
    lcd.on_touch_callback(uix_touch,touch_handle);
}

static size16 chess_piece_bmp_dim;
static uint8_t* chess_piece_bmp_data[6];

static const_buffer_stream* chess_piece_svg[] = {
    &piece_chess_pawn,
    &piece_chess_bishop,
    &piece_chess_rook,
    &piece_chess_knight,
    &piece_chess_queen,
    &piece_chess_king
};
static sizef chess_piece_size[] = {
    piece_chess_pawn_dimensions,
    piece_chess_bishop_dimensions,
    piece_chess_rook_dimensions,
    piece_chess_knight_dimensions,
    piece_chess_queen_dimensions,
    piece_chess_king_dimensions
};

static gfx::rectf correct_aspect(const gfx::rect16& r, float aspect) {
    rect16 result = r;
    if (aspect>=1.f) {
        result.y2 /= aspect;
    } else {
        result.x2 *= aspect;
    }
    return (gfx::rectf)result;
}
// Take the SVG chess pieces, scale them to the target size and rasterize them
static bool create_piece_data(uint16_t piece_size) {
    memset(chess_piece_bmp_data,0,sizeof(chess_piece_bmp_data));
    chess_piece_bmp_dim  = {piece_size,piece_size};
    for(size_t i = 0;i<6;++i) {
        // create a new bitmap in 4-bit grayscale
        auto bmp = create_bitmap<gsc_pixel<4>>({piece_size,piece_size});
        if(bmp.begin()==nullptr) {
            // out of memory
            goto error;
        }
        // fill with white
        bmp.fill(bmp.bounds(),gsc_pixel<4>(15));
        // assign the bitmap array entry to the bitmap's buffer
        chess_piece_bmp_data[i]=bmp.begin();
        canvas cvs(bmp.dimensions());
        if(gfx_result::success!=cvs.initialize()) {
            // out of memory
            goto error;
        }
        // link the canvas and the bitmap so the canvas can draw on it
        if(gfx_result::success!=draw::canvas(bmp,cvs)) {
            // can't imagine why this would fail
            puts("Failed to bind canvas to bitmap");
            goto error;
        }
        sizef cps = chess_piece_size[i];
        gfx::rectf corrected = correct_aspect(bmp.bounds(), cps.aspect_ratio());
        corrected.center_inplace((gfx::rectf)bmp.bounds());
        matrix fit = matrix::create_fit_to(cps,corrected);
        const_buffer_stream& stm = *chess_piece_svg[i];
        stm.seek(0); // make sure we're at the beginning
        if(gfx_result::success!=cvs.render_svg(stm,fit)) {
            puts("Error rasterizing SVG");
            goto error;
        }
        cvs.deinitialize();
        // invert, because it's black on white, but we need an alpha transparency map
        const size_t size = bitmap<gsc_pixel<4>>::sizeof_buffer(bmp.dimensions());
        for(size_t j = 0;j<size;++j) {
            uint8_t d = chess_piece_bmp_data[i][j];
            chess_piece_bmp_data[i][j]=255-d;
        }
    }
    return true;
error:
    for(size_t i = 0; i < 6; ++i) {
        if(chess_piece_bmp_data[i]!=NULL) {
            free(chess_piece_bmp_data[i]);
            chess_piece_bmp_data[i]=NULL;
        }
    }
    puts("Error creating pieces");
    return false;
}

// declare our core GFX and UIX types
using pixel_t = rgb_pixel<LCD_BIT_DEPTH>; // screen bitmap format
using color_t = color<pixel_t>; // color enum (screen native)
using vcolor_t = color<vector_pixel>; // color enum (vector)
using uix_color_t = color<rgba_pixel<32>>; // color enum (UIX)
using screen_t = screen<pixel_t>; // the screen

// Workaround: When a piece gets promoted it needs to be redrawn, but
// we won't know that within the chess board itself, so we set
// it in the board and check it in the app loop and use it to invalidate
// the right square
static chess_index_t just_promoted = -1;

// The control to display the chess promotion UI
template <typename ControlSurfaceType>
class chess_promotion : public control<ControlSurfaceType> {
    using base_type = control<ControlSurfaceType>;
public:
    using control_surface_type = ControlSurfaceType;
    using pixel_type = typename ControlSurfaceType::pixel_type;
    using palette_type = typename ControlSurfaceType::palette_type;
    /// @brief Moves a chess_promotion control
    /// @param rhs The control to move
    chess_promotion(chess_promotion&& rhs) {
        do_move_control(rhs);
    }
    /// @brief Moves a chess_promotion control
    /// @param rhs The control to move
    /// @return this
    chess_promotion& operator=(chess_promotion&& rhs) {
        do_move_control(rhs);
        return *this;
    }
    /// @brief Copies a chess_promotion control
    /// @param rhs The control to copy
    chess_promotion(const chess_promotion& rhs) {
        do_copy_control(rhs);
    }
    /// @brief Copies a chess_promotion control
    /// @param rhs The control to copy
    /// @return this
    chess_promotion& operator=(const chess_promotion& rhs) {
        do_copy_control(rhs);
        return *this;
    }
    /// @brief Constructs a chess_promotion from a given parent with an optional palette
    /// @param parent The parent the control is bound to - usually the screen
    /// @param palette The palette associated with the control. This is usually the screen's palette.
    chess_promotion(invalidation_tracker& parent, const palette_type* palette = nullptr) : base_type(parent, palette) {
        init();
    }
    /// @brief Constructs a chess_promotion from a given parent with an optional palette
    chess_promotion() : base_type() {
        init();
    }
    chess_index_t index() const {
        return m_index;
    }
    void index(chess_index_t value) {
        if(value<0 || value>63) {
            return;
        }
        m_index = value;
    }
    chess_game_t& game() const {
        return *m_game;
    }
    void game(chess_game_t& value) {
        m_game = &value;
    }
   private:
    chess_game_t* m_game;
    chess_index_t m_index;
    spoint16 m_last_touch;
    void init() {
        m_index = -1;
        m_game = nullptr;
    }
    int point_to_square(spoint16 point) {
        const int16_t extent = this->dimensions().width;
        const int x = point.x / (extent / 4);
        return x;
    }
    void square_coords(int index, srect16* out_rect) {
        const int16_t extent = this->dimensions().width;
        const ssize16 square_size(extent / 4, extent / 4);
        const int x = index % 4;
        const spoint16 origin(x * (extent / 4), 0);
        *out_rect = srect16(origin, square_size);
    }
   
   protected:
    void do_move_control(chess_promotion& rhs) {
        do_copy_control(rhs);
    }
    void do_copy_control(chess_promotion& rhs) {
        m_index = rhs.m_index;
        m_game = rhs.m_game;
        m_last_touch = rhs.m_last_touch;
    }

    void on_paint(control_surface_type& destination, const srect16& clip) override {
        const int16_t extent = this->dimensions().width;
        const int16_t sq_width = (extent/4);
        const int idx = point_to_square(m_last_touch);
        int type = 1;
        for(int x = 0; x < extent; x+=sq_width) {
            const ssize16 square_size(sq_width,sq_width);
            const srect16 square(spoint16(x,0),square_size);
            auto bg = uix_color_t::dark_green,
                bd = uix_color_t::dark_olive_green;
            if(idx==(x/sq_width)) {
                bg=uix_color_t::green;
                bd=uix_color_t::dark_green;
            }
            draw::filled_rectangle(destination,square,uix_color_t::dark_green);
            draw::rectangle(destination,square.inflate(-2,-2),uix_color_t::dark_olive_green);
            const auto ico = const_bitmap<alpha_pixel<4>>(chess_piece_bmp_dim,chess_piece_bmp_data[(size_t)type]);
            const srect16 icon = ((srect16)ico.bounds()).center(square_size.bounds()).offset(x, 0);
            draw::icon(destination,icon.top_left(),ico,uix_color_t::khaki);
            ++type;
        }
    }
    bool on_touch(size_t locations_size, const spoint16* locations) override {
        if(locations_size) {
            m_last_touch = locations[0];
        }
        return true;
    }
    void on_release() override {
        if(m_game!=nullptr && m_index>-1 && m_index<64) {
            chess_type_t type = (chess_type_t)(1+point_to_square(m_last_touch));
            if(CHESS_SUCCESS==chess_promote_pawn(m_game,m_index,type)) {
                just_promoted = m_index;
            }
        }
        this->visible(false);
    }
};

// the screen has a control surface type used by each control
// which presents a drawing surface
using surface_t = screen_t::control_surface_type;
using chess_promotion_t = chess_promotion<surface_t>;
using label_t = vlabel<surface_t>;
using icon_t = vicon<surface_t>;

static chess_promotion_t promotion_top;
static chess_promotion_t promotion_bottom;
static char score_white_text[32], score_black_text[32];
static label_t score_white, score_black;

// operates the chess board
template <typename ControlSurfaceType>
class chess_board : public control<ControlSurfaceType> {
    using base_type = control<ControlSurfaceType>;
    chess_game_t m_game;
    chess_index_t m_moves[64];
    size_t m_moves_size;
    chess_index_t m_touched;
    spoint16 m_last_touch;

    void init_board() {
        chess_init(&m_game);
        m_moves_size = 0;
        m_touched = -1;
    }

    int point_to_square(spoint16 point) {
        const int16_t extent = this->dimensions().aspect_ratio() >= 1 ? this->dimensions().height : this->dimensions().width;
        const int x = point.x / (extent / 8);
        const int y = point.y / (extent / 8);
        return y * 8 + x;
    }
    void square_coords(int index, srect16* out_rect) {
        const int16_t extent = this->dimensions().aspect_ratio() >= 1 ? this->dimensions().height : this->dimensions().width;
        const ssize16 square_size(extent / 8, extent / 8);
        const int x = index % 8;
        const int y = index / 8;
        const spoint16 origin(x * (extent / 8), y * (extent / 8));
        *out_rect = srect16(origin, square_size);
    }
    
   public:
    using control_surface_type = ControlSurfaceType;
    using pixel_type = typename ControlSurfaceType::pixel_type;
    using palette_type = typename ControlSurfaceType::palette_type;
    /// @brief Moves a chess_board control
    /// @param rhs The control to move
    chess_board(chess_board&& rhs) {
        do_move_control(rhs);
    }
    chess_team_t turn() const {
        return chess_turn(&m_game);
    }
    const chess_game_t& game() const { 
        return m_game;
    }
    void game(const chess_game_t& value) {
        m_game=value;
        m_moves_size = 0;
        m_touched = -1;
    }
    /// @brief Moves a chess_board control
    /// @param rhs The control to move
    /// @return this
    chess_board& operator=(chess_board&& rhs) {
        do_move_control(rhs);
        return *this;
    }
    /// @brief Copies a chess_board control
    /// @param rhs The control to copy
    chess_board(const chess_board& rhs) {
        do_copy_control(rhs);
    }
    /// @brief Copies a chess_board control
    /// @param rhs The control to copy
    /// @return this
    chess_board& operator=(const chess_board& rhs) {
        do_copy_control(rhs);
        return *this;
    }
    /// @brief Constructs a chess_board from a given parent with an optional palette
    /// @param parent The parent the control is bound to - usually the screen
    /// @param palette The palette associated with the control. This is usually the screen's palette.
    chess_board(invalidation_tracker& parent, const palette_type* palette = nullptr) : base_type(parent, palette) {
        init_board();
    }
    /// @brief Constructs a chess_board from a given parent with an optional palette
    chess_board() : base_type() {
        init_board();
    }

   protected:
    void do_move_control(chess_board& rhs) {
        do_copy_control(rhs);
    }
    void do_copy_control(chess_board& rhs) {
        memcpy(&m_game, &rhs.m_game, sizeof(m_game));
        if (rhs.m_moves_size) {
            memcpy(m_moves, rhs.m_moves, rhs.m_moves_size * sizeof(int));
        }
        m_moves_size = rhs.m_moves_size;
        m_last_touch = rhs.m_last_touch;
    }
    void on_paint(control_surface_type& destination, const srect16& clip) override {
        const int16_t extent = destination.dimensions().aspect_ratio() >= 1 ? destination.dimensions().height : destination.dimensions().width;
        const ssize16 square_size(extent / 8, extent / 8);
        bool toggle = false;
        int idx = 0;
        for (int y = 0; y < extent; y += square_size.height) {
            int i = toggle;
            for (int x = 0; x < extent; x += square_size.width) {
                const srect16 square(spoint16(x, y), square_size);
                if (square.intersects(clip)) {
                    const chess_id_t id = chess_index_to_id(&m_game,idx);
                    const auto bg = (i & 1) ? uix_color_t::brown : uix_color_t::dark_khaki;
                    const auto bd = (i & 1) ? uix_color_t::gold : uix_color_t::black;
                    auto px_bg = bg;
                    auto px_bd = bd;
                    if (id > -1 && CHESS_TYPE(id) == CHESS_KING && chess_status(&m_game,CHESS_TEAM(id)) == CHESS_CHECK) {
                        px_bd = uix_color_t::red;
                    }
                    bool is_move = false;
                    if (m_touched == idx || chess_contains_move(m_moves, m_moves_size, idx)) {
                        px_bg = uix_color_t::light_blue;
                        is_move = true;
                    }
                    
                    if(!is_move) {
                        draw::filled_rectangle(destination, square, px_bg);
                        draw::rectangle(destination, square.inflate(-2, -2), px_bd); 
                    } else {
                        draw::filled_rectangle(destination, square, bg);
                        draw::rectangle(destination, square.inflate(-2, -2), px_bd);
                        const int16_t deflate = (idx==m_touched)?0: -(square_size.width/4);
                        draw::filled_rectangle(destination, square.inflate(deflate,deflate), px_bg);
                        
                    }
                    if (CHESS_NONE != id) {
                        auto ico = const_bitmap<alpha_pixel<4>>(chess_piece_bmp_dim,chess_piece_bmp_data[CHESS_TYPE(id)]);
                        const srect16 bounds = ((srect16)ico.bounds()).center(square_size.bounds()).offset(x, y);
                        auto px_piece = CHESS_TEAM(id) ? uix_color_t::black : uix_color_t::white;
                        draw::icon(destination, bounds.location(), ico, px_piece);
                    }
                }
                ++i;
                ++idx;
            }
            toggle = !toggle;
        }
    }
    bool on_touch(size_t locations_size, const spoint16* locations) override {
        if(promotion_bottom.visible() || promotion_top.visible()) {
            return false;
        }
        if (m_touched > -1) {
            if (locations_size) m_last_touch = locations[0];
            return true;
        }
        if (locations_size) {
            const int16_t extent = this->dimensions().aspect_ratio() >= 1 ? this->dimensions().height : this->dimensions().width;
            const srect16 square(spoint16::zero(), ssize16(extent / 8, extent / 8));
            int sq = point_to_square(*locations);
            if (sq > -1) {
                const chess_id_t id = chess_index_to_id(&m_game,sq);
                if (id > -1) {
                    const chess_team_t team = CHESS_TEAM(id);
                    if (chess_turn(&m_game) == team) {
                        m_touched = sq;
                        m_moves_size = chess_compute_moves(&m_game,sq,m_moves);
                        srect16 sq_bnds;
                        square_coords(sq, &sq_bnds);
                        this->invalidate(sq_bnds);
                    }
                }
                if (m_moves_size > 0) {
                    for (size_t i = 0; i < m_moves_size; ++i) {
                        srect16 sq_bnds;
                        square_coords(m_moves[i], &sq_bnds);
                        this->invalidate(sq_bnds);
                    }
                    return true;
                }
            }
        }
        return false;
    }
    void on_release() override {
        if (m_touched > -1) {
            const chess_id_t id = chess_index_to_id(&m_game,m_touched);
            const chess_team_t team = CHESS_TEAM(id);
            const int16_t extent = this->dimensions().aspect_ratio() >= 1 ? this->dimensions().height : this->dimensions().width;
            const srect16 square(spoint16::zero(), ssize16(extent / 8, extent / 8));
            const int x = m_touched % 8 * (extent / 8), y = m_touched / 8 * (extent / 8);
            this->invalidate(square.offset(x, y));
            if (m_moves_size > 0) {
                for (size_t i = 0; i < m_moves_size; ++i) {
                    srect16 sq_bnds;
                    square_coords(m_moves[i], &sq_bnds);
                    this->invalidate(sq_bnds);
                }
                const int release_idx = point_to_square(m_last_touch);
                if (release_idx != -1) {
                    chess_index_t victim = chess_move(&m_game,m_touched,release_idx);
                    FILE* f = fopen("/spiffs/chess.bin","wb");
                    if(f!=NULL) {
                        fwrite(&m_game,1,sizeof(m_game),f);
                        fclose(f);
                    }    
                    srect16 sq_bnds;
                    square_coords(release_idx, &sq_bnds);
                    this->invalidate(sq_bnds);
                    if(victim!=CHESS_NONE) { 
                        // update the score
                        chess_score_t score = chess_score(&m_game, team);
                        if(team==0) {
                            sprintf(score_white_text,"%d",(int)score);
                            score_white.text(score_white_text);
                        } else {
                            sprintf(score_black_text,"%d",(int)score);
                            score_black.text(score_black_text);
                        }
                        if(victim!=release_idx) { // en passant
                            square_coords(victim,&sq_bnds);
                            this->invalidate(sq_bnds);
                        }
                    }
                    if(CHESS_TYPE(id)==CHESS_PAWN) { // check for a promotion
                        if(team==CHESS_WHITE && release_idx>=64-8) {
                            // promote 
                            promotion_bottom.game(m_game);
                            promotion_bottom.index(release_idx);
                            promotion_bottom.visible(true);
                        } else if(team==CHESS_BLACK && release_idx<8) {
                            // promote 
                            promotion_top.game(m_game);
                            promotion_top.index(release_idx);
                            promotion_top.visible(true);
                        }
                    }
                }
                m_moves_size = 0;
            }
        }
        m_touched = -1;
    }
};

using chess_board_t = chess_board<surface_t>;
static screen_t main_screen;
static chess_board_t board;
static icon_t reset_button;

static void reset_button_on_pressed_changed(bool pressed, void* state) {
    if(!pressed) {
        chess_game_t g;
        chess_init(&g);
        FILE* f = fopen("/spiffs/chess.bin", "wb");
        if(f!=NULL) {
            fwrite(&g,1,sizeof(g),f);
            fclose(f);
        }
        esp_restart();
    }
}

extern "C" void app_main() {
    printf("ESP-IDF version: %d.%d.%d\n",ESP_IDF_VERSION_MAJOR,ESP_IDF_VERSION_MINOR,ESP_IDF_VERSION_PATCH);
    i2c_initialize();
    spiffs_initialize();
    lcd_initialize();
    touch_initialize();
    main_screen.dimensions({LCD_WIDTH,LCD_HEIGHT});
    const float screen_aspect = main_screen.dimensions().aspect_ratio();
    int16_t piece_size = 0;
    if(screen_aspect>1.f) {
        piece_size= main_screen.dimensions().height/8;
    } else if(screen_aspect<1.f) {
        piece_size = main_screen.dimensions().width/8;
    } else {
        piece_size = main_screen.dimensions().width/10;
    }
    srect16 board_bounds(spoint16::zero(),ssize16(piece_size*8,piece_size*8));
    board_bounds.center_inplace(main_screen.bounds());
    main_screen.background_color(color_t::black);
    if(!create_piece_data(piece_size)) {
        puts("could not crete pieces");
        return;
    }
    const size16 sq_size(piece_size,piece_size);
    srect16 contained;
    if(screen_aspect<=1.f) {
        contained = srect16(0,0,main_screen.dimensions().width-1,board.bounds().y1-1);
    } else {
        contained = srect16(0,0,main_screen.dimensions().width-1,piece_size-1);
    }
    board.bounds(board_bounds);
    FILE* f = fopen("/spiffs/chess.bin","rb");
    chess_game_t g;
    if(f!=NULL) {
        if(sizeof(g)!=fread(&g,1,sizeof(g),f)) {
            chess_init(&g);
        }
        fclose(f);
    } else {
        chess_init(&g);
    }
    board.game(g);
    promotion_top.game(g);
    promotion_bottom.game(g);
    main_screen.register_control(board);
    
    reset_button.bounds(srect16(0,0,piece_size-1,piece_size-1));
    reset_button.background_color(uix_color_t::orange_red);
    reset_button.svg(power_power_off);
    reset_button.svg_dimensions(power_power_off_dimensions);
    reset_button.on_pressed_changed_callback(reset_button_on_pressed_changed);
    main_screen.register_control(reset_button);
    
    const srect16 promo_rect(0,0,piece_size*4-1,piece_size-1);
    promotion_top.bounds(promo_rect.center(contained));
    promotion_top.visible(false);
    main_screen.register_control(promotion_top);
    contained.offset_inplace(0,main_screen.dimensions().height-contained.height());
    promotion_bottom.bounds(promo_rect.center(contained));
    promotion_bottom.visible(false);
    main_screen.register_control(promotion_bottom);
    if(screen_aspect<=1.f) {
        contained = srect16(0,0,board.bounds().x1-1,board.bounds().x1-1);
        contained.center_vertical_inplace(main_screen.bounds());
    } else {
        contained = srect16(0,0,board.bounds().x1-1,board.bounds().x1-1);
        contained.center_vertical_inplace(main_screen.bounds());
    }
    contained.inflate_inplace(-2,-2);
    score_white.bounds(contained);
    score_white.font(font_stream);
    score_white.text_justify(uix_justify::center);
    score_white.background_color(uix_color_t::green);
    score_white.color(uix_color_t::white);
    score_white.radiuses(sizef(chess_piece_size->width/10,chess_piece_size->width/10));
    sprintf(score_white_text,"%d",(int)chess_score(&g,CHESS_WHITE));
    score_white.text(score_white_text);
    main_screen.register_control(score_white);
    if(screen_aspect<=1.f) {
        contained = srect16(board.bounds().x2+1,0,main_screen.bounds().x2,contained.width()-1);
        contained.center_vertical_inplace(main_screen.bounds());
    } else {
        contained = srect16(0,0,board.bounds().x1-1,board.bounds().x1-1);
        contained.offset_inplace(main_screen.dimensions().width-contained.width(),0);
        contained.center_vertical_inplace(main_screen.bounds());
    }
    score_black.font(font_stream);
    score_black.text_justify(uix_justify::center);
    score_black.bounds(contained);
    score_black.background_color(uix_color_t::dark_olive_green);
    score_black.color(uix_color_t::black);
    score_black.radiuses(sizef(chess_piece_size->width/10,chess_piece_size->width/10));
    sprintf(score_black_text,"%d",(int)chess_score(&g,CHESS_BLACK));
    score_black.text(score_black_text);
    main_screen.register_control(score_black);
    lcd.active_screen(main_screen);
    uint32_t ms = pdTICKS_TO_MS(xTaskGetTickCount());
    uint32_t blink_turn_ms = 0;
    while(1) {
        if(pdTICKS_TO_MS(xTaskGetTickCount()) >= ms+200) {
            ms = pdTICKS_TO_MS(xTaskGetTickCount());
            vTaskDelay(5);
        }
        if(pdTICKS_TO_MS(xTaskGetTickCount()) >= blink_turn_ms+500) {
            // blink the active turn score label
            blink_turn_ms = pdTICKS_TO_MS(xTaskGetTickCount());
            if(CHESS_WHITE==board.turn()) {
                score_white.visible(!score_white.visible());
                score_black.visible(true);
            } else {
                score_black.visible(!score_black.visible());
                score_white.visible(true);
            }
        }
        if(just_promoted!=-1) {
            const int16_t extent = board.dimensions().aspect_ratio()>=1.f?board.dimensions().width:board.dimensions().height;
            const ssize16 square_size(extent / 8, extent / 8);
            const int x = just_promoted % 8;
            const int y = just_promoted / 8;
            const spoint16 origin(x * (extent / 8), y * (extent / 8));
            just_promoted = -1;
            main_screen.invalidate(srect16(origin.offset(board.bounds().top_left()),square_size));
        }
        if(lcd_is_vsync==1) {
            lcd_is_vsync=0;
            lcd.update();
        }
    }
}