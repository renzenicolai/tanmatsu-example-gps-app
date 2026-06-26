#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "bsp/device.h"
#include "bsp/display.h"
#include "bsp/input.h"
#include "bsp/led.h"
#include "bsp/power.h"
#include "custom_certificates.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_types.h"
#include "esp_log.h"
#include "hal/lcd_types.h"
#include "minmea.h"
#include "nvs_flash.h"
#include "pax_fonts.h"
#include "pax_gfx.h"
#include "pax_text.h"
#include "pax_types.h"
#include "portmacro.h"

// Constants
static char const TAG[] = "main";

#if defined(CONFIG_BSP_TARGET_KAMI)
#define BLACK 0
#define WHITE 1
#define RED   2
#else
#define BLACK 0xFF000000
#define WHITE 0xFFFFFFFF
#define RED   0xFFFF0000
#endif

// Global variables
static size_t                     display_h_res        = 0;
static size_t                     display_v_res        = 0;
static bsp_display_color_format_t display_color_format = 0;
static bsp_display_endianness_t   display_data_endian  = 0;
static pax_buf_t                  fb                   = {0};
static QueueHandle_t              input_event_queue    = NULL;

#if defined(CONFIG_BSP_TARGET_KAMI)
// Temporary addition for supporting epaper devices (irrelevant for Tanmatsu)
static pax_col_t palette[] = {0xffffffff, 0xff000000, 0xffff0000};  // white, black, red
#endif

static void blit(void) {
    esp_err_t res = bsp_display_blit(0, 0, display_h_res, display_v_res, pax_buf_get_pixels(&fb));
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to blit to display: %d", res);
    }
}

#define GPS_UART_PORT UART_NUM_1
#define GPS_UART_BAUD 9600
#define GPS_BUF_SIZE  1024

#define MAX_SATS_IN_VIEW 32
#define DEG2RAD          (3.14159265f / 180.0f)

typedef struct {
    int   prn;
    int   elevation;
    int   azimuth;
    float snr;
} sat_info_t;

typedef struct {
    bool               has_fix;
    bool               has_time;
    float              latitude;
    float              longitude;
    float              altitude;
    char               altitude_units;
    float              speed_knots;
    float              course;
    int                satellites;
    int                fix_type;
    float              hdop;
    struct minmea_time time;
    struct minmea_date date;
    sat_info_t         sats[MAX_SATS_IN_VIEW];
    int                sat_count;
} gps_data_t;

static gps_data_t gps_data = {0};

static pax_col_t sat_signal_color(float snr) {
    if (isnan(snr) || snr <= 0) return 0xFF555555;  // no signal data
    if (snr >= 35) return 0xFF00DD55;               // good
    if (snr >= 20) return 0xFFFFCC00;               // medium
    return 0xFFFF6600;                              // weak
}

// Sky plot: center (cx,cy), radius r. North=up, azimuth clockwise.
static void draw_sky_plot(float cx, float cy, float r) {
    // Dark filled background
    pax_draw_circle(&fb, 0xFF111111, cx, cy, r);

    // Dim elevation rings and crosshairs
    pax_outline_circle(&fb, 0xFF2A2A2A, cx, cy, r * 2.0f / 3.0f);
    pax_outline_circle(&fb, 0xFF2A2A2A, cx, cy, r / 3.0f);
    pax_simple_line(&fb, 0xFF222222, cx - r, cy, cx + r, cy);
    pax_simple_line(&fb, 0xFF222222, cx, cy - r, cx, cy + r);

    // Horizon circle
    pax_outline_circle(&fb, 0xFF666666, cx, cy, r);

    // Elevation ring labels (30° and 60°)
    float lfs = r * 0.075f;
    pax_draw_text(&fb, 0xFF444444, pax_font_sky_mono, lfs, cx + r * 2.0f / 3.0f + 3.0f, cy - lfs * 0.5f, "30");
    pax_draw_text(&fb, 0xFF444444, pax_font_sky_mono, lfs, cx + r / 3.0f + 3.0f, cy - lfs * 0.5f, "60");

    // Cardinal labels, sized and positioned relative to the plot radius
    float     cfs = r * 0.09f;
    pax_vec2f chs = pax_text_size(pax_font_sky_mono, cfs, "N");
    pax_draw_text(&fb, 0xFFCCCCCC, pax_font_sky_mono, cfs, cx - chs.x * 0.5f, cy - r - chs.y - 3.0f, "N");
    pax_draw_text(&fb, 0xFFCCCCCC, pax_font_sky_mono, cfs, cx - chs.x * 0.5f, cy + r + 4.0f, "S");
    pax_draw_text(&fb, 0xFFCCCCCC, pax_font_sky_mono, cfs, cx + r + 5.0f, cy - chs.y * 0.5f, "E");
    pax_draw_text(&fb, 0xFFCCCCCC, pax_font_sky_mono, cfs, cx - r - chs.x - 5.0f, cy - chs.y * 0.5f, "W");

    // Satellite dots + PRN labels, scaled to plot size
    float dot_r = r * 0.045f;
    float pfs   = r * 0.065f;
    for (int i = 0; i < gps_data.sat_count; i++) {
        sat_info_t* s      = &gps_data.sats[i];
        float       az     = s->azimuth * DEG2RAD;
        float       plot_r = r * (1.0f - (float)s->elevation / 90.0f);
        float       sx     = cx + plot_r * sinf(az);
        float       sy     = cy - plot_r * cosf(az);
        pax_col_t   col    = sat_signal_color(s->snr);
        pax_draw_circle(&fb, col, sx, sy, dot_r);
        char prn_str[5];
        snprintf(prn_str, sizeof(prn_str), "%d", s->prn);
        pax_draw_text(&fb, col, pax_font_sky_mono, pfs, sx + dot_r + 1.0f, sy - pfs * 0.5f, prn_str);
    }
}

static void display_gps_data(void) {
    if (pax_buf_get_width(&fb) == 0) return;

    float w = pax_buf_get_widthf(&fb);
    float h = pax_buf_get_heightf(&fb);

    // Left panel: 45% of width; sky plot fills the right 55%
    float panel_w = floorf(w * 0.45f);
    float cx      = panel_w + (w - panel_w) * 0.5f;
    float cy      = h * 0.5f;
    float r       = fminf((w - panel_w) * 0.5f, h * 0.5f) - 44.0f;

    char text[48];
    pax_background(&fb, 0xFF000000);

    // Title
    pax_draw_text(&fb, 0xFFFFFFFF, pax_font_sky_mono, 26, 10, 10, "GPS Receiver");

    // Fix status: coloured dot + label
    if (gps_data.has_fix) {
        pax_draw_circle(&fb, 0xFF00DD55, 19, 52, 7);
        pax_draw_text(&fb, 0xFF00DD55, pax_font_sky_mono, 18, 32, 43, "FIX OK");
    } else {
        pax_draw_circle(&fb, 0xFFFF4444, 19, 52, 7);
        pax_draw_text(&fb, 0xFFFF4444, pax_font_sky_mono, 18, 32, 43, "NO FIX");
    }

    // Shared layout constants for label/value rows
    float lx = 10, vx = 155, lfs = 18, lh = 26, y = 82;

    // Section: time & date
    pax_simple_line(&fb, 0xFF333333, 0, y, panel_w, y);
    y += 12;

    pax_draw_text(&fb, 0xFF777777, pax_font_sky_mono, lfs, lx, y, "Time");
    if (gps_data.has_time) {
        snprintf(text, sizeof(text), "%02d:%02d:%02d UTC", gps_data.time.hours, gps_data.time.minutes,
                 gps_data.time.seconds);
    } else {
        snprintf(text, sizeof(text), "--:--:--");
    }
    pax_draw_text(&fb, 0xFFFFFFFF, pax_font_sky_mono, lfs, vx, y, text);
    y += lh;

    pax_draw_text(&fb, 0xFF777777, pax_font_sky_mono, lfs, lx, y, "Date");
    if (gps_data.has_time) {
        int year = gps_data.date.year < 100 ? gps_data.date.year + 2000 : gps_data.date.year;
        snprintf(text, sizeof(text), "%02d / %02d / %04d", gps_data.date.day, gps_data.date.month, year);
    } else {
        snprintf(text, sizeof(text), "--/--/----");
    }
    pax_draw_text(&fb, 0xFFFFFFFF, pax_font_sky_mono, lfs, vx, y, text);

    // Section: position
    y += lh + 10;
    pax_simple_line(&fb, 0xFF333333, 0, y, panel_w, y);
    y += 12;

    pax_draw_text(&fb, 0xFF777777, pax_font_sky_mono, lfs, lx, y, "Latitude");
    pax_draw_text(&fb, 0xFF777777, pax_font_sky_mono, lfs, lx, y + lh, "Longitude");
    if (gps_data.has_fix) {
        snprintf(text, sizeof(text), "%+.6f", gps_data.latitude);
        pax_draw_text(&fb, 0xFFFFFFFF, pax_font_sky_mono, lfs, vx, y, text);
        snprintf(text, sizeof(text), "%+.6f", gps_data.longitude);
        pax_draw_text(&fb, 0xFFFFFFFF, pax_font_sky_mono, lfs, vx, y + lh, text);
    } else {
        pax_draw_text(&fb, 0xFF555555, pax_font_sky_mono, lfs, vx, y, "--");
        pax_draw_text(&fb, 0xFF555555, pax_font_sky_mono, lfs, vx, y + lh, "--");
    }

    // Section: altitude & satellites
    y += 2 * lh + 10;
    pax_simple_line(&fb, 0xFF333333, 0, y, panel_w, y);
    y += 12;

    pax_draw_text(&fb, 0xFF777777, pax_font_sky_mono, lfs, lx, y, "Altitude");
    snprintf(text, sizeof(text), "%.1f %c", gps_data.altitude, gps_data.altitude_units ? gps_data.altitude_units : 'm');
    pax_draw_text(&fb, 0xFFFFFFFF, pax_font_sky_mono, lfs, vx, y, text);
    y += lh;

    pax_draw_text(&fb, 0xFF777777, pax_font_sky_mono, lfs, lx, y, "Satellites");
    snprintf(text, sizeof(text), "%d", gps_data.satellites);
    pax_draw_text(&fb, 0xFFFFFFFF, pax_font_sky_mono, lfs, vx, y, text);

    // Section: speed & course
    y += lh + 10;
    pax_simple_line(&fb, 0xFF333333, 0, y, panel_w, y);
    y += 12;

    pax_draw_text(&fb, 0xFF777777, pax_font_sky_mono, lfs, lx, y, "Speed");
    pax_draw_text(&fb, 0xFF777777, pax_font_sky_mono, lfs, lx, y + lh, "Course");
    if (gps_data.has_fix) {
        snprintf(text, sizeof(text), "%.1f kn", gps_data.speed_knots);
        pax_draw_text(&fb, 0xFFFFFFFF, pax_font_sky_mono, lfs, vx, y, text);
        snprintf(text, sizeof(text), "%.1f deg", gps_data.course);
        pax_draw_text(&fb, 0xFFFFFFFF, pax_font_sky_mono, lfs, vx, y + lh, text);
    } else {
        pax_draw_text(&fb, 0xFF555555, pax_font_sky_mono, lfs, vx, y, "--");
        pax_draw_text(&fb, 0xFF555555, pax_font_sky_mono, lfs, vx, y + lh, "--");
    }

    // Section: fix quality
    y += 2 * lh + 10;
    pax_simple_line(&fb, 0xFF333333, 0, y, panel_w, y);
    y += 12;

    pax_draw_text(&fb, 0xFF777777, pax_font_sky_mono, lfs, lx, y, "Fix type");
    const char* fix_str;
    pax_col_t   fix_col;
    switch (gps_data.fix_type) {
        case MINMEA_GPGSA_FIX_2D:
            fix_str = "2D";
            fix_col = 0xFFFFCC00;
            break;
        case MINMEA_GPGSA_FIX_3D:
            fix_str = "3D";
            fix_col = 0xFF00DD55;
            break;
        default:
            fix_str = "--";
            fix_col = 0xFF555555;
            break;
    }
    pax_draw_text(&fb, fix_col, pax_font_sky_mono, lfs, vx, y, fix_str);
    y += lh;

    pax_draw_text(&fb, 0xFF777777, pax_font_sky_mono, lfs, lx, y, "HDOP");
    pax_col_t hdop_col;
    if (isnan(gps_data.hdop) || gps_data.hdop <= 0.0f) {
        snprintf(text, sizeof(text), "--");
        hdop_col = 0xFF555555;
    } else {
        snprintf(text, sizeof(text), "%.1f", gps_data.hdop);
        hdop_col = gps_data.hdop < 2.0f ? 0xFF00DD55 : gps_data.hdop < 5.0f ? 0xFFFFCC00 : 0xFFFF4444;
    }
    pax_draw_text(&fb, hdop_col, pax_font_sky_mono, lfs, vx, y, text);

    // Vertical divider between panels
    pax_simple_line(&fb, 0xFF333333, panel_w, 0, panel_w, h);

    // Sky plot centred in the right panel
    draw_sky_plot(cx, cy, r);

    blit();
}

static void gps_task(void* arg) {
    uint8_t* buf = malloc(GPS_BUF_SIZE);
    char     line[256];
    int      line_pos = 0;

    while (1) {
        int len = uart_read_bytes(GPS_UART_PORT, buf, GPS_BUF_SIZE - 1, pdMS_TO_TICKS(100));
        for (int i = 0; i < len; i++) {
            char c = (char)buf[i];
            if (c == '\n') {
                line[line_pos] = '\0';
                if (line_pos > 0) {
                    switch (minmea_sentence_id(line, false)) {
                        case MINMEA_SENTENCE_RMC: {
                            struct minmea_sentence_rmc frame;
                            if (minmea_parse_rmc(&frame, line)) {
                                gps_data.has_fix     = frame.valid;
                                gps_data.has_time    = (frame.date.year > 0);
                                gps_data.time        = frame.time;
                                gps_data.date        = frame.date;
                                gps_data.latitude    = minmea_tocoord(&frame.latitude);
                                gps_data.longitude   = minmea_tocoord(&frame.longitude);
                                gps_data.speed_knots = minmea_tofloat(&frame.speed);
                                gps_data.course      = minmea_tofloat(&frame.course);
                                printf("RMC: %02d:%02d:%02d fix=%s lat=%.6f lon=%.6f spd=%.1f crs=%.1f\n",
                                       frame.time.hours, frame.time.minutes, frame.time.seconds,
                                       frame.valid ? "valid" : "invalid", gps_data.latitude, gps_data.longitude,
                                       gps_data.speed_knots, gps_data.course);
                                display_gps_data();
                            }
                            break;
                        }
                        case MINMEA_SENTENCE_GGA: {
                            struct minmea_sentence_gga frame;
                            if (minmea_parse_gga(&frame, line)) {
                                gps_data.satellites     = frame.satellites_tracked;
                                gps_data.altitude       = minmea_tofloat(&frame.altitude);
                                gps_data.altitude_units = frame.altitude_units;
                                printf("GGA: sats=%d alt=%.1f%c quality=%d\n", frame.satellites_tracked,
                                       gps_data.altitude, gps_data.altitude_units, frame.fix_quality);
                            }
                            break;
                        }
                        case MINMEA_SENTENCE_GSA: {
                            struct minmea_sentence_gsa frame;
                            if (minmea_parse_gsa(&frame, line)) {
                                gps_data.fix_type = frame.fix_type;
                                gps_data.hdop     = minmea_tofloat(&frame.hdop);
                                printf("GSA: fix=%d hdop=%.1f\n", frame.fix_type, gps_data.hdop);
                            }
                            break;
                        }
                        case MINMEA_SENTENCE_GSV: {
                            struct minmea_sentence_gsv frame;
                            if (minmea_parse_gsv(&frame, line)) {
                                static sat_info_t gsv_buf[MAX_SATS_IN_VIEW];
                                static int        gsv_count = 0;
                                if (frame.msg_nr == 1) {
                                    gsv_count = 0;
                                }
                                for (int i = 0; i < 4 && gsv_count < MAX_SATS_IN_VIEW; i++) {
                                    if (frame.sats[i].nr > 0) {
                                        gsv_buf[gsv_count].prn       = frame.sats[i].nr;
                                        gsv_buf[gsv_count].elevation = frame.sats[i].elevation;
                                        gsv_buf[gsv_count].azimuth   = frame.sats[i].azimuth;
                                        gsv_buf[gsv_count].snr       = minmea_tofloat(&frame.sats[i].snr);
                                        gsv_count++;
                                    }
                                }
                                if (frame.msg_nr == frame.total_msgs) {
                                    memcpy(gps_data.sats, gsv_buf, gsv_count * sizeof(sat_info_t));
                                    gps_data.sat_count = gsv_count;
                                    printf("GSV: %d sats in view\n", gsv_count);
                                }
                            }
                            break;
                        }
                        case MINMEA_INVALID:
                            break;
                        default:
                            break;
                    }
                }
                line_pos = 0;
            } else if (c != '\r' && line_pos < (int)sizeof(line) - 1) {
                line[line_pos++] = c;
            }
        }
    }
}

void app_main(void) {
    // Start the GPIO interrupt service
    gpio_install_isr_service(0);

    // Initialize the Non Volatile Storage partition
    esp_err_t res = nvs_flash_init();
    if (res == ESP_ERR_NVS_NO_FREE_PAGES || res == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        res = nvs_flash_erase();
        if (res != ESP_OK) {
            ESP_LOGE(TAG, "Failed to erase NVS flash: %d", res);
            return;
        }
        res = nvs_flash_init();
    }
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize NVS flash: %d", res);
        return;
    }

    // Initialize the Board Support Package
    const bsp_configuration_t bsp_configuration = {
        .display =
            {
                .requested_color_format = BSP_DISPLAY_COLOR_FORMAT_24_888RGB,
                .num_fbs                = 1,
            },
    };
    res = bsp_device_initialize(&bsp_configuration);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize BSP: %d", res);
        return;
    }

    // Get display parameters and rotation
    res = bsp_display_get_parameters(&display_h_res, &display_v_res, &display_color_format, &display_data_endian);
    if (res != ESP_ERR_NOT_SUPPORTED) {
        if (res != ESP_OK) {
            ESP_LOGE(TAG, "Failed to get display parameters: %d", res);
            return;
        }

        // Convert ESP-IDF color format into PAX buffer type
        pax_buf_type_t format = PAX_BUF_24_888RGB;
        switch (display_color_format) {
            case BSP_DISPLAY_COLOR_FORMAT_1_PAL:
                format = PAX_BUF_1_PAL;
                break;
            case BSP_DISPLAY_COLOR_FORMAT_2_PAL:
                format = PAX_BUF_2_PAL;
                break;
            case BSP_DISPLAY_COLOR_FORMAT_4_PAL:
                format = PAX_BUF_4_PAL;
                break;
            case BSP_DISPLAY_COLOR_FORMAT_8_PAL:
                format = PAX_BUF_8_PAL;
                break;
            case BSP_DISPLAY_COLOR_FORMAT_16_PAL:
                format = PAX_BUF_16_PAL;
                break;
            case BSP_DISPLAY_COLOR_FORMAT_1_GREY:
                format = PAX_BUF_1_GREY;
                break;
            case BSP_DISPLAY_COLOR_FORMAT_2_GREY:
                format = PAX_BUF_2_GREY;
                break;
            case BSP_DISPLAY_COLOR_FORMAT_4_GREY:
                format = PAX_BUF_4_GREY;
                break;
            case BSP_DISPLAY_COLOR_FORMAT_8_GREY:
                format = PAX_BUF_8_GREY;
                break;
            case BSP_DISPLAY_COLOR_FORMAT_8_332RGB:
                format = PAX_BUF_8_332RGB;
                break;
            case BSP_DISPLAY_COLOR_FORMAT_16_565RGB:
                format = PAX_BUF_16_565RGB;
                break;
            case BSP_DISPLAY_COLOR_FORMAT_4_1111ARGB:
                format = PAX_BUF_4_1111ARGB;
                break;
            case BSP_DISPLAY_COLOR_FORMAT_8_2222ARGB:
                format = PAX_BUF_8_2222ARGB;
                break;
            case BSP_DISPLAY_COLOR_FORMAT_16_4444ARGB:
                format = PAX_BUF_16_4444ARGB;
                break;
            case BSP_DISPLAY_COLOR_FORMAT_24_888RGB:
                format = PAX_BUF_24_888RGB;
                break;
            case BSP_DISPLAY_COLOR_FORMAT_32_8888ARGB:
                format = PAX_BUF_32_8888ARGB;
                break;
            case BSP_DISPLAY_COLOR_FORMAT_18_666RGB:
            default:
                ESP_LOGW(TAG, "BSP requests color format not supported by PAX (%u)", format);
                break;
        }

        // Convert BSP display rotation format into PAX orientation type
        bsp_display_rotation_t display_rotation = bsp_display_get_default_rotation();
        pax_orientation_t      orientation      = PAX_O_UPRIGHT;
        switch (display_rotation) {
            case BSP_DISPLAY_ROTATION_90:
                orientation = PAX_O_ROT_CCW;
                break;
            case BSP_DISPLAY_ROTATION_180:
                orientation = PAX_O_ROT_HALF;
                break;
            case BSP_DISPLAY_ROTATION_270:
                orientation = PAX_O_ROT_CW;
                break;
            case BSP_DISPLAY_ROTATION_0:
            default:
                orientation = PAX_O_UPRIGHT;
                break;
        }

        // Initialize graphics stack
        printf("Initializing framebuffer with w=%d h=%d format=%d endian=%d orientation=%d\n", display_h_res,
               display_v_res, format, display_data_endian, orientation);
        pax_buf_init(&fb, NULL, display_h_res, display_v_res, format);
        pax_buf_reversed(&fb, display_data_endian == BSP_DISPLAY_ENDIAN_BIG);
#if defined(CONFIG_BSP_TARGET_KAMI)
        // Temporary addition for supporting Kami
        fb.palette      = palette;
        fb.palette_size = sizeof(palette) / sizeof(pax_col_t);
#endif
        pax_buf_set_orientation(&fb, orientation);
    } else {
        ESP_LOGI(TAG, "This board has no display support");
    }

    // Get input event queue from BSP
    ESP_ERROR_CHECK(bsp_input_get_queue(&input_event_queue));

    // LEDs
    bsp_led_set_pixel(4, 0x000000);
    bsp_led_set_pixel(5, 0x000000);
    bsp_led_send();
    bsp_led_set_mode(true);

    // Main section of the app

    gpio_num_t gpio_gps_rx = 32;  // RX pin of GPS module, TX from ESP32-P4
    gpio_num_t gpio_gps_tx = 33;  // TX pin of GPS module, RX from ESP32-P4

    const uart_config_t uart_config = {
        .baud_rate = GPS_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    display_gps_data();

    ESP_ERROR_CHECK(uart_param_config(GPS_UART_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(GPS_UART_PORT, gpio_gps_rx, gpio_gps_tx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(GPS_UART_PORT, GPS_BUF_SIZE * 2, 0, 0, NULL, 0));
    xTaskCreate(gps_task, "gps_task", 4096, NULL, 5, NULL);

    while (1) {
        bsp_input_event_t event;
        if (xQueueReceive(input_event_queue, &event, portMAX_DELAY) == pdTRUE) {
            if (event.type == INPUT_EVENT_TYPE_NAVIGATION) {
                if (event.args_navigation.key == BSP_INPUT_NAVIGATION_KEY_F1) {
                    bsp_device_restart_to_launcher();
                }
                if (event.args_navigation.key == BSP_INPUT_NAVIGATION_KEY_F2) {
                    bsp_input_set_backlight_brightness(0);
                }
                if (event.args_navigation.key == BSP_INPUT_NAVIGATION_KEY_F3) {
                    bsp_input_set_backlight_brightness(100);
                }
            }
        }
    }
}
