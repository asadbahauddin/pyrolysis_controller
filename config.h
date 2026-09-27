// ============================================================
// config.h — Pin map, tuning constants, shared types
// ============================================================
#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ------------------------------------------------------------
// PIN ASSIGNMENT
// ------------------------------------------------------------

// HSPI — MAX6675 (dedicated bus, no conflict with VSPI devices)
#define PIN_MAX6675_SCK   14
#define PIN_MAX6675_MISO   2
#define PIN_MAX6675_CS     5

// VSPI — SD Card + ILI9341 + XPT2046 (shared bus)
#define PIN_SPI_SCK       18
#define PIN_SPI_MISO      19
#define PIN_SPI_MOSI      23
#define PIN_SD_CS          4
#define PIN_LCD_CS        33
#define PIN_LCD_DC        21
#define PIN_LCD_RST       22
#define PIN_TOUCH_CS      13
#define PIN_TOUCH_IRQ     35

// Output aktuator
#define PIN_FAN_PWM       25  // MOSFET IRLB8721
#define PIN_OIL_PUMP      26  // Relay via PC817+BD139
#define PIN_WATER_PUMP    27  // Relay via PC817+BD139

// Input sensor
#define PIN_TIPPING       34  // Hall sensor, interrupt RISING

// PWM (LEDC)
#define PWM_CHANNEL        0
#define PWM_FREQ        5000
#define PWM_RESOLUTION     8  // 0-255

// SPI clock speeds
#define SD_SPI_SPEED    25000000UL
#define LCD_SPI_SPEED   40000000UL
#define TOUCH_SPI_SPEED  2500000UL

// ------------------------------------------------------------
// WIFI / SERVER
// ------------------------------------------------------------
#define WIFI_SSID       "Pyrolysis"
#define WIFI_PASSWORD   "12345678"
#define HTTP_PORT       80
#define WS_PORT         81

// ------------------------------------------------------------
// KALIBRASI & KONSTANTA
// ------------------------------------------------------------
#define VOLUME_PER_TIP_ML    1.42
#define COLLECTOR_AREA_CM2   19.25
#define MM_PER_TIP           0.70

#define TEMP_EMERGENCY       800.0

#define AT_PWM_HIGH          200
#define AT_PWM_LOW            0
#define AT_CYCLES             4
#define AT_TEMP_BAND          5.0
#define AT_STABIL_BAND        2.0
#define AT_TIMEOUT_MS         3600000UL

#define I_MAX                255.0
#define I_MIN               -255.0

#define LOOP_INTERVAL_MS      100
#define LOG_INTERVAL_MS       1000
#define LCD_INTERVAL_MS       500
#define WS_INTERVAL_MS        200

#define MIN_RUNS_RECOMMEND     3
#define CD_STEP_RATIO          0.05

#define DEFAULT_KP            2.0
#define DEFAULT_KI            0.1
#define DEFAULT_KD            10.0
#define DEFAULT_SETPOINT      300.0
#define DEFAULT_TSHUTDOWN     80.0
#define DEFAULT_TARGET_VOL    1000.0

#define DEFAULT_W1            0.6
#define DEFAULT_W2            0.2
#define DEFAULT_W3            0.2

// Gain scheduling zone boundaries (deg C)
#define ZONE_COLD_MAX         200.0
#define ZONE_MID_MAX          400.0

// Zone multipliers applied on top of the active setpoint's base Kp/Ki/Kd
#define ZONE_COLD_KP_MULT     1.3
#define ZONE_MID_KP_MULT      1.0
#define ZONE_HOT_KP_MULT      0.7

#define HEAP_WARN_THRESHOLD   10000

// ------------------------------------------------------------
// SD CARD PATHS
// ------------------------------------------------------------
#define PATH_CONFIG_DIR       "/config"
#define PATH_CONFIG_JSON      "/config/config.json"
#define PATH_BEST_PARAMS      "/config/best_params.json"
#define PATH_DATA_DIR         "/data"
#define PATH_AUTOTUNE_LOG     "/data/autotune_log.csv"
#define PATH_SUMMARY_DIR      "/summary"
#define PATH_EFFICIENCY_MAP   "/summary/efficiency_map.json"
#define PATH_HISTORY_CSV      "/summary/history.csv"

// ------------------------------------------------------------
// SHARED TYPES
// ------------------------------------------------------------
enum SystemState {
  STATE_IDLE = 0,
  STATE_AUTOTUNE,
  STATE_RUNNING,
  STATE_SHUTDOWN,
  STATE_LEARNING
};

// Per-setpoint PID parameter record
struct PidParamEntry {
  bool   used = false;
  float  setpoint = 0;
  float  kp = DEFAULT_KP;
  float  ki = DEFAULT_KI;
  float  kd = DEFAULT_KD;
  uint16_t runs = 0;
};

#define MAX_PARAM_ENTRIES 32

// Global runtime configuration (mirrors config.json)
struct RuntimeConfig {
  float setpoint_aktif   = DEFAULT_SETPOINT;
  float t_shutdown       = DEFAULT_TSHUTDOWN;
  float target_volume_ml = DEFAULT_TARGET_VOL;
  uint32_t run_count     = 0;
  float best_ise         = 1e12;
  float w1 = DEFAULT_W1, w2 = DEFAULT_W2, w3 = DEFAULT_W3;

  PidParamEntry params[MAX_PARAM_ENTRIES];
  uint8_t param_count = 0;

  // Active (possibly interpolated) PID values in use right now
  float active_kp = DEFAULT_KP;
  float active_ki = DEFAULT_KI;
  float active_kd = DEFAULT_KD;
};

#endif // CONFIG_H
