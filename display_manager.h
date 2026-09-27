// ============================================================
// display_manager.h — ILI9341 + XPT2046 touch, 4-page dashboard
// ============================================================
#ifndef DISPLAY_MANAGER_H
#define DISPLAY_MANAGER_H

#include <Arduino.h>
#include "config.h"

enum LcdPage {
  PAGE_STATUS = 0,
  PAGE_PID_SETTINGS,
  PAGE_EFFICIENCY,
  PAGE_TEST_MODE,
  PAGE_COUNT
};

// Snapshot of everything the display needs to render. Filled by main.ino
// each cycle from the live system state.
struct LcdData {
  SystemState state;
  float suhu;
  float setpoint;
  uint8_t fan_pct;
  float volume_ml;
  float efficiency_ml_h;
  unsigned long duration_ms;
  float ise;
  uint32_t run_count;

  float kp, ki, kd;
  float t_shutdown;
  float target_volume_ml;
  float w1, w2, w3;

  bool oil_pump, water_pump, fan_enabled;

  // Efficiency map (for PAGE_EFFICIENCY) — pre-flattened for display
  uint8_t eff_count;
  float eff_setpoint[8];
  uint16_t eff_runs[8];
  float eff_score[8];
  float eff_efficiency[8];
  float recommended_setpoint;

  bool sd_ok;
};

// Commands the touch UI can raise back to main.ino (polled once per loop).
enum LcdCommand {
  LCD_CMD_NONE = 0,
  LCD_CMD_START,
  LCD_CMD_STOP,
  LCD_CMD_AUTOTUNE,
  LCD_CMD_TEST_FAN,      // value = pwm 0-255
  LCD_CMD_TEST_OIL_ON,
  LCD_CMD_TEST_OIL_OFF,
  LCD_CMD_TEST_WATER_ON,
  LCD_CMD_TEST_WATER_OFF,
  LCD_CMD_SET_SETPOINT,   // value = new setpoint (delta already applied)
  LCD_CMD_SET_TSHUTDOWN,  // value = new t_shutdown
  LCD_CMD_SET_TARGET_VOL  // value = new target_volume_ml
};

struct LcdCommandResult {
  LcdCommand cmd = LCD_CMD_NONE;
  float value = 0;
};

void display_init();
void display_update(const LcdData &data);   // non-blocking, partial redraw
void display_poll_touch();                  // call every loop; handles nav + widgets
LcdCommandResult display_get_pending_command(); // consumes and clears pending command

#endif // DISPLAY_MANAGER_H
