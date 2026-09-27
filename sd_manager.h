// ============================================================
// sd_manager.h — SD Card mount, config persistence, CSV/JSON logs
// ============================================================
#ifndef SD_MANAGER_H
#define SD_MANAGER_H

#include <Arduino.h>
#include "config.h"

bool sd_init();
bool sd_is_ok();

// ---- config.json ----
bool sd_load_config(RuntimeConfig &cfg);
bool sd_save_config(const RuntimeConfig &cfg);

// ---- best_params.json ----
struct BestParams {
  bool valid = false;
  float setpoint = DEFAULT_SETPOINT;
  float kp = DEFAULT_KP;
  float ki = DEFAULT_KI;
  float kd = DEFAULT_KD;
  float ise = 1e12;
};
bool sd_load_best_params(BestParams &bp);
bool sd_save_best_params(const BestParams &bp);

// ---- run_NNN.csv ----
bool sd_run_log_start(uint32_t run_num);
void sd_run_log_row(unsigned long timestamp_ms, float elapsed_s, float suhu_c, float setpoint_c,
                     uint8_t fan_pwm, float error, float kp, float ki, float kd,
                     float volume_ml, float efficiency_ml_h, const char *state);
void sd_run_log_close();

// Row-by-row streaming reader for coordinate descent replay (avoids
// loading a whole run's CSV into RAM). Callback receives elapsed_s,
// error, fan_pwm and dt (seconds) between this row and the previous one.
typedef void (*RunRowCallback)(float elapsed_s, float error, uint8_t fan_pwm, float dt);
bool sd_read_run_csv(uint32_t run_num, RunRowCallback cb);

// ---- autotune_log.csv ----
void sd_log_autotune_event(unsigned long elapsed_ms, uint8_t cycle, const char *event,
                            float extreme, float peak, float valley);

// ---- summary/history.csv ----
void sd_append_history(uint32_t run_num, float setpoint, float total_vol_ml, unsigned long duration_ms,
                        float efficiency_ml_h, float score, float ise, float avg_pwm,
                        float kp, float ki, float kd);

// ---- summary/efficiency_map.json ----
struct EfficiencyEntry {
  bool used = false;
  float setpoint = 0;
  uint16_t runs = 0;
  float avg_efficiency = 0;
  float avg_score = 0;
  float avg_ise = 0;
};
#define MAX_EFFICIENCY_ENTRIES 32
struct EfficiencyMap {
  EfficiencyEntry entries[MAX_EFFICIENCY_ENTRIES];
  uint8_t count = 0;
  float recommended_setpoint = 0;
};
bool sd_load_efficiency_map(EfficiencyMap &map);
bool sd_save_efficiency_map(const EfficiencyMap &map);

// Builds a JSON string of the efficiency map for WebSocket delivery.
String sd_efficiency_map_to_json(const EfficiencyMap &map);

// Returns up to maxLines of the most recent history.csv rows as raw text
// (for the web "Data Log" / history export features).
String sd_read_history_tail(uint16_t maxLines);

#endif // SD_MANAGER_H
