// ============================================================
// learning_system.h — Scoring, efficiency map, coordinate descent
// ============================================================
#ifndef LEARNING_SYSTEM_H
#define LEARNING_SYSTEM_H

#include <Arduino.h>
#include "config.h"

struct RunResult {
  uint32_t run_num;
  float setpoint;
  float total_vol_ml;
  unsigned long duration_ms;
  float ise;
  float avg_pwm;
  float kp, ki, kd;
};

// score = w1*produktivitas - w2*pwm_avg - w3*ISE_norm
float learning_compute_score(const RunResult &run, float w1, float w2, float w3);
float learning_compute_efficiency_ml_h(float volume_ml, unsigned long duration_ms);

// Runs the full end-of-run learning pipeline:
//   1. Updates efficiency_map.json for run.setpoint
//   2. Coordinate-descent optimizes Kp/Ki/Kd against the run's logged CSV
//   3. Writes the new gains back into cfg.params[] for run.setpoint
//   4. Updates best_params.json if this run improved on best_ise
//   5. Computes efficiency_map recommended_setpoint (if enough runs)
//   6. Appends history.csv
// Returns the (possibly updated) gains via kp/ki/kd out-params.
void learning_process_run(RuntimeConfig &cfg, const RunResult &run,
                           float &out_kp, float &out_ki, float &out_kd);

// Resolves Kp/Ki/Kd for a (possibly new) setpoint:
//   - exact match in cfg.params -> use it directly
//   - no exact match but neighbors on both sides exist -> linear interpolation
//   - no data at all -> DEFAULT_KP/KI/KD
void learning_resolve_gains_for_setpoint(const RuntimeConfig &cfg, float setpoint,
                                          float &kp, float &ki, float &kd);

#endif // LEARNING_SYSTEM_H
