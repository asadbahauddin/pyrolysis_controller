// ============================================================
// autotune.h — Ziegler-Nichols relay method auto-tuner
// ============================================================
#ifndef AUTOTUNE_H
#define AUTOTUNE_H

#include <Arduino.h>

struct AutotuneStatus {
  bool running = false;
  bool finished = false;
  bool timed_out = false;
  uint8_t cycle = 0;          // completed half-cycles / 2
  bool heating = true;        // current relay phase
  float peak = 0;
  float valley = 0;
  float ku = 0;
  float tu = 0;                // ultimate period, seconds
  float result_kp = 0, result_ki = 0, result_kd = 0;
};

void autotune_start(float setpoint);
void autotune_stop();

// Call every loop iteration while state == STATE_AUTOTUNE.
// Returns true once tuning has finished (success or timeout) — caller
// should transition to STATE_RUNNING and apply results afterwards.
bool autotune_update(float current_temp);

const AutotuneStatus &autotune_get_status();

#endif // AUTOTUNE_H
