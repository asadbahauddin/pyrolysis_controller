// ============================================================
// autotune.cpp — Ziegler-Nichols relay (bang-bang) auto-tuner
// ============================================================
#include "autotune.h"
#include "config.h"
#include "actuator_manager.h"
#include "sd_manager.h"

static AutotuneStatus s_status;
static float s_target_setpoint = DEFAULT_SETPOINT;

static float s_phase_extreme = 0;   // running max (heating) / min (cooling) this phase
static float s_prev_temp = 0;
static unsigned long s_prev_ms = 0;
static unsigned long s_phase_start_ms = 0;
static unsigned long s_last_peak_ms = 0;
static unsigned long s_start_ms = 0;

static float s_peak_sum = 0, s_valley_sum = 0;
static uint8_t s_peak_n = 0, s_valley_n = 0;
static float s_period_sum = 0;
static uint8_t s_period_n = 0;

static bool s_have_first_peak = false;

static void switchPhase(bool toHeating, float extremeValue, unsigned long now) {
  if (toHeating) {
    // We just finished a cooling phase; extremeValue is a valley.
    s_status.valley = extremeValue;
    s_valley_sum += extremeValue;
    s_valley_n++;
  } else {
    // We just finished a heating phase; extremeValue is a peak.
    s_status.peak = extremeValue;
    s_peak_sum += extremeValue;
    s_peak_n++;

    if (s_have_first_peak) {
      float period_s = (now - s_last_peak_ms) / 1000.0f;
      s_period_sum += period_s;
      s_period_n++;
    }
    s_have_first_peak = true;
    s_last_peak_ms = now;
    s_status.cycle++;
  }

  sd_log_autotune_event(now - s_start_ms, s_status.cycle, toHeating ? "COOL_TO_HEAT" : "HEAT_TO_COOL",
                         extremeValue, s_status.peak, s_status.valley);

  s_status.heating = toHeating;
  actuator_set_fan_pwm(toHeating ? AT_PWM_HIGH : AT_PWM_LOW);
  s_phase_extreme = extremeValue;
  s_phase_start_ms = now;
}

void autotune_start(float setpoint) {
  s_target_setpoint = setpoint;
  s_status = AutotuneStatus();
  s_status.running = true;
  s_status.finished = false;
  s_status.timed_out = false;
  s_status.heating = true;
  s_status.cycle = 0;

  s_peak_sum = s_valley_sum = s_period_sum = 0;
  s_peak_n = s_valley_n = s_period_n = 0;
  s_have_first_peak = false;

  unsigned long now = millis();
  s_start_ms = now;
  s_phase_start_ms = now;
  s_last_peak_ms = now;
  s_prev_ms = now;
  s_phase_extreme = -1000; // will be overwritten on first sample

  actuator_set_oil_pump(true);
  actuator_set_water_pump(true);
  actuator_set_fan_pwm(AT_PWM_HIGH);

  Serial.println(F("[autotune] Started relay auto-tune"));
}

void autotune_stop() {
  s_status.running = false;
  actuator_set_fan_pwm(0);
}

const AutotuneStatus &autotune_get_status() {
  return s_status;
}

bool autotune_update(float current_temp) {
  if (!s_status.running) return s_status.finished;

  unsigned long now = millis();
  float dt = (now - s_prev_ms) / 1000.0f;
  if (dt <= 0) dt = 0.001f;
  float rate = (current_temp - s_prev_temp) / dt; // deg C / s
  s_prev_temp = current_temp;
  s_prev_ms = now;

  // Safety timeout
  if (now - s_start_ms > AT_TIMEOUT_MS) {
    s_status.running = false;
    s_status.finished = true;
    s_status.timed_out = true;
    actuator_set_fan_pwm(0);
    Serial.println(F("[autotune] TIMEOUT — aborting, keeping previous PID params"));
    return true;
  }

  if (s_status.heating) {
    if (current_temp > s_phase_extreme) s_phase_extreme = current_temp;

    bool fellFromPeak = (s_phase_extreme - current_temp) >= AT_TEMP_BAND;
    bool ratesFalling  = rate <= -AT_STABIL_BAND || fellFromPeak;
    if (fellFromPeak && ratesFalling) {
      switchPhase(false, s_phase_extreme, now); // -> cooling; extreme is peak
      s_phase_extreme = current_temp;
    }
  } else {
    if (current_temp < s_phase_extreme) s_phase_extreme = current_temp;

    bool roseFromValley = (current_temp - s_phase_extreme) >= AT_TEMP_BAND;
    bool ratesRising = rate >= AT_STABIL_BAND || roseFromValley;
    if (roseFromValley && ratesRising) {
      switchPhase(true, s_phase_extreme, now); // -> heating; extreme is valley
      s_phase_extreme = current_temp;
    }
  }

  if (s_status.cycle >= AT_CYCLES && s_peak_n > 0 && s_valley_n > 0 && s_period_n > 0) {
    // Enough oscillations collected — compute Ku, Tu and finish.
    float avg_peak = s_peak_sum / s_peak_n;
    float avg_valley = s_valley_sum / s_valley_n;
    float a = (avg_peak - avg_valley) / 2.0f;      // process oscillation amplitude
    float d = (AT_PWM_HIGH - AT_PWM_LOW) / 2.0f;   // relay amplitude
    float tu = s_period_sum / s_period_n;

    if (a < 0.5f) a = 0.5f; // guard against divide-by-zero on a flat/faulty read

    float ku = (4.0f * d) / (PI * a);

    s_status.ku = ku;
    s_status.tu = tu;
    s_status.result_kp = 0.6f * ku;
    s_status.result_ki = (1.2f * ku) / tu;
    s_status.result_kd = 0.075f * ku * tu;

    s_status.running = false;
    s_status.finished = true;
    actuator_set_fan_pwm(0);

    Serial.printf("[autotune] DONE Ku=%.3f Tu=%.2fs -> Kp=%.3f Ki=%.4f Kd=%.3f\n",
                  ku, tu, s_status.result_kp, s_status.result_ki, s_status.result_kd);
    return true;
  }

  return false;
}
