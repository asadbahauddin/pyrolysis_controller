// ============================================================
// pid_controller.cpp
// ============================================================
#include "pid_controller.h"
#include "config.h"

PidController::PidController() {
  _kp = DEFAULT_KP;
  _ki = DEFAULT_KI;
  _kd = DEFAULT_KD;
  reset();
}

void PidController::setBaseGains(float kp, float ki, float kd) {
  _kp = kp;
  _ki = ki;
  _kd = kd;
}

void PidController::getBaseGains(float &kp, float &ki, float &kd) const {
  kp = _kp;
  ki = _ki;
  kd = _kd;
}

void PidController::reset() {
  _integral = 0;
  _error = 0;
  _error_prev = 0;
  _ise_accum = 0;
  _first_step = true;
  _zone = ZONE_MID;
}

PidZone PidController::zoneForTemperature(float temp_c) {
  if (temp_c < ZONE_COLD_MAX) return ZONE_COLD;
  if (temp_c <= ZONE_MID_MAX) return ZONE_MID;
  return ZONE_HOT;
}

void PidController::scheduledGains(float base_kp, float base_ki, float base_kd,
                                    PidZone zone, float &kp, float &ki, float &kd) {
  float mult;
  switch (zone) {
    case ZONE_COLD: mult = ZONE_COLD_KP_MULT; break;
    case ZONE_HOT:  mult = ZONE_HOT_KP_MULT;  break;
    default:        mult = ZONE_MID_KP_MULT;  break;
  }
  kp = base_kp * mult;
  ki = base_ki; // Ki/Kd left as scheduled by setpoint learning; only Kp scheduled by zone
  kd = base_kd;
}

uint8_t PidController::compute(float setpoint, float measurement, float dt) {
  if (dt <= 0) dt = 0.001f;

  _zone = zoneForTemperature(measurement);
  float kp, ki, kd;
  scheduledGains(_kp, _ki, _kd, _zone, kp, ki, kd);

  _error = setpoint - measurement;

  // Anti-windup: clamp integral BEFORE using it
  _integral += ki * _error * dt;
  if (_integral > I_MAX) _integral = I_MAX;
  if (_integral < I_MIN) _integral = I_MIN;

  float derivative = 0;
  if (!_first_step) {
    derivative = kd * (_error - _error_prev) / dt;
  }
  _first_step = false;

  float output = kp * _error + _integral + derivative;
  output = constrain(output, 0.0f, 255.0f);

  _error_prev = _error;
  _ise_accum += (_error * _error) * dt;

  return (uint8_t)output;
}
