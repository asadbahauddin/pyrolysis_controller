// ============================================================
// pid_controller.h — PID with anti-windup + 3-zone gain scheduling
// ============================================================
#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <Arduino.h>

enum PidZone { ZONE_COLD = 0, ZONE_MID, ZONE_HOT };

class PidController {
public:
  PidController();

  // Base parameters (as loaded/interpolated for the active setpoint)
  void setBaseGains(float kp, float ki, float kd);
  void getBaseGains(float &kp, float &ki, float &kd) const;

  void reset();

  // Computes one PID step. dt in seconds. Applies gain scheduling
  // internally based on current temperature, then anti-windup clamped
  // integral, and returns a PWM value constrained to [0,255].
  uint8_t compute(float setpoint, float measurement, float dt);

  float getError() const { return _error; }
  float getIntegral() const { return _integral; }
  float getISEAccum() const { return _ise_accum; }
  void resetISE() { _ise_accum = 0; }

  PidZone getZone() const { return _zone; }
  static PidZone zoneForTemperature(float temp_c);
  static void scheduledGains(float base_kp, float base_ki, float base_kd,
                             PidZone zone, float &kp, float &ki, float &kd);

private:
  float _kp, _ki, _kd;
  float _integral;
  float _error;
  float _error_prev;
  float _ise_accum;
  bool  _first_step;
  PidZone _zone;
};

#endif // PID_CONTROLLER_H
