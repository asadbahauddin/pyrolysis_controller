// ============================================================
// actuator_manager.cpp
// ============================================================
#include "actuator_manager.h"
#include "config.h"

static uint8_t s_fan_pwm = 0;
static bool s_fan_enabled = true; // web toggle: allows/blocks PWM output
static bool s_oil_pump = false;
static bool s_water_pump = false;

void actuator_init() {
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(PIN_FAN_PWM, PWM_CHANNEL);
  ledcWrite(PWM_CHANNEL, 0);

  pinMode(PIN_OIL_PUMP, OUTPUT);
  pinMode(PIN_WATER_PUMP, OUTPUT);
  digitalWrite(PIN_OIL_PUMP, LOW);
  digitalWrite(PIN_WATER_PUMP, LOW);

  s_fan_pwm = 0;
  s_fan_enabled = true;
  s_oil_pump = false;
  s_water_pump = false;

  Serial.println(F("[actuator] Fan PWM + relays initialized"));
}

void actuator_set_fan_pwm(uint8_t pwm) {
  s_fan_pwm = pwm;
  ledcWrite(PWM_CHANNEL, s_fan_enabled ? s_fan_pwm : 0);
}

uint8_t actuator_get_fan_pwm() {
  return s_fan_pwm;
}

uint8_t actuator_get_fan_pct() {
  return (uint8_t)((s_fan_pwm * 100UL) / 255UL);
}

void actuator_set_fan_enabled(bool en) {
  s_fan_enabled = en;
  ledcWrite(PWM_CHANNEL, s_fan_enabled ? s_fan_pwm : 0);
}

bool actuator_get_fan_enabled() {
  return s_fan_enabled;
}

void actuator_set_oil_pump(bool on) {
  s_oil_pump = on;
  digitalWrite(PIN_OIL_PUMP, on ? HIGH : LOW);
}

bool actuator_get_oil_pump() {
  return s_oil_pump;
}

void actuator_set_water_pump(bool on) {
  s_water_pump = on;
  digitalWrite(PIN_WATER_PUMP, on ? HIGH : LOW);
}

bool actuator_get_water_pump() {
  return s_water_pump;
}

void actuator_all_off_except_water() {
  s_fan_pwm = 0;
  ledcWrite(PWM_CHANNEL, 0);
  actuator_set_oil_pump(false);
}

void actuator_emergency_stop() {
  s_fan_pwm = 0;
  ledcWrite(PWM_CHANNEL, 0);
  actuator_set_oil_pump(false);
  actuator_set_water_pump(true); // keep cooling
}
