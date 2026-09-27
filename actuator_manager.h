// ============================================================
// actuator_manager.h — Fan PWM (MOSFET) + relay control
// ============================================================
#ifndef ACTUATOR_MANAGER_H
#define ACTUATOR_MANAGER_H

#include <Arduino.h>

void actuator_init();

// Fan (0-255 PWM). Also updates the "fan_enabled" toggle state.
void actuator_set_fan_pwm(uint8_t pwm);
uint8_t actuator_get_fan_pwm();
uint8_t actuator_get_fan_pct();

void actuator_set_fan_enabled(bool en);
bool actuator_get_fan_enabled();

void actuator_set_oil_pump(bool on);
bool actuator_get_oil_pump();

void actuator_set_water_pump(bool on);
bool actuator_get_water_pump();

// Immediate hard stop for emergency/shutdown paths.
void actuator_all_off_except_water();
void actuator_emergency_stop();

#endif // ACTUATOR_MANAGER_H
