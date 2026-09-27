// ============================================================
// sensor_manager.h — MAX6675 thermocouple + tipping bucket
// ============================================================
#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include <Arduino.h>

void sensor_init();

// Reads MAX6675 over HSPI. Returns last good value on transient
// read glitches (NaN / disconnected code) so PID never sees garbage.
float sensor_read_temperature();

// Tipping bucket: atomic counter access
void IRAM_ATTR sensor_tip_isr();
unsigned long sensor_get_tip_count();
void sensor_reset_tip_count();

// Derived volume (mL) from tip count
float sensor_get_volume_ml();

// True if thermocouple is disconnected/faulty (MAX6675 bit D2)
bool sensor_is_fault();

#endif // SENSOR_MANAGER_H
