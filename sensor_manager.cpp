// ============================================================
// sensor_manager.cpp
// ============================================================
#include "sensor_manager.h"
#include "config.h"
#include <max6675.h>

static MAX6675 s_thermo(PIN_MAX6675_SCK, PIN_MAX6675_CS, PIN_MAX6675_MISO);

static float s_last_good_temp = 25.0;
static bool  s_fault = false;

static volatile unsigned long s_tip_count = 0;
static unsigned long s_last_isr_ms = 0;
#define TIP_DEBOUNCE_MS 15

void sensor_init() {
  // GPIO34 is input-only and has no internal pull resistor on the ESP32,
  // so INPUT_PULLUP would be silently ignored here. Hall sensor modules
  // (e.g. A3144) normally carry their own onboard pull-up already.
  pinMode(PIN_TIPPING, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_TIPPING), sensor_tip_isr, RISING);

  // Let the MAX6675 settle before first conversion read.
  delay(250);
  Serial.println(F("[sensor] MAX6675 + tipping bucket initialized"));
}

float sensor_read_temperature() {
  float c = s_thermo.readCelsius();

  if (isnan(c) || c <= 0.0f || c > 1024.0f) {
    s_fault = true;
    return s_last_good_temp;
  }

  s_fault = false;
  s_last_good_temp = c;
  return c;
}

bool sensor_is_fault() {
  return s_fault;
}

void IRAM_ATTR sensor_tip_isr() {
  unsigned long now = millis();
  if (now - s_last_isr_ms >= TIP_DEBOUNCE_MS) {
    s_tip_count++;
    s_last_isr_ms = now;
  }
}

unsigned long sensor_get_tip_count() {
  noInterrupts();
  unsigned long count = s_tip_count;
  interrupts();
  return count;
}

void sensor_reset_tip_count() {
  noInterrupts();
  s_tip_count = 0;
  interrupts();
}

float sensor_get_volume_ml() {
  return (float)sensor_get_tip_count() * VOLUME_PER_TIP_ML;
}
