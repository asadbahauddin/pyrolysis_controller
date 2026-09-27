// ============================================================
// web_server.h — HTTP (dashboard) + WebSocket (control protocol)
// ============================================================
#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include <Arduino.h>
#include "config.h"

struct WsBroadcastData {
  SystemState state = STATE_IDLE;
  float suhu = 0, setpoint = 0;
  uint8_t fan_pwm = 0, fan_pct = 0;
  float error = 0;
  float kp = 0, ki = 0, kd = 0;
  float volume_ml = 0, target_vol = 0;
  float efficiency = 0;
  unsigned long duration_ms = 0;
  long est_finish_ms = -1;
  float ise = 0;
  uint32_t run_count = 0;
  bool oil_pump = false, water_pump = false, fan_enabled = true;
  uint8_t at_cycle = 0;
  bool at_heating = false;
  float at_peak = 0, at_valley = 0;
  bool sd_ok = false;
  float best_ise = 0;
  float recommended_sp = 0;
  uint32_t heap_free = 0;
};

enum WsCommand {
  WS_CMD_NONE = 0,
  WS_CMD_START,
  WS_CMD_STOP,
  WS_CMD_AUTOTUNE,
  WS_CMD_SET_SETPOINT,
  WS_CMD_SET_TSHUTDOWN,
  WS_CMD_SET_TARGET_VOL,
  WS_CMD_SET_KP,
  WS_CMD_SET_KI,
  WS_CMD_SET_KD,
  WS_CMD_SET_WEIGHTS,
  WS_CMD_TOGGLE_OIL,
  WS_CMD_TOGGLE_WATER,
  WS_CMD_TOGGLE_FAN,
  WS_CMD_APPLY_BEST,
  WS_CMD_RESET_DEFAULT,
  WS_CMD_GET_EFFICIENCY_MAP,
  WS_CMD_GET_HISTORY,
  WS_CMD_TEST_FAN,
  WS_CMD_TEST_OIL_ON,
  WS_CMD_TEST_OIL_OFF,
  WS_CMD_TEST_WATER_ON,
  WS_CMD_TEST_WATER_OFF
};

struct WsCommandResult {
  WsCommand cmd = WS_CMD_NONE;
  float value = 0;
  bool bvalue = false;
  float w1 = 0, w2 = 0, w3 = 0;
};

void web_server_init();
void web_server_loop();

void web_server_broadcast(const WsBroadcastData &d);
void web_server_broadcast_emergency(float suhu);

// Fixed-size command queue (browser -> ESP32). Push happens from the
// WebSocket event callback; main.ino drains it once per loop iteration.
bool web_server_has_pending_command();
WsCommandResult web_server_pop_command();

// Replies to GET_EFFICIENCY_MAP / GET_HISTORY (broadcast to all clients;
// single-AP-client use case makes this simple and sufficient).
void web_server_send_efficiency_map_json(const String &entriesJson);
void web_server_send_history_csv(const String &csvText);

#endif // WEB_SERVER_H
