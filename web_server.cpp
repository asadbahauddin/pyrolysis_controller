// ============================================================
// web_server.cpp
// ============================================================
#include "web_server.h"
#include "web_page.h"
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>

static WebServer s_http(HTTP_PORT);
static WebSocketsServer s_ws(WS_PORT);

#define CMD_QUEUE_CAP 16
static WsCommandResult s_queue[CMD_QUEUE_CAP];
static uint8_t s_qHead = 0, s_qTail = 0, s_qCount = 0;

static void pushCommand(const WsCommandResult &r) {
  if (s_qCount >= CMD_QUEUE_CAP) return; // drop if full — commands are cheap to re-send
  s_queue[s_qTail] = r;
  s_qTail = (s_qTail + 1) % CMD_QUEUE_CAP;
  s_qCount++;
}

bool web_server_has_pending_command() {
  return s_qCount > 0;
}

WsCommandResult web_server_pop_command() {
  if (s_qCount == 0) return WsCommandResult();
  WsCommandResult r = s_queue[s_qHead];
  s_qHead = (s_qHead + 1) % CMD_QUEUE_CAP;
  s_qCount--;
  return r;
}

static void handleRoot() {
  s_http.send_P(200, "text/html", INDEX_HTML);
}

static void parseAndQueue(const char *payload, size_t length) {
  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, payload, length);
  if (err) return;

  const char *cmd = doc["cmd"] | "";
  WsCommandResult r;

  if (!strcmp(cmd, "START")) r.cmd = WS_CMD_START;
  else if (!strcmp(cmd, "STOP")) r.cmd = WS_CMD_STOP;
  else if (!strcmp(cmd, "AUTOTUNE")) r.cmd = WS_CMD_AUTOTUNE;
  else if (!strcmp(cmd, "SET_SETPOINT")) { r.cmd = WS_CMD_SET_SETPOINT; r.value = doc["value"] | 0.0f; }
  else if (!strcmp(cmd, "SET_TSHUTDOWN")) { r.cmd = WS_CMD_SET_TSHUTDOWN; r.value = doc["value"] | 0.0f; }
  else if (!strcmp(cmd, "SET_TARGET_VOL")) { r.cmd = WS_CMD_SET_TARGET_VOL; r.value = doc["value"] | 0.0f; }
  else if (!strcmp(cmd, "SET_KP")) { r.cmd = WS_CMD_SET_KP; r.value = doc["value"] | 0.0f; }
  else if (!strcmp(cmd, "SET_KI")) { r.cmd = WS_CMD_SET_KI; r.value = doc["value"] | 0.0f; }
  else if (!strcmp(cmd, "SET_KD")) { r.cmd = WS_CMD_SET_KD; r.value = doc["value"] | 0.0f; }
  else if (!strcmp(cmd, "SET_WEIGHTS")) {
    r.cmd = WS_CMD_SET_WEIGHTS;
    r.w1 = doc["w1"] | DEFAULT_W1;
    r.w2 = doc["w2"] | DEFAULT_W2;
    r.w3 = doc["w3"] | DEFAULT_W3;
  }
  else if (!strcmp(cmd, "TOGGLE_OIL")) { r.cmd = WS_CMD_TOGGLE_OIL; r.bvalue = doc["value"] | false; }
  else if (!strcmp(cmd, "TOGGLE_WATER")) { r.cmd = WS_CMD_TOGGLE_WATER; r.bvalue = doc["value"] | false; }
  else if (!strcmp(cmd, "TOGGLE_FAN")) { r.cmd = WS_CMD_TOGGLE_FAN; r.bvalue = doc["value"] | false; }
  else if (!strcmp(cmd, "APPLY_BEST")) r.cmd = WS_CMD_APPLY_BEST;
  else if (!strcmp(cmd, "RESET_DEFAULT")) r.cmd = WS_CMD_RESET_DEFAULT;
  else if (!strcmp(cmd, "GET_EFFICIENCY_MAP")) r.cmd = WS_CMD_GET_EFFICIENCY_MAP;
  else if (!strcmp(cmd, "GET_HISTORY")) r.cmd = WS_CMD_GET_HISTORY;
  else if (!strcmp(cmd, "TEST_FAN")) { r.cmd = WS_CMD_TEST_FAN; r.value = doc["value"] | 0.0f; }
  else if (!strcmp(cmd, "TEST_OIL_ON")) r.cmd = WS_CMD_TEST_OIL_ON;
  else if (!strcmp(cmd, "TEST_OIL_OFF")) r.cmd = WS_CMD_TEST_OIL_OFF;
  else if (!strcmp(cmd, "TEST_WATER_ON")) r.cmd = WS_CMD_TEST_WATER_ON;
  else if (!strcmp(cmd, "TEST_WATER_OFF")) r.cmd = WS_CMD_TEST_WATER_OFF;
  else return; // unknown command, ignore

  pushCommand(r);
}

static void onWsEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED:
      Serial.printf("[ws] client #%u connected\n", num);
      break;
    case WStype_DISCONNECTED:
      Serial.printf("[ws] client #%u disconnected\n", num);
      break;
    case WStype_TEXT:
      parseAndQueue((const char *)payload, length);
      break;
    default:
      break;
  }
}

void web_server_init() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(WIFI_SSID, WIFI_PASSWORD);
  Serial.print(F("[web] AP started, IP: "));
  Serial.println(WiFi.softAPIP());

  s_http.on("/", handleRoot);
  s_http.onNotFound(handleRoot); // captive-ish single page app
  s_http.begin();

  s_ws.begin();
  s_ws.onEvent(onWsEvent);

  Serial.printf("[web] HTTP :%d  WebSocket :%d\n", HTTP_PORT, WS_PORT);
}

void web_server_loop() {
  s_http.handleClient();
  s_ws.loop();
}

void web_server_broadcast(const WsBroadcastData &d) {
  JsonDocument doc;

  const char *stateStr = "IDLE";
  switch (d.state) {
    case STATE_IDLE: stateStr = "IDLE"; break;
    case STATE_AUTOTUNE: stateStr = "AUTOTUNE"; break;
    case STATE_RUNNING: stateStr = "RUNNING"; break;
    case STATE_SHUTDOWN: stateStr = "SHUTDOWN"; break;
    case STATE_LEARNING: stateStr = "LEARNING"; break;
  }

  doc["state"] = stateStr;
  doc["suhu"] = d.suhu;
  doc["setpoint"] = d.setpoint;
  doc["fan_pwm"] = d.fan_pwm;
  doc["fan_pct"] = d.fan_pct;
  doc["error"] = d.error;
  doc["kp"] = d.kp;
  doc["ki"] = d.ki;
  doc["kd"] = d.kd;
  doc["volume_ml"] = d.volume_ml;
  doc["target_vol"] = d.target_vol;
  doc["efficiency"] = d.efficiency;
  doc["duration_ms"] = d.duration_ms;
  doc["est_finish_ms"] = d.est_finish_ms;
  doc["ise"] = d.ise;
  doc["run_count"] = d.run_count;
  doc["oil_pump"] = d.oil_pump;
  doc["water_pump"] = d.water_pump;
  doc["fan_enabled"] = d.fan_enabled;
  doc["at_cycle"] = d.at_cycle;
  doc["at_heating"] = d.at_heating;
  doc["at_peak"] = d.at_peak;
  doc["at_valley"] = d.at_valley;
  doc["sd_ok"] = d.sd_ok;
  doc["best_ise"] = d.best_ise;
  doc["recommended_sp"] = d.recommended_sp;
  doc["heap_free"] = d.heap_free;

  char buf[640];
  size_t n = serializeJson(doc, buf, sizeof(buf));
  s_ws.broadcastTXT(buf, n);
}

void web_server_broadcast_emergency(float suhu) {
  JsonDocument doc;
  doc["event"] = "emergency";
  doc["suhu"] = suhu;
  char buf[96];
  size_t n = serializeJson(doc, buf, sizeof(buf));
  s_ws.broadcastTXT(buf, n);
}

void web_server_send_efficiency_map_json(const String &entriesJson) {
  // entriesJson already looks like {"entries":[...],"recommended_setpoint":N}
  // Splice in a "type" field without a full re-parse/re-serialize round trip.
  String out = "{\"type\":\"efficiency_map\",";
  int firstBrace = entriesJson.indexOf('{');
  out += entriesJson.substring(firstBrace + 1);
  s_ws.broadcastTXT(out);
}

void web_server_send_history_csv(const String &csvText) {
  JsonDocument doc;
  doc["type"] = "history";
  doc["csv"] = csvText;
  String out;
  serializeJson(doc, out);
  s_ws.broadcastTXT(out);
}
