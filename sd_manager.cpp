// ============================================================
// sd_manager.cpp
// ============================================================
#include "sd_manager.h"
#include <SPI.h>
#include <SD.h>
#include <ArduinoJson.h>

// Dedicated SPIClass instance for the SD card on VSPI. TFT_eSPI owns its
// own VSPI transactions for the LCD/touch; since only one device is ever
// mid-transaction at a time from the single main loop, sharing the bus
// this way is safe as long as every other device's CS line is left HIGH
// whenever it isn't the one being addressed.
static SPIClass s_sdSPI(VSPI);
static bool s_sd_ok = false;
static File s_runFile;
static unsigned long s_runLastRowMs = 0;
static bool s_runOpen = false;
static float s_runLastElapsed = 0;

static String formatSetpointKey(float sp) {
  if (fabs(sp - roundf(sp)) < 0.01f) {
    return String((long)roundf(sp));
  }
  return String(sp, 1);
}

static void deselectOtherCS() {
  pinMode(PIN_LCD_CS, OUTPUT);
  digitalWrite(PIN_LCD_CS, HIGH);
  pinMode(PIN_TOUCH_CS, OUTPUT);
  digitalWrite(PIN_TOUCH_CS, HIGH);
}

static void ensureDir(const char *path) {
  if (!SD.exists(path)) {
    SD.mkdir(path);
  }
}

bool sd_init() {
  deselectOtherCS();

  s_sdSPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_SD_CS);
  s_sd_ok = SD.begin(PIN_SD_CS, s_sdSPI, SD_SPI_SPEED);

  if (!s_sd_ok) {
    Serial.println(F("[sd] SD Card mount FAILED"));
    return false;
  }

  ensureDir(PATH_CONFIG_DIR);
  ensureDir(PATH_DATA_DIR);
  ensureDir(PATH_SUMMARY_DIR);

  Serial.println(F("[sd] SD Card mounted OK"));
  return true;
}

bool sd_is_ok() {
  return s_sd_ok;
}

// ------------------------------------------------------------
// config.json
// ------------------------------------------------------------
bool sd_load_config(RuntimeConfig &cfg) {
  if (!s_sd_ok || !SD.exists(PATH_CONFIG_JSON)) return false;

  File f = SD.open(PATH_CONFIG_JSON, FILE_READ);
  if (!f) return false;

  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, f);
  f.close();
  if (err) {
    Serial.printf("[sd] config.json parse error: %s\n", err.c_str());
    return false;
  }

  cfg.setpoint_aktif   = doc["setpoint_aktif"]   | DEFAULT_SETPOINT;
  cfg.t_shutdown       = doc["t_shutdown"]       | DEFAULT_TSHUTDOWN;
  cfg.target_volume_ml = doc["target_volume_ml"] | DEFAULT_TARGET_VOL;
  cfg.run_count        = doc["run_count"]        | 0;
  cfg.best_ise         = doc["best_ise"]         | 1e12;
  cfg.w1 = doc["w1"] | DEFAULT_W1;
  cfg.w2 = doc["w2"] | DEFAULT_W2;
  cfg.w3 = doc["w3"] | DEFAULT_W3;

  cfg.param_count = 0;
  JsonObject params = doc["params"];
  if (!params.isNull()) {
    for (JsonPair kv : params) {
      if (cfg.param_count >= MAX_PARAM_ENTRIES) break;
      PidParamEntry &e = cfg.params[cfg.param_count];
      e.used = true;
      e.setpoint = atof(kv.key().c_str());
      JsonObject v = kv.value().as<JsonObject>();
      e.kp = v["kp"] | DEFAULT_KP;
      e.ki = v["ki"] | DEFAULT_KI;
      e.kd = v["kd"] | DEFAULT_KD;
      e.runs = v["runs"] | 0;
      cfg.param_count++;
    }
  }

  Serial.println(F("[sd] config.json loaded"));
  return true;
}

bool sd_save_config(const RuntimeConfig &cfg) {
  if (!s_sd_ok) return false;

  JsonDocument doc;
  doc["setpoint_aktif"]   = cfg.setpoint_aktif;
  doc["t_shutdown"]       = cfg.t_shutdown;
  doc["target_volume_ml"] = cfg.target_volume_ml;
  doc["run_count"]        = cfg.run_count;
  doc["best_ise"]         = cfg.best_ise;
  doc["w1"] = cfg.w1;
  doc["w2"] = cfg.w2;
  doc["w3"] = cfg.w3;

  JsonObject params = doc["params"].to<JsonObject>();
  for (uint8_t i = 0; i < cfg.param_count; i++) {
    if (!cfg.params[i].used) continue;
    JsonObject v = params[formatSetpointKey(cfg.params[i].setpoint)].to<JsonObject>();
    v["kp"] = cfg.params[i].kp;
    v["ki"] = cfg.params[i].ki;
    v["kd"] = cfg.params[i].kd;
    v["runs"] = cfg.params[i].runs;
  }

  File f = SD.open(PATH_CONFIG_JSON, FILE_WRITE);
  if (!f) {
    Serial.println(F("[sd] failed to open config.json for write"));
    return false;
  }
  serializeJsonPretty(doc, f);
  f.close();
  return true;
}

// ------------------------------------------------------------
// best_params.json
// ------------------------------------------------------------
bool sd_load_best_params(BestParams &bp) {
  if (!s_sd_ok || !SD.exists(PATH_BEST_PARAMS)) return false;
  File f = SD.open(PATH_BEST_PARAMS, FILE_READ);
  if (!f) return false;

  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, f);
  f.close();
  if (err) return false;

  bp.valid = true;
  bp.setpoint = doc["setpoint"] | DEFAULT_SETPOINT;
  bp.kp = doc["kp"] | DEFAULT_KP;
  bp.ki = doc["ki"] | DEFAULT_KI;
  bp.kd = doc["kd"] | DEFAULT_KD;
  bp.ise = doc["ise"] | 1e12;
  return true;
}

bool sd_save_best_params(const BestParams &bp) {
  if (!s_sd_ok) return false;
  JsonDocument doc;
  doc["setpoint"] = bp.setpoint;
  doc["kp"] = bp.kp;
  doc["ki"] = bp.ki;
  doc["kd"] = bp.kd;
  doc["ise"] = bp.ise;

  File f = SD.open(PATH_BEST_PARAMS, FILE_WRITE);
  if (!f) return false;
  serializeJsonPretty(doc, f);
  f.close();
  return true;
}

// ------------------------------------------------------------
// run_NNN.csv
// ------------------------------------------------------------
static String runPath(uint32_t run_num) {
  char buf[32];
  snprintf(buf, sizeof(buf), "%s/run_%03lu.csv", PATH_DATA_DIR, (unsigned long)run_num);
  return String(buf);
}

bool sd_run_log_start(uint32_t run_num) {
  if (!s_sd_ok) return false;
  String path = runPath(run_num);
  s_runFile = SD.open(path, FILE_WRITE);
  if (!s_runFile) {
    Serial.printf("[sd] failed to open %s\n", path.c_str());
    return false;
  }
  s_runFile.println(F("timestamp_ms,elapsed_s,suhu_c,setpoint_c,fan_pwm,error,kp,ki,kd,volume_ml,efficiency_ml_h,state"));
  s_runOpen = true;
  s_runLastRowMs = millis();
  return true;
}

void sd_run_log_row(unsigned long timestamp_ms, float elapsed_s, float suhu_c, float setpoint_c,
                     uint8_t fan_pwm, float error, float kp, float ki, float kd,
                     float volume_ml, float efficiency_ml_h, const char *state) {
  if (!s_runOpen) return;
  s_runFile.printf("%lu,%.2f,%.2f,%.2f,%u,%.2f,%.4f,%.4f,%.4f,%.2f,%.2f,%s\n",
                    timestamp_ms, elapsed_s, suhu_c, setpoint_c, fan_pwm, error,
                    kp, ki, kd, volume_ml, efficiency_ml_h, state);
}

void sd_run_log_close() {
  if (s_runOpen) {
    s_runFile.flush();
    s_runFile.close();
    s_runOpen = false;
  }
}

bool sd_read_run_csv(uint32_t run_num, RunRowCallback cb) {
  if (!s_sd_ok || !cb) return false;
  String path = runPath(run_num);
  File f = SD.open(path, FILE_READ);
  if (!f) return false;

  bool headerSkipped = false;
  float prevElapsed = 0;
  bool first = true;

  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) continue;
    if (!headerSkipped) { headerSkipped = true; continue; }

    // timestamp_ms,elapsed_s,suhu_c,setpoint_c,fan_pwm,error,kp,ki,kd,volume_ml,efficiency_ml_h,state
    int c1 = line.indexOf(',');
    int c2 = line.indexOf(',', c1 + 1);
    int c3 = line.indexOf(',', c2 + 1);
    int c4 = line.indexOf(',', c3 + 1);
    int c5 = line.indexOf(',', c4 + 1);
    int c6 = line.indexOf(',', c5 + 1);
    float elapsed_s = line.substring(c1 + 1, c2).toFloat();
    float fan_pwm_f = line.substring(c4 + 1, c5).toFloat();
    float error = line.substring(c5 + 1, c6).toFloat();

    float dt = first ? 0.1f : (elapsed_s - prevElapsed);
    if (dt <= 0) dt = 0.1f;
    prevElapsed = elapsed_s;
    first = false;

    cb(elapsed_s, error, (uint8_t)fan_pwm_f, dt);
  }
  f.close();
  return true;
}

// ------------------------------------------------------------
// autotune_log.csv
// ------------------------------------------------------------
void sd_log_autotune_event(unsigned long elapsed_ms, uint8_t cycle, const char *event,
                            float extreme, float peak, float valley) {
  if (!s_sd_ok) return;
  bool isNew = !SD.exists(PATH_AUTOTUNE_LOG);
  File f = SD.open(PATH_AUTOTUNE_LOG, FILE_APPEND);
  if (!f) return;
  if (isNew) {
    f.println(F("elapsed_ms,cycle,event,extreme,peak,valley"));
  }
  f.printf("%lu,%u,%s,%.2f,%.2f,%.2f\n", elapsed_ms, cycle, event, extreme, peak, valley);
  f.close();
}

// ------------------------------------------------------------
// summary/history.csv
// ------------------------------------------------------------
void sd_append_history(uint32_t run_num, float setpoint, float total_vol_ml, unsigned long duration_ms,
                        float efficiency_ml_h, float score, float ise, float avg_pwm,
                        float kp, float ki, float kd) {
  if (!s_sd_ok) return;
  bool isNew = !SD.exists(PATH_HISTORY_CSV);
  File f = SD.open(PATH_HISTORY_CSV, FILE_APPEND);
  if (!f) return;
  if (isNew) {
    f.println(F("run_num,setpoint,total_vol_ml,duration_ms,efficiency_ml_h,score,ise,avg_pwm,kp,ki,kd"));
  }
  f.printf("%lu,%.1f,%.2f,%lu,%.2f,%.2f,%.2f,%.2f,%.4f,%.4f,%.4f\n",
            (unsigned long)run_num, setpoint, total_vol_ml, duration_ms,
            efficiency_ml_h, score, ise, avg_pwm, kp, ki, kd);
  f.close();
}

// ------------------------------------------------------------
// summary/efficiency_map.json
// ------------------------------------------------------------
bool sd_load_efficiency_map(EfficiencyMap &map) {
  map = EfficiencyMap();
  if (!s_sd_ok || !SD.exists(PATH_EFFICIENCY_MAP)) return false;

  File f = SD.open(PATH_EFFICIENCY_MAP, FILE_READ);
  if (!f) return false;

  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, f);
  f.close();
  if (err) return false;

  map.recommended_setpoint = doc["recommended_setpoint"] | 0;
  for (JsonPair kv : doc.as<JsonObject>()) {
    String key = kv.key().c_str();
    if (key == "recommended_setpoint") continue;
    if (map.count >= MAX_EFFICIENCY_ENTRIES) break;

    EfficiencyEntry &e = map.entries[map.count];
    e.used = true;
    e.setpoint = atof(key.c_str());
    JsonObject v = kv.value().as<JsonObject>();
    e.runs = v["runs"] | 0;
    e.avg_efficiency = v["avg_efficiency"] | 0;
    e.avg_score = v["avg_score"] | 0;
    e.avg_ise = v["avg_ise"] | 0;
    map.count++;
  }
  return true;
}

bool sd_save_efficiency_map(const EfficiencyMap &map) {
  if (!s_sd_ok) return false;
  JsonDocument doc;
  for (uint8_t i = 0; i < map.count; i++) {
    if (!map.entries[i].used) continue;
    JsonObject v = doc[formatSetpointKey(map.entries[i].setpoint)].to<JsonObject>();
    v["runs"] = map.entries[i].runs;
    v["avg_efficiency"] = map.entries[i].avg_efficiency;
    v["avg_score"] = map.entries[i].avg_score;
    v["avg_ise"] = map.entries[i].avg_ise;
  }
  doc["recommended_setpoint"] = map.recommended_setpoint;

  File f = SD.open(PATH_EFFICIENCY_MAP, FILE_WRITE);
  if (!f) return false;
  serializeJsonPretty(doc, f);
  f.close();
  return true;
}

String sd_efficiency_map_to_json(const EfficiencyMap &map) {
  JsonDocument doc;
  JsonArray arr = doc["entries"].to<JsonArray>();
  for (uint8_t i = 0; i < map.count; i++) {
    if (!map.entries[i].used) continue;
    JsonObject o = arr.add<JsonObject>();
    o["setpoint"] = map.entries[i].setpoint;
    o["runs"] = map.entries[i].runs;
    o["avg_efficiency"] = map.entries[i].avg_efficiency;
    o["avg_score"] = map.entries[i].avg_score;
    o["avg_ise"] = map.entries[i].avg_ise;
  }
  doc["recommended_setpoint"] = map.recommended_setpoint;

  String out;
  serializeJson(doc, out);
  return out;
}

String sd_read_history_tail(uint16_t maxLines) {
  if (!s_sd_ok || !SD.exists(PATH_HISTORY_CSV)) return String("");

  File f = SD.open(PATH_HISTORY_CSV, FILE_READ);
  if (!f) return String("");

  String header = f.available() ? f.readStringUntil('\n') : String("");

  // Ring buffer of the last maxLines DATA rows (header kept separately so
  // it always survives, even when the file has more rows than the tail).
  const uint16_t CAP = 256;
  if (maxLines > CAP) maxLines = CAP;
  String *lines = new String[maxLines];
  uint16_t count = 0, head = 0;

  while (f.available()) {
    String line = f.readStringUntil('\n');
    if (line.length() == 0) continue;
    lines[head] = line;
    head = (head + 1) % maxLines;
    if (count < maxLines) count++;
  }
  f.close();

  String out = header + "\n";
  uint16_t start = (count < maxLines) ? 0 : head;
  for (uint16_t i = 0; i < count; i++) {
    out += lines[(start + i) % maxLines];
    out += "\n";
  }
  delete[] lines;
  return out;
}
