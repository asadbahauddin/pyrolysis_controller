// ============================================================
// pyrolysis_controller.ino — main sketch / state machine
// ============================================================
#include "config.h"
#include "sensor_manager.h"
#include "actuator_manager.h"
#include "sd_manager.h"
#include "pid_controller.h"
#include "autotune.h"
#include "learning_system.h"
#include "display_manager.h"
#include "web_server.h"

// ------------------------------------------------------------
// Global state
// ------------------------------------------------------------
static RuntimeConfig cfg;
static PidController pid;
static SystemState state = STATE_IDLE;

static uint32_t currentRunNumber = 0;
static unsigned long runStartMs = 0;
static unsigned long shutdownStartMs = 0;
static unsigned long learningEnterMs = 0;
static bool skipLearningAfterShutdown = false; // true for an autotune-abort cool-down (no run data to learn from)

static float runPwmSum = 0;
static uint32_t runPwmSamples = 0;

// Final stats captured the instant a run stops heating (used by LEARNING).
static float finalVolumeMl = 0, finalIse = 0, finalAvgPwm = 0;
static float finalKp = DEFAULT_KP, finalKi = DEFAULT_KI, finalKd = DEFAULT_KD;
static unsigned long finalDurationMs = 0;

// Live telemetry, refreshed continuously and consumed by LCD/WS builders.
static float g_suhu = 25.0f;
static uint8_t g_fan_pwm = 0, g_fan_pct = 0;
static float g_error = 0;
static float g_volume_ml = 0;
static float g_efficiency = 0;
static unsigned long g_duration_ms = 0;
static float g_ise = 0;
static float g_kp = DEFAULT_KP, g_ki = DEFAULT_KI, g_kd = DEFAULT_KD;
static float g_recommended_sp = DEFAULT_SETPOINT;
static bool g_sd_ok = false;

static unsigned long lastPidMs = 0;
static unsigned long lastLogMs = 0;
static unsigned long lastLcdMs = 0;
static unsigned long lastWsMs = 0;
static unsigned long lastHeapWarnMs = 0;

// ------------------------------------------------------------
// Helpers shared by web + LCD command handlers
// ------------------------------------------------------------
static void applySetpoint(float sp) {
  cfg.setpoint_aktif = sp;
  learning_resolve_gains_for_setpoint(cfg, sp, g_kp, g_ki, g_kd);
  pid.setBaseGains(g_kp, g_ki, g_kd);
  cfg.active_kp = g_kp; cfg.active_ki = g_ki; cfg.active_kd = g_kd;
  sd_save_config(cfg);
  Serial.printf("[main] Setpoint -> %.1f (Kp=%.3f Ki=%.4f Kd=%.3f)\n", sp, g_kp, g_ki, g_kd);
}

static void applyTShutdown(float v) { cfg.t_shutdown = v; sd_save_config(cfg); }
static void applyTargetVol(float v) { cfg.target_volume_ml = v; sd_save_config(cfg); }

static PidParamEntry *currentSetpointEntry() {
  for (uint8_t i = 0; i < cfg.param_count; i++) {
    if (cfg.params[i].used && fabs(cfg.params[i].setpoint - cfg.setpoint_aktif) < 0.01f) return &cfg.params[i];
  }
  if (cfg.param_count < MAX_PARAM_ENTRIES) {
    PidParamEntry &e = cfg.params[cfg.param_count];
    e = PidParamEntry();
    e.used = true;
    e.setpoint = cfg.setpoint_aktif;
    cfg.param_count++;
    return &e;
  }
  return nullptr;
}

static void applyKp(float v) {
  g_kp = v; cfg.active_kp = v; pid.setBaseGains(g_kp, g_ki, g_kd);
  PidParamEntry *e = currentSetpointEntry(); if (e) e->kp = v;
  sd_save_config(cfg);
}
static void applyKi(float v) {
  g_ki = v; cfg.active_ki = v; pid.setBaseGains(g_kp, g_ki, g_kd);
  PidParamEntry *e = currentSetpointEntry(); if (e) e->ki = v;
  sd_save_config(cfg);
}
static void applyKd(float v) {
  g_kd = v; cfg.active_kd = v; pid.setBaseGains(g_kp, g_ki, g_kd);
  PidParamEntry *e = currentSetpointEntry(); if (e) e->kd = v;
  sd_save_config(cfg);
}

static void applyWeights(float w1, float w2, float w3) {
  float sum = w1 + w2 + w3;
  if (sum <= 0.0001f) sum = 1.0f;
  cfg.w1 = w1 / sum; cfg.w2 = w2 / sum; cfg.w3 = w3 / sum;
  sd_save_config(cfg);
}

static void doApplyBest() {
  BestParams bp;
  if (sd_load_best_params(bp) && bp.valid) {
    g_kp = bp.kp; g_ki = bp.ki; g_kd = bp.kd;
    cfg.active_kp = g_kp; cfg.active_ki = g_ki; cfg.active_kd = g_kd;
    pid.setBaseGains(g_kp, g_ki, g_kd);
    sd_save_config(cfg);
    Serial.println(F("[main] Applied best-ever PID params"));
  }
}

static void doResetDefault() {
  g_kp = DEFAULT_KP; g_ki = DEFAULT_KI; g_kd = DEFAULT_KD;
  cfg.active_kp = g_kp; cfg.active_ki = g_ki; cfg.active_kd = g_kd;
  pid.setBaseGains(g_kp, g_ki, g_kd);
  sd_save_config(cfg);
}

static void beginShutdown() {
  sd_run_log_close();
  actuator_set_fan_pwm(0);
  actuator_set_oil_pump(false);
  actuator_set_water_pump(true);
  skipLearningAfterShutdown = false;

  finalVolumeMl = g_volume_ml;
  finalDurationMs = g_duration_ms;
  finalIse = pid.getISEAccum();
  finalAvgPwm = (runPwmSamples > 0) ? (runPwmSum / runPwmSamples) : 0;
  finalKp = g_kp; finalKi = g_ki; finalKd = g_kd;

  state = STATE_SHUTDOWN;
  shutdownStartMs = millis();
  Serial.println(F("[main] -> SHUTDOWN"));
}

static void beginRun() {
  currentRunNumber = cfg.run_count + 1;
  pid.reset();
  sensor_reset_tip_count();
  sd_run_log_start(currentRunNumber);

  actuator_set_oil_pump(true);
  actuator_set_water_pump(true);
  actuator_set_fan_enabled(true);

  runStartMs = millis();
  runPwmSum = 0; runPwmSamples = 0;
  g_volume_ml = 0; g_duration_ms = 0; g_ise = 0;

  state = STATE_RUNNING;
  Serial.printf("[main] -> RUNNING (run #%lu, setpoint=%.1f)\n", (unsigned long)currentRunNumber, cfg.setpoint_aktif);
}

static bool checkEmergency() {
  if (g_suhu < TEMP_EMERGENCY) return false;

  Serial.println(F("[main] !!! EMERGENCY: over-temperature !!!"));
  bool wasRunning = (state == STATE_RUNNING);
  if (state == STATE_AUTOTUNE) autotune_stop();
  sd_run_log_close();
  actuator_emergency_stop();
  web_server_broadcast_emergency(g_suhu);
  // Only feed the learning pipeline if a real production run was in
  // progress — an emergency during AUTOTUNE has no meaningful run data.
  skipLearningAfterShutdown = !wasRunning;

  finalVolumeMl = g_volume_ml;
  finalDurationMs = g_duration_ms;
  finalIse = pid.getISEAccum();
  finalAvgPwm = (runPwmSamples > 0) ? (runPwmSum / runPwmSamples) : 0;
  finalKp = g_kp; finalKi = g_ki; finalKd = g_kd;

  state = STATE_SHUTDOWN;
  shutdownStartMs = millis();
  return true;
}

// ------------------------------------------------------------
// Command processing (web + LCD both funnel into these)
// ------------------------------------------------------------
static void processWebCommand(const WsCommandResult &r) {
  switch (r.cmd) {
    case WS_CMD_START: if (state == STATE_IDLE) beginRun(); break;
    case WS_CMD_STOP: if (state == STATE_RUNNING) beginShutdown(); break;
    case WS_CMD_AUTOTUNE: if (state == STATE_IDLE) { autotune_start(cfg.setpoint_aktif); state = STATE_AUTOTUNE; } break;
    case WS_CMD_SET_SETPOINT: applySetpoint(r.value); break;
    case WS_CMD_SET_TSHUTDOWN: applyTShutdown(r.value); break;
    case WS_CMD_SET_TARGET_VOL: applyTargetVol(r.value); break;
    case WS_CMD_SET_KP: applyKp(r.value); break;
    case WS_CMD_SET_KI: applyKi(r.value); break;
    case WS_CMD_SET_KD: applyKd(r.value); break;
    case WS_CMD_SET_WEIGHTS: applyWeights(r.w1, r.w2, r.w3); break;
    case WS_CMD_TOGGLE_OIL: actuator_set_oil_pump(r.bvalue); break;
    case WS_CMD_TOGGLE_WATER: actuator_set_water_pump(r.bvalue); break;
    case WS_CMD_TOGGLE_FAN: actuator_set_fan_enabled(r.bvalue); break;
    case WS_CMD_APPLY_BEST: doApplyBest(); break;
    case WS_CMD_RESET_DEFAULT: doResetDefault(); break;
    case WS_CMD_GET_EFFICIENCY_MAP: {
      EfficiencyMap map;
      sd_load_efficiency_map(map);
      g_recommended_sp = map.recommended_setpoint;
      web_server_send_efficiency_map_json(sd_efficiency_map_to_json(map));
      break;
    }
    case WS_CMD_GET_HISTORY:
      web_server_send_history_csv(sd_read_history_tail(100));
      break;
    case WS_CMD_TEST_FAN: if (state == STATE_IDLE) actuator_set_fan_pwm((uint8_t)constrain(r.value, 0.0f, 255.0f)); break;
    case WS_CMD_TEST_OIL_ON: if (state == STATE_IDLE) actuator_set_oil_pump(true); break;
    case WS_CMD_TEST_OIL_OFF: if (state == STATE_IDLE) actuator_set_oil_pump(false); break;
    case WS_CMD_TEST_WATER_ON: if (state == STATE_IDLE) actuator_set_water_pump(true); break;
    case WS_CMD_TEST_WATER_OFF: if (state == STATE_IDLE) actuator_set_water_pump(false); break;
    default: break;
  }
}

static void processLcdCommand(const LcdCommandResult &r) {
  switch (r.cmd) {
    case LCD_CMD_START: if (state == STATE_IDLE) beginRun(); break;
    case LCD_CMD_STOP: if (state == STATE_RUNNING) beginShutdown(); break;
    case LCD_CMD_AUTOTUNE: if (state == STATE_IDLE) { autotune_start(cfg.setpoint_aktif); state = STATE_AUTOTUNE; } break;
    case LCD_CMD_TEST_FAN: if (state == STATE_IDLE) actuator_set_fan_pwm((uint8_t)constrain(r.value, 0.0f, 255.0f)); break;
    case LCD_CMD_TEST_OIL_ON: if (state == STATE_IDLE) actuator_set_oil_pump(true); break;
    case LCD_CMD_TEST_OIL_OFF: if (state == STATE_IDLE) actuator_set_oil_pump(false); break;
    case LCD_CMD_TEST_WATER_ON: if (state == STATE_IDLE) actuator_set_water_pump(true); break;
    case LCD_CMD_TEST_WATER_OFF: if (state == STATE_IDLE) actuator_set_water_pump(false); break;
    case LCD_CMD_SET_SETPOINT: applySetpoint(r.value); break;
    case LCD_CMD_SET_TSHUTDOWN: applyTShutdown(r.value); break;
    case LCD_CMD_SET_TARGET_VOL: applyTargetVol(r.value); break;
    default: break;
  }
}

// ------------------------------------------------------------
// State handlers
// ------------------------------------------------------------
static void loopIdle(unsigned long now) {
  (void)now;
  g_fan_pwm = actuator_get_fan_pwm();
  g_fan_pct = actuator_get_fan_pct();
}

static void loopAutotune(unsigned long now) {
  if (now - lastPidMs < LOOP_INTERVAL_MS) return;
  lastPidMs = now;

  if (checkEmergency()) return;

  bool finished = autotune_update(g_suhu);
  const AutotuneStatus &st = autotune_get_status();
  g_fan_pwm = actuator_get_fan_pwm();
  g_fan_pct = actuator_get_fan_pct();

  if (finished) {
    if (!st.timed_out) {
      g_kp = st.result_kp; g_ki = st.result_ki; g_kd = st.result_kd;
      cfg.active_kp = g_kp; cfg.active_ki = g_ki; cfg.active_kd = g_kd;
      pid.setBaseGains(g_kp, g_ki, g_kd);

      PidParamEntry *e = currentSetpointEntry();
      if (e) { e->kp = g_kp; e->ki = g_ki; e->kd = g_kd; }
      sd_save_config(cfg);

      beginRun(); // auto-transition to RUNNING with the freshly tuned gains
    } else {
      // Timeout: abort, but still cool down under water-pump monitoring
      // before going idle rather than dropping straight to IDLE while hot.
      actuator_set_fan_pwm(0);
      actuator_set_oil_pump(false);
      actuator_set_water_pump(true);
      skipLearningAfterShutdown = true;
      state = STATE_SHUTDOWN;
      shutdownStartMs = millis();
    }
  }
}

static void loopRunning(unsigned long now) {
  if (now - lastPidMs >= LOOP_INTERVAL_MS) {
    float dt = (now - lastPidMs) / 1000.0f;
    lastPidMs = now;

    if (checkEmergency()) return;

    uint8_t pwm = pid.compute(cfg.setpoint_aktif, g_suhu, dt);
    actuator_set_fan_pwm(pwm);
    g_fan_pwm = pwm;
    g_fan_pct = actuator_get_fan_pct();
    g_error = pid.getError();
    g_ise = pid.getISEAccum();

    runPwmSum += pwm;
    runPwmSamples++;

    g_volume_ml = sensor_get_volume_ml();
    g_duration_ms = now - runStartMs;
    g_efficiency = learning_compute_efficiency_ml_h(g_volume_ml, g_duration_ms);
  }

  if (now - lastLogMs >= LOG_INTERVAL_MS) {
    lastLogMs = now;
    sd_run_log_row(now, g_duration_ms / 1000.0f, g_suhu, cfg.setpoint_aktif, g_fan_pwm,
                    g_error, g_kp, g_ki, g_kd, g_volume_ml, g_efficiency, "RUNNING");
  }

  if (g_volume_ml >= cfg.target_volume_ml) {
    beginShutdown();
  }
}

static void loopShutdown(unsigned long now) {
  (void)now;
  g_fan_pwm = 0;
  g_fan_pct = 0;

  if (g_suhu < cfg.t_shutdown) {
    actuator_set_water_pump(false);
    if (skipLearningAfterShutdown) {
      skipLearningAfterShutdown = false;
      state = STATE_IDLE;
      Serial.println(F("[main] -> IDLE (cool-down complete, no learning data)"));
    } else {
      state = STATE_LEARNING;
      learningEnterMs = millis();
      Serial.println(F("[main] -> LEARNING"));
    }
  }
}

static void loopLearning(unsigned long now) {
  // Hold in LEARNING for a short dwell so the dashboard/LCD actually get a
  // chance to show it before the (synchronous, SD-bound) crunch happens.
  if (now - learningEnterMs < 300) return;

  RunResult run;
  run.run_num = currentRunNumber;
  run.setpoint = cfg.setpoint_aktif;
  run.total_vol_ml = finalVolumeMl;
  run.duration_ms = finalDurationMs;
  run.ise = finalIse;
  run.avg_pwm = finalAvgPwm;
  run.kp = finalKp; run.ki = finalKi; run.kd = finalKd;

  float newKp, newKi, newKd;
  learning_process_run(cfg, run, newKp, newKi, newKd);

  g_kp = newKp; g_ki = newKi; g_kd = newKd;
  cfg.active_kp = newKp; cfg.active_ki = newKi; cfg.active_kd = newKd;
  pid.setBaseGains(newKp, newKi, newKd);

  EfficiencyMap map;
  sd_load_efficiency_map(map);
  g_recommended_sp = map.recommended_setpoint;

  state = STATE_IDLE;
  Serial.println(F("[main] -> IDLE"));
}

// ------------------------------------------------------------
// LCD / WebSocket telemetry builders
// ------------------------------------------------------------
static void refreshDisplay() {
  LcdData d;
  d.state = state;
  d.suhu = g_suhu;
  d.setpoint = cfg.setpoint_aktif;
  d.fan_pct = g_fan_pct;
  d.volume_ml = g_volume_ml;
  d.efficiency_ml_h = g_efficiency;
  d.duration_ms = g_duration_ms;
  d.ise = g_ise;
  d.run_count = cfg.run_count;
  d.kp = g_kp; d.ki = g_ki; d.kd = g_kd;
  d.t_shutdown = cfg.t_shutdown;
  d.target_volume_ml = cfg.target_volume_ml;
  d.w1 = cfg.w1; d.w2 = cfg.w2; d.w3 = cfg.w3;
  d.oil_pump = actuator_get_oil_pump();
  d.water_pump = actuator_get_water_pump();
  d.fan_enabled = actuator_get_fan_enabled();
  d.sd_ok = g_sd_ok;

  EfficiencyMap map;
  sd_load_efficiency_map(map);
  d.eff_count = min((uint8_t)8, map.count);
  for (uint8_t i = 0; i < d.eff_count; i++) {
    d.eff_setpoint[i] = map.entries[i].setpoint;
    d.eff_runs[i] = map.entries[i].runs;
    d.eff_score[i] = map.entries[i].avg_score;
    d.eff_efficiency[i] = map.entries[i].avg_efficiency;
  }
  d.recommended_setpoint = map.recommended_setpoint;

  display_update(d);
}

static void broadcastWs() {
  WsBroadcastData d;
  d.state = state;
  d.suhu = g_suhu;
  d.setpoint = cfg.setpoint_aktif;
  d.fan_pwm = g_fan_pwm;
  d.fan_pct = g_fan_pct;
  d.error = g_error;
  d.kp = g_kp; d.ki = g_ki; d.kd = g_kd;
  d.volume_ml = g_volume_ml;
  d.target_vol = cfg.target_volume_ml;
  d.efficiency = g_efficiency;
  d.duration_ms = g_duration_ms;
  d.est_finish_ms = (g_efficiency > 0.001f)
      ? (long)(((cfg.target_volume_ml - g_volume_ml) / g_efficiency) * 3600000.0f)
      : -1;
  d.ise = g_ise;
  d.run_count = cfg.run_count;
  d.oil_pump = actuator_get_oil_pump();
  d.water_pump = actuator_get_water_pump();
  d.fan_enabled = actuator_get_fan_enabled();

  const AutotuneStatus &at = autotune_get_status();
  d.at_cycle = at.cycle;
  d.at_heating = at.heating;
  d.at_peak = at.peak;
  d.at_valley = at.valley;

  d.sd_ok = g_sd_ok;
  d.best_ise = cfg.best_ise;
  d.recommended_sp = g_recommended_sp;
  d.heap_free = ESP.getFreeHeap();

  web_server_broadcast(d);
}

// ------------------------------------------------------------
// Arduino setup / loop
// ------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println(F("\n[main] Pyrolysis Controller booting..."));

  sensor_init();
  actuator_init();

  g_sd_ok = sd_init();
  if (g_sd_ok) {
    if (!sd_load_config(cfg)) {
      Serial.println(F("[main] no config.json found, writing defaults"));
      sd_save_config(cfg);
    }
  } else {
    Serial.println(F("[main] WARNING: SD unavailable, running with in-RAM defaults only"));
  }

  learning_resolve_gains_for_setpoint(cfg, cfg.setpoint_aktif, g_kp, g_ki, g_kd);
  cfg.active_kp = g_kp; cfg.active_ki = g_ki; cfg.active_kd = g_kd;
  pid.setBaseGains(g_kp, g_ki, g_kd);

  {
    EfficiencyMap map;
    if (g_sd_ok) sd_load_efficiency_map(map);
    g_recommended_sp = map.recommended_setpoint > 0 ? map.recommended_setpoint : cfg.setpoint_aktif;
  }

  display_init();
  web_server_init();

  state = STATE_IDLE;
  Serial.println(F("[main] Ready. State=IDLE"));
}

void loop() {
  unsigned long now = millis();

  g_suhu = sensor_read_temperature();

  display_poll_touch();

  while (web_server_has_pending_command()) {
    processWebCommand(web_server_pop_command());
  }
  LcdCommandResult lcdCmd = display_get_pending_command();
  if (lcdCmd.cmd != LCD_CMD_NONE) processLcdCommand(lcdCmd);

  switch (state) {
    case STATE_IDLE: loopIdle(now); break;
    case STATE_AUTOTUNE: loopAutotune(now); break;
    case STATE_RUNNING: loopRunning(now); break;
    case STATE_SHUTDOWN: loopShutdown(now); break;
    case STATE_LEARNING: loopLearning(now); break;
  }

  web_server_loop();

  if (now - lastLcdMs >= LCD_INTERVAL_MS) {
    lastLcdMs = now;
    refreshDisplay();
  }
  if (now - lastWsMs >= WS_INTERVAL_MS) {
    lastWsMs = now;
    broadcastWs();
  }
  if (now - lastHeapWarnMs >= 5000) {
    lastHeapWarnMs = now;
    uint32_t heap = ESP.getFreeHeap();
    if (heap < HEAP_WARN_THRESHOLD) {
      Serial.printf("[main] WARNING low heap: %lu bytes free\n", (unsigned long)heap);
    }
  }
}
