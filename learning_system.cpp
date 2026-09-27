// ============================================================
// learning_system.cpp
// ============================================================
#include "learning_system.h"
#include "sd_manager.h"

static const float ISE_NORM_SCALE = 1000.0f;
static const int   MAX_CD_SAMPLES = 1200;

// -- scratch buffers for coordinate-descent replay (static: single run at a time) --
static float   s_cd_dt[MAX_CD_SAMPLES];
static float   s_cd_error[MAX_CD_SAMPLES];
static uint8_t s_cd_pwm[MAX_CD_SAMPLES];
static int     s_cd_count = 0;

static long s_counting_rows = 0;
static int  s_collect_stride = 1;
static int  s_collect_seen = 0;

float learning_compute_efficiency_ml_h(float volume_ml, unsigned long duration_ms) {
  float hours = duration_ms / 3600000.0f;
  if (hours <= 0.0001f) return 0;
  return volume_ml / hours;
}

float learning_compute_score(const RunResult &run, float w1, float w2, float w3) {
  float produktivitas = learning_compute_efficiency_ml_h(run.total_vol_ml, run.duration_ms);
  float ise_norm = run.ise / ISE_NORM_SCALE;
  return (w1 * produktivitas) - (w2 * run.avg_pwm) - (w3 * ise_norm);
}

static void countRowCb(float elapsed_s, float error, uint8_t fan_pwm, float dt) {
  (void)elapsed_s; (void)error; (void)fan_pwm; (void)dt;
  s_counting_rows++;
}

static void collectRowCb(float elapsed_s, float error, uint8_t fan_pwm, float dt) {
  (void)elapsed_s;
  if (s_collect_seen % s_collect_stride == 0 && s_cd_count < MAX_CD_SAMPLES) {
    s_cd_dt[s_cd_count] = dt;
    s_cd_error[s_cd_count] = error;
    s_cd_pwm[s_cd_count] = fan_pwm;
    s_cd_count++;
  }
  s_collect_seen++;
}

static bool loadRunForReplay(uint32_t run_num) {
  s_counting_rows = 0;
  if (!sd_read_run_csv(run_num, countRowCb) || s_counting_rows == 0) return false;

  s_collect_stride = (int)(s_counting_rows / MAX_CD_SAMPLES) + 1;
  s_collect_seen = 0;
  s_cd_count = 0;
  sd_read_run_csv(run_num, collectRowCb);
  return s_cd_count > 0;
}

// Simulates ISE for trial gains against the recorded error/pwm trace,
// using a linear process-gain proxy to estimate how the temperature
// error would have shifted had this trial output been applied instead
// of the output that was actually logged.
static float simulateISE(float kp, float ki, float kd, float k_process) {
  float integral = 0, prev_error = 0, ise = 0;
  bool first = true;

  for (int i = 0; i < s_cd_count; i++) {
    float dt = s_cd_dt[i];
    float error = s_cd_error[i];

    integral += ki * error * dt;
    if (integral > I_MAX) integral = I_MAX;
    if (integral < I_MIN) integral = I_MIN;

    float derivative = first ? 0 : kd * (error - prev_error) / dt;
    first = false;

    float u_trial = kp * error + integral + derivative;
    u_trial = constrain(u_trial, 0.0f, 255.0f);

    float u_actual = (float)s_cd_pwm[i];
    float e_new = error - k_process * (u_trial - u_actual);

    ise += (e_new * e_new) * dt;
    prev_error = error;
  }
  return ise;
}

static float estimateProcessGain() {
  float minE = s_cd_error[0], maxE = s_cd_error[0];
  float minP = s_cd_pwm[0], maxP = s_cd_pwm[0];
  for (int i = 1; i < s_cd_count; i++) {
    if (s_cd_error[i] < minE) minE = s_cd_error[i];
    if (s_cd_error[i] > maxE) maxE = s_cd_error[i];
    if (s_cd_pwm[i] < minP) minP = s_cd_pwm[i];
    if (s_cd_pwm[i] > maxP) maxP = s_cd_pwm[i];
  }
  float dE = maxE - minE;
  float dP = maxP - minP;
  if (dP < 1.0f) dP = 1.0f;
  return dE / dP;
}

// Coordinate descent: adjusts kp, then ki, then kd by +/-CD_STEP_RATIO,
// keeping whichever direction lowers simulated ISE.
static void coordinateDescent(uint32_t run_num, float kp0, float ki0, float kd0,
                               float &kp, float &ki, float &kd) {
  kp = kp0; ki = ki0; kd = kd0;

  if (!loadRunForReplay(run_num)) {
    Serial.println(F("[learning] no CSV data for coordinate descent, keeping gains"));
    return;
  }

  float k_process = estimateProcessGain();
  float best_ise = simulateISE(kp, ki, kd, k_process);

  // --- Kp ---
  {
    float plus = kp * (1.0f + CD_STEP_RATIO);
    float minus = kp * (1.0f - CD_STEP_RATIO);
    float ise_plus = simulateISE(plus, ki, kd, k_process);
    float ise_minus = simulateISE(minus, ki, kd, k_process);
    if (ise_plus < best_ise && ise_plus <= ise_minus) { kp = plus; best_ise = ise_plus; }
    else if (ise_minus < best_ise) { kp = minus; best_ise = ise_minus; }
  }
  // --- Ki ---
  {
    float plus = ki * (1.0f + CD_STEP_RATIO);
    float minus = ki * (1.0f - CD_STEP_RATIO);
    float ise_plus = simulateISE(kp, plus, kd, k_process);
    float ise_minus = simulateISE(kp, minus, kd, k_process);
    if (ise_plus < best_ise && ise_plus <= ise_minus) { ki = plus; best_ise = ise_plus; }
    else if (ise_minus < best_ise) { ki = minus; best_ise = ise_minus; }
  }
  // --- Kd ---
  {
    float plus = kd * (1.0f + CD_STEP_RATIO);
    float minus = kd * (1.0f - CD_STEP_RATIO);
    float ise_plus = simulateISE(kp, ki, plus, k_process);
    float ise_minus = simulateISE(kp, ki, minus, k_process);
    if (ise_plus < best_ise && ise_plus <= ise_minus) { kd = plus; best_ise = ise_plus; }
    else if (ise_minus < best_ise) { kd = minus; best_ise = ise_minus; }
  }

  Serial.printf("[learning] CD result: Kp %.3f->%.3f Ki %.4f->%.4f Kd %.3f->%.3f (sim ISE=%.2f)\n",
                kp0, kp, ki0, ki, kd0, kd, best_ise);
}

static PidParamEntry *findOrCreateParamEntry(RuntimeConfig &cfg, float setpoint) {
  for (uint8_t i = 0; i < cfg.param_count; i++) {
    if (cfg.params[i].used && fabs(cfg.params[i].setpoint - setpoint) < 0.01f) {
      return &cfg.params[i];
    }
  }
  if (cfg.param_count < MAX_PARAM_ENTRIES) {
    PidParamEntry &e = cfg.params[cfg.param_count];
    e = PidParamEntry();
    e.used = true;
    e.setpoint = setpoint;
    cfg.param_count++;
    return &e;
  }
  return nullptr; // table full
}

static void updateEfficiencyMap(RuntimeConfig &cfg, const RunResult &run, float score) {
  EfficiencyMap map;
  sd_load_efficiency_map(map);

  EfficiencyEntry *entry = nullptr;
  for (uint8_t i = 0; i < map.count; i++) {
    if (map.entries[i].used && fabs(map.entries[i].setpoint - run.setpoint) < 0.01f) {
      entry = &map.entries[i];
      break;
    }
  }
  if (!entry && map.count < MAX_EFFICIENCY_ENTRIES) {
    entry = &map.entries[map.count];
    *entry = EfficiencyEntry();
    entry->used = true;
    entry->setpoint = run.setpoint;
    map.count++;
  }

  if (entry) {
    float eff = learning_compute_efficiency_ml_h(run.total_vol_ml, run.duration_ms);
    uint16_t n = entry->runs;
    entry->avg_efficiency = (entry->avg_efficiency * n + eff) / (n + 1);
    entry->avg_score = (entry->avg_score * n + score) / (n + 1);
    entry->avg_ise = (entry->avg_ise * n + run.ise) / (n + 1);
    entry->runs = n + 1;
  }

  // Recommend the setpoint with the best average score among entries
  // that have accumulated enough runs to be statistically meaningful.
  float best_score = -1e18;
  float recommended = map.count > 0 ? map.entries[0].setpoint : run.setpoint;
  bool found = false;
  for (uint8_t i = 0; i < map.count; i++) {
    if (!map.entries[i].used || map.entries[i].runs < MIN_RUNS_RECOMMEND) continue;
    if (map.entries[i].avg_score > best_score) {
      best_score = map.entries[i].avg_score;
      recommended = map.entries[i].setpoint;
      found = true;
    }
  }
  if (found) map.recommended_setpoint = recommended;

  sd_save_efficiency_map(map);
}

void learning_resolve_gains_for_setpoint(const RuntimeConfig &cfg, float setpoint,
                                          float &kp, float &ki, float &kd) {
  // Exact match
  for (uint8_t i = 0; i < cfg.param_count; i++) {
    if (cfg.params[i].used && fabs(cfg.params[i].setpoint - setpoint) < 0.01f) {
      kp = cfg.params[i].kp;
      ki = cfg.params[i].ki;
      kd = cfg.params[i].kd;
      return;
    }
  }

  // Find nearest neighbor below and above the requested setpoint
  const PidParamEntry *below = nullptr, *above = nullptr;
  for (uint8_t i = 0; i < cfg.param_count; i++) {
    if (!cfg.params[i].used) continue;
    const PidParamEntry &e = cfg.params[i];
    if (e.setpoint < setpoint && (!below || e.setpoint > below->setpoint)) below = &e;
    if (e.setpoint > setpoint && (!above || e.setpoint < above->setpoint)) above = &e;
  }

  if (below && above) {
    float ratio = (setpoint - below->setpoint) / (above->setpoint - below->setpoint);
    kp = below->kp + ratio * (above->kp - below->kp);
    ki = below->ki + ratio * (above->ki - below->ki);
    kd = below->kd + ratio * (above->kd - below->kd);
  } else if (below) {
    kp = below->kp; ki = below->ki; kd = below->kd;
  } else if (above) {
    kp = above->kp; ki = above->ki; kd = above->kd;
  } else {
    kp = DEFAULT_KP; ki = DEFAULT_KI; kd = DEFAULT_KD;
  }
}

void learning_process_run(RuntimeConfig &cfg, const RunResult &run,
                           float &out_kp, float &out_ki, float &out_kd) {
  float score = learning_compute_score(run, cfg.w1, cfg.w2, cfg.w3);
  float efficiency = learning_compute_efficiency_ml_h(run.total_vol_ml, run.duration_ms);

  updateEfficiencyMap(cfg, run, score);

  float new_kp, new_ki, new_kd;
  coordinateDescent(run.run_num, run.kp, run.ki, run.kd, new_kp, new_ki, new_kd);

  PidParamEntry *entry = findOrCreateParamEntry(cfg, run.setpoint);
  if (entry) {
    entry->kp = new_kp;
    entry->ki = new_ki;
    entry->kd = new_kd;
    entry->runs += 1;
  }

  if (run.ise < cfg.best_ise) {
    cfg.best_ise = run.ise;
    BestParams bp;
    bp.valid = true;
    bp.setpoint = run.setpoint;
    bp.kp = new_kp;
    bp.ki = new_ki;
    bp.kd = new_kd;
    bp.ise = run.ise;
    sd_save_best_params(bp);
  }

  cfg.run_count += 1;
  sd_save_config(cfg);

  sd_append_history(run.run_num, run.setpoint, run.total_vol_ml, run.duration_ms,
                     efficiency, score, run.ise, run.avg_pwm, new_kp, new_ki, new_kd);

  out_kp = new_kp;
  out_ki = new_ki;
  out_kd = new_kd;

  Serial.printf("[learning] Run #%lu setpoint=%.1f score=%.2f eff=%.1f mL/h -> Kp=%.3f Ki=%.4f Kd=%.3f\n",
                (unsigned long)run.run_num, run.setpoint, score, efficiency, new_kp, new_ki, new_kd);
}
