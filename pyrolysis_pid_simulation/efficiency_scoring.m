function api = efficiency_scoring()
% ============================================================
% efficiency_scoring.m
% Efficiency score & pencarian setpoint optimal
%
% Judul  : Implementasi Adaptive PID untuk Kendali Suhu Tungku
%          Pyrolysis Berbasis ESP32
% Nama   : [Nama Mahasiswa]
% NIM    : [NIM]
% Tanggal: [Tanggal]
%
% CATATAN FIDELITAS:
% Formula score = w1*produktivitas - w2*avg_pwm - w3*(ISE/1000) adalah
% PERSIS formula learning_compute_score() di learning_system.cpp
% (ISE_NORM_SCALE = 1000, avg_pwm dipakai APA ADANYA tanpa normalisasi
% 0-1, w1=0.6/w2=0.2/w3=0.2 default).
%
% Produktivitas (mL/jam minyak pirolisis) TIDAK diukur oleh model
% termal FOPDT (itu proses kondensasi/tetesan terpisah yang di ESP32
% diukur sensor hall tipping-bucket). Untuk keperluan simulasi &
% pencarian setpoint optimal, produktivitas dimodelkan empiris
% mengikuti pola umum pirolisis: naik seiring suhu lalu turun lagi
% di suhu tinggi karena cracking sekunder mengubah minyak jadi gas.
% Ini ASUMSI PEMODELAN, bukan data terukur -- ganti productivityModel()
% dengan data eksperimen nyata bila tersedia.
% ============================================================

api.computeScore       = @computeScore;
api.productivityModel  = @productivityModel;
api.findOptimalSetpoint = @findOptimalSetpoint;
end

% ------------------------------------------------------------
function score = computeScore(productivity, avg_pwm, ISE, w1, w2, w3)
% score = w1*produktivitas - w2*avg_pwm - w3*ISE_norm  (ISE_norm = ISE/1000)
ISE_NORM_SCALE = 1000;
score = w1*productivity - w2*avg_pwm - w3*(ISE/ISE_NORM_SCALE);
end

% ------------------------------------------------------------
function prod_ml_h = productivityModel(setpoint)
% Model empiris hasil minyak pirolisis (mL/jam) vs suhu tungku.
% Puncak hasil di sekitar 400 degC, menurun di suhu lebih tinggi
% akibat cracking sekunder (khas pirolisis biomassa/plastik).
T_opt = 400; sigma = 160; prod_max = 800;
prod_ml_h = prod_max * exp(-((setpoint - T_opt)/sigma).^2);
end

% ------------------------------------------------------------
function [setpoint_opt, score_map] = findOptimalSetpoint(...
    setpoint_range, plant_params, pid_params, w1, w2, w3)
% Simulasikan tiap setpoint dalam setpoint_range (closed loop, 1 jam,
% Adaptive PID / gain scheduling), hitung score, kembalikan setpoint
% dengan score tertinggi.
plant = plant_model();
pidm = pid_design();

n = numel(setpoint_range);
score_map.setpoint = setpoint_range(:);
score_map.productivity = zeros(n,1);
score_map.avg_pwm = zeros(n,1);
score_map.ISE = zeros(n,1);
score_map.score = zeros(n,1);

T_sim = 3600; % 1 jam per setpoint

for i = 1:n
    sp = setpoint_range(i);
    ctrl = pidm.defaultCtrl(pid_params.Kp, pid_params.Ki, pid_params.Kd, 'zone');
    opts.dt = 0.5; opts.T_sim = T_sim; opts.T_init = plant_params.T_ambient;
    opts.setpoint = sp; opts.noise = false;
    out = plant.simulateClosedLoop(plant_params, ctrl, opts);

    score_map.productivity(i) = productivityModel(sp);
    score_map.avg_pwm(i) = mean(out.u);
    score_map.ISE(i) = out.ISE;
    score_map.score(i) = computeScore(score_map.productivity(i), score_map.avg_pwm(i), out.ISE, w1, w2, w3);
end

[~, idx_best] = max(score_map.score);
setpoint_opt = setpoint_range(idx_best);
score_map.idx_best = idx_best;
% Plotting dipusatkan di plot_results.plotEfficiencyMap(score_map) --
% dipanggil dari main_simulation.m.
end
