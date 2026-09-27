function api = ziegler_nichols()
% ============================================================
% ziegler_nichols.m
% Auto-tune Ziegler-Nichols relay method (meniru autotune.cpp)
%
% Judul  : Implementasi Adaptive PID untuk Kendali Suhu Tungku
%          Pyrolysis Berbasis ESP32
% Nama   : [Nama Mahasiswa]
% NIM    : [NIM]
% Tanggal: [Tanggal]
% ============================================================

api.runRelayAutoTune = @runRelayAutoTune;
end

% ------------------------------------------------------------
function [Kp, Ki, Kd, Ku, Tu, peaks, valleys, trace] = ...
    runRelayAutoTune(plant_params, setpoint, n_cycles, doPlot)
% Simulasi relay method Ziegler-Nichols, PROSEDUR SAMA DENGAN ESP32:
%   1. Mulai dari suhu ambient
%   2. Fan ON (AT_PWM_HIGH) -> tunggu peak (band + rate hysteresis)
%   3. Fan OFF (AT_PWM_LOW) -> tunggu valley
%   4. Ulangi n_cycles kali
%   5. Ku = 4*d / (pi*a) ; Tu = rata-rata periode antar-peak
%   6. Kp = 0.6*Ku ; Ki = 1.2*Ku/Tu ; Kd = 0.075*Ku*Tu
%
% trace (output ke-8) berisi t/T/u/peak_t/valley_t mentah, dipakai
% plot_results.plotAutoTuneProcess() untuk membuat fig1_relay_autotune.
% doPlot: jika true, langsung panggil plotAutoTuneProcess di sini juga
% (berguna untuk pemakaian standalone/cepat).
if nargin < 4, doPlot = true; end

plant = plant_model();

relayOpts.dt = 0.5;
relayOpts.T_sim_max = 3600*3;    % batas waktu simulasi (detik), mirip AT_TIMEOUT_MS
relayOpts.T_init = plant_params.T_ambient;
relayOpts.setpoint = setpoint;
relayOpts.PWM_high = 78;         % setara AT_PWM_HIGH=200/255 pada firmware
relayOpts.PWM_low  = 0;          % AT_PWM_LOW
relayOpts.n_cycles = n_cycles;   % AT_CYCLES
relayOpts.temp_band = 5.0;       % AT_TEMP_BAND
relayOpts.stabil_band = 2.0;     % AT_STABIL_BAND

res = plant.simulateRelayOpenLoop(plant_params, relayOpts);

peaks = res.peaks; peak_t = res.peak_t;
valleys = res.valleys; valley_t = res.valley_t;
Tu = res.Tu;

avg_peak = mean(peaks);
avg_valley = mean(valleys);
a = (avg_peak - avg_valley) / 2;   % amplitudo osilasi proses
if a < 0.5, a = 0.5; end           % jaga-jaga pembagian dengan nol
d = (relayOpts.PWM_high - relayOpts.PWM_low) / 2; % amplitudo relay

Ku = (4*d) / (pi*a);
[Kp, Ki, Kd] = deal(0.6*Ku, 1.2*Ku/Tu, 0.075*Ku*Tu);

fprintf('  Auto-tune selesai: %d siklus, %d peak, %d valley\n', ...
    numel(peak_t), numel(peaks), numel(valleys));
fprintf('  Amplitudo proses (a) = %.2f degC | Ku = %.4f | Tu = %.2f s\n', a, Ku, Tu);
fprintf('  Hasil PID: Kp=%.4f  Ki=%.4f  Kd=%.4f\n', Kp, Ki, Kd);

trace.t = res.t; trace.T = res.T; trace.u = res.u;
trace.peak_t = peak_t; trace.peaks = peaks;
trace.valley_t = valley_t; trace.valleys = valleys;
trace.setpoint = setpoint;
trace.Ku = Ku; trace.Tu = Tu; trace.Kp = Kp; trace.Ki = Ki; trace.Kd = Kd;

if doPlot
    plots = plot_results();
    plots.plotAutoTuneProcess(trace);
end
end
