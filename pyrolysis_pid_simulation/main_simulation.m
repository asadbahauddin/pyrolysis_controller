%% ============================================================
%% MAIN SIMULATION SCRIPT
%% Adaptive PID Pyrolysis Controller — Digital Twin dari firmware ESP32
%%
%% Judul  : Implementasi Adaptive PID untuk Kendali Suhu Tungku
%%          Pyrolysis Berbasis ESP32
%% Nama   : [Nama Mahasiswa]
%% NIM    : [NIM]
%% Tanggal: [Tanggal]
%%
%% Jalankan script ini (F5 / run) untuk menjalankan seluruh simulasi.
%% Hasil (figure PNG 300 DPI + tabel) otomatis tersimpan di results/.
%% Membutuhkan: MATLAB R2020a+, Control System Toolbox.
%% ============================================================

clear; clc; close all;
fprintf('=== Simulasi Adaptive PID Pyrolysis Controller ===\n\n');

thisDir = fileparts(mfilename('fullpath'));
resultsDir = fullfile(thisDir, 'results');
if ~exist(resultsDir, 'dir'), mkdir(resultsDir); end

plant = plant_model();
pidm  = pid_design();
zn    = ziegler_nichols();
gs    = gain_scheduling();
cd_   = coordinate_descent();
eff   = efficiency_scoring();
cmp   = compare_methods();
plots = plot_results();

%% ============================================================
%% BAGIAN 1: DEFINISI PARAMETER
%% ============================================================
params = plant.defaultParams();
setpoint = params.setpoint; % 300 degC

fprintf('--- Parameter Tungku ---\n');
fprintf('K   = %.2f degC/%%PWM (zona tengah)\n', params.K_plant);
fprintf('tau = %.0f s\n', params.tau_plant);
fprintf('Td  = %.0f s\n', params.Td_plant);
fprintf('T_ambient = %.0f degC | Setpoint = %.0f degC\n\n', params.T_ambient, setpoint);

%% ============================================================
%% BAGIAN 2: BANGUN MODEL PLANT
%% ============================================================
[G, G_approx] = plant.buildPlantModel(params.K_plant, params.tau_plant, params.Td_plant, 'mid');
fprintf('--- Model Plant FOPDT (zona tengah) ---\n');
fprintf('G(s) = K * exp(-Td*s) / (tau*s + 1)\n');
disp(G_approx);
fprintf('(dead time didekati Pade orde-2 pada variabel G)\n\n');

%% ============================================================
%% BAGIAN 3: AUTO-TUNE ZIEGLER-NICHOLS (relay, meniru autotune.cpp)
%% ============================================================
fprintf('--- Menjalankan Auto-tune Ziegler-Nichols (relay) ---\n');
[Kp_zn, Ki_zn, Kd_zn, Ku, Tu, ~, ~, trace_at] = zn.runRelayAutoTune(params, setpoint, 4, false);
fig1 = plots.plotAutoTuneProcess(trace_at);
plots.saveFigure(fig1, fullfile(resultsDir, 'fig1_relay_autotune.png'));
fprintf('Ku=%.4f Tu=%.2fs -> Kp=%.4f Ki=%.4f Kd=%.4f\n\n', Ku, Tu, Kp_zn, Ki_zn, Kd_zn);

%% ============================================================
%% BAGIAN 4: GAIN SCHEDULING
%% ============================================================
ctrlSched = pidm.defaultCtrl(Kp_zn, Ki_zn, Kd_zn, 'zone');
Trange = 30:5:600;
figGS = gs.plotGainSchedule(ctrlSched.schedule_table, Trange);
plots.saveFigure(figGS, fullfile(resultsDir, 'fig0_gain_schedule_table.png'));
fprintf('--- Gain Scheduling (hard-switch, meniru firmware) ---\n');
for Tz = [100 300 500]
    [kp_z, ki_z, kd_z] = gs.gainSchedule(Tz, Kp_zn, Ki_zn, Kd_zn);
    fprintf('  T=%3d degC -> Kp=%.4f Ki=%.4f Kd=%.4f\n', Tz, kp_z, ki_z, kd_z);
end
fprintf('\n');

%% ============================================================
%% BAGIAN 5: COORDINATE DESCENT OPTIMIZATION
%% ============================================================
fprintf('--- Menjalankan Coordinate Descent (resimulasi closed-loop) ---\n');
n_cd_iter = 15;
[Kp_cd, Ki_cd, Kd_cd, ise_hist_cd, Kp_hist_cd, Ki_hist_cd, Kd_hist_cd] = ...
    cd_.coordinateDescent(params, setpoint, Kp_zn, Ki_zn, Kd_zn, n_cd_iter, 0.05);
fig5 = plots.plotPIDParams(1:n_cd_iter, Kp_hist_cd, Ki_hist_cd, Kd_hist_cd);
plots.saveFigure(fig5, fullfile(resultsDir, 'fig5_pid_convergence.png'));
fprintf('ISE turun dari %.2f -> %.2f setelah %d iterasi\n', ise_hist_cd(1), ise_hist_cd(end), n_cd_iter);
fprintf('Kp: %.4f -> %.4f | Ki: %.4f -> %.4f | Kd: %.4f -> %.4f\n\n', ...
    Kp_zn, Kp_cd, Ki_zn, Ki_cd, Kd_zn, Kd_cd);

%% ============================================================
%% BAGIAN 6: PERBANDINGAN SEMUA METODE (Skenario 1: Setpoint Tracking)
%% ============================================================
fprintf('--- Membandingkan 5 metode kontrol (Skenario 1: Setpoint Tracking) ---\n');
T_sim_compare = 7200; % 2 jam, sesuai spesifikasi
results_all = cmp.runAllMethods(params, setpoint, T_sim_compare);
cmp.printComparisonTable(results_all);

fig2 = plots.plotComparisonAll(results_all, setpoint);
plots.saveFigure(fig2, fullfile(resultsDir, 'fig2_comparison_step_response.png'));
fig3 = plots.plotErrorComparison(results_all);
plots.saveFigure(fig3, fullfile(resultsDir, 'fig3_error_comparison.png'));
fig4 = plots.plotPerformanceMetrics(results_all);
plots.saveFigure(fig4, fullfile(resultsDir, 'fig4_performance_metrics.png'));

% Simpan tabel perbandingan ke file teks
tableFile = fullfile(resultsDir, 'table_comparison.txt');
fid = fopen(tableFile, 'w');
fprintf(fid, '%-17s %10s %10s %10s %8s %10s %10s %10s\n', ...
    'Metode','ISE','IAE','ITAE','Over(%)','Rise(s)','Settle(s)','SSE(degC)');
for i = 1:numel(results_all)
    r = results_all(i);
    fprintf(fid, '%-17s %10.2f %10.2f %10.0f %8.1f %10.1f %10.1f %10.2f\n', ...
        r.name, r.ISE, r.IAE, r.ITAE, r.overshoot, r.rise_time, r.settling_time, r.sse);
end
fclose(fid);
fprintf('Tabel perbandingan disimpan ke %s\n\n', tableFile);

%% ============================================================
%% BAGIAN 6b: Efek Gain Scheduling pada respon Adaptive PID (fig6)
%% ============================================================
idxAdaptive = find(strcmp({results_all.name}, 'Adaptive PID'), 1);
rAdaptive = results_all(idxAdaptive);
ctrlAdaptive = pidm.defaultCtrl(rAdaptive.Kp, rAdaptive.Ki, rAdaptive.Kd, 'zone');
optsAdaptive.dt = 0.5; optsAdaptive.T_sim = T_sim_compare; optsAdaptive.T_init = params.T_ambient;
optsAdaptive.setpoint = setpoint; optsAdaptive.noise = false;
outAdaptive = plant.simulateClosedLoop(params, ctrlAdaptive, optsAdaptive);
fig6 = plots.plotGainScheduleEffect(outAdaptive.t, outAdaptive.T, outAdaptive.Kp, outAdaptive.zone);
plots.saveFigure(fig6, fullfile(resultsDir, 'fig6_gain_scheduling.png'));

%% ============================================================
%% BAGIAN 7: EFFICIENCY SCORING (Skenario 5: Optimal Setpoint Search)
%% ============================================================
fprintf('--- Mencari setpoint optimal (Skenario 5) ---\n');
setpoint_range = 200:50:400;
pid_params_opt.Kp = Kp_cd; pid_params_opt.Ki = Ki_cd; pid_params_opt.Kd = Kd_cd;
[setpoint_opt, score_map] = eff.findOptimalSetpoint(setpoint_range, params, pid_params_opt, ...
    0.6, 0.2, 0.2); % w1, w2, w3 sesuai firmware (DEFAULT_W1/W2/W3)
fig7 = plots.plotEfficiencyMap(score_map);
plots.saveFigure(fig7, fullfile(resultsDir, 'fig7_efficiency_map.png'));
fprintf('Setpoint optimal = %.0f degC (score = %.2f)\n\n', setpoint_opt, score_map.score(score_map.idx_best));

%% ============================================================
%% BAGIAN 8: SKENARIO 2 — DISTURBANCE REJECTION
%% ============================================================
fprintf('--- Skenario 2: Disturbance Rejection (+20 degC pada t=1800s) ---\n');
t_dist = 1800; dist_duration = 30; dist_total = 20; % degC
distFcn = @(t) (t >= t_dist && t < t_dist+dist_duration) * (dist_total/dist_duration);

methodsForDist = results_all; % pakai gain yang sama dgn Skenario 1
results_dist = struct([]);
for i = 1:numel(methodsForDist)
    m = methodsForDist(i);
    mode_i = 'fixed'; if any(strcmp(m.name, {'Gain Schedule','Adaptive PID'})), mode_i = 'zone'; end
    ctrl_i = pidm.defaultCtrl(m.Kp, m.Ki, m.Kd, mode_i);
    opts_i.dt = 0.5; opts_i.T_sim = 3600; opts_i.T_init = params.T_ambient;
    opts_i.setpoint = setpoint; opts_i.noise = false; opts_i.disturbance = distFcn;
    out_i = plant.simulateClosedLoop(params, ctrl_i, opts_i);
    r.name = m.name; r.t = out_i.t; r.T = out_i.T;
    results_dist = [results_dist, r]; %#ok<AGROW>
end
fig9 = plots.plotDisturbanceRejection(results_dist, t_dist);
plots.saveFigure(fig9, fullfile(resultsDir, 'fig9_disturbance_rejection.png'));
fprintf('Selesai.\n\n');

%% ============================================================
%% BAGIAN 8b: SKENARIO 3 — SETPOINT CHANGE (Adaptive PID)
%% ============================================================
fprintf('--- Skenario 3: Setpoint Change (Adaptive PID) ---\n');
spProfile = @(t) 250*(t<1200) + 350*(t>=1200 & t<3600) + 300*(t>=3600);
ctrlSp = pidm.defaultCtrl(Kp_cd, Ki_cd, Kd_cd, 'zone');
optsSp.dt = 0.5; optsSp.T_sim = 5400; optsSp.T_init = params.T_ambient;
optsSp.setpoint = spProfile; optsSp.noise = false;
outSp = plant.simulateClosedLoop(params, ctrlSp, optsSp);
fig10 = figure('Name','Setpoint Change Tracking','Color','white');
spTrace = arrayfun(spProfile, outSp.t);
plot(outSp.t/60, outSp.T, 'b-', 'LineWidth', 1.8); hold on;
plot(outSp.t/60, spTrace, 'k--', 'LineWidth', 1.2);
grid on; box on; set(gca,'FontSize',12);
xlabel('Waktu (menit)'); ylabel('Suhu (degC)');
title('Skenario 3: Tracking Perubahan Setpoint (Adaptive PID)');
legend('Suhu aktual','Setpoint','Location','best');
plots.saveFigure(fig10, fullfile(resultsDir, 'fig10_setpoint_change.png'));
fprintf('Selesai.\n\n');

%% ============================================================
%% BAGIAN 8c: SKENARIO 4 — MULTI-RUN ADAPTIVE LEARNING
%% ============================================================
fprintf('--- Skenario 4: Multi-Run Adaptive Learning (5 run) ---\n');
n_runs = 5;
Kp_r = Kp_zn; Ki_r = Ki_zn; Kd_r = Kd_zn; % mulai dari hasil ZN relay, spt firmware
ise_per_run = zeros(n_runs,1);
Kp_runs = zeros(n_runs,1); Ki_runs = zeros(n_runs,1); Kd_runs = zeros(n_runs,1);

for run = 1:n_runs
    ctrl_r = pidm.defaultCtrl(Kp_r, Ki_r, Kd_r, 'zone');
    opts_r.dt = 0.5; opts_r.T_sim = 3600; opts_r.T_init = params.T_ambient;
    opts_r.setpoint = setpoint; opts_r.noise = true; % noise sensor spt hardware asli
    out_r = plant.simulateClosedLoop(params, ctrl_r, opts_r);

    ise_per_run(run) = out_r.ISE;
    Kp_runs(run) = Kp_r; Ki_runs(run) = Ki_r; Kd_runs(run) = Kd_r;

    % coordinate descent berbasis replay (spt learning_system.cpp),
    % dipakai untuk run berikutnya
    [Kp_r, Ki_r, Kd_r] = cd_.coordinateDescentReplay(out_r, Kp_r, Ki_r, Kd_r, 0.05);

    fprintf('  Run %d: ISE=%.2f -> Kp=%.4f Ki=%.4f Kd=%.4f (utk run berikutnya)\n', ...
        run, ise_per_run(run), Kp_r, Ki_r, Kd_r);
end

fig8 = plots.plotISEConvergence(ise_per_run, (1:n_runs)');
plots.saveFigure(fig8, fullfile(resultsDir, 'fig8_multirun_learning.png'));
figPidRuns = plots.plotPIDParams((1:n_runs)', Kp_runs, Ki_runs, Kd_runs);
plots.saveFigure(figPidRuns, fullfile(resultsDir, 'fig8b_pid_evolution_per_run.png'));
fprintf('\n');

%% ============================================================
%% BAGIAN 9: RINGKASAN AKHIR
%% ============================================================
idxKonv = find(strcmp({results_all.name}, 'Konvensional'), 1);
ise_conventional = results_all(idxKonv).ISE;
ise_adaptive = results_all(idxAdaptive).ISE;
improvement_pct = (ise_conventional - ise_adaptive) / ise_conventional * 100;

fprintf('\n=== RINGKASAN HASIL SIMULASI ===\n');
fprintf('Metode terbaik    : Adaptive PID\n');
fprintf('ISE Konvensional  : %.2f\n', ise_conventional);
fprintf('ISE Adaptive PID  : %.2f\n', ise_adaptive);
fprintf('Perbaikan ISE     : %.1f%%\n', improvement_pct);
fprintf('Overshoot         : %.1f%%\n', results_all(idxAdaptive).overshoot);
fprintf('Settling time     : %.1f detik\n', results_all(idxAdaptive).settling_time);
fprintf('Setpoint optimal  : %.0f degC\n', setpoint_opt);
fprintf('================================\n');
fprintf('Semua figure tersimpan di: %s\n', resultsDir);
