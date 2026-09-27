%% ============================================================
%% MAIN PYROLYSIS SIMULATION SCRIPT
%% Judul : Simulasi Adaptive PID Kontrol Suhu Tungku Pyrolysis
%% Nama  : [Nama Mahasiswa]
%% NIM   : [NIM]
%% Tahun : 2025
%%
%% Jalankan: matlab -batch "run('main_pyrolysis_simulation.m')"
%% atau di dalam MATLAB: run('main_pyrolysis_simulation.m')
%%
%% CATATAN: parameter plant (K=3.5, tau=180s, L=15s) adalah ASUMSI/
%% PLACEHOLDER untuk verifikasi STRUKTUR simulasi -- BUKAN hasil
%% kalibrasi dari alat fisik. Wajib dikalibrasi ulang setelah tungku
%% pyrolysis sungguhan diuji.
%% ============================================================

clear; clc; close all;

thisDir = fileparts(mfilename('fullpath'));
cd(thisDir);
resultsDir = fullfile(thisDir, 'results');
if ~exist(resultsDir, 'dir'), mkdir(resultsDir); end

fprintf('=== Simulasi Adaptive PID Pyrolysis (Simulink) ===\n\n');

% ---- Cek toolbox ----
hasSimulink = license('test', 'Simulink') && ~isempty(ver('simulink'));
hasControl  = license('test', 'Control_Toolbox') && ~isempty(ver('control'));
if ~hasSimulink
    error('main_pyrolysis_simulation:noSimulink', ...
        'Simulink tidak terdeteksi/berlisensi -- skrip ini butuh Simulink.');
end
if ~hasControl
    fprintf('[peringatan] Control System Toolbox tidak terdeteksi -- tidak masalah,\n');
    fprintf('             skrip ini tidak memakai fungsi tf()/pid() dari toolbox itu.\n\n');
end

%% ---- 1) Bangun model Simulink ----
fprintf('[1/4] Membangun model Simulink (pyrolysis_adaptive_pid.slx)...\n');
build_simulink_model();
mdl = 'pyrolysis_adaptive_pid';
load_system(mdl);
fprintf('      OK\n\n');

%% ---- 2) Jalankan 5 skenario setpoint ----
fprintf('[2/4] Menjalankan 5 skenario setpoint (Adaptive PID lengkap)...\n');
scenarioResults = run_all_scenarios(mdl);
fprintf('      OK\n\n');

%% ---- 3) Bandingkan 3 metode kontrol ----
fprintf('[3/4] Membandingkan 3 metode kontrol (setpoint 300 degC)...\n');
methodResults = compare_methods(mdl);
fprintf('      OK\n\n');

%% ---- 4) Plot & ekspor ----
fprintf('[4/4] Membuat figure & menyimpan hasil...\n');
plot_and_export(scenarioResults, methodResults, resultsDir);
fprintf('      OK\n\n');

close_system(mdl, 0);

%% ---- Ringkasan akhir ----
idxKonv = find(strcmp({methodResults.name}, 'PID Konvensional'), 1);
idxAdap = find(strcmp({methodResults.name}, 'Adaptive PID'), 1);

fprintf('=== RINGKASAN AKHIR ===\n');
fprintf('Model Simulink   : %s\n', fullfile(thisDir, [mdl '.slx']));
fprintf('ISE Konvensional : %.2f\n', methodResults(idxKonv).metrics.ise);
fprintf('ISE Adaptive PID : %.2f\n', methodResults(idxAdap).metrics.ise);
improvement = (methodResults(idxKonv).metrics.ise - methodResults(idxAdap).metrics.ise) ...
    / methodResults(idxKonv).metrics.ise * 100;
fprintf('Perbaikan ISE    : %.1f%%\n', improvement);
fprintf('Overshoot (Adaptive) : %.1f%%\n', methodResults(idxAdap).metrics.overshoot);
fprintf('Settling (Adaptive)  : %.1f s\n', methodResults(idxAdap).metrics.settling_time);
fprintf('Hasil figure & tabel : %s\n', resultsDir);
fprintf('\nPENTING: parameter plant (K, tau, L) adalah ASUMSI/PLACEHOLDER,\n');
fprintf('BUKAN hasil kalibrasi alat fisik -- kalibrasi ulang setelah pengujian nyata.\n');
fprintf('========================\n');
