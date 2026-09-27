% ============================================================
% test_noise_toggle.m
% STEP 3-4 dari debugging: jalankan setpoint 300 degC dulu dengan
% noise_variance_ws=0 (harus settle bersih -> membuktikan parameter
% PID sendiri sudah benar), lalu dengan noise_variance_ws=2.25 (nilai
% produksi) untuk memverifikasi filter noise->controller (STEP 2)
% cukup meredam osilasi steady-state ke bawah +-3 degC.
%
% Script diagnostik terpisah -- TIDAK mengubah alur main_pyrolysis_
% simulation.m (yang selalu memakai noise_variance_ws=2.25/produksi).
% ============================================================
clear; clc; close all;
thisDir = fileparts(mfilename('fullpath'));
cd(thisDir);
resultsDir = fullfile(thisDir, 'results');

build_simulink_model();
mdl = 'pyrolysis_adaptive_pid';
load_system(mdl);

setpoint = 300;
T_sim = 1800;

cases = {'Noise OFF (variance=0)', 0; 'Noise ON (variance=2.25)', 2.25};
results = struct([]);

for c = 1:size(cases,1)
    label = cases{c,1};
    noiseVar = cases{c,2};
    fprintf('--- %s ---\n', label);

    assignin('base', 'setpoint_ws', setpoint);
    assignin('base', 'gs_enable_ws', 1);
    assignin('base', 'lambda_p_ws', 0.01);
    assignin('base', 'lambda_i_ws', 0.0001);
    assignin('base', 'lambda_d_ws', 0.003);
    assignin('base', 'noise_variance_ws', noiseVar);

    simOut = sim(mdl, 'StopTime', num2str(T_sim));
    t = simOut.get('T_out').Time;
    T = simOut.get('T_out').Data;

    metrics = compute_response_metrics(t, T, setpoint);
    n_tail = round(0.3*numel(T)); % 30% waktu terakhir utk cek amplitudo steady-state
    tail = T(end-n_tail+1:end);
    ripple = max(tail) - min(tail);

    fprintf('  Rise=%.1fs Settle=%.1fs Over=%.1f%% SSE=%.2fdegC ISE=%.1f Ripple(30%%tail)=%.2fdegC\n', ...
        metrics.rise_time, metrics.settling_time, metrics.overshoot, metrics.sse, metrics.ise, ripple);

    r.label = label; r.t = t; r.T = T; r.metrics = metrics; r.ripple = ripple;
    results = [results, r]; %#ok<AGROW>
end

close_system(mdl, 0);

fig = figure('Name', 'Noise Toggle Test', 'Color', 'white', 'Units', 'inches', 'Position', [1 1 10 7]);
subplot(2,1,1);
plot(results(1).t/60, results(1).T, 'b-', 'LineWidth', 1.5); hold on;
yline(setpoint, 'k--');
grid on; box on; set(gca, 'FontSize', 11);
xlabel('Waktu (menit)'); ylabel('Suhu (degC)');
title(sprintf('%s (Rise=%.0fs, Settle=%.0fs, Ripple=%.2fdegC)', ...
    results(1).label, results(1).metrics.rise_time, results(1).metrics.settling_time, results(1).ripple));

subplot(2,1,2);
plot(results(2).t/60, results(2).T, 'r-', 'LineWidth', 1.2); hold on;
yline(setpoint, 'k--');
grid on; box on; set(gca, 'FontSize', 11);
xlabel('Waktu (menit)'); ylabel('Suhu (degC)');
title(sprintf('%s (Rise=%.0fs, Settle=%.0fs, Ripple=%.2fdegC)', ...
    results(2).label, results(2).metrics.rise_time, results(2).metrics.settling_time, results(2).ripple));

exportgraphics(fig, fullfile(resultsDir, 'fig_noise_toggle_test.png'), 'Resolution', 300);
fprintf('\nDisimpan: %s\n', fullfile(resultsDir, 'fig_noise_toggle_test.png'));

fprintf('\n=== KESIMPULAN ===\n');
if results(1).ripple < 1.0
    fprintf('Noise OFF: sistem settle bersih (ripple=%.2fdegC) -> parameter PID/DEAD_ZONE OK.\n', results(1).ripple);
else
    fprintf('Noise OFF: MASIH ripple %.2fdegC walau noise dimatikan -> masalah ADA di parameter/logika PID, BUKAN di noise.\n', results(1).ripple);
end
if results(2).ripple < 3.0
    fprintf('Noise ON : ripple steady-state = %.2fdegC (target <3degC) -> TERCAPAI.\n', results(2).ripple);
else
    fprintf('Noise ON : ripple steady-state = %.2fdegC (target <3degC) -> BELUM tercapai, perlu filter lebih kuat (naikkan tau_f).\n', results(2).ripple);
end
