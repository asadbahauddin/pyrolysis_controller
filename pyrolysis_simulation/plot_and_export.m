function plot_and_export(scenarioResults, methodResults, resultsDir)
% ============================================================
% plot_and_export.m
% Semua figure (300 DPI) + tabel ASCII di console + simulation_results.txt
%
% Judul : Simulasi Adaptive PID Kontrol Suhu Tungku Pyrolysis
% Nama  : [Nama Mahasiswa]
% NIM   : [NIM]
% Tahun : 2025
% ============================================================

if ~exist(resultsDir, 'dir'), mkdir(resultsDir); end

% ---- Figure 1: respon 5 setpoint (subplot) ----
fig1 = figure('Name', 'Step Response Semua Setpoint', 'Color', 'white', ...
    'Units', 'inches', 'Position', [1 1 12 8]);
for i = 1:numel(scenarioResults)
    r = scenarioResults(i);
    subplot(3, 2, i);
    plot(r.t/60, r.T, 'b-', 'LineWidth', 1.5); hold on;
    yline(r.setpoint, 'k--', 'LineWidth', 1.0);
    grid on; box on; set(gca, 'FontSize', 10);
    xlabel('Waktu (menit)'); ylabel('Suhu (degC)');
    title(sprintf('Setpoint %d degC (Rise=%.0fs, Settle=%.0fs, Over=%.1f%%)', ...
        r.setpoint, r.metrics.rise_time, r.metrics.settling_time, r.metrics.overshoot));
end
sgtitle('Respon Suhu Adaptive PID — 5 Setpoint', 'FontSize', 14, 'FontWeight', 'bold');
exportgraphics(fig1, fullfile(resultsDir, 'fig1_step_response_all.png'), 'Resolution', 300);

% ---- Figure 2: perbandingan 3 metode ----
fig2 = figure('Name', 'Perbandingan 3 Metode', 'Color', 'white', ...
    'Units', 'inches', 'Position', [1 1 10 7]);
colors = {[0.5 0.5 0.5], [0.1 0.4 0.8], [0.85 0.2 0.2]};
styles = {'--', '-.', '-'};
subplot(2,1,1);
hold on;
for m = 1:numel(methodResults)
    r = methodResults(m);
    plot(r.t/60, r.T, 'LineStyle', styles{m}, 'Color', colors{m}, ...
        'LineWidth', 1.8, 'DisplayName', r.name);
end
yline(300, 'k:', 'Setpoint', 'LineWidth', 1.0, 'HandleVisibility', 'off');
grid on; box on; set(gca, 'FontSize', 11);
xlabel('Waktu (menit)'); ylabel('Suhu (degC)');
title('Respon Suhu — Perbandingan Metode Kontrol');
legend('Location', 'southeast');

subplot(2,1,2);
hold on;
for m = 1:numel(methodResults)
    r = methodResults(m);
    e = 300 - r.T;
    plot(r.t/60, e, 'LineStyle', styles{m}, 'Color', colors{m}, ...
        'LineWidth', 1.5, 'DisplayName', r.name);
end
yline(0, 'k-', 'LineWidth', 0.8, 'HandleVisibility', 'off');
grid on; box on; set(gca, 'FontSize', 11);
xlabel('Waktu (menit)'); ylabel('Error (degC)');
title('Sinyal Error — Perbandingan Metode Kontrol');
legend('Location', 'best');
exportgraphics(fig2, fullfile(resultsDir, 'fig2_comparison_methods.png'), 'Resolution', 300);

% ---- Figure 3: bar chart ISE (+ overshoot & settling) ----
fig3 = figure('Name', 'Perbandingan Metrik', 'Color', 'white', ...
    'Units', 'inches', 'Position', [1 1 10 7]);
names = {methodResults.name};
ISE = arrayfun(@(r) r.metrics.ise, methodResults);
Over = arrayfun(@(r) r.metrics.overshoot, methodResults);
Settle = arrayfun(@(r) r.metrics.settling_time, methodResults);

subplot(2,1,1);
bar(ISE, 'FaceColor', [0.2 0.5 0.8]);
set(gca, 'XTickLabel', names, 'FontSize', 11);
ylabel('ISE'); title('Integral Squared Error per Metode');
grid on; box on;

subplot(2,1,2);
yyaxis left; bar((1:3)-0.15, Over, 0.3, 'FaceColor', [0.85 0.33 0.1]);
ylabel('Overshoot (%)');
yyaxis right; bar((1:3)+0.15, Settle, 0.3, 'FaceColor', [0.3 0.7 0.3]);
ylabel('Settling time (s)');
set(gca, 'XTick', 1:3, 'XTickLabel', names, 'FontSize', 11);
title('Overshoot & Settling Time per Metode');
grid on; box on;
exportgraphics(fig3, fullfile(resultsDir, 'fig3_ise_comparison.png'), 'Resolution', 300);

% ---- Tabel ASCII di console + file teks ----
lines = {};
lines{end+1} = '+====================+========+==========+===========+========+';
lines{end+1} = sprintf('| %-18s | %6s | %8s | %9s | %6s |', 'Metode', 'ISE', 'Over(%)', 'Settle(s)', 'SSE(C)');
lines{end+1} = '+====================+========+==========+===========+========+';
for m = 1:numel(methodResults)
    r = methodResults(m);
    lines{end+1} = sprintf('| %-18s | %6.1f | %8.2f | %9.1f | %6.2f |', ...
        r.name, r.metrics.ise, r.metrics.overshoot, r.metrics.settling_time, r.metrics.sse); %#ok<AGROW>
end
lines{end+1} = '+====================+========+==========+===========+========+';

fprintf('\n');
for i = 1:numel(lines), fprintf('%s\n', lines{i}); end
fprintf('\n');

txtFile = fullfile(resultsDir, 'simulation_results.txt');
fid = fopen(txtFile, 'w');
fprintf(fid, '=== HASIL SIMULASI ADAPTIVE PID PYROLYSIS ===\n\n');
fprintf(fid, '--- Skenario 5 Setpoint (Adaptive PID Lengkap) ---\n');
fprintf(fid, '%-10s %10s %10s %10s %10s %10s\n', ...
    'Setpoint', 'Rise(s)', 'Settle(s)', 'Over(%)', 'SSE(C)', 'ISE');
for i = 1:numel(scenarioResults)
    r = scenarioResults(i);
    fprintf(fid, '%-10d %10.1f %10.1f %10.2f %10.2f %10.1f\n', ...
        r.setpoint, r.metrics.rise_time, r.metrics.settling_time, ...
        r.metrics.overshoot, r.metrics.sse, r.metrics.ise);
end
fprintf(fid, '\n--- Perbandingan 3 Metode (setpoint 300 degC) ---\n');
for i = 1:numel(lines), fprintf(fid, '%s\n', lines{i}); end
fprintf(fid, '\nCATATAN: parameter plant (K, tau, L) adalah ASUMSI/PLACEHOLDER,\n');
fprintf(fid, 'belum dikalibrasi dari alat fisik.\n');
fclose(fid);
fprintf('Tabel & metrik tersimpan ke %s\n', txtFile);
end
