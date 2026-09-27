function api = plot_results()
% ============================================================
% plot_results.m
% Semua fungsi visualisasi (publikasi-ready, 300 DPI)
%
% Judul  : Implementasi Adaptive PID untuk Kendali Suhu Tungku
%          Pyrolysis Berbasis ESP32
% Nama   : [Nama Mahasiswa]
% NIM    : [NIM]
% Tanggal: [Tanggal]
% ============================================================

api.plotStepResponse       = @plotStepResponse;
api.plotComparisonAll      = @plotComparisonAll;
api.plotErrorComparison    = @plotErrorComparison;
api.plotPerformanceMetrics = @plotPerformanceMetrics;
api.plotPIDParams          = @plotPIDParams;
api.plotEfficiencyMap      = @plotEfficiencyMap;
api.plotAutoTuneProcess    = @plotAutoTuneProcess;
api.plotGainScheduleEffect = @plotGainScheduleEffect;
api.plotISEConvergence     = @plotISEConvergence;
api.plotDisturbanceRejection = @plotDisturbanceRejection;
api.saveFigure             = @saveFigure;
end

% ------------------------------------------------------------
function applyStyle(ax)
if nargin < 1, ax = gca; end
set(ax, 'FontSize', 12, 'LineWidth', 1);
grid(ax, 'on'); box(ax, 'on');
end

function saveFigure(fig, filepath)
% Simpan figure sebagai PNG 300 DPI, ukuran 10x8 inci (siap publikasi).
set(fig, 'Color', 'white');
set(fig, 'Units', 'Inches', 'Position', [1 1 10 8]);
exportgraphics(fig, filepath, 'Resolution', 300);
end

% ------------------------------------------------------------
function fig = plotStepResponse(t, T, setpoint, method_name)
fig = figure('Name', ['Step Response - ' method_name], 'Color', 'white');
plot(t, T, 'b-', 'LineWidth', 1.8); hold on;
yline(setpoint, 'k--', 'LineWidth', 1.2);
applyStyle();
xlabel('Waktu (s)'); ylabel('Suhu (degC)');
title(['Respon Suhu - ' method_name]);
legend('Suhu aktual', 'Setpoint', 'Location', 'best');
end

% ------------------------------------------------------------
function fig = plotComparisonAll(results, setpoint)
% Figure 2: respon suhu semua metode dalam satu grafik
fig = figure('Name', 'Perbandingan Respon Suhu', 'Color', 'white', 'Position', [50 50 1000 650]);
colors = lines(numel(results));
styles = {'-','--',':','-.','-'};
hold on;
for i = 1:numel(results)
    r = results(i);
    plot(r.t/60, r.T, 'LineStyle', styles{mod(i-1,numel(styles))+1}, ...
        'Color', colors(i,:), 'LineWidth', 1.8, 'DisplayName', r.name);
end
if nargin >= 2 && ~isempty(setpoint)
    yline(setpoint, 'k:', 'Setpoint', 'LineWidth', 1.2, 'HandleVisibility', 'off');
end
applyStyle();
xlabel('Waktu (menit)'); ylabel('Suhu (degC)');
title('Perbandingan Respon Suhu — Semua Metode Kontrol');
legend('Location', 'southeast');
end

% ------------------------------------------------------------
function fig = plotErrorComparison(results)
% Figure 3: error vs waktu semua metode
fig = figure('Name', 'Perbandingan Error', 'Color', 'white', 'Position', [50 50 1000 650]);
colors = lines(numel(results));
styles = {'-','--',':','-.','-'};
hold on;
for i = 1:numel(results)
    r = results(i);
    plot(r.t/60, r.e, 'LineStyle', styles{mod(i-1,numel(styles))+1}, ...
        'Color', colors(i,:), 'LineWidth', 1.5, 'DisplayName', r.name);
end
yline(0, 'k-', 'LineWidth', 0.8);
applyStyle();
xlabel('Waktu (menit)'); ylabel('Error = Setpoint - Suhu (degC)');
title('Perbandingan Error — Semua Metode Kontrol');
legend('Location', 'best');
end

% ------------------------------------------------------------
function fig = plotPerformanceMetrics(results)
% Figure 4: bar chart ISE/IAE/ITAE (dinormalisasi) + overshoot/rise/settle
fig = figure('Name', 'Metrik Performa', 'Color', 'white', 'Position', [50 50 1100 700]);
names = {results.name};
n = numel(results);

ISE = [results.ISE]; IAE = [results.IAE]; ITAE = [results.ITAE];
ISE_n = ISE / max(ISE); IAE_n = IAE / max(IAE); ITAE_n = ITAE / max(ITAE);

subplot(2,1,1);
bar([ISE_n; IAE_n; ITAE_n]');
applyStyle();
set(gca, 'XTickLabel', names);
ylabel('Nilai ternormalisasi (0-1)');
title('ISE, IAE, ITAE (ternormalisasi terhadap nilai maksimum)');
legend('ISE', 'IAE', 'ITAE', 'Location', 'best');

subplot(2,1,2);
over = [results.overshoot]; rise = [results.rise_time]; settle = [results.settling_time];
yyaxis left;
bar(1:n, over, 0.25, 'FaceColor', [0.85 0.33 0.1]);
ylabel('Overshoot (%)');
yyaxis right;
hold on;
bar((1:n)+0.28, rise, 0.25, 'FaceColor', [0.1 0.5 0.8]);
bar((1:n)+0.56, settle, 0.25, 'FaceColor', [0.3 0.7 0.3]);
ylabel('Waktu (s)');
set(gca, 'XTick', (1:n)+0.28, 'XTickLabel', names);
applyStyle();
title('Overshoot, Rise Time, Settling Time');
legend('Overshoot(%)', 'Rise time(s)', 'Settling time(s)', 'Location', 'best');
end

% ------------------------------------------------------------
function fig = plotPIDParams(iterations, Kp_hist, Ki_hist, Kd_hist)
% Figure 5: evolusi Kp, Ki, Kd selama coordinate descent
fig = figure('Name', 'Konvergensi PID (Coordinate Descent)', 'Color', 'white', 'Position', [50 50 900 800]);

subplot(3,1,1);
plot(iterations, Kp_hist, 'b-o', 'LineWidth', 1.5, 'MarkerSize', 4);
applyStyle(); ylabel('Kp'); title('Konvergensi Kp');

subplot(3,1,2);
plot(iterations, Ki_hist, 'r-o', 'LineWidth', 1.5, 'MarkerSize', 4);
applyStyle(); ylabel('Ki'); title('Konvergensi Ki');

subplot(3,1,3);
plot(iterations, Kd_hist, 'g-o', 'LineWidth', 1.5, 'MarkerSize', 4);
applyStyle(); ylabel('Kd'); xlabel('Iterasi / Run ke-'); title('Konvergensi Kd');
end

% ------------------------------------------------------------
function fig = plotEfficiencyMap(score_map)
% Figure 7: peta efisiensi (score, produktivitas, energi) vs setpoint
fig = figure('Name', 'Efficiency Map', 'Color', 'white', 'Position', [50 50 900 800]);
sp = score_map.setpoint;

subplot(3,1,1);
b = bar(sp, score_map.score, 'FaceColor', 'flat');
cdata = repmat([0.2 0.5 0.8], numel(sp), 1);
cdata(score_map.idx_best, :) = [0.85 0.2 0.2];
b.CData = cdata;
applyStyle();
xlabel('Setpoint (degC)'); ylabel('Score');
title(sprintf('Efficiency Score vs Setpoint (optimal = %.0f degC)', sp(score_map.idx_best)));

subplot(3,1,2);
bar(sp, score_map.productivity, 'FaceColor', [0.3 0.7 0.4]);
applyStyle();
xlabel('Setpoint (degC)'); ylabel('Produktivitas (mL/jam)');
title('Estimasi Produktivitas Minyak Pirolisis');

subplot(3,1,3);
bar(sp, score_map.avg_pwm, 'FaceColor', [0.9 0.6 0.2]);
applyStyle();
xlabel('Setpoint (degC)'); ylabel('Rata-rata PWM (%)');
title('Estimasi Konsumsi Energi (Rata-rata PWM Fan)');
end

% ------------------------------------------------------------
function fig = plotAutoTuneProcess(trace)
% Figure 1: proses relay auto-tune Ziegler-Nichols
fig = figure('Name', 'Relay Auto-tune Response', 'Color', 'white', 'Position', [50 50 900 700]);

subplot(2,1,1);
plot(trace.t, trace.T, 'b-', 'LineWidth', 1.5); hold on;
plot(trace.peak_t, trace.peaks, 'r^', 'MarkerFaceColor', 'r', 'MarkerSize', 8);
plot(trace.valley_t, trace.valleys, 'gv', 'MarkerFaceColor', 'g', 'MarkerSize', 8);
yline(trace.setpoint, 'k--');
applyStyle();
xlabel('Waktu (s)'); ylabel('Suhu (degC)');
title('Proses Relay Auto-tune Ziegler-Nichols');
legend('Suhu tungku', 'Peak', 'Valley', 'Location', 'best');
txt = sprintf('Tu = %.1f s | Ku = %.3f\nKp=%.3f  Ki=%.4f  Kd=%.3f', ...
    trace.Tu, trace.Ku, trace.Kp, trace.Ki, trace.Kd);
xl = xlim; yl = ylim;
text(xl(1)+0.02*diff(xl), yl(2)-0.05*diff(yl), txt, 'VerticalAlignment', 'top', ...
    'BackgroundColor', 'white', 'EdgeColor', 'black', 'FontSize', 10);

subplot(2,1,2);
stairs(trace.t, trace.u, 'm-', 'LineWidth', 1.5);
applyStyle(); ylim([-5 105]);
xlabel('Waktu (s)'); ylabel('Fan PWM (%)');
title('Sinyal Relay (ON/OFF)');
end

% ------------------------------------------------------------
function fig = plotGainScheduleEffect(t, T, Kp_hist, zone_hist)
% Figure 6: efek gain scheduling — suhu & Kp aktif vs waktu, dengan
% latar belakang warna per zona
fig = figure('Name', 'Efek Gain Scheduling', 'Color', 'white', 'Position', [50 50 900 700]);

zoneColor = containers.Map({'cold','mid','hot'}, ...
    {[0.75 0.85 1.0], [0.8 1.0 0.8], [1.0 0.8 0.8]});

ax1 = subplot(2,1,1);
hold(ax1, 'on');
shadeZones(ax1, t, zone_hist, zoneColor, [min(T)-5, max(T)+5]);
plot(ax1, t, T, 'b-', 'LineWidth', 1.8);
applyStyle(ax1);
xlabel(ax1, 'Waktu (s)'); ylabel(ax1, 'Suhu (degC)');
title(ax1, 'Suhu Tungku dan Zona Aktif');

ax2 = subplot(2,1,2);
hold(ax2, 'on');
shadeZones(ax2, t, zone_hist, zoneColor, [min(Kp_hist)-0.1, max(Kp_hist)+0.1]);
plot(ax2, t, Kp_hist, 'r-', 'LineWidth', 1.8);
applyStyle(ax2);
xlabel(ax2, 'Waktu (s)'); ylabel(ax2, 'Kp aktif');
title(ax2, 'Kp Aktif (Gain Scheduling 3-Zona)');
end

function shadeZones(ax, t, zone_hist, zoneColor, ylims)
zvals = string(zone_hist);
changeIdx = [1; find(zvals(2:end) ~= zvals(1:end-1))+1; numel(zvals)+1];
for i = 1:numel(changeIdx)-1
    idxStart = changeIdx(i); idxEnd = min(changeIdx(i+1)-1, numel(t));
    z = char(zvals(idxStart));
    if ~isKey(zoneColor, z), continue; end
    xs = [t(idxStart), t(idxEnd), t(idxEnd), t(idxStart)];
    ys = [ylims(1), ylims(1), ylims(2), ylims(2)];
    patch(ax, xs, ys, zoneColor(z), 'EdgeColor', 'none', 'FaceAlpha', 0.5, 'HandleVisibility', 'off');
end
ylim(ax, ylims);
end

% ------------------------------------------------------------
function fig = plotISEConvergence(ise_per_run, run_numbers)
% Figure 8: ISE per run (harusnya menurun — bukti sistem belajar)
fig = figure('Name', 'ISE per Run (Multi-Run Learning)', 'Color', 'white', 'Position', [50 50 850 600]);
scatter(run_numbers, ise_per_run, 70, 'filled'); hold on;
p = polyfit(run_numbers, ise_per_run, 1);
trend = polyval(p, run_numbers);
plot(run_numbers, trend, 'r--', 'LineWidth', 1.8);
applyStyle();
xlabel('Run ke-'); ylabel('ISE');
title('ISE per Run — Adaptive Learning');
legend('ISE per run', 'Tren linear', 'Location', 'best');
if p(1) < 0
    text(run_numbers(1), max(ise_per_run), '  Learning improves over time', ...
        'FontSize', 11, 'Color', [0 0.5 0], 'FontWeight', 'bold', 'VerticalAlignment', 'top');
end
end

% ------------------------------------------------------------
function fig = plotDisturbanceRejection(results_disturbance, t_disturbance)
% Figure 9: recovery dari gangguan (+20 degC pada t = t_disturbance)
fig = figure('Name', 'Disturbance Rejection', 'Color', 'white', 'Position', [50 50 1000 650]);
colors = lines(numel(results_disturbance));
hold on;
for i = 1:numel(results_disturbance)
    r = results_disturbance(i);
    plot(r.t, r.T, 'Color', colors(i,:), 'LineWidth', 1.8, 'DisplayName', r.name);
end
xline(t_disturbance, 'k--', 'Gangguan +20degC', 'LabelVerticalAlignment', 'bottom');
applyStyle();
xlabel('Waktu (s)'); ylabel('Suhu (degC)');
title('Disturbance Rejection — Pemulihan dari Gangguan +20 degC');
legend('Location', 'best');
end
