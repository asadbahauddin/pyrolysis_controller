function metrics = compute_response_metrics(t, T, setpoint)
% ============================================================
% compute_response_metrics.m
% Hitung rise time (10%-90%), settling time (band 2%), overshoot (%),
% steady-state error (degC), dan ISE (Integral Squared Error).
%
% Judul : Simulasi Adaptive PID Kontrol Suhu Tungku Pyrolysis
% Nama  : [Nama Mahasiswa]
% NIM   : [NIM]
% Tahun : 2025
% ============================================================

T0 = T(1);
span = setpoint - T0;

if span > 0
    peakT = max(T);
    overshoot = max(0, (peakT - setpoint) / abs(span) * 100);
    t10 = T0 + 0.1*span; t90 = T0 + 0.9*span;
    i10 = find(T >= t10, 1, 'first');
    i90 = find(T >= t90, 1, 'first');
else
    peakT = min(T);
    overshoot = max(0, (setpoint - peakT) / abs(span) * 100);
    t10 = T0 - 0.1*abs(span); t90 = T0 - 0.9*abs(span);
    i10 = find(T <= t10, 1, 'first');
    i90 = find(T <= t90, 1, 'first');
end
if isempty(i10) || isempty(i90)
    rise_time = NaN;
else
    rise_time = t(i90) - t(i10);
end

% Settling time: band 2% dari setpoint, titik terakhir keluar band + 1
band = 0.02 * max(abs(setpoint), 1);
outside = abs(T - setpoint) > band;
last_outside = find(outside, 1, 'last');
if isempty(last_outside)
    settling_time = t(1);
else
    settling_time = t(min(last_outside+1, numel(t)));
end

n_tail = max(1, round(0.1*numel(T)));
sse = mean(setpoint - T(end-n_tail+1:end));

e = setpoint - T;
ise = trapz(t, e.^2);

metrics.overshoot = overshoot;
metrics.rise_time = rise_time;
metrics.settling_time = settling_time;
metrics.sse = sse;
metrics.ise = ise;
end
