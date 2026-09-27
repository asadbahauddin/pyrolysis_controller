function api = compare_methods()
% ============================================================
% compare_methods.m
% Perbandingan 5 metode kontrol suhu tungku pyrolysis
%
% Judul  : Implementasi Adaptive PID untuk Kendali Suhu Tungku
%          Pyrolysis Berbasis ESP32
% Nama   : [Nama Mahasiswa]
% NIM    : [NIM]
% Tanggal: [Tanggal]
%
% Metode yang dibandingkan:
%   1. PID Konvensional      - DEFAULT_KP/KI/KD firmware, tanpa tuning
%   2. PID + Z-N Step        - Z-N open-loop step response, fixed
%   3. PID + Z-N Relay       - auto-tune relay (spt firmware), fixed
%   4. PID + Gain Scheduling - Z-N relay + zona Kp firmware (hard switch)
%   5. Adaptive PID          - Gain Scheduling + coordinate descent
%                              (spt firmware setelah beberapa run)
% ============================================================

api.runAllMethods         = @runAllMethods;
api.computeResponseMetrics = @computeResponseMetrics;
api.printComparisonTable  = @printComparisonTable;
end

% ------------------------------------------------------------
function results = runAllMethods(plant_params, setpoint, T_sim)
plant = plant_model();
pidm  = pid_design();
zn    = ziegler_nichols();
cd_   = coordinate_descent();

opts_base.dt = 0.5;
opts_base.T_sim = T_sim;
opts_base.T_init = plant_params.T_ambient;
opts_base.setpoint = setpoint;
opts_base.noise = false;

methods = struct('name',{},'Kp',{},'Ki',{},'Kd',{},'mode',{});

% --- 1. Konvensional ---
[Kp1,Ki1,Kd1] = pidm.firmwareDefaultGains();
methods(1) = struct('name','Konvensional','Kp',Kp1,'Ki',Ki1,'Kd',Kd1,'mode','fixed');

% --- 2. Z-N Step (open loop) ---
[Kp2,Ki2,Kd2] = pidm.znOpenLoop(plant_params.K_plant, plant_params.tau_plant, plant_params.Td_plant);
methods(2) = struct('name','ZN Step','Kp',Kp2,'Ki',Ki2,'Kd',Kd2,'mode','fixed');

% --- 3. Z-N Relay (auto-tune, seperti firmware) ---
fprintf('  [compare_methods] Menjalankan relay auto-tune untuk metode 3-5...\n');
[Kp3,Ki3,Kd3,~,~] = zn.runRelayAutoTune(plant_params, setpoint, 4, false);
methods(3) = struct('name','ZN Relay','Kp',Kp3,'Ki',Ki3,'Kd',Kd3,'mode','fixed');

% --- 4. Gain Scheduling (gain relay + zona Kp firmware) ---
methods(4) = struct('name','Gain Schedule','Kp',Kp3,'Ki',Ki3,'Kd',Kd3,'mode','zone');

% --- 5. Adaptive PID (Gain Scheduling + coordinate descent) ---
fprintf('  [compare_methods] Menjalankan coordinate descent untuk metode 5...\n');
[Kp5,Ki5,Kd5,~] = cd_.coordinateDescent(plant_params, setpoint, Kp3, Ki3, Kd3, 15, 0.05);
methods(5) = struct('name','Adaptive PID','Kp',Kp5,'Ki',Ki5,'Kd',Kd5,'mode','zone');

results = struct([]);
for i = 1:numel(methods)
    m = methods(i);
    ctrl = pidm.defaultCtrl(m.Kp, m.Ki, m.Kd, m.mode);
    out = plant.simulateClosedLoop(plant_params, ctrl, opts_base);
    metrics = computeResponseMetrics(out.t, out.T, setpoint);

    r.name = m.name; r.Kp = m.Kp; r.Ki = m.Ki; r.Kd = m.Kd;
    r.t = out.t; r.T = out.T; r.e = out.e; r.u = out.u;
    r.ISE = out.ISE; r.IAE = out.IAE; r.ITAE = out.ITAE;
    r.overshoot = metrics.overshoot; r.rise_time = metrics.rise_time;
    r.settling_time = metrics.settling_time; r.sse = metrics.sse;
    results = [results, r]; %#ok<AGROW>
end
end

% ------------------------------------------------------------
function metrics = computeResponseMetrics(t, T, setpoint)
% overshoot (%), rise time (10%-90%, detik), settling time (masuk &
% tetap dalam +-2% band, detik), steady-state error (degC, rata-rata
% 10% waktu terakhir).
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

metrics.overshoot = overshoot;
metrics.rise_time = rise_time;
metrics.settling_time = settling_time;
metrics.sse = sse;
end

% ------------------------------------------------------------
function printComparisonTable(results)
fprintf('+============================================================+\n');
fprintf('|            PERBANDINGAN METODE KONTROL PID                |\n');
fprintf('+===================+========+========+========+===========+\n');
fprintf('| %-17s | %6s | %6s | %6s | %9s |\n', 'Metode', 'ISE', 'Over(%)', 'Settle(s)', 'SSE(degC)');
fprintf('+===================+========+========+========+===========+\n');
for i = 1:numel(results)
    r = results(i);
    fprintf('| %-17s | %6.1f | %6.1f | %6.1f | %9.2f |\n', ...
        r.name, r.ISE, r.overshoot, r.settling_time, r.sse);
end
fprintf('+===================+========+========+========+===========+\n');
fprintf('\nDetail metrik lengkap:\n');
fprintf('%-17s %10s %10s %10s %8s %10s %10s\n', ...
    'Metode','ISE','IAE','ITAE','Over(%)','Rise(s)','Settle(s)');
for i = 1:numel(results)
    r = results(i);
    fprintf('%-17s %10.2f %10.2f %10.0f %8.1f %10.1f %10.1f\n', ...
        r.name, r.ISE, r.IAE, r.ITAE, r.overshoot, r.rise_time, r.settling_time);
end
end
