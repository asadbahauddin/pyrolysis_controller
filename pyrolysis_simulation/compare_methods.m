function results = compare_methods(mdl)
% ============================================================
% compare_methods.m
% Bandingkan 3 metode kontrol di setpoint 300 degC, 3600 detik:
%   1. PID Konvensional      - gs_enable=0, lambda=0 (Kp/Ki/Kd tetap)
%   2. Gain Scheduling saja  - gs_enable=1, lambda=0
%   3. Adaptive PID Lengkap  - gs_enable=1, lambda aktif
%
% Ketiganya memakai MODEL SIMULINK YANG SAMA (pyrolysis_adaptive_pid);
% perbedaan metode diatur lewat variabel workspace yang dibaca oleh
% Constant block gs_enable/lambda_p/i/d di dalam subsystem controller.
%
% Judul : Simulasi Adaptive PID Kontrol Suhu Tungku Pyrolysis
% Nama  : [Nama Mahasiswa]
% NIM   : [NIM]
% Tahun : 2025
% ============================================================

setpoint = 300;
methods = { ...
    'PID Konvensional', 0, 0,     0,      0;
    'Gain Scheduling',  1, 0,     0,      0;
    'Adaptive PID',     1, 0.01,  0.0001, 0.003 ...
};

n = size(methods, 1);
results = struct([]);

for m = 1:n
    name = methods{m,1};
    fprintf('  Menjalankan metode: %s ...\n', name);

    assignin('base', 'setpoint_ws', setpoint);
    assignin('base', 'gs_enable_ws', methods{m,2});
    assignin('base', 'lambda_p_ws', methods{m,3});
    assignin('base', 'lambda_i_ws', methods{m,4});
    assignin('base', 'lambda_d_ws', methods{m,5});

    simOut = sim(mdl, 'StopTime', '3600');

    t = simOut.get('T_out').Time;
    T = simOut.get('T_out').Data;

    metrics = compute_response_metrics(t, T, setpoint);

    r.name = name;
    r.t = t; r.T = T;
    r.metrics = metrics;
    results = [results, r]; %#ok<AGROW>

    fprintf('    ISE=%.1f Over=%.1f%% Settle=%.1fs SSE=%.2fdegC\n', ...
        metrics.ise, metrics.overshoot, metrics.settling_time, metrics.sse);
end
end
