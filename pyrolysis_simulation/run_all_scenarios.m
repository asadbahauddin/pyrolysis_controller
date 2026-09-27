function results = run_all_scenarios(mdl)
% ============================================================
% run_all_scenarios.m
% Simulasikan 5 setpoint terpisah (200/250/300/350/400 degC) dengan
% Adaptive PID lengkap (gain scheduling + adaptasi lambda), masing-
% masing 1800 detik, suhu awal 30 degC (reset otomatis tiap sim()).
%
% Judul : Simulasi Adaptive PID Kontrol Suhu Tungku Pyrolysis
% Nama  : [Nama Mahasiswa]
% NIM   : [NIM]
% Tahun : 2025
% ============================================================

sp_list = [200 250 300 350 400];
results = struct([]);

for i = 1:numel(sp_list)
    sp = sp_list(i);
    fprintf('  Menjalankan setpoint %d degC ...\n', sp);

    assignin('base', 'setpoint_ws', sp);
    assignin('base', 'gs_enable_ws', 1);
    assignin('base', 'lambda_p_ws', 0.01);
    assignin('base', 'lambda_i_ws', 0.0001);
    assignin('base', 'lambda_d_ws', 0.003);

    simOut = sim(mdl, 'StopTime', '1800');

    t = simOut.get('T_out').Time;
    T = simOut.get('T_out').Data;

    metrics = compute_response_metrics(t, T, sp);

    r.setpoint = sp;
    r.t = t; r.T = T;
    r.metrics = metrics;
    results = [results, r]; %#ok<AGROW>

    fprintf('    Rise=%.1fs Settle=%.1fs Over=%.1f%% SSE=%.2fdegC ISE=%.1f\n', ...
        metrics.rise_time, metrics.settling_time, metrics.overshoot, metrics.sse, metrics.ise);
end
end
