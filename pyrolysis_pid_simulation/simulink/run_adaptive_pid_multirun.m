function run_adaptive_pid_multirun(n_runs)
% ============================================================
% run_adaptive_pid_multirun.m
% Menjalankan adaptive_pid.slx berulang kali (multi-run learning),
% meng-update Kp/Ki/Kd di ANTARA run memakai coordinateDescentReplay()
% dari coordinate_descent.m -- PERSIS alur learning_system.cpp firmware
% (jalankan run -> log error & PWM -> replay coordinate descent ->
% simpan gain baru -> pakai di run berikutnya).
%
% Judul  : Implementasi Adaptive PID untuk Kendali Suhu Tungku
%          Pyrolysis Berbasis ESP32
% Nama   : [Nama Mahasiswa]
% NIM    : [NIM]
% Tanggal: [Tanggal]
%
% Syarat: adaptive_pid.slx sudah dibuat (jalankan build_adaptive_pid.m
% atau build_all_simulink_models.m terlebih dahulu).
% ============================================================
if nargin < 1, n_runs = 5; end

thisDir = fileparts(mfilename('fullpath'));
projRoot = fileparts(thisDir);
addpath(projRoot);
cd_ = coordinate_descent();

mdl = 'adaptive_pid';
load_system(fullfile(thisDir, [mdl '.slx']));

Kp = 2.0; Ki = 0.1; Kd = 10.0; % mulai dari default firmware
assignin('base', 'Kp_current', Kp);
assignin('base', 'Ki_current', Ki);
assignin('base', 'Kd_current', Kd);
assignin('base', 'prev_run_T', [0 30; 3600 30]);

ise_per_run = zeros(n_runs, 1);

for run = 1:n_runs
    simOut = sim(mdl);

    t_out = simOut.get('T_out').Time;
    T_out = simOut.get('T_out').Data;
    e_out = simOut.get('e_out').Data;
    u_out = simOut.get('u_out').Data;

    dt = t_out(2) - t_out(1);
    ise_per_run(run) = sum(e_out.^2) * dt;

    fprintf('Run %d: ISE = %.2f (Kp=%.4f Ki=%.4f Kd=%.4f)\n', run, ise_per_run(run), Kp, Ki, Kd);

    run_log.t = t_out; run_log.e = e_out; run_log.u = u_out;
    [Kp, Ki, Kd] = cd_.coordinateDescentReplay(run_log, Kp, Ki, Kd, 0.05);

    assignin('base', 'Kp_current', Kp);
    assignin('base', 'Ki_current', Ki);
    assignin('base', 'Kd_current', Kd);
    assignin('base', 'prev_run_T', [t_out, T_out]); % overlay run berikutnya
end

fprintf('\nGain akhir setelah %d run: Kp=%.4f Ki=%.4f Kd=%.4f\n', n_runs, Kp, Ki, Kd);
fprintf('ISE per run: '); fprintf('%.2f ', ise_per_run); fprintf('\n');

figure('Name', 'Adaptive PID Multi-Run (Simulink)', 'Color', 'white');
plot(1:n_runs, ise_per_run, '-o', 'LineWidth', 1.8, 'MarkerSize', 6);
grid on; box on; set(gca, 'FontSize', 12);
xlabel('Run ke-'); ylabel('ISE');
title('ISE per Run — adaptive\_pid.slx (Simulink)');

close_system(mdl, 0);
end
