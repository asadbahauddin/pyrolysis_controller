% ============================================================
% build_all_simulink_models.m
% Jalankan sekali di MATLAB (dengan Simulink terpasang) untuk
% menghasilkan ketiga file .slx pada folder simulink/:
%   pid_controller_block.slx, pyrolysis_system.slx, adaptive_pid.slx
%
% Judul  : Implementasi Adaptive PID untuk Kendali Suhu Tungku
%          Pyrolysis Berbasis ESP32
% Nama   : [Nama Mahasiswa]
% NIM    : [NIM]
% Tanggal: [Tanggal]
% ============================================================

fprintf('=== Membangun model Simulink ===\n');
build_pid_controller_block();
build_pyrolysis_system();
build_adaptive_pid();
fprintf('=== Selesai. Buka file .slx di folder ini untuk memeriksa hasilnya. ===\n');
