function build_pid_controller_block()
% ============================================================
% build_pid_controller_block.m
% Membangun & menyimpan pid_controller_block.slx secara terprogram
% (Simulink API), karena file .slx adalah biner dan tidak bisa
% ditulis langsung sebagai teks.
%
% Blok ini berisi PID dengan:
%   - Gain scheduling 3-zona (Kp saja, hard-switch — sesuai firmware)
%   - Anti-windup: clamp integral langsung (back-calculation sederhana)
%   - Derivative pada error (mentah, tanpa filter — sesuai firmware)
%   - Saturasi output 0-100% PWM
%
% Judul  : Implementasi Adaptive PID untuk Kendali Suhu Tungku
%          Pyrolysis Berbasis ESP32
% Nama   : [Nama Mahasiswa]
% NIM    : [NIM]
% Tanggal: [Tanggal]
%
% Jalankan fungsi ini di MATLAB (dengan Simulink terpasang) untuk
% menghasilkan pid_controller_block.slx pada folder yang sama.
% ============================================================

mdl = 'pid_controller_block';
thisDir = fileparts(mfilename('fullpath'));
outFile = fullfile(thisDir, [mdl '.slx']);

if bdIsLoaded(mdl), close_system(mdl, 0); end
if exist(outFile, 'file'), delete(outFile); end

new_system(mdl);
open_system(mdl);

% --- Port masukan/keluaran subsystem (In1/Out1 ada di library
%     "Ports & Subsystems", BUKAN Sources/Sinks) ---
add_block('simulink/Ports & Subsystems/In1', [mdl '/setpoint'], 'Position', [30 40 60 60]);
add_block('simulink/Ports & Subsystems/In1', [mdl '/measurement'], 'Position', [30 100 60 120]);
add_block('simulink/Ports & Subsystems/In1', [mdl '/Kp_base'], 'Position', [30 160 60 180]);
add_block('simulink/Ports & Subsystems/In1', [mdl '/Ki_base'], 'Position', [30 220 60 240]);
add_block('simulink/Ports & Subsystems/In1', [mdl '/Kd_base'], 'Position', [30 280 60 300]);
set_param([mdl '/setpoint'], 'Port', '1');
set_param([mdl '/measurement'], 'Port', '2');
set_param([mdl '/Kp_base'], 'Port', '3');
set_param([mdl '/Ki_base'], 'Port', '4');
set_param([mdl '/Kd_base'], 'Port', '5');

% --- Konstanta sample time (Ts), sesuaikan dengan Fixed-step size solver ---
add_block('simulink/Sources/Constant', [mdl '/Ts'], ...
    'Value', '0.1', 'Position', [30 340 60 360]);

% --- MATLAB Function: PID + gain scheduling + anti-windup ---
% PENTING: isi Script LANGSUNG setelah blok dibuat, SEBELUM add_line
% menyambung port-portnya -- port block MATLAB Function baru mengikuti
% signature function di Script-nya (default hanya 1 in/1 out sebelum
% Script diisi), jadi urutan ini wajib supaya add_line tidak gagal.
add_block('simulink/User-Defined Functions/MATLAB Function', ...
    [mdl '/PID_GainSchedule'], 'Position', [150 100 340 300]);
setPidChartScript(mdl);

% --- Saturasi output PWM 0-100% ---
add_block('simulink/Discontinuities/Saturation', [mdl '/Saturation_PWM'], ...
    'Position', [400 180 430 210], 'UpperLimit', '100', 'LowerLimit', '0');

% --- Port keluaran ---
add_block('simulink/Ports & Subsystems/Out1', [mdl '/u_pwm'], 'Position', [480 180 510 210]);

% --- Sambungan ---
add_line(mdl, 'setpoint/1', 'PID_GainSchedule/1', 'autorouting', 'on');
add_line(mdl, 'measurement/1', 'PID_GainSchedule/2', 'autorouting', 'on');
add_line(mdl, 'Kp_base/1', 'PID_GainSchedule/3', 'autorouting', 'on');
add_line(mdl, 'Ki_base/1', 'PID_GainSchedule/4', 'autorouting', 'on');
add_line(mdl, 'Kd_base/1', 'PID_GainSchedule/5', 'autorouting', 'on');
add_line(mdl, 'Ts/1', 'PID_GainSchedule/6', 'autorouting', 'on');
add_line(mdl, 'PID_GainSchedule/1', 'Saturation_PWM/1', 'autorouting', 'on');
add_line(mdl, 'Saturation_PWM/1', 'u_pwm/1', 'autorouting', 'on');

Simulink.BlockDiagram.arrangeSystem(mdl);
save_system(mdl, outFile);
close_system(mdl, 0);
fprintf('[build_pid_controller_block] Tersimpan: %s\n', outFile);
end

% ------------------------------------------------------------
function setPidChartScript(mdl)
% Mengisi kode blok MATLAB Function PID_GainSchedule.
lines = { ...
'function u = pidGainScheduledFcn(setpoint, measurement, Kp_base, Ki_base, Kd_base, Ts)'
'%#codegen'
'% PID dengan anti-windup (clamp integral) + gain scheduling 3-zona.'
'% Meniru persis pid_controller.cpp pada firmware ESP32:'
'%   - hanya Kp yang dijadwal per zona suhu (dingin/tengah/panas)'
'%   - integral di-clamp SEBELUM dipakai (anti-windup)'
'%   - derivative memakai error mentah (tanpa filter)'
'persistent I e_prev first_call'
'if isempty(I)'
'    I = 0; e_prev = 0; first_call = true;'
'end'
''
'if measurement < 200'
'    Kp = Kp_base * 1.3;'
'elseif measurement <= 400'
'    Kp = Kp_base * 1.0;'
'else'
'    Kp = Kp_base * 0.7;'
'end'
'Ki = Ki_base;'
'Kd = Kd_base;'
''
'err = setpoint - measurement;'
''
'I = I + Ki*err*Ts;'
'I_MAX = 100; I_MIN = -100;'
'if I > I_MAX, I = I_MAX; end'
'if I < I_MIN, I = I_MIN; end'
''
'if first_call'
'    deriv = 0;'
'    first_call = false;'
'else'
'    deriv = Kd * (err - e_prev) / Ts;'
'end'
'e_prev = err;'
''
'u = Kp*err + I + deriv;'
'end' ...
};
scriptCode = strjoin(lines, newline);

% Nama parameter yang BENAR adalah 'SystemSampleTime' pada BLOK (bukan
% 'SampleTime' -- itu bukan parameter valid utk block wrapper bertipe
% SubSystem ini). Wajib diisi diskrit karena blok ini pakai persistent
% state.
rt = sfroot;
chartObj = rt.find('-isa', 'Stateflow.EMChart', 'Path', [mdl '/PID_GainSchedule']);
chartObj.Script = scriptCode;
set_param([mdl '/PID_GainSchedule'], 'SystemSampleTime', '0.1');
end
