function build_simulink_model()
% ============================================================
% build_simulink_model.m
% Membangun & menyimpan pyrolysis_adaptive_pid.slx secara terprogram
% lewat Simulink API (new_system/add_block/add_line/save_system).
%
% Judul : Simulasi Adaptive PID Kontrol Suhu Tungku Pyrolysis
% Nama  : [Nama Mahasiswa]
% NIM   : [NIM]
% Tahun : 2025
%
% ================================================================
% PENTING -- SEMUA PARAMETER PLANT (K, tau, L) DI BAWAH INI ADALAH
% ASUMSI/PLACEHOLDER untuk verifikasi STRUKTUR simulasi, BUKAN hasil
% kalibrasi dari alat fisik. Wajib dikalibrasi ulang setelah tungku
% pyrolysis sungguhan diuji (identifikasi step response nyata).
% ================================================================
%
% Struktur model (top level):
%   Setpoint -> [AdaptivePIDController subsystem] -> Saturation (0-100%)
%             -> ActuatorLag (first-order, tau=0.5s, respons mekanis fan)
%             -> [Plant subsystem: TransportDelay 15s + FOPDT nonlinear
%                 per-zona + Band-Limited White Noise sensor]
%             -> (sinyal MENTAH/noisy) -> Scope + To Workspace (logging
%                 realistis, TIDAK disaring)
%             -> (sinyal MENTAH) -> MeasurementFilter (low-pass tau_f=5s)
%                 -> umpan balik ke AdaptivePIDController SAJA (meniru
%                 software averaging MAX6675 di firmware ESP32)
%
% Subsystem AdaptivePIDController membaca Kp_base/Ki_base/Kd_base,
% lambda_p/i/d, dan gs_enable dari VARIABEL WORKSPACE (bukan angka
% literal) supaya run_all_scenarios.m / compare_methods.m bisa
% mengubahnya sebelum tiap panggilan sim() tanpa membangun ulang model.
% ============================================================

mdl = 'pyrolysis_adaptive_pid';
thisDir = fileparts(mfilename('fullpath'));
outFile = fullfile(thisDir, [mdl '.slx']);

if bdIsLoaded(mdl), close_system(mdl, 0); end
if exist(outFile, 'file'), delete(outFile); end

% Nilai default di base workspace supaya model bisa dibuka & di-run manual
% (diturunkan dari 0.02/0.001/0.01 -- terlalu agresif utk noise sensor,
% menyebabkan osilasi persisten di steady state)
setDefaultIfMissing('setpoint_ws', 300);
setDefaultIfMissing('gs_enable_ws', 1);
setDefaultIfMissing('lambda_p_ws', 0.01);
setDefaultIfMissing('lambda_i_ws', 0.0001);
setDefaultIfMissing('lambda_d_ws', 0.003);
setDefaultIfMissing('noise_variance_ws', 2.25);

new_system(mdl);
open_system(mdl);

% ---- Sumber ----
add_block('simulink/Sources/Constant', [mdl '/Setpoint'], ...
    'Value', 'setpoint_ws', 'Position', [30 140 70 160]);

% ---- Subsystem: Adaptive PID Controller ----
add_block('simulink/Ports & Subsystems/Subsystem', [mdl '/AdaptivePIDController'], ...
    'Position', [140 100 320 220]);
buildControllerSubsystem([mdl '/AdaptivePIDController']);

% ---- Saturasi output PWM 0-100% (bagian aktuator) ----
add_block('simulink/Discontinuities/Saturation', [mdl '/Saturation_PWM'], ...
    'Position', [370 148 400 172], 'UpperLimit', '100', 'LowerLimit', '0');

% ---- Lag mekanis fan (first-order, tau ~0.5 s) ----
add_block('simulink/Continuous/Transfer Fcn', [mdl '/ActuatorLag'], ...
    'Position', [440 148 500 172], 'Numerator', '[1]', 'Denominator', '[0.5 1]');

% ---- Subsystem: Plant (Transport Delay + FOPDT nonlinear + noise) ----
add_block('simulink/Ports & Subsystems/Subsystem', [mdl '/Plant'], 'Position', [560 130 720 210]);
buildPlantSubsystem([mdl '/Plant']);

% ---- Filter pengukuran (low-pass, tau_f=5.0s) sebelum umpan balik ke
%      controller -- redam noise sensor supaya tidak memicu derivative
%      kick / adaptasi lambda berlebihan. ----
add_block('simulink/Continuous/Transfer Fcn', [mdl '/MeasurementFilter'], ...
    'Position', [750 148 800 172], 'Numerator', '[1]', 'Denominator', '[5 1]');

% ---- Sinks ----
add_block('simulink/Sinks/Scope', [mdl '/Scope'], 'Position', [790 100 830 140]);
set_param([mdl '/Scope'], 'NumInputPorts', '2');
add_block('simulink/Sinks/To Workspace', [mdl '/ToWS_T'], 'Position', [790 200 850 230], ...
    'VariableName', 'T_out', 'SaveFormat', 'Timeseries');
add_block('simulink/Sinks/To Workspace', [mdl '/ToWS_u'], 'Position', [560 260 620 290], ...
    'VariableName', 'u_out', 'SaveFormat', 'Timeseries');

% ---- Display Kp, Ki, Kd aktif real-time ----
add_block('simulink/Sinks/Display', [mdl '/Disp_Kp'], 'Position', [140 260 220 285]);
add_block('simulink/Sinks/Display', [mdl '/Disp_Ki'], 'Position', [140 300 220 325]);
add_block('simulink/Sinks/Display', [mdl '/Disp_Kd'], 'Position', [140 340 220 365]);

% ---- Sambungan ----
add_line(mdl, 'Setpoint/1', 'AdaptivePIDController/1', 'autorouting', 'on');
add_line(mdl, 'AdaptivePIDController/1', 'Saturation_PWM/1', 'autorouting', 'on');
add_line(mdl, 'AdaptivePIDController/2', 'Disp_Kp/1', 'autorouting', 'on');
add_line(mdl, 'AdaptivePIDController/3', 'Disp_Ki/1', 'autorouting', 'on');
add_line(mdl, 'AdaptivePIDController/4', 'Disp_Kd/1', 'autorouting', 'on');
add_line(mdl, 'Saturation_PWM/1', 'ActuatorLag/1', 'autorouting', 'on');
add_line(mdl, 'Saturation_PWM/1', 'ToWS_u/1', 'autorouting', 'on');
add_line(mdl, 'ActuatorLag/1', 'Plant/1', 'autorouting', 'on');
% PENTING (arsitektur): sinyal MENTAH (dengan noise) dipakai untuk
% Scope/logging (realistis, sama seperti data mentah MAX6675 di ESP32),
% sedangkan HANYA jalur ke controller yang melewati MeasurementFilter --
% meniru software averaging yang firmware lakukan sebelum dipakai PID,
% tanpa menyembunyikan noise sungguhan dari tampilan/analisis.
add_line(mdl, 'Plant/1', 'MeasurementFilter/1', 'autorouting', 'on');
add_line(mdl, 'MeasurementFilter/1', 'AdaptivePIDController/2', 'autorouting', 'on');
add_line(mdl, 'Plant/1', 'Scope/1', 'autorouting', 'on');
add_line(mdl, 'Setpoint/1', 'Scope/2', 'autorouting', 'on');
add_line(mdl, 'Plant/1', 'ToWS_T/1', 'autorouting', 'on');

% ---- Konfigurasi solver: fixed-step (perlu utk blok MATLAB Function
%      ber-state di dalam Controller & Plant). FixedStep=0.05 s dipilih
%      supaya Ts sensor noise (0.25 s) tetap kelipatan bulat dari base
%      step (0.25/0.05 = 5). ----
set_param(mdl, 'Solver', 'ode4', 'SolverType', 'Fixed-step', ...
    'FixedStep', '0.05', 'StopTime', '1800');

% ---- InitFcn: definisikan default semua variabel *_ws jika belum ada
%      di base workspace. Ini membuat model AMAN dijalankan LANGSUNG
%      dari Simulink (tombol Run) atau lewat sim() di sesi MATLAB baru
%      tanpa lebih dulu menjalankan build_simulink_model()/main script
%      -- InitFcn dieksekusi di base workspace tiap kali model mulai
%      disimulasikan, apa pun cara pemanggilannya.
initLines = { ...
'if ~exist(''setpoint_ws'',''var''), setpoint_ws = 300; end'
'if ~exist(''gs_enable_ws'',''var''), gs_enable_ws = 1; end'
'if ~exist(''lambda_p_ws'',''var''), lambda_p_ws = 0.01; end'
'if ~exist(''lambda_i_ws'',''var''), lambda_i_ws = 0.0001; end'
'if ~exist(''lambda_d_ws'',''var''), lambda_d_ws = 0.003; end'
'if ~exist(''noise_variance_ws'',''var''), noise_variance_ws = 2.25; end' ...
};
set_param(mdl, 'InitFcn', strjoin(initLines, newline));

Simulink.BlockDiagram.arrangeSystem(mdl);
save_system(mdl, outFile);
close_system(mdl, 0);
fprintf('[build_simulink_model] Tersimpan: %s\n', outFile);
end

% ------------------------------------------------------------
function setDefaultIfMissing(name, val)
if ~evalin('base', sprintf('exist(''%s'',''var'')', name))
    assignin('base', name, val);
end
end

% ------------------------------------------------------------
function buildControllerSubsystem(subPath)
% Isi Subsystem AdaptivePIDController:
%   In1=Tref, In2=Tact -> Out1=u, Out2=Kp_act, Out3=Ki_act, Out4=Kd_act
% Kp_base/Ki_base/Kd_base tetap (1.5/0.02/8.0); gs_enable & lambda_p/i/d
% dibaca dari workspace supaya bisa diubah antar-run tanpa membangun
% ulang model (dipakai compare_methods.m untuk 3 metode).
delete_block([subPath '/In1']);
delete_block([subPath '/Out1']);

add_block('simulink/Ports & Subsystems/In1', [subPath '/Tref'], 'Position', [30 30 60 50]);
add_block('simulink/Ports & Subsystems/In1', [subPath '/Tact'], 'Position', [30 90 60 110]);

add_block('simulink/Sources/Constant', [subPath '/Kp_base'], 'Value', '1.5', ...
    'Position', [30 150 90 170]);
add_block('simulink/Sources/Constant', [subPath '/Ki_base'], 'Value', '0.02', ...
    'Position', [30 190 90 210]);
add_block('simulink/Sources/Constant', [subPath '/Kd_base'], 'Value', '8.0', ...
    'Position', [30 230 90 250]);
add_block('simulink/Sources/Constant', [subPath '/lambda_p'], 'Value', 'lambda_p_ws', ...
    'Position', [30 270 100 290]);
add_block('simulink/Sources/Constant', [subPath '/lambda_i'], 'Value', 'lambda_i_ws', ...
    'Position', [30 310 100 330]);
add_block('simulink/Sources/Constant', [subPath '/lambda_d'], 'Value', 'lambda_d_ws', ...
    'Position', [30 350 100 370]);
add_block('simulink/Sources/Constant', [subPath '/gs_enable'], 'Value', 'gs_enable_ws', ...
    'Position', [30 390 100 410]);
add_block('simulink/Sources/Constant', [subPath '/Ts_const'], 'Value', '0.05', ...
    'Position', [30 430 90 450]);

% PENTING: isi Script LANGSUNG setelah blok MATLAB Function dibuat,
% SEBELUM add_line menyambung port-portnya -- port block MATLAB
% Function baru mengikuti signature function di Script-nya (default
% hanya 1 in/1 out sebelum Script diisi).
add_block('simulink/User-Defined Functions/MATLAB Function', ...
    [subPath '/AdaptivePidFcn'], 'Position', [160 150 380 400]);
% Sample time DISKRIT eksplisit (0.05s) -- blok ini pakai persistent
% state, tidak valid jika mewarisi sample time kontinu dari sinyal
% masukan (Tact berasal dari rantai kontinu Transport Delay/Transfer Fcn).
setChartScript([subPath '/AdaptivePidFcn'], adaptivePidScript(), '0.05');

add_block('simulink/Ports & Subsystems/Out1', [subPath '/u'], 'Position', [430 150 460 170]);
add_block('simulink/Ports & Subsystems/Out1', [subPath '/Kp_act'], 'Position', [430 220 460 240]);
add_block('simulink/Ports & Subsystems/Out1', [subPath '/Ki_act'], 'Position', [430 260 460 280]);
add_block('simulink/Ports & Subsystems/Out1', [subPath '/Kd_act'], 'Position', [430 300 460 320]);

add_line(subPath, 'Tref/1', 'AdaptivePidFcn/1', 'autorouting', 'on');
add_line(subPath, 'Tact/1', 'AdaptivePidFcn/2', 'autorouting', 'on');
add_line(subPath, 'Kp_base/1', 'AdaptivePidFcn/3', 'autorouting', 'on');
add_line(subPath, 'Ki_base/1', 'AdaptivePidFcn/4', 'autorouting', 'on');
add_line(subPath, 'Kd_base/1', 'AdaptivePidFcn/5', 'autorouting', 'on');
add_line(subPath, 'lambda_p/1', 'AdaptivePidFcn/6', 'autorouting', 'on');
add_line(subPath, 'lambda_i/1', 'AdaptivePidFcn/7', 'autorouting', 'on');
add_line(subPath, 'lambda_d/1', 'AdaptivePidFcn/8', 'autorouting', 'on');
add_line(subPath, 'gs_enable/1', 'AdaptivePidFcn/9', 'autorouting', 'on');
add_line(subPath, 'Ts_const/1', 'AdaptivePidFcn/10', 'autorouting', 'on');
add_line(subPath, 'AdaptivePidFcn/1', 'u/1', 'autorouting', 'on');
add_line(subPath, 'AdaptivePidFcn/2', 'Kp_act/1', 'autorouting', 'on');
add_line(subPath, 'AdaptivePidFcn/3', 'Ki_act/1', 'autorouting', 'on');
add_line(subPath, 'AdaptivePidFcn/4', 'Kd_act/1', 'autorouting', 'on');
end

% ------------------------------------------------------------
function s = adaptivePidScript()
lines = { ...
'function [u, Kp_act, Ki_act, Kd_act] = adaptivePidFcn(Tref, Tact, Kp_base, Ki_base, Kd_base, lambda_p, lambda_i, lambda_d, gs_enable, Ts)'
'%#codegen'
'% Adaptive PID dua lapis:'
'%   Lapis 1 - gain scheduling per zona suhu (aktif bila gs_enable~=0)'
'%   Lapis 2 - adaptasi lambda (gain scheduling dinamis berbasis error),'
'%             dinonaktifkan (gains dibekukan di nilai dasar) selama'
'%             |e| <= DEAD_ZONE supaya tidak terus "berburu" di sekitar'
'%             setpoint akibat noise kecil (osilasi persisten).'
'persistent sum_e e_filt_prev initialized'
'if isempty(initialized)'
'    sum_e = 0; e_filt_prev = 0; initialized = true;'
'end'
''
'if gs_enable ~= 0'
'    if Tact < 200'
'        Kp = Kp_base*1.3; Ki = Ki_base*0.8; Kd = Kd_base*0.7;'
'    elseif Tact < 400'
'        Kp = Kp_base; Ki = Ki_base; Kd = Kd_base;'
'    else'
'        Kp = Kp_base*0.7; Ki = Ki_base*1.2; Kd = Kd_base*1.3;'
'    end'
'else'
'    Kp = Kp_base; Ki = Ki_base; Kd = Kd_base;'
'end'
''
'e = Tref - Tact;'
''
'% Derivative dari error yang DIFILTER (low-pass alpha=0.1), bukan'
'% error mentah -- meredam noise sensor supaya tidak memicu derivative'
'% kick / lonjakan adaptasi lambda tiap sample.'
'alpha = 0.1;'
'e_filt = alpha*e + (1-alpha)*e_filt_prev;'
'delta_e = (e_filt - e_filt_prev) / Ts;'
'e_filt_prev = e_filt;'
''
'% Coba akumulasi integral (trial) -- baru dikonfirmasi/dibatalkan'
'% oleh logika anti-windup di bawah.'
'sum_e_trial = sum_e + e*Ts;'
''
'DEAD_ZONE = 8.0; % degC'
'if abs(e) > DEAD_ZONE'
'    Kp_act = Kp + lambda_p*abs(e);'
'    Ki_act = Ki + lambda_i*abs(sum_e_trial);'
'    Kd_act = Kd + lambda_d*abs(delta_e);'
'else'
'    % Dekat setpoint: bekukan gain di nilai dasar, jangan adapt.'
'    Kp_act = Kp;'
'    Ki_act = Ki;'
'    Kd_act = Kd;'
'end'
''
'u_trial = Kp_act*e + Ki_act*sum_e_trial + Kd_act*delta_e;'
'u = min(max(u_trial, 0), 100);'
''
'% Anti-windup: integrator HANYA diperbarui jika output TIDAK saturasi.'
'% Tanpa ini, saat pemanasan panjang dari 30 degC ke setpoint (mis. 300'
'% degC), error besar terus menumpuk di sum_e ("integrator windup")'
'% walau keluaran sudah mentok 100%; begitu suhu mendekati setpoint,'
'% sum_e yang sudah "menggunung" itu membuat u tetap besar terlalu'
'% lama, menyebabkan overshoot parah.'
'if u_trial == u'
'    sum_e = sum_e_trial;'
'elseif abs(e) > DEAD_ZONE'
'    Ki_act = Ki + lambda_i*abs(sum_e); % integrator dibekukan, tampilkan nilai lama'
'end'
'end' ...
};
s = strjoin(lines, newline);
end

% ------------------------------------------------------------
function buildPlantSubsystem(subPath)
% Isi Subsystem Plant: In1=u (PWM 0-100, sudah lewat lag aktuator)
%   -> Transport Delay (L=15s) -> FOPDT nonlinear per-zona (K berubah,
%   tau=180s tetap) -> tambah noise sensor -> Out1=T (dengan noise)
delete_block([subPath '/In1']);
delete_block([subPath '/Out1']);

add_block('simulink/Ports & Subsystems/In1', [subPath '/u_pwm'], 'Position', [30 90 60 110]);

add_block('simulink/Continuous/Transport Delay', [subPath '/DeadTime'], ...
    'DelayTime', '15', 'Position', [100 88 160 112]);

% PENTING: isi Script LANGSUNG setelah blok dibuat, SEBELUM add_line.
add_block('simulink/User-Defined Functions/MATLAB Function', ...
    [subPath '/PlantThermalFcn'], 'Position', [200 80 340 120]);
% Sample time DISKRIT eksplisit (0.05s) -- blok ini pakai persistent
% state, tidak valid jika mewarisi sample time kontinu dari Transport Delay.
setChartScript([subPath '/PlantThermalFcn'], plantThermalScript(), '0.05');
add_block('simulink/Sources/Constant', [subPath '/Ts_const2'], 'Value', '0.05', ...
    'Position', [200 150 260 170]);

add_block('simulink/Sources/Band-Limited White Noise', [subPath '/SensorNoise'], ...
    'Position', [380 150 420 180], 'Cov', 'noise_variance_ws', 'Ts', '0.25', 'seed', '23341');
add_block('simulink/Math Operations/Add', [subPath '/AddNoise'], ...
    'Position', [400 88 430 112], 'Inputs', '++');

add_block('simulink/Ports & Subsystems/Out1', [subPath '/T_out'], 'Position', [470 90 500 110]);

add_line(subPath, 'u_pwm/1', 'DeadTime/1', 'autorouting', 'on');
add_line(subPath, 'DeadTime/1', 'PlantThermalFcn/1', 'autorouting', 'on');
add_line(subPath, 'Ts_const2/1', 'PlantThermalFcn/2', 'autorouting', 'on');
add_line(subPath, 'PlantThermalFcn/1', 'AddNoise/1', 'autorouting', 'on');
add_line(subPath, 'SensorNoise/1', 'AddNoise/2', 'autorouting', 'on');
add_line(subPath, 'AddNoise/1', 'T_out/1', 'autorouting', 'on');
end

% ------------------------------------------------------------
function s = plantThermalScript()
lines = { ...
'function T = plantThermalFcn(u_delayed, Ts)'
'%#codegen'
'% Model FOPDT nonlinear: gain K berubah per zona suhu, tau tetap.'
'% NILAI K, tau BERIKUT ADALAH ASUMSI/PLACEHOLDER -- kalibrasi ulang'
'% setelah tungku fisik diuji (belum ada data eksperimen nyata).'
'persistent T_state initialized'
'T_ambient = 30; K_base = 3.5; tau = 180;'
'if isempty(initialized)'
'    T_state = T_ambient;'
'    initialized = true;'
'end'
'T = T_state;'
'if T < 200'
'    Kz = K_base * 1.3;   % zona dingin'
'elseif T < 400'
'    Kz = K_base * 1.0;   % zona tengah (baseline)'
'else'
'    Kz = K_base * 0.7;   % zona panas'
'end'
'dT = (-(T_state - T_ambient) + Kz*u_delayed) / tau;'
'T_state = T_state + Ts*dT;'
'end' ...
};
s = strjoin(lines, newline);
end

% ------------------------------------------------------------
function setChartScript(blockPath, scriptCode, sampleTime)
% sampleTime (opsional): sample time DISKRIT eksplisit, mis. '0.05'.
% Wajib diisi untuk blok yang memakai persistent state. Nama parameter
% yang BENAR adalah 'SystemSampleTime' pada BLOK (bukan 'SampleTime' --
% itu bukan parameter valid untuk block wrapper bertipe SubSystem ini,
% dan chartObj.SampleTime di objek Stateflow.EMChart hanya READ-ONLY
% reflektif, tidak benar-benar mengubah apa pun meski assignment-nya
% tidak error).
rt = sfroot;
chartObj = rt.find('-isa', 'Stateflow.EMChart', 'Path', blockPath);
if isempty(chartObj)
    error('setChartScript: chart tidak ditemukan untuk %s', blockPath);
end
chartObj.Script = scriptCode;
if nargin >= 3 && ~isempty(sampleTime)
    set_param(blockPath, 'SystemSampleTime', sampleTime);
end
end
