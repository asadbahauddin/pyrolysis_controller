function build_pyrolysis_system()
% ============================================================
% build_pyrolysis_system.m
% Membangun & menyimpan pyrolysis_system.slx secara terprogram.
%
% Judul  : Implementasi Adaptive PID untuk Kendali Suhu Tungku
%          Pyrolysis Berbasis ESP32
% Nama   : [Nama Mahasiswa]
% NIM    : [NIM]
% Tanggal: [Tanggal]
%
% CATATAN DESAIN:
% Plant nonlinear (gain & tau berubah per zona suhu) + dead time
% diimplementasikan sebagai SATU blok MATLAB Function ber-state
% (persistent), bukan rangkaian Transfer Fcn + Transport Delay
% terpisah -- karena gain/tau yang berubah-ubah terhadap suhu tidak
% bisa direpresentasikan oleh blok Transfer Fcn linear standar.
% Pendekatan ini meniru persis plant_model.m (fungsi zoneParams &
% simulateClosedLoop) sehingga hasil Simulink konsisten dengan hasil
% main_simulation.m.
%
% Jalankan fungsi ini di MATLAB (dengan Simulink terpasang).
% ============================================================

mdl = 'pyrolysis_system';
thisDir = fileparts(mfilename('fullpath'));
outFile = fullfile(thisDir, [mdl '.slx']);

if bdIsLoaded(mdl), close_system(mdl, 0); end
if exist(outFile, 'file'), delete(outFile); end

new_system(mdl);
open_system(mdl);

% --- Sumber ---
add_block('simulink/Sources/Step', [mdl '/Setpoint'], 'Position', [30 60 60 90], ...
    'Time', '0', 'Before', '30', 'After', '300');
add_block('simulink/Sources/Constant', [mdl '/Kp_base'], 'Position', [30 140 60 160], 'Value', '2.0');
add_block('simulink/Sources/Constant', [mdl '/Ki_base'], 'Position', [30 180 60 200], 'Value', '0.1');
add_block('simulink/Sources/Constant', [mdl '/Kd_base'], 'Position', [30 220 60 240], 'Value', '10.0');
add_block('simulink/Sources/Constant', [mdl '/Ts_const'], 'Position', [30 260 60 280], 'Value', '0.1');
add_block('simulink/Sources/Step', [mdl '/Disturbance'], 'Position', [30 400 60 430], ...
    'Time', '1800', 'Before', '0', 'After', '0'); % edit 'After' utk uji disturbance rejection
add_block('simulink/Sources/Band-Limited White Noise', [mdl '/Sensor_Noise'], ...
    'Position', [430 300 460 330], 'Cov', '2.25', 'Ts', '0.25', 'seed', '23341');

% --- Kontroler: PID + gain scheduling + anti-windup (MATLAB Function) ---
% PENTING: isi Script LANGSUNG setelah tiap blok MATLAB Function dibuat,
% SEBELUM add_line menyambung port-portnya -- port block baru mengikuti
% signature function di Script-nya (default hanya 1 in/1 out).
add_block('simulink/User-Defined Functions/MATLAB Function', ...
    [mdl '/PID_GainSchedule'], 'Position', [150 100 300 260]);
% Sample time DISKRIT eksplisit -- blok ini pakai persistent state,
% tidak valid jika mewarisi sample time kontinu.
setChartScript(mdl, 'PID_GainSchedule', pidScript(), '0.1');

add_block('simulink/Discontinuities/Saturation', [mdl '/Saturation_PWM'], ...
    'Position', [340 170 370 200], 'UpperLimit', '100', 'LowerLimit', '0');

% --- Plant nonlinear FOPDT + dead time (MATLAB Function ber-state) ---
add_block('simulink/User-Defined Functions/MATLAB Function', ...
    [mdl '/Plant_Nonlinear'], 'Position', [420 160 570 210]);
setChartScript(mdl, 'Plant_Nonlinear', plantScript(), '0.1');

% --- Penjumlah gangguan & noise sensor ---
add_block('simulink/Math Operations/Add', [mdl '/Add_Disturbance'], ...
    'Position', [600 170 630 200], 'Inputs', '++');
add_block('simulink/Math Operations/Add', [mdl '/Add_Noise'], ...
    'Position', [660 250 690 280], 'Inputs', '++');

% --- Error display (opsional, hanya untuk Scope) ---
add_block('simulink/Math Operations/Sum', [mdl '/Error_Display'], ...
    'Position', [420 60 450 90], 'Inputs', '+-');

% --- Sinks ---
add_block('simulink/Sinks/Scope', [mdl '/Scope_Temperature'], 'Position', [750 60 790 100]);
set_param([mdl '/Scope_Temperature'], 'NumInputPorts', '2');
add_block('simulink/Sinks/Scope', [mdl '/Scope_PWM'], 'Position', [420 380 460 420]);
add_block('simulink/Sinks/To Workspace', [mdl '/ToWS_T'], 'Position', [750 150 800 180], ...
    'VariableName', 'T_out', 'SaveFormat', 'Timeseries');
add_block('simulink/Sinks/To Workspace', [mdl '/ToWS_u'], 'Position', [420 440 470 470], ...
    'VariableName', 'u_out', 'SaveFormat', 'Timeseries');

% --- Sambungan kontroler ---
add_line(mdl, 'Setpoint/1', 'PID_GainSchedule/1', 'autorouting', 'on');
add_line(mdl, 'Kp_base/1', 'PID_GainSchedule/3', 'autorouting', 'on');
add_line(mdl, 'Ki_base/1', 'PID_GainSchedule/4', 'autorouting', 'on');
add_line(mdl, 'Kd_base/1', 'PID_GainSchedule/5', 'autorouting', 'on');
add_line(mdl, 'Ts_const/1', 'PID_GainSchedule/6', 'autorouting', 'on');
add_line(mdl, 'PID_GainSchedule/1', 'Saturation_PWM/1', 'autorouting', 'on');
add_line(mdl, 'Saturation_PWM/1', 'Plant_Nonlinear/1', 'autorouting', 'on');
add_line(mdl, 'Ts_const/1', 'Plant_Nonlinear/2', 'autorouting', 'on');
add_line(mdl, 'Saturation_PWM/1', 'Scope_PWM/1', 'autorouting', 'on');
add_line(mdl, 'Saturation_PWM/1', 'ToWS_u/1', 'autorouting', 'on');

% --- Plant -> gangguan -> noise -> feedback ---
add_line(mdl, 'Plant_Nonlinear/1', 'Add_Disturbance/1', 'autorouting', 'on');
add_line(mdl, 'Disturbance/1', 'Add_Disturbance/2', 'autorouting', 'on');
add_line(mdl, 'Add_Disturbance/1', 'Add_Noise/1', 'autorouting', 'on');
add_line(mdl, 'Sensor_Noise/1', 'Add_Noise/2', 'autorouting', 'on');
add_line(mdl, 'Add_Noise/1', 'PID_GainSchedule/2', 'autorouting', 'on');
add_line(mdl, 'Add_Noise/1', 'Scope_Temperature/1', 'autorouting', 'on');
add_line(mdl, 'Add_Noise/1', 'ToWS_T/1', 'autorouting', 'on');

% --- Error display ---
add_line(mdl, 'Setpoint/1', 'Error_Display/1', 'autorouting', 'on');
add_line(mdl, 'Add_Noise/1', 'Error_Display/2', 'autorouting', 'on');
add_line(mdl, 'Setpoint/1', 'Scope_Temperature/2', 'autorouting', 'on');

% --- Konfigurasi solver ---
set_param(mdl, 'Solver', 'ode23tb', 'FixedStep', '0.1', 'StopTime', '7200');
set_param(mdl, 'SolverType', 'Fixed-step');

Simulink.BlockDiagram.arrangeSystem(mdl);
save_system(mdl, outFile);
close_system(mdl, 0);
fprintf('[build_pyrolysis_system] Tersimpan: %s\n', outFile);
end

% ------------------------------------------------------------
function setChartScript(mdl, blockName, scriptCode, sampleTime)
% Mengisi kode sebuah blok MATLAB Function berdasarkan path lengkapnya.
% sampleTime (opsional): sample time diskrit eksplisit, mis. '0.1'.
% Nama parameter yang BENAR adalah 'SystemSampleTime' pada BLOK (bukan
% 'SampleTime' -- itu bukan parameter valid utk block wrapper bertipe
% SubSystem ini, dan chartObj.SampleTime pada objek Stateflow.EMChart
% hanya READ-ONLY reflektif, tidak benar-benar mengubah apa pun).
blockPath = [mdl '/' blockName];
rt = sfroot;
chartObj = rt.find('-isa', 'Stateflow.EMChart', 'Path', blockPath);
if isempty(chartObj)
    error('setChartScript: chart tidak ditemukan untuk %s', blockPath);
end
chartObj.Script = scriptCode;
if nargin >= 4 && ~isempty(sampleTime)
    set_param(blockPath, 'SystemSampleTime', sampleTime);
end
end

% ------------------------------------------------------------
function s = pidScript()
lines = { ...
'function u = pidGainScheduledFcn(setpoint, measurement, Kp_base, Ki_base, Kd_base, Ts)'
'%#codegen'
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
s = strjoin(lines, newline);
end

% ------------------------------------------------------------
function s = plantScript()
lines = { ...
'function T = plantNonlinearFcn(u, Ts)'
'%#codegen'
'% Plant FOPDT nonlinear 3-zona + dead time, meniru plant_model.m'
'persistent buf T_state initialized'
'Td = 15; T_ambient = 30;'
'Kcold = 4.55; Kmid = 3.5; Khot = 2.45;'
'taucold = 144; taumid = 180; tauhot = 234;'
''
'if isempty(initialized)'
'    Nd = max(1, round(Td/Ts));'
'    buf = zeros(Nd,1);'
'    T_state = T_ambient;'
'    initialized = true;'
'end'
''
'T = T_state;'
'u_delayed = buf(1);'
'buf = [buf(2:end); u];'
''
'if T <= 200'
'    Kz = Kcold; tauz = taucold;'
'elseif T >= 400'
'    Kz = Khot; tauz = tauhot;'
'elseif T <= 300'
'    r = (T-200)/100; Kz = Kcold + r*(Kmid-Kcold); tauz = taucold + r*(taumid-taucold);'
'else'
'    r = (T-300)/100; Kz = Kmid + r*(Khot-Kmid); tauz = taumid + r*(tauhot-taumid);'
'end'
''
'dT = (-(T_state - T_ambient) + Kz*u_delayed) / tauz;'
'T_state = T_state + Ts*dT;'
'end' ...
};
s = strjoin(lines, newline);
end
