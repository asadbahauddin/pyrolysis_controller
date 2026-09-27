function build_adaptive_pid()
% ============================================================
% build_adaptive_pid.m
% Membangun & menyimpan adaptive_pid.slx secara terprogram.
%
% Judul  : Implementasi Adaptive PID untuk Kendali Suhu Tungku
%          Pyrolysis Berbasis ESP32
% Nama   : [Nama Mahasiswa]
% NIM    : [NIM]
% Tanggal: [Tanggal]
%
% CATATAN DESAIN:
% Sama seperti firmware ESP32 (learning_system.cpp), coordinate
% descent HANYA berjalan di ANTARA run (dari log run yang baru
% selesai), bukan di tengah satu run yang sedang berjalan. Karena
% itu, logika coordinate descent TIDAK ditaruh sebagai blok yang
% berjalan tiap time-step di dalam model ini -- melainkan dijalankan
% di MATLAB, di antara panggilan sim(), lewat run_adaptive_pid_multirun.m
% (memakai ulang coordinate_descent.m -> coordinateDescentReplay(),
% persis logika learning_system.cpp).
%
% Blok Kp_current/Ki_current/Kd_current membaca NILAI dari workspace
% (nama variabel, bukan angka literal) sehingga run_adaptive_pid_multirun.m
% bisa meng-assign gain baru sebelum tiap sim() berikutnya. Blok Memory
% + Display menampilkan gain yang aktif. Blok From Workspace memuat
% trace suhu run sebelumnya untuk overlay perbandingan pada Scope.
%
% Jalankan fungsi ini di MATLAB (dengan Simulink terpasang).
% ============================================================

mdl = 'adaptive_pid';
thisDir = fileparts(mfilename('fullpath'));
outFile = fullfile(thisDir, [mdl '.slx']);

if bdIsLoaded(mdl), close_system(mdl, 0); end
if exist(outFile, 'file'), delete(outFile); end

% Nilai default di workspace dasar supaya model bisa langsung dibuka & di-run manual
if ~evalin('base', 'exist(''Kp_current'',''var'')')
    assignin('base', 'Kp_current', 2.0);
    assignin('base', 'Ki_current', 0.1);
    assignin('base', 'Kd_current', 10.0);
end
if ~evalin('base', 'exist(''prev_run_T'',''var'')')
    assignin('base', 'prev_run_T', [0 30; 3600 30]); % placeholder [t, T]
end

new_system(mdl);
open_system(mdl);

% --- Sumber ---
add_block('simulink/Sources/Step', [mdl '/Setpoint'], 'Position', [30 60 60 90], ...
    'Time', '0', 'Before', '30', 'After', '300');
add_block('simulink/Sources/Constant', [mdl '/Kp_current'], 'Position', [30 140 90 160], ...
    'Value', 'Kp_current');
add_block('simulink/Sources/Constant', [mdl '/Ki_current'], 'Position', [30 180 90 200], ...
    'Value', 'Ki_current');
add_block('simulink/Sources/Constant', [mdl '/Kd_current'], 'Position', [30 220 90 240], ...
    'Value', 'Kd_current');
add_block('simulink/Sources/Constant', [mdl '/Ts_const'], 'Position', [30 260 60 280], 'Value', '0.1');
add_block('simulink/Sources/Band-Limited White Noise', [mdl '/Sensor_Noise'], ...
    'Position', [430 300 460 330], 'Cov', '2.25', 'Ts', '0.25', 'seed', '23341');
add_block('simulink/Sources/From Workspace', [mdl '/PrevRun_T'], 'Position', [750 300 820 330], ...
    'VariableName', 'prev_run_T');

% --- Kontroler & plant (identik dengan pyrolysis_system.slx) ---
% PENTING: isi Script LANGSUNG setelah tiap blok MATLAB Function dibuat,
% SEBELUM add_line menyambung port-portnya.
add_block('simulink/User-Defined Functions/MATLAB Function', ...
    [mdl '/PID_GainSchedule'], 'Position', [150 100 300 260]);
setChartScriptLocal(mdl, 'PID_GainSchedule', pidScriptLocal(), '0.1');
add_block('simulink/Discontinuities/Saturation', [mdl '/Saturation_PWM'], ...
    'Position', [340 170 370 200], 'UpperLimit', '100', 'LowerLimit', '0');
add_block('simulink/User-Defined Functions/MATLAB Function', ...
    [mdl '/Plant_Nonlinear'], 'Position', [420 160 570 210]);
setChartScriptLocal(mdl, 'Plant_Nonlinear', plantScriptLocal(), '0.1');
add_block('simulink/Math Operations/Add', [mdl '/Add_Noise'], ...
    'Position', [660 250 690 280], 'Inputs', '++');

% --- Memory + Display gain aktif ---
add_block('simulink/Discrete/Memory', [mdl '/Mem_Kp'], 'Position', [110 500 140 520]);
add_block('simulink/Discrete/Memory', [mdl '/Mem_Ki'], 'Position', [110 540 140 560]);
add_block('simulink/Discrete/Memory', [mdl '/Mem_Kd'], 'Position', [110 580 140 600]);
add_block('simulink/Sinks/Display', [mdl '/Disp_Kp'], 'Position', [200 500 260 520]);
add_block('simulink/Sinks/Display', [mdl '/Disp_Ki'], 'Position', [200 540 260 560]);
add_block('simulink/Sinks/Display', [mdl '/Disp_Kd'], 'Position', [200 580 260 600]);

% --- Sinks ---
add_block('simulink/Sinks/Scope', [mdl '/Scope_Temperature'], 'Position', [850 60 890 100]);
set_param([mdl '/Scope_Temperature'], 'NumInputPorts', '3');
add_block('simulink/Sinks/To Workspace', [mdl '/ToWS_T'], 'Position', [750 150 800 180], ...
    'VariableName', 'T_out', 'SaveFormat', 'Timeseries');
add_block('simulink/Sinks/To Workspace', [mdl '/ToWS_u'], 'Position', [420 380 470 410], ...
    'VariableName', 'u_out', 'SaveFormat', 'Timeseries');
add_block('simulink/Sinks/To Workspace', [mdl '/ToWS_e'], 'Position', [420 60 470 90], ...
    'VariableName', 'e_out', 'SaveFormat', 'Timeseries');
add_block('simulink/Math Operations/Sum', [mdl '/Error_Display'], ...
    'Position', [700 60 730 90], 'Inputs', '+-');

% --- Sambungan kontroler & plant ---
add_line(mdl, 'Setpoint/1', 'PID_GainSchedule/1', 'autorouting', 'on');
add_line(mdl, 'Kp_current/1', 'PID_GainSchedule/3', 'autorouting', 'on');
add_line(mdl, 'Ki_current/1', 'PID_GainSchedule/4', 'autorouting', 'on');
add_line(mdl, 'Kd_current/1', 'PID_GainSchedule/5', 'autorouting', 'on');
add_line(mdl, 'Ts_const/1', 'PID_GainSchedule/6', 'autorouting', 'on');
add_line(mdl, 'PID_GainSchedule/1', 'Saturation_PWM/1', 'autorouting', 'on');
add_line(mdl, 'Saturation_PWM/1', 'Plant_Nonlinear/1', 'autorouting', 'on');
add_line(mdl, 'Ts_const/1', 'Plant_Nonlinear/2', 'autorouting', 'on');
add_line(mdl, 'Saturation_PWM/1', 'ToWS_u/1', 'autorouting', 'on');
add_line(mdl, 'Plant_Nonlinear/1', 'Add_Noise/1', 'autorouting', 'on');
add_line(mdl, 'Sensor_Noise/1', 'Add_Noise/2', 'autorouting', 'on');
add_line(mdl, 'Add_Noise/1', 'PID_GainSchedule/2', 'autorouting', 'on');
add_line(mdl, 'Add_Noise/1', 'ToWS_T/1', 'autorouting', 'on');
add_line(mdl, 'Add_Noise/1', 'Scope_Temperature/1', 'autorouting', 'on');
add_line(mdl, 'Setpoint/1', 'Scope_Temperature/2', 'autorouting', 'on');
add_line(mdl, 'PrevRun_T/1', 'Scope_Temperature/3', 'autorouting', 'on');
add_line(mdl, 'Setpoint/1', 'Error_Display/1', 'autorouting', 'on');
add_line(mdl, 'Add_Noise/1', 'Error_Display/2', 'autorouting', 'on');
add_line(mdl, 'Error_Display/1', 'ToWS_e/1', 'autorouting', 'on');

% --- Memory & Display gain aktif ---
add_line(mdl, 'Kp_current/1', 'Mem_Kp/1', 'autorouting', 'on');
add_line(mdl, 'Ki_current/1', 'Mem_Ki/1', 'autorouting', 'on');
add_line(mdl, 'Kd_current/1', 'Mem_Kd/1', 'autorouting', 'on');
add_line(mdl, 'Mem_Kp/1', 'Disp_Kp/1', 'autorouting', 'on');
add_line(mdl, 'Mem_Ki/1', 'Disp_Ki/1', 'autorouting', 'on');
add_line(mdl, 'Mem_Kd/1', 'Disp_Kd/1', 'autorouting', 'on');

% --- Solver ---
set_param(mdl, 'Solver', 'ode23tb', 'FixedStep', '0.1', 'StopTime', '3600');
set_param(mdl, 'SolverType', 'Fixed-step');

Simulink.BlockDiagram.arrangeSystem(mdl);
save_system(mdl, outFile);
close_system(mdl, 0);
fprintf('[build_adaptive_pid] Tersimpan: %s\n', outFile);
end

% ------------------------------------------------------------
function setChartScriptLocal(mdl, blockName, scriptCode, sampleTime)
% sampleTime (opsional): nama parameter yang BENAR adalah
% 'SystemSampleTime' pada BLOK (bukan 'SampleTime' pada objek
% Stateflow.EMChart -- itu hanya READ-ONLY reflektif).
blockPath = [mdl '/' blockName];
rt = sfroot;
chartObj = rt.find('-isa', 'Stateflow.EMChart', 'Path', blockPath);
if isempty(chartObj)
    error('setChartScriptLocal: chart tidak ditemukan untuk %s', blockPath);
end
chartObj.Script = scriptCode;
if nargin >= 4 && ~isempty(sampleTime)
    set_param(blockPath, 'SystemSampleTime', sampleTime);
end
end

function s = pidScriptLocal()
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

function s = plantScriptLocal()
lines = { ...
'function T = plantNonlinearFcn(u, Ts)'
'%#codegen'
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
