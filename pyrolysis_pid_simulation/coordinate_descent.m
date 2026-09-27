function api = coordinate_descent()
% ============================================================
% coordinate_descent.m
% Algoritma adaptive coordinate descent untuk optimasi Kp, Ki, Kd
%
% Judul  : Implementasi Adaptive PID untuk Kendali Suhu Tungku
%          Pyrolysis Berbasis ESP32
% Nama   : [Nama Mahasiswa]
% NIM    : [NIM]
% Tanggal: [Tanggal]
%
% Dua varian disediakan:
%   coordinateDescent()       - resimulasi closed-loop PENUH setiap
%                                trial (ground truth, dipakai untuk
%                                studi optimasi offline / analisis).
%   coordinateDescentReplay() - MENIRU learning_system.cpp: replay
%                                dari data satu run yang sudah terekam
%                                (error & PWM), pakai proxy gain proses
%                                linear k_process = dE/dP, tanpa
%                                menjalankan ulang plant. Ini metode
%                                yang benar-benar dipakai di ESP32
%                                (murah secara komputasi & memori),
%                                dipakai pada Skenario 4 (multi-run).
% ============================================================

api.coordinateDescent       = @coordinateDescent;
api.coordinateDescentReplay = @coordinateDescentReplay;
api.computeISE  = @computeISE;
api.computeIAE  = @computeIAE;
api.computeITAE = @computeITAE;
end

% ------------------------------------------------------------
function [Kp_new, Ki_new, Kd_new, ise_history, Kp_hist, Ki_hist, Kd_hist] = coordinateDescent(...
    plant_params, setpoint, Kp0, Ki0, Kd0, n_iterations, step_ratio)
% Coordinate descent dengan resimulasi closed-loop PENUH setiap trial.
% Urutan per iterasi: Kp -> Ki -> Kd, masing-masing coba +step lalu
% -step, ambil yang menurunkan ISE paling banyak.
Kp = Kp0; Ki = Ki0; Kd = Kd0;
T_sim = 1800; % 30 menit per trial (cukup utk transient + steady state)
ise_history = zeros(n_iterations,1);
Kp_hist = zeros(n_iterations,1); Ki_hist = zeros(n_iterations,1); Kd_hist = zeros(n_iterations,1);

for it = 1:n_iterations
    best_ise = computeISE(plant_params, setpoint, Kp, Ki, Kd, T_sim);

    [Kp, best_ise] = tryCoordinate(@(v) computeISE(plant_params,setpoint,v,Ki,Kd,T_sim), Kp, step_ratio, best_ise);
    [Ki, best_ise] = tryCoordinate(@(v) computeISE(plant_params,setpoint,Kp,v,Kd,T_sim), Ki, step_ratio, best_ise);
    [Kd, best_ise] = tryCoordinate(@(v) computeISE(plant_params,setpoint,Kp,Ki,v,T_sim), Kd, step_ratio, best_ise);

    ise_history(it) = best_ise;
    Kp_hist(it) = Kp; Ki_hist(it) = Ki; Kd_hist(it) = Kd;
end
Kp_new = Kp; Ki_new = Ki; Kd_new = Kd;
end

function [val, best_ise] = tryCoordinate(fISE, val0, step_ratio, best_ise)
plus = val0*(1+step_ratio);
minus = val0*(1-step_ratio);
ise_plus = fISE(plus);
ise_minus = fISE(minus);
val = val0;
if ise_plus < best_ise && ise_plus <= ise_minus
    val = plus; best_ise = ise_plus;
elseif ise_minus < best_ise
    val = minus; best_ise = ise_minus;
end
end

% ------------------------------------------------------------
function ise = computeISE(plant_params, setpoint, Kp, Ki, Kd, T_sim)
out = runTrial(plant_params, setpoint, Kp, Ki, Kd, T_sim);
ise = out.ISE;
end

function iae = computeIAE(plant_params, setpoint, Kp, Ki, Kd, T_sim)
out = runTrial(plant_params, setpoint, Kp, Ki, Kd, T_sim);
iae = out.IAE;
end

function itae = computeITAE(plant_params, setpoint, Kp, Ki, Kd, T_sim)
out = runTrial(plant_params, setpoint, Kp, Ki, Kd, T_sim);
itae = out.ITAE;
end

function out = runTrial(plant_params, setpoint, Kp, Ki, Kd, T_sim)
plant = plant_model();
pidm = pid_design();
ctrl = pidm.defaultCtrl(Kp, Ki, Kd, 'zone');
opts.dt = 0.5;
opts.T_sim = T_sim;
opts.T_init = plant_params.T_ambient;
opts.setpoint = setpoint;
opts.noise = false;
out = plant.simulateClosedLoop(plant_params, ctrl, opts);
end

% ------------------------------------------------------------
function [Kp_new, Ki_new, Kd_new, ise_sim] = coordinateDescentReplay(...
    run_log, Kp0, Ki0, Kd0, step_ratio)
% Coordinate descent berbasis replay data satu run (persis
% learning_system.cpp / fungsi coordinateDescent() firmware).
%
% run_log : struct hasil plant_model.simulateClosedLoop(), minimal
%           berisi field t (uniform time vector) dan e, u (error &
%           PWM sepanjang run tersebut)
I_MAX = 100; I_MIN = -100;
dt = run_log.t(2) - run_log.t(1);
err = run_log.e(:); upwm = run_log.u(:);

k_process = estimateProcessGain(err, upwm);

Kp = Kp0; Ki = Ki0; Kd = Kd0;
best_ise = simulateISEReplay(err, upwm, dt, Kp, Ki, Kd, k_process, I_MAX, I_MIN);

[Kp, best_ise] = tryCoordinate(@(v) simulateISEReplay(err,upwm,dt,v,Ki,Kd,k_process,I_MAX,I_MIN), Kp, step_ratio, best_ise);
[Ki, best_ise] = tryCoordinate(@(v) simulateISEReplay(err,upwm,dt,Kp,v,Kd,k_process,I_MAX,I_MIN), Ki, step_ratio, best_ise);
[Kd, best_ise] = tryCoordinate(@(v) simulateISEReplay(err,upwm,dt,Kp,Ki,v,k_process,I_MAX,I_MIN), Kd, step_ratio, best_ise);

Kp_new = Kp; Ki_new = Ki; Kd_new = Kd;
ise_sim = best_ise;
end

function k = estimateProcessGain(err, upwm)
dE = max(err) - min(err);
dP = max(upwm) - min(upwm);
if dP < 1, dP = 1; end
k = dE / dP;
end

function ise = simulateISEReplay(err, upwm, dt, Kp, Ki, Kd, k_process, I_MAX, I_MIN)
% Mensimulasikan ISE seandainya gain [Kp,Ki,Kd] dipakai pada trace
% error yang SAMA, dikoreksi linear terhadap selisih PWM trial vs
% PWM yang benar-benar terekam (proxy gain proses k_process).
integral = 0; prev_e = 0; first = true; ise = 0;
for i = 1:numel(err)
    e = err(i);
    integral = integral + Ki*e*dt;
    integral = min(max(integral, I_MIN), I_MAX);
    if first
        deriv = 0;
    else
        deriv = Kd*(e - prev_e)/dt;
    end
    first = false;
    u_trial = Kp*e + integral + deriv;
    u_trial = min(max(u_trial, 0), 100);
    e_new = e - k_process*(u_trial - upwm(i));
    ise = ise + e_new^2*dt;
    prev_e = e;
end
end
