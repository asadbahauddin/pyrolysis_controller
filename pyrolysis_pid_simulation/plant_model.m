function api = plant_model()
% ============================================================
% plant_model.m
% Model matematik tungku pyrolysis (FOPDT nonlinear 3-zona)
%
% Judul  : Implementasi Adaptive PID untuk Kendali Suhu Tungku
%          Pyrolysis Berbasis ESP32
% Nama   : [Nama Mahasiswa]
% NIM    : [NIM]
% Tanggal: [Tanggal]
%
% File ini adalah "module" MATLAB: panggil plant_model() untuk
% mendapatkan struct berisi function handle. Pola ini dipakai
% supaya satu file boleh berisi banyak fungsi namun tetap bisa
% dipanggil dari file lain (MATLAB hanya mengizinkan satu fungsi
% publik per file jika dipanggil lewat nama file).
%
% Contoh pemakaian dari file lain:
%   plant = plant_model();
%   params = plant.defaultParams();
%   out = plant.simulateClosedLoop(params, ctrl, opts);
% ============================================================

api.defaultParams      = @defaultParams;
api.buildPlantModel    = @buildPlantModel;
api.plantStateSpace    = @plantStateSpace;
api.simulatePlant      = @simulatePlant;
api.zoneParams         = @zoneParams;
api.resolveControllerGains = @resolveControllerGains;
api.simulateClosedLoop  = @simulateClosedLoop;
api.simulateRelayOpenLoop = @simulateRelayOpenLoop;
end

% ------------------------------------------------------------
function params = defaultParams()
% Parameter tungku pyrolysis (estimasi untuk tungku sedang).
% Konvensi PWM dipakai dalam persen 0-100%, setara dengan nilai
% mentah 0-255 pada firmware ESP32 (AT_PWM_HIGH=200/255 = 78%).

params.K_plant   = 3.5;      % degC per unit PWM (0-100%), zona tengah
params.tau_plant = 180;      % detik, konstanta waktu termal nominal
params.Td_plant  = 15;       % detik, dead time

params.T_ambient = 30;       % degC suhu awal
params.T_max     = 800;      % degC batas darurat (TEMP_EMERGENCY)
params.setpoint  = 300;      % degC setpoint default (DEFAULT_SETPOINT)

% Gain & tau nonlinear per zona suhu (menyebabkan gain scheduling
% dibutuhkan pada sisi controller)
params.K_cold   = params.K_plant * 1.3;
params.K_mid    = params.K_plant * 1.0;
params.K_hot    = params.K_plant * 0.7;
params.tau_cold = params.tau_plant * 0.8;
params.tau_mid  = params.tau_plant * 1.0;
params.tau_hot  = params.tau_plant * 1.3;

% Batas aktuator (PWM fan, persen)
params.PWM_MAX = 100;
params.PWM_MIN = 0;

% Noise sensor MAX6675 (+-1.5 degC), sampling 4 Hz
params.noise_std    = 1.5;
params.noise_Ts     = 0.25;

% Batas zona suhu (sama dengan firmware config.h)
params.ZONE_COLD_MAX = 200;
params.ZONE_MID_MAX  = 400;
end

% ------------------------------------------------------------
function [Kz, tauz] = zoneParams(T, params)
% Interpolasi halus gain & tau statik tungku terhadap suhu aktual.
% Zona dingin (<=200), tengah (200-400, linear lewat titik 300),
% panas (>=400). Ini merepresentasikan NONLINEARITAS FISIK plant,
% berbeda dari gain scheduling di sisi controller.
if T <= 200
    Kz = params.K_cold; tauz = params.tau_cold;
elseif T >= 400
    Kz = params.K_hot; tauz = params.tau_hot;
elseif T <= 300
    r = (T - 200) / 100;
    Kz = params.K_cold + r * (params.K_mid - params.K_cold);
    tauz = params.tau_cold + r * (params.tau_mid - params.tau_cold);
else
    r = (T - 300) / 100;
    Kz = params.K_mid + r * (params.K_hot - params.K_mid);
    tauz = params.tau_mid + r * (params.tau_hot - params.tau_mid);
end
end

% ------------------------------------------------------------
function [G, G_approx] = buildPlantModel(K, tau, Td, zone) %#ok<INUSD>
% Membangun transfer function tungku pyrolysis FOPDT:
%   G(s) = K * exp(-Td*s) / (tau*s + 1)
% Dead time didekati dengan Pade orde 2 supaya bisa dipakai untuk
% analisis frekuensi (Bode, root locus) dan blok Transfer Function
% di Simulink.
s = tf('s');
G_approx = K / (tau*s + 1);           % tanpa dead time, untuk analisis cepat
G = G_approx * pade_delay(Td, 2);     % dengan Pade orde 2
end

function Gd = pade_delay(Td, order)
% Pendekatan Pade untuk exp(-Td*s)
if Td <= 0
    Gd = tf(1,1);
else
    [num, den] = pade(Td, order);
    Gd = tf(num, den);
end
end

% ------------------------------------------------------------
function sys_ss = plantStateSpace(K, tau, Td)
% Konversi FOPDT (dengan Pade orde 2) ke state space, untuk
% dipakai analisis atau blok State-Space di Simulink.
[G, ~] = buildPlantModel(K, tau, Td, 'mid');
sys_ss = ss(G);
end

% ------------------------------------------------------------
function T_out = simulatePlant(u_input, T_init, t_span, params)
% Simulasi open-loop respon plant terhadap sinyal PWM u_input(t).
% u_input : vector PWM (0-100) sepanjang t_span, atau scalar (step)
% t_span  : vector waktu seragam (detik)
dt = t_span(2) - t_span(1);
N = numel(t_span);
if isscalar(u_input)
    u_input = u_input * ones(N,1);
end
Td_steps = max(1, round(params.Td_plant/dt));
u_buf = zeros(Td_steps,1);
T_out = zeros(N,1);
T_out(1) = T_init;
for k = 1:N-1
    u_delayed = u_buf(1);
    u_buf = [u_buf(2:end); u_input(k)];
    [Kz, tauz] = zoneParams(T_out(k), params);
    dT = (-(T_out(k) - params.T_ambient) + Kz*u_delayed) / tauz;
    T_out(k+1) = T_out(k) + dt*dT;
end
end

% ------------------------------------------------------------
function [Kp, Ki, Kd, zone] = resolveControllerGains(ctrl, T)
% Menentukan gain PID aktif berdasarkan mode controller & suhu
% terukur saat ini. Mencerminkan pid_controller.cpp:
%   - 'fixed'       : Kp/Ki/Kd tetap (tanpa scheduling)
%   - 'zone'        : hanya Kp yang dijadwal per-zona (SESUAI FIRMWARE),
%                      switching keras (hard switch) pada T=200/400
%   - 'zone_smooth' : varian penelitian, interpolasi halus utk Kp,Ki,Kd
%                      sekaligus (dipakai method "Adaptive PID" untuk studi
%                      perbandingan terhadap versi firmware)
if T < ctrl.zone_cold_max
    zone = "cold";
elseif T <= ctrl.zone_mid_max
    zone = "mid";
else
    zone = "hot";
end

switch ctrl.mode
    case 'fixed'
        Kp = ctrl.Kp; Ki = ctrl.Ki; Kd = ctrl.Kd;

    case 'zone'
        switch zone
            case "cold", multKp = 1.3;
            case "mid",  multKp = 1.0;
            case "hot",  multKp = 0.7;
        end
        Kp = ctrl.Kp * multKp;
        Ki = ctrl.Ki;   % TIDAK dijadwal (sesuai firmware)
        Kd = ctrl.Kd;   % TIDAK dijadwal (sesuai firmware)

    case 'zone_smooth'
        tbl = ctrl.schedule_table; % [T_breakpoint, multKp, multKi, multKd]
        Tb = tbl(:,1);
        if T <= Tb(1)
            m = tbl(1,2:4);
        elseif T >= Tb(end)
            m = tbl(end,2:4);
        else
            m = interp1(Tb, tbl(:,2:4), T, 'linear');
        end
        Kp = ctrl.Kp * m(1);
        Ki = ctrl.Ki * m(2);
        Kd = ctrl.Kd * m(3);

    otherwise
        error('plant_model:resolveControllerGains: mode tidak dikenal: %s', ctrl.mode);
end
end

% ------------------------------------------------------------
function out = simulateClosedLoop(params, ctrl, opts)
% Simulasi closed-loop PID + plant nonlinear, langkah waktu tetap
% (Euler, dt kecil) -- meniru loop kendali diskrit ESP32
% (LOOP_INTERVAL_MS = 100 ms secara default).
%
% params : struct dari defaultParams()
% ctrl   : struct berisi mode, Kp, Ki, Kd, zone_cold_max, zone_mid_max,
%          I_max, I_min, (opsional) N_filter, schedule_table
% opts   : struct berisi dt, T_sim, T_init, setpoint (scalar atau
%          function handle @(t)), disturbance (function handle @(t)
%          -> degC/s, opsional), noise (logical)
%
% out : struct t, T, Tmeas, u, e, Kp, Ki, Kd, zone, ISE, IAE, ITAE

dt = opts.dt;
t = (0:dt:opts.T_sim)';
N = numel(t);

Td_steps = max(1, round(params.Td_plant/dt));
u_buf = zeros(Td_steps,1);

T = zeros(N,1); T(1) = opts.T_init;
Tmeas_hist = zeros(N,1); Tmeas_hist(1) = opts.T_init;
u_hist = zeros(N,1);
e_hist = zeros(N,1);
Kp_hist = zeros(N,1); Ki_hist = zeros(N,1); Kd_hist = zeros(N,1);
zone_hist = strings(N,1);

I = 0; e_prev = 0; first_step = true; Df = 0;
ISE = 0; IAE = 0; ITAE = 0;

for k = 1:N-1
    tk = t(k);
    sp = getSetpointAt(opts.setpoint, tk);

    Tmeas = T(k);
    if isfield(opts,'noise') && opts.noise
        Tmeas = Tmeas + params.noise_std * randn();
    end
    Tmeas_hist(k) = Tmeas;

    err = sp - Tmeas;
    [Kp, Ki, Kd, zone] = resolveControllerGains(ctrl, Tmeas);

    % Anti-windup: clamp suku integral SEBELUM dipakai (sesuai firmware)
    I = I + Ki*err*dt;
    I = min(max(I, ctrl.I_min), ctrl.I_max);

    if first_step
        deriv = 0;
    else
        raw_d = (err - e_prev) / dt;
        if isfield(ctrl,'N_filter') && ctrl.N_filter > 0
            % derivative dengan low-pass filter orde-1 (dipakai versi
            % Simulink/IMC); firmware sesungguhnya pakai derivative mentah.
            alpha = ctrl.N_filter*dt / (1 + ctrl.N_filter*dt);
            Df = Df + alpha*(Kd*raw_d - Df);
            deriv = Df;
        else
            deriv = Kd * raw_d;
        end
    end
    first_step = false;

    u_unsat = Kp*err + I + deriv;
    u = min(max(u_unsat, params.PWM_MIN), params.PWM_MAX);

    u_delayed = u_buf(1);
    u_buf = [u_buf(2:end); u];

    [Kz, tauz] = zoneParams(T(k), params);
    dT = (-(T(k) - params.T_ambient) + Kz*u_delayed) / tauz;
    if isfield(opts,'disturbance') && ~isempty(opts.disturbance)
        dT = dT + opts.disturbance(tk);
    end
    T(k+1) = T(k) + dt*dT;

    e_prev = err;
    u_hist(k) = u; e_hist(k) = err;
    Kp_hist(k) = Kp; Ki_hist(k) = Ki; Kd_hist(k) = Kd; zone_hist(k) = zone;

    ISE = ISE + err^2*dt;
    IAE = IAE + abs(err)*dt;
    ITAE = ITAE + tk*abs(err)*dt;
end
% lengkapi sample terakhir
u_hist(N) = u_hist(N-1); e_hist(N) = opts_lastError(opts, t(N), T(N));
Kp_hist(N) = Kp_hist(N-1); Ki_hist(N) = Ki_hist(N-1); Kd_hist(N) = Kd_hist(N-1);
zone_hist(N) = zone_hist(N-1);
Tmeas_hist(N) = T(N);

out.t = t; out.T = T; out.Tmeas = Tmeas_hist; out.u = u_hist; out.e = e_hist;
out.Kp = Kp_hist; out.Ki = Ki_hist; out.Kd = Kd_hist; out.zone = zone_hist;
out.ISE = ISE; out.IAE = IAE; out.ITAE = ITAE;
end

function sp = getSetpointAt(setpoint, t)
if isa(setpoint,'function_handle')
    sp = setpoint(t);
else
    sp = setpoint;
end
end

function e = opts_lastError(opts, t, T)
sp = getSetpointAt(opts.setpoint, t);
e = sp - T;
end

% ------------------------------------------------------------
function out = simulateRelayOpenLoop(params, relayOpts)
% Simulasi bang-bang relay (open loop) untuk auto-tune Ziegler-Nichols.
%
% CATATAN PENTING (penyimpangan yang disengaja dari autotune.cpp):
% Fungsi autotune_update() pada firmware menyimpan s_target_setpoint
% tapi TIDAK PERNAH memakainya untuk menyalakan/mematikan fan --
% keputusan ON/OFF di sana murni bergantung pada switchPhase(), yang
% hanya terpicu oleh deteksi "turun dari puncak" / "naik dari lembah".
% Pada plant FOPDT (orde-1 + dead time) yang monoton, jika fan
% dibiarkan ON terus-menerus, suhu TIDAK PERNAH turun dari puncaknya
% sendiri -- sehingga switchPhase() tidak akan pernah terpicu dan
% relay tidak akan pernah berosilasi (auto-tune akan macet sampai
% AT_TIMEOUT_MS). Ini kemungkinan bug laten di firmware yang layak
% diperiksa ulang.
%
% Simulasi ini memakai relay bang-bang KLASIK Astrom-Hagglund yang
% benar: ON selama T < setpoint, OFF selama T >= setpoint. Dead time
% (Td) menyebabkan suhu tetap naik sesaat setelah fan dimatikan
% (begitu juga sebaliknya), sehingga terbentuk osilasi limit-cycle
% yang genuine. Deteksi puncak/lembah untuk menghitung Ku & Tu TETAP
% memakai algoritma band+laju (AT_TEMP_BAND, AT_STABIL_BAND) PERSIS
% seperti switchPhase() firmware, hanya dipisah dari logika ON/OFF.
%
% relayOpts: dt, T_sim_max, T_init, setpoint, PWM_high, PWM_low,
%            n_cycles, temp_band, stabil_band

dt = relayOpts.dt;
Nmax = round(relayOpts.T_sim_max/dt);
Td_steps = max(1, round(params.Td_plant/dt));
u_buf = zeros(Td_steps,1);

t = zeros(Nmax,1); T = zeros(Nmax,1); u_hist = zeros(Nmax,1);
T(1) = relayOpts.T_init;
sp = relayOpts.setpoint;

rising = true;          % status detektor ekstrem lokal (bukan status relay)
extreme = T(1);
cycle = 0;

peaks = []; peak_t = [];
valleys = []; valley_t = [];
last_peak_t = 0; have_first_peak = false;
period_sum = 0; period_n = 0;

k = 1;
while k < Nmax
    tk = (k-1)*dt;
    t(k) = tk;

    % --- perintah relay: bang-bang murni terhadap setpoint ---
    if T(k) < sp
        u_now = relayOpts.PWM_high;
    else
        u_now = relayOpts.PWM_low;
    end
    u_hist(k) = u_now;

    % --- deteksi puncak/lembah lokal (band + laju), independen dari relay ---
    rate = 0;
    if k > 1, rate = (T(k)-T(k-1))/dt; end
    if rising
        if T(k) > extreme, extreme = T(k); end
        fellFromPeak = (extreme - T(k)) >= relayOpts.temp_band;
        ratesFalling = (rate <= -relayOpts.stabil_band) || fellFromPeak;
        if fellFromPeak && ratesFalling
            peaks(end+1) = extreme; peak_t(end+1) = tk; %#ok<AGROW>
            if have_first_peak
                period_sum = period_sum + (tk - last_peak_t);
                period_n = period_n + 1;
            end
            have_first_peak = true; last_peak_t = tk;
            cycle = cycle + 1;
            rising = false;
            extreme = T(k);
        end
    else
        if T(k) < extreme, extreme = T(k); end
        roseFromValley = (T(k) - extreme) >= relayOpts.temp_band;
        ratesRising = (rate >= relayOpts.stabil_band) || roseFromValley;
        if roseFromValley && ratesRising
            valleys(end+1) = extreme; valley_t(end+1) = tk; %#ok<AGROW>
            rising = true;
            extreme = T(k);
        end
    end

    u_delayed = u_buf(1);
    u_buf = [u_buf(2:end); u_now];
    [Kz, tauz] = zoneParams(T(k), params);
    dT = (-(T(k) - params.T_ambient) + Kz*u_delayed) / tauz;
    T(k+1) = T(k) + dt*dT;

    if cycle >= relayOpts.n_cycles && ~isempty(peaks) && ~isempty(valleys) && period_n > 0
        k = k + 1;
        break;
    end
    k = k + 1;
end

t = t(1:k-1); T = T(1:k-1); u_hist = u_hist(1:k-1);

out.t = t; out.T = T; out.u = u_hist;
out.peaks = peaks; out.peak_t = peak_t;
out.valleys = valleys; out.valley_t = valley_t;
out.Tu = period_sum / max(period_n,1);
out.timed_out = (k >= Nmax);
end
