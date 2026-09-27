function api = pid_design()
% ============================================================
% pid_design.m
% Desain & tuning PID: Z-N step response (open loop), Z-N relay
% (closed loop), dan IMC (Internal Model Control).
%
% Judul  : Implementasi Adaptive PID untuk Kendali Suhu Tungku
%          Pyrolysis Berbasis ESP32
% Nama   : [Nama Mahasiswa]
% NIM    : [NIM]
% Tanggal: [Tanggal]
% ============================================================

api.znOpenLoop              = @znOpenLoop;
api.znRelayMethod           = @znRelayMethod;
api.imcTuning                = @imcTuning;
api.buildPIDTransferFunction = @buildPIDTransferFunction;
api.defaultCtrl              = @defaultCtrl;
api.firmwareDefaultGains     = @firmwareDefaultGains;
end

% ------------------------------------------------------------
function [Kp, Ki, Kd] = znOpenLoop(K, tau, Td)
% Ziegler-Nichols berdasarkan step response (open loop, PID klasik)
Kp = 1.2*tau / (K*Td);
Ki = Kp / (2*Td);
Kd = Kp * 0.5*Td;
end

% ------------------------------------------------------------
function [Kp, Ki, Kd] = znRelayMethod(Ku, Tu)
% Ziegler-Nichols berdasarkan osilasi relay (closed loop).
% SAMA PERSIS dengan formula di autotune.cpp firmware ESP32:
%   Kp = 0.6*Ku ; Ki = 1.2*Ku/Tu (= 2*Kp/Tu) ; Kd = 0.075*Ku*Tu (= Kp*Tu/8)
Kp = 0.6 * Ku;
Ki = 2 * Kp / Tu;
Kd = Kp * Tu / 8;
end

% ------------------------------------------------------------
function [Kp, Ki, Kd] = imcTuning(K, tau, Td, lambda)
% IMC tuning untuk FOPDT. lambda = konstanta waktu closed-loop yang
% diinginkan (disarankan lambda antara 0.3*tau sampai 1.0*tau).
% Rumus IMC-PID standar (Rivera/Skogestad, dengan Pade orde-1 utk Td):
Kp = (2*tau + Td) / (K * (2*lambda + Td));
Ti = tau + Td/2;
Ki = Kp / Ti;
Td_eq = (tau*Td) / (2*tau + Td);
Kd = Kp * Td_eq;
end

% ------------------------------------------------------------
function pid_tf = buildPIDTransferFunction(Kp, Ki, Kd, N)
% Transfer function PID dengan filter derivative:
%   C(s) = Kp + Ki/s + Kd*N*s/(s+N)
s = tf('s');
pid_tf = Kp + Ki/s + Kd*N*s/(s+N);
end

% ------------------------------------------------------------
function [Kp, Ki, Kd] = firmwareDefaultGains()
% Nilai default hardcoded di firmware (config.h): DEFAULT_KP/KI/KD.
% Dipakai sebagai baseline "PID Konvensional (tanpa tuning)".
Kp = 2.0; Ki = 0.1; Kd = 10.0;
end

% ------------------------------------------------------------
function ctrl = defaultCtrl(Kp, Ki, Kd, mode)
% Bangun struct ctrl standar untuk plant_model.simulateClosedLoop(),
% konsisten dengan batas anti-windup & batas zona firmware
% (I_MAX/I_MIN = +-255 pada skala PWM 0-255 => +-100 pada skala %).
if nargin < 4, mode = 'fixed'; end
ctrl.mode = mode;
ctrl.Kp = Kp; ctrl.Ki = Ki; ctrl.Kd = Kd;
ctrl.I_max = 100; ctrl.I_min = -100;
ctrl.zone_cold_max = 200; ctrl.zone_mid_max = 400;
ctrl.N_filter = 0; % 0 = derivative mentah (sesuai firmware)

% tabel default untuk mode 'zone_smooth': [T_breakpoint, multKp, multKi, multKd]
ctrl.schedule_table = [ ...
    100, 1.3, 0.8, 0.7; ...
    300, 1.0, 1.0, 1.0; ...
    500, 0.7, 1.2, 1.3];
end
