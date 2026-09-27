function api = gain_scheduling()
% ============================================================
% gain_scheduling.m
% Gain scheduling 3-zona untuk PID
%
% Judul  : Implementasi Adaptive PID untuk Kendali Suhu Tungku
%          Pyrolysis Berbasis ESP32
% Nama   : [Nama Mahasiswa]
% NIM    : [NIM]
% Tanggal: [Tanggal]
%
% CATATAN FIDELITAS:
% Firmware ESP32 (pid_controller.cpp) HANYA menjadwal Kp per zona
% (hard switch di T=200 & T=400degC), sedangkan Ki/Kd dibiarkan tetap
% (diatur lewat coordinate descent per-setpoint, bukan per-zona).
% gainSchedule() di bawah ini mereplikasi perilaku firmware itu.
% interpolateGainSchedule() adalah varian penelitian tambahan yang
% menghaluskan transisi zona dan (opsional) turut menjadwal Ki & Kd,
% dipakai untuk studi perbandingan pada method "Adaptive PID".
% ============================================================

api.gainSchedule            = @gainSchedule;
api.interpolateGainSchedule = @interpolateGainSchedule;
api.plotGainSchedule        = @plotGainSchedule;
end

% ------------------------------------------------------------
function [Kp_act, Ki_act, Kd_act] = gainSchedule(T_current, Kp_base, Ki_base, Kd_base)
% Gain scheduling sesuai firmware: hard-switch, HANYA Kp yang berubah.
%   Zona dingin  < 200 degC : Kp*1.3
%   Zona tengah 200-400 degC: Kp*1.0
%   Zona panas   > 400 degC : Kp*0.7
if T_current < 200
    multKp = 1.3;
elseif T_current <= 400
    multKp = 1.0;
else
    multKp = 0.7;
end
Kp_act = Kp_base * multKp;
Ki_act = Ki_base;
Kd_act = Kd_base;
end

% ------------------------------------------------------------
function [Kp_gs, Ki_gs, Kd_gs] = interpolateGainSchedule(T_current, schedule_table)
% Interpolasi halus antar breakpoint zona (tidak ada step jump).
% schedule_table: [T_breakpoint, multKp, multKi, multKd] -- baris terurut
% naik berdasarkan T_breakpoint. multKp/Ki/Kd adalah pengali terhadap
% gain dasar (base gain); base gain dikalikan di luar fungsi ini.
Tb = schedule_table(:,1);
if T_current <= Tb(1)
    m = schedule_table(1,2:4);
elseif T_current >= Tb(end)
    m = schedule_table(end,2:4);
else
    m = interp1(Tb, schedule_table(:,2:4), T_current, 'linear');
end
Kp_gs = m(1); Ki_gs = m(2); Kd_gs = m(3);
end

% ------------------------------------------------------------
function fig = plotGainSchedule(schedule_table, T_range)
% Plot pengali Kp, Ki, Kd terhadap suhu, dengan garis hard-switch
% firmware sebagai pembanding.
Kp_line = zeros(size(T_range)); Ki_line = zeros(size(T_range)); Kd_line = zeros(size(T_range));
Kp_hard = zeros(size(T_range));
for i = 1:numel(T_range)
    [Kp_gs, Ki_gs, Kd_gs] = interpolateGainSchedule(T_range(i), schedule_table);
    Kp_line(i) = Kp_gs; Ki_line(i) = Ki_gs; Kd_line(i) = Kd_gs;
    [khp, ~, ~] = gainSchedule(T_range(i), 1, 1, 1);
    Kp_hard(i) = khp;
end

fig = figure('Name','Gain Schedule vs Suhu','Color','white','Position',[100 100 800 600]);
plot(T_range, Kp_line, 'b-', 'LineWidth', 1.8); hold on;
plot(T_range, Ki_line, 'r-', 'LineWidth', 1.8);
plot(T_range, Kd_line, 'g-', 'LineWidth', 1.8);
plot(T_range, Kp_hard, 'b--', 'LineWidth', 1.2);
xline(200, 'k:'); xline(400, 'k:');
grid on; box on; set(gca,'FontSize',12);
xlabel('Suhu (degC)'); ylabel('Pengali gain (terhadap base)');
title('Gain Scheduling 3-Zona: halus (garis penuh) vs firmware hard-switch (putus-putus)');
legend('multKp (halus)','multKi (halus)','multKd (halus)','multKp (firmware, hard-switch)', ...
    'Location','best');
text(200, max(ylim)*0.95, ' Dingin|Tengah', 'FontSize',9);
text(400, max(ylim)*0.95, ' Tengah|Panas', 'FontSize',9);
end
