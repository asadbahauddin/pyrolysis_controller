# Simulasi Adaptive PID Pyrolysis — MATLAB/Simulink

Simulasi lengkap sistem kontrol adaptive PID untuk tungku pyrolysis,
dibuat sebagai **digital twin** dari firmware ESP32 pada repo ini
(`../pyrolysis_controller.ino` dan file pendukungnya). Persamaan,
konstanta, dan alur algoritma (relay auto-tune, gain scheduling,
coordinate descent, efficiency scoring) sengaja dibuat semirip
mungkin dengan `autotune.cpp`, `pid_controller.cpp`, dan
`learning_system.cpp` supaya hasil simulasi bisa dipakai sebagai
validasi/analisis firmware, bukan sekadar simulasi generik.

Judul skripsi: **Implementasi Adaptive PID untuk Kendali Suhu Tungku
Pyrolysis Berbasis ESP32**
(Isi `[Nama Mahasiswa]` / `[NIM]` / `[Tanggal]` di header tiap file
sebelum dikumpulkan.)

## Kebutuhan

- MATLAB R2020a atau lebih baru
- Control System Toolbox (dipakai `tf`, `pade`, `ss`)
- Simulink (hanya untuk file di folder `simulink/`)

## Cara menjalankan (pipeline MATLAB murni — tidak butuh Simulink)

```matlab
cd pyrolysis_pid_simulation
run('main_simulation.m')
```

Script ini menjalankan seluruh pipeline: bangun model plant, auto-tune
Z-N relay, gain scheduling, coordinate descent, perbandingan 5 metode
kontrol, pencarian setpoint optimal, dan 4 skenario tambahan
(disturbance rejection, setpoint change, multi-run learning). Semua
figure (300 DPI) tersimpan otomatis ke `results/`, beserta
`results/table_comparison.txt`.

Total waktu jalan ada di kisaran puluhan detik hingga beberapa menit
tergantung spesifikasi komputer (paling berat: 15 iterasi coordinate
descent x 3 percobaan x simulasi 30 menit untuk Bagian 5, dan simulasi
2 jam x 5 metode untuk Bagian 6).

## Cara pakai modul .m secara terpisah

Setiap file library (`plant_model.m`, `pid_design.m`, dst.) adalah
sebuah **module**: panggil sebagai fungsi tanpa argumen untuk
mendapatkan struct berisi function handle, karena MATLAB hanya
mengizinkan satu fungsi publik per nama file jika ingin dipanggil
dari file lain.

```matlab
plant = plant_model();
params = plant.defaultParams();
pidm = pid_design();
[Kp, Ki, Kd] = pidm.znOpenLoop(params.K_plant, params.tau_plant, params.Td_plant);
```

## Untuk Simulink

File `.slx` adalah berkas biner sehingga tidak bisa ditulis langsung
sebagai teks. Sebagai gantinya, folder `simulink/` berisi skrip
**pembangun model** yang memanggil Simulink API (`add_block`,
`add_line`, `save_system`, dst.) untuk menghasilkan file `.slx` yang
sesungguhnya. Jalankan sekali di MATLAB:

```matlab
cd pyrolysis_pid_simulation/simulink
build_all_simulink_models
```

Ini akan menghasilkan:
- `pid_controller_block.slx` — blok PID + gain scheduling + anti-windup
  (bisa dibuka & diperiksa berdiri sendiri)
- `pyrolysis_system.slx` — model lengkap: Setpoint (Step), PID +
  gain scheduling, saturasi PWM, plant nonlinear 3-zona + dead time,
  gangguan (disturbance), noise sensor MAX6675, Scope, To Workspace
- `adaptive_pid.slx` — sama seperti di atas, ditambah blok Memory +
  Display untuk Kp/Ki/Kd aktif dan blok From Workspace untuk overlay
  suhu run sebelumnya. Gain Kp/Ki/Kd dibaca dari variabel workspace
  (`Kp_current`, `Ki_current`, `Kd_current`) supaya bisa diperbarui
  MATLAB di antara run — jalankan
  `run_adaptive_pid_multirun.m` untuk demo multi-run learning yang
  benar-benar menjalankan model Simulink ini berulang kali.

**Skrip pembangun ini ditulis mengikuti Simulink API standar namun
belum sempat diuji di sesi MATLAB interaktif (lingkungan pembuatan
kode ini tidak memiliki instalasi MATLAB/Simulink).** Setelah
menjalankan `build_all_simulink_models`, buka tiap `.slx` dan periksa
sambungan blok secara visual; laporkan/​perbaiki bila ada blok yang
gagal tersambung. Pipeline MATLAB murni (`main_simulation.m` dan
seluruh file `.m` di root folder) **sudah lengkap dan mandiri** —
tidak bergantung pada file Simulink sama sekali.

Solver: `ode23tb` (stiff), fixed-step 0.1 s, mengikuti
`LOOP_INTERVAL_MS = 100` pada firmware.

## Struktur file

```
pyrolysis_pid_simulation/
├── main_simulation.m       - jalankan ini
├── plant_model.m            - model FOPDT nonlinear 3-zona + closed-loop simulator
├── pid_design.m              - Z-N step, Z-N relay, IMC, gain default firmware
├── ziegler_nichols.m         - auto-tune relay (meniru autotune.cpp)
├── gain_scheduling.m         - gain scheduling 3-zona (hard-switch spt firmware + varian halus)
├── coordinate_descent.m      - coordinate descent (resimulasi penuh + replay spt firmware)
├── efficiency_scoring.m      - efficiency score (formula sama dgn learning_system.cpp)
├── compare_methods.m         - 5 metode kontrol + tabel metrik
├── plot_results.m            - semua fungsi visualisasi (300 DPI)
├── simulink/
│   ├── build_pid_controller_block.m
│   ├── build_pyrolysis_system.m
│   ├── build_adaptive_pid.m
│   ├── build_all_simulink_models.m   - jalankan ini utk generate .slx
│   ├── run_adaptive_pid_multirun.m
│   └── (*.slx dihasilkan setelah build_all_simulink_models dijalankan)
└── results/
    ├── fig1_relay_autotune.png
    ├── fig2_comparison_step_response.png
    ├── fig3_error_comparison.png
    ├── fig4_performance_metrics.png
    ├── fig5_pid_convergence.png
    ├── fig6_gain_scheduling.png
    ├── fig7_efficiency_map.png
    ├── fig8_multirun_learning.png
    ├── fig8b_pid_evolution_per_run.png
    ├── fig9_disturbance_rejection.png
    ├── fig10_setpoint_change.png
    └── table_comparison.txt
```

## Catatan fidelitas terhadap firmware ESP32

| Aspek | Firmware (`config.h` / `.cpp`) | Simulasi MATLAB |
|---|---|---|
| Gain default | `DEFAULT_KP/KI/KD = 2.0 / 0.1 / 10.0` | sama (dipakai metode "Konvensional") |
| Relay auto-tune | `AT_PWM_HIGH=200/255≈78%`, `AT_CYCLES=4`, `AT_TEMP_BAND=5`, `AT_STABIL_BAND=2`, deteksi peak/valley via band+laju | sama, unit PWM dalam % (0-100) |
| Formula Z-N relay | `Kp=0.6Ku, Ki=1.2Ku/Tu, Kd=0.075KuTu` | sama persis |
| Gain scheduling | **hanya Kp** dijadwal per zona (hard-switch di 200/400°C), Ki/Kd tetap | sama (`mode='zone'`); varian halus 3-gain (`zone_smooth`) disediakan sebagai studi tambahan |
| Anti-windup | clamp integral ke `±255` (skala PWM 0-255) | clamp ke `±100` (skala PWM 0-100%, setara) |
| Coordinate descent | replay dari log run sebelumnya, proxy `k_process = dE/dP`, langkah `±5%` per Kp→Ki→Kd | `coordinateDescentReplay()` = replika persis; `coordinateDescent()` = varian resimulasi penuh (lebih akurat, dipakai studi offline) |
| Efficiency score | `w1*produktivitas - w2*avg_pwm - w3*(ISE/1000)`, `w1/w2/w3 = 0.6/0.2/0.2` | sama persis |
| Produktivitas (mL/jam) | diukur sensor hall tipping-bucket (perangkat keras nyata) | **dimodelkan empiris** (puncak ~400°C, turun di suhu tinggi akibat cracking sekunder) — bukan data terukur, lihat `efficiency_scoring.m` |

Plant termal (`K=3.5 °C/%PWM`, `tau=180 s`, `Td=15 s`, serta variasi
per-zona) adalah **parameter estimasi** untuk tungku pyrolysis sedang
(bukan hasil identifikasi dari data eksperimen nyata) — sesuaikan
`plant_model.m` → `defaultParams()` bila tersedia data pengukuran asli.
