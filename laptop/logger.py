#!/usr/bin/env python3
"""
logger.py - pengambil data ESP32 pyrolysis lewat WiFi (pengganti SD card).

Cara pakai:
  1. Di laptop, sambungkan WiFi ke access point ESP32:
        SSID     : Pyrolysis-Diag
        Password : 12345678        (ESP32 = 192.168.4.1)
     Laptop tidak butuh internet.
  2. Jalankan:
        python logger.py log                # catat data live ke CSV + grafik langsung
                                              # (grafik otomatis, butuh matplotlib)
        python logger.py log --no-plot      # tanpa grafik (cuma teks+CSV)
        python logger.py step-data          # unduh seluruh rekaman Step Test terakhir dari ESP32
        python logger.py status             # cek koneksi, tampilkan satu paket data

Grafik (subplot atas) menampilkan suhu mentah, suhu hasil EMA 3-tahap (lebih
tebal), dan garis putus-putus setpoint yang sedang aktif (kosong saat idle)
-- dipakai untuk melihat sekilas overshoot dan settling time selama Uji
Metode berjalan. CSV menyimpan SEMUA: temp_c (mentah) serta temp_ema1/2/3
(hasil tiap tahap EMA, --ema-alpha mengatur kehalusannya, default 0.3).
Filter ini murni untuk tampilan/analisis di laptop -- TIDAK mengubah suhu
yang dipakai loop kontrol PID di ESP32 (itu sudah punya EMA sendiri,
internal, terpisah). EMA ini cocok untuk meredam noise sensor antar-sampel,
TAPI TIDAK memperbaiki osilasi lambat (periode menit) akibat tuning PID --
itu soal lain yang butuh penyesuaian gain, bukan filter.

Saat `log` berjalan dan Anda menjalankan Step Test / Relay Test / Uji Metode
dari dashboard (http://192.168.4.1), logger otomatis menyimpan HASIL uji itu
begitu selesai (baik karena sukses, distop manual, maupun gagal):
  data/result_step_<waktu>.json    + data/step_<waktu>.csv    (rekaman penuh)
  data/result_relay_<waktu>.json
  data/result_method_<waktu>.json  + data/method_<waktu>.csv  (waktu tetes)
Untuk Uji Metode, logger JUGA menghitung sendiri overshoot (%) dan settling
time (detik, band +-2% sekitar setpoint) dari data yang baru dicatat, dan
mencetaknya ke layar -- overshoot/settling time cuma relevan untuk Uji
Metode (closed-loop mengejar setpoint), bukan Step/Relay Test.

Hanya memakai library standar Python 3 (matplotlib opsional -- tanpanya,
logger tetap jalan penuh, cuma tanpa grafik).
"""
import argparse
import csv
import json
import os
import sys
import time
import urllib.error
import urllib.request
from collections import deque
from datetime import datetime

DEFAULT_HOST = "192.168.4.1"
CSV_HEADER = ["pc_time", "esp_ms", "temp_c", "temp_ema1", "temp_ema2", "temp_ema3",
              "fan_pct", "oil", "water", "tips", "mode", "relay", "sp", "kp", "ki", "kd", "zone"]


class CascadedEMA:
    """Exponential Moving Average 3 TAHAP (cascade): keluaran tahap 1 jadi
    masukan tahap 2, keluaran tahap 2 jadi masukan tahap 3. Ini pure analisis/
    tampilan di sisi laptop -- dihitung dari suhu MENTAH yang sama persis
    dengan yang dipakai ESP32 (field "temp" di /data/live, bukan nilai hasil
    filter internal controller), TIDAK mengubah apa pun di kontrol ESP32.

    CATATAN PENTING: ini meredam NOISE cepat (antar-sampel), BUKAN obat utk
    osilasi lambat (periode menit) akibat tuning PID/dead time -- makin
    berat filter dipasang di JALUR KONTROL (bukan cuma tampilan ini), makin
    besar risiko menambah lag dan memperparah osilasi semacam itu.
    """

    def __init__(self, alpha=0.3):
        self.alpha = alpha
        self.s1 = self.s2 = self.s3 = None

    def update(self, raw):
        if raw is None:
            return (None, None, None) if self.s1 is None else (self.s1, self.s2, self.s3)
        a = self.alpha
        self.s1 = raw if self.s1 is None else self.s1 + a * (raw - self.s1)
        self.s2 = self.s1 if self.s2 is None else self.s2 + a * (self.s1 - self.s2)
        self.s3 = self.s2 if self.s3 is None else self.s3 + a * (self.s2 - self.s3)
        return (self.s1, self.s2, self.s3)


def http_get(host, path, timeout=3.0):
    """GET http://host/path -> teks respons. Melempar OSError/URLError jika gagal."""
    url = "http://%s%s" % (host, path)
    with urllib.request.urlopen(url, timeout=timeout) as resp:
        return resp.read().decode("utf-8", errors="replace")


def gui_sleep(seconds, plot):
    """time.sleep() yang tetap memompa event loop Tk/matplotlib supaya window
    grafik tidak dianggap Windows sebagai "Not Responding" saat menunggu
    (dipakai terutama saat koneksi ESP32 putus-nyambung dan logger retry)."""
    if not plot or seconds <= 0:
        time.sleep(max(seconds, 0))
        return
    end = time.time() + seconds
    while True:
        remaining = end - time.time()
        if remaining <= 0:
            break
        plot.plt.pause(min(0.1, remaining))


def get_json(host, path, timeout=3.0):
    return json.loads(http_get(host, path, timeout))


def stamp():
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def compute_overshoot_settling(times, temps, sp, t0_temp):
    """Overshoot (%) & settling time (detik, band +-2% sekitar sp) dari satu
    segmen Uji Metode. t0_temp = suhu di awal segmen (dipakai sbg baseline,
    sama seperti compute_response_metrics.m di simulasi MATLAB)."""
    if not temps or sp is None:
        return None, None
    span = sp - t0_temp
    if span == 0:
        return None, None
    peak = max(temps) if span > 0 else min(temps)
    overshoot = max(0.0, (peak - sp) / abs(span) * 100.0) if span > 0 else max(0.0, (sp - peak) / abs(span) * 100.0)
    band = 0.02 * max(abs(sp), 1.0)
    settle_t = times[0]
    for t, v in zip(times, temps):
        if abs(v - sp) > band:
            settle_t = t
    return overshoot, settle_t


def save_finished_test(host, out_dir, kind):
    """Dipanggil saat mode berubah step/relay/method -> idle: simpan hasil uji dari ESP32."""
    ts = stamp()
    try:
        if kind == "step":
            res = get_json(host, "/step/status")
            path = os.path.join(out_dir, "result_step_%s.json" % ts)
            with open(path, "w", encoding="utf-8") as f:
                json.dump(res, f, indent=2)
            if res.get("status") == "done":
                print("\n[STEP SELESAI] L=%.1f s  T=%.1f s  K=%.4f C/%%  (dT=%.1f C)%s" % (
                    res["L"], res["T"], res["K"], res["dT"],
                    ("  ! " + res["warn"]) if res.get("warn") else ""))
                try:
                    csv_path = os.path.join(out_dir, "step_%s.csv" % ts)
                    with open(csv_path, "w", encoding="utf-8", newline="") as f:
                        f.write(http_get(host, "/step/data", timeout=15.0))
                    print("  rekaman penuh: %s" % csv_path)
                except (OSError, urllib.error.URLError) as e:
                    print("  gagal unduh /step/data: %s" % e)
            else:
                print("\n[STEP %s] %s" % (res.get("status", "?").upper(), res.get("error", "")))
            print("  hasil: %s" % path)
        elif kind == "relay":
            res = get_json(host, "/autotune/status")
            path = os.path.join(out_dir, "result_relay_%s.json" % ts)
            with open(path, "w", encoding="utf-8") as f:
                json.dump(res, f, indent=2)
            if res.get("status") == "done":
                print("\n[RELAY SELESAI] Ku=%.4f Tu=%.1fs  Kp=%.4f Ki=%.4f Kd=%.4f" % (
                    res["Ku"], res["Tu"], res["Kp"], res["Ki"], res["Kd"]))
            else:
                print("\n[RELAY %s] %s" % (res.get("status", "?").upper(), res.get("error", "")))
            print("  hasil: %s" % path)
        else:  # method
            res = get_json(host, "/method/status")
            path = os.path.join(out_dir, "result_method_%s.json" % ts)
            with open(path, "w", encoding="utf-8") as f:
                json.dump(res, f, indent=2)
            if res.get("status") == "done":
                print("\n[METODE SELESAI] durasi=%ds  tetes=%d (%.2f mL)  RMS error=%.2f C" % (
                    res["durS"], res["tips"], res["volumeMl"], res["rmsErr"]))
            else:
                print("\n[METODE %s] %s" % (res.get("status", "?").upper(), res.get("error", "")))
            print("  hasil: %s" % path)
            # Waktu tetes tersimpan di ESP32 walau uji distop manual/gagal, bukan hanya "done"
            try:
                csv_path = os.path.join(out_dir, "method_%s.csv" % ts)
                with open(csv_path, "w", encoding="utf-8", newline="") as f:
                    f.write(http_get(host, "/method/data", timeout=15.0))
                print("  waktu tetes  : %s" % csv_path)
            except urllib.error.HTTPError:
                pass  # belum ada tetes tercatat -> wajar, tidak perlu dilaporkan sbg error
            except (OSError, urllib.error.URLError) as e:
                print("  gagal unduh /method/data: %s" % e)
    except (OSError, urllib.error.URLError, ValueError, KeyError) as e:
        print("\n[!] gagal menyimpan hasil uji %s: %s" % (kind, e))


class LivePlot:
    """Grafik suhu & fan langsung. Nonaktif otomatis jika matplotlib tidak ada."""

    def __init__(self, window_s=1800):
        import matplotlib.pyplot as plt  # ImportError ditangani pemanggil
        self.plt = plt
        self.t = deque(); self.temp = deque(); self.temp_ema = deque(); self.fan = deque(); self.sp = deque()
        self.window = window_s
        plt.ion()
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, sharex=True, figsize=(9, 6))
        self.l1, = self.ax1.plot([], [], "g-", linewidth=1, alpha=0.5, label="Suhu (mentah)")
        self.lema, = self.ax1.plot([], [], "orange", linewidth=1.8, label="Suhu (EMA 3 tahap)")
        self.lsp, = self.ax1.plot([], [], "k--", linewidth=1.2, label="Setpoint")
        self.l2, = self.ax2.plot([], [], "b-", linewidth=1.2)
        self.ax1.set_ylabel("Suhu (C)"); self.ax2.set_ylabel("Fan (%)")
        self.ax2.set_xlabel("Waktu (s)"); self.ax2.set_ylim(-5, 105)
        self.ax1.legend(loc="upper left", fontsize=9)
        for ax in (self.ax1, self.ax2):
            ax.grid(True)
        self.fig.suptitle("ESP32 Pyrolysis - data live")

    def add(self, t, temp, fan, sp=None, temp_ema=None):
        # sp<=0 berarti idle (tidak ada uji aktif) -> putus garis (NaN), bukan nyambung ke 0
        self.t.append(t); self.temp.append(temp); self.fan.append(fan)
        self.temp_ema.append(temp_ema)
        self.sp.append(sp if sp else float("nan"))
        while self.t and t - self.t[0] > self.window:
            self.t.popleft(); self.temp.popleft(); self.temp_ema.popleft(); self.fan.popleft(); self.sp.popleft()
        pts = [(a, b) for a, b in zip(self.t, self.temp) if b is not None]
        pts_e = [(a, b) for a, b in zip(self.t, self.temp_ema) if b is not None]
        self.l1.set_data([p[0] for p in pts], [p[1] for p in pts])
        self.lema.set_data([p[0] for p in pts_e], [p[1] for p in pts_e])
        self.lsp.set_data(list(self.t), list(self.sp))
        self.l2.set_data(list(self.t), list(self.fan))
        self.ax1.relim(); self.ax1.autoscale_view()
        self.ax2.set_xlim(self.t[0], max(self.t[-1], self.t[0] + 60))
        self.plt.pause(0.001)

    def annotate(self, text):
        """Tampilkan kotak teks hasil (overshoot/settling) di pojok grafik suhu."""
        if hasattr(self, "_annot"):
            self._annot.remove()
        self._annot = self.ax1.text(
            0.99, 0.02, text, transform=self.ax1.transAxes, fontsize=9,
            ha="right", va="bottom",
            bbox=dict(boxstyle="round", facecolor="lightyellow", edgecolor="black"))
        self.plt.pause(0.001)


def cmd_log(args):
    os.makedirs(args.out_dir, exist_ok=True)
    csv_path = os.path.join(args.out_dir, "log_%s.csv" % stamp())

    plot = None
    if not args.no_plot:
        try:
            plot = LivePlot()
        except ImportError:
            print("matplotlib tidak terpasang -> grafik dimatikan (pip install matplotlib untuk mengaktifkan)")

    print("Menghubungi ESP32 di %s ... (Ctrl+C untuk berhenti)" % args.host)
    print("Menyimpan ke: %s\n" % csv_path)

    prev_mode = "idle"
    method_seg = None   # {"times":[], "temps":[], "sp":..} selama mode=="method"
    ema = CascadedEMA(args.ema_alpha)
    lost = 0
    n = 0
    t0 = time.time()
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(CSV_HEADER)
        try:
            while args.count == 0 or n < args.count:
                loop_start = time.time()
                try:
                    d = get_json(args.host, "/data/live", timeout=2.0)
                except (OSError, urllib.error.URLError, ValueError) as e:
                    lost += 1
                    print("\r[!] tidak ada respons dari ESP32 (%dx): %s        " % (lost, e), end="")
                    gui_sleep(args.interval, plot)
                    continue
                if lost:
                    print("\n[OK] koneksi pulih setelah %d percobaan gagal" % lost)
                    lost = 0

                now = datetime.now()
                temp = d.get("temp")
                sp = d.get("sp")
                e1, e2, e3 = ema.update(temp)

                w.writerow([now.isoformat(timespec="milliseconds"), d.get("ms"),
                            "" if temp is None else temp,
                            "" if e1 is None else round(e1, 3),
                            "" if e2 is None else round(e2, 3),
                            "" if e3 is None else round(e3, 3),
                            d.get("fan"), d.get("oil"), d.get("water"), d.get("tips"),
                            d.get("mode"), int(bool(d.get("relay"))), sp,
                            d.get("kp"), d.get("ki"), d.get("kd"), d.get("zone")])
                f.flush()
                n += 1

                print("%s  %-5s  T=%s C  (EMA3=%s)  fan=%3s%%  oil=%s water=%s tips=%s" % (
                    now.strftime("%H:%M:%S"), d.get("mode"),
                    ("%7.2f" % temp) if temp is not None else "   ERR ",
                    ("%.2f" % e3) if e3 is not None else "  --",
                    d.get("fan"), d.get("oil"), d.get("water"), d.get("tips")))

                t_rel = time.time() - t0
                if plot:
                    plot.add(t_rel, temp, d.get("fan") or 0, sp, e3)

                mode = d.get("mode", "idle")

                # Lacak segmen Uji Metode berjalan supaya bisa hitung overshoot/settling
                # time sendiri dari data yang baru dicatat (lepas dari ringkasan ESP32).
                if mode == "method":
                    if method_seg is None:
                        method_seg = {"t_ref": t_rel, "times": [], "temps": [], "sp": sp}
                    method_seg["times"].append(t_rel - method_seg["t_ref"])
                    if temp is not None:
                        method_seg["temps"].append(temp)
                    if sp:
                        method_seg["sp"] = sp

                if prev_mode in ("step", "relay", "method") and mode == "idle":
                    save_finished_test(args.host, args.out_dir, prev_mode)
                    if prev_mode == "method" and method_seg and method_seg["temps"]:
                        ov, settl = compute_overshoot_settling(
                            method_seg["times"], method_seg["temps"],
                            method_seg["sp"], method_seg["temps"][0])
                        if ov is not None:
                            txt = "Overshoot=%.1f%%  Settling=%.0fs  (sp=%.1fC)" % (ov, settl, method_seg["sp"])
                            print("  perkiraan dari log      : %s" % txt)
                            if plot:
                                plot.annotate(txt)
                    method_seg = None
                prev_mode = mode

                delay = args.interval - (time.time() - loop_start)
                gui_sleep(delay, plot)
        except KeyboardInterrupt:
            print("\nDihentikan.")
    print("%d baris tersimpan di %s" % (n, csv_path))


def cmd_step_data(args):
    try:
        text = http_get(args.host, "/step/data", timeout=20.0)
    except urllib.error.HTTPError as e:
        print("ESP32 menjawab %d: belum ada data Step Test (jalankan dulu dari dashboard)." % e.code)
        return 1
    except (OSError, urllib.error.URLError) as e:
        print("Gagal terhubung ke %s: %s" % (args.host, e))
        return 1
    out = args.out or os.path.join(args.out_dir, "step_%s.csv" % stamp())
    os.makedirs(os.path.dirname(out) or ".", exist_ok=True)
    with open(out, "w", encoding="utf-8", newline="") as f:
        f.write(text)
    print("%d baris tersimpan di %s" % (text.count("\n") - 1, out))
    return 0


def cmd_status(args):
    try:
        d = get_json(args.host, "/data/live")
    except (OSError, urllib.error.URLError, ValueError) as e:
        print("Gagal terhubung ke %s: %s\nPastikan WiFi laptop tersambung ke 'Pyrolysis-Diag'." % (args.host, e))
        return 1
    print(json.dumps(d, indent=2))
    return 0


def main():
    p = argparse.ArgumentParser(description="Logger data ESP32 pyrolysis lewat WiFi")
    p.add_argument("command", choices=["log", "step-data", "status"])
    p.add_argument("--host", default=DEFAULT_HOST, help="alamat ESP32 (default %s)" % DEFAULT_HOST)
    p.add_argument("--interval", type=float, default=1.0, help="detik antar pembacaan (default 1.0)")
    p.add_argument("--out-dir", default="data", help="folder keluaran (default ./data)")
    p.add_argument("--out", help="nama file keluaran (khusus step-data)")
    p.add_argument("--no-plot", action="store_true", help="matikan grafik langsung (default: aktif kalau matplotlib ada)")
    p.add_argument("--ema-alpha", type=float, default=0.3,
                    help="koefisien EMA 3-tahap (0-1, makin kecil makin halus/lambat; default 0.3)")
    p.add_argument("--count", type=int, default=0, help="berhenti setelah N sampel (0 = tanpa batas)")
    args = p.parse_args()

    if args.interval < 0.5:
        print("interval minimum 0.5 detik (MAX6675 butuh ~250 ms per konversi)")
        return 2
    if not (0.0 < args.ema_alpha <= 1.0):
        print("--ema-alpha harus di antara 0 (eksklusif) dan 1")
        return 2
    if args.command == "log":
        cmd_log(args)
        return 0
    if args.command == "step-data":
        return cmd_step_data(args)
    return cmd_status(args)


if __name__ == "__main__":
    sys.exit(main())
