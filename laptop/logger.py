#!/usr/bin/env python3
"""
logger.py - pengambil data ESP32 pyrolysis lewat WiFi (pengganti SD card).

Cara pakai:
  1. Di laptop, sambungkan WiFi ke access point ESP32:
        SSID     : Pyrolysis-Diag
        Password : 12345678        (ESP32 = 192.168.4.1)
     Laptop tidak butuh internet.
  2. Jalankan:
        python logger.py log                # catat data live ke CSV (Ctrl+C untuk berhenti)
        python logger.py log --plot         # + grafik suhu/fan langsung (butuh matplotlib)
        python logger.py step-data          # unduh seluruh rekaman Step Test terakhir dari ESP32
        python logger.py status             # cek koneksi, tampilkan satu paket data

Saat `log` berjalan dan Anda menjalankan Step Test / Relay Test / Uji Metode
dari dashboard (http://192.168.4.1), logger otomatis menyimpan HASIL uji itu
begitu selesai (baik karena sukses, distop manual, maupun gagal):
  data/result_step_<waktu>.json    + data/step_<waktu>.csv    (rekaman penuh)
  data/result_relay_<waktu>.json
  data/result_method_<waktu>.json  + data/method_<waktu>.csv  (waktu tetes)

Hanya memakai library standar Python 3 (matplotlib opsional).
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
CSV_HEADER = ["pc_time", "esp_ms", "temp_c", "fan_pct", "oil", "water",
              "tips", "mode", "relay", "sp", "kp", "ki", "kd", "zone"]


def http_get(host, path, timeout=3.0):
    """GET http://host/path -> teks respons. Melempar OSError/URLError jika gagal."""
    url = "http://%s%s" % (host, path)
    with urllib.request.urlopen(url, timeout=timeout) as resp:
        return resp.read().decode("utf-8", errors="replace")


def get_json(host, path, timeout=3.0):
    return json.loads(http_get(host, path, timeout))


def stamp():
    return datetime.now().strftime("%Y%m%d_%H%M%S")


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
        self.t = deque(); self.temp = deque(); self.fan = deque()
        self.window = window_s
        plt.ion()
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, sharex=True, figsize=(9, 6))
        self.l1, = self.ax1.plot([], [], "g-")
        self.l2, = self.ax2.plot([], [], "b-")
        self.ax1.set_ylabel("Suhu (C)"); self.ax2.set_ylabel("Fan (%)")
        self.ax2.set_xlabel("Waktu (s)"); self.ax2.set_ylim(-5, 105)
        for ax in (self.ax1, self.ax2):
            ax.grid(True)
        self.fig.suptitle("ESP32 Pyrolysis - data live")

    def add(self, t, temp, fan):
        self.t.append(t); self.temp.append(temp); self.fan.append(fan)
        while self.t and t - self.t[0] > self.window:
            self.t.popleft(); self.temp.popleft(); self.fan.popleft()
        pts = [(a, b) for a, b in zip(self.t, self.temp) if b is not None]
        self.l1.set_data([p[0] for p in pts], [p[1] for p in pts])
        self.l2.set_data(list(self.t), list(self.fan))
        self.ax1.relim(); self.ax1.autoscale_view()
        self.ax2.set_xlim(self.t[0], max(self.t[-1], self.t[0] + 60))
        self.plt.pause(0.001)


def cmd_log(args):
    os.makedirs(args.out_dir, exist_ok=True)
    csv_path = os.path.join(args.out_dir, "log_%s.csv" % stamp())

    plot = None
    if args.plot:
        try:
            plot = LivePlot()
        except ImportError:
            print("matplotlib tidak terpasang -> grafik dimatikan (pip install matplotlib)")

    print("Menghubungi ESP32 di %s ... (Ctrl+C untuk berhenti)" % args.host)
    print("Menyimpan ke: %s\n" % csv_path)

    prev_mode = "idle"
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
                    d = get_json(args.host, "/data/live")
                except (OSError, urllib.error.URLError, ValueError) as e:
                    lost += 1
                    print("\r[!] tidak ada respons dari ESP32 (%dx): %s        " % (lost, e), end="")
                    time.sleep(args.interval)
                    continue
                if lost:
                    print("\n[OK] koneksi pulih setelah %d percobaan gagal" % lost)
                    lost = 0

                now = datetime.now()
                w.writerow([now.isoformat(timespec="milliseconds"), d.get("ms"),
                            "" if d.get("temp") is None else d["temp"], d.get("fan"),
                            d.get("oil"), d.get("water"), d.get("tips"),
                            d.get("mode"), int(bool(d.get("relay"))), d.get("sp"),
                            d.get("kp"), d.get("ki"), d.get("kd"), d.get("zone")])
                f.flush()
                n += 1

                temp = d.get("temp")
                print("%s  %-5s  T=%s C  fan=%3s%%  oil=%s water=%s tips=%s" % (
                    now.strftime("%H:%M:%S"), d.get("mode"),
                    ("%7.2f" % temp) if temp is not None else "   ERR ",
                    d.get("fan"), d.get("oil"), d.get("water"), d.get("tips")))

                if plot:
                    plot.add(time.time() - t0, temp, d.get("fan") or 0)

                mode = d.get("mode", "idle")
                if prev_mode in ("step", "relay", "method") and mode == "idle":
                    save_finished_test(args.host, args.out_dir, prev_mode)
                prev_mode = mode

                delay = args.interval - (time.time() - loop_start)
                if delay > 0:
                    time.sleep(delay)
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
    p.add_argument("--plot", action="store_true", help="grafik langsung (butuh matplotlib)")
    p.add_argument("--count", type=int, default=0, help="berhenti setelah N sampel (0 = tanpa batas)")
    args = p.parse_args()

    if args.interval < 0.5:
        print("interval minimum 0.5 detik (MAX6675 butuh ~250 ms per konversi)")
        return 2
    if args.command == "log":
        cmd_log(args)
        return 0
    if args.command == "step-data":
        return cmd_step_data(args)
    return cmd_status(args)


if __name__ == "__main__":
    sys.exit(main())
