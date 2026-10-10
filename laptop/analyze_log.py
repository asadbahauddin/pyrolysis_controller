#!/usr/bin/env python3
"""
analyze_log.py - analisis offline satu "run" dari CSV log suhu (hasil
logger.py, atau CSV generik: timestamp, suhu, setpoint, fan, mode).

Satu file CSV dari logger.py biasanya berisi BANYAK segmen (idle/step/
relay/method bercampur dalam satu file, karena logger jalan terus selama
sesi). Skrip ini mengelompokkan baris jadi segmen kontinu per mode, lalu
menganalisis satu segmen (default: yang TERAKHIR / Uji Metode paling baru).

Cara pakai:
    python analyze_log.py data/log_XXXX.csv                # analisis segmen mode=method terakhir
    python analyze_log.py data/log_XXXX.csv --list         # daftar semua segmen dalam file
    python analyze_log.py data/log_XXXX.csv --run 1        # segmen ke-2 terhitung dari belakang
    python analyze_log.py data/log_XXXX.csv --band 0.02    # settling time band +-2% (default 0.05 = +-5%)
    python analyze_log.py data/log_XXXX.csv --out hasil.png

Definisi metrik:
  SP            = setpoint segmen (nilai sp yang paling sering muncul)
  T0            = suhu awal = rata-rata 10 sampel PERTAMA
  Tpeak         = suhu maksimum (raw) selama segmen
  Overshoot     = (Tpeak - SP) / SP x 100% (0% kalau Tpeak <= SP)
  Rise Time     = waktu dari 10% ke 90% dari (SP - T0)
  Settling Time = waktu PERTAMA KALI suhu (versi dihaluskan rolling average,
                  default 10 sampel) MASUK dan TETAP dalam band +-5% SP.
                  Pakai versi halus supaya noise sensor MAX6675 sesaat
                  (lonjakan 1-2 sampel) tidak dihitung sebagai "belum
                  settle" -- itu noise, bukan osilasi kontrol beneran.
  Tss           = rata-rata suhu (raw) dari titik settled sampai akhir data
                  (fallback: rata-rata 10% sampel terakhir, kalau belum
                  pernah settle sampai akhir segmen)
  SS Error      = |SP - Tss|

Grafik (PNG) menampilkan suhu mentah (abu-abu tipis transparan) dan versi
dihaluskan rolling average (merah tebal), plus pita toleransi +-5% SP
(hijau transparan). Overshoot/Rise Time dihitung dari data MENTAH; Settling
Time/Tss dihitung dari versi halus (lihat di atas). Skrip juga menyimpan
PNG kedua berisi tabel ringkasan hasil ("<csv>_hasil_pid.png").

Butuh: pandas, matplotlib (pip install pandas matplotlib).
"""
import argparse
import sys

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")  # aman dipakai tanpa layar; skrip ini cuma menyimpan PNG
import matplotlib.pyplot as plt

# Nama kolom yang dicoba (urutan prioritas): nama asli logger.py dulu, baru
# nama generik sesuai spesifikasi (timestamp/suhu/setpoint/fan/mode).
COLUMN_ALIASES = {
    "time": ["pc_time", "timestamp", "time", "waktu"],
    "temp": ["temp_c", "suhu", "temperature", "temp"],
    "sp":   ["sp", "setpoint", "target"],
    "fan":  ["fan_pct", "fan", "pwm"],
    "mode": ["mode", "status"],
}


def find_col(df, key):
    for name in COLUMN_ALIASES[key]:
        if name in df.columns:
            return name
    return None


def load_csv(path):
    df = pd.read_csv(path)
    cols = {}
    for key in ("time", "temp", "sp", "fan", "mode"):
        c = find_col(df, key)
        if c is None and key in ("time", "temp", "sp"):
            raise SystemExit(
                "Kolom %r tidak ditemukan di %s (dicoba nama: %s). Kolom yang ada: %s"
                % (key, path, COLUMN_ALIASES[key], list(df.columns)))
        cols[key] = c

    out = pd.DataFrame()
    out["temp"] = pd.to_numeric(df[cols["temp"]], errors="coerce")
    out["sp"] = pd.to_numeric(df[cols["sp"]], errors="coerce")
    out["fan"] = pd.to_numeric(df[cols["fan"]], errors="coerce") if cols["fan"] else float("nan")
    out["mode"] = df[cols["mode"]].astype(str) if cols["mode"] else "unknown"

    # Waktu -> detik sejak baris pertama FILE. Coba parse sbg timestamp dulu
    # (format pc_time logger.py), baru fallback ke angka mentah (mis. sudah
    # berupa detik, atau esp_ms dalam milidetik akan salah skala -- itu
    # tanggung jawab pengguna kalau pakai kolom waktu non-standar).
    raw_t = df[cols["time"]]
    try:
        t = pd.to_datetime(raw_t)
        out["t"] = (t - t.iloc[0]).dt.total_seconds()
    except (ValueError, TypeError):
        out["t"] = pd.to_numeric(raw_t, errors="coerce")
    return out


def find_segments(df, mode_filter):
    """Kelompokkan indeks baris jadi segmen kontinu dengan mode == mode_filter."""
    target = mode_filter.strip().lower()
    segs = []
    cur_start = None
    prev_i = None
    for i, mode_val in zip(df.index, df["mode"]):
        is_target = (str(mode_val).strip().lower() == target)
        if is_target:
            if cur_start is None:
                cur_start = i
            prev_i = i
        elif cur_start is not None:
            segs.append((cur_start, prev_i))
            cur_start = None
    if cur_start is not None:
        segs.append((cur_start, prev_i))
    return segs


def compute_metrics(seg_df, band_frac, smooth_window=10):
    seg_df = seg_df.reset_index(drop=True)
    t = (seg_df["t"] - seg_df["t"].iloc[0]).to_numpy()
    temp = seg_df["temp"].to_numpy()
    temp_smooth = pd.Series(temp).rolling(smooth_window, min_periods=1, center=True).mean().to_numpy()

    sp_counts = seg_df["sp"].dropna()
    if sp_counts.empty:
        raise SystemExit("Segmen ini tidak punya nilai setpoint (sp) yang valid -- cek --mode/--run.")
    sp = float(sp_counts.mode().iloc[0])

    n_head = max(1, min(10, len(temp)))
    T0 = float(pd.Series(temp[:n_head]).mean())
    span = sp - T0

    rising = span > 0
    Tpeak = float(np.nanmax(temp) if rising else np.nanmin(temp))
    overshoot_sp = max(0.0, (Tpeak - sp) / sp * 100.0) if (rising and sp != 0) else \
        (max(0.0, (sp - Tpeak) / sp * 100.0) if sp != 0 else float("nan"))
    overshoot_span = max(0.0, (Tpeak - sp) / span * 100.0) if rising else \
        max(0.0, (sp - Tpeak) / abs(span) * 100.0) if span != 0 else float("nan")

    thr10 = T0 + 0.10 * span
    thr90 = T0 + 0.90 * span
    if rising:
        i10 = next((i for i, v in enumerate(temp) if v >= thr10), None)
        i90 = next((i for i, v in enumerate(temp) if v >= thr90), None)
    else:
        i10 = next((i for i, v in enumerate(temp) if v <= thr10), None)
        i90 = next((i for i, v in enumerate(temp) if v <= thr90), None)
    rise_time = (t[i90] - t[i10]) if (i10 is not None and i90 is not None) else float("nan")

    band = band_frac * abs(sp)
    outside = [i for i, v in enumerate(temp_smooth) if abs(v - sp) > band]
    never_settled = bool(outside) and outside[-1] + 1 >= len(t)
    if not outside:
        settle_idx = 0              # sudah di dalam band sejak awal segmen
    elif outside[-1] + 1 < len(t):
        settle_idx = outside[-1] + 1
    else:
        settle_idx = len(t) - 1     # tidak pernah settle sampai segmen berakhir
    settling_time = t[settle_idx]

    if never_settled:
        n_tail = max(1, int(round(0.1 * len(temp))))
        Tss = float(pd.Series(temp[-n_tail:]).mean())
    else:
        Tss = float(pd.Series(temp[settle_idx:]).mean())
    sse = abs(sp - Tss)

    return {
        "SP": sp, "T0": T0, "Tss": Tss, "Tpeak": Tpeak,
        "overshoot_sp": overshoot_sp, "overshoot_span": overshoot_span,
        "rise_time": rise_time, "settling_time": settling_time,
        "settle_idx": settle_idx, "never_settled": never_settled,
        "sse": sse, "band_pct": band_frac * 100, "band": band,
        "i10": i10, "i90": i90,
        "t": t, "temp": temp, "temp_smooth": temp_smooth, "rising": rising,
    }


def plot_result(m, out_path, smooth_window):
    t, temp, temp_smooth = m["t"], m["temp"], m["temp_smooth"]
    sp, band = m["SP"], m["band"]

    fig, ax = plt.subplots(figsize=(10, 6))
    ax.axhspan(sp - band, sp + band, color="green", alpha=0.12,
               label="Toleransi +-%.0f%% SP" % m["band_pct"], zorder=0)
    ax.plot(t, temp, color="gray", alpha=0.4, linewidth=0.8, label="Suhu mentah")
    ax.plot(t, temp_smooth, color="red", linewidth=1.8,
            label="Suhu halus (rolling-%d)" % smooth_window)
    ax.axhline(sp, color="green", linestyle="--", linewidth=1.5, label="Setpoint")

    peak_idx = int(np.nanargmax(temp) if m["rising"] else np.nanargmin(temp))
    ax.plot(t[peak_idx], temp[peak_idx], "r^", markersize=10, zorder=5)
    ax.annotate("Overshoot %.1f%%" % m["overshoot_sp"],
                xy=(t[peak_idx], temp[peak_idx]), xytext=(15, -18), textcoords="offset points",
                fontsize=9, color="red", arrowprops=dict(arrowstyle="->", color="red"),
                bbox=dict(boxstyle="round", facecolor="white", edgecolor="red", alpha=0.85))

    if m["i10"] is not None and m["i90"] is not None:
        ax.plot(t[m["i10"]], temp[m["i10"]], "ko", markersize=5, label="Rise time start/end")
        ax.plot(t[m["i90"]], temp[m["i90"]], "ko", markersize=5)
        ax.annotate("Rise time = %.1fs" % m["rise_time"],
                    xy=(t[m["i90"]], temp[m["i90"]]), xytext=(10, -22), textcoords="offset points",
                    fontsize=9, arrowprops=dict(arrowstyle="->"))

    settle_label = "Settling = %.1fs\n(band +-%.0f%%)" % (m["settling_time"], m["band_pct"])
    if m["never_settled"]:
        settle_label += "\n(!) belum settle"
    ax.axvline(m["settling_time"], color="purple", linestyle=":", linewidth=1.5,
               label="Settling time")
    ax.annotate(settle_label, xy=(m["settling_time"], 1), xycoords=("data", "axes fraction"),
                xytext=(6, -6), textcoords="offset points", fontsize=8, color="purple",
                va="top", ha="left", bbox=dict(boxstyle="round", facecolor="white",
                                                edgecolor="purple", alpha=0.85))

    ax.set_xlabel("Waktu (s)")
    ax.set_ylabel("Suhu (C)")
    ax.set_title("Respon Sistem PID Pyrolysis — SP=%.1f°C" % sp)
    # Legend DI LUAR area plot (di bawah) -- data & anotasi bentuknya beda-beda
    # tiap run, jadi tidak ada satu sudut pun di dalam axes yang aman dari tabrakan.
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.14), ncol=3, fontsize=8)
    ax.grid(True, alpha=0.3)
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print("Grafik tersimpan: %s" % out_path)


def print_summary(m):
    rows = [
        ("Setpoint", "%.1f C" % m["SP"]),
        ("Suhu Awal (T0)", "%.1f C" % m["T0"]),
        ("Suhu Puncak", "%.1f C" % m["Tpeak"]),
        ("Rise Time", "%.0f s" % m["rise_time"]),
        ("Overshoot", "%.1f %%" % m["overshoot_sp"]),
        ("Settling Time", "%.0f s%s" % (m["settling_time"], " (!)" if m["never_settled"] else "")),
        ("Steady State", "%.1f C" % m["Tss"]),
        ("SS Error", "%.1f C" % m["sse"]),
    ]
    label_w = max(len(r[0]) for r in rows)
    val_w = max(len(r[1]) for r in rows)
    inner_w = max(label_w + val_w + 3, len("HASIL ANALISIS SISTEM PID"))
    title = "HASIL ANALISIS SISTEM PID"

    print()
    print("┌" + "─" * (inner_w + 2) + "┐")
    print("│ %s │" % title.center(inner_w))
    print("├" + "─" * (inner_w + 2) + "┤")
    for label, val in rows:
        line = "%s : %s" % (label.ljust(label_w), val.rjust(val_w))
        print("│ %s │" % line.ljust(inner_w))
    print("└" + "─" * (inner_w + 2) + "┘")
    if m["never_settled"]:
        print("(!) belum settle sampai akhir segmen -- Steady State dihitung dari 10%% sampel terakhir")
    print("Overshoot (vs rentang SP-T0, definisi klasik buku teks): %.1f %%" % m["overshoot_span"])


def plot_table(m, out_path):
    rows = [
        ("Setpoint", "%.1f °C" % m["SP"]),
        ("Suhu Awal T0", "%.1f °C" % m["T0"]),
        ("Suhu Puncak", "%.1f °C" % m["Tpeak"]),
        ("Rise Time", "%.0f s" % m["rise_time"]),
        ("Overshoot", "%.1f %%" % m["overshoot_sp"]),
        ("Settling Time", "%.0f s" % m["settling_time"]),
        ("Steady State", "%.1f °C" % m["Tss"]),
        ("SS Error", "%.1f °C" % m["sse"]),
    ]

    fig, ax = plt.subplots(figsize=(6, 4))
    ax.axis("off")
    tabel = ax.table(cellText=rows, colLabels=["Parameter", "Nilai"],
                      loc="center", cellLoc="left")
    tabel.auto_set_font_size(False)
    tabel.set_fontsize(13)
    tabel.scale(1.5, 2.2)

    for (row, col), cell in tabel.get_celld().items():
        cell.set_edgecolor("#999999")
        if row == 0:
            cell.set_facecolor("#1f4e79")
            cell.set_text_props(color="white", fontweight="bold")
        else:
            cell.set_facecolor("#dce6f1" if row % 2 == 0 else "white")

    plt.title("HASIL ANALISIS SISTEM PID PYROLYSIS", fontsize=14, fontweight="bold", pad=20)
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print("Tabel tersimpan: %s" % out_path)


def plot_fan(t, fan, sp, out_path):
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(t, fan, color="darkorange", linewidth=1.3, label="Fan (%)")
    ax.fill_between(t, fan, color="darkorange", alpha=0.15)
    ax.set_xlabel("Waktu (s)")
    ax.set_ylabel("Fan (%)")
    ax.set_ylim(-5, 105)
    ax.set_title("Grafik Fan — SP=%.1f°C" % sp)
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right", fontsize=8)
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print("Grafik fan tersimpan: %s" % out_path)


def main():
    p = argparse.ArgumentParser(description="Analisis offline CSV log suhu (rise/settling time, overshoot)")
    p.add_argument("csv", help="path file CSV (hasil logger.py atau CSV generik timestamp/suhu/setpoint/fan/mode)")
    p.add_argument("--mode", default="method", help="nilai kolom mode utk satu 'run' (default: method)")
    p.add_argument("--run", type=int, default=0, help="pilih segmen dari belakang: 0=terakhir, 1=sblm itu, dst")
    p.add_argument("--list", action="store_true", help="tampilkan semua segmen yang ditemukan lalu keluar")
    p.add_argument("--band", type=float, default=0.05, help="band settling time, pecahan dari SP (default 0.05 = +-5%%)")
    p.add_argument("--smooth", type=int, default=10, help="jumlah sampel rolling average utk grafik+settling (default 10)")
    p.add_argument("--out", help="path PNG grafik keluaran (default: <nama csv>_analysis.png)")
    p.add_argument("--out-table", help="path PNG tabel hasil (default: <nama csv>_hasil_pid.png)")
    p.add_argument("--out-fan", help="path PNG grafik fan (default: <nama csv>_fan.png)")
    args = p.parse_args()

    df = load_csv(args.csv)
    segs = find_segments(df, args.mode)
    if not segs:
        avail = sorted(df["mode"].astype(str).str.strip().str.lower().unique().tolist())
        raise SystemExit("Tidak ada segmen dengan mode=%r di %s.\nNilai mode yang ada di file: %s"
                          % (args.mode, args.csv, avail))

    if args.list:
        print("Segmen mode=%r ditemukan di %s:" % (args.mode, args.csv))
        for disp_i, (a, b) in enumerate(reversed(segs)):
            seg_df = df.loc[a:b]
            dur = seg_df["t"].iloc[-1] - seg_df["t"].iloc[0]
            sp_m = seg_df["sp"].dropna().mode()
            sp_val = float(sp_m.iloc[0]) if not sp_m.empty else float("nan")
            print("  --run %-2d  baris %d-%d, durasi=%.0fs, sp=%.1fC, n=%d sampel"
                  % (disp_i, a, b, dur, sp_val, b - a + 1))
        return 0

    idx = len(segs) - 1 - args.run
    if not (0 <= idx < len(segs)):
        raise SystemExit("--run %d di luar jangkauan (ada %d segmen; pakai --list utk lihat pilihannya)"
                          % (args.run, len(segs)))
    a, b = segs[idx]
    seg_df = df.loc[a:b]
    print("Menganalisis segmen baris %d-%d (%d sampel) dari %s ..." % (a, b, b - a + 1, args.csv))

    m = compute_metrics(seg_df, band_frac=args.band, smooth_window=args.smooth)
    print_summary(m)

    out_path = args.out or (args.csv.rsplit(".", 1)[0] + "_analysis.png")
    plot_result(m, out_path, smooth_window=args.smooth)

    out_table_path = args.out_table or (args.csv.rsplit(".", 1)[0] + "_hasil_pid.png")
    plot_table(m, out_table_path)

    fan = seg_df.reset_index(drop=True)["fan"].to_numpy()
    out_fan_path = args.out_fan or (args.csv.rsplit(".", 1)[0] + "_fan.png")
    plot_fan(m["t"], fan, m["SP"], out_fan_path)
    return 0


if __name__ == "__main__":
    sys.exit(main())
