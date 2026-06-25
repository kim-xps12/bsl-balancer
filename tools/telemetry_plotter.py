#!/usr/bin/env python3
"""Record and plot telemetry from bsl-balancer for PID tuning."""

import argparse
import csv
import sys
import time
from datetime import datetime
from pathlib import Path

import serial
import serial.tools.list_ports
import matplotlib.pyplot as plt
import numpy as np

COLUMNS = [
    "t_ms", "dt_us", "pitch", "pitch_err", "vel_offset",
    "theta_dot", "vel_L", "vel_R", "v_fwd",
    "i_unsat", "i_sat", "z_theta", "vel_int",
]


def find_port():
    ports = serial.tools.list_ports.comports()
    for p in ports:
        desc = (p.description or "").lower()
        if any(k in p.device.lower() for k in ("usbserial", "usbmodem", "wchusbserial")):
            return p.device
        if any(k in desc for k in ("cp210", "ch340", "ftdi", "slab")):
            return p.device
    if ports:
        return ports[0].device
    return None


def record(port, baudrate, duration, output):
    print(f"Connecting to {port} at {baudrate} baud...")
    ser = serial.Serial(port, baudrate, timeout=1)
    time.sleep(2)
    ser.reset_input_buffer()

    rows = []
    meta_lines = []
    t_start = time.monotonic()
    print(f"Recording for {duration}s... (Ctrl+C to stop early)")

    try:
        while time.monotonic() - t_start < duration:
            line = ser.readline().decode("utf-8", errors="replace").strip()
            if not line:
                continue
            if line.startswith("#"):
                meta_lines.append(line)
                print(line)
                continue
            if not line.startswith("T,"):
                continue
            parts = line[2:].split(",")
            if len(parts) != len(COLUMNS):
                continue
            try:
                row = [float(v) for v in parts]
            except ValueError:
                continue
            rows.append(row)
            elapsed = time.monotonic() - t_start
            sys.stdout.write(f"\r  {len(rows)} samples | {elapsed:.1f}s / {duration}s")
            sys.stdout.flush()
    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        ser.close()

    print(f"\n{len(rows)} samples recorded.")
    if not rows:
        print("No telemetry data received. Check firmware and serial connection.")
        return None

    with open(output, "w", newline="") as f:
        for m in meta_lines:
            f.write(m + "\n")
        writer = csv.writer(f)
        writer.writerow(COLUMNS)
        writer.writerows(rows)
    print(f"Saved to {output}")
    return output


def load_csv(path):
    rows = []
    with open(path) as f:
        for line in f:
            if line.startswith("#"):
                print(line.strip())
                continue
            break
        reader = csv.DictReader(f, fieldnames=COLUMNS if line.strip().startswith("t_ms") else None)
        if not line.strip().startswith("t_ms"):
            f.seek(0)
            for skip in f:
                if skip.startswith("#"):
                    continue
                break
            reader = csv.DictReader(f)
        for r in reader:
            try:
                rows.append({k: float(v) for k, v in r.items()})
            except (ValueError, TypeError):
                continue
    return rows


def plot_telemetry(csv_path):
    with open(csv_path) as f:
        meta_lines = []
        header_line = None
        for line in f:
            stripped = line.strip()
            if stripped.startswith("#"):
                meta_lines.append(stripped)
            elif stripped:
                header_line = stripped
                break
        reader = csv.DictReader(f, fieldnames=header_line.split(","))
        rows = []
        for r in reader:
            try:
                rows.append({k: float(v) for k, v in r.items()})
            except (ValueError, TypeError):
                continue

    if not rows:
        print("No data to plot.")
        return

    gains_str = ""
    for m in meta_lines:
        if m.startswith("# ") and "," in m:
            gains_str = m[2:]
        print(m)

    t = np.array([(r["t_ms"] - rows[0]["t_ms"]) / 1000.0 for r in rows])
    get = lambda k: np.array([r[k] for r in rows])

    fig, axes = plt.subplots(5, 1, figsize=(14, 12), sharex=True)
    fig.suptitle(f"bsl-balancer Telemetry   gains=[{gains_str}]", fontsize=11)

    # 1: Pitch angle
    ax = axes[0]
    pitch = get("pitch")
    pitch_err = get("pitch_err")
    target = pitch + pitch_err
    ax.plot(t, pitch, label="pitch (measured)", linewidth=0.8)
    ax.plot(t, target, "--", label="pitch target", linewidth=0.8, alpha=0.7)
    ax.set_ylabel("Pitch [deg]")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.3)

    # 2: Pitch error + velocity offset
    ax = axes[1]
    ax.plot(t, pitch_err, label="pitch error", linewidth=0.8)
    ax.plot(t, get("vel_offset"), label="vel PI offset", linewidth=0.8)
    ax.set_ylabel("Angle [deg]")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.3)
    ax.axhline(0, color="gray", linewidth=0.5)

    # 3: Forward velocity
    ax = axes[2]
    ax.plot(t, get("v_fwd"), label="v_fwd", linewidth=0.8)
    ax.plot(t, get("vel_L"), label="vel_L", linewidth=0.5, alpha=0.5)
    ax.plot(t, get("vel_R"), label="vel_R", linewidth=0.5, alpha=0.5)
    ax.set_ylabel("Velocity [RPM]")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.3)
    ax.axhline(0, color="gray", linewidth=0.5)

    # 4: Current command
    ax = axes[3]
    ax.plot(t, get("i_unsat"), label="i_unsat", linewidth=0.8)
    ax.plot(t, get("i_sat"), label="i_sat", linewidth=0.8)
    ax.set_ylabel("Current [A]")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.3)
    ax.axhline(0, color="gray", linewidth=0.5)

    # 5: Integral states
    ax = axes[4]
    ax.plot(t, get("z_theta"), label="z_theta (inner)", linewidth=0.8)
    ax.plot(t, get("vel_int"), label="vel_int (outer)", linewidth=0.8)
    ax.set_ylabel("Integral")
    ax.set_xlabel("Time [s]")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.3)
    ax.axhline(0, color="gray", linewidth=0.5)

    plt.tight_layout()

    plot_path = Path(csv_path).with_suffix(".png")
    plt.savefig(plot_path, dpi=150)
    print(f"Plot saved to {plot_path}")
    plt.show()


def print_summary(csv_path):
    with open(csv_path) as f:
        rows = []
        for line in f:
            if line.startswith("#") or line.startswith("t_ms"):
                continue
            parts = line.strip().split(",")
            if len(parts) == len(COLUMNS):
                try:
                    rows.append({k: float(v) for k, v in zip(COLUMNS, parts)})
                except ValueError:
                    continue

    if not rows:
        return

    t_span = (rows[-1]["t_ms"] - rows[0]["t_ms"]) / 1000.0
    err = np.array([r["pitch_err"] for r in rows])
    v_fwd = np.array([r["v_fwd"] for r in rows])
    i_sat = np.array([r["i_sat"] for r in rows])
    i_unsat = np.array([r["i_unsat"] for r in rows])
    dt_us = np.array([r["dt_us"] for r in rows])

    sat_pct = np.mean(np.abs(i_unsat) > np.abs(i_sat) + 0.001) * 100

    print(f"\n{'='*50}")
    print(f" Recording: {t_span:.1f}s, {len(rows)} samples")
    print(f"{'='*50}")
    print(f" Pitch error  : mean={np.mean(err):+.3f} std={np.std(err):.3f} max={np.max(np.abs(err)):.3f} deg")
    print(f" Fwd velocity : mean={np.mean(v_fwd):+.2f} std={np.std(v_fwd):.2f} max={np.max(np.abs(v_fwd)):.1f} RPM")
    print(f" Current cmd  : mean={np.mean(np.abs(i_sat)):.4f} max={np.max(np.abs(i_sat)):.4f} A")
    print(f" Saturation   : {sat_pct:.1f}% of samples")
    print(f" Loop dt      : mean={np.mean(dt_us):.0f} std={np.std(dt_us):.0f} max={np.max(dt_us):.0f} us")
    print(f"{'='*50}")

    if np.std(err) > 3.0:
        print(" [!] High pitch oscillation - consider increasing Kd or reducing Kp")
    if sat_pct > 20:
        print(" [!] Frequent saturation - consider reducing gains or raising current limit")
    if np.abs(np.mean(v_fwd)) > 10:
        print(" [!] Significant drift - check outer velocity PI (Kp_vel, Ki_vel)")
    if np.max(dt_us) > 15000:
        print(" [!] Loop timing jitter detected - check for blocking operations")


def main():
    parser = argparse.ArgumentParser(description="bsl-balancer telemetry recorder & plotter")
    sub = parser.add_subparsers(dest="cmd")

    p_rec = sub.add_parser("record", help="Record telemetry to CSV")
    p_rec.add_argument("-p", "--port", help="Serial port (auto-detect if omitted)")
    p_rec.add_argument("-b", "--baud", type=int, default=115200)
    p_rec.add_argument("-d", "--duration", type=float, default=30, help="Recording duration in seconds")
    p_rec.add_argument("-o", "--output", help="Output CSV path")

    p_plot = sub.add_parser("plot", help="Plot from CSV file")
    p_plot.add_argument("csv", help="CSV file to plot")

    p_run = sub.add_parser("run", help="Record then plot (default)")
    p_run.add_argument("-p", "--port", help="Serial port")
    p_run.add_argument("-b", "--baud", type=int, default=115200)
    p_run.add_argument("-d", "--duration", type=float, default=30)
    p_run.add_argument("-o", "--output", help="Output CSV path")

    args = parser.parse_args()

    if args.cmd is None:
        args.cmd = "run"
        args.port = None
        args.baud = 115200
        args.duration = 30
        args.output = None

    if args.cmd in ("record", "run"):
        port = args.port or find_port()
        if not port:
            print("No serial port found. Specify with -p.")
            sys.exit(1)
        output = args.output or f"telemetry_{datetime.now():%Y%m%d_%H%M%S}.csv"
        csv_path = record(port, args.baud, args.duration, output)
        if csv_path and args.cmd == "run":
            print_summary(csv_path)
            plot_telemetry(csv_path)
        elif csv_path:
            print_summary(csv_path)
    elif args.cmd == "plot":
        print_summary(args.csv)
        plot_telemetry(args.csv)


if __name__ == "__main__":
    main()
