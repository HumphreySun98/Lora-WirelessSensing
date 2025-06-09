import argparse, collections, csv, os, sys, time, platform
from datetime import datetime

import numpy as np
import serial
import matplotlib
if platform.system() == "Windows":
    matplotlib.use("TkAgg")        # Windows 默认使用 TkAgg，有图形窗口
else:
    matplotlib.use("TkAgg" if os.environ.get("DISPLAY") else "Agg")
import matplotlib.pyplot as plt

def parse_args():
    ap = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument("-p", "--port", required=True,
                    help="Serial port (e.g. COM4 or /dev/ttyUSB0)")
    ap.add_argument("-b", "--baud", type=int, default=115200,
                    help="Baud rate")
    ap.add_argument("-w", "--window", type=int, default=50,
                    help="Sliding window size (samples)")
    ap.add_argument("--th-var", type=float, default=3.0,
                    help="Variance threshold for motion event")
    ap.add_argument("--th-mean", type=float, default=10.0,
                    help="Mean-shift threshold (dB)")
    ap.add_argument("--plot", action="store_true",
                    help="Enable live plot (requires GUI)")
    ap.add_argument("--csv", type=str, default=None,
                    help="Path to save raw data & events")
    return ap.parse_args()


class WirelessSensing:
    def __init__(self, port: str, baud: int, win: int, plot: bool):
        try:
            self.ser = serial.Serial(port, baud, timeout=0.5)
        except serial.SerialException as e:
            sys.exit(f" Cannot open {port}: {e}")

        self.win = win
        self.dq_ts, self.dq_rssi, self.dq_snr = (
            collections.deque(maxlen=win) for _ in range(3))

        self.plot_enabled = plot and matplotlib.get_backend().lower() != "agg"
        if self.plot_enabled:
            plt.ion()
            self.fig, self.ax = plt.subplots()
            (self.line_rssi,) = self.ax.plot([], [], label="RSSI (dBm)")
            (self.line_snr,) = self.ax.plot([], [], label="SNR  (dB)")
            self.ax.set_xlabel("Samples")
            self.ax.legend()

    def read_one(self):
        raw = self.ser.readline()
        if not raw:
            return None
        try:
            line = raw.decode(errors="ignore").strip()
            if line.count(",") != 2:           
                return None
            ts_ms, rssi, snr = map(float, line.split(","))
            return ts_ms, rssi, snr
        except ValueError:
            return None
    def update(self, sample, th_var: float, th_mean: float):
        ts_ms, rssi, snr = sample
        self.dq_ts.append(ts_ms); self.dq_rssi.append(rssi); self.dq_snr.append(snr)

        if len(self.dq_rssi) < self.win:
            return None

        var_sum = np.var(self.dq_rssi) + np.var(self.dq_snr)
        mean_shift = abs(rssi - np.mean(list(self.dq_rssi)[:-1]))
        if var_sum > th_var or mean_shift > th_mean:
            return dict(type="MOTION", ts=ts_ms,
                        var=round(var_sum, 3),
                        mean_shift=round(mean_shift, 2),
                        rssi=rssi, snr=snr)
        return None

    def refresh_plot(self):
        if not self.plot_enabled:
            return
        x = range(len(self.dq_rssi))
        self.line_rssi.set_data(x, self.dq_rssi)
        self.line_snr.set_data(x, self.dq_snr)
        self.ax.relim(); self.ax.autoscale_view()
        plt.pause(0.001)

    def close(self):
        if self.ser.is_open:
            self.ser.close()
        if self.plot_enabled:
            plt.ioff(); plt.show()

def main():
    args = parse_args()
    sensor = WirelessSensing(args.port, args.baud, args.window, args.plot)

    csv_file = open(args.csv, "w", newline="") if args.csv else None
    csv_writer = csv.writer(csv_file) if csv_file else None
    if csv_writer:
        csv_writer.writerow(["timestamp_ms", "rssi_dbm", "snr_db", "event"])

    print("✅ Listening …  (Ctrl+C to quit)")
    try:
        while True:
            sample = sensor.read_one()
            if sample is None:
                sensor.refresh_plot()
                continue

            ts_ms, rssi, snr = sample
            event = sensor.update(sample, args.th_var, args.th_mean)

            if csv_writer:
                csv_writer.writerow([int(ts_ms), rssi, snr,
                                     event["type"] if event else ""])

            if event:
                now = datetime.now().strftime("%H:%M:%S")
                print(f"[{now}] {event['type']}  RSSI={rssi:.1f} dBm  var={event['var']}")

            sensor.refresh_plot()
    except KeyboardInterrupt:
        print("\n Exit.")
    finally:
        if csv_file:
            csv_file.close()
        sensor.close()


if __name__ == "__main__":
    main()
