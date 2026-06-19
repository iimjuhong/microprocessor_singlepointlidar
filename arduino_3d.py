from __future__ import annotations
import argparse
import csv
import math
import os
import queue
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", str((Path.cwd() / ".matplotlib-cache").resolve()))
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import serial
from serial.tools import list_ports


@dataclass
class ScanPoint:
    yaw_deg: float
    pitch_deg: float
    distance_cm: float
    strength: int


@dataclass(frozen=True)
class FilterConfig:
    center_offset_cm: float
    min_distance_cm: float
    max_distance_cm: float
    min_strength: int
    invert_yaw: bool
    time_offset_ms: float


class YawKalmanFilter:
    def __init__(self, initial_yaw=0.0):
        self.x = np.array([[initial_yaw], [0.0]])
        self.P = np.array([[1.0, 0.0], [0.0, 1.0]])
        self.Q = np.array([[0.01, 0.0], [0.0, 0.5]])
        self.R = np.array([[5.0]])
        self.last_time = None

    def update(self, time_s: float, meas_yaw: float) -> float:
        if self.last_time is None:
            self.last_time = time_s
            self.x[0, 0] = meas_yaw
            return meas_yaw
        dt = time_s - self.last_time
        if dt <= 0:
            return self.x[0, 0] % 360.0
        self.last_time = time_s
        diff = meas_yaw - (self.x[0, 0] % 360.0)
        if diff > 180.0:
            diff -= 360.0
        elif diff < -180.0:
            diff += 360.0
        meas_yaw_unwrapped = self.x[0, 0] + diff
        F = np.array([[1.0, dt], [0.0, 1.0]])
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + self.Q
        H = np.array([[1.0, 0.0]])
        y = meas_yaw_unwrapped - (H @ self.x)[0, 0]
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K * y
        self.P = (np.eye(2) - K @ H) @ self.P
        return self.x[0, 0] % 360.0


class LidarClient:
    def __init__(self, port: str, baud: int = 115200, time_offset_ms: float = 0.0):
        self.port = port
        self.baud = baud
        self.time_offset_ms = time_offset_ms
        self.serial = serial.Serial(port, baud, timeout=0.2)
        self.points: list[ScanPoint] = []
        self.messages: queue.Queue[str] = queue.Queue()
        self._lock = threading.Lock()
        self._running = True
        self._thread = threading.Thread(target=self._reader, daemon=True)
        self._thread.start()

    def _reader(self) -> None:
        kf = YawKalmanFilter()
        while self._running:
            try:
                line = self.serial.readline().decode("ascii", errors="ignore").strip()
            except serial.SerialException as exc:
                self.messages.put(f"SERIAL_ERROR,{exc}")
                break
            if not line:
                continue
            fields = line.split(",")
            if fields[0] == "SYNC" and len(fields) >= 6:
                try:
                    time_ms = float(fields[1]) + self.time_offset_ms
                    enc_cnt = int(fields[2])
                    step_pos = int(fields[3])
                    distance_cm = float(fields[4])
                    strength = int(fields[5])
                    raw_yaw_deg = (enc_cnt % 1320) / 1320.0 * 360.0
                    raw_pitch_deg = -step_pos * 360.0 / 4096.0
                    est_yaw_deg = kf.update(time_ms / 1000.0, raw_yaw_deg)
                    point = ScanPoint(
                        yaw_deg=est_yaw_deg,
                        pitch_deg=raw_pitch_deg,
                        distance_cm=distance_cm,
                        strength=strength,
                    )
                except ValueError:
                    self.messages.put(f"BAD_LINE,{line}")
                    continue
                with self._lock:
                    self.points.append(point)
            elif fields[0] == "SCAN" and len(fields) >= 5:
                try:
                    point = ScanPoint(
                        yaw_deg=float(fields[1]),
                        pitch_deg=float(fields[2]),
                        distance_cm=float(fields[3]),
                        strength=int(float(fields[4])),
                    )
                except ValueError:
                    self.messages.put(f"BAD_LINE,{line}")
                    continue
                with self._lock:
                    self.points.append(point)
            else:
                self.messages.put(line)

    def send(self, command: str) -> None:
        self.serial.write(command.encode("ascii"))

    def snapshot(self) -> list[ScanPoint]:
        with self._lock:
            return list(self.points)

    def clear(self) -> None:
        with self._lock:
            self.points.clear()

    def close(self) -> None:
        self._running = False
        self._thread.join(timeout=1.0)
        if self.serial.is_open:
            self.serial.close()


class SimulatedClient:
    def __init__(self):
        self.points: list[ScanPoint] = []
        self.messages: queue.Queue[str] = queue.Queue()
        self._lock = threading.Lock()
        self._running = True
        self._scanning = False
        self._thread = threading.Thread(target=self._reader, daemon=True)
        self._thread.start()

    def _reader(self) -> None:
        yaw = 0.0
        pitch = 0.0
        while self._running:
            with self._lock:
                scanning = self._scanning
            if scanning:
                radius = 22.0 + 3.0 * math.sin(math.radians(yaw * 3.0))
                radius += 2.0 * math.cos(math.radians(pitch * 5.0))
                with self._lock:
                    self.points.append(ScanPoint(yaw, pitch, radius, 1000))
                yaw += 4.0
                if yaw >= 360.0:
                    yaw -= 360.0
                    pitch += 4.0
                    if pitch > 44.0:
                        with self._lock:
                            self._scanning = False
                        self.messages.put("STATUS,DONE")
                time.sleep(0.025)
            else:
                time.sleep(0.05)

    def send(self, command: str) -> None:
        if command == "s":
            with self._lock:
                self._scanning = True
            self.messages.put("STATUS,START")
        elif command == "h":
            with self._lock:
                self._scanning = False
            self.messages.put("STATUS,HALT")
        elif command == "c":
            self.clear()
            self.messages.put("STATUS,CLEAR")

    def snapshot(self) -> list[ScanPoint]:
        with self._lock:
            return list(self.points)

    def clear(self) -> None:
        with self._lock:
            self.points.clear()

    def close(self) -> None:
        self._running = False
        self._thread.join(timeout=1.0)


def point_to_xyz(
    point: ScanPoint,
    config: FilterConfig,
) -> tuple[float, float, float, int] | None:
    r = point.distance_cm
    if r < config.min_distance_cm or r > config.max_distance_cm:
        return None
    if point.strength < config.min_strength:
        return None
    yaw_sign = 1.0 if config.invert_yaw else -1.0
    yaw = math.radians(point.yaw_deg) * yaw_sign
    pitch = math.radians(point.pitch_deg)
    r_horizontal = r * math.cos(pitch)
    if config.center_offset_cm != 0:
        actual_radius = config.center_offset_cm - r_horizontal
        x = actual_radius * math.sin(yaw)
        y = actual_radius * math.cos(yaw)
    else:
        x = r_horizontal * math.sin(yaw)
        y = r_horizontal * math.cos(yaw)
    z = r * math.sin(pitch)
    return x, y, z, point.strength


def polar_to_xyz(
    points: list[ScanPoint],
    config: FilterConfig,
) -> np.ndarray:
    xyz = []
    for point in points:
        row = point_to_xyz(point, config)
        if row is not None:
            xyz.append(row)
    if not xyz:
        return np.empty((0, 4))
    return np.array(xyz, dtype=float)


def save_csv(
    path: Path,
    points: list[ScanPoint],
    config: FilterConfig,
) -> int:
    path.parent.mkdir(parents=True, exist_ok=True)
    saved_count = 0
    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.writer(file)
        writer.writerow(
            [
                "yaw_deg",
                "pitch_deg",
                "distance_cm",
                "strength",
                "x_cm",
                "y_cm",
                "z_cm",
            ]
        )
        for point in points:
            row = point_to_xyz(point, config)
            if row is None:
                continue
            writer.writerow(
                [
                    point.yaw_deg,
                    point.pitch_deg,
                    point.distance_cm,
                    point.strength,
                    row[0],
                    row[1],
                    row[2],
                ]
            )
            saved_count += 1
    return saved_count


def choose_port() -> str:
    ports = list(list_ports.comports())
    if not ports:
        raise SystemExit(
            "No serial ports found. Use --simulate to test without Arduino."
        )
    print("Available serial ports:")
    for index, port in enumerate(ports, start=1):
        print(f"{index}. {port.device}  {port.description}")
    if len(ports) == 1:
        print(f"Using {ports[0].device}")
        return ports[0].device
    while True:
        choice = input("Select port number (Enter = 1): ").strip()
        if not choice:
            return ports[0].device
        try:
            index = int(choice)
        except ValueError:
            print("Enter a number from the list.")
            continue
        if 1 <= index <= len(ports):
            return ports[index - 1].device
        print("That port number is not in the list.")


def color_values(xyz: np.ndarray, color_by: str) -> np.ndarray:
    if color_by == "z":
        return xyz[:, 2]
    if color_by == "distance":
        return np.linalg.norm(xyz[:, :3], axis=1)
    return xyz[:, 3]


def input_worker(client, stop_event: threading.Event) -> None:
    print("Commands: s=start, h=halt, c=clear, q=quit and save")
    print(
        "Debug: l/r/x=DC motor, u/d=pitch stepper, p=ULN LED test, v=LiDAR-only points, z=pitch zero, t=diagnostics"
    )
    while not stop_event.is_set():
        command = sys.stdin.readline().strip().lower()
        if not command:
            continue
        if command[0] == "q":
            print("Returning stepper to origin before quitting...")
            client.send("z")
            time.sleep(3.0)
            stop_event.set()
            return
        if command[0] in {"s", "h", "c", "l", "r", "x", "u", "d", "p", "v", "z", "t"}:
            client.send(command[0])
            if command[0] == "c":
                client.clear()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--port", default=None, help="Arduino serial port, for example COM5"
    )
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument(
        "--center-offset",
        type=float,
        default=48.0,
        help="회전 중심에서 센서까지의 거리(cm)",
    )
    parser.add_argument(
        "--min-distance", type=float, default=10.0, help="TFmini 사각지대 0~10cm 제거"
    )
    parser.add_argument("--max-distance", type=float, default=120.0)
    parser.add_argument("--min-strength", type=int, default=0)
    parser.add_argument(
        "--time-offset",
        type=float,
        default=0.0,
        help="동기화 타이밍 캘리브레이션 오프셋 (ms 단위)",
    )
    parser.add_argument("--output", default="scan_points.csv")
    parser.add_argument("--invert-yaw", action="store_true")
    parser.add_argument(
        "--color-by",
        choices=["z", "strength", "distance"],
        default="z",
        help="Point cloud color 기준",
    )
    parser.add_argument("--simulate", action="store_true")
    args = parser.parse_args()
    config = FilterConfig(
        center_offset_cm=args.center_offset,
        min_distance_cm=args.min_distance,
        max_distance_cm=args.max_distance,
        min_strength=args.min_strength,
        invert_yaw=args.invert_yaw,
        time_offset_ms=args.time_offset,
    )
    if args.simulate:
        client = SimulatedClient()
    else:
        port = args.port or choose_port()
        client = LidarClient(port, args.baud, config.time_offset_ms)
    stop_event = threading.Event()
    threading.Thread(
        target=input_worker, args=(client, stop_event), daemon=True
    ).start()
    fig = plt.figure("Single Point LiDAR Scanner")
    ax = fig.add_subplot(111, projection="3d")
    scatter = None

    def on_close(_event):
        stop_event.set()

    fig.canvas.mpl_connect("close_event", on_close)
    plt.ion()
    plt.show()
    last_count = -1
    xyz = np.empty((0, 4))
    try:
        while not stop_event.is_set() and plt.fignum_exists(fig.number):
            while not client.messages.empty():
                print(client.messages.get())
            points = client.snapshot()
            if len(points) != last_count:
                xyz = polar_to_xyz(
                    points,
                    config=config,
                )
                ax.cla()
                ax.set_title(f"Points: {len(xyz)}")
                ax.set_xlabel("x [cm]")
                ax.set_ylabel("y [cm]")
                ax.set_zlabel("z [cm]")
                ax.set_box_aspect((1, 1, 0.6))
                if len(xyz) > 0:
                    scatter = ax.scatter(
                        xyz[:, 0],
                        xyz[:, 1],
                        xyz[:, 2],
                        c=color_values(xyz, args.color_by),
                        cmap="viridis",
                        s=8,
                    )
                    limit = max(10.0, np.max(np.abs(xyz[:, :3])) * 1.1)
                    ax.set_xlim(-limit, limit)
                    ax.set_ylim(-limit, limit)
                    ax.set_zlim(0, max(10.0, limit * 0.8))
                last_count = len(points)
            fig.canvas.draw_idle()
            plt.pause(0.05)
    finally:
        points = client.snapshot()
        xyz = polar_to_xyz(
            points,
            config=config,
        )
        saved_count = save_csv(
            Path(args.output),
            points,
            config=config,
        )
        client.close()
        print(f"Saved {saved_count} points to {args.output}")


if __name__ == "__main__":
    main()
