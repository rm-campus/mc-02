"""Read-only live plot for the RobotControlTelemetry USB serial stream."""

from collections import deque
import argparse
from time import monotonic
import tkinter as tk
from tkinter import ttk

import serial
from serial.tools import list_ports


PREFIX = b"servo_wave:"
INVALID_SPEED = -2147483648
WINDOW_SEC = 8
JOINTS = ("001 腕俯仰", "002 肘俯仰", "003 肩俯仰", "004 腕滚转", "005 夹爪")
MODES = {1: "上档 · 底盘", 0: "中档 · 肩/肘", -1: "下档 · 腕/夹爪", 2: "等待遥控"}
ACTUAL, TARGET, SPEED = "#086dbe", "#d26113", "#107653"


def parse_frame(line):
    if not line.startswith(PREFIX):
        return None
    try:
        fields = [int(value) for value in line[len(PREFIX):].split(b",")]
    except ValueError:
        return None
    if len(fields) != 17 or not 0 <= fields[0] <= 0xFFFFFFFF or fields[1] not in MODES:
        return None
    return fields


class Scope:
    def __init__(self, root):
        self.root = root
        self.port = None
        self.buffer = b""
        self.history = deque(maxlen=500)
        self.last_tick = None
        self.time_sec = 0.0
        self.latest = None
        self.last_rx = None

        root.title("MC02 舵机示波器")
        root.minsize(700, 520)
        root.columnconfigure(0, weight=1)
        root.rowconfigure(3, weight=1)
        root.rowconfigure(4, weight=1)
        controls = ttk.Frame(root, padding=(12, 12, 12, 4))
        controls.grid(row=0, column=0, sticky="ew")
        ttk.Label(controls, text="MC02 串口").pack(side="left")
        self.port_box = ttk.Combobox(controls, state="readonly", width=9)
        self.port_box.pack(side="left", padx=6)
        ttk.Button(controls, text="刷新", command=self.refresh).pack(side="left")
        self.connect_button = ttk.Button(controls, text="连接", command=self.toggle)
        self.connect_button.pack(side="left", padx=6)
        ttk.Label(controls, text="舵机").pack(side="left", padx=(18, 4))
        self.joint = ttk.Combobox(controls, values=JOINTS, state="readonly", width=14)
        self.joint.current(2)
        self.joint.pack(side="left")
        self.joint.bind("<<ComboboxSelected>>", lambda _: self.render())

        self.status = tk.StringVar()
        self.readout = tk.StringVar(value="暂无舵机反馈")
        ttk.Label(root, textvariable=self.status, padding=(12, 4)).grid(row=1, column=0, sticky="w")
        ttk.Label(root, textvariable=self.readout, padding=(12, 4)).grid(row=2, column=0, sticky="w")
        self.position_chart = tk.Canvas(root, background="white", highlightthickness=1, highlightbackground="#d6dde5")
        self.speed_chart = tk.Canvas(root, background="white", highlightthickness=1, highlightbackground="#d6dde5")
        for row, canvas in ((3, self.position_chart), (4, self.speed_chart)):
            canvas.grid(row=row, column=0, padx=12, pady=(4, 8), sticky="nsew")
            canvas.bind("<Configure>", lambda _: self.render())
        root.protocol("WM_DELETE_WINDOW", self.close)
        self.refresh()
        self.poll()

    def refresh(self):
        ports = [port for port in list_ports.comports() if port.vid is not None]
        names = [port.device for port in ports]
        self.port_box.config(values=names)
        if self.port_box.get() not in names:
            self.port_box.set(names[0] if len(names) == 1 else "")
        if self.port is None:
            self.status.set("选择 MC02 的 USB 串口" if names else "未找到 USB 串口，请接入已烧录回传固件的 MC02")

    def disconnect(self):
        if self.port is not None:
            self.port.close()
            self.port = None
        self.connect_button.config(text="连接")

    def toggle(self):
        if self.port is not None:
            self.disconnect()
            self.status.set("已断开")
            return
        name = self.port_box.get()
        if not name:
            self.status.set("请先选择 MC02 的 USB 串口")
            return
        port = serial.Serial(baudrate=115200, timeout=0)
        port.port = name
        port.dtr = False
        port.rts = False
        try:
            port.open()
        except (serial.SerialException, OSError) as error:
            self.status.set(f"打开 {name} 失败：{error}")
            return
        self.port = port
        self.buffer = b""
        self.history.clear()
        self.last_tick = self.latest = None
        self.last_rx = monotonic()
        self.time_sec = 0.0
        self.connect_button.config(text="断开")
        self.status.set(f"{name} 已连接，等待 servo_wave 数据")
        self.render()

    def collect(self, fields):
        tick, mode = fields[:2]
        delta = 0 if self.last_tick is None else (tick - self.last_tick) & 0xFFFFFFFF
        if delta > 0x7FFFFFFF:
            self.history.clear()
            self.time_sec = 0.0
            delta = 0
        self.time_sec += delta / 1000
        self.last_tick = tick
        positions, speeds, targets = fields[2:7], fields[7:12], fields[12:17]
        row = [value if 500 <= value <= 2500 else None for value in positions]
        row += [value if value != INVALID_SPEED and row[i] is not None else None
                for i, value in enumerate(speeds)]
        row += [value if 500 <= value <= 2500 and row[i] is not None else None
                for i, value in enumerate(targets)]
        self.history.append((self.time_sec, row))
        self.latest = row
        self.last_rx = monotonic()
        self.status.set(f"{self.port.port} · {MODES[mode]}")

    def poll(self):
        if self.port is not None:
            try:
                data = self.port.read(min(self.port.in_waiting, 4096))
                lines = (self.buffer + data).split(b"\n")
                self.buffer = lines.pop()
                if len(self.buffer) > 256:
                    self.buffer = b""
                received = False
                for line in lines:
                    fields = parse_frame(line)
                    if fields is not None:
                        self.collect(fields)
                        received = True
                if received:
                    self.render()
                elif monotonic() - self.last_rx > 1:
                    self.status.set(f"{self.port.port} · 超过 1 秒未收到波形数据")
                    self.readout.set("暂无实时反馈")
            except (serial.SerialException, OSError) as error:
                self.disconnect()
                self.status.set(f"串口已断开：{error}")
        self.root.after(40, self.poll)

    def chart(self, canvas, title, series, unit):
        canvas.delete("all")
        width, height = canvas.winfo_width(), canvas.winfo_height()
        x0, y0, x1, y1 = 60, 32, width - 20, height - 26
        if x1 <= x0 or y1 <= y0:
            return
        canvas.create_text(12, 14, anchor="w", text=title, fill="#1c2936")
        start = self.time_sec - WINDOW_SEC
        rows = [(t, row) for t, row in self.history if t >= start]
        values = [row[index] for _, row in rows for index, _, _ in series if row[index] is not None]
        low, high = (min(values), max(values)) if values else (0, 1)
        margin = max(1, (high - low) * 0.08)
        low, high = low - margin, high + margin
        for i in range(5):
            y = y1 - (y1 - y0) * i / 4
            x = x0 + (x1 - x0) * i / 4
            canvas.create_line(x0, y, x1, y, fill="#e6edf3")
            canvas.create_line(x, y0, x, y1, fill="#e6edf3")
            canvas.create_text(x0 - 5, y, anchor="e", text=f"{low + (high - low) * i / 4:.0f}")
            canvas.create_text(x, y1 + 12, text=f"{i * 2 - WINDOW_SEC}s")
        canvas.create_text(5, y0 - 8, anchor="w", text=unit, fill="#536878")
        for label_index, (index, color, label) in enumerate(series):
            canvas.create_text(x1 - 75 * label_index, 14, anchor="e", text=label, fill=color)
            points = []
            for t, row in rows:
                value = row[index]
                if value is None:
                    if len(points) >= 4:
                        canvas.create_line(*points, fill=color, width=2)
                    points.clear()
                    continue
                points.extend((x0 + (x1 - x0) * (t - start) / WINDOW_SEC,
                               y1 - (y1 - y0) * (value - low) / (high - low)))
            if len(points) >= 4:
                canvas.create_line(*points, fill=color, width=2)

    def render(self):
        i = self.joint.current()
        self.chart(self.position_chart, "位置", [(i, ACTUAL, "实测"), (10 + i, TARGET, "已发送目标")], "P")
        self.chart(self.speed_chart, "速度（位置差分）", [(5 + i, SPEED, "估算速度")], "P/s")
        if self.latest is None or self.latest[i] is None:
            self.readout.set(f"{JOINTS[i]} · 暂无有效位置")
            return
        position, speed, target = (self.latest[j + i] for j in (0, 5, 10))
        self.readout.set(f"{JOINTS[i]}    实测 {position} P    "
                         f"目标 {target if target is not None else '—'} P    "
                         f"速度 {speed if speed is not None else '—'} P/s")

    def close(self):
        self.disconnect()
        self.root.destroy()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="Connect to this MC02 USB serial port at launch, e.g. COM5.")
    parser.add_argument("--joint", type=int, choices=range(1, 6), default=1,
                        help="Select a joint at launch, e.g. 5 for the gripper.")
    args = parser.parse_args()
    window = tk.Tk()
    scope = Scope(window)
    scope.joint.current(args.joint - 1)
    if args.port:
        scope.port_box.set(args.port)
        scope.toggle()
    window.mainloop()
