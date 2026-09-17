from __future__ import annotations

import argparse
import csv
import math
import sys
import threading
import time
from pathlib import Path

import mujoco

try:
    import mujoco.viewer
except Exception:
    mujoco_viewer = None
else:
    mujoco_viewer = mujoco.viewer

from five_bar_kinematics import bilateral_hip_targets


THIS_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = THIS_DIR.parent
MODEL_PATH = THIS_DIR / "models" / "two_wheel_legged.xml"
GUI_DIR = (
    PROJECT_ROOT
    / "Integrated_Control_System"
    / "Integrated_Robot_Control_Package"
    / "Python_GUI"
)

if str(GUI_DIR) not in sys.path:
    sys.path.insert(0, str(GUI_DIR))

from lqr_controller import LQRController  # noqa: E402


WHEEL_RADIUS_M = 0.033
DEFAULT_LEG_HEIGHT_M = 0.120
DEFAULT_MIN_LEG_HEIGHT_M = 0.060
DEFAULT_MAX_LEG_HEIGHT_M = 0.156
DEFAULT_PRINT_PERIOD_S = 0.05
DEFAULT_GUI_PLOT_WINDOW_S = 15.0
DEFAULT_GUI_HISTORY_S = 300.0
DEFAULT_GUI_SAMPLE_PERIOD_S = 0.02


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def make_lqr_model_xml(torque_limit_nm: float) -> str:
    xml = MODEL_PATH.read_text(encoding="utf-8")
    xml = xml.replace(
        '<velocity name="left_wheel_speed" joint="left_wheel_joint"\n'
        '              class="wheel_velocity_servo"/>',
        '<motor name="left_wheel_torque" joint="left_wheel_joint"\n'
        f'           gear="1" ctrllimited="true" ctrlrange="{-torque_limit_nm:.6f} {torque_limit_nm:.6f}"/>',
    )
    xml = xml.replace(
        '<velocity name="right_wheel_speed" joint="right_wheel_joint"\n'
        '              class="wheel_velocity_servo"/>',
        '<motor name="right_wheel_torque" joint="right_wheel_joint"\n'
        f'           gear="1" ctrllimited="true" ctrlrange="{-torque_limit_nm:.6f} {torque_limit_nm:.6f}"/>',
    )
    return xml


def set_root_pitch(data: mujoco.MjData, pitch_rad: float) -> None:
    half = 0.5 * pitch_rad
    data.qpos[3:7] = [math.cos(half), 0.0, math.sin(half), 0.0]


def root_pitch_rad(data: mujoco.MjData) -> float:
    w, x, y, z = data.qpos[3:7]
    r00 = 1.0 - 2.0 * (y * y + z * z)
    r20 = 2.0 * (x * z - w * y)
    return math.atan2(-r20, r00)


def root_attitude_deg(data: mujoco.MjData) -> tuple[float, float, float]:
    w, x, y, z = [float(value) for value in data.qpos[3:7]]
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(clamp(2.0 * (w * y - z * x), -1.0, 1.0))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def pitch_rate_radps(data: mujoco.MjData) -> float:
    return float(data.sensor("imu_gyro").data[1])


def average_wheel_state(data: mujoco.MjData) -> tuple[float, float]:
    left_pos = float(data.joint("left_wheel_joint").qpos[0])
    right_pos = float(data.joint("right_wheel_joint").qpos[0])
    left_vel = float(data.joint("left_wheel_joint").qvel[0])
    right_vel = float(data.joint("right_wheel_joint").qvel[0])
    return (
        0.5 * (left_pos + right_pos) * WHEEL_RADIUS_M,
        0.5 * (left_vel + right_vel) * WHEEL_RADIUS_M,
    )


def apply_leg_height_targets(data: mujoco.MjData, leg_height_m: float) -> None:
    data.ctrl[0:4] = bilateral_hip_targets(leg_height_m, leg_height_m)


class LiveLegHeight:
    def __init__(self, initial_m: float, minimum_m: float, maximum_m: float) -> None:
        self.minimum_m = minimum_m
        self.maximum_m = maximum_m
        self._value_m = clamp(initial_m, minimum_m, maximum_m)
        self._lock = threading.Lock()

    def get(self) -> float:
        with self._lock:
            return self._value_m

    def set(self, value_m: float) -> float:
        with self._lock:
            self._value_m = clamp(value_m, self.minimum_m, self.maximum_m)
            return self._value_m


class LiveAttitudeTrace:
    def __init__(self, window_s: float, history_s: float, sample_period_s: float) -> None:
        self.window_s = max(0.5, window_s)
        self.history_s = max(self.window_s, history_s)
        self.sample_period_s = max(0.001, sample_period_s)
        self._samples: list[dict[str, float]] = []
        self._last_sample_time = -math.inf
        self._lock = threading.Lock()

    def append(
        self,
        time_s: float,
        roll_deg: float,
        pitch_deg: float,
        yaw_deg: float,
    ) -> None:
        with self._lock:
            if time_s - self._last_sample_time < self.sample_period_s:
                return
            self._last_sample_time = time_s
            self._samples.append(
                {
                    "time_s": time_s,
                    "roll_deg": roll_deg,
                    "pitch_deg": pitch_deg,
                    "yaw_deg": yaw_deg,
                }
            )
            cutoff = time_s - self.history_s
            while len(self._samples) > 2 and self._samples[0]["time_s"] < cutoff:
                self._samples.pop(0)

    def snapshot(self) -> list[dict[str, float]]:
        with self._lock:
            return list(self._samples)


class LegHeightGui:
    def __init__(
        self,
        command: LiveLegHeight,
        attitude_trace: LiveAttitudeTrace,
    ) -> None:
        self.command = command
        self.attitude_trace = attitude_trace
        self.thread = threading.Thread(target=self._run, name="leg-height-gui", daemon=True)

    def start(self) -> None:
        self.thread.start()

    def _run(self) -> None:
        import tkinter as tk
        from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
        from matplotlib.figure import Figure

        root = tk.Tk()
        root.title("MuJoCo leg height")
        root.resizable(False, False)
        root.attributes("-topmost", True)

        value_mm = tk.DoubleVar(value=self.command.get() * 1000.0)
        label = tk.Label(root, text="")
        label.pack(padx=12, pady=(12, 4))

        def update_label(value: float) -> None:
            label.config(text=f"Leg height: {value:.1f} mm")

        def on_slider(raw_value: str) -> None:
            value = self.command.set(float(raw_value) / 1000.0)
            update_label(value * 1000.0)

        slider = tk.Scale(
            root,
            from_=self.command.minimum_m * 1000.0,
            to=self.command.maximum_m * 1000.0,
            orient=tk.HORIZONTAL,
            resolution=1.0,
            length=460,
            variable=value_mm,
            command=on_slider,
        )
        slider.pack(padx=12, pady=4)

        def reset_height() -> None:
            value = self.command.set(DEFAULT_LEG_HEIGHT_M)
            value_mm.set(value * 1000.0)
            update_label(value * 1000.0)

        tk.Button(root, text="Reset 120 mm", command=reset_height).pack(padx=12, pady=(4, 8))

        line_controls = tk.Frame(root)
        line_controls.pack(padx=12, pady=(0, 6), fill=tk.X)
        show_roll = tk.BooleanVar(value=True)
        show_pitch = tk.BooleanVar(value=True)
        show_yaw = tk.BooleanVar(value=True)

        for text, variable in (
            ("Roll", show_roll),
            ("Pitch", show_pitch),
            ("Yaw", show_yaw),
        ):
            tk.Checkbutton(
                line_controls,
                text=text,
                variable=variable,
                indicatoron=False,
                width=8,
            ).pack(side=tk.LEFT, padx=2)

        def show_all_lines() -> None:
            show_roll.set(True)
            show_pitch.set(True)
            show_yaw.set(True)

        def hide_all_lines() -> None:
            show_roll.set(False)
            show_pitch.set(False)
            show_yaw.set(False)

        tk.Button(line_controls, text="All", command=show_all_lines, width=6).pack(side=tk.LEFT, padx=(8, 2))
        tk.Button(line_controls, text="None", command=hide_all_lines, width=6).pack(side=tk.LEFT, padx=2)

        figure = Figure(figsize=(5.4, 3.2), dpi=100)
        axis = figure.add_subplot(111)
        axis.set_title("Body attitude")
        axis.set_xlabel("Time (s)")
        axis.set_ylabel("Angle (deg)")
        axis.grid(True, alpha=0.3)
        axis.axhline(0.0, color="0.65", linewidth=0.8)
        roll_line, = axis.plot([], [], label="Roll", linewidth=1.2)
        pitch_line, = axis.plot([], [], label="Pitch", linewidth=1.8)
        yaw_line, = axis.plot([], [], label="Yaw", linewidth=1.2)
        axis.legend(loc="upper right")

        canvas = FigureCanvasTkAgg(figure, master=root)
        canvas.get_tk_widget().pack(padx=12, pady=(4, 4))

        timeline_frame = tk.Frame(root)
        timeline_frame.pack(padx=12, pady=(0, 12), fill=tk.X)
        follow_live = tk.BooleanVar(value=True)
        timeline_var = tk.DoubleVar(value=0.0)
        timeline_label = tk.Label(timeline_frame, text="Time view: live")
        timeline_label.pack(anchor=tk.W)
        slider_is_syncing = False

        def on_timeline_drag(_raw_value: str) -> None:
            nonlocal slider_is_syncing
            if not slider_is_syncing:
                follow_live.set(False)

        timeline = tk.Scale(
            timeline_frame,
            from_=0.0,
            to=0.0,
            orient=tk.HORIZONTAL,
            resolution=0.1,
            length=460,
            variable=timeline_var,
            command=on_timeline_drag,
        )
        timeline.pack(fill=tk.X)
        tk.Checkbutton(timeline_frame, text="Follow live", variable=follow_live).pack(anchor=tk.W)

        def visible_value_sets(
            roll_values: list[float],
            pitch_values: list[float],
            yaw_values: list[float],
        ) -> list[float]:
            values: list[float] = []
            if show_roll.get():
                values.extend(roll_values)
            if show_pitch.get():
                values.extend(pitch_values)
            if show_yaw.get():
                values.extend(yaw_values)
            return values

        def refresh_plot() -> None:
            nonlocal slider_is_syncing
            samples = self.attitude_trace.snapshot()
            if samples:
                latest_time = samples[-1]["time_s"]
                window_s = self.attitude_trace.window_s
                max_start = max(0.0, latest_time - window_s)
                timeline.config(to=max_start)

                if follow_live.get():
                    view_start = max_start
                    slider_is_syncing = True
                    timeline_var.set(view_start)
                    slider_is_syncing = False
                else:
                    view_start = clamp(float(timeline_var.get()), 0.0, max_start)
                view_end = view_start + window_s

                visible_samples = [
                    sample for sample in samples
                    if view_start <= sample["time_s"] <= view_end
                ]
                if not visible_samples:
                    visible_samples = samples[-1:]

                time_values = [sample["time_s"] for sample in visible_samples]
                roll_values = [sample["roll_deg"] for sample in visible_samples]
                pitch_values = [sample["pitch_deg"] for sample in visible_samples]
                yaw_values = [sample["yaw_deg"] for sample in visible_samples]
                roll_line.set_data(time_values, roll_values)
                pitch_line.set_data(time_values, pitch_values)
                yaw_line.set_data(time_values, yaw_values)
                roll_line.set_visible(show_roll.get())
                pitch_line.set_visible(show_pitch.get())
                yaw_line.set_visible(show_yaw.get())

                axis.set_xlim(view_start, max(view_start + 1.0, view_end))
                all_values = visible_value_sets(roll_values, pitch_values, yaw_values)
                if not all_values:
                    all_values = [0.0]
                low = min(all_values) - 2.0
                high = max(all_values) + 2.0
                if high - low < 10.0:
                    center = 0.5 * (high + low)
                    low = center - 5.0
                    high = center + 5.0
                axis.set_ylim(low, high)
                timeline_label.config(
                    text=(
                        f"Time view: {view_start:.1f}-{min(view_end, latest_time):.1f} s"
                        f" / latest {latest_time:.1f} s"
                    )
                )
                canvas.draw_idle()
            root.after(100, refresh_plot)

        update_label(self.command.get() * 1000.0)
        refresh_plot()
        root.mainloop()


def build_controller(args: argparse.Namespace) -> LQRController:
    controller = LQRController()
    controller.set_enabled(True)
    controller.set_torque_limit(args.total_torque_limit)
    controller.set_output_scale(args.output_scale)
    controller.set_weights(args.q1, args.q2, args.q3, args.q4, args.r)
    return controller


def control_step(
    data: mujoco.MjData,
    controller: LQRController,
    args: argparse.Namespace,
    leg_height_m: float,
) -> tuple[float, float, float, float]:
    wheel_position_m, wheel_velocity_mps = average_wheel_state(data)
    pitch = root_pitch_rad(data)
    rate = pitch_rate_radps(data)

    total_torque_nm = controller.compute_balance_torque_from_wheel_state(
        wheel_position_m=args.state_sign * wheel_position_m,
        wheel_velocity_mps=args.state_sign * wheel_velocity_mps,
        pitch_deg=math.degrees(pitch),
        pitch_rate_dps=math.degrees(rate),
        target_position_m=args.target_position,
        target_velocity_mps=args.target_velocity,
        target_pitch_deg=args.target_pitch_deg,
        leg_height_mm=leg_height_m * 1000.0,
    )

    per_wheel = 0.5 * args.torque_sign * total_torque_nm
    per_wheel = clamp(per_wheel, -args.per_wheel_torque_limit, args.per_wheel_torque_limit)
    data.ctrl[4] = per_wheel
    data.ctrl[5] = per_wheel
    return pitch, rate, wheel_position_m, total_torque_nm


def run(args: argparse.Namespace) -> list[dict[str, float]]:
    per_wheel_limit = abs(float(args.per_wheel_torque_limit))
    model = mujoco.MjModel.from_xml_string(make_lqr_model_xml(per_wheel_limit))
    data = mujoco.MjData(model)
    controller = build_controller(args)
    leg_height_command = LiveLegHeight(
        args.leg_height,
        args.leg_height_min,
        args.leg_height_max,
    )
    attitude_trace = LiveAttitudeTrace(
        args.gui_plot_window,
        args.gui_history,
        args.gui_sample_period,
    )
    if args.leg_height_gui:
        LegHeightGui(leg_height_command, attitude_trace).start()

    mujoco.mj_resetDataKeyframe(model, data, 0)
    set_root_pitch(data, math.radians(args.initial_pitch_deg))
    apply_leg_height_targets(data, leg_height_command.get())
    mujoco.mj_forward(model, data)

    records: list[dict[str, float]] = []
    next_print = 0.0
    steps = int(args.seconds / model.opt.timestep)

    def one_step() -> None:
        nonlocal next_print
        leg_height_m = leg_height_command.get()
        apply_leg_height_targets(data, leg_height_m)
        pitch, rate, wheel_position_m, total_torque_nm = control_step(
            data,
            controller,
            args,
            leg_height_m,
        )
        roll_deg, pitch_deg, yaw_deg = root_attitude_deg(data)
        attitude_trace.append(data.time, roll_deg, pitch_deg, yaw_deg)
        mujoco.mj_step(model, data)
        if data.time + 1.0e-12 >= next_print:
            records.append(
                {
                    "time_s": float(data.time),
                    "roll_deg": roll_deg,
                    "pitch_deg": math.degrees(pitch),
                    "pitch_rate_dps": math.degrees(rate),
                    "yaw_deg": yaw_deg,
                    "wheel_position_m": wheel_position_m,
                    "total_torque_nm": total_torque_nm,
                    "left_ctrl_nm": float(data.ctrl[4]),
                    "right_ctrl_nm": float(data.ctrl[5]),
                    "leg_height_m": leg_height_m,
                }
            )
            next_print += args.log_period

    if args.viewer:
        if mujoco_viewer is None:
            raise RuntimeError("mujoco.viewer is not available in this Python environment")
        with mujoco_viewer.launch_passive(model, data) as viewer:
            start = time.perf_counter()
            while viewer.is_running() and data.time < args.seconds:
                one_step()
                viewer.sync()
                sleep_s = start + data.time - time.perf_counter()
                if sleep_s > 0:
                    time.sleep(sleep_s)
    else:
        for _ in range(steps):
            one_step()

    return records


def write_csv(path: Path, records: list[dict[str, float]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=list(records[0].keys()))
        writer.writeheader()
        writer.writerows(records)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run MuJoCo wheel torque LQR simulation.")
    parser.add_argument("--seconds", type=float, default=8.0)
    parser.add_argument("--initial-pitch-deg", type=float, default=3.0)
    parser.add_argument("--target-pitch-deg", type=float, default=0.0)
    parser.add_argument("--target-position", type=float, default=0.0)
    parser.add_argument("--target-velocity", type=float, default=0.0)
    parser.add_argument("--leg-height", type=float, default=DEFAULT_LEG_HEIGHT_M)
    parser.add_argument("--leg-height-min", type=float, default=DEFAULT_MIN_LEG_HEIGHT_M)
    parser.add_argument("--leg-height-max", type=float, default=DEFAULT_MAX_LEG_HEIGHT_M)
    parser.add_argument("--leg-height-gui", action="store_true")
    parser.add_argument("--gui-plot-window", type=float, default=DEFAULT_GUI_PLOT_WINDOW_S)
    parser.add_argument("--gui-history", type=float, default=DEFAULT_GUI_HISTORY_S)
    parser.add_argument("--gui-sample-period", type=float, default=DEFAULT_GUI_SAMPLE_PERIOD_S)
    parser.add_argument("--total-torque-limit", type=float, default=1.2)
    parser.add_argument("--per-wheel-torque-limit", type=float, default=0.6)
    parser.add_argument("--output-scale", type=float, default=1.0)
    parser.add_argument("--torque-sign", type=float, choices=(-1.0, 1.0), default=1.0)
    parser.add_argument("--state-sign", type=float, choices=(-1.0, 1.0), default=1.0)
    parser.add_argument("--q1", type=float, default=1.0)
    parser.add_argument("--q2", type=float, default=1.0)
    parser.add_argument("--q3", type=float, default=8.0)
    parser.add_argument("--q4", type=float, default=1.0)
    parser.add_argument("--r", type=float, default=5.0)
    parser.add_argument("--log-period", type=float, default=DEFAULT_PRINT_PERIOD_S)
    parser.add_argument("--csv", type=Path, default=None)
    parser.add_argument("--viewer", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    records = run(args)
    first = records[0]
    last = records[-1]
    print(f"model             : {MODEL_PATH}")
    print("wheel actuator    : torque motor, generated at load time")
    print(f"samples           : {len(records)}")
    print(f"initial pitch deg : {first['pitch_deg']:.3f}")
    print(f"final pitch deg   : {last['pitch_deg']:.3f}")
    print(f"final position m  : {last['wheel_position_m']:.3f}")
    print(f"final torque Nm   : {last['total_torque_nm']:.3f}")
    print(f"left/right ctrl   : {last['left_ctrl_nm']:.3f}, {last['right_ctrl_nm']:.3f} Nm")
    if args.csv:
        write_csv(args.csv, records)
        print(f"csv               : {args.csv}")


if __name__ == "__main__":
    main()
