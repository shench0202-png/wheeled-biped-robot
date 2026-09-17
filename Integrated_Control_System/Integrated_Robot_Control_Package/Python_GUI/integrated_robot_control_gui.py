from __future__ import annotations

import csv
import datetime as dt
import json
import math
import struct
import sys
import time
from collections import deque
from dataclasses import dataclass
from queue import Empty, Queue

import serial
import serial.tools.list_ports
import pyqtgraph as pg
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QCloseEvent
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFileDialog,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSlider,
    QSpinBox,
    QTabWidget,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

from fuzzy_controller import FuzzyBalanceController
from lqr_controller import LQRController, MODEL_WHEEL_RADIUS

try:
    import numpy as np
    import pyqtgraph.opengl as gl
except Exception:
    np = None
    gl = None


PACKET_HEADER = b"\xAA\x55"
PACKET_TYPE_IMU = 0x01
PACKET_TYPE_IMU_STATUS = 0x02
PACKET_TYPE_LQR = 0x03
IMU_PAYLOAD_LEN_LEGACY = 13
IMU_PAYLOAD_LEN = 14
IMU_STATUS_PAYLOAD_LEN = 1
LQR_PAYLOAD_LEN = 41

ROUTING_SINGLE_DENG_M1_M2 = "single_deng_m1_m2"
ROUTING_DUAL_DENG_M1 = "dual_deng_m1"


def xor_checksum(payload: str) -> int:
    checksum = 0
    for value in payload.encode("ascii"):
        checksum ^= value
    return checksum


def build_text_packet(payload: str) -> bytes:
    return f"{payload}*{xor_checksum(payload):02X}\n".encode("ascii")


def verify_text_packet(line: str) -> str | None:
    if "*" not in line:
        return None
    payload, checksum_text = line.rsplit("*", 1)
    try:
        expected = int(checksum_text[:2], 16)
    except ValueError:
        return None
    return payload if xor_checksum(payload) == expected else None


@dataclass
class ImuTelemetry:
    roll_deg: float = 0.0
    pitch_deg: float = 0.0
    yaw_deg: float = 0.0
    motion_state: int = 0
    mag_interference: int = 0


@dataclass
class LqrTelemetry:
    pitch_deg: float = 0.0
    pitch_rate_dps: float = 0.0
    target_pitch_deg: float = 0.0
    leg_height_mm: float = 60.0
    enabled: bool = False
    raw_torque_nm: float = 0.0
    limited_torque_nm: float = 0.0
    k1: float = 0.0
    k2: float = 0.0
    k3: float = 0.0
    k4: float = 0.0


@dataclass
class ServoReadResult:
    imu: ImuTelemetry | None = None
    lqr: LqrTelemetry | None = None
    message: str | None = None


@dataclass
class MotorTelemetry:
    mode: str
    command_torque_nm: float
    angle_rad: float
    electrical_angle_rad: float
    velocity_radps: float
    measured_torque_nm: float
    iq_measured_a: float
    iq_target_a: float
    uq_command_v: float
    kt_nm_per_a: float
    torque_limit_nm: float


@dataclass
class DengTelemetry:
    timestamp_ms: int
    enabled: bool
    m1: MotorTelemetry
    m2: MotorTelemetry
    voltage_limit_v: float
    current_limit_a: float
    faults: int

    @classmethod
    def from_payload(cls, payload: str) -> "DengTelemetry | None":
        fields = payload.split(",")
        if len(fields) < 28 or fields[0] != "TQC2":
            return None
        try:
            return cls(
                timestamp_ms=int(fields[1]),
                enabled=bool(int(fields[2])),
                m1=MotorTelemetry(
                    fields[3],
                    float(fields[4]),
                    float(fields[5]),
                    float(fields[6]),
                    float(fields[7]),
                    float(fields[8]),
                    float(fields[9]),
                    float(fields[10]),
                    float(fields[11]),
                    float(fields[12]),
                    float(fields[13]),
                ),
                m2=MotorTelemetry(
                    fields[14],
                    float(fields[15]),
                    float(fields[16]),
                    float(fields[17]),
                    float(fields[18]),
                    float(fields[19]),
                    float(fields[20]),
                    float(fields[21]),
                    float(fields[22]),
                    float(fields[23]),
                    float(fields[24]),
                ),
                voltage_limit_v=float(fields[25]),
                current_limit_a=float(fields[26]),
                faults=int(fields[27]),
            )
        except ValueError:
            return None


@dataclass
class FocDiagnosticTelemetry:
    timestamp_ms: int
    test: str
    active_motor: str
    enabled: bool
    m1_angle_deg: float
    m1_velocity_radps: float
    m1_iq_a: float
    m1_iq_target_a: float
    m1_uq_v: float
    m2_angle_deg: float
    m2_velocity_radps: float
    m2_iq_a: float
    m2_iq_target_a: float
    m2_uq_v: float
    zero_electrical_angle_rad: float
    saturated: bool
    estop: bool
    voltage_limit_v: float
    current_limit_a: float

    @classmethod
    def from_payload(cls, payload: str) -> "FocDiagnosticTelemetry | None":
        fields = payload.split(",")
        if len(fields) < 20 or fields[0] != "FDD":
            return None
        try:
            return cls(
                timestamp_ms=int(fields[1]),
                test=fields[2],
                active_motor=fields[3],
                enabled=bool(int(fields[4])),
                m1_angle_deg=float(fields[5]),
                m1_velocity_radps=float(fields[6]),
                m1_iq_a=float(fields[7]),
                m1_iq_target_a=float(fields[8]),
                m1_uq_v=float(fields[9]),
                m2_angle_deg=float(fields[10]),
                m2_velocity_radps=float(fields[11]),
                m2_iq_a=float(fields[12]),
                m2_iq_target_a=float(fields[13]),
                m2_uq_v=float(fields[14]),
                zero_electrical_angle_rad=float(fields[15]),
                saturated=bool(int(fields[16])),
                estop=bool(int(fields[17])),
                voltage_limit_v=float(fields[18]),
                current_limit_a=float(fields[19]),
            )
        except ValueError:
            return None


@dataclass
class FocDiagnosticResult:
    test: str
    motor: str
    target: float
    average_iq_a: float
    average_uq_v: float
    angle_delta_rad: float
    average_velocity_radps: float
    saturated: bool
    reason: str

    @classmethod
    def from_payload(cls, payload: str) -> "FocDiagnosticResult | None":
        fields = payload.split(",")
        if len(fields) < 10 or fields[0] != "DIAG":
            return None
        try:
            return cls(
                test=fields[1],
                motor=fields[2],
                target=float(fields[3]),
                average_iq_a=float(fields[4]),
                average_uq_v=float(fields[5]),
                angle_delta_rad=float(fields[6]),
                average_velocity_radps=float(fields[7]),
                saturated=bool(int(fields[8])),
                reason=fields[9],
            )
        except ValueError:
            return None


@dataclass
class WheelState:
    position_m: float = 0.0
    velocity_mps: float = 0.0
    valid: bool = False
    source: str = "no odom"


class WheelStateEstimator:
    def __init__(self) -> None:
        self.zero_angle_rad: float | None = None
        self.position_m = 0.0
        self.velocity_mps = 0.0
        self.valid = False

    def reset(self) -> None:
        self.zero_angle_rad = None
        self.position_m = 0.0
        self.velocity_mps = 0.0
        self.valid = False

    def update(
        self,
        positive_motor: MotorTelemetry | None,
        negative_motor: MotorTelemetry | None,
        wheel_radius_m: float,
        positive_direction: float,
        negative_direction: float,
    ) -> WheelState:
        angles: list[float] = []
        velocities: list[float] = []
        if positive_motor is not None:
            angles.append(float(positive_direction) * positive_motor.angle_rad)
            velocities.append(float(positive_direction) * positive_motor.velocity_radps)
        if negative_motor is not None:
            angles.append(float(negative_direction) * negative_motor.angle_rad)
            velocities.append(float(negative_direction) * negative_motor.velocity_radps)

        if not angles:
            return WheelState(self.position_m, self.velocity_mps, False, "no odom")

        mean_angle = sum(angles) / len(angles)
        if self.zero_angle_rad is None:
            self.zero_angle_rad = mean_angle

        radius = max(0.001, abs(float(wheel_radius_m)))
        self.position_m = (mean_angle - self.zero_angle_rad) * radius
        raw_velocity_mps = (sum(velocities) / len(velocities)) * radius
        if self.valid:
            self.velocity_mps = 0.7 * self.velocity_mps + 0.3 * raw_velocity_mps
        else:
            self.velocity_mps = raw_velocity_mps
        self.valid = True
        return WheelState(
            self.position_m,
            self.velocity_mps,
            True,
            "avg wheels" if len(angles) >= 2 else "single wheel",
        )


class ServoIntegratedClient:
    def __init__(self, baudrate: int = 115200, timeout: float = 0.0, device_id: str = "esp32") -> None:
        self.baudrate = baudrate
        self.timeout = timeout
        self.device_id = device_id
        self.port: serial.Serial | None = None
        self.rx_buffer = bytearray()
        self.rx_queue: Queue[ServoReadResult] = Queue()

    @staticmethod
    def list_ports() -> list[str]:
        return [port.device for port in serial.tools.list_ports.comports()]

    @property
    def is_connected(self) -> bool:
        return self.port is not None and self.port.is_open

    def connect(self, port_name: str) -> None:
        self.close()
        self.port = serial.Serial(port_name, self.baudrate, timeout=self.timeout, write_timeout=0.05)
        self.port.reset_input_buffer()
        self.port.reset_output_buffer()
        self.rx_buffer.clear()
        self._clear_queue()

    def close(self) -> None:
        if self.port and self.port.is_open:
            self.port.close()
        self.port = None
        self.rx_buffer.clear()
        self._clear_queue()

    def send_control_command(
        self,
        target_pitch_deg: float,
        left_height_mm: float,
        right_height_mm: float,
        wheel_x_mm: float,
        servo_enabled: bool,
        lqr_enabled: bool,
        torque_limit_nm: float,
        output_scale: float,
        lqr_gain: object | None = None,
        lqr_state: object | None = None,
        lqr_target_position_m: float | None = None,
        lqr_target_velocity_mps: float | None = None,
    ) -> None:
        if not self.is_connected:
            return
        lqr_command = {
            "enable": bool(lqr_enabled),
            "torque_limit_nm": float(torque_limit_nm),
            "output_scale": float(output_scale),
        }
        if lqr_gain is not None:
            lqr_command["K"] = [
                round(float(lqr_gain.k1), 6),
                round(float(lqr_gain.k2), 6),
                round(float(lqr_gain.k3), 6),
                round(float(lqr_gain.k4), 6),
            ]
        if lqr_state is not None and getattr(lqr_state, "valid", False):
            lqr_command["state"] = {
                "position_m": round(float(lqr_state.position_m), 6),
                "velocity_mps": round(float(lqr_state.velocity_mps), 6),
                "target_position_m": round(float(lqr_target_position_m or 0.0), 6),
                "target_velocity_mps": round(float(lqr_target_velocity_mps or 0.0), 6),
            }
        command = {
            "id": self.device_id,
            "target_pitch_deg": float(target_pitch_deg),
            "enable": bool(servo_enabled),
            "leg": {
                "L": float(left_height_mm),
                "R": float(right_height_mm),
                "X": float(wheel_x_mm),
            },
            "lqr": lqr_command,
        }
        self.port.write((json.dumps(command, separators=(",", ":")) + "\n").encode("utf-8"))

    def read_available(self) -> list[ServoReadResult]:
        if not self.is_connected:
            return []
        waiting = self.port.in_waiting
        if waiting:
            self.rx_buffer.extend(self.port.read(waiting))
            self._parse_rx_buffer()
        results: list[ServoReadResult] = []
        while True:
            try:
                results.append(self.rx_queue.get_nowait())
            except Empty:
                break
        return results

    def _parse_rx_buffer(self) -> None:
        while True:
            header_index = self.rx_buffer.find(PACKET_HEADER)
            if header_index < 0:
                if self.rx_buffer[-1:] == PACKET_HEADER[:1]:
                    self.rx_buffer[:] = self.rx_buffer[-1:]
                else:
                    self.rx_buffer.clear()
                return
            if header_index > 0:
                del self.rx_buffer[:header_index]
            if len(self.rx_buffer) < 5:
                return

            packet_type = self.rx_buffer[2]
            payload_len = self.rx_buffer[3]
            total_len = 2 + 1 + 1 + payload_len + 1
            if len(self.rx_buffer) < total_len:
                return

            payload = bytes(self.rx_buffer[4 : 4 + payload_len])
            checksum = self.rx_buffer[4 + payload_len]
            if checksum != calculate_binary_checksum(packet_type, payload_len, payload):
                del self.rx_buffer[0]
                continue

            del self.rx_buffer[:total_len]
            self._handle_packet(packet_type, payload)

    def _handle_packet(self, packet_type: int, payload: bytes) -> None:
        if packet_type == PACKET_TYPE_IMU and len(payload) in (IMU_PAYLOAD_LEN_LEGACY, IMU_PAYLOAD_LEN):
            roll, pitch, yaw, moving = struct.unpack("<fffB", payload[:13])
            mag_interference = payload[13] if len(payload) >= IMU_PAYLOAD_LEN else 0
            self.rx_queue.put(
                ServoReadResult(
                    imu=ImuTelemetry(roll, pitch, yaw, int(moving), int(mag_interference))
                )
            )
            return
        if packet_type == PACKET_TYPE_IMU_STATUS and len(payload) == IMU_STATUS_PAYLOAD_LEN:
            status = "CONNECTED" if payload[0] else "DISCONNECTED"
            self.rx_queue.put(ServoReadResult(message=f"IMU_STATUS\t{status}"))
            return
        if packet_type == PACKET_TYPE_LQR and len(payload) == LQR_PAYLOAD_LEN:
            pitch, rate, target, height = struct.unpack("<ffff", payload[:16])
            enabled = bool(payload[16])
            raw, limited, k1, k2, k3, k4 = struct.unpack("<ffffff", payload[17:41])
            self.rx_queue.put(
                ServoReadResult(
                    lqr=LqrTelemetry(pitch, rate, target, height, enabled, raw, limited, k1, k2, k3, k4)
                )
            )

    def _clear_queue(self) -> None:
        while True:
            try:
                self.rx_queue.get_nowait()
            except Empty:
                break


def calculate_binary_checksum(packet_type: int, payload_len: int, payload: bytes) -> int:
    checksum = packet_type ^ payload_len
    for value in payload:
        checksum ^= value
    return checksum


class DengFocClient:
    def __init__(self, baudrate: int = 115200, timeout: float = 0.0) -> None:
        self.baudrate = baudrate
        self.timeout = timeout
        self.port: serial.Serial | None = None

    @staticmethod
    def list_ports() -> list[str]:
        return [port.device for port in serial.tools.list_ports.comports()]

    @property
    def is_connected(self) -> bool:
        return self.port is not None and self.port.is_open

    def connect(self, port_name: str) -> None:
        self.close()
        self.port = serial.Serial(port_name, self.baudrate, timeout=self.timeout, write_timeout=0.05)
        self.port.reset_input_buffer()
        self.port.reset_output_buffer()

    def close(self) -> None:
        if self.port and self.port.is_open:
            self.port.close()
        self.port = None

    def send_payload(self, payload: str) -> None:
        if self.is_connected:
            self.port.write(build_text_packet(payload))

    def read_available(self, max_lines: int = 160) -> list[str]:
        if not self.is_connected:
            return []
        lines: list[str] = []
        while self.port.in_waiting and len(lines) < max_lines:
            raw = self.port.readline().decode("ascii", errors="ignore").strip()
            if raw:
                lines.append(raw)
        return lines


class IntegratedRobotControlGUI(QMainWindow):
    MAX_POINTS = 2500
    CONTROL_PERIOD_MS = 2
    POLL_PERIOD_MS = 20
    HEARTBEAT_PERIOD_MS = 250
    PLOT_PERIOD_MS = 50

    FAULT_LABELS = {
        0: "ESTOP",
        1: "WATCHDOG",
        2: "DISABLED",
        3: "SENSOR",
        4: "VOLTAGE",
    }

    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Integrated Robot Control GUI")
        self.resize(1650, 980)

        self.servo_client = ServoIntegratedClient()
        self.deng_a_client = DengFocClient()
        self.deng_b_client = DengFocClient()
        self.servo_connected = False
        self.deng_a_connected = False
        self.deng_b_connected = False

        self.latest_imu = ImuTelemetry()
        self.latest_lqr = LqrTelemetry()
        self.fuzzy_controller = FuzzyBalanceController()
        self.gui_lqr_controller = LQRController()
        self.lqr_gain_calculator = LQRController()
        self.last_deng_a_sample: DengTelemetry | None = None
        self.last_deng_b_sample: DengTelemetry | None = None
        self.wheel_estimator = WheelStateEstimator()
        self.current_wheel_state = WheelState()
        self.last_deng_a_ack = "---"
        self.last_deng_b_ack = "---"
        self.last_deng_a_command = "---"
        self.last_deng_b_command = "---"
        self.last_servo_message = "---"
        self.last_relay_ms = 0
        self.last_plot_ms = 0
        self.start_time = time.monotonic()
        self.latest_imu_pitch_rate_dps = 0.0
        self.prev_imu_pitch_deg: float | None = None
        self.prev_imu_pitch_time: float | None = None
        self.fuzzy_torque_nm = 0.0
        self.fuzzy_positive_torque_nm = 0.0
        self.fuzzy_negative_torque_nm = 0.0
        self.fuzzy_mode_limits = {"torque": 0.35, "velocity": 5.0}
        self.last_fuzzy_output_mode = "torque"
        self.pid_balance_torque_nm = 0.0
        self.pid_balance_raw_torque_nm = 0.0
        self.pid_balance_positive_torque_nm = 0.0
        self.pid_balance_negative_torque_nm = 0.0
        self.pid_balance_error_deg = 0.0
        self.pid_balance_integral = 0.0
        self.pid_balance_prev_time: float | None = None
        self.gui_lqr_torque_nm = 0.0
        self.gui_lqr_positive_torque_nm = 0.0
        self.gui_lqr_negative_torque_nm = 0.0

        self.time_history = deque(maxlen=self.MAX_POINTS)
        self.imu_history = {name: deque(maxlen=self.MAX_POINTS) for name in ("Roll", "Pitch", "Yaw")}
        self.lqr_history = {
            name: deque(maxlen=self.MAX_POINTS)
            for name in (
                "Pitch",
                "Pitch Rate",
                "Target Pitch",
                "Wheel Position",
                "Wheel Velocity",
                "Target Position",
                "Raw Torque",
                "Limited Torque",
                "GUI LQR Torque",
                "Fuzzy Output",
                "PID Torque",
            )
        }
        self.is_recording = False
        self.csv_file = None
        self.csv_writer = None
        self.is_deng_recording = False
        self.deng_csv_file = None
        self.deng_csv_writer = None
        self.deng_record_rows = deque(maxlen=self.MAX_POINTS)
        self.plot_channel_boxes: list[tuple[dict[str, object], dict[str, QCheckBox]]] = []
        self.foc_diag_samples: dict[str, FocDiagnosticTelemetry | None] = {
            "a": None,
            "b": None,
        }
        self.foc_diag_results: dict[str, FocDiagnosticResult | None] = {
            "a": None,
            "b": None,
        }
        self.foc_diag_start_ms: dict[str, int | None] = {"a": None, "b": None}
        self.foc_diag_history = {
            name: deque(maxlen=self.MAX_POINTS)
            for name in ("Time", "Angle", "Velocity", "Iq Measured", "Iq Target", "Uq Command")
        }

        self.init_ui()

        self.poll_timer = QTimer(self)
        self.poll_timer.timeout.connect(self.poll_serial)
        self.poll_timer.start(self.POLL_PERIOD_MS)

        self.control_timer = QTimer(self)
        self.control_timer.timeout.connect(self.send_servo_control)
        self.control_timer.start(self.CONTROL_PERIOD_MS)

        self.heartbeat_timer = QTimer(self)
        self.heartbeat_timer.timeout.connect(self.send_heartbeats)
        self.heartbeat_timer.start(self.HEARTBEAT_PERIOD_MS)

    def init_ui(self) -> None:
        root = QWidget()
        self.setCentralWidget(root)
        layout = QVBoxLayout(root)
        layout.addWidget(self.build_connection_panel())

        self.tabs = QTabWidget()
        self.tabs.addTab(self.build_imu_tab(), "IMU")
        self.tabs.addTab(self.build_motor_tab(), "Motor / PID")
        self.tabs.addTab(self.build_foc_diagnostic_tab(), "FOC Diagnostic")
        self.tabs.addTab(self.build_pid_balance_tab(), "PID Balance")
        self.tabs.addTab(self.build_lqr_tab(), "LQR")
        self.tabs.addTab(self.build_fuzzy_tab(), "Fuzzy")
        layout.addWidget(self.tabs, stretch=1)

        self.log = QTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumHeight(120)
        self.log.setStyleSheet("font-family: Consolas;")
        layout.addWidget(self.log)
        self.refresh_transmit_gain_labels()

    def build_connection_panel(self) -> QGroupBox:
        group = QGroupBox("Connections")
        layout = QGridLayout(group)
        self.servo_port_combo = QComboBox()
        self.deng_a_port_combo = QComboBox()
        self.deng_b_port_combo = QComboBox()
        self.refresh_ports()

        layout.addWidget(QLabel("Servo / IMU"), 0, 0)
        layout.addWidget(self.servo_port_combo, 0, 1)
        self.servo_connect_button = QPushButton("Connect Servo")
        self.servo_connect_button.clicked.connect(self.toggle_servo_connection)
        layout.addWidget(self.servo_connect_button, 0, 2)
        self.servo_status_label = self.make_status_label("Disconnected", False)
        layout.addWidget(self.servo_status_label, 0, 3)

        self.servo_board_combo = QComboBox()
        self.servo_board_combo.addItem("ESP32", "esp32")
        self.servo_board_combo.addItem("Teensy 4.1", "teensy41")
        layout.addWidget(QLabel("Servo board"), 0, 4)
        layout.addWidget(self.servo_board_combo, 0, 5)

        layout.addWidget(QLabel("DengFOC A"), 1, 0)
        layout.addWidget(self.deng_a_port_combo, 1, 1)
        self.deng_a_connect_button = QPushButton("Connect A")
        self.deng_a_connect_button.clicked.connect(lambda: self.toggle_deng_connection("a"))
        layout.addWidget(self.deng_a_connect_button, 1, 2)
        self.deng_a_status_label = self.make_status_label("Disconnected", False)
        layout.addWidget(self.deng_a_status_label, 1, 3)

        layout.addWidget(QLabel("DengFOC B"), 2, 0)
        layout.addWidget(self.deng_b_port_combo, 2, 1)
        self.deng_b_connect_button = QPushButton("Connect B")
        self.deng_b_connect_button.clicked.connect(lambda: self.toggle_deng_connection("b"))
        layout.addWidget(self.deng_b_connect_button, 2, 2)
        self.deng_b_status_label = self.make_status_label("Disconnected", False)
        layout.addWidget(self.deng_b_status_label, 2, 3)

        refresh_button = QPushButton("Refresh Ports")
        refresh_button.clicked.connect(self.refresh_ports)
        layout.addWidget(refresh_button, 1, 4, 2, 1)

        self.routing_combo = QComboBox()
        self.routing_combo.addItem("Single DengFOC A: M1 positive / M2 negative", ROUTING_SINGLE_DENG_M1_M2)
        self.routing_combo.addItem("Dual DengFOC: A M1 positive / B M1 negative", ROUTING_DUAL_DENG_M1)
        layout.addWidget(QLabel("Routing"), 1, 5)
        layout.addWidget(self.routing_combo, 1, 6, 1, 2)

        self.m1_direction_combo = self.build_direction_combo(default_sign=-1.0)
        self.m2_direction_combo = self.build_direction_combo(default_sign=-1.0)
        layout.addWidget(QLabel("M1 direction"), 2, 5)
        layout.addWidget(self.m1_direction_combo, 2, 6)
        layout.addWidget(QLabel("M2 direction"), 2, 7)
        layout.addWidget(self.m2_direction_combo, 2, 8)
        self.min_torque_comp_input = self.make_double_spin(0.0, 1.0, 0.0, 0.005, 4)
        layout.addWidget(QLabel("Min torque comp"), 3, 5)
        layout.addWidget(self.min_torque_comp_input, 3, 6)
        layout.addWidget(QLabel("Nm / motor"), 3, 7)
        return group

    def build_motor_tab(self) -> QWidget:
        tab = QWidget()
        layout = QHBoxLayout(tab)

        controls = QVBoxLayout()
        controls.addWidget(self.build_motor_manual_group())
        controls.addWidget(self.build_safe_group())
        controls.addWidget(self.build_pid_group())
        controls.addStretch()
        layout.addLayout(controls, stretch=2)

        status = QVBoxLayout()
        status.addWidget(self.build_motor_status_group())
        self.motor_plot = pg.PlotWidget()
        self.motor_plot.setBackground("w")
        self.motor_plot.addLegend()
        self.motor_plot.showGrid(x=True, y=True)
        self.motor_plot.setLabel("bottom", "Time", units="s")
        self.motor_plot.setLabel("left", "Torque / Current")
        self.motor_curves = {
            "Positive Torque": self.motor_plot.plot(name="Positive Torque", pen=pg.mkPen("#2ca02c", width=2)),
            "Negative Torque": self.motor_plot.plot(name="Negative Torque", pen=pg.mkPen("#17becf", width=2)),
            "Controller Torque": self.motor_plot.plot(name="Controller Torque", pen=pg.mkPen("#d62728", width=2)),
        }
        self.motor_history = {name: deque(maxlen=self.MAX_POINTS) for name in self.motor_curves}
        status.addWidget(self.build_plot_channel_group(self.motor_curves))
        status.addWidget(self.motor_plot, stretch=1)
        layout.addLayout(status, stretch=4)
        return tab

    def build_foc_diagnostic_tab(self) -> QWidget:
        tab = QWidget()
        layout = QHBoxLayout(tab)

        controls = QVBoxLayout()
        selection = QGroupBox("Diagnostic Target")
        selection_layout = QGridLayout(selection)
        self.foc_diag_board_combo = QComboBox()
        self.foc_diag_board_combo.addItem("DengFOC A", "a")
        self.foc_diag_board_combo.addItem("DengFOC B", "b")
        self.foc_diag_motor_combo = QComboBox()
        self.foc_diag_motor_combo.addItem("M1", "m1")
        self.foc_diag_motor_combo.addItem("M2", "m2")
        self.foc_diag_board_combo.currentIndexChanged.connect(self.reset_foc_diagnostic_plot)
        self.foc_diag_motor_combo.currentIndexChanged.connect(self.reset_foc_diagnostic_plot)
        selection_layout.addWidget(QLabel("Board"), 0, 0)
        selection_layout.addWidget(self.foc_diag_board_combo, 0, 1)
        selection_layout.addWidget(QLabel("Motor"), 0, 2)
        selection_layout.addWidget(self.foc_diag_motor_combo, 0, 3)
        ping_button = QPushButton("PING")
        ping_button.clicked.connect(lambda: self.send_foc_diagnostic_command("PING"))
        selection_layout.addWidget(ping_button, 1, 0)
        clear_button = QPushButton("CLR fault")
        clear_button.clicked.connect(lambda: self.send_foc_diagnostic_command("CLR"))
        selection_layout.addWidget(clear_button, 1, 1)
        zero_button = QPushButton("ZERO selected")
        zero_button.clicked.connect(
            lambda: self.send_foc_diagnostic_command(f"ZERO,{self.foc_diagnostic_motor()}")
        )
        selection_layout.addWidget(zero_button, 1, 2)
        estop_button = QPushButton("ESTOP")
        estop_button.setStyleSheet("background-color: #c0392b; color: white; font-weight: bold;")
        estop_button.clicked.connect(lambda: self.send_foc_diagnostic_command("ESTOP"))
        selection_layout.addWidget(estop_button, 1, 3)
        controls.addWidget(selection)

        safety = QGroupBox("Diagnostic Safety")
        safety_layout = QGridLayout(safety)
        self.foc_diag_voltage_input = self.make_double_spin(0.5, 8.0, 2.0, 0.1, 3)
        self.foc_diag_current_input = self.make_double_spin(0.1, 8.0, 0.5, 0.1, 3)
        self.foc_diag_watchdog_input = QSpinBox()
        self.foc_diag_watchdog_input.setRange(100, 5000)
        self.foc_diag_watchdog_input.setValue(800)
        safety_layout.addWidget(QLabel("Voltage limit"), 0, 0)
        safety_layout.addWidget(self.foc_diag_voltage_input, 0, 1)
        safety_layout.addWidget(QLabel("V"), 0, 2)
        safety_layout.addWidget(QLabel("Current limit"), 1, 0)
        safety_layout.addWidget(self.foc_diag_current_input, 1, 1)
        safety_layout.addWidget(QLabel("A"), 1, 2)
        safety_layout.addWidget(QLabel("Watchdog"), 2, 0)
        safety_layout.addWidget(self.foc_diag_watchdog_input, 2, 1)
        safety_layout.addWidget(QLabel("ms"), 2, 2)
        apply_safe = QPushButton("Apply SAFE")
        apply_safe.clicked.connect(self.apply_foc_diagnostic_safe)
        safety_layout.addWidget(apply_safe, 3, 0, 1, 3)
        controls.addWidget(safety)

        direction = QGroupBox("Runtime Direction Signs")
        direction_layout = QGridLayout(direction)
        self.foc_diag_sign_combos: dict[str, QComboBox] = {}
        sign_defaults = {
            "Commutation": -1.0,
            "Encoder": -1.0,
            "Velocity": 1.0,
            "Current": 1.0,
            "Actuator": 1.0,
        }
        for index, (name, default) in enumerate(sign_defaults.items()):
            combo = self.build_direction_combo(default)
            self.foc_diag_sign_combos[name.lower()] = combo
            direction_layout.addWidget(QLabel(name), index // 3 * 2, index % 3)
            direction_layout.addWidget(combo, index // 3 * 2 + 1, index % 3)
        apply_dir = QPushButton("Apply DIR")
        apply_dir.clicked.connect(self.apply_foc_diagnostic_directions)
        direction_layout.addWidget(apply_dir, 4, 0, 1, 3)
        controls.addWidget(direction)

        tests = QGroupBox("Timed Motor Tests")
        tests_layout = QGridLayout(tests)
        self.foc_diag_align_voltage = self.make_double_spin(0.2, 5.0, 1.0, 0.1, 3)
        self.foc_diag_align_duration = QSpinBox()
        self.foc_diag_align_duration.setRange(100, 8000)
        self.foc_diag_align_duration.setValue(700)
        tests_layout.addWidget(QLabel("ALIGN voltage / ms"), 0, 0)
        tests_layout.addWidget(self.foc_diag_align_voltage, 0, 1)
        tests_layout.addWidget(self.foc_diag_align_duration, 0, 2)
        align_button = QPushButton("ALIGN")
        align_button.clicked.connect(self.start_foc_diagnostic_align)
        tests_layout.addWidget(align_button, 0, 3)

        self.foc_diag_iq_input = self.make_double_spin(0.05, 3.0, 0.30, 0.05, 3)
        self.foc_diag_iq_duration = QSpinBox()
        self.foc_diag_iq_duration.setRange(100, 8000)
        self.foc_diag_iq_duration.setValue(600)
        tests_layout.addWidget(QLabel("TESTIQ current / ms"), 1, 0)
        tests_layout.addWidget(self.foc_diag_iq_input, 1, 1)
        tests_layout.addWidget(self.foc_diag_iq_duration, 1, 2)
        iq_positive = QPushButton("TESTIQ +")
        iq_positive.clicked.connect(lambda: self.start_foc_diagnostic_iq(1.0))
        tests_layout.addWidget(iq_positive, 1, 3)
        iq_negative = QPushButton("TESTIQ -")
        iq_negative.clicked.connect(lambda: self.start_foc_diagnostic_iq(-1.0))
        tests_layout.addWidget(iq_negative, 1, 4)

        self.foc_diag_open_voltage = self.make_double_spin(0.2, 5.0, 1.0, 0.1, 3)
        self.foc_diag_open_speed = self.make_double_spin(0.1, 100.0, 3.0, 0.5, 3)
        self.foc_diag_open_duration = QSpinBox()
        self.foc_diag_open_duration.setRange(100, 8000)
        self.foc_diag_open_duration.setValue(700)
        tests_layout.addWidget(QLabel("OPEN V / elec rad/s / ms"), 2, 0)
        tests_layout.addWidget(self.foc_diag_open_voltage, 2, 1)
        tests_layout.addWidget(self.foc_diag_open_speed, 2, 2)
        tests_layout.addWidget(self.foc_diag_open_duration, 2, 3)
        open_cw = QPushButton("OPEN CW")
        open_cw.clicked.connect(lambda: self.start_foc_diagnostic_open("cw"))
        tests_layout.addWidget(open_cw, 3, 1)
        open_ccw = QPushButton("OPEN CCW")
        open_ccw.clicked.connect(lambda: self.start_foc_diagnostic_open("ccw"))
        tests_layout.addWidget(open_ccw, 3, 2)

        self.foc_diag_phase_voltage = self.make_double_spin(0.2, 5.0, 0.8, 0.1, 3)
        self.foc_diag_phase_duration = QSpinBox()
        self.foc_diag_phase_duration.setRange(100, 3000)
        self.foc_diag_phase_duration.setValue(400)
        tests_layout.addWidget(QLabel("PHASE voltage / ms"), 4, 0)
        tests_layout.addWidget(self.foc_diag_phase_voltage, 4, 1)
        tests_layout.addWidget(self.foc_diag_phase_duration, 4, 2)
        for column, phase in enumerate(("a", "b", "c"), start=1):
            button = QPushButton(f"PHASE {phase.upper()}")
            button.clicked.connect(
                lambda _checked=False, selected_phase=phase: self.start_foc_diagnostic_phase(
                    selected_phase
                )
            )
            tests_layout.addWidget(button, 5, column)
        controls.addWidget(tests)

        status = QGroupBox("Diagnostic Status")
        status_layout = QGridLayout(status)
        self.foc_diag_status_labels = {}
        for index, name in enumerate(
            (
                "Test",
                "Active Motor",
                "Enabled",
                "M1 Angle",
                "M1 Velocity",
                "M1 Iq / Target",
                "M1 Uq",
                "M2 Angle",
                "M2 Velocity",
                "M2 Iq / Target",
                "M2 Uq",
                "Zero Electrical",
                "Fault",
                "Limits",
            )
        ):
            row = index // 2
            column = (index % 2) * 2
            status_layout.addWidget(QLabel(f"{name}:"), row, column)
            label = self.make_value_label()
            self.foc_diag_status_labels[name] = label
            status_layout.addWidget(label, row, column + 1)
        controls.addWidget(status)

        diagnosis = QGroupBox("Test Result")
        diagnosis_layout = QVBoxLayout(diagnosis)
        self.foc_diag_result_text = QTextEdit()
        self.foc_diag_result_text.setReadOnly(True)
        self.foc_diag_result_text.setMaximumHeight(115)
        self.foc_diag_result_text.setPlainText("No diagnostic result received.")
        diagnosis_layout.addWidget(self.foc_diag_result_text)
        save_button = QPushButton("Save Diagnostic CSV")
        save_button.clicked.connect(self.save_foc_diagnostic_csv)
        diagnosis_layout.addWidget(save_button)
        controls.addWidget(diagnosis)
        controls.addStretch()
        layout.addLayout(controls, stretch=2)

        plot_layout = QVBoxLayout()
        self.foc_diag_plot = pg.PlotWidget()
        self.foc_diag_plot.setBackground("w")
        self.foc_diag_plot.addLegend()
        self.foc_diag_plot.showGrid(x=True, y=True)
        self.foc_diag_plot.setLabel("bottom", "Time", units="s")
        self.foc_diag_plot.setLabel("left", "Diagnostic Value")
        colors = {
            "Angle": "#1f77b4",
            "Velocity": "#ff7f0e",
            "Iq Measured": "#2ca02c",
            "Iq Target": "#d62728",
            "Uq Command": "#9467bd",
        }
        self.foc_diag_curves = {
            name: self.foc_diag_plot.plot(name=name, pen=pg.mkPen(color, width=2))
            for name, color in colors.items()
        }
        plot_layout.addWidget(self.build_plot_channel_group(self.foc_diag_curves))
        clear_plot_button = QPushButton("Clear Diagnostic Plot")
        clear_plot_button.clicked.connect(self.reset_foc_diagnostic_plot)
        plot_layout.addWidget(clear_plot_button)
        plot_layout.addWidget(self.foc_diag_plot, stretch=1)
        layout.addLayout(plot_layout, stretch=4)
        return tab

    def build_motor_manual_group(self) -> QGroupBox:
        group = QGroupBox("Manual BLDC / PID Test")
        layout = QGridLayout(group)
        self.manual_mode_combo = QComboBox()
        self.manual_mode_combo.addItem("Torque", "torque")
        self.manual_mode_combo.addItem("Velocity", "velocity")
        self.manual_mode_combo.addItem("Angle", "position")
        self.manual_mode_combo.currentIndexChanged.connect(self.update_manual_bldc_units)
        self.manual_m1_target = self.make_double_spin(-3600.0, 3600.0, 0.0, 0.01, 4)
        self.manual_m2_target = self.make_double_spin(-3600.0, 3600.0, 0.0, 0.01, 4)
        self.manual_m1_unit_label = QLabel("Nm")
        self.manual_m2_unit_label = QLabel("Nm")
        self.manual_m1_current_label = self.make_value_label()
        self.manual_m2_current_label = self.make_value_label()
        self.manual_enable_box = QCheckBox("Enable manual BLDC output")
        layout.addWidget(QLabel("Mode"), 0, 0)
        layout.addWidget(self.manual_mode_combo, 0, 1, 1, 3)
        layout.addWidget(QLabel("M1 target"), 1, 0)
        layout.addWidget(self.manual_m1_target, 1, 1)
        layout.addWidget(self.manual_m1_unit_label, 1, 2)
        layout.addWidget(self.manual_m1_current_label, 1, 3)
        layout.addWidget(QLabel("M2 target"), 2, 0)
        layout.addWidget(self.manual_m2_target, 2, 1)
        layout.addWidget(self.manual_m2_unit_label, 2, 2)
        layout.addWidget(self.manual_m2_current_label, 2, 3)
        layout.addWidget(self.manual_enable_box, 3, 0, 1, 3)
        send_button = QPushButton("Send SET2")
        send_button.clicked.connect(self.send_manual_bldc_control)
        layout.addWidget(send_button, 4, 0)
        stop_button = QPushButton("Stop SET2")
        stop_button.clicked.connect(self.stop_manual_bldc_control)
        layout.addWidget(stop_button, 4, 1)
        zero_button = QPushButton("ZERO all")
        zero_button.clicked.connect(lambda: self.send_deng_payload_to_connected("ZERO,all"))
        layout.addWidget(zero_button, 4, 2)
        self.deng_record_button = QPushButton("Start Deng CSV")
        self.deng_record_button.clicked.connect(self.toggle_deng_recording)
        layout.addWidget(self.deng_record_button, 5, 0)
        snapshot_button = QPushButton("Save Deng Snapshot CSV")
        snapshot_button.clicked.connect(self.save_deng_snapshot_csv)
        layout.addWidget(snapshot_button, 5, 1, 1, 2)
        estop_button = QPushButton("ESTOP")
        estop_button.setStyleSheet("background-color: #c0392b; color: white; font-weight: bold;")
        estop_button.clicked.connect(self.estop)
        layout.addWidget(estop_button, 6, 0, 1, 4)
        for spin in (self.manual_m1_target, self.manual_m2_target):
            spin.valueChanged.connect(self.refresh_manual_current_labels)
        self.update_manual_bldc_units()
        return group

    def build_safe_group(self) -> QGroupBox:
        group = QGroupBox("Safety / Motor Constants")
        layout = QGridLayout(group)
        self.voltage_limit_input = self.make_double_spin(0.1, 12.6, 3.0, 0.1, 3)
        self.current_limit_input = self.make_double_spin(0.1, 12.0, 3.0, 0.1, 3)
        self.velocity_limit_input = self.make_double_spin(0.1, 300.0, 20.0, 0.5, 3)
        self.watchdog_input = self.make_double_spin(50, 5000, 500, 10, 0)
        self.kt_input = self.make_double_spin(0.001, 5.0, 0.0955, 0.001, 5)
        self.current_limit_input.valueChanged.connect(self.refresh_manual_current_labels)
        self.kt_input.valueChanged.connect(self.refresh_manual_current_labels)
        rows = (
            ("Voltage limit", self.voltage_limit_input, "V"),
            ("Current limit", self.current_limit_input, "A"),
            ("Velocity limit", self.velocity_limit_input, "rad/s"),
            ("Watchdog", self.watchdog_input, "ms"),
            ("Kt all", self.kt_input, "Nm/A"),
        )
        for row, (label, widget, unit) in enumerate(rows):
            layout.addWidget(QLabel(label), row, 0)
            layout.addWidget(widget, row, 1)
            layout.addWidget(QLabel(unit), row, 2)
        apply_safe = QPushButton("Apply SAFE")
        apply_safe.clicked.connect(self.send_safe)
        layout.addWidget(apply_safe, 5, 0)
        apply_kt = QPushButton("Apply KT")
        apply_kt.clicked.connect(lambda: self.send_deng_payload_to_connected(f"KT,all,{self.kt_input.value():.6f}"))
        layout.addWidget(apply_kt, 5, 1)
        query_button = QPushButton("Query CFG/PID")
        query_button.clicked.connect(self.query_deng_config)
        layout.addWidget(query_button, 5, 2)
        return group

    def build_pid_group(self) -> QGroupBox:
        group = QGroupBox("Runtime PID Tuning")
        layout = QGridLayout(group)
        self.pid_inputs: dict[str, dict[str, QDoubleSpinBox]] = {}
        defaults = {
            "PIDQ": (5.0, 80.0, 0.0, 100000.0, 0.0),
            "PIDV": (0.2, 4.0, 0.0005, 1000.0, 3.0),
            "PIDP": (8.0, 0.0, 0.0, 100000.0, 20.0),
        }
        labels = ("Kp", "Ki", "Kd", "Ramp", "Limit")
        for col, label in enumerate(("Loop", *labels, "")):
            layout.addWidget(QLabel(label), 0, col)
        for row, (command, values) in enumerate(defaults.items(), start=1):
            layout.addWidget(QLabel(command), row, 0)
            self.pid_inputs[command] = {}
            for col, (label, value) in enumerate(zip(labels, values), start=1):
                spin = self.make_double_spin(-100000.0, 100000.0, value, 0.01, 6)
                self.pid_inputs[command][label] = spin
                layout.addWidget(spin, row, col)
            button = QPushButton("Apply")
            button.clicked.connect(lambda _checked=False, cmd=command: self.send_pid(cmd))
            layout.addWidget(button, row, 6)
        return group

    def build_motor_status_group(self) -> QGroupBox:
        group = QGroupBox("DengFOC Telemetry")
        layout = QGridLayout(group)
        names = (
            "Routing",
            "Deng A",
            "Deng B",
            "Faults A/B",
            "Positive Cmd Torque",
            "Negative Cmd Torque",
            "Positive Measured Torque",
            "Negative Measured Torque",
            "Positive Iq Target",
            "Positive Iq Measured",
            "Negative Iq Target",
            "Negative Iq Measured",
            "Limits A",
            "Last Cmd A/B",
        )
        self.motor_status_labels = {}
        for index, name in enumerate(names):
            layout.addWidget(QLabel(f"{name}:"), index // 3, (index % 3) * 2)
            label = self.make_value_label()
            self.motor_status_labels[name] = label
            layout.addWidget(label, index // 3, (index % 3) * 2 + 1)
        return group

    def build_lqr_tab(self) -> QWidget:
        tab = QWidget()
        layout = QHBoxLayout(tab)
        controls = QVBoxLayout()
        controls.addWidget(self.build_lqr_control_group())
        controls.addWidget(self.build_lqr_status_group())
        controls.addStretch()
        layout.addLayout(controls, stretch=2)

        plot_area = QVBoxLayout()
        self.lqr_plot = pg.PlotWidget()
        self.lqr_plot.setBackground("w")
        self.lqr_plot.addLegend()
        self.lqr_plot.showGrid(x=True, y=True)
        self.lqr_plot.setLabel("bottom", "Time", units="s")
        self.lqr_curves = {
            "Pitch": self.lqr_plot.plot(name="Pitch", pen=pg.mkPen("#1f77b4", width=2)),
            "Pitch Rate": self.lqr_plot.plot(name="Pitch Rate", pen=pg.mkPen("#ff7f0e", width=2)),
            "Target Pitch": self.lqr_plot.plot(name="Target Pitch", pen=pg.mkPen("#111111", width=2, style=Qt.DashLine)),
            "Wheel Position": self.lqr_plot.plot(name="Wheel Position", pen=pg.mkPen("#2ca02c", width=2)),
            "Wheel Velocity": self.lqr_plot.plot(name="Wheel Velocity", pen=pg.mkPen("#9467bd", width=2)),
            "Target Position": self.lqr_plot.plot(name="Target Position", pen=pg.mkPen("#7f7f7f", width=2, style=Qt.DashLine)),
            "Raw Torque": self.lqr_plot.plot(name="Raw Torque", pen=pg.mkPen("#8c564b", width=2, style=Qt.DashLine)),
            "Limited Torque": self.lqr_plot.plot(name="Limited Torque", pen=pg.mkPen("#d62728", width=2)),
            "GUI LQR Torque": self.lqr_plot.plot(name="GUI LQR Torque", pen=pg.mkPen("#2ca02c", width=2)),
        }
        plot_area.addWidget(self.build_plot_channel_group(self.lqr_curves))
        plot_area.addWidget(self.lqr_plot)
        layout.addLayout(plot_area, stretch=5)
        return tab

    def build_pid_balance_tab(self) -> QWidget:
        tab = QWidget()
        layout = QHBoxLayout(tab)
        controls = QVBoxLayout()
        controls.addWidget(self.build_pid_balance_control_group())
        controls.addWidget(self.build_pid_balance_status_group())
        controls.addStretch()
        layout.addLayout(controls, stretch=2)

        plot_area = QVBoxLayout()
        self.pid_balance_plot = pg.PlotWidget()
        self.pid_balance_plot.setBackground("w")
        self.pid_balance_plot.addLegend()
        self.pid_balance_plot.showGrid(x=True, y=True)
        self.pid_balance_plot.setLabel("bottom", "Time", units="s")
        self.pid_balance_curves = {
            "Pitch": self.pid_balance_plot.plot(name="Pitch", pen=pg.mkPen("#1f77b4", width=2)),
            "Pitch Rate": self.pid_balance_plot.plot(name="Pitch Rate", pen=pg.mkPen("#ff7f0e", width=2)),
            "Target Pitch": self.pid_balance_plot.plot(
                name="Target Pitch",
                pen=pg.mkPen("#111111", width=2, style=Qt.DashLine),
            ),
            "PID Torque": self.pid_balance_plot.plot(name="PID Torque", pen=pg.mkPen("#d62728", width=2)),
        }
        plot_area.addWidget(self.build_plot_channel_group(self.pid_balance_curves))
        plot_area.addWidget(self.pid_balance_plot)
        layout.addLayout(plot_area, stretch=5)
        return tab

    def build_fuzzy_tab(self) -> QWidget:
        tab = QWidget()
        layout = QHBoxLayout(tab)
        controls = QVBoxLayout()
        controls.addWidget(self.build_fuzzy_control_group())
        controls.addWidget(self.build_fuzzy_status_group())
        controls.addStretch()
        layout.addLayout(controls, stretch=2)

        plot_area = QVBoxLayout()
        self.fuzzy_plot = pg.PlotWidget()
        self.fuzzy_plot.setBackground("w")
        self.fuzzy_plot.addLegend()
        self.fuzzy_plot.showGrid(x=True, y=True)
        self.fuzzy_plot.setLabel("bottom", "Time", units="s")
        self.fuzzy_curves = {
            "Pitch": self.fuzzy_plot.plot(name="Pitch", pen=pg.mkPen("#1f77b4", width=2)),
            "Pitch Rate": self.fuzzy_plot.plot(name="Pitch Rate", pen=pg.mkPen("#ff7f0e", width=2)),
            "Target Pitch": self.fuzzy_plot.plot(
                name="Target Pitch",
                pen=pg.mkPen("#111111", width=2, style=Qt.DashLine),
            ),
            "Fuzzy Output": self.fuzzy_plot.plot(name="Fuzzy Output", pen=pg.mkPen("#9467bd", width=2)),
        }
        plot_area.addWidget(self.build_plot_channel_group(self.fuzzy_curves))
        plot_area.addWidget(self.fuzzy_plot)
        layout.addLayout(plot_area, stretch=5)
        return tab

    def build_lqr_control_group(self) -> QGroupBox:
        group = QGroupBox("Servo ESP32 LQR Control")
        layout = QGridLayout(group)
        self.servo_enable_box = QCheckBox("Enable servos")
        self.lqr_enable_box = QCheckBox("Enable LQR")
        self.lqr_use_wheel_state_box = QCheckBox("Use wheel position / velocity")
        self.lqr_use_wheel_state_box.setChecked(False)
        self.relay_enable_box = QCheckBox("Relay GUI IMU LQR torque to DengFOC")
        self.relay_enable_box.toggled.connect(self.on_lqr_relay_toggled)
        layout.addWidget(self.servo_enable_box, 0, 0, 1, 2)
        layout.addWidget(self.lqr_enable_box, 0, 2, 1, 2)
        layout.addWidget(self.relay_enable_box, 1, 0, 1, 4)
        layout.addWidget(self.lqr_use_wheel_state_box, 1, 4, 1, 2)

        self.target_pitch_input = self.make_double_spin(-45.0, 45.0, 0.0, 0.1, 3)
        self.target_position_input = self.make_double_spin(-5.0, 5.0, 0.0, 0.01, 3)
        self.target_velocity_input = self.make_double_spin(-2.0, 2.0, 0.0, 0.01, 3)
        self.leg_height_slider, leg_height_layout = self.build_height_slider("Both legs")
        self.wheel_ik_x_input = self.make_double_spin(1.0, 39.0, 20.0, 1.0, 2)
        self.wheel_radius_input = self.make_double_spin(0.005, 0.300, MODEL_WHEEL_RADIUS, 0.001, 4)
        self.torque_limit_input = self.make_double_spin(0.0, 20.0, 2.0, 0.1, 3)
        self.output_scale_input = self.make_double_spin(-20.0, 20.0, 1.0, 0.1, 3)
        layout.addWidget(QLabel("Target pitch"), 2, 0)
        layout.addWidget(self.target_pitch_input, 2, 1)
        layout.addWidget(QLabel("deg"), 2, 2)
        layout.addWidget(QLabel("Target position"), 3, 0)
        layout.addWidget(self.target_position_input, 3, 1)
        layout.addWidget(QLabel("m"), 3, 2)
        layout.addWidget(QLabel("Target velocity"), 4, 0)
        layout.addWidget(self.target_velocity_input, 4, 1)
        layout.addWidget(QLabel("m/s"), 4, 2)

        height_layout = QHBoxLayout()
        height_layout.addLayout(leg_height_layout)
        layout.addWidget(QLabel("Leg height"), 5, 0)
        layout.addLayout(height_layout, 5, 1, 1, 3)
        layout.addWidget(QLabel("Wheel IK X"), 6, 0)
        layout.addWidget(self.wheel_ik_x_input, 6, 1)
        layout.addWidget(QLabel("mm"), 6, 2)
        layout.addWidget(QLabel("Wheel radius"), 7, 0)
        layout.addWidget(self.wheel_radius_input, 7, 1)
        layout.addWidget(QLabel("m"), 7, 2)

        rows = (
            ("Torque limit", self.torque_limit_input, "Nm"),
            ("Output scale", self.output_scale_input, ""),
        )
        for row, (name, widget, unit) in enumerate(rows, start=8):
            layout.addWidget(QLabel(name), row, 0)
            layout.addWidget(widget, row, 1)
            layout.addWidget(QLabel(unit), row, 2)

        self.q1_input = self.make_double_spin(0.000001, 100000.0, 1.0, 0.1, 6)
        self.q2_input = self.make_double_spin(0.000001, 100000.0, 1.0, 0.1, 6)
        self.q3_input = self.make_double_spin(0.000001, 100000.0, 8.0, 0.1, 6)
        self.q4_input = self.make_double_spin(0.000001, 100000.0, 1.0, 0.1, 6)
        self.r_input = self.make_double_spin(0.000001, 100000.0, 5.0, 0.1, 6)
        weight_widgets = (
            ("Q1", self.q1_input, "Q2", self.q2_input),
            ("Q3", self.q3_input, "Q4", self.q4_input),
        )
        for index, (left_name, left_widget, right_name, right_widget) in enumerate(weight_widgets, start=10):
            layout.addWidget(QLabel(left_name), index, 0)
            layout.addWidget(left_widget, index, 1)
            layout.addWidget(QLabel(right_name), index, 2)
            layout.addWidget(right_widget, index, 3)
        layout.addWidget(QLabel("R"), 12, 0)
        layout.addWidget(self.r_input, 12, 1)

        self.tx_gain_labels = {}
        for index, name in enumerate(("TX K1", "TX K2", "TX K3", "TX K4")):
            layout.addWidget(QLabel(name), 13 + index // 2, (index % 2) * 2)
            label = self.make_value_label()
            self.tx_gain_labels[name] = label
            layout.addWidget(label, 13 + index // 2, (index % 2) * 2 + 1)
        for spin in (self.q1_input, self.q2_input, self.q3_input, self.q4_input, self.r_input):
            spin.valueChanged.connect(self.refresh_transmit_gain_labels)
        self.leg_height_slider.valueChanged.connect(self.refresh_transmit_gain_labels)

        self.positive_odom_combo = self.build_direction_combo(default_sign=1.0)
        self.negative_odom_combo = self.build_direction_combo(default_sign=1.0)
        layout.addWidget(QLabel("Positive odom dir"), 15, 0)
        layout.addWidget(self.positive_odom_combo, 15, 1)
        layout.addWidget(QLabel("Negative odom dir"), 15, 2)
        layout.addWidget(self.negative_odom_combo, 15, 3)
        reset_odom_button = QPushButton("Reset Odom")
        reset_odom_button.clicked.connect(self.reset_wheel_state)
        layout.addWidget(reset_odom_button, 16, 3)

        send_button = QPushButton("Send Servo Command")
        send_button.clicked.connect(self.send_servo_control)
        layout.addWidget(send_button, 16, 0, 1, 3)
        return group

    def build_pid_balance_control_group(self) -> QGroupBox:
        group = QGroupBox("IMU PID Balance Torque Control")
        layout = QGridLayout(group)
        self.pid_balance_enable_box = QCheckBox("Relay PID torque to DengFOC")
        self.pid_balance_enable_box.toggled.connect(self.on_pid_balance_toggled)
        layout.addWidget(self.pid_balance_enable_box, 0, 0, 1, 4)

        self.pid_balance_kp_input = self.make_double_spin(-100.0, 100.0, 0.035, 0.001, 6)
        self.pid_balance_ki_input = self.make_double_spin(-100.0, 100.0, 0.0, 0.001, 6)
        self.pid_balance_kd_input = self.make_double_spin(-100.0, 100.0, 0.001, 0.001, 6)
        self.pid_balance_torque_limit_input = self.make_double_spin(0.0, 5.0, 0.35, 0.01, 3)
        self.pid_balance_output_scale_input = self.make_double_spin(-20.0, 20.0, 1.0, 0.05, 3)
        self.pid_balance_integral_limit_input = self.make_double_spin(0.0, 1000.0, 30.0, 1.0, 3)
        self.pid_balance_polarity_combo = QComboBox()
        self.pid_balance_polarity_combo.addItem("+1", 1.0)
        self.pid_balance_polarity_combo.addItem("-1", -1.0)

        rows = (
            ("Kp", self.pid_balance_kp_input, "Nm/deg", "Ki", self.pid_balance_ki_input, "Nm/(deg*s)"),
            ("Kd", self.pid_balance_kd_input, "Nm/(deg/s)", "Torque limit", self.pid_balance_torque_limit_input, "Nm"),
            ("Output scale", self.pid_balance_output_scale_input, "", "Integral limit", self.pid_balance_integral_limit_input, "deg*s"),
        )
        for row, (left_name, left_widget, left_unit, right_name, right_widget, right_unit) in enumerate(rows, start=1):
            layout.addWidget(QLabel(left_name), row, 0)
            layout.addWidget(left_widget, row, 1)
            layout.addWidget(QLabel(left_unit), row, 2)
            layout.addWidget(QLabel(right_name), row, 3)
            layout.addWidget(right_widget, row, 4)
            layout.addWidget(QLabel(right_unit), row, 5)

        layout.addWidget(QLabel("Polarity"), 4, 0)
        layout.addWidget(self.pid_balance_polarity_combo, 4, 1)
        reset_button = QPushButton("Reset I")
        reset_button.clicked.connect(self.reset_pid_balance_integral)
        layout.addWidget(reset_button, 4, 3)
        return group

    def build_fuzzy_control_group(self) -> QGroupBox:
        group = QGroupBox("Fuzzy IMU Motor Control")
        layout = QGridLayout(group)
        self.fuzzy_enable_box = QCheckBox("Relay fuzzy output to DengFOC")
        self.fuzzy_enable_box.toggled.connect(self.on_fuzzy_toggled)
        layout.addWidget(self.fuzzy_enable_box, 0, 0, 1, 4)

        self.fuzzy_output_mode_combo = QComboBox()
        self.fuzzy_output_mode_combo.addItem("Torque", "torque")
        self.fuzzy_output_mode_combo.addItem("Motor velocity", "velocity")
        layout.addWidget(QLabel("Output mode"), 0, 4)
        layout.addWidget(self.fuzzy_output_mode_combo, 0, 5)

        self.fuzzy_torque_limit_input = self.make_double_spin(0.0, 5.0, 0.35, 0.01, 3)
        self.fuzzy_output_scale_input = self.make_double_spin(0.0, 5.0, 1.0, 0.05, 3)
        self.fuzzy_angle_range_input = self.make_double_spin(0.5, 30.0, 8.0, 0.5, 3)
        self.fuzzy_rate_range_input = self.make_double_spin(5.0, 500.0, 120.0, 5.0, 3)
        self.fuzzy_deadband_angle_input = self.make_double_spin(0.0, 5.0, 0.20, 0.05, 3)
        self.fuzzy_deadband_rate_input = self.make_double_spin(0.0, 30.0, 2.0, 0.5, 3)
        self.fuzzy_safe_pitch_input = self.make_double_spin(2.0, 60.0, 25.0, 1.0, 3)
        self.fuzzy_height_gain_input = self.make_double_spin(-1.0, 1.0, 0.0, 0.05, 3)

        self.fuzzy_polarity_combo = QComboBox()
        self.fuzzy_polarity_combo.addItem("+1", 1.0)
        self.fuzzy_polarity_combo.addItem("-1", -1.0)

        rows = (
            ("Output limit", self.fuzzy_torque_limit_input, "Nm", "Output scale", self.fuzzy_output_scale_input, ""),
            ("Pitch range", self.fuzzy_angle_range_input, "deg", "Rate range", self.fuzzy_rate_range_input, "dps"),
            ("Pitch deadband", self.fuzzy_deadband_angle_input, "deg", "Rate deadband", self.fuzzy_deadband_rate_input, "dps"),
            ("Safe pitch", self.fuzzy_safe_pitch_input, "deg", "Height gain", self.fuzzy_height_gain_input, ""),
        )
        for row, (left_name, left_widget, left_unit, right_name, right_widget, right_unit) in enumerate(rows, start=1):
            left_label = QLabel(left_name)
            left_unit_label = QLabel(left_unit)
            layout.addWidget(left_label, row, 0)
            layout.addWidget(left_widget, row, 1)
            layout.addWidget(left_unit_label, row, 2)
            layout.addWidget(QLabel(right_name), row, 3)
            layout.addWidget(right_widget, row, 4)
            layout.addWidget(QLabel(right_unit), row, 5)
            if row == 1:
                self.fuzzy_output_limit_label = left_label
                self.fuzzy_output_unit_label = left_unit_label

        layout.addWidget(QLabel("Polarity"), 5, 0)
        layout.addWidget(self.fuzzy_polarity_combo, 5, 1)
        self.fuzzy_output_mode_combo.currentIndexChanged.connect(self.on_fuzzy_output_mode_changed)
        self.update_fuzzy_output_mode_ui(stop_output=False)
        return group

    def build_lqr_status_group(self) -> QGroupBox:
        group = QGroupBox("LQR Telemetry")
        layout = QGridLayout(group)
        names = (
            "Pitch",
            "Pitch Rate",
            "Target Pitch",
            "Target Position",
            "Target Velocity",
            "Wheel Position",
            "Wheel Velocity",
            "Wheel Source",
            "LQR Wheel State",
            "Leg Height",
            "Wheel IK X",
            "LQR Enabled",
            "Raw Torque",
            "Limited Torque",
            "K1",
            "K2",
            "K3",
            "K4",
            "IMU Pitch Rate",
            "GUI LQR Torque",
            "Last Servo Message",
        )
        self.lqr_status_labels = {}
        for index, name in enumerate(names):
            layout.addWidget(QLabel(f"{name}:"), index, 0)
            label = self.make_value_label()
            self.lqr_status_labels[name] = label
            layout.addWidget(label, index, 1)
        return group

    def build_fuzzy_status_group(self) -> QGroupBox:
        group = QGroupBox("Fuzzy Status")
        layout = QGridLayout(group)
        names = (
            "Pitch",
            "Pitch Rate",
            "Target Pitch",
            "Output Mode",
            "Fuzzy Output",
            "M1 Command",
            "M2 Command",
            "M1 Measured Velocity",
            "M2 Measured Velocity",
            "Fuzzy Out",
            "Last Servo Message",
        )
        self.fuzzy_status_labels = {}
        for index, name in enumerate(names):
            layout.addWidget(QLabel(f"{name}:"), index, 0)
            label = self.make_value_label()
            self.fuzzy_status_labels[name] = label
            layout.addWidget(label, index, 1)
        return group

    def build_pid_balance_status_group(self) -> QGroupBox:
        group = QGroupBox("PID Balance Status")
        layout = QGridLayout(group)
        names = (
            "Pitch",
            "Pitch Rate",
            "Target Pitch",
            "PID Error",
            "PID Integral",
            "PID Raw Torque",
            "PID Torque",
            "PID M1",
            "PID M2",
            "Last Servo Message",
        )
        self.pid_balance_status_labels = {}
        for index, name in enumerate(names):
            layout.addWidget(QLabel(f"{name}:"), index, 0)
            label = self.make_value_label()
            self.pid_balance_status_labels[name] = label
            layout.addWidget(label, index, 1)
        return group

    def build_imu_tab(self) -> QWidget:
        tab = QWidget()
        layout = QHBoxLayout(tab)
        left = QVBoxLayout()
        left.addWidget(self.build_imu_status_group())
        self.imu_plot = pg.PlotWidget()
        self.imu_plot.setBackground("w")
        self.imu_plot.addLegend()
        self.imu_plot.showGrid(x=True, y=True)
        self.imu_plot.setLabel("bottom", "Time", units="s")
        self.imu_plot.setLabel("left", "Angle", units="deg")
        self.imu_curves = {
            "Roll": self.imu_plot.plot(name="Roll", pen=pg.mkPen("#d62728", width=2)),
            "Pitch": self.imu_plot.plot(name="Pitch", pen=pg.mkPen("#2ca02c", width=2)),
            "Yaw": self.imu_plot.plot(name="Yaw", pen=pg.mkPen("#1f77b4", width=2)),
        }
        left.addWidget(self.build_plot_channel_group(self.imu_curves))
        left.addWidget(self.imu_plot, stretch=1)
        record_row = QHBoxLayout()
        self.record_button = QPushButton("Start CSV")
        self.record_button.clicked.connect(self.toggle_recording)
        record_row.addWidget(self.record_button)
        snapshot_button = QPushButton("Save Snapshot")
        snapshot_button.clicked.connect(self.save_snapshot_csv)
        record_row.addWidget(snapshot_button)
        record_row.addStretch()
        left.addLayout(record_row)
        layout.addLayout(left, stretch=4)

        if gl is not None and np is not None:
            self.gl_widget = gl.GLViewWidget()
            self.gl_widget.setCameraPosition(distance=15)
            grid = gl.GLGridItem()
            grid.scale(1, 1, 1)
            self.gl_widget.addItem(grid)
            box_verts = np.array(
                [
                    [-2, -1, -0.2], [2, -1, -0.2], [2, 1, -0.2], [-2, 1, -0.2],
                    [-2, -1, 0.2], [2, -1, 0.2], [2, 1, 0.2], [-2, 1, 0.2],
                ]
            )
            box_faces = np.array(
                [
                    [0, 1, 2], [0, 2, 3], [4, 5, 6], [4, 6, 7],
                    [0, 1, 5], [0, 5, 4], [1, 2, 6], [1, 6, 5],
                    [2, 3, 7], [2, 7, 6], [3, 0, 4], [3, 4, 7],
                ]
            )
            colors = np.array([[1, 0, 0, 0.8]] * 4 + [[0, 1, 0, 0.8]] * 4 + [[0, 0, 1, 0.8]] * 4)
            self.box_mesh = gl.GLMeshItem(
                vertexes=box_verts,
                faces=box_faces,
                faceColors=colors,
                smooth=False,
                drawEdges=True,
                edgeColor=(0, 0, 0, 1),
            )
            self.gl_widget.addItem(self.box_mesh)
            axis = gl.GLAxisItem()
            axis.setSize(x=3, y=3, z=3)
            self.gl_widget.addItem(axis)
            layout.addWidget(self.gl_widget, stretch=2)
        else:
            self.box_mesh = None
            layout.addWidget(QLabel("3D view unavailable"), stretch=2)
        return tab

    def build_imu_status_group(self) -> QGroupBox:
        group = QGroupBox("IMU Telemetry")
        layout = QGridLayout(group)
        names = ("Roll", "Pitch", "Yaw", "Motion State", "Mag Interference", "IMU Status")
        self.imu_status_labels = {}
        for index, name in enumerate(names):
            layout.addWidget(QLabel(f"{name}:"), 0, index * 2)
            label = self.make_value_label()
            self.imu_status_labels[name] = label
            layout.addWidget(label, 0, index * 2 + 1)
        self.imu_status_labels["IMU Status"].setText("UNKNOWN")
        return group

    def refresh_ports(self) -> None:
        ports = ServoIntegratedClient.list_ports()
        for combo in (
            getattr(self, "servo_port_combo", None),
            getattr(self, "deng_a_port_combo", None),
            getattr(self, "deng_b_port_combo", None),
        ):
            if combo is None:
                continue
            selected = combo.currentText()
            combo.clear()
            combo.addItems(ports)
            if selected in ports:
                combo.setCurrentText(selected)

    def toggle_servo_connection(self) -> None:
        if self.servo_connected:
            self.servo_client.close()
            self.servo_connected = False
            self.servo_connect_button.setText("Connect Servo")
            self.set_status_label(self.servo_status_label, "Disconnected", False)
            return
        port = self.servo_port_combo.currentText()
        if not port:
            QMessageBox.warning(self, "No port", "Select Servo / IMU COM first.")
            return
        try:
            self.servo_client.device_id = self.servo_board_combo.currentData() or "esp32"
            self.servo_client.connect(port)
            self.servo_connected = True
            self.servo_connect_button.setText("Disconnect Servo")
            self.set_status_label(self.servo_status_label, f"Connected: {port}", True)
            self.append_log(f"Servo connected: {port}")
        except serial.SerialException as exc:
            QMessageBox.critical(self, "Servo connection error", str(exc))

    def toggle_deng_connection(self, side: str) -> None:
        if self.deng_connected(side):
            self.safe_deng_disconnect(side)
            return
        port = self.deng_port_combo(side).currentText()
        if not port:
            QMessageBox.warning(self, "No port", f"Select DengFOC {side.upper()} COM first.")
            return
        try:
            self.deng_client(side).connect(port)
            self.set_deng_connected(side, True)
            self.deng_connect_button(side).setText(f"Disconnect {side.upper()}")
            self.set_status_label(self.deng_status_label(side), f"Connected: {port}", True)
            self.send_deng_payload("PING", side=side)
            self.send_deng_payload("CFG?", side=side)
            self.send_deng_payload("PID?", side=side)
        except serial.SerialException as exc:
            QMessageBox.critical(self, f"DengFOC {side.upper()} connection error", str(exc))

    def poll_serial(self) -> None:
        if self.servo_connected:
            try:
                for result in self.servo_client.read_available():
                    self.handle_servo_result(result)
            except serial.SerialException as exc:
                self.last_servo_message = f"Servo read error: {exc}"
                self.servo_client.close()
                self.servo_connected = False
                self.set_status_label(self.servo_status_label, "Disconnected", False)

        for side in ("a", "b"):
            if self.deng_connected(side):
                try:
                    for line in self.deng_client(side).read_available():
                        payload = verify_text_packet(line)
                        if payload is None:
                            self.set_last_deng_ack(side, f"Bad packet: {line}")
                            continue
                        sample = DengTelemetry.from_payload(payload)
                        if sample:
                            self.handle_deng_sample(side, sample)
                        else:
                            self.handle_deng_message(payload, side)
                except serial.SerialException:
                    self.safe_deng_disconnect(side)

        self.relay_lqr_to_deng()
        self.relay_fuzzy_to_deng()
        self.relay_pid_balance_to_deng()
        now_ms = int(time.monotonic() * 1000)
        if now_ms - self.last_plot_ms >= self.PLOT_PERIOD_MS:
            self.last_plot_ms = now_ms
            self.append_plot_samples()
            self.refresh_plots()
            self.update_status_panels()

    def handle_servo_result(self, result: ServoReadResult) -> None:
        if result.imu:
            self.update_imu_pitch_rate(result.imu.pitch_deg)
            self.latest_imu = result.imu
            self.update_3d_model(result.imu.roll_deg, result.imu.pitch_deg, result.imu.yaw_deg)
            return
        if result.lqr:
            self.latest_lqr = result.lqr
            return
        if result.message:
            self.last_servo_message = result.message
            if result.message.startswith("IMU_STATUS\t"):
                self.imu_status_labels["IMU Status"].setText(result.message.split("\t", 1)[1])

    def update_imu_pitch_rate(self, pitch_deg: float) -> None:
        now = time.monotonic()
        if self.prev_imu_pitch_deg is not None and self.prev_imu_pitch_time is not None:
            dt_s = max(0.001, now - self.prev_imu_pitch_time)
            self.latest_imu_pitch_rate_dps = (float(pitch_deg) - self.prev_imu_pitch_deg) / dt_s
        self.prev_imu_pitch_deg = float(pitch_deg)
        self.prev_imu_pitch_time = now

    def estimate_wheel_state(self) -> WheelState:
        positive_motor, negative_motor = self.current_motor_samples()
        self.current_wheel_state = self.wheel_estimator.update(
            positive_motor,
            negative_motor,
            self.wheel_radius_input.value(),
            self.positive_odom_combo.currentData(),
            self.negative_odom_combo.currentData(),
        )
        return self.current_wheel_state

    def reset_wheel_state(self) -> None:
        self.wheel_estimator.reset()
        self.current_wheel_state = WheelState()
        self.append_log("Wheel odometry zero reset")

    def send_servo_control(self) -> None:
        if not self.servo_connected:
            return
        try:
            leg_height = self.leg_height_slider.value()
            lqr_gain = self.calculate_transmit_gain(leg_height)
            wheel_state = self.estimate_wheel_state()
            lqr_state, target_position_m, target_velocity_mps = self.selected_lqr_wheel_state(wheel_state)
            self.servo_client.send_control_command(
                self.target_pitch_input.value(),
                leg_height,
                leg_height,
                self.wheel_ik_x_input.value(),
                self.servo_enable_box.isChecked(),
                self.lqr_enable_box.isChecked(),
                self.torque_limit_input.value(),
                self.output_scale_input.value(),
                lqr_gain,
                lqr_state,
                target_position_m,
                target_velocity_mps,
            )
        except serial.SerialException as exc:
            self.last_servo_message = f"Servo write error: {exc}"

    def calculate_transmit_gain(self, leg_height_mm: float):
        self.lqr_gain_calculator.set_weights(
            self.q1_input.value(),
            self.q2_input.value(),
            self.q3_input.value(),
            self.q4_input.value(),
            self.r_input.value(),
        )
        return self.lqr_gain_calculator.update_gains(leg_height_mm)

    def selected_lqr_wheel_state(self, wheel_state: WheelState) -> tuple[WheelState, float, float]:
        if self.lqr_use_wheel_state_box.isChecked() and wheel_state.valid:
            return wheel_state, self.target_position_input.value(), self.target_velocity_input.value()
        return WheelState(0.0, 0.0, True, "forced zero"), 0.0, 0.0

    def refresh_transmit_gain_labels(self, *_unused) -> None:
        if not hasattr(self, "tx_gain_labels"):
            return
        gain = self.calculate_transmit_gain(self.leg_height_slider.value())
        values = {
            "TX K1": gain.k1,
            "TX K2": gain.k2,
            "TX K3": gain.k3,
            "TX K4": gain.k4,
        }
        for name, value in values.items():
            self.tx_gain_labels[name].setText(f"{value:.6f}")

    def on_fuzzy_toggled(self, checked: bool) -> None:
        if checked and self.relay_enable_box.isChecked():
            self.relay_enable_box.setChecked(False)
        if checked and self.pid_balance_enable_box.isChecked():
            self.pid_balance_enable_box.setChecked(False)
        if not checked:
            self.fuzzy_torque_nm = 0.0
            self.fuzzy_positive_torque_nm = 0.0
            self.fuzzy_negative_torque_nm = 0.0
            self.stop_fuzzy_motor_output()

    def on_fuzzy_output_mode_changed(self, *_unused) -> None:
        self.update_fuzzy_output_mode_ui(stop_output=True)

    def update_fuzzy_output_mode_ui(self, stop_output: bool) -> None:
        mode = self.fuzzy_output_mode()
        if hasattr(self, "fuzzy_torque_limit_input"):
            self.fuzzy_mode_limits[self.last_fuzzy_output_mode] = self.fuzzy_torque_limit_input.value()
            if mode == "velocity":
                self.fuzzy_output_limit_label.setText("Velocity limit")
                self.fuzzy_output_unit_label.setText("rad/s")
                self.fuzzy_torque_limit_input.setRange(0.0, 300.0)
                self.fuzzy_torque_limit_input.setSingleStep(0.5)
                self.fuzzy_torque_limit_input.setDecimals(3)
            else:
                self.fuzzy_output_limit_label.setText("Torque limit")
                self.fuzzy_output_unit_label.setText("Nm")
                self.fuzzy_torque_limit_input.setRange(0.0, 5.0)
                self.fuzzy_torque_limit_input.setSingleStep(0.01)
                self.fuzzy_torque_limit_input.setDecimals(3)
            self.fuzzy_torque_limit_input.setValue(self.fuzzy_mode_limits[mode])
        self.last_fuzzy_output_mode = mode
        self.fuzzy_torque_nm = 0.0
        self.fuzzy_positive_torque_nm = 0.0
        self.fuzzy_negative_torque_nm = 0.0
        if stop_output:
            self.stop_fuzzy_motor_output()

    def fuzzy_output_mode(self) -> str:
        if not hasattr(self, "fuzzy_output_mode_combo"):
            return "torque"
        return self.fuzzy_output_mode_combo.currentData() or "torque"

    def stop_fuzzy_motor_output(self) -> None:
        self.send_deng_payload_to_connected("LQR2,0.000000,0.000000,0", quiet=True)
        self.send_deng_payload_to_connected(
            "SET2,velocity,0.000000,0,velocity,0.000000,0",
            quiet=True,
        )

    def on_lqr_relay_toggled(self, checked: bool) -> None:
        if checked and self.fuzzy_enable_box.isChecked():
            self.fuzzy_enable_box.setChecked(False)
        if checked and self.pid_balance_enable_box.isChecked():
            self.pid_balance_enable_box.setChecked(False)
        if not checked:
            self.gui_lqr_controller.set_enabled(False)
            self.gui_lqr_torque_nm = 0.0
            self.gui_lqr_positive_torque_nm = 0.0
            self.gui_lqr_negative_torque_nm = 0.0
            self.send_deng_payload_to_connected("LQR2,0.000000,0.000000,0", quiet=True)

    def on_pid_balance_toggled(self, checked: bool) -> None:
        if checked and self.relay_enable_box.isChecked():
            self.relay_enable_box.setChecked(False)
        if checked and self.fuzzy_enable_box.isChecked():
            self.fuzzy_enable_box.setChecked(False)
        self.reset_pid_balance_integral()
        if not checked:
            self.pid_balance_torque_nm = 0.0
            self.pid_balance_raw_torque_nm = 0.0
            self.pid_balance_positive_torque_nm = 0.0
            self.pid_balance_negative_torque_nm = 0.0
            self.send_deng_payload_to_connected("LQR2,0.000000,0.000000,0", quiet=True)

    def relay_lqr_to_deng(self) -> None:
        if hasattr(self, "fuzzy_enable_box") and self.fuzzy_enable_box.isChecked():
            return
        if hasattr(self, "pid_balance_enable_box") and self.pid_balance_enable_box.isChecked():
            return
        if not self.relay_enable_box.isChecked():
            self.gui_lqr_controller.set_enabled(False)
            self.gui_lqr_torque_nm = 0.0
            self.gui_lqr_positive_torque_nm = 0.0
            self.gui_lqr_negative_torque_nm = 0.0
            return
        now_ms = int(time.monotonic() * 1000)
        if now_ms - self.last_relay_ms < self.CONTROL_PERIOD_MS:
            return
        self.last_relay_ms = now_ms
        self.apply_gui_lqr_settings()
        leg_height = float(self.leg_height_slider.value())
        wheel_state = self.estimate_wheel_state()
        lqr_state, target_position_m, target_velocity_mps = self.selected_lqr_wheel_state(wheel_state)
        self.gui_lqr_torque_nm = self.gui_lqr_controller.compute_balance_torque_from_wheel_state(
            lqr_state.position_m,
            lqr_state.velocity_mps,
            self.latest_imu.pitch_deg,
            self.latest_imu_pitch_rate_dps,
            target_position_m,
            target_velocity_mps,
            self.target_pitch_input.value(),
            leg_height,
        )
        enabled = 1 if self.lqr_enable_box.isChecked() else 0
        positive, negative = self.split_controller_torque(self.gui_lqr_torque_nm, bool(enabled))
        self.gui_lqr_positive_torque_nm = positive
        self.gui_lqr_negative_torque_nm = negative
        if self.motor_routing() == ROUTING_SINGLE_DENG_M1_M2:
            if self.deng_a_connected:
                self.send_deng_payload(f"LQR2,{positive:.6f},{negative:.6f},{enabled}", side="a", quiet=True)
            if self.deng_b_connected:
                self.send_deng_payload("LQR2,0.000000,0.000000,0", side="b", quiet=True)
        else:
            if self.deng_a_connected:
                self.send_deng_payload(f"LQR2,{positive:.6f},0.000000,{enabled}", side="a", quiet=True)
            if self.deng_b_connected:
                self.send_deng_payload(f"LQR2,{negative:.6f},0.000000,{enabled}", side="b", quiet=True)

    def apply_gui_lqr_settings(self) -> None:
        self.gui_lqr_controller.set_enabled(self.lqr_enable_box.isChecked())
        self.gui_lqr_controller.set_torque_limit(self.torque_limit_input.value())
        self.gui_lqr_controller.set_output_scale(self.output_scale_input.value())
        self.gui_lqr_controller.set_weights(
            self.q1_input.value(),
            self.q2_input.value(),
            self.q3_input.value(),
            self.q4_input.value(),
            self.r_input.value(),
        )

    def apply_fuzzy_settings(self) -> None:
        self.fuzzy_controller.set_enabled(self.fuzzy_enable_box.isChecked())
        self.fuzzy_controller.set_torque_limit(self.fuzzy_torque_limit_input.value())
        self.fuzzy_controller.set_output_scale(self.fuzzy_output_scale_input.value())
        self.fuzzy_controller.set_polarity(float(self.fuzzy_polarity_combo.currentData()))
        self.fuzzy_controller.set_input_ranges(
            self.fuzzy_angle_range_input.value(),
            self.fuzzy_rate_range_input.value(),
        )
        self.fuzzy_controller.set_deadband(
            self.fuzzy_deadband_angle_input.value(),
            self.fuzzy_deadband_rate_input.value(),
        )
        self.fuzzy_controller.set_safety_pitch(self.fuzzy_safe_pitch_input.value())
        self.fuzzy_controller.set_height_gain(self.fuzzy_height_gain_input.value())

    def relay_fuzzy_to_deng(self) -> None:
        if hasattr(self, "pid_balance_enable_box") and self.pid_balance_enable_box.isChecked():
            return
        if not self.fuzzy_enable_box.isChecked():
            self.fuzzy_controller.set_enabled(False)
            self.fuzzy_torque_nm = 0.0
            self.fuzzy_positive_torque_nm = 0.0
            self.fuzzy_negative_torque_nm = 0.0
            return
        now_ms = int(time.monotonic() * 1000)
        if now_ms - self.last_relay_ms < self.CONTROL_PERIOD_MS:
            return
        self.last_relay_ms = now_ms
        self.apply_fuzzy_settings()
        leg_height = float(self.leg_height_slider.value())
        self.fuzzy_torque_nm = self.fuzzy_controller.compute_balance_torque(
            self.latest_imu.pitch_deg,
            self.latest_imu_pitch_rate_dps,
            self.target_pitch_input.value(),
            leg_height,
        )
        enabled = 1 if not self.fuzzy_controller.last_debug.safety_limited else 0
        if self.fuzzy_output_mode() == "velocity":
            self.fuzzy_positive_torque_nm, self.fuzzy_negative_torque_nm = self.split_controller_velocity(
                self.fuzzy_torque_nm,
                bool(enabled),
            )
            self.send_fuzzy_velocity_output(enabled)
        else:
            self.fuzzy_positive_torque_nm, self.fuzzy_negative_torque_nm = self.split_controller_torque(
                self.fuzzy_torque_nm,
                bool(enabled),
            )
            self.send_fuzzy_torque_output(enabled)

    def send_fuzzy_torque_output(self, enabled: int) -> None:
        if self.motor_routing() == ROUTING_SINGLE_DENG_M1_M2:
            if self.deng_a_connected:
                self.send_deng_payload(
                    f"LQR2,{self.fuzzy_positive_torque_nm:.6f},{self.fuzzy_negative_torque_nm:.6f},{enabled}",
                    side="a",
                    quiet=True,
                )
            if self.deng_b_connected:
                self.send_deng_payload("LQR2,0.000000,0.000000,0", side="b", quiet=True)
            return
        if self.deng_a_connected:
            self.send_deng_payload(
                f"LQR2,{self.fuzzy_positive_torque_nm:.6f},0.000000,{enabled}",
                side="a",
                quiet=True,
            )
        if self.deng_b_connected:
            self.send_deng_payload(
                f"LQR2,{self.fuzzy_negative_torque_nm:.6f},0.000000,{enabled}",
                side="b",
                quiet=True,
            )

    def send_fuzzy_velocity_output(self, enabled: int) -> None:
        if self.motor_routing() == ROUTING_SINGLE_DENG_M1_M2:
            if self.deng_a_connected:
                self.send_deng_payload(
                    "SET2,"
                    f"velocity,{self.fuzzy_positive_torque_nm:.6f},{enabled},"
                    f"velocity,{self.fuzzy_negative_torque_nm:.6f},{enabled}",
                    side="a",
                    quiet=True,
                )
            if self.deng_b_connected:
                self.send_deng_payload(
                    "SET2,velocity,0.000000,0,velocity,0.000000,0",
                    side="b",
                    quiet=True,
                )
            return
        if self.deng_a_connected:
            self.send_deng_payload(
                "SET2,"
                f"velocity,{self.fuzzy_positive_torque_nm:.6f},{enabled},"
                "velocity,0.000000,0",
                side="a",
                quiet=True,
            )
        if self.deng_b_connected:
            self.send_deng_payload(
                "SET2,"
                f"velocity,{self.fuzzy_negative_torque_nm:.6f},{enabled},"
                "velocity,0.000000,0",
                side="b",
                quiet=True,
            )

    def relay_pid_balance_to_deng(self) -> None:
        if self.relay_enable_box.isChecked() or self.fuzzy_enable_box.isChecked():
            return
        if not self.pid_balance_enable_box.isChecked():
            self.update_pid_balance_torque(enabled=False)
            return
        now_ms = int(time.monotonic() * 1000)
        if now_ms - self.last_relay_ms < self.CONTROL_PERIOD_MS:
            return
        self.last_relay_ms = now_ms
        self.update_pid_balance_torque(enabled=True)
        enabled = 1 if self.pid_balance_torque_limit_input.value() > 0.0 else 0
        self.pid_balance_positive_torque_nm, self.pid_balance_negative_torque_nm = self.split_controller_torque(
            self.pid_balance_torque_nm,
            bool(enabled),
        )

        if self.motor_routing() == ROUTING_SINGLE_DENG_M1_M2:
            if self.deng_a_connected:
                self.send_deng_payload(
                    f"LQR2,{self.pid_balance_positive_torque_nm:.6f},{self.pid_balance_negative_torque_nm:.6f},{enabled}",
                    side="a",
                    quiet=True,
                )
            if self.deng_b_connected:
                self.send_deng_payload("LQR2,0.000000,0.000000,0", side="b", quiet=True)
        else:
            if self.deng_a_connected:
                self.send_deng_payload(
                    f"LQR2,{self.pid_balance_positive_torque_nm:.6f},0.000000,{enabled}",
                    side="a",
                    quiet=True,
                )
            if self.deng_b_connected:
                self.send_deng_payload(
                    f"LQR2,{self.pid_balance_negative_torque_nm:.6f},0.000000,{enabled}",
                    side="b",
                    quiet=True,
                )

    def update_pid_balance_torque(self, enabled: bool) -> None:
        now = time.monotonic()
        dt_s = self.CONTROL_PERIOD_MS / 1000.0
        if self.pid_balance_prev_time is not None:
            dt_s = max(0.001, min(0.25, now - self.pid_balance_prev_time))
        self.pid_balance_prev_time = now

        self.pid_balance_error_deg = self.latest_imu.pitch_deg - self.target_pitch_input.value()
        if not enabled:
            self.pid_balance_integral = 0.0
            self.pid_balance_raw_torque_nm = 0.0
            self.pid_balance_torque_nm = 0.0
            self.pid_balance_positive_torque_nm = 0.0
            self.pid_balance_negative_torque_nm = 0.0
            return

        integral_limit = abs(self.pid_balance_integral_limit_input.value())
        self.pid_balance_integral += self.pid_balance_error_deg * dt_s
        self.pid_balance_integral = self.clamp(self.pid_balance_integral, -integral_limit, integral_limit)

        polarity = float(self.pid_balance_polarity_combo.currentData())
        self.pid_balance_raw_torque_nm = (
            polarity
            * self.pid_balance_output_scale_input.value()
            * (
                self.pid_balance_kp_input.value() * self.pid_balance_error_deg
                + self.pid_balance_ki_input.value() * self.pid_balance_integral
                + self.pid_balance_kd_input.value() * self.latest_imu_pitch_rate_dps
            )
        )
        limit = abs(self.pid_balance_torque_limit_input.value())
        self.pid_balance_torque_nm = self.clamp(self.pid_balance_raw_torque_nm, -limit, limit)

    def reset_pid_balance_integral(self) -> None:
        self.pid_balance_integral = 0.0
        self.pid_balance_prev_time = None

    def send_manual_bldc_control(self) -> None:
        self.disable_balance_relays_for_pid_test()
        enabled = 1 if self.manual_enable_box.isChecked() else 0
        mode = self.manual_mode_combo.currentData() or "torque"
        m1_target = self.manual_m1_target.value()
        m2_target = self.manual_m2_target.value()
        if mode == "position":
            m1_target = math.radians(m1_target)
            m2_target = math.radians(m2_target)
        self.send_deng_payload_to_connected(
            f"SET2,{mode},{m1_target:.6f},{enabled},{mode},{m2_target:.6f},{enabled}"
        )

    def stop_manual_bldc_control(self) -> None:
        self.disable_balance_relays_for_pid_test()
        self.send_deng_payload_to_connected("SET2,torque,0.000000,0,torque,0.000000,0")

    def disable_balance_relays_for_pid_test(self) -> None:
        self.relay_enable_box.setChecked(False)
        self.fuzzy_enable_box.setChecked(False)
        self.pid_balance_enable_box.setChecked(False)

    def foc_diagnostic_side(self) -> str:
        return str(self.foc_diag_board_combo.currentData() or "a")

    def foc_diagnostic_motor(self) -> str:
        return str(self.foc_diag_motor_combo.currentData() or "m1")

    def prepare_foc_diagnostic_test(self) -> None:
        self.disable_balance_relays_for_pid_test()
        self.manual_enable_box.setChecked(False)
        self.reset_foc_diagnostic_plot()

    def send_foc_diagnostic_command(self, payload: str) -> None:
        self.send_deng_payload(payload, side=self.foc_diagnostic_side())

    def apply_foc_diagnostic_safe(self) -> None:
        self.send_foc_diagnostic_command(
            f"SAFE,{self.foc_diag_voltage_input.value():.3f},"
            f"{self.foc_diag_current_input.value():.3f},"
            f"{self.foc_diag_watchdog_input.value()}"
        )

    def apply_foc_diagnostic_directions(self) -> None:
        signs = [
            int(self.foc_diag_sign_combos[name].currentData())
            for name in ("commutation", "encoder", "velocity", "current", "actuator")
        ]
        self.send_foc_diagnostic_command(
            f"DIR,{self.foc_diagnostic_motor()}," + ",".join(str(sign) for sign in signs)
        )

    def start_foc_diagnostic_align(self) -> None:
        self.prepare_foc_diagnostic_test()
        self.send_foc_diagnostic_command(
            f"ALIGN,{self.foc_diagnostic_motor()},"
            f"{self.foc_diag_align_voltage.value():.3f},"
            f"{self.foc_diag_align_duration.value()}"
        )

    def start_foc_diagnostic_iq(self, sign: float) -> None:
        self.prepare_foc_diagnostic_test()
        target = sign * self.foc_diag_iq_input.value()
        self.send_foc_diagnostic_command(
            f"TESTIQ,{self.foc_diagnostic_motor()},{target:.4f},"
            f"{self.foc_diag_iq_duration.value()}"
        )

    def start_foc_diagnostic_open(self, direction: str) -> None:
        self.prepare_foc_diagnostic_test()
        self.send_foc_diagnostic_command(
            f"OPEN,{self.foc_diagnostic_motor()},{direction},"
            f"{self.foc_diag_open_voltage.value():.3f},"
            f"{self.foc_diag_open_speed.value():.3f},"
            f"{self.foc_diag_open_duration.value()}"
        )

    def start_foc_diagnostic_phase(self, phase: str) -> None:
        self.prepare_foc_diagnostic_test()
        self.send_foc_diagnostic_command(
            f"PHASE,{self.foc_diagnostic_motor()},{phase},"
            f"{self.foc_diag_phase_voltage.value():.3f},"
            f"{self.foc_diag_phase_duration.value()}"
        )

    def reset_foc_diagnostic_plot(self, *_unused) -> None:
        for history in self.foc_diag_history.values():
            history.clear()
        if hasattr(self, "foc_diag_board_combo"):
            self.foc_diag_start_ms[self.foc_diagnostic_side()] = None
        if hasattr(self, "foc_diag_curves"):
            for curve in self.foc_diag_curves.values():
                curve.setData([], [])

    def handle_foc_diagnostic_sample(
        self,
        side: str,
        sample: FocDiagnosticTelemetry,
    ) -> None:
        self.foc_diag_samples[side] = sample
        if side != self.foc_diagnostic_side():
            return
        if self.foc_diag_start_ms[side] is None:
            self.foc_diag_start_ms[side] = sample.timestamp_ms
        elapsed = (sample.timestamp_ms - int(self.foc_diag_start_ms[side])) * 0.001
        motor = self.foc_diagnostic_motor()
        if motor == "m1":
            values = (
                sample.m1_angle_deg,
                sample.m1_velocity_radps,
                sample.m1_iq_a,
                sample.m1_iq_target_a,
                sample.m1_uq_v,
            )
        else:
            values = (
                sample.m2_angle_deg,
                sample.m2_velocity_radps,
                sample.m2_iq_a,
                sample.m2_iq_target_a,
                sample.m2_uq_v,
            )
        self.foc_diag_history["Time"].append(elapsed)
        for name, value in zip(
            ("Angle", "Velocity", "Iq Measured", "Iq Target", "Uq Command"),
            values,
        ):
            self.foc_diag_history[name].append(value)
        self.update_foc_diagnostic_status(sample)

    def update_foc_diagnostic_status(self, sample: FocDiagnosticTelemetry) -> None:
        values = {
            "Test": sample.test,
            "Active Motor": sample.active_motor,
            "Enabled": "YES" if sample.enabled else "NO",
            "M1 Angle": f"{sample.m1_angle_deg:.2f} deg",
            "M1 Velocity": f"{sample.m1_velocity_radps:.3f} rad/s",
            "M1 Iq / Target": f"{sample.m1_iq_a:.4f} / {sample.m1_iq_target_a:.4f} A",
            "M1 Uq": f"{sample.m1_uq_v:.3f} V",
            "M2 Angle": f"{sample.m2_angle_deg:.2f} deg",
            "M2 Velocity": f"{sample.m2_velocity_radps:.3f} rad/s",
            "M2 Iq / Target": f"{sample.m2_iq_a:.4f} / {sample.m2_iq_target_a:.4f} A",
            "M2 Uq": f"{sample.m2_uq_v:.3f} V",
            "Zero Electrical": f"{sample.zero_electrical_angle_rad:.4f} rad",
            "Fault": "ESTOP" if sample.estop else ("Uq SAT" if sample.saturated else "NONE"),
            "Limits": f"{sample.voltage_limit_v:.2f} V / {sample.current_limit_a:.2f} A",
        }
        for name, value in values.items():
            self.foc_diag_status_labels[name].setText(value)

    def render_foc_diagnostic_result(self, result: FocDiagnosticResult) -> None:
        ratio_text = "---"
        if result.test == "iq" and abs(result.target) > 1e-6:
            ratio_text = f"{result.average_iq_a / result.target:.3f}"
        assessment = "Result recorded."
        if result.reason != "done":
            assessment = f"Test stopped: {result.reason}."
        elif result.test == "iq" and abs(result.target) > 1e-6:
            ratio = result.average_iq_a / result.target
            if ratio < -0.3:
                assessment = "Iq feedback sign is opposite to the target. Check Current DIR and phase mapping."
            elif abs(ratio) < 0.25:
                assessment = "Iq response is too small. Check current offsets, phase mapping, and voltage limit."
            elif result.saturated:
                assessment = "Iq direction is plausible, but Uq reached the voltage limit."
            else:
                assessment = "Iq feedback follows the requested direction."
        elif result.test == "open" and abs(result.angle_delta_rad) < 0.05:
            assessment = "Open-loop motion is very small. Check phase wiring and commutation direction."
        self.foc_diag_result_text.setPlainText(
            f"{result.test.upper()} / {result.motor} / {result.reason}\n"
            f"target={result.target:.4f}, avg Iq={result.average_iq_a:.4f} A, "
            f"avg Uq={result.average_uq_v:.4f} V, ratio={ratio_text}\n"
            f"angle delta={result.angle_delta_rad:.4f} rad, "
            f"avg velocity={result.average_velocity_radps:.4f} rad/s, "
            f"saturated={'YES' if result.saturated else 'NO'}\n"
            f"{assessment}"
        )

    def refresh_foc_diagnostic_plot(self) -> None:
        times = list(self.foc_diag_history["Time"])
        for name, curve in self.foc_diag_curves.items():
            curve.setData(times, list(self.foc_diag_history[name]))
        if times:
            self.foc_diag_plot.setXRange(
                max(0.0, times[-1] - 15.0),
                max(15.0, times[-1]),
                padding=0.0,
            )

    def save_foc_diagnostic_csv(self) -> None:
        if not self.foc_diag_history["Time"]:
            QMessageBox.information(self, "No data", "No FOC diagnostic telemetry has been received.")
            return
        filename, _ = QFileDialog.getSaveFileName(
            self,
            "Save FOC diagnostic data",
            "foc_diagnostic_snapshot.csv",
            "CSV Files (*.csv)",
        )
        if not filename:
            return
        columns = ("Time", "Angle", "Velocity", "Iq Measured", "Iq Target", "Uq Command")
        with open(filename, "w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(("board", "motor", *columns))
            for row in zip(*(self.foc_diag_history[name] for name in columns)):
                writer.writerow((self.foc_diagnostic_side(), self.foc_diagnostic_motor(), *row))
        self.append_log(f"Saved FOC diagnostic snapshot: {filename}")

    def update_manual_bldc_units(self) -> None:
        mode = self.manual_mode_combo.currentData() or "torque"
        units = {
            "torque": ("Nm", 0.01, 4),
            "velocity": ("rad/s", 0.5, 3),
            "position": ("deg", 1.0, 3),
        }
        unit, step, decimals = units[mode]
        for spin in (self.manual_m1_target, self.manual_m2_target):
            spin.setSingleStep(step)
            spin.setDecimals(decimals)
        self.manual_m1_unit_label.setText(unit)
        self.manual_m2_unit_label.setText(unit)
        self.refresh_manual_current_labels()

    def refresh_manual_current_labels(self, *_unused) -> None:
        if not hasattr(self, "manual_m1_current_label"):
            return
        mode = self.manual_mode_combo.currentData() or "torque"
        if mode != "torque":
            self.manual_m1_current_label.setText("Iq ---")
            self.manual_m2_current_label.setText("Iq ---")
            return
        kt = max(self.kt_input.value(), 0.001) if hasattr(self, "kt_input") else 0.0955
        current_limit = abs(self.current_limit_input.value()) if hasattr(self, "current_limit_input") else 3.0
        self.manual_m1_current_label.setText(self.format_torque_current(self.manual_m1_target.value(), kt, current_limit))
        self.manual_m2_current_label.setText(self.format_torque_current(self.manual_m2_target.value(), kt, current_limit))

    @staticmethod
    def format_torque_current(torque_nm: float, kt_nm_per_a: float, current_limit_a: float) -> str:
        raw_iq = float(torque_nm) / max(float(kt_nm_per_a), 0.001)
        limited_iq = IntegratedRobotControlGUI.clamp(raw_iq, -abs(current_limit_a), abs(current_limit_a))
        suffix = " lim" if abs(raw_iq - limited_iq) > 1e-6 else ""
        return f"Iq {limited_iq:.3f} A{suffix}"

    def send_safe(self) -> None:
        self.send_deng_payload_to_connected(
            "SAFE,"
            f"{self.voltage_limit_input.value():.6f},"
            f"{self.current_limit_input.value():.6f},"
            f"{self.velocity_limit_input.value():.6f},"
            f"{int(self.watchdog_input.value())}"
        )

    def send_pid(self, command: str) -> None:
        values = self.pid_inputs[command]
        if command == "PIDQ":
            payload = (
                f"PIDQ,{values['Kp'].value():.6f},{values['Ki'].value():.6f},"
                f"{values['Kd'].value():.6f},{values['Ramp'].value():.6f}"
            )
        else:
            payload = (
                f"{command},{values['Kp'].value():.6f},{values['Ki'].value():.6f},"
                f"{values['Kd'].value():.6f},{values['Ramp'].value():.6f},{values['Limit'].value():.6f}"
            )
        self.send_deng_payload_to_connected(payload)

    def query_deng_config(self) -> None:
        self.send_deng_payload_to_connected("CFG?")
        self.send_deng_payload_to_connected("PID?")

    def estop(self) -> None:
        self.lqr_enable_box.setChecked(False)
        self.relay_enable_box.setChecked(False)
        self.fuzzy_enable_box.setChecked(False)
        self.pid_balance_enable_box.setChecked(False)
        self.send_deng_payload_to_connected("LQR2,0.000000,0.000000,0")
        self.send_deng_payload_to_connected("ESTOP")

    def send_heartbeats(self) -> None:
        for side in ("a", "b"):
            if self.deng_connected(side):
                try:
                    self.deng_client(side).send_payload("PING")
                except serial.SerialException:
                    self.safe_deng_disconnect(side)

    def send_deng_payload_to_connected(self, payload: str, quiet: bool = False) -> None:
        sent = False
        for side in ("a", "b"):
            if self.deng_connected(side):
                self.send_deng_payload(payload, side=side, quiet=True)
                sent = True
        if not sent and not quiet:
            QMessageBox.information(self, "Not connected", "Connect to at least one DengFOC first.")

    def send_deng_payload(self, payload: str, side: str, quiet: bool = False) -> None:
        if not self.deng_connected(side):
            if not quiet:
                QMessageBox.information(self, "Not connected", f"Connect DengFOC {side.upper()} first.")
            return
        self.deng_client(side).send_payload(payload)
        if side == "a":
            self.last_deng_a_command = payload
        else:
            self.last_deng_b_command = payload
        if not quiet:
            self.append_log(f"Deng {side.upper()} > {payload}")

    def handle_deng_sample(self, side: str, sample: DengTelemetry) -> None:
        self.set_last_deng_sample(side, sample)
        row = self.deng_csv_row(side, sample)
        self.deng_record_rows.append(row)
        if self.is_deng_recording and self.deng_csv_writer:
            self.deng_csv_writer.writerow(row)

    def handle_deng_message(self, payload: str, side: str) -> None:
        if payload == "OK,pong":
            return
        diagnostic_sample = FocDiagnosticTelemetry.from_payload(payload)
        if diagnostic_sample:
            self.handle_foc_diagnostic_sample(side, diagnostic_sample)
            return
        diagnostic_result = FocDiagnosticResult.from_payload(payload)
        if diagnostic_result:
            self.foc_diag_results[side] = diagnostic_result
            if side == self.foc_diagnostic_side():
                self.render_foc_diagnostic_result(diagnostic_result)
            self.append_log(f"Deng {side.upper()}: {payload}")
            return
        if payload.startswith("CFG,"):
            self.apply_cfg_payload(payload)
        elif payload.startswith("PID,"):
            self.apply_pid_payload(payload)
        if payload.startswith("OK,") or payload.startswith("ERR,"):
            self.set_last_deng_ack(side, payload)
            return
        if payload.startswith(("CFG,", "PID,")):
            self.set_last_deng_ack(side, payload)
        self.append_log(f"Deng {side.upper()}: {payload}")

    def toggle_deng_recording(self) -> None:
        if not self.is_deng_recording:
            filename = f"dengfoc_pid_test_log_{dt.datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
            self.deng_csv_file = open(filename, "w", newline="", encoding="utf-8")
            self.deng_csv_writer = csv.writer(self.deng_csv_file)
            self.deng_csv_writer.writerow(self.deng_csv_header())
            self.is_deng_recording = True
            self.deng_record_button.setText("Stop Deng CSV")
            self.append_log(f"Recording DengFOC PID test data to {filename}")
            return
        self.is_deng_recording = False
        self.deng_record_button.setText("Start Deng CSV")
        if self.deng_csv_file:
            self.deng_csv_file.close()
        self.deng_csv_file = None
        self.deng_csv_writer = None
        self.append_log("DengFOC PID test recording stopped")

    def save_deng_snapshot_csv(self) -> None:
        if not self.deng_record_rows:
            QMessageBox.information(self, "No data", "No DengFOC telemetry has been received yet.")
            return
        filename, _ = QFileDialog.getSaveFileName(
            self,
            "Save DengFOC PID test data",
            "dengfoc_pid_test_snapshot.csv",
            "CSV Files (*.csv)",
        )
        if not filename:
            return
        with open(filename, "w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(self.deng_csv_header())
            writer.writerows(self.deng_record_rows)
        self.append_log(f"Saved DengFOC PID test snapshot: {filename}")

    @staticmethod
    def deng_csv_header() -> list[str]:
        motor_fields = (
            "mode",
            "target",
            "angle",
            "electrical_angle",
            "velocity",
            "measured_torque",
            "iq_measured",
            "iq_target",
            "uq_command",
            "kt",
            "torque_limit",
        )
        return [
            "wall_time",
            "side",
            "timestamp_ms",
            "enabled",
            *[f"m1_{name}" for name in motor_fields],
            *[f"m2_{name}" for name in motor_fields],
            "voltage_limit",
            "current_limit",
            "faults",
        ]

    def deng_csv_row(self, side: str, sample: DengTelemetry) -> list[object]:
        return [
            dt.datetime.now().isoformat(timespec="milliseconds"),
            side.upper(),
            sample.timestamp_ms,
            int(sample.enabled),
            *self.deng_motor_csv_values(sample.m1),
            *self.deng_motor_csv_values(sample.m2),
            sample.voltage_limit_v,
            sample.current_limit_a,
            sample.faults,
        ]

    @staticmethod
    def deng_motor_csv_values(motor: MotorTelemetry) -> list[object]:
        return [
            motor.mode,
            motor.command_torque_nm,
            motor.angle_rad,
            motor.electrical_angle_rad,
            motor.velocity_radps,
            motor.measured_torque_nm,
            motor.iq_measured_a,
            motor.iq_target_a,
            motor.uq_command_v,
            motor.kt_nm_per_a,
            motor.torque_limit_nm,
        ]

    def apply_cfg_payload(self, payload: str) -> None:
        fields = payload.split(",")
        if len(fields) < 8:
            return
        try:
            self.voltage_limit_input.setValue(float(fields[1]))
            self.current_limit_input.setValue(float(fields[2]))
            self.velocity_limit_input.setValue(float(fields[3]))
            self.watchdog_input.setValue(float(fields[4]))
            self.kt_input.setValue(float(fields[5]))
        except ValueError:
            return

    def apply_pid_payload(self, payload: str) -> None:
        fields = payload.split(",")
        if len(fields) < 16:
            return
        try:
            values = [float(value) for value in fields[1:16]]
        except ValueError:
            return
        for command, offset in (("PIDQ", 0), ("PIDV", 5), ("PIDP", 10)):
            self.pid_inputs[command]["Kp"].setValue(values[offset])
            self.pid_inputs[command]["Ki"].setValue(values[offset + 1])
            self.pid_inputs[command]["Kd"].setValue(values[offset + 2])
            self.pid_inputs[command]["Ramp"].setValue(values[offset + 3])
            self.pid_inputs[command]["Limit"].setValue(values[offset + 4])

    def append_plot_samples(self) -> None:
        wheel_state = self.estimate_wheel_state()
        elapsed = time.monotonic() - self.start_time
        self.time_history.append(elapsed)
        self.imu_history["Roll"].append(self.latest_imu.roll_deg)
        self.imu_history["Pitch"].append(self.latest_imu.pitch_deg)
        self.imu_history["Yaw"].append(self.latest_imu.yaw_deg)
        self.lqr_history["Pitch"].append(self.latest_imu.pitch_deg)
        self.lqr_history["Pitch Rate"].append(self.latest_imu_pitch_rate_dps)
        self.lqr_history["Target Pitch"].append(self.target_pitch_input.value())
        self.lqr_history["Wheel Position"].append(wheel_state.position_m)
        self.lqr_history["Wheel Velocity"].append(wheel_state.velocity_mps)
        self.lqr_history["Target Position"].append(self.target_position_input.value())
        self.lqr_history["Raw Torque"].append(self.latest_lqr.raw_torque_nm)
        self.lqr_history["Limited Torque"].append(self.latest_lqr.limited_torque_nm)
        self.lqr_history["GUI LQR Torque"].append(self.gui_lqr_torque_nm)
        self.lqr_history["Fuzzy Output"].append(self.fuzzy_torque_nm)
        self.lqr_history["PID Torque"].append(self.pid_balance_torque_nm)
        positive, negative = self.current_routed_motor_torque()
        self.motor_history["Positive Torque"].append(positive)
        self.motor_history["Negative Torque"].append(negative)
        plotted_lqr_torque = (
            self.gui_lqr_torque_nm
            if self.relay_enable_box.isChecked()
            else self.pid_balance_torque_nm
            if self.pid_balance_enable_box.isChecked()
            else self.latest_lqr.limited_torque_nm
        )
        self.motor_history["Controller Torque"].append(plotted_lqr_torque)
        if self.is_recording and self.csv_writer:
            self.csv_writer.writerow(
                [
                    dt.datetime.now().isoformat(timespec="milliseconds"),
                    elapsed,
                    self.latest_imu.roll_deg,
                    self.latest_imu.pitch_deg,
                    self.latest_imu.yaw_deg,
                    self.latest_imu.motion_state,
                    self.latest_imu.mag_interference,
                    self.latest_imu_pitch_rate_dps,
                    self.target_pitch_input.value(),
                    self.target_position_input.value(),
                    self.target_velocity_input.value(),
                    self.leg_height_slider.value(),
                    self.wheel_ik_x_input.value(),
                    self.wheel_radius_input.value(),
                    self.positive_odom_combo.currentData(),
                    self.negative_odom_combo.currentData(),
                    int(wheel_state.valid),
                    wheel_state.source,
                    wheel_state.position_m,
                    wheel_state.velocity_mps,
                    int(self.latest_lqr.enabled),
                    self.latest_lqr.raw_torque_nm,
                    self.latest_lqr.limited_torque_nm,
                    self.latest_lqr.k1,
                    self.latest_lqr.k2,
                    self.latest_lqr.k3,
                    self.latest_lqr.k4,
                    self.latest_imu_pitch_rate_dps,
                    self.gui_lqr_torque_nm,
                    self.gui_lqr_controller.last_raw_torque,
                    self.fuzzy_output_mode(),
                    self.fuzzy_torque_nm,
                    self.fuzzy_controller.last_raw_torque,
                    int(self.fuzzy_enable_box.isChecked()),
                    self.pid_balance_torque_nm,
                    self.pid_balance_raw_torque_nm,
                    self.pid_balance_error_deg,
                    self.pid_balance_integral,
                    int(self.pid_balance_enable_box.isChecked()),
                ]
            )

    def refresh_plots(self) -> None:
        times = list(self.time_history)
        for name, curve in self.imu_curves.items():
            curve.setData(times, list(self.imu_history[name]))
        for name, curve in self.lqr_curves.items():
            curve.setData(times, list(self.lqr_history[name]))
        for name, curve in self.fuzzy_curves.items():
            curve.setData(times, list(self.lqr_history[name]))
        for name, curve in self.pid_balance_curves.items():
            curve.setData(times, list(self.lqr_history[name]))
        for name, curve in self.motor_curves.items():
            curve.setData(times, list(self.motor_history[name]))
        if times:
            start = max(0.0, times[-1] - 15.0)
            end = max(15.0, times[-1])
            self.imu_plot.setXRange(start, end, padding=0.0)
            self.lqr_plot.setXRange(start, end, padding=0.0)
            self.fuzzy_plot.setXRange(start, end, padding=0.0)
            self.pid_balance_plot.setXRange(start, end, padding=0.0)
            self.motor_plot.setXRange(start, end, padding=0.0)
        self.refresh_foc_diagnostic_plot()

    def update_status_panels(self) -> None:
        imu_state = "STILL" if self.latest_imu.motion_state == 0 else "MOVING"
        mag_state = "YES" if self.latest_imu.mag_interference else "NO"
        self.imu_status_labels["Roll"].setText(f"{self.latest_imu.roll_deg:.3f} deg")
        self.imu_status_labels["Pitch"].setText(f"{self.latest_imu.pitch_deg:.3f} deg")
        self.imu_status_labels["Yaw"].setText(f"{self.latest_imu.yaw_deg:.3f} deg")
        self.imu_status_labels["Motion State"].setText(imu_state)
        self.imu_status_labels["Mag Interference"].setText(mag_state)
        wheel_state = self.current_wheel_state

        values = {
            "Pitch": f"{self.latest_imu.pitch_deg:.3f} deg",
            "Pitch Rate": f"{self.latest_imu_pitch_rate_dps:.3f} dps",
            "Target Pitch": f"{self.target_pitch_input.value():.3f} deg",
            "Target Position": f"{self.target_position_input.value():.3f} m",
            "Target Velocity": f"{self.target_velocity_input.value():.3f} m/s",
            "Wheel Position": f"{wheel_state.position_m:.4f} m",
            "Wheel Velocity": f"{wheel_state.velocity_mps:.4f} m/s",
            "Wheel Source": wheel_state.source,
            "LQR Wheel State": (
                "Wheel telemetry"
                if self.lqr_use_wheel_state_box.isChecked() and wheel_state.valid
                else "Forced position=0, velocity=0"
            ),
            "Leg Height": f"{self.leg_height_slider.value():.2f} mm",
            "Wheel IK X": f"{self.wheel_ik_x_input.value():.2f} mm",
            "LQR Enabled": "YES" if self.latest_lqr.enabled else "NO",
            "Raw Torque": f"{self.latest_lqr.raw_torque_nm:.6f} Nm",
            "Limited Torque": f"{self.latest_lqr.limited_torque_nm:.6f} Nm",
            "K1": f"{self.latest_lqr.k1:.6f}",
            "K2": f"{self.latest_lqr.k2:.6f}",
            "K3": f"{self.latest_lqr.k3:.6f}",
            "K4": f"{self.latest_lqr.k4:.6f}",
            "IMU Pitch Rate": f"{self.latest_imu_pitch_rate_dps:.3f} dps",
            "GUI LQR Torque": f"{self.gui_lqr_torque_nm:.6f} Nm",
            "Last Servo Message": self.last_servo_message,
        }
        for name, value in values.items():
            self.lqr_status_labels[name].setText(value)

        positive_motor, negative_motor = self.current_motor_samples()
        fuzzy_values = {
            "Pitch": f"{self.latest_imu.pitch_deg:.3f} deg",
            "Pitch Rate": f"{self.latest_imu_pitch_rate_dps:.3f} dps",
            "Target Pitch": f"{self.target_pitch_input.value():.3f} deg",
            "Output Mode": "Torque" if self.fuzzy_output_mode() == "torque" else "Motor velocity",
            "Fuzzy Output": f"{self.fuzzy_torque_nm:.6f} {self.fuzzy_output_unit()}",
            "M1 Command": f"{self.fuzzy_positive_torque_nm:.6f} {self.fuzzy_motor_command_unit()}",
            "M2 Command": f"{self.fuzzy_negative_torque_nm:.6f} {self.fuzzy_motor_command_unit()}",
            "M1 Measured Velocity": (
                f"{positive_motor.velocity_radps:.6f} rad/s" if positive_motor else "---"
            ),
            "M2 Measured Velocity": (
                f"{negative_motor.velocity_radps:.6f} rad/s" if negative_motor else "---"
            ),
            "Fuzzy Out": self.format_fuzzy_status(),
            "Last Servo Message": self.last_servo_message,
        }
        for name, value in fuzzy_values.items():
            self.fuzzy_status_labels[name].setText(value)

        pid_values = {
            "Pitch": f"{self.latest_imu.pitch_deg:.3f} deg",
            "Pitch Rate": f"{self.latest_imu_pitch_rate_dps:.3f} dps",
            "Target Pitch": f"{self.target_pitch_input.value():.3f} deg",
            "PID Error": f"{self.pid_balance_error_deg:.3f} deg",
            "PID Integral": f"{self.pid_balance_integral:.3f} deg*s",
            "PID Raw Torque": f"{self.pid_balance_raw_torque_nm:.6f} Nm",
            "PID Torque": f"{self.pid_balance_torque_nm:.6f} Nm",
            "PID M1": f"{self.pid_balance_positive_torque_nm:.6f} Nm",
            "PID M2": f"{self.pid_balance_negative_torque_nm:.6f} Nm",
            "Last Servo Message": self.last_servo_message,
        }
        for name, value in pid_values.items():
            self.pid_balance_status_labels[name].setText(value)

        motor_values = {
            "Routing": "Single A M1/M2" if self.motor_routing() == ROUTING_SINGLE_DENG_M1_M2 else "Dual A/B M1",
            "Deng A": "Connected" if self.deng_a_connected else "Disconnected",
            "Deng B": "Connected" if self.deng_b_connected else "Disconnected",
            "Faults A/B": f"{self.format_faults(self.last_deng_a_sample.faults) if self.last_deng_a_sample else '---'} / {self.format_faults(self.last_deng_b_sample.faults) if self.last_deng_b_sample else '---'}",
            "Positive Cmd Torque": f"{positive_motor.command_torque_nm:.6f} Nm" if positive_motor else "---",
            "Negative Cmd Torque": f"{negative_motor.command_torque_nm:.6f} Nm" if negative_motor else "---",
            "Positive Measured Torque": f"{positive_motor.measured_torque_nm:.6f} Nm" if positive_motor else "---",
            "Negative Measured Torque": f"{negative_motor.measured_torque_nm:.6f} Nm" if negative_motor else "---",
            "Positive Iq Target": f"{positive_motor.iq_target_a:.6f} A" if positive_motor else "---",
            "Positive Iq Measured": f"{positive_motor.iq_measured_a:.6f} A" if positive_motor else "---",
            "Negative Iq Target": f"{negative_motor.iq_target_a:.6f} A" if negative_motor else "---",
            "Negative Iq Measured": f"{negative_motor.iq_measured_a:.6f} A" if negative_motor else "---",
            "Limits A": f"{self.last_deng_a_sample.voltage_limit_v:.2f} V / {self.last_deng_a_sample.current_limit_a:.2f} A" if self.last_deng_a_sample else "---",
            "Last Cmd A/B": f"{self.last_deng_a_command} / {self.last_deng_b_command}",
        }
        for name, value in motor_values.items():
            self.motor_status_labels[name].setText(value)

    def update_3d_model(self, roll: float, pitch: float, yaw: float) -> None:
        if getattr(self, "box_mesh", None) is None:
            return
        self.box_mesh.resetTransform()
        self.box_mesh.rotate(yaw, 0, 0, 1)
        self.box_mesh.rotate(pitch, 0, 1, 0)
        self.box_mesh.rotate(roll, 1, 0, 0)

    def toggle_recording(self) -> None:
        if not self.is_recording:
            filename = f"integrated_robot_log_{dt.datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
            self.csv_file = open(filename, "w", newline="", encoding="utf-8")
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(
                [
                    "wall_time",
                    "time_s",
                    "roll_deg",
                    "pitch_deg",
                    "yaw_deg",
                    "motion_state",
                    "mag_interference",
                    "pitch_rate_dps",
                    "target_pitch_deg",
                    "target_position_m",
                    "target_velocity_mps",
                    "leg_height_mm",
                    "wheel_ik_x_mm",
                    "wheel_radius_m",
                    "positive_odom_direction",
                    "negative_odom_direction",
                    "wheel_state_valid",
                    "wheel_state_source",
                    "wheel_position_m",
                    "wheel_velocity_mps",
                    "lqr_enabled",
                    "raw_torque_nm",
                    "limited_torque_nm",
                    "k1",
                    "k2",
                    "k3",
                    "k4",
                    "imu_pitch_rate_dps",
                    "gui_lqr_torque_nm",
                    "gui_lqr_raw_torque_nm",
                    "fuzzy_output_mode",
                    "fuzzy_output",
                    "fuzzy_raw_output",
                    "fuzzy_enabled",
                    "pid_balance_torque_nm",
                    "pid_balance_raw_torque_nm",
                    "pid_balance_error_deg",
                    "pid_balance_integral_deg_s",
                    "pid_balance_enabled",
                ]
            )
            self.is_recording = True
            self.record_button.setText("Stop CSV")
            self.append_log(f"Recording to {filename}")
            return
        self.is_recording = False
        self.record_button.setText("Start CSV")
        if self.csv_file:
            self.csv_file.close()
        self.csv_file = None
        self.csv_writer = None
        self.append_log("Recording stopped")

    def save_snapshot_csv(self) -> None:
        if not self.time_history:
            QMessageBox.information(self, "No data", "No samples have been recorded yet.")
            return
        filename, _ = QFileDialog.getSaveFileName(
            self,
            "Save current integrated data",
            "integrated_robot_snapshot.csv",
            "CSV Files (*.csv)",
        )
        if not filename:
            return
        with open(filename, "w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(["time_s", "roll", "pitch", "yaw", *self.lqr_history.keys(), *self.motor_history.keys()])
            rows = zip(
                list(self.time_history),
                list(self.imu_history["Roll"]),
                list(self.imu_history["Pitch"]),
                list(self.imu_history["Yaw"]),
                *[list(self.lqr_history[name]) for name in self.lqr_history],
                *[list(self.motor_history[name]) for name in self.motor_history],
            )
            writer.writerows(rows)
        self.append_log(f"Saved snapshot: {filename}")

    def current_motor_samples(self) -> tuple[MotorTelemetry | None, MotorTelemetry | None]:
        positive = self.last_deng_a_sample.m1 if self.last_deng_a_sample else None
        if self.motor_routing() == ROUTING_SINGLE_DENG_M1_M2:
            negative = self.last_deng_a_sample.m2 if self.last_deng_a_sample else None
        else:
            negative = self.last_deng_b_sample.m1 if self.last_deng_b_sample else None
        return positive, negative

    def current_routed_motor_torque(self) -> tuple[float, float]:
        if hasattr(self, "pid_balance_enable_box") and self.pid_balance_enable_box.isChecked():
            return (self.pid_balance_positive_torque_nm, self.pid_balance_negative_torque_nm)
        if (
            hasattr(self, "fuzzy_enable_box")
            and self.fuzzy_enable_box.isChecked()
            and self.fuzzy_output_mode() == "torque"
        ):
            return (self.fuzzy_positive_torque_nm, self.fuzzy_negative_torque_nm)
        if hasattr(self, "relay_enable_box") and self.relay_enable_box.isChecked():
            return (self.gui_lqr_positive_torque_nm, self.gui_lqr_negative_torque_nm)
        positive, negative = self.current_motor_samples()
        if positive or negative:
            return (
                positive.measured_torque_nm if positive else 0.0,
                negative.measured_torque_nm if negative else 0.0,
            )
        return self.split_controller_torque(self.latest_lqr.limited_torque_nm, True)

    def split_controller_torque(self, torque_nm: float, enabled: bool) -> tuple[float, float]:
        if not enabled:
            return 0.0, 0.0
        half_torque = 0.5 * float(torque_nm)
        return (
            self.apply_min_torque_comp(self.m1_direction() * half_torque),
            self.apply_min_torque_comp(self.m2_direction() * half_torque),
        )

    def split_controller_velocity(self, velocity_radps: float, enabled: bool) -> tuple[float, float]:
        if not enabled:
            return 0.0, 0.0
        return (
            self.m1_direction() * float(velocity_radps),
            self.m2_direction() * float(velocity_radps),
        )

    def apply_min_torque_comp(self, motor_torque_nm: float) -> float:
        min_comp = self.min_torque_comp_input.value() if hasattr(self, "min_torque_comp_input") else 0.0
        if abs(motor_torque_nm) <= 0.001 or min_comp <= 0.0:
            return motor_torque_nm
        return math.copysign(abs(motor_torque_nm) + min_comp, motor_torque_nm)

    def m1_direction(self) -> float:
        return float(self.m1_direction_combo.currentData())

    def m2_direction(self) -> float:
        return float(self.m2_direction_combo.currentData())

    def format_fuzzy_status(self) -> str:
        debug = self.fuzzy_controller.last_debug
        suffix = " SAFETY" if debug.safety_limited else ""
        return f"{debug.normalized_output:.3f}, err={debug.pitch_error_deg:.3f} deg, h={debug.height_scale:.3f}{suffix}"

    def fuzzy_output_unit(self) -> str:
        return "Nm" if self.fuzzy_output_mode() == "torque" else "rad/s"

    def fuzzy_motor_command_unit(self) -> str:
        return "Nm" if self.fuzzy_output_mode() == "torque" else "rad/s"

    def safe_deng_disconnect(self, side: str | None = None) -> None:
        sides = ("a", "b") if side is None else (side,)
        for target in sides:
            if self.deng_connected(target):
                try:
                    self.deng_client(target).send_payload("LQR2,0.000000,0.000000,0")
                    self.deng_client(target).send_payload("ESTOP")
                except serial.SerialException:
                    pass
            self.deng_client(target).close()
            self.set_deng_connected(target, False)
            self.deng_connect_button(target).setText(f"Connect {target.upper()}")
            self.set_status_label(self.deng_status_label(target), "Disconnected", False)

    def deng_client(self, side: str) -> DengFocClient:
        return self.deng_a_client if side == "a" else self.deng_b_client

    def deng_connected(self, side: str) -> bool:
        return self.deng_a_connected if side == "a" else self.deng_b_connected

    def set_deng_connected(self, side: str, connected: bool) -> None:
        if side == "a":
            self.deng_a_connected = connected
        else:
            self.deng_b_connected = connected

    def deng_port_combo(self, side: str) -> QComboBox:
        return self.deng_a_port_combo if side == "a" else self.deng_b_port_combo

    def deng_connect_button(self, side: str) -> QPushButton:
        return self.deng_a_connect_button if side == "a" else self.deng_b_connect_button

    def deng_status_label(self, side: str) -> QLabel:
        return self.deng_a_status_label if side == "a" else self.deng_b_status_label

    def motor_routing(self) -> str:
        return self.routing_combo.currentData() or ROUTING_SINGLE_DENG_M1_M2

    def set_last_deng_ack(self, side: str, message: str) -> None:
        if side == "a":
            self.last_deng_a_ack = message
        else:
            self.last_deng_b_ack = message

    def set_last_deng_sample(self, side: str, sample: DengTelemetry) -> None:
        if side == "a":
            self.last_deng_a_sample = sample
        else:
            self.last_deng_b_sample = sample

    def format_faults(self, fault_mask: int) -> str:
        if fault_mask == 0:
            return "NONE"
        active = [label for bit, label in self.FAULT_LABELS.items() if fault_mask & (1 << bit)]
        return "|".join(active) if active else str(fault_mask)

    def append_log(self, message: str) -> None:
        self.log.append(f"[{dt.datetime.now().strftime('%H:%M:%S')}] {message}")
        self.log.verticalScrollBar().setValue(self.log.verticalScrollBar().maximum())

    def build_height_slider(self, title: str) -> tuple[QSlider, QVBoxLayout]:
        slider = QSlider(Qt.Vertical)
        slider.setRange(70, 155)
        slider.setValue(70)
        slider.setSingleStep(1)
        slider.setPageStep(4)
        slider.setTickInterval(12)
        slider.setTickPosition(QSlider.TicksBothSides)
        slider.setFixedHeight(180)

        value_label = QLabel("70 mm", alignment=Qt.AlignCenter)
        value_label.setStyleSheet("font-family: Consolas; font-weight: bold;")
        slider.valueChanged.connect(lambda value: value_label.setText(f"{value} mm"))

        layout = QVBoxLayout()
        layout.addWidget(QLabel(title, alignment=Qt.AlignCenter))
        layout.addWidget(slider, alignment=Qt.AlignHCenter)
        layout.addWidget(value_label)
        return slider, layout

    def build_plot_channel_group(
        self,
        curves: dict[str, object],
        default_visible: set[str] | None = None,
    ) -> QGroupBox:
        group = QGroupBox("Plot Channels")
        layout = QGridLayout(group)
        boxes: dict[str, QCheckBox] = {}
        visible = set(curves) if default_visible is None else default_visible
        for index, name in enumerate(curves):
            box = QCheckBox(name)
            box.setChecked(name in visible)
            boxes[name] = box
            layout.addWidget(box, index // 4, index % 4)
        self.plot_channel_boxes.append((curves, boxes))
        for box in boxes.values():
            box.stateChanged.connect(lambda _state=0, c=curves, b=boxes: self.update_plot_curve_visibility(c, b))
        self.update_plot_curve_visibility(curves, boxes)
        return group

    @staticmethod
    def update_plot_curve_visibility(curves: dict[str, object], boxes: dict[str, QCheckBox]) -> None:
        for name, curve in curves.items():
            if boxes[name].isChecked():
                curve.show()
            else:
                curve.hide()

    @staticmethod
    def build_direction_combo(default_sign: float) -> QComboBox:
        combo = QComboBox()
        combo.addItem("+1", 1.0)
        combo.addItem("-1", -1.0)
        combo.setCurrentIndex(0 if default_sign >= 0.0 else 1)
        return combo

    @staticmethod
    def clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))

    @staticmethod
    def make_double_spin(minimum: float, maximum: float, value: float, step: float, decimals: int) -> QDoubleSpinBox:
        spin = QDoubleSpinBox()
        spin.setRange(minimum, maximum)
        spin.setValue(value)
        spin.setSingleStep(step)
        spin.setDecimals(decimals)
        return spin

    @staticmethod
    def make_value_label() -> QLabel:
        label = QLabel("---")
        label.setStyleSheet("font-family: Consolas; font-weight: bold;")
        return label

    @staticmethod
    def make_status_label(text: str, ok: bool) -> QLabel:
        label = QLabel()
        IntegratedRobotControlGUI.set_status_label(label, text, ok)
        return label

    @staticmethod
    def set_status_label(label: QLabel, text: str, ok: bool) -> None:
        label.setText(text)
        color = "#27ae60" if ok else "#c0392b"
        label.setStyleSheet(f"font-weight: bold; color: {color};")

    def closeEvent(self, event: QCloseEvent) -> None:
        if self.is_recording:
            self.toggle_recording()
        if self.is_deng_recording:
            self.toggle_deng_recording()
        self.safe_deng_disconnect()
        self.servo_client.close()
        event.accept()


def main() -> None:
    app = QApplication(sys.argv)
    pg.setConfigOptions(antialias=True)
    window = IntegratedRobotControlGUI()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
