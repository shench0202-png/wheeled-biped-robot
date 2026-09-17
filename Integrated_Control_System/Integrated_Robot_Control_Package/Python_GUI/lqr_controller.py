from __future__ import annotations

from dataclasses import dataclass

try:
    import numpy as np
    from scipy.linalg import solve_continuous_are
except Exception:
    np = None
    solve_continuous_are = None


DEG_TO_RAD = 0.017453292519943295
MIN_WEIGHT = 1.0e-9

MODEL_M_WHEEL = 0.15
MODEL_WHEEL_INERTIA = 0.00004
MODEL_M_BODY = 0.2222 + 0.1546 + 0.0463 + 0.025
MODEL_M_LEGS = 0.2
MODEL_MASS_REDUCTION = 0.222
MODEL_M_PENDULUM = MODEL_M_BODY + MODEL_M_LEGS - MODEL_MASS_REDUCTION
MODEL_GRAVITY = 9.81
MODEL_WHEEL_RADIUS = 0.03
MODEL_TRACK_WIDTH = 0.16
MODEL_YAW_INERTIA = 0.001
MODEL_BODY_INERTIA_SCALE = 1.0 / 3.0


@dataclass(frozen=True)
class LQRState:
    position_m: float
    velocity_mps: float
    pitch_rad: float
    pitch_rate_radps: float


@dataclass(frozen=True)
class LQRGain:
    k1: float
    k2: float
    k3: float
    k4: float


@dataclass(frozen=True)
class LQRWeights:
    q1: float = 1.0
    q2: float = 1.0
    q3: float = 8.0
    q4: float = 1.0
    r: float = 5.0


@dataclass(frozen=True)
class Segment:
    min_mm: float
    max_mm: float
    slope: float
    intercept: float


K1_SEGMENTS = (
    Segment(60.0, 84.0, 0.0000000000, -0.4472135955),
    Segment(84.0, 108.0, 0.0000000000, -0.4472135955),
    Segment(108.0, 132.0, -0.0000000000, -0.4472135955),
    Segment(132.0, 156.0, 0.0000000000, -0.4472135955),
)

K2_SEGMENTS = (
    Segment(60.0, 84.0, -0.0000970615, -0.6710113658),
    Segment(84.0, 108.0, -0.0000981736, -0.6709179520),
    Segment(108.0, 132.0, -0.0000992334, -0.6708034926),
    Segment(132.0, 156.0, -0.0001002395, -0.6706706904),
)

K3_SEGMENTS = (
    Segment(60.0, 84.0, -0.0014435459, -2.9791489749),
    Segment(84.0, 108.0, -0.0014651305, -2.9773358734),
    Segment(108.0, 132.0, -0.0014861034, -2.9750707937),
    Segment(132.0, 156.0, -0.0015064336, -2.9723872133),
)

K4_SEGMENTS = (
    Segment(60.0, 84.0, -0.0007481298, -0.4455678693),
    Segment(84.0, 108.0, -0.0007637917, -0.4442522679),
    Segment(108.0, 132.0, -0.0007794139, -0.4425650674),
    Segment(132.0, 156.0, -0.0007949742, -0.4405111098),
)


class LQRController:
    MIN_LEG_HEIGHT_MM = 60.0
    MAX_LEG_HEIGHT_MM = 156.0

    def __init__(self) -> None:
        self.enabled = False
        self.torque_limit = 2.0
        self.output_scale = 1.0
        self.last_raw_torque = 0.0
        self.last_torque = 0.0
        self.last_leg_height_mm = self.MIN_LEG_HEIGHT_MM
        self.weights = LQRWeights()
        self.gain = LQRGain(-0.4472135955, -0.6768350558, -3.0657617289, -0.4904556573)
        self._gain_cache_key: tuple[float, LQRWeights] | None = None

    def set_enabled(self, enabled: bool) -> None:
        self.enabled = enabled
        if not enabled:
            self.reset()

    def set_torque_limit(self, torque_limit: float) -> None:
        self.torque_limit = abs(float(torque_limit))

    def set_output_scale(self, output_scale: float) -> None:
        self.output_scale = float(output_scale)

    def set_weights(
        self,
        q1: float,
        q2: float,
        q3: float,
        q4: float,
        r: float,
    ) -> None:
        self.weights = LQRWeights(
            max(float(q1), MIN_WEIGHT),
            max(float(q2), MIN_WEIGHT),
            max(float(q3), MIN_WEIGHT),
            max(float(q4), MIN_WEIGHT),
            max(float(r), MIN_WEIGHT),
        )

    def reset(self) -> None:
        self.last_raw_torque = 0.0
        self.last_torque = 0.0

    def update_gains(self, leg_height_mm: float) -> LQRGain:
        self.last_leg_height_mm = self._clamp(
            float(leg_height_mm),
            self.MIN_LEG_HEIGHT_MM,
            self.MAX_LEG_HEIGHT_MM,
        )
        cache_key = (self.last_leg_height_mm, self.weights)
        if cache_key == self._gain_cache_key:
            return self.gain
        self.gain = self.calculate_gain(self.last_leg_height_mm, self.weights)
        self._gain_cache_key = cache_key
        return self.gain

    def compute_torque(self, state: LQRState, target: LQRState, leg_height_mm: float) -> float:
        gain = self.update_gains(leg_height_mm)
        x_err = state.position_m - target.position_m
        v_err = state.velocity_mps - target.velocity_mps
        theta_err = state.pitch_rad - target.pitch_rad
        theta_rate_err = state.pitch_rate_radps - target.pitch_rate_radps

        self.last_raw_torque = -(
            gain.k1 * x_err
            + gain.k2 * v_err
            + gain.k3 * theta_err
            + gain.k4 * theta_rate_err
        ) * self.output_scale

        if not self.enabled:
            self.last_torque = 0.0
            return self.last_torque

        self.last_torque = self._clamp(self.last_raw_torque, -self.torque_limit, self.torque_limit)
        return self.last_torque

    def compute_balance_torque(
        self,
        pitch_deg: float,
        pitch_rate_dps: float,
        target_pitch_deg: float,
        leg_height_mm: float,
    ) -> float:
        state = LQRState(
            position_m=0.0,
            velocity_mps=0.0,
            pitch_rad=float(pitch_deg) * DEG_TO_RAD,
            pitch_rate_radps=float(pitch_rate_dps) * DEG_TO_RAD,
        )
        target = LQRState(
            position_m=0.0,
            velocity_mps=0.0,
            pitch_rad=float(target_pitch_deg) * DEG_TO_RAD,
            pitch_rate_radps=0.0,
        )
        return self.compute_torque(state, target, leg_height_mm)

    def compute_balance_torque_from_wheel_state(
        self,
        wheel_position_m: float,
        wheel_velocity_mps: float,
        pitch_deg: float,
        pitch_rate_dps: float,
        target_position_m: float,
        target_velocity_mps: float,
        target_pitch_deg: float,
        leg_height_mm: float,
    ) -> float:
        state = LQRState(
            position_m=float(wheel_position_m),
            velocity_mps=float(wheel_velocity_mps),
            pitch_rad=float(pitch_deg) * DEG_TO_RAD,
            pitch_rate_radps=float(pitch_rate_dps) * DEG_TO_RAD,
        )
        target = LQRState(
            position_m=float(target_position_m),
            velocity_mps=float(target_velocity_mps),
            pitch_rad=float(target_pitch_deg) * DEG_TO_RAD,
            pitch_rate_radps=0.0,
        )
        return self.compute_torque(state, target, leg_height_mm)

    @staticmethod
    def _clamp(value: float, low: float, high: float) -> float:
        if value < low:
            return low
        if value > high:
            return high
        return value

    @classmethod
    def _eval_piecewise(cls, segments: tuple[Segment, ...], leg_height_mm: float) -> float:
        clamped_height = cls._clamp(leg_height_mm, cls.MIN_LEG_HEIGHT_MM, cls.MAX_LEG_HEIGHT_MM)
        for index, segment in enumerate(segments):
            if clamped_height <= segment.max_mm or index == len(segments) - 1:
                return segment.slope * clamped_height + segment.intercept
        segment = segments[-1]
        return segment.slope * clamped_height + segment.intercept

    @classmethod
    def _piecewise_gain(cls, leg_height_mm: float) -> LQRGain:
        return LQRGain(
            cls._eval_piecewise(K1_SEGMENTS, leg_height_mm),
            cls._eval_piecewise(K2_SEGMENTS, leg_height_mm),
            cls._eval_piecewise(K3_SEGMENTS, leg_height_mm),
            cls._eval_piecewise(K4_SEGMENTS, leg_height_mm),
        )

    @classmethod
    def calculate_gain(cls, leg_height_mm: float, weights: LQRWeights | None = None) -> LQRGain:
        weights = weights or LQRWeights()
        clamped_height = cls._clamp(
            float(leg_height_mm),
            cls.MIN_LEG_HEIGHT_MM,
            cls.MAX_LEG_HEIGHT_MM,
        )
        if np is None or solve_continuous_are is None:
            return cls._piecewise_gain(clamped_height)

        l_m = clamped_height / 1000.0
        mb = MODEL_M_PENDULUM
        mw = MODEL_M_WHEEL
        iw = MODEL_WHEEL_INERTIA
        r = MODEL_WHEEL_RADIUS
        iy = max(MODEL_BODY_INERTIA_SCALE * mb * l_m * l_m, MIN_WEIGHT)

        denominator = (
            2.0 * iw * (iy + mb * l_m * l_m)
            + (2.0 * l_m * l_m * mb * mw + iy * (mb + 2.0 * mw)) * r * r
        )
        if abs(denominator) < MIN_WEIGHT:
            return cls._piecewise_gain(clamped_height)

        a1 = -(MODEL_GRAVITY * l_m * l_m * mb * mb * r * r) / denominator
        a2 = (
            MODEL_GRAVITY
            * l_m
            * mb
            * (2.0 * iw + (mb + 2.0 * mw) * r * r)
        ) / denominator
        b1 = (r * (iy + l_m * mb * (l_m + r))) / denominator
        b2 = -(
            2.0 * iw + r * (l_m * mb + (mb + 2.0 * mw) * r)
        ) / denominator

        a_matrix = np.array(
            [
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, a1, 0.0],
                [0.0, 0.0, 0.0, 1.0],
                [0.0, 0.0, a2, 0.0],
            ],
            dtype=float,
        )
        b_matrix = np.array([[0.0], [2.0 * b1], [0.0], [2.0 * b2]], dtype=float)
        q_matrix = np.diag(
            [
                max(float(weights.q1), MIN_WEIGHT),
                max(float(weights.q2), MIN_WEIGHT),
                max(float(weights.q3), MIN_WEIGHT),
                max(float(weights.q4), MIN_WEIGHT),
            ]
        )
        r_value = max(float(weights.r), MIN_WEIGHT)
        r_matrix = np.array([[r_value]], dtype=float)

        try:
            p_matrix = solve_continuous_are(a_matrix, b_matrix, q_matrix, r_matrix)
            k_matrix = (b_matrix.T @ p_matrix) / r_value
        except Exception:
            return cls._piecewise_gain(clamped_height)

        return LQRGain(*(float(value) for value in k_matrix.reshape(4)))
