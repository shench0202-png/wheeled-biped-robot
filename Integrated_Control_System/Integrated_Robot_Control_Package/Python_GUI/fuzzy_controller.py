from __future__ import annotations

from dataclasses import dataclass


LABELS = ("NB", "NS", "ZE", "PS", "PB")
NB_FULL = -1.0
NB_ZERO = -0.65
NS_LEFT = -1.0
NS_CENTER = -0.35
NS_RIGHT = 0.0
ZE_LEFT = -0.45
ZE_CENTER = 0.0
ZE_RIGHT = 0.45
PS_LEFT = 0.0
PS_CENTER = 0.35
PS_RIGHT = 1.0
PB_ZERO = 0.65
PB_FULL = 1.0


@dataclass(frozen=True)
class FuzzyDebug:
    pitch_error_deg: float = 0.0
    normalized_error: float = 0.0
    normalized_rate: float = 0.0
    rule_weight_sum: float = 0.0
    normalized_output: float = 0.0
    height_scale: float = 1.0
    safety_limited: bool = False


class FuzzyBalanceController:
    MIN_LEG_HEIGHT_MM = 60.0
    MAX_LEG_HEIGHT_MM = 156.0

    RULE_TABLE = {
        "NB": {"NB": "NB", "NS": "NB", "ZE": "NB", "PS": "NS", "PB": "ZE"},
        "NS": {"NB": "NB", "NS": "NB", "ZE": "NS", "PS": "ZE", "PB": "PS"},
        "ZE": {"NB": "NB", "NS": "NS", "ZE": "ZE", "PS": "PS", "PB": "PB"},
        "PS": {"NB": "NS", "NS": "ZE", "ZE": "PS", "PS": "PB", "PB": "PB"},
        "PB": {"NB": "ZE", "NS": "PS", "ZE": "PB", "PS": "PB", "PB": "PB"},
    }

    OUTPUT_SINGLETONS = {
        "NB": -1.0,
        "NS": -0.45,
        "ZE": 0.0,
        "PS": 0.45,
        "PB": 1.0,
    }

    def __init__(self) -> None:
        self.enabled = False
        self.torque_limit = 0.35
        self.output_scale = 1.0
        self.polarity = 1.0
        self.angle_range_deg = 8.0
        self.rate_range_dps = 120.0
        self.deadband_angle_deg = 0.20
        self.deadband_rate_dps = 2.0
        self.safety_pitch_deg = 25.0
        self.height_gain = 0.0
        self.last_raw_torque = 0.0
        self.last_torque = 0.0
        self.last_debug = FuzzyDebug()

    def set_enabled(self, enabled: bool) -> None:
        self.enabled = bool(enabled)
        if not self.enabled:
            self.reset()

    def set_torque_limit(self, torque_limit: float) -> None:
        self.torque_limit = max(0.0, abs(float(torque_limit)))

    def set_output_scale(self, output_scale: float) -> None:
        self.output_scale = max(0.0, float(output_scale))

    def set_polarity(self, polarity: float) -> None:
        self.polarity = 1.0 if float(polarity) >= 0.0 else -1.0

    def set_input_ranges(self, angle_range_deg: float, rate_range_dps: float) -> None:
        self.angle_range_deg = max(0.1, abs(float(angle_range_deg)))
        self.rate_range_dps = max(1.0, abs(float(rate_range_dps)))

    def set_deadband(self, angle_deg: float, rate_dps: float) -> None:
        self.deadband_angle_deg = max(0.0, abs(float(angle_deg)))
        self.deadband_rate_dps = max(0.0, abs(float(rate_dps)))

    def set_safety_pitch(self, safety_pitch_deg: float) -> None:
        self.safety_pitch_deg = max(1.0, abs(float(safety_pitch_deg)))

    def set_height_gain(self, height_gain: float) -> None:
        self.height_gain = float(height_gain)

    def reset(self) -> None:
        self.last_raw_torque = 0.0
        self.last_torque = 0.0

    def compute_balance_torque(
        self,
        pitch_deg: float,
        pitch_rate_dps: float,
        target_pitch_deg: float,
        leg_height_mm: float,
    ) -> float:
        pitch_error = float(pitch_deg) - float(target_pitch_deg)
        normalized_error = self._clamp(pitch_error / self.angle_range_deg, -1.0, 1.0)
        normalized_rate = self._clamp(float(pitch_rate_dps) / self.rate_range_dps, -1.0, 1.0)

        if abs(pitch_error) >= self.safety_pitch_deg:
            self.last_debug = FuzzyDebug(
                pitch_error,
                normalized_error,
                normalized_rate,
                0.0,
                0.0,
                1.0,
                True,
            )
            self.last_raw_torque = 0.0
            self.last_torque = 0.0
            return 0.0

        if abs(pitch_error) <= self.deadband_angle_deg and abs(pitch_rate_dps) <= self.deadband_rate_dps:
            normalized_output = 0.0
            weight_sum = 1.0
        else:
            error_membership = self._memberships(normalized_error)
            rate_membership = self._memberships(normalized_rate)
            weighted_output = 0.0
            weight_sum = 0.0
            for error_label, error_grade in error_membership.items():
                if error_grade <= 0.0:
                    continue
                for rate_label, rate_grade in rate_membership.items():
                    if rate_grade <= 0.0:
                        continue
                    weight = min(error_grade, rate_grade)
                    output_label = self.RULE_TABLE[error_label][rate_label]
                    weighted_output += weight * self.OUTPUT_SINGLETONS[output_label]
                    weight_sum += weight
            normalized_output = weighted_output / weight_sum if weight_sum > 0.0 else 0.0

        height_scale = self._height_scale(leg_height_mm)
        self.last_debug = FuzzyDebug(
            pitch_error,
            normalized_error,
            normalized_rate,
            weight_sum,
            normalized_output,
            height_scale,
            False,
        )
        self.last_raw_torque = (
            self.polarity
            * normalized_output
            * self.torque_limit
            * self.output_scale
            * height_scale
        )

        if not self.enabled:
            self.last_torque = 0.0
            return 0.0

        self.last_torque = self._clamp(self.last_raw_torque, -self.torque_limit, self.torque_limit)
        return self.last_torque

    def _height_scale(self, leg_height_mm: float) -> float:
        clamped_height = self._clamp(
            float(leg_height_mm),
            self.MIN_LEG_HEIGHT_MM,
            self.MAX_LEG_HEIGHT_MM,
        )
        normalized_height = (
            (clamped_height - self.MIN_LEG_HEIGHT_MM)
            / (self.MAX_LEG_HEIGHT_MM - self.MIN_LEG_HEIGHT_MM)
        )
        return max(0.1, 1.0 + self.height_gain * (normalized_height - 0.5))

    @classmethod
    def _memberships(cls, x: float) -> dict[str, float]:
        return {
            "NB": cls._left_shoulder(x, NB_FULL, NB_ZERO),
            "NS": cls._triangle(x, NS_LEFT, NS_CENTER, NS_RIGHT),
            "ZE": cls._triangle(x, ZE_LEFT, ZE_CENTER, ZE_RIGHT),
            "PS": cls._triangle(x, PS_LEFT, PS_CENTER, PS_RIGHT),
            "PB": cls._right_shoulder(x, PB_ZERO, PB_FULL),
        }

    @staticmethod
    def _triangle(x: float, left: float, center: float, right: float) -> float:
        if x <= left or x >= right:
            return 0.0
        if x == center:
            return 1.0
        if x < center:
            return (x - left) / (center - left)
        return (right - x) / (right - center)

    @staticmethod
    def _left_shoulder(x: float, high_until: float, zero_at: float) -> float:
        if x <= high_until:
            return 1.0
        if x >= zero_at:
            return 0.0
        return (zero_at - x) / (zero_at - high_until)

    @staticmethod
    def _right_shoulder(x: float, zero_until: float, high_at: float) -> float:
        if x <= zero_until:
            return 0.0
        if x >= high_at:
            return 1.0
        return (x - zero_until) / (high_at - zero_until)

    @staticmethod
    def _clamp(value: float, low: float, high: float) -> float:
        if value < low:
            return low
        if value > high:
            return high
        return value
