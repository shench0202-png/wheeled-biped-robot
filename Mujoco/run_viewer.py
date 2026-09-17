from __future__ import annotations

from pathlib import Path

import mujoco
import mujoco.viewer


MODEL_PATH = Path(__file__).resolve().parent / "models" / "two_wheel_legged.xml"


def print_help() -> None:
    print(
        """
MuJoCo Joint Test 模式
======================
這個模式不執行 LQR；torso 含 root freejoint，整台機器人遵循物理法則。
輪子初始接觸地面，車體受到重力、接觸、摩擦與馬達反作用力。
沒有平衡控制時模型可能自然翻倒，這是預期的物理結果。

請在 Viewer 右側展開 Control：
  left_front_hip_target   左腿前髖目標角度，單位 rad
  left_rear_hip_target    左腿後髖目標角度，單位 rad
  right_front_hip_target  右腿前髖目標角度，單位 rad
  right_rear_hip_target   右腿後髖目標角度，單位 rad
  left_wheel_speed        左輪目標速度，單位 rad/s
  right_wheel_speed       右輪目標速度，單位 rad/s

操作：
  1. 左側 Simulation 按 Run。
  2. 拖曳右側 Control 的紫色數值條。
  3. Clear all 將所有命令歸零。
  4. Reset 將模型狀態重設，但 Control 命令可能仍需 Clear all。

五連桿是閉鏈。建議左右對稱、前後髖反向調整：
  左腿：front = +a，rear = -a
  右腿：front = +a，rear = -a

Viewer 使用內建 physics thread，不會覆寫 Control 面板數值。
"""
    )


def main() -> None:
    model = mujoco.MjModel.from_xml_string(MODEL_PATH.read_text(encoding="utf-8"))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)

    print_help()

    # The built-in physics thread makes the native Run/Pause/Reset controls
    # authoritative. No Python control loop overwrites data.ctrl.
    mujoco.viewer.launch(model, data)


if __name__ == "__main__":
    main()
