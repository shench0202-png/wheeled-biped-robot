from __future__ import annotations

from pathlib import Path

import mujoco
import numpy as np

from five_bar_kinematics import bilateral_hip_targets, solve_five_bar


MODEL_PATH = Path(__file__).resolve().parent / "models" / "two_wheel_legged.xml"

ACTIVE_JOINTS = (
    "left_front_hip",
    "left_rear_hip",
    "right_front_hip",
    "right_rear_hip",
    "left_wheel_joint",
    "right_wheel_joint",
)

LEG_GEOMS = (
    "left_front_upper_geom",
    "left_front_lower_geom",
    "left_rear_upper_geom",
    "left_rear_lower_geom",
    "right_front_upper_geom",
    "right_front_lower_geom",
    "right_rear_upper_geom",
    "right_rear_lower_geom",
)


def main() -> None:
    model = mujoco.MjModel.from_xml_string(MODEL_PATH.read_text(encoding="utf-8"))
    data = mujoco.MjData(model)

    mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)

    assert model.nu == 6, f"Expected 6 actuators, got {model.nu}"
    assert model.nq == 17, f"Expected 17 qpos values, got {model.nq}"
    assert model.nv == 16, f"Expected 16 qvel values, got {model.nv}"
    assert model.neq == 2, f"Expected 2 five-bar closures, got {model.neq}"
    assert np.isclose(model.body_mass.sum(), 0.7981, atol=1e-6)
    assert np.isclose(model.geom("left_tire").size[0], 0.033)
    assert np.isclose(model.geom("right_tire").size[0], 0.033)
    assert model.geom_contype[model.geom("left_tire").id] == 1
    assert model.geom_conaffinity[model.geom("left_tire").id] == 1
    assert model.geom_contype[model.geom("right_tire").id] == 1
    assert model.geom_conaffinity[model.geom("right_tire").id] == 1
    for geom_name in LEG_GEOMS:
        geom_id = model.geom(geom_name).id
        assert model.geom_contype[geom_id] == 1
        assert model.geom_conaffinity[geom_id] == 1
    assert np.isfinite(data.qpos).all()

    left_closure_error = np.linalg.norm(
        data.site("left_front_endpoint").xpos
        - data.site("left_rear_endpoint").xpos
    )
    right_closure_error = np.linalg.norm(
        data.site("right_front_endpoint").xpos
        - data.site("right_rear_endpoint").xpos
    )
    assert left_closure_error < 1e-6
    assert right_closure_error < 1e-6
    assert np.allclose(
        bilateral_hip_targets(0.120, 0.120),
        np.zeros(4),
        atol=1e-7,
    )
    for test_length in (0.070, 0.120, 0.155):
        pose = solve_five_bar(test_length)
        assert np.isfinite(
            [
                pose.front_hip,
                pose.rear_hip,
                pose.front_knee,
                pose.rear_knee,
            ]
        ).all()

    # Zero commands intentionally do not balance the free robot. Validate
    # natural falling, wheel-ground contact and closed-chain stability.
    data.ctrl[:] = 0.0
    tire_contact_observed = False
    leg_ground_contact_observed = False
    for _ in range(10000):
        mujoco.mj_step(model, data)
        contact_pairs = {
            frozenset(
                (
                    model.geom(contact.geom1).name,
                    model.geom(contact.geom2).name,
                )
            )
            for contact in data.contact[: data.ncon]
        }
        tire_contact_observed |= (
            frozenset(("ground", "left_tire")) in contact_pairs
            and frozenset(("ground", "right_tire")) in contact_pairs
        )
        leg_ground_contact_observed |= any(
            frozenset(("ground", geom_name)) in contact_pairs
            for geom_name in LEG_GEOMS
        )

    assert np.isfinite(data.qpos).all()
    assert np.isfinite(data.qvel).all()
    assert np.linalg.norm(
        data.site("left_front_endpoint").xpos
        - data.site("left_rear_endpoint").xpos
    ) < 1e-5
    assert np.linalg.norm(
        data.site("right_front_endpoint").xpos
        - data.site("right_rear_endpoint").xpos
    ) < 1e-5
    assert tire_contact_observed
    assert leg_ground_contact_observed
    assert data.body("left_wheel").xpos[2] > 0.029
    assert data.body("right_wheel").xpos[2] > 0.029

    # A fresh state verifies that the native Control-panel wheel speed command
    # produces wheel motion and whole-body reaction without an LQR controller.
    motor_data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, motor_data, 0)
    motor_data.ctrl[4:] = 10.0
    for _ in range(1000):
        mujoco.mj_step(model, motor_data)

    assert motor_data.joint("left_wheel_joint").qvel[0] > 8.0
    assert motor_data.joint("right_wheel_joint").qvel[0] > 8.0
    assert abs(motor_data.qpos[0]) > 0.005
    assert np.isfinite(motor_data.qpos).all()
    assert np.isfinite(motor_data.qvel).all()

    print(f"MuJoCo version : {mujoco.__version__}")
    print(f"Model          : {MODEL_PATH}")
    print(f"Model size     : nq={model.nq}, nv={model.nv}, nu={model.nu}")
    print(f"Constraints    : neq={model.neq}")
    print(f"Wheel radius   : 0.033 m")
    print(f"Mode           : free-body physical joint test")
    print(f"Hip control    : position target [rad]")
    print(f"Wheel control  : velocity target [rad/s]")
    print(f"Simulated time : {data.time:.3f} s")
    print("Smoke test     : PASS")


if __name__ == "__main__":
    main()
