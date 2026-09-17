from __future__ import annotations

import gymnasium as gym
import numpy as np
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env


def check_cuda() -> None:
    print(f"PyTorch        : {torch.__version__}")
    print(f"CUDA available : {torch.cuda.is_available()}")

    if torch.cuda.is_available():
        device = torch.device("cuda")
        result = torch.ones(1024, device=device).square().sum()
        torch.cuda.synchronize()
        print(f"GPU             : {torch.cuda.get_device_name(0)}")
        print(f"CUDA tensor test: {result.item():.0f}")
    else:
        print("GPU             : CPU mode")


def check_gymnasium_mujoco() -> None:
    env = gym.make("HalfCheetah-v5")
    observation, info = env.reset(seed=42)
    assert np.isfinite(observation).all()

    action = env.action_space.sample()
    observation, reward, terminated, truncated, info = env.step(action)
    assert np.isfinite(observation).all()
    assert np.isfinite(reward)

    print(f"Gym environment : {env.spec.id}")
    print(f"Observation     : {env.observation_space.shape}")
    print(f"Action          : {env.action_space.shape}")
    env.close()


def check_stable_baselines() -> None:
    # Pendulum keeps this installation test quick. The custom biped environment
    # can later replace it without changing the PPO training interface.
    env = gym.make("Pendulum-v1")
    check_env(env.unwrapped, warn=True)

    model = PPO(
        "MlpPolicy",
        env,
        n_steps=64,
        batch_size=32,
        n_epochs=1,
        device="cpu",
        verbose=0,
        seed=42,
    )
    model.learn(total_timesteps=128)

    observation, info = env.reset(seed=42)
    action, _ = model.predict(observation, deterministic=True)
    assert env.action_space.contains(action)
    env.close()
    print("SB3 PPO test    : PASS")


def main() -> None:
    check_cuda()
    check_gymnasium_mujoco()
    check_stable_baselines()
    print("RL stack test   : PASS")


if __name__ == "__main__":
    main()
