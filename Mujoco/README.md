# MuJoCo 模擬

本資料夾包含雙輪足五連桿模型、運動學、手動 Viewer 與輪端力矩 LQR 平衡模擬。

主要模型是 `models/two_wheel_legged.xml`；基本檢查使用 `smoke_test.py`，平衡模擬使用 `lqr_balance_sim.py`。`rl_smoke_test.py` 只測試 RL 套件環境，尚非本機器人的訓練程式。
