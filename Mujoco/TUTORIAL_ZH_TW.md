# MuJoCo 從零到雙足機器人：繁體中文實作教學

> **目前專案模型更新：** `Mujoco/models/two_wheel_legged.xml` 已改為左右各一組五連桿腿與 33 mm 驅動輪。模型的最新幾何、質量、關節與執行方式請先參考同目錄的 `README.md`。本文部分早期程式片段仍用簡化四關節名稱說明 MuJoCo API 概念；實際執行應以目前 XML 與 Python 檔案中的六個 actuator 名稱為準。

這份教學直接使用本專案的 `Mujoco/` 目錄。內容依序涵蓋：

1. MuJoCo 是什麼，以及程式如何組成
2. 在 Windows 啟動目前已安裝好的環境
3. 執行模型、Viewer 與基礎物理模擬
4. 閱讀與修改 MJCF XML 模型
5. 取得狀態、輸入馬達命令、讀取感測器
6. PD 控制器的原理
7. Gymnasium 與 PPO 強化學習的銜接方式
8. 雙足機器人建模、訓練與除錯建議
9. MuJoCo、ROS 2、ros2_control 與 RL policy 的整合方式

---

## 1. MuJoCo 是什麼？

MuJoCo（Multi-Joint dynamics with Contact）是剛體動力學模擬器，特別適合：

- 機器人關節、馬達、接觸與摩擦模擬
- 雙足、四足、機械手臂等控制研究
- 強化學習環境
- 控制器在真機部署前的初步驗證

它的工作可以簡化成：

```text
模型 XML
   ↓ 載入、編譯
MjModel：不常改變的模型資料
   +
MjData：每個時間點的狀態與計算結果
   ↓
寫入 data.ctrl
   ↓
mujoco.mj_step(model, data)
   ↓
更新位置、速度、接觸力與感測器
```

最重要的兩個 Python 物件是：

- `mujoco.MjModel`：質量、幾何、關節、致動器、時間步長等模型資料。
- `mujoco.MjData`：目前的 `qpos`、`qvel`、控制輸入、感測器與模擬時間。

---

## 2. 目前專案的環境

本專案已建立：

```text
two wheel-legged robot/
├─ .venv/                         Python 虛擬環境
└─ Mujoco/
   ├─ models/
   │  └─ two_wheel_legged.xml     五連桿雙輪足 MJCF 模型
   ├─ smoke_test.py               模型載入與物理步進測試
   ├─ run_viewer.py               Viewer 與 PD 控制範例
   ├─ rl_smoke_test.py            Gymnasium、PPO、CUDA 測試
   ├─ requirements.txt
   └─ setup.ps1
```

目前鎖定的主要套件：

- MuJoCo 3.9.0
- Gymnasium 1.3.0
- Stable-Baselines3 2.9.0
- PyTorch 2.12.1 + CUDA 12.6

已在本機驗證：

- MuJoCo 模型可載入並正常模擬
- Gymnasium `HalfCheetah-v5` 可執行
- Stable-Baselines3 PPO 可訓練
- RTX 4060 Laptop GPU 可被 PyTorch 使用

---

## 3. 第一次啟動

在 PowerShell 進入專案：

```powershell
cd "C:\project\two wheel-legged robot"
```

### 方法 A：直接指定虛擬環境 Python

這是最不容易受到 PowerShell 執行原則影響的方式：

```powershell
.\.venv\Scripts\python.exe .\Mujoco\smoke_test.py
.\.venv\Scripts\python.exe .\Mujoco\rl_smoke_test.py
.\.venv\Scripts\python.exe .\Mujoco\run_viewer.py
```

### 方法 B：先啟用虛擬環境

```powershell
.\.venv\Scripts\Activate.ps1
python .\Mujoco\smoke_test.py
python .\Mujoco\run_viewer.py
```

如果環境尚未建立，才需要執行：

```powershell
powershell -ExecutionPolicy Bypass -File .\Mujoco\setup.ps1
```

目前環境已經安裝完成，不需要重跑安裝。

---

## 4. 最小 MuJoCo Python 程式

核心流程只有四步：

```python
from pathlib import Path
import mujoco

model_path = Path("Mujoco/models/two_wheel_legged.xml")

# 1. 載入模型
model = mujoco.MjModel.from_xml_path(str(model_path))

# 2. 配置模擬狀態
data = mujoco.MjData(model)

# 3. 執行 1000 個 physics steps
for _ in range(1000):
    data.ctrl[:] = 0.0
    mujoco.mj_step(model, data)

# 4. 讀取結果
print("time =", data.time)
print("qpos =", data.qpos)
print("qvel =", data.qvel)
```

`mj_step()` 每呼叫一次，就讓模擬前進一個 `model.opt.timestep`。

本模型設定：

```xml
<option timestep="0.002" integrator="implicitfast" gravity="0 0 -9.81"/>
```

因此：

- 一步是 0.002 秒
- 500 步是 1 秒
- physics frequency 是 `1 / 0.002 = 500 Hz`

---

## 5. `qpos`、`qvel`、`ctrl` 是什麼？

### `data.qpos`

廣義位置，包括：

- 浮動基座位置：`x, y, z`
- 浮動基座姿態 quaternion：`qw, qx, qy, qz`
- 各旋轉關節角度

目前模型：

```text
7 個 root freejoint 狀態
+ 4 個 hinge joint 角度
= nq 11
```

### `data.qvel`

廣義速度，包括：

- 浮動基座線速度 3 維
- 浮動基座角速度 3 維
- 四個關節角速度

所以目前：

```text
6 + 4 = nv 10
```

注意：自由關節姿態用 quaternion 表示，所以 `nq` 不一定等於 `nv`。

### `data.ctrl`

致動器命令。目前有四顆 motor：

```text
ctrl[0] = left_hip_motor
ctrl[1] = right_hip_motor
ctrl[2] = left_knee_motor
ctrl[3] = right_knee_motor
```

XML 中的控制範圍是：

```xml
<motor ctrllimited="true" ctrlrange="-60 60"/>
```

因為 actuator 使用 `<motor ... gear="1"/>`，目前可將 `ctrl` 理解成限制在約 `-60` 到 `60` 的關節力矩命令。

---

## 6. 建議使用名稱存取狀態

不要把所有邏輯綁死在陣列索引。MuJoCo Python API 可以依名稱存取：

```python
left_hip_angle = data.joint("left_hip").qpos[0]
left_hip_speed = data.joint("left_hip").qvel[0]
torso_position = data.body("torso").xpos.copy()
```

需要 `.copy()` 的原因是 MuJoCo 的 NumPy 欄位通常是底層記憶體的 view。執行下一次 `mj_step()` 後，沒有複製的舊值可能跟著改變。

致動器 ID 也能由名稱取得：

```python
motor_id = model.actuator("left_hip_motor").id
data.ctrl[motor_id] = 5.0
```

---

## 7. 開啟 Viewer

執行：

```powershell
.\.venv\Scripts\python.exe .\Mujoco\run_viewer.py
```

目前程式使用：

```python
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        # 計算控制
        data.ctrl[:] = ...

        # 物理前進一步
        mujoco.mj_step(model, data)

        # 將最新狀態同步到視窗
        viewer.sync()
```

`launch_passive()` 不會封鎖控制程式，因此適合：

- 自己控制 physics loop
- 每一步執行 PD、MPC 或神經網路 policy
- 即時記錄狀態

也可以只用官方獨立 Viewer 載入 XML：

```powershell
.\.venv\Scripts\python.exe -m mujoco.viewer --mjcf=.\Mujoco\models\two_wheel_legged.xml
```

這種方式適合快速檢查模型，但不會執行 `run_viewer.py` 裡的控制器。

---

## 8. 目前雙足模型的 MJCF 結構

MJCF 是 MuJoCo 的 XML 模型格式。模型主要分成：

```xml
<mujoco>
  <compiler/>      單位與編譯設定
  <option/>        時間步長、重力、積分器
  <asset/>         mesh、材質、貼圖
  <default/>       共用的預設參數
  <worldbody/>     剛體階層、關節、碰撞幾何
  <actuator/>      馬達或其他致動器
  <sensor/>        IMU、接觸、關節等感測器
  <keyframe/>      初始姿勢
</mujoco>
```

### 剛體是樹狀階層

目前結構：

```text
world
└─ torso（freejoint）
   ├─ left_thigh（left_hip）
   │  └─ left_shin（left_knee）
   │     └─ left_foot
   └─ right_thigh（right_hip）
      └─ right_shin（right_knee）
         └─ right_foot
```

子 body 的位置是相對於父 body。

例如：

```xml
<body name="left_shin" pos="0 0 -0.35">
```

表示左小腿原點在左大腿座標系向下 0.35 m。

### 關節

```xml
<joint name="left_hip"
       type="hinge"
       axis="0 1 0"
       range="-1.2 1.2"/>
```

意思是：

- 單軸旋轉關節
- 沿局部 Y 軸旋轉
- 角度限制為 -1.2 到 1.2 rad

模型已設定：

```xml
<compiler angle="radian" autolimits="true"/>
```

所以角度全部使用弧度。

### 幾何與碰撞

```xml
<geom type="capsule"
      fromto="0 0 0 0 0 -0.35"
      size="0.045"
      density="650"/>
```

`geom` 同時可參與：

- 外觀顯示
- 碰撞
- 由密度估算質量與慣量

真實機器人建模時，建議將「視覺 mesh」與「簡化碰撞 geom」分開。碰撞模型使用 box、capsule、sphere 通常更穩定、更快速。

---

## 9. PD 關節控制

`run_viewer.py` 使用：

```python
torque = kp * (target - position) - kd * velocity
data.ctrl[:] = torque
```

數學式：

```text
τ = Kp(q_target - q) - Kd q_dot
```

- `Kp`：位置誤差越大，回正力矩越大
- `Kd`：抑制速度與振盪
- `q_target`：目標關節角
- `q`：目前角度
- `q_dot`：目前角速度

目前參數：

```python
kp = [18, 18, 12, 12]
kd = [1.2, 1.2, 0.8, 0.8]
```

調參方向：

- 動作太軟、跟不上：增加 `Kp`
- 高頻振盪：增加 `Kd`，或降低 `Kp`
- 接觸時爆震：降低增益、縮小 timestep、檢查質量與碰撞
- 力矩經常飽和：檢查 actuator `ctrlrange`

測試固定姿勢：

```python
target = np.array([0.15, 0.15, 0.55, 0.55])
```

修改後再執行 Viewer，即可看到髖與膝關節追蹤這組角度。

---

## 10. 讀取 IMU 與足底接觸

XML 已定義：

```xml
<framequat name="imu_orientation" objtype="site" objname="imu"/>
<gyro name="imu_gyro" site="imu"/>
<accelerometer name="imu_accelerometer" site="imu"/>
<touch name="left_foot_contact" site="left_foot_site"/>
<touch name="right_foot_contact" site="right_foot_site"/>
```

Python 可依名稱讀取：

```python
imu_quat = data.sensor("imu_orientation").data.copy()
gyro = data.sensor("imu_gyro").data.copy()
accel = data.sensor("imu_accelerometer").data.copy()
left_contact = float(data.sensor("left_foot_contact").data[0])
right_contact = float(data.sensor("right_foot_contact").data[0])
```

這些資料可以組成強化學習 observation，例如：

```python
observation = np.concatenate([
    data.qpos.copy(),
    data.qvel.copy(),
    gyro,
    accel,
    [left_contact, right_contact],
])
```

實際訓練通常不直接使用 quaternion 四個值做所有判斷，也可以轉成重力方向、roll/pitch 或軀幹朝上向量。

---

## 11. Reset 與初始姿勢

XML 內有：

```xml
<keyframe>
  <key name="home" qpos="0 0 0.77 1 0 0 0 0 0 0 0"/>
</keyframe>
```

程式使用：

```python
mujoco.mj_resetDataKeyframe(model, data, 0)
mujoco.mj_forward(model, data)
```

作用：

1. 將狀態重設到第 0 個 keyframe
2. 重新計算 body pose、sensor、接觸等衍生資料

強化學習中，建議在 reset 時加入小幅隨機化：

```python
mujoco.mj_resetDataKeyframe(model, data, 0)
data.qpos[7:] += np.random.uniform(-0.03, 0.03, size=4)
data.qvel[:] += np.random.uniform(-0.02, 0.02, size=model.nv)
mujoco.mj_forward(model, data)
```

這可避免 policy 只記住完全相同的初始狀態。

---

## 12. 從 MuJoCo 接到 Gymnasium

Gymnasium 環境的基本介面是：

```python
observation, info = env.reset(seed=42)

observation, reward, terminated, truncated, info = env.step(action)
```

自訂雙足環境通常需要實作：

```python
class TwoLegEnv(gym.Env):
    def __init__(self):
        # 載入 MjModel、建立 MjData
        # 定義 observation_space 與 action_space
        ...

    def reset(self, seed=None, options=None):
        # 重設模型與隨機初始狀態
        ...
        return observation, info

    def step(self, action):
        # action -> 馬達命令
        # 執行多個 physics steps
        # 計算 observation、reward、終止條件
        ...
        return observation, reward, terminated, truncated, info
```

### Action 建議正規化

Stable-Baselines3 建議連續 action 使用對稱的 `[-1, 1]`：

```python
self.action_space = gym.spaces.Box(
    low=-1.0,
    high=1.0,
    shape=(model.nu,),
    dtype=np.float32,
)
```

再將 action 映射成力矩：

```python
max_torque = np.array([60, 60, 60, 60])
data.ctrl[:] = np.clip(action, -1, 1) * max_torque
```

若 policy 輸出的是關節目標角，則可用 action 產生 `q_target`，再由 PD 控制器轉成力矩。這通常比一開始直接學力矩容易。

### Control frequency 與 frame skip

物理頻率是 500 Hz，但 policy 不一定要 500 Hz 執行。

例如每個 action 維持 10 個 physics steps：

```python
for _ in range(10):
    data.ctrl[:] = torque
    mujoco.mj_step(model, data)
```

則控制頻率是：

```text
500 Hz / 10 = 50 Hz
```

---

## 13. Reward 怎麼設計？

站立任務的簡化 reward：

```text
reward =
    + 存活獎勵
    + 軀幹高度獎勵
    + 軀幹保持直立獎勵
    - 關節速度懲罰
    - 馬達能量懲罰
```

走路任務可加入：

```text
+ 前進速度追蹤
+ 左右腳交替或步態週期追蹤
- 身體側向偏移
- 足部打滑
- 過大的接觸衝擊
```

概念範例：

```python
height_reward = np.exp(-20.0 * (torso_z - target_height) ** 2)
upright_reward = torso_up_z
velocity_reward = np.exp(-2.0 * (forward_velocity - target_velocity) ** 2)
control_cost = 1e-3 * np.sum(np.square(action))

reward = (
    1.0
    + 1.0 * height_reward
    + 1.0 * upright_reward
    + 2.0 * velocity_reward
    - control_cost
)
```

重要原則：

- 先讓模型學會站，再學走
- 每個 reward 項目的量級要可比較
- 記錄各 reward component，不只記總分
- 終止條件不能太嚴，否則早期幾乎沒有學習資料

---

## 14. PPO 訓練基本形式

環境完成後：

```python
from stable_baselines3 import PPO

env = TwoLegEnv()

model = PPO(
    "MlpPolicy",
    env,
    n_steps=2048,
    batch_size=64,
    learning_rate=3e-4,
    gamma=0.99,
    device="auto",
    tensorboard_log="Mujoco/output/tensorboard",
    verbose=1,
)

model.learn(total_timesteps=1_000_000)
model.save("Mujoco/output/two_leg_ppo")
```

推論：

```python
model = PPO.load("Mujoco/output/two_leg_ppo")
obs, info = env.reset()

while True:
    action, _ = model.predict(obs, deterministic=True)
    obs, reward, terminated, truncated, info = env.step(action)
    if terminated or truncated:
        obs, info = env.reset()
```

查看 TensorBoard：

```powershell
.\.venv\Scripts\tensorboard.exe --logdir .\Mujoco\output
```

瀏覽器開啟終端顯示的 localhost 網址。

---

## 15. CPU、GPU 與訓練速度

MuJoCo 本身的傳統 rigid-body physics 主要在 CPU 執行。GPU 在這套流程中主要負責神經網路。

因此：

- 小型 MLP PPO 不一定比 CPU 快很多
- 多個並行環境通常比單純換 GPU 更能提高採樣速度
- 先確保環境 step 快、模型碰撞簡潔
- Viewer 只用於觀察，不要在正式大量訓練時開啟

典型流程：

```text
多個 CPU MuJoCo 環境產生資料
             ↓
       GPU 更新 policy
```

---

## 16. 目前模型的限制

現在的 `two_wheel_legged.xml` 是依五連桿幾何與 LQR 等效質量建立的雙輪足起始模型，但仍不是完整可泛化的實體模型。

目前只有：

- 左右 hip pitch
- 左右 knee pitch

缺少：

- hip roll / yaw
- ankle pitch / roll
- 更精確的 link mass、COM、inertia
- 馬達減速比、轉子慣量與扭矩速度限制
- 編碼器、IMU noise 與延遲
- 更精確的足底接觸模型

因此它比較適合：

- 熟悉 MuJoCo
- 驗證 API
- 練習 PD 與 Gymnasium
- 建立初版 RL pipeline

若目標是三維穩定行走，至少需要增加控制側向平衡的自由度，或先將模型明確限制成平面雙足。

---

## 17. 從模擬走向真機

推薦順序：

1. 用 box/capsule 建立正確尺寸與關節方向。
2. 填入每個 link 的實測質量、COM、慣量。
3. 驗證單關節方向、零點與限制。
4. 驗證無接觸狀態下的重力與自由落體。
5. 加入 PD 控制，確認固定姿勢。
6. 加入感測器與控制延遲。
7. 設計站立 reward，先學會不跌倒。
8. 加入速度命令，再學走路。
9. 加入 domain randomization。
10. 真機先限制力矩、角度與速度，再逐步測試。

Domain randomization 可隨機化：

- 質量與 COM
- 摩擦係數
- 馬達強度
- 控制延遲
- 感測器 noise
- 地面坡度
- 初始姿勢

這能降低 policy 只適用於單一理想模擬參數的風險，但不能取代真實參數校正。

---

## 18. 常見問題

### 模型一開始就爆開或飛走

檢查：

- body 是否一開始彼此重疊
- geom 尺寸與位置是否正確
- 質量或慣量是否極端
- 關節初始角是否超出 range
- 控制器力矩是否立即飽和
- timestep 是否太大

### 腳穿過地面

檢查：

- 足部 geom 是否啟用碰撞
- 地面 geom 是否存在
- 接觸參數與 timestep
- mesh 法向、尺度與模型單位

### PD 一直振盪

依序嘗試：

1. 降低 `Kp`
2. 增加 `Kd`
3. 降低最大力矩
4. 檢查 link mass 與 actuator gear
5. 將 timestep 從 0.002 降到 0.001 做比較

### 訓練 reward 不上升

先確認：

- random action 是否真的會改變機器人
- observation 全部有限且尺度合理
- action 已正規化
- reset 後狀態正確
- reward component 沒有互相抵銷
- episode 不會在第一、二步就結束
- policy control frequency 合理

### 訓練正常但 Viewer 表現不同

常見原因：

- 推論時 observation 正規化資訊沒載入
- 訓練與 Viewer 的 frame skip 不同
- 推論使用不同模型 XML
- `deterministic` 設定不同
- reset、action scaling 或 PD gains 不一致

---

## 19. 建議的學習實作順序

### 第一階段：理解物理迴圈

執行：

```powershell
.\.venv\Scripts\python.exe .\Mujoco\smoke_test.py
```

接著修改程式，列印：

```python
print(data.time, data.qpos, data.qvel)
```

### 第二階段：改關節目標

修改 `run_viewer.py`：

```python
target = np.array([0.15, 0.15, 0.55, 0.55])
```

觀察 PD 追蹤。

### 第三階段：讀取 IMU 與足底接觸

在 loop 中列印或記錄 sensor：

```python
print(data.sensor("imu_gyro").data)
```

不要每個 500 Hz step 都大量列印；可每 50 或 100 步輸出一次。

### 第四階段：做自訂 Gymnasium 環境

先只做「站立」：

- action：四個關節目標
- observation：姿態、角速度、關節角、關節速度
- reward：高度、直立、低能量
- termination：軀幹高度過低或傾角過大

### 第五階段：再加入走路

站立穩定後才加入：

- 目標前進速度
- 腳步切換
- 足部打滑懲罰
- 不同速度命令

---

## 20. MuJoCo 如何與 ROS 2 結合？

MuJoCo 負責物理模擬，ROS 2 負責機器人軟體各模組之間的通訊與整合。

可以把兩者的責任分成：

```text
MuJoCo
├─ 剛體動力學
├─ 關節與接觸
├─ 馬達輸入力矩
├─ IMU、接觸等模擬感測器
└─ 模擬時間

ROS 2
├─ Topic、Service、Action
├─ TF 座標系
├─ robot_state_publisher
├─ ros2_control
├─ RViz、rosbag
├─ 高階規劃與命令
└─ 真機驅動與模擬器之間的統一介面
```

典型資料流：

```text
ROS 2 控制器或 RL policy
          │
          │ 關節目標或力矩命令
          ▼
MuJoCo ROS 2 bridge
          │
          ├─ 寫入 data.ctrl
          ├─ mujoco.mj_step()
          ├─ 讀取 qpos、qvel
          └─ 讀取 IMU、接觸
          │
          ▼
ROS 2 topics、TF、RViz、rosbag
```

### 20.1 建議使用哪一版 ROS 2？

ROS 2 與 MuJoCo 的整合在 Linux 最成熟。對新專案，實務上可考慮：

- ROS 2 Jazzy：仍受支援，生態較穩定，適合 Ubuntu 24.04 長期專案。
- ROS 2 Kilted：目前較新的穩定版本，但非長期支援版。
- Rolling：開發版本，不建議作為初學與機器人部署基線。

目前專案位於 Windows。可以選擇：

1. Windows 原生 ROS 2，加上目前的 Windows MuJoCo Python 環境。
2. WSL2 Ubuntu 24.04 安裝 ROS 2 Jazzy，再於 WSL2 建立另一套 MuJoCo 環境。
3. 將 ROS 2 與 MuJoCo 全部放在原生 Ubuntu。

若目標包含 `ros2_control`、RViz、真機部署與大量 ROS 套件，推薦第 2 或第 3 種。不要直接把 Windows `.venv` 複製到 WSL2；Windows 與 Linux 必須各自建立虛擬環境。

---

### 20.2 最小整合方式：Python ROS 2 bridge

對目前這個 Python MuJoCo 專案，最快的做法是建立一個 `rclpy` node，讓這個 node：

- 擁有 `MjModel` 與 `MjData`
- 接收 ROS 2 馬達命令
- 執行 `mj_step()`
- 發布 `JointState`
- 發布 `Imu`
- 發布 `/clock`
- 提供 reset service

建議 topic：

| ROS 2 名稱 | 型別 | 方向 | 用途 |
|---|---|---|---|
| `/joint_states` | `sensor_msgs/msg/JointState` | MuJoCo 發布 | 關節角與角速度 |
| `/imu/data` | `sensor_msgs/msg/Imu` | MuJoCo 發布 | 軀幹姿態、角速度、加速度 |
| `/two_leg/effort_command` | `std_msgs/msg/Float64MultiArray` | MuJoCo 訂閱 | 四個關節力矩 |
| `/two_leg/contact_state` | 自訂 message 或 array | MuJoCo 發布 | 左右足接觸 |
| `/clock` | `rosgraph_msgs/msg/Clock` | MuJoCo 發布 | 模擬時間 |
| `/two_leg/reset` | `std_srvs/srv/Trigger` | Service | 重設模擬 |

第一版可用 `Float64MultiArray`。當專案開始擴大，建議改成帶有 joint name、控制模式與時間戳的自訂 message，避免陣列順序錯誤。

---

### 20.3 ROS 2 package 結構

在 ROS 2 workspace 中建立 Python package：

```bash
mkdir -p ~/two_leg_ws/src
cd ~/two_leg_ws/src

ros2 pkg create two_leg_mujoco_bridge \
  --build-type ament_python \
  --dependencies rclpy sensor_msgs std_msgs std_srvs rosgraph_msgs
```

建議結構：

```text
two_leg_ws/
└─ src/
   └─ two_leg_mujoco_bridge/
      ├─ package.xml
      ├─ setup.py
      ├─ resource/
      ├─ launch/
      │  └─ mujoco_bridge.launch.py
      ├─ config/
      │  └─ bridge.yaml
      ├─ models/
      │  └─ two_wheel_legged.xml
      └─ two_leg_mujoco_bridge/
         ├─ __init__.py
         └─ bridge_node.py
```

MuJoCo Python 套件必須安裝在執行 ROS 2 node 的同一個 Python 環境中：

```bash
python3 -m pip install mujoco numpy
```

如果 ROS 2 使用系統 Python，而 MuJoCo 裝在另一個虛擬環境，node 可能出現：

```text
ModuleNotFoundError: No module named 'mujoco'
```

此時要確認：

```bash
which python3
python3 -c "import mujoco; print(mujoco.__version__)"
```

---

### 20.4 Bridge node 核心範例

以下是教學用骨架。正式版本還應補上 QoS、例外處理、參數檔與完整 covariance：

```python
from pathlib import Path

import mujoco
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from builtin_interfaces.msg import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger


JOINT_NAMES = [
    "left_hip",
    "right_hip",
    "left_knee",
    "right_knee",
]


def seconds_to_time(value: float) -> Time:
    seconds = int(value)
    nanoseconds = int((value - seconds) * 1_000_000_000)
    return Time(sec=seconds, nanosec=nanoseconds)


class MujocoBridge(Node):
    def __init__(self):
        super().__init__("mujoco_bridge")

        model_path = Path(
            self.declare_parameter(
                "model_path",
                "models/two_wheel_legged.xml",
            ).value
        )

        self.model = mujoco.MjModel.from_xml_path(str(model_path))
        self.data = mujoco.MjData(self.model)
        mujoco.mj_resetDataKeyframe(self.model, self.data, 0)
        mujoco.mj_forward(self.model, self.data)

        # physics 500 Hz，ROS control/publish 50 Hz
        self.frame_skip = 10
        self.control_period = self.model.opt.timestep * self.frame_skip

        self.command = np.zeros(self.model.nu)

        self.joint_pub = self.create_publisher(
            JointState,
            "/joint_states",
            qos_profile_sensor_data,
        )
        self.imu_pub = self.create_publisher(
            Imu,
            "/imu/data",
            qos_profile_sensor_data,
        )
        self.clock_pub = self.create_publisher(Clock, "/clock", 10)

        self.command_sub = self.create_subscription(
            Float64MultiArray,
            "/two_leg/effort_command",
            self.command_callback,
            1,
        )

        self.reset_service = self.create_service(
            Trigger,
            "/two_leg/reset",
            self.reset_callback,
        )

        self.timer = self.create_timer(
            self.control_period,
            self.control_callback,
        )

    def command_callback(self, message):
        command = np.asarray(message.data, dtype=np.float64)

        if command.shape != (self.model.nu,):
            self.get_logger().warning(
                f"Expected {self.model.nu} commands, got {command.size}"
            )
            return

        self.command[:] = np.clip(command, -60.0, 60.0)

    def reset_callback(self, request, response):
        mujoco.mj_resetDataKeyframe(self.model, self.data, 0)
        mujoco.mj_forward(self.model, self.data)
        self.command[:] = 0.0

        response.success = True
        response.message = "MuJoCo simulation reset"
        return response

    def control_callback(self):
        for _ in range(self.frame_skip):
            self.data.ctrl[:] = self.command
            mujoco.mj_step(self.model, self.data)

        stamp = seconds_to_time(self.data.time)
        self.publish_clock(stamp)
        self.publish_joint_state(stamp)
        self.publish_imu(stamp)

    def publish_clock(self, stamp):
        message = Clock()
        message.clock = stamp
        self.clock_pub.publish(message)

    def publish_joint_state(self, stamp):
        message = JointState()
        message.header.stamp = stamp
        message.name = JOINT_NAMES
        message.position = [
            float(self.data.joint(name).qpos[0])
            for name in JOINT_NAMES
        ]
        message.velocity = [
            float(self.data.joint(name).qvel[0])
            for name in JOINT_NAMES
        ]
        message.effort = self.data.ctrl.astype(float).tolist()
        self.joint_pub.publish(message)

    def publish_imu(self, stamp):
        quaternion = self.data.sensor("imu_orientation").data
        gyro = self.data.sensor("imu_gyro").data
        acceleration = self.data.sensor("imu_accelerometer").data

        message = Imu()
        message.header.stamp = stamp
        message.header.frame_id = "imu"

        # MuJoCo quaternion 順序是 w, x, y, z；
        # ROS geometry_msgs 順序欄位是 x, y, z, w。
        message.orientation.w = float(quaternion[0])
        message.orientation.x = float(quaternion[1])
        message.orientation.y = float(quaternion[2])
        message.orientation.z = float(quaternion[3])

        message.angular_velocity.x = float(gyro[0])
        message.angular_velocity.y = float(gyro[1])
        message.angular_velocity.z = float(gyro[2])

        message.linear_acceleration.x = float(acceleration[0])
        message.linear_acceleration.y = float(acceleration[1])
        message.linear_acceleration.z = float(acceleration[2])

        self.imu_pub.publish(message)


def main():
    rclpy.init()
    node = MujocoBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

這個骨架使用 wall timer 觸發模擬，每次 callback 執行 10 個 0.002 秒的 physics step，所以 ROS 層約為 50 Hz。

若需要：

- 比即時更快的訓練
- 暫停、單步、倍速
- 高穩定度 500 Hz 控制

應把 physics loop 放到獨立執行緒或 C++ node，而不是只依賴 Python wall timer。

---

### 20.5 模擬時間 `/clock`

ROS 2 node 預設使用電腦 wall clock。但模擬器可能：

- 跑得比真實時間快
- 跑得比真實時間慢
- 暫停
- reset 回到 0 秒

因此 MuJoCo bridge 應發布：

```text
/clock
```

其他 ROS 2 nodes 設定：

```yaml
use_sim_time: true
```

命令列也可設定：

```bash
ros2 param set /robot_state_publisher use_sim_time true
```

時間戳應來自：

```python
self.data.time
```

而不是 `time.time()`。否則 rosbag、TF、感測器資料可能互相對不上。

Reset 時模擬時間倒退，某些控制器、filter 或 TF buffer 可能保留舊資料。正式系統應在 reset 時同步重設控制器、observation history 與 policy recurrent state。

---

### 20.6 關節狀態、URDF 與 TF

MuJoCo 使用 MJCF 描述物理模型；ROS 2 常用 URDF/Xacro 描述 link、joint 與 TF。

整合時通常保留兩份用途不同的模型：

```text
MJCF
└─ MuJoCo 物理、接觸、actuator、sensor

URDF/Xacro
└─ ROS 2 robot_description、TF、RViz、ros2_control
```

兩份模型的以下項目必須一致：

- link 名稱
- joint 名稱
- parent/child 關係
- joint axis
- joint zero position
- joint limits
- mesh 座標與單位

MuJoCo 可以直接載入 URDF，但 URDF 的表達能力比 MJCF 有限。雙足接觸、sensor、default class 與複雜 actuator 通常仍建議在 MJCF 中維護。

ROS 2 的 `robot_state_publisher`：

1. 讀取 `robot_description`
2. 訂閱 `/joint_states`
3. 發布各 link 的 TF

資料流：

```text
MuJoCo data.qpos
       ↓
/joint_states
       ↓
robot_state_publisher + URDF
       ↓
/tf
       ↓
RViz
```

注意：浮動軀幹的世界姿態不能只靠普通 revolute joint 的 `JointState` 表達。可以由 bridge 額外發布：

```text
world → torso
```

的 dynamic TF，位置來自 root freejoint，姿態來自 root quaternion。

---

### 20.7 座標系與 quaternion

本 MuJoCo 模型採用：

```text
+Z 向上
+X 向前
```

ROS REP-103 常見機器人座標也使用：

```text
x forward
y left
z up
```

但仍需確認模型左右腳的 Y 軸方向、IMU frame 和 mesh 原始座標。

特別注意 quaternion：

```text
MuJoCo array：w, x, y, z
ROS message：x, y, z, w 欄位
```

不能直接把四個陣列元素依原順序塞進 ROS message。

還要確認 IMU sensor 輸出是以哪個 local frame 表示。建議在模型靜止、水平放置時檢查：

- quaternion 是否接近單位姿態
- gyro 是否接近 0
- accelerometer 的重力方向是否符合控制器定義

---

### 20.8 控制命令應該傳位置還是力矩？

有三種常見介面。

#### 方式 A：ROS 2 傳力矩

```text
ROS policy/controller → effort command → data.ctrl
```

優點：

- 最直接
- 適合 torque policy
- 適合控制研究

缺點：

- 對延遲與頻率敏感
- 真機風險較高
- policy 必須自己學會低階穩定控制

#### 方式 B：ROS 2 傳關節目標，MuJoCo bridge 做 PD

```text
ROS policy → q_target → bridge PD → torque → data.ctrl
```

例如：

```python
torque = kp * (q_target - q) - kd * qvel
data.ctrl[:] = np.clip(torque, -max_torque, max_torque)
```

這是目前雙足專案推薦的第一版：

- ROS / policy：50 Hz
- MuJoCo physics：500 Hz
- PD：500 Hz

#### 方式 C：ROS 2 傳 trajectory

```text
JointTrajectory
       ↓
joint_trajectory_controller
       ↓
position/velocity/effort command
```

適合：

- 預先規劃動作
- 起身、蹲下等固定軌跡
- 與 MoveIt 2 或一般 ROS 控制器整合

對動態平衡雙足，trajectory controller 通常只是系統的一部分，不能取代平衡控制器。

---

### 20.9 與 `ros2_control` 的正式整合

當目標是讓「模擬器與真機共用相同控制器」時，應考慮 `ros2_control`。

`ros2_control` 的主要流程：

```text
Hardware interface read()
          ↓
Controller Manager update()
          ↓
Hardware interface write()
```

套用到 MuJoCo：

```text
read()
├─ data.qpos → position state interface
└─ data.qvel → velocity state interface

controller update()
└─ 根據 state 計算 command

write()
└─ effort command → data.ctrl

simulation loop
└─ mujoco.mj_step()
```

需要實作一個 C++ `hardware_interface::SystemInterface` plugin，例如：

```text
two_leg_mujoco_hardware/MujocoSystem
```

URDF/Xacro 中加入：

```xml
<ros2_control name="TwoLegMujocoSystem" type="system">
  <hardware>
    <plugin>two_leg_mujoco_hardware/MujocoSystem</plugin>
    <param name="model_path">/path/to/two_wheel_legged.xml</param>
  </hardware>

  <joint name="left_hip">
    <command_interface name="effort">
      <param name="min">-60</param>
      <param name="max">60</param>
    </command_interface>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>

  <joint name="right_hip">
    <command_interface name="effort">
      <param name="min">-60</param>
      <param name="max">60</param>
    </command_interface>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>

  <joint name="left_knee">
    <command_interface name="effort">
      <param name="min">-60</param>
      <param name="max">60</param>
    </command_interface>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>

  <joint name="right_knee">
    <command_interface name="effort">
      <param name="min">-60</param>
      <param name="max">60</param>
    </command_interface>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
</ros2_control>
```

Controller Manager YAML 概念：

```yaml
controller_manager:
  ros__parameters:
    update_rate: 500

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    effort_controller:
      type: forward_command_controller/ForwardCommandController

effort_controller:
  ros__parameters:
    joints:
      - left_hip
      - right_hip
      - left_knee
      - right_knee
    interface_name: effort
```

這樣上層控制器面對的是標準 ROS 2 command/state interface。之後將 MuJoCo hardware plugin 換成真機 CAN、EtherCAT 或 serial hardware plugin，上層控制器可以盡量保持不變。

---

### 20.10 為什麼不能只靠 Topic 做 500 Hz 硬即時控制？

Python ROS 2 topic 適合原型，但不保證硬即時：

- Python garbage collection
- executor scheduling
- DDS serialization
- 作業系統 scheduling jitter
- Viewer 與 logging 造成延遲

推薦頻率分層：

```text
MuJoCo physics                 500～1000 Hz
低階 PD／力矩限制             500～1000 Hz
RL policy                     25～100 Hz
狀態發布、視覺化              20～100 Hz
RViz                           10～30 Hz
```

高頻 loop 中不要：

- 大量 `print`
- 每一步發布所有 ROS topics
- 每一步更新 Viewer
- 每一步寫檔
- 配置大型 NumPy array

控制與 physics 最好在同一 process 或共享記憶體中執行；ROS 2 用於較低頻的命令、狀態、監控與模組整合。

---

### 20.11 強化學習應該怎麼接 ROS 2？

分成訓練與部署。

#### 離線訓練

```text
Gymnasium
    ↕ 直接 Python function call
MuJoCo
    ↕
Stable-Baselines3 PPO
```

訓練 inner loop 不建議經過 ROS 2 topic，原因是：

- DDS 增加延遲
- 序列化降低速度
- vectorized environments 較難
- 訓練通常需要跑得比即時快

ROS 2 可在訓練時負責外部監控，但不要放在每個 environment step 的必要路徑。

#### Policy 部署

```text
/joint_states + /imu/data
          ↓
ROS 2 policy node
          ↓
observation normalization
          ↓
PPO model.predict()
          ↓
/two_leg/joint_target
          ↓
低階 PD controller
          ↓
MuJoCo 或真機
```

Policy node 概念：

```python
from stable_baselines3 import PPO

self.policy = PPO.load("two_leg_ppo.zip", device="cpu")

action, self.recurrent_state = self.policy.predict(
    observation,
    deterministic=True,
)
```

部署時必須保存並重現：

- observation 欄位順序
- observation scaling
- action scaling
- frame stacking
- policy frequency
- PD gains
- joint name 順序
- quaternion 定義

如果訓練使用 `VecNormalize`，部署時也要載入同一組統計量。漏掉這一步是 policy 在訓練環境正常、進入 ROS 2 後立刻失效的常見原因。

---

### 20.12 ROS 2 QoS 建議

感測器資料通常重視最新值：

```python
from rclpy.qos import qos_profile_sensor_data
```

它適合：

- IMU
- 高頻 joint state
- 接觸資訊

控制命令通常建議：

- queue depth 小，例如 1
- 不累積過期命令
- 加入 command timeout

例如超過 100 ms 沒收到新命令：

```python
if command_age > 0.1:
    self.command[:] = 0.0
```

真機應進一步切換到安全姿勢或停用馬達，不能只依靠最後一筆 ROS command。

---

### 20.13 Reset、Pause 與 Step Service

建議 bridge 提供：

```text
/two_leg/reset
/two_leg/pause
/two_leg/unpause
/two_leg/step
```

用途：

- RL episode reset
- 控制器除錯
- 在固定狀態檢查 sensor
- 重現單一步驟問題

Reset 時應同步清除：

- `MjData`
- 上一筆 command
- PD integrator（如果有 I 項）
- observation history
- action history
- filter state
- recurrent policy hidden state
- episode reward counter

---

### 20.14 rosbag、RViz 與除錯

記錄資料：

```bash
ros2 bag record \
  /joint_states \
  /imu/data \
  /two_leg/effort_command \
  /two_leg/contact_state \
  /tf \
  /tf_static \
  /clock
```

檢查 topic：

```bash
ros2 topic list
ros2 topic hz /joint_states
ros2 topic echo /imu/data
ros2 topic info /two_leg/effort_command
```

檢查 node 與參數：

```bash
ros2 node list
ros2 param get /robot_state_publisher use_sim_time
```

檢查 `ros2_control`：

```bash
ros2 control list_controllers
ros2 control list_hardware_components
ros2 control list_hardware_interfaces
```

RViz 中至少加入：

- RobotModel
- TF
- Imu（或自行轉成 Marker）
- 足底接觸 Marker

RViz 顯示的是 ROS URDF 與 TF；MuJoCo Viewer 顯示的是 MJCF。因此兩邊外觀或姿態不同時，優先比對 joint names、axis、zero offset 與 root transform。

---

### 20.15 ROS 2 整合的推薦開發順序

第一階段：只發布狀態

```text
MuJoCo → /joint_states、/imu/data、/clock
```

確認：

- topic 頻率
- 時間戳
- quaternion
- RViz 姿態

第二階段：加入命令 topic

```text
/joint_target → PD → data.ctrl
```

先用固定站姿，不要立刻接 RL。

第三階段：加入 reset、pause 與 timeout

確保：

- command 中斷會安全停止
- reset 不會留下舊命令
- episode 可重現

第四階段：部署 RL policy node

確認 ROS observation 與 Gymnasium observation 逐元素相同。

第五階段：導入 `ros2_control`

讓模擬器與真機共享：

- controller configuration
- joint interfaces
- topic/service contract
- 高階控制器

第六階段：真機硬體 plugin

將 MuJoCo hardware interface 替換為真實馬達通訊，並保留力矩、角度、速度、溫度與通訊 timeout 等安全限制。

---

## 21. 官方參考資料

- MuJoCo Python API：<https://mujoco.readthedocs.io/en/stable/python.html>
- MuJoCo Modeling：<https://mujoco.readthedocs.io/en/stable/modeling.html>
- MuJoCo Simulation：<https://mujoco.readthedocs.io/en/stable/programming/simulation.html>
- MJCF XML Reference：<https://mujoco.readthedocs.io/en/stable/XMLreference.html>
- Gymnasium MuJoCo：<https://gymnasium.farama.org/environments/mujoco/>
- Stable-Baselines3 PPO：<https://stable-baselines3.readthedocs.io/en/master/modules/ppo.html>
- ROS 2 文件：<https://docs.ros.org/>
- ros2_control Getting Started：<https://control.ros.org/jazzy/doc/getting_started/getting_started.html>
- ros2_control Controller Manager：<https://control.ros.org/jazzy/doc/ros2_control/controller_manager/doc/userdoc.html>
- ros2_control Simulator Integrations：<https://control.ros.org/jazzy/doc/simulators/simulators.html>
