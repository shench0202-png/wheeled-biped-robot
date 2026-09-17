# DengFOC Library 電流回授控制

本韌體在 ESP32 上控制兩顆無刷馬達，讀取 AS5600 角度和兩路相電流，以電流 PID 控制扭矩；也提供速度、位置控制。它使用 DengFOC V0.6 的 SVPWM 控制邏輯移植程式，並不需要另外安裝同名 Arduino 函式庫。入口為 `DengFOC_Library_Current_Control_Integrated_GUI.ino`。

## 對應 GUI

使用 `../../Python_GUI/dengfoc_library_current_gui.py`。這個入口沿用整合 GUI 的通訊、圖表和紀錄功能，啟動時直接顯示 **FOC Diagnostic** 頁面；同一視窗也保留 IMU、手動馬達控制、PID、LQR 和模糊控制頁面。需要 Python 3.10+、PyQt5、pyqtgraph、pyserial；3D 顯示需要 numpy。

在 `Integrated_Robot_Control_Package` 目錄執行：

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install PyQt5 pyqtgraph pyserial numpy
.\.venv\Scripts\python.exe .\Python_GUI\dengfoc_library_current_gui.py
```

選擇 DengFOC A 的 COM 埠並連線；第二塊板子可選 DengFOC B。每塊板子各有 M1、M2。韌體與 GUI 透過 USB Serial、115200 baud 交換帶 XOR checksum 的文字封包（`payload*XX`）。

| GUI 功能 | 對應用途 |
| --- | --- |
| `PING`、`CFG?`、`PID?` | 確認連線，讀取限制與 PID 參數；連線後會自動查詢。 |
| `SAFE`、`DIR` | 設定電壓／電流／逾時限制和馬達方向。變更方向後須重新對齊。 |
| `ALIGN` | 分別對齊 M1、M2；未完成對齊的馬達不能進入一般閉迴路控制。 |
| `TESTIQ`、`OPEN`、`PHASE` | 限時檢查電流回授、開環轉動和相位。 |
| `ESTOP`、`CLR`、`ZERO` | 停止輸出、清除故障和設定角度零點。 |
| 圖表、狀態、CSV | 讀取 `TQC2` 馬達狀態、`FDD` 即時診斷及 `DIAG` 測試結果。 |

首次測試請先讓車輪離地，核對供電與接線；連線後設定 `SAFE`，逐顆執行 `ALIGN`，確認診斷資料與方向，再啟用扭矩或平衡控制。韌體有電流／電壓限制、通訊逾時及急停，但整機實機平衡尚未驗證。
