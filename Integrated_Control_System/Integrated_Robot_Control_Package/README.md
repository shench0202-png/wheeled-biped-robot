# ESP32、DengFOC 與 Python GUI

本資料夾整理雙輪足實機控制程式：`Servo_motor_Integrated_GUI` 負責 IMU、姿態估測與腿部伺服；DengFOC 韌體負責雙輪馬達；`Python_GUI` 用於監控、調參與命令轉送。

`Arduino_Firmware` 中的兩種 DengFOC 韌體是替代版本，使用時擇一燒錄。整機平衡效果仍需實機驗證。

`DengFOC_Library_Current_Control_Integrated_GUI` 對應的 GUI 入口為 `Python_GUI/dengfoc_library_current_gui.py`，可直接開啟 FOC 診斷頁；操作方式見該韌體資料夾的 README。
