"""Open the integrated GUI at the DengFOC Library current-control page."""

import sys

import pyqtgraph as pg
from PyQt5.QtWidgets import QApplication

from integrated_robot_control_gui import IntegratedRobotControlGUI


def create_window() -> IntegratedRobotControlGUI:
    window = IntegratedRobotControlGUI()
    window.setWindowTitle("DengFOC Library Current Control GUI")
    for index in range(window.tabs.count()):
        if window.tabs.tabText(index) == "FOC Diagnostic":
            window.tabs.setCurrentIndex(index)
            break
    else:
        raise RuntimeError("FOC Diagnostic tab is unavailable")
    window.statusBar().showMessage(
        "Connect DengFOC A/B, apply SAFE, then align M1 and M2 before torque control."
    )
    return window


def main() -> None:
    app = QApplication(sys.argv)
    pg.setConfigOptions(antialias=True)
    window = create_window()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
