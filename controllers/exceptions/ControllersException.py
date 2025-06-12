#!/usr/bin/env python3
# =============================================================================
#  ControllersException.py
#
#  Description : Custom exception classes for controller hardware errors and warnings.
#
#  Author      : Antoine BLASIAK <antoineblasiak66@gmail.com>
#  Copyright   : (c) 2025 Antoine BLASIAK
#  License     : MIT License
#  Repository  : https://github.com/abpydev/LouCOMAX_Controllers
#
#  This file is part of the LouCOMAX Controllers project from LAB-BC research unit of CNRS.
#  See the LICENSE file for more details.
# =============================================================================

# built-in imports
import os
from pathlib import Path

# PyQT imports
from PyQt5.QtWidgets import QMessageBox
from PyQt5.QtGui import QIcon

ASSETS_PATH = Path(os.path.dirname(os.path.abspath(__file__))) / "assets"

class ControllerNotConnected(Exception):

    def __init__(self, name="Unknown controller") -> None:
        text = f"{name} not connected"

        dlg = QMessageBox()
        dlg.setWindowTitle("Controller Connection ERROR")
        error_icon_path = ASSETS_PATH / "icons" / "error.png"
        dlg.setWindowIcon(QIcon(error_icon_path.as_posix()))
        dlg.setText(text)
        dlg.exec()

class ControllerWarningMsg(Exception):

    def __init__(self, text:str) -> None:

        dlg = QMessageBox()
        dlg.setWindowTitle("Controller WARNING message")
        warning_icon_path = ASSETS_PATH / "icons" / "warning.png"
        dlg.setWindowIcon(QIcon(warning_icon_path.as_posix()))
        dlg.setText(text)
        dlg.exec()

class ControllerErrorMsg(Exception):

    def __init__(self, text:str) -> None:

        dlg = QMessageBox()
        dlg.setWindowTitle("Controller ERROR message")
        warning_icon_path = ASSETS_PATH / "icons" / "warning.png"
        dlg.setWindowIcon(QIcon(warning_icon_path.as_posix()))
        dlg.setText(text)
        dlg.exec()

class ZaberNotConnected(ControllerNotConnected):
    def __init__(self, *args: object) -> None:
        super().__init__(name="Zaber positionning stages")

class AmptekMaxrfNotConnected(ControllerNotConnected):
    def __init__(self, *args: object) -> None:
        super().__init__(name="Amptek MAXRF sensor")

class AmptekCxrfNotConnected(ControllerNotConnected):
    def __init__(self, *args: object) -> None:
        super().__init__(name="Amptek CXRF sensor")

class LaserNotConnected(ControllerNotConnected):
    def __init__(self, *args: object) -> None:
        super().__init__(name="Telemetry Laser Sensor")

class CSUNotConnected(ControllerNotConnected):
    def __init__(self, *args: object) -> None:
        super().__init__(name="CSU for RX control")

class CSUCantOpenShutterError(ControllerWarningMsg):
    def __init__(self, *args: object) -> None:
        csu_error = args[0]
        super().__init__(text=f"Could not open the RX Shutter.\nCheck CSU control panel for Errors or retart the CSU hardware\n{csu_error}")

class CSURxNotWarmedUp(ControllerWarningMsg):
    def __init__(self, *args: object) -> None:
        super().__init__(text="RX Tube was not warmed up yet.\nPlease power ON the RX Tube's High Voltage and wait for warmup")

class LaserOutOfRangeWarning(ControllerWarningMsg):
    def __init__(self, *args: object) -> None:
        text = "Distance LASER sensor is disconnected\
\nor\n\
No Z distance target, please reach manualy the wanted distance then click on 'save Z'\
\nor\n\
The distance LASER is out of bounds or disconnected.\
Please reach manualy inside the working range before activating Z control.\
\nor\n\
The current Z distance is too far from the target 'saved z'.\
Please reach manualy closer to the target before activating Z control."
        super().__init__(text)

class MappingOutOfBoundsError(ControllerErrorMsg):

    def __init__(self, text:str) -> None:

        super().__init__(text)