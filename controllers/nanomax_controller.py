"""This module integrates classes for control of NANOMAX300 device
through the BSC203 controller (Thorlabs Devices)"""

__all__ = ["ZaberController"]

# Standard imports
import os
import configparser
import time
from typing import Literal
import logging
import clr

# DLL handling
clr.AddReference(os.path.join(os.path.dirname(__file__), "bin", "Thorlabs.MotionControl.DeviceManagerCLI.dll"))
clr.AddReference(os.path.join(os.path.dirname(__file__), "bin", "Thorlabs.MotionControl.GenericMotorCLI.dll"))
clr.AddReference(os.path.join(os.path.dirname(__file__), "bin", "ThorLabs.MotionControl.Benchtop.StepperMotorCLI.dll"))

# Imports from Thorlabs DLLs
from Thorlabs.MotionControl.DeviceManagerCLI import *
from Thorlabs.MotionControl.GenericMotorCLI import *
from Thorlabs.MotionControl.Benchtop.StepperMotorCLI import *
from System import Decimal  # necessary for real world units

# Sub-modules imports
from .abstractcontroller import AbsController

# logger configuration
logger = logging.getLogger(f'core.{__name__}')

class NanoMaxController(AbsController):
    """
    This class provides an interface for controlling Thorlabs NanoMax300 stages
    connected to a BSC203 benchtop controller. It manages the connection,
    configuration, and movement of the X, Y, and Z axes, allowing for absolute
    and relative positioning, homing, and stopping of individual or all axes.

    Attributes:
        - _NANOMAX300_CHAN_ALLOC_MAP (dict): Maps controller channels to stage axes.
        - _NANOMAX300_ORIENTATION (dict): Stores orientation (+/-) for each axis.
        - controllers_config (configparser.ConfigParser): User-defined configuration.
        - axis_control (dict): Holds references to axis control objects for x, y, z.
        - connexion_established (bool): Connection status with the controller.

    Methods:
        - __init__(controllers_config):
            Initializes the controller with user configuration and prepares axis control.
        - start_connection():
            Establishes connection to the NanoMax300 stages via the BSC203 controller,
            initializes axis allocation, and enables device polling.
        - stop_connection():
            Disconnects from the controller and updates connection status.
        - is_connected() -> bool:
            Returns the current connection status.
        - move_to(direction, target_mm, move_timeout=60000):
            Moves the specified axis to an absolute position in millimeters.
        - move_relative_forward(direction, step_mm, move_timeout=60000):
            Moves the specified axis forward by a relative step in millimeters.
        - move_relative_backward(direction, step_mm, move_timeout=60000):
            Moves the specified axis backward by a relative step in millimeters.
        - home_axis(direction, home_timeout=60000):
            Homes the specified axis.
        - stop_movement(direction):
            Stops movement of the specified axis.
        - is_all_axes_homed() -> bool:
            Checks if all axes are homed.
        - home_all_axes(timeout=60000):
            Homes all axes sequentially.
        - get_xyz_pos() -> tuple[float, float, float]:
            Returns the current positions of all axes as a tuple.
        - get_dir_pos(direction) -> int:
            Returns the stored position of the specified axis.
            (Internal methods for configuration and axis allocation are also provided.)"""

    _NANOMAX300_CHAN_ALLOC_MAP =  {
        "1" : "y",
        "2" : "x",
        "3" : "z"
    }
    _NANOMAX300_ORIENTATION = {
        "x" : 0,
        "y" : 0,
        "z" : 0
    }

    def __init__(self, controllers_config:configparser.ConfigParser):

        # Configuration defined by the User in .cfg file
        self.controllers_config = controllers_config
        NanoMaxController._update_config(controllers_config)

        # Axis control (zaber axis)
        self.axis_control = {
            "x" : None,
            "y" : None,
            "z" : None
        }
        self.connexion_established = False

    def start_connection(self) -> None:
        """
        Start connection with NanoMax300 stages via the BSC203 benchtop controller
        """

        NanoMaxController._update_config(self.controllers_config)

        # Detect Thorlabs devices
        DeviceManagerCLI.BuildDeviceList()

        # Create new device
        # Connect, begin polling, and enable
        self.serial_no = "70470634"  # BSC203 benchtop controller serial number
        self.bench_controller_device = BenchtopStepperMotor.CreateBenchtopStepperMotor(self.serial_no)
        try :
            self.bench_controller_device.Connect(self.serial_no)
        except Exception as e :
            logger.exception('Error while connecting to the NanoMax300 device : %s', e)
            raise ConnectionError("Error while connecting to the NanoMax300 device") from e
        time.sleep(0.25)  # wait statements are important to allow settings to be sent to the device

        self._init_axis_allocation()

        self.connexion_established = self.bench_controller_device.IsConnected

    def stop_connection(self):
        self.bench_controller_device.Disconnect(False)
        self.connexion_established = self.bench_controller_device.IsConnected

    def is_connected(self) -> bool:
        if hasattr(self, "bench_controller_device") :
            self.connexion_established = self.bench_controller_device.IsConnected
        return super().is_connected()

    @classmethod
    def _update_config(cls, config:configparser.ConfigParser) -> None:
        """Reading and interpreting the config file .cfg"""
        cls._update_alloc_map(config)
        cls._update_orientation(config)

    @classmethod
    def _update_alloc_map(cls, config:configparser.ConfigParser):
        """Reading and interpreting the config file .cfg
        
            Parameters related to :

                -   Allocation of each channel to axes X, Y and Z
        """
        try :
            cls._NANOMAX300_CHAN_ALLOC_MAP["1"] = config.get('NanoMax300', 'CHANNEL_1')
            cls._NANOMAX300_CHAN_ALLOC_MAP["2"]  = config.get('NanoMax300', 'CHANNEL_2')
            cls._NANOMAX300_CHAN_ALLOC_MAP["3"]  = config.get('NanoMax300', 'CHANNEL_3')
        except Exception as e :
            logger.exception('Error while reading Controllers.cfg file : %s', e)
            raise ValueError("Error while reading Controllers.cfg file") from e

    @classmethod
    def _update_orientation(cls, config:configparser.ConfigParser):
        """Reading and interpreting the config file .cfg
        
            Parameters related to :

                -   Orientation (+/-) of each axis 
        """
        try :
            cls._NANOMAX300_ORIENTATION["x"] = int(config.getint('NanoMax300',
                                                    'NANOMAX300_ORIENTATION_X'))
            cls._NANOMAX300_ORIENTATION["y"] = int(config.getint('NanoMax300',
                                                    'NANOMAX300_ORIENTATION_Y'))
            cls._NANOMAX300_ORIENTATION["z"] = int(config.getint('NanoMax300',
                                                    'NANOMAX300_ORIENTATION_Z'))

        except Exception as e :
            logger.exception('Error while reading Controllers.cfg file : %s', e)
            raise ValueError("Error while reading Controllers.cfg file") from e

    def _init_axis_allocation(self) -> None :
        # print(f'{self._NANOMAX300_CHAN_ALLOC_MAP=}')
        for index, val in self._NANOMAX300_CHAN_ALLOC_MAP.items():
            self.axis_control[val] = self.bench_controller_device.GetChannel(int(index))

        for stage_axis in self.axis_control.values():
            # Ensure that the device settings have been initialized
            if not stage_axis.IsSettingsInitialized():
                stage_axis.WaitForSettingsInitialized(10000)  # 10 second timeout
                assert stage_axis.IsSettingsInitialized() is True

            # Start polling and enable
            stage_axis.StartPolling(250)  # 250ms polling rate
            time.sleep(0.5)
            stage_axis.EnableDevice()
            time.sleep(0.25)  # Wait for device to enable

            # Load any configuration settings needed by the controller/stage
            channel_config = stage_axis.LoadMotorConfiguration(stage_axis.DeviceID)
            chan_settings = stage_axis.MotorDeviceSettings

            stage_axis.GetSettings(chan_settings)

            channel_config.DeviceSettingsName = 'NanoMax300'

            channel_config.UpdateCurrentConfiguration()

            stage_axis.SetSettings(chan_settings, True, False)

    def move_to(self, direction:Literal["x", "y", "z"], target_mm:float, move_timeout=60000):
        axis_control = self.axis_control[direction]
        axis_control.MoveTo(Decimal(target_mm), move_timeout)

    def move_relative_forward(self, direction:Literal["x", "y", "z"], step_mm:float, move_timeout=60000):
        axis_control = self.axis_control[direction]
        axis_control.MoveRelative(MotorDirection.Forward, Decimal(step_mm), move_timeout)

    def move_relative_backward(self, direction:Literal["x", "y", "z"], step_mm:float, move_timeout=60000):
        axis_control = self.axis_control[direction]
        axis_control.MoveRelative(MotorDirection.Backward, Decimal(step_mm), move_timeout)

    def home_axis(self, direction:Literal["x", "y", "z"], home_timeout=60000):
        self.axis_control[direction].Home(home_timeout)

    def stop_movement(self, direction:Literal["x", "y", "z"]) -> None:
        """Stop the movement of a given axis"""

        logger.debug(f'Stop {direction} movement')
        match direction:
            case "x":
                self._stop_x()
            case "y":
                self._stop_y()
            case "z":
                self._stop_z()
            case _:
                raise KeyError(f"Wrong direction provided to stop_movement() method, therefore emergency_stop() wass called.\tdirection entered : {direction}")

    def is_all_axes_homed(self):
        """Check if all axes are homed
        Returns False if any axis is not homed"""
        for idx in range(1, 4):
            channel = self.bench_controller_device.GetChannel(idx)
            if channel.NeedsHoming :
                return False

        return True

    def home_all_axes(self, timeout=60000):
        self.home_axis("x", timeout)
        self.home_axis("y", timeout)
        self.home_axis("z", timeout)

    def _stop_x(self, timeout=60000) -> None:
        """Stop X axis movement"""
        try :
            self.axis_control["x"].Stop(timeout)
        except AttributeError as attrerr:
            raise ConnectionError("The NanoMax300 stage is not connected. Impossible to send STOP X command") from attrerr

    def _stop_y(self, timeout=60000) -> None:
        """Stop Y axis movement"""
        try:
            self.axis_control["y"].Stop(timeout)
        except AttributeError as attrerr:
            raise ConnectionError("The NanoMax300 stage is not connected. Impossible to send STOP Y command") from attrerr

    def _stop_z(self, timeout=60000) -> None:
        """Stop Z axis movement"""
        try:
            self.axis_control["z"].Stop(timeout)
        except AttributeError as attrerr:
            raise ConnectionError("The NanoMax300 stage is not connected. Impossible to send STOP Z command") from attrerr

    def get_xyz_pos(self) -> tuple[float, float, float]:
        """Interogate the device to get all axes positions
        """
        if not self.is_connected():
            return (None, None, None)

        x_pos = str(self.get_dir_pos("x"))
        y_pos = str(self.get_dir_pos("y"))
        z_pos = str(self.get_dir_pos("z"))

        return (x_pos, y_pos, z_pos)

    def get_dir_pos(self, direction:Literal["x", "y", "z"]) -> int:
        """Get the given axis stored position 
        (does not interogate the device)
        """
        return self.axis_control[direction].DevicePosition

    def __repr__(self):
        return (f"<NanoMaxController("
                f"Connected={self.connexion_established}, "
                f"AxisControl={{'x': {self.axis_control['x']}, "
                f"'y': {self.axis_control['y']}, "
                f"'z': {self.axis_control['z']}}}, "
                f"Positions={self.get_xyz_pos()})>")

    def __str__(self):
        return (f"<NanoMaxController(\n"
                f"  Connected={self.connexion_established},\n"
                f"  AxisControl={{'x': {self.axis_control['x']},\n"
                f"               'y': {self.axis_control['y']},\n"
                f"               'z': {self.axis_control['z']}}},\n"
                f"  Positions={self.get_xyz_pos()}\n"
                f")>")

def main():

    config = configparser.ConfigParser()
    config.add_section('NanoMax300')
    config.set('NanoMax300', 'channel_1', 'y')
    config.set('NanoMax300', 'channel_2', 'x')
    config.set('NanoMax300', 'channel_3', 'z')
    config.set('NanoMax300', 'nanomax300_orientation_x', '0')
    config.set('NanoMax300', 'nanomax300_orientation_y', '0')
    config.set('NanoMax300', 'nanomax300_orientation_z', '0')

    nanomax_controller = NanoMaxController(config)
    print(nanomax_controller)

if __name__ == "__main__":

    main()