"""This module integrates classes for control of NANOMAX300 device
through the BSC203 controller (Thorlabs Devices)"""

__all__ = ["ZaberController"]

# Standard imports
import os
import configparser
import time
from typing import Literal
from pathlib import PurePath
import logging
import clr
import numpy

# Imports and DLL handling
clr.AddReference("C:\\Program Files\\Thorlabs\\Kinesis\\Thorlabs.MotionControl.DeviceManagerCLI.dll")
clr.AddReference("C:\\Program Files\\Thorlabs\\Kinesis\\Thorlabs.MotionControl.GenericMotorCLI.dll")
clr.AddReference("C:\\Program Files\\Thorlabs\\Kinesis\\ThorLabs.MotionControl.Benchtop.StepperMotorCLI.dll")
from Thorlabs.MotionControl.DeviceManagerCLI import *
from Thorlabs.MotionControl.GenericMotorCLI import *
from Thorlabs.MotionControl.Benchtop.StepperMotorCLI import *
from System import Decimal  # necessary for real world units

# project sub-modules imports
try :
    from controllers.abstractcontroller import AbsController
    from controllers.amptek_controller import AmptekDevice
    from controllers.utils.hdf5_utils import Hdf5Handler
except ModuleNotFoundError :
    from abstractcontroller import AbsController
    from amptek_controller import AmptekDevice
    from utils.hdf5_utils import Hdf5Handler

# logger configuration
logger = logging.getLogger(f'core.{__name__}')

class NanoMaxController(AbsController):

    """This class aims at controlling an NanoMax """

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

        x_pos = str(self.get_dir_pos("x"))
        y_pos = str(self.get_dir_pos("y"))
        z_pos = str(self.get_dir_pos("z"))

        return (x_pos, y_pos, z_pos)

    def get_dir_pos(self, direction:Literal["x", "y", "z"]) -> int:
        """Get the given axis stored position 
        (does not interogate the device)
        """
        return self.axis_control[direction].DevicePosition


def main_test():

    config = configparser.ConfigParser()
    config_filepath = r"C:\Users\CXRF\Code\depthpaint-c-xrf-interface\corapp\configurations\main_config.cfg"
    config.read(config_filepath)

    nanomax_controller = NanoMaxController(config)
    nanomax_controller.start_connection()

    # print(f'{nanomax_controller.get_xyz_pos()=}')

    # nanomax_controller.home_axis("x")
    # nanomax_controller.home_axis("y")
    # nanomax_controller.home_axis("z")

    print(f'{nanomax_controller.get_xyz_pos()=}')

    nanomax_controller.move_to("x", 2)
    nanomax_controller.move_to("y", 3)
    nanomax_controller.move_to("z", 4)

    print(f'{nanomax_controller.get_xyz_pos()=}')

    nanomax_controller.move_relative_forward("x", 1)
    print(f'{nanomax_controller.get_xyz_pos()=}')
    nanomax_controller.move_relative_forward("x", 1)
    print(f'{nanomax_controller.get_xyz_pos()=}')
    nanomax_controller.move_relative_forward("y", 3)
    print(f'{nanomax_controller.get_xyz_pos()=}')

def cube_scan_origin(amptek_controller: AmptekDevice, dwell_time, start_xyz, roi, x_pixel_size, x_pixel_num, y_pixel_size, y_pixel_num, z_pixel_size, z_pixel_num):
    
    hdf5_filepath = r"C:\Users\CXRF\Code\depthpaint-c-xrf-interface\corapp\tests\results\3d_scan\test_confocal_align.hdf5"

    amptek_controller.start_connection()
    print((x_pixel_num, y_pixel_num, z_pixel_num))
    data_cube = numpy.zeros((x_pixel_num, y_pixel_num, z_pixel_num))

    Hdf5Handler.create_empty_hdf5(hdf5_filepath, data_cube.shape, group_name="Test confocal alignment")

    config = configparser.ConfigParser()
    config_filepath = r"C:\Users\CXRF\Code\depthpaint-c-xrf-interface\corapp\configurations\main_config.cfg"
    config.read(config_filepath)

    nanomax_controller = NanoMaxController(config)
    nanomax_controller.start_connection()

    start_x, start_y, start_z = start_xyz
    print(f'{start_x=}, {start_y=}, {start_z=}')
    
    nanomax_controller.move_to("x", start_x)
    nanomax_controller.move_to("y", start_y)
    nanomax_controller.move_to("z", start_z)

    amptek_controller.enable_mca_mcs()
    time.sleep(0.1)
    maximum = 0
    max_xyz = (0., 0., 0.)
    
    t_start = time.perf_counter()
    
    for x in range(x_pixel_num):
        print(f'Time spent : {time.perf_counter() - t_start}')
        max_chan, int_spectrum, _ = amptek_controller.get_spectrum(get_status=False, clear_spectrum=True)
        for y in range(y_pixel_num):
            max_chan, int_spectrum, _ = amptek_controller.get_spectrum(get_status=False, clear_spectrum=True)
            for z in range(z_pixel_num):
                
                # max_chan, int_spectrum, _ = amptek_controller.get_spectrum(get_status=False, clear_spectrum=True)
                time.sleep(dwell_time / 1000)
                max_chan, int_spectrum, _ = amptek_controller.get_spectrum(get_status=False, clear_spectrum=True)

                current_xyz = nanomax_controller.get_xyz_pos()
                # Change the ROI here
                sum_spectrum = sum(int_spectrum[roi[0] : roi[1]])

                if sum_spectrum > maximum :
                    maximum = sum_spectrum
                    max_xyz = current_xyz
                    print(f'{maximum=}, {max_xyz=}')

                x_index = int(x)
                y_index = int(y)
                z_index = int(z)
                data_cube[x_index,y_index,z_index] = sum_spectrum

                nanomax_controller.move_relative_forward("z", z_pixel_size)
                print(f'{x, y, z}, {maximum=}, {current_xyz=}')

            nanomax_controller.move_relative_forward("y", y_pixel_size)
            nanomax_controller.move_to("z", start_z)

        Hdf5Handler.save_data_to_hdf5(hdf5_filepath, data_cube, [0,1,0], project_name="Test confocal alignment")
        nanomax_controller.move_relative_forward("x", x_pixel_size)
        nanomax_controller.move_to("y", start_y)

    amptek_controller.disable_mca_mcs()
    amptek_controller.stop_connection()

    print(f'{maximum=}, {max_xyz=}')
    Hdf5Handler.visualize_3d_mapping(hdf5_filepath, data_cube.shape, group_name="Test confocal alignment")

    return data_cube

def cube_scan_center(amptek_controller: AmptekDevice, dwell_time, center, roi, x_pixel_size, x_range, y_pixel_size, y_range, z_pixel_size, z_range):
    """Start a cube scan around the given center"""
    x_origin = center[0] - x_range / 2
    y_origin = center[1] - y_range / 2
    z_origin = center[2] - z_range / 2

    x_pixel_num = round(x_range / x_pixel_size)
    y_pixel_num = round(y_range / y_pixel_size)
    z_pixel_num = round(z_range / z_pixel_size)

    return cube_scan_origin(amptek_controller,
                            dwell_time=dwell_time,
                            start_xyz=(x_origin, y_origin, z_origin),
                            roi=roi,
                                x_pixel_size=x_pixel_size, x_pixel_num=x_pixel_num,
                                y_pixel_size=y_pixel_size, y_pixel_num=y_pixel_num,
                                z_pixel_size=z_pixel_size, z_pixel_num=z_pixel_num
                                )

def test_cube_scan_center():
    import matplotlib.pyplot as plt
    import numpy as np

    idvendor_cxrf = "0x10c4"
    idproduct_cxrf = "0x842a"
    researched_sn_cxrf = "36133"

    amptek_cxrf_controller = AmptekDevice(idvendor_cxrf,
                                            idproduct_cxrf,
                                            researched_sn_cxrf,
                                            maxrf_device=False)

    data_cube = cube_scan_center(amptek_cxrf_controller,
                                    dwell_time=50,
                                    center=(3.690, 3.950, 2.900),
                                    roi=[0, 511],
                                    x_pixel_size=0.020, x_range=0.100,
                                    y_pixel_size=0.020, y_range=0.200,
                                    z_pixel_size=0.020, z_range=0.200)

    print("data cube shape : ", data_cube.shape)

if __name__ == "__main__":
    
    test_cube_scan_center()