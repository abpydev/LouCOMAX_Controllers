"""This module integrates classes for control of ZABER devices"""

__all__ = ["ZaberController"]

# Standard imports
import os
import sys
import configparser
import threading
import time
from typing import Literal
import logging

# Third-party imports
from zaber_motion import Units, exceptions, UnitTable
from zaber_motion.ascii import Connection, Axis
from zaber_motion.exceptions.movement_failed_exception import MovementFailedException

# Pymca imports
from PyQt5.QtWidgets import QLCDNumber, QApplication, QMainWindow

# Sub-modules imports
from .abstractcontroller import AbsController
from .abstractsliderposition import AbsSliderPosition
from .threads.control_threads import CyclicJob
from .lasercontroller import LaserController
from .exceptions import ControllersException
from .utils.pid_controller import PIDController

# logger configuration
logger = logging.getLogger(f'core.{__name__}')

class ZaberController(AbsController):
    """
    Controller class for managing an XYZ (plus joystick) daisy chain of Zaber devices.

    This class provides a high-level interface for connecting to, configuring, and controlling
    Zaber motorized stages in an XYZ configuration, with optional joystick support.
    
    It supports device discovery, axis allocation, movement commands (absolute, relative, velocity), homing,
    parking, emergency stop, and PID-based Z distance control (with a laser distance sensor).

    This class uses QAbstract slider to record the positions, therefore a QApplication (PyQT or Pyside)
    must be launched before instantiating this class

    Key Features:
        - Reads configuration from a .cfg file for device allocation, COM port, and axis orientation.
        - Connects to Zaber devices via serial port and initializes axis mapping.
        - Provides methods for moving axes forward/backward, to absolute targets, or at constant velocity.
        - Supports homing, parking, and unparking of axes.
        - Allows setting and querying of speed, acceleration, step size, and target positions.
        - Periodically updates and displays current axis positions.
        - Integrates with a laser controller for PID-based Z distance control.
        - Implements safety features such as emergency stop and safe jump (collision avoidance).
        - Handles firmware version checks and logs configuration/connection issues.
        - controllers_config (configparser.ConfigParser): Configuration object for controller settings.
    
    Attributes:
        - device_list (list): List of detected Zaber devices in the daisy chain.
        - axis_control (dict): Mapping of axis labels ("x", "y", "z") to Axis objects.
        - speed (dict): Current speed settings for each axis.
        - acceleration (dict): Current acceleration settings for each axis.
        - step (dict): Step size for relative movements for each axis.
        - target (dict): Target positions for absolute movements for each axis.
        - curr_pos (dict): Current position objects for each axis.
        - units (dict): Units used for position, velocity, and acceleration.
        - origin (list): Origin positions for each axis.
        - connexion_established (bool): Connection status flag.
        - _pid_kp, _pid_ki, _pid_kd (float): PID controller parameters for Z distance control.
    
    Methods:
        - start_connection(): Establish connection to Zaber stages and initialize axes.
        - stop_connection(): Close connection and reset axis controls.
        - move_forward(direction, wait_idle): Move axis forward by one step.
        - move_backward(direction, wait_idle): Move axis backward by one step.
        - move_to_target(direction, wait_idle): Move axis to absolute target position.
        - move_velocity(direction, invert_axis): Move axis at constant velocity.
        - set_speed(direction, speed): Set speed for an axis.
        - set_step(direction, step): Set step size for an axis.
        - set_target(direction, target): Set absolute target position for an axis.
        - set_origin(direction, origin_value): Set origin for an axis.
        - move_to_origin(directions): Move specified axes to their origin positions.
        - check_can_move_backward(distance): Check if axis can move backward by a given distance.
        - home_all_axes(): Home all axes.
        - force_home_pos(direction, speed): Force home position for an axis.
        - home_pos(direction): Home axis if not already homed.
        - home_pos_x/y/z(): Force home position for X/Y/Z axis.
        - park_all_axes(): Park all axes.
        - unpark_all_axes(): Unpark all axes.
        - stop_movement(direction): Stop movement of a specified axis.
        - get_limits(direction, unit): Get axis limits in specified unit.
        - get_xyz_pos(): Get current XYZ positions.
        - get_dir_pos(direction): Get last recorded position of an axis.
        - get_velocity(direction): Get last recorded speed of an axis.
        - get_acceleration(direction): Get last recorded acceleration of an axis.
        - start_update_pos_job(sampling_freq): Start periodic position update job.
        - stop_update_pos_job(): Stop periodic position update job.
        - connect_direction_to_lcd(direction, lcd_display): Connect axis position to LCD display.
        - emergency_stop(): Stop all axes and move Z backward for safety.
        - get_pos_unit(): Get symbol of current position unit.
        - get_velocity_unit(): Get symbol of current velocity unit.
        - set_pid_kp/ki/kd(str_val): Set PID controller parameters for Z distance control.
        - start_z_distance_control(distance_aim_mm, laser_controller, stop_event, update_frequency_secs): Start PID-based Z distance control.
        - move_to_closest(distance_controller): Move Z axis until closest point detected by laser.
        - safe_jump(xyz_position, backward_z_step, xy_speed, PointReachedEvent, laser_controller): Safely move to XYZ position with collision avoidance.

    Exceptions:
        Raises various exceptions for connection errors, movement failures, configuration issues, and safety violations.

    Note:
        This class assumes the presence of supporting classes such as Axis, AbsSliderPosition, Units, UnitTable,
        PIDController, LaserController, and exception types, as well as a logger for logging events."""

    _ZABER_ALLOC_MAP =  {
        "1" : "joystick",
        "2" : "x",
        "3" : "y",
        "4" : "z"
    }
    _ZABER_COM_PORT = "COM4"
    _ZABER_ORIENTATION_X = 0
    _ZABER_ORIENTATION_Y = 0
    _ZABER_ORIENTATION_Z = 0

    def __init__(self, controllers_config:configparser.ConfigParser):
        super().__init__()

        # List of devices in the daisy chain
        self.device_list = [None, None, None]

        # Configuration defined by the User in .cfg file
        self.controllers_config = controllers_config
        ZaberController._update_config(controllers_config)

        # Axis control (zaber axis)
        self.axis_control: dict[str, Axis] = {
            "x" : Axis(None, None),
            "y" : Axis(None, None),
            "z" : Axis(None, None)
        }
        # Speed stored
        self.speed = {
            "x" : 5000,
            "y" : 5000,
            "z" : 1000
        }
        # Acceleration stored
        self.acceleration = {
            "x" : 350,
            "y" : 350,
            "z" : 350
        }
        # Position step stored
        self.step = {
            "x" : 0,
            "y" : 0,
            "z" : 0
        }
        # Position target stored
        self.target = {
            "x" : 0,
            "y" : 0,
            "z" : 0
        }
        # Objects storing current position
        self.curr_pos = {
            "x" : AbsSliderPosition([0, 150000]),
            "y" : AbsSliderPosition([0, 150000]),
            "z" : AbsSliderPosition([0, 75000])
        }
        # Units currently used
        self.units = {
            "position" : Units.LENGTH_MICROMETRES,
            "velocity" : Units.VELOCITY_MICROMETRES_PER_SECOND,
            "acceleration" : Units.ACCELERATION_MILLIMETRES_PER_SECOND_SQUARED
        }

        # PID controller for Z distance control
        self._pid_kp = -0.006
        self._pid_ki = -0.035
        self._pid_kd = -0.001

        self.dist_sum_error = 0
        self.last_dist_error = 0
        self.last_time_pid = time.perf_counter()

        self.origin = [0, 0, 0]

    @classmethod
    def _update_config(cls, config:configparser.ConfigParser) -> None:
        """Reading and interpreting the config file .cfg"""
        cls._update_alloc_map(config)
        cls._update_com_port(config)
        cls._update_orientation(config)

    @classmethod
    def _update_alloc_map(cls, config:configparser.ConfigParser):
        """Reading and interpreting the config file .cfg
        
            Parameters related to :

                -   Allocation of each zaber device to the convention axes
                  X, Y and Z
        """
        try :
            cls._ZABER_ALLOC_MAP["1"] = config.get('Zaber', 'ZABER_DEVICE_1')
            cls._ZABER_ALLOC_MAP["2"]  = config.get('Zaber', 'ZABER_DEVICE_2')
            cls._ZABER_ALLOC_MAP["3"]  = config.get('Zaber', 'ZABER_DEVICE_3')
            cls._ZABER_ALLOC_MAP["4"]  = config.get('Zaber', 'ZABER_DEVICE_4')
        except Exception as e :
            logger.exception('Error while reading Controllers.cfg file : %s', e)
            raise ValueError("Error while reading Controllers.cfg file") from e

    @classmethod
    def _update_com_port(cls, config:configparser.ConfigParser):
        """Reading and interpreting the config file .cfg
        
            Parameters related to :

                -   USB COM port to connect to Zaber daisy chain
        """
        try :
            cls._ZABER_COM_PORT = config.get('Zaber', 'COM_PORT')
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
            cls._ZABER_ORIENTATION_X = config.getint('Zaber',
                                                      'ZABER_ORIENTATION_X')
            cls._ZABER_ORIENTATION_Y = config.getint('Zaber',
                                                      'ZABER_ORIENTATION_Y')
            cls._ZABER_ORIENTATION_Z = config.getint('Zaber',
                                                      'ZABER_ORIENTATION_Z')
        except Exception as e :
            logger.exception('Error while reading Controllers.cfg file : %s', e)
            raise ValueError("Error while reading Controllers.cfg file") from e

    def start_connection(self) -> None:
        """
        Establishes a connection to Zaber XYZ stages via the configured COM port and initializes device communication.

        Raises:
            exceptions.SerialPortBusyException: If the serial port is already in use.
            exceptions.ConnectionFailedException: If the connection to the stages fails.
            exceptions.NoDeviceFoundException: If no Zaber devices are found on the daisy chain.
            ValueError: If an invalid value is encountered during connection."""

        ZaberController._update_config(self.controllers_config)

        try:
            # Connection to ZABER stages via COM port
            logger.debug("...Attempting to connect to Zaber XYZ stages on port %s", 
                        ZaberController._ZABER_COM_PORT)
            self.connection = Connection.open_serial_port(
                ZaberController._ZABER_COM_PORT)

            # Detect all devices from the daisy chain
            self.device_list = self.connection.detect_devices()
            logger.info(f"Found {len(self.device_list)} ZABER devices : {self.device_list}")
            
            self._check_firmwares()
            self._init_allocation_map()

            self.connexion_established = True
            logger.info(f'ZABER stages connected to {ZaberController._ZABER_COM_PORT}')
            self.start_update_pos_job()

            if self.axis_control['x'].is_parked() \
                or self.axis_control['y'].is_parked() \
                or self.axis_control['z'].is_parked():

                self.unpark_all_axes()

        except exceptions.SerialPortBusyException :
            logger.info(f'Zaber XYZ stages still connected to {ZaberController._ZABER_COM_PORT}')
            
        except exceptions.ConnectionFailedException as zaber_err :
            logger.error((f"During connection to the stages on port \
{ZaberController._ZABER_COM_PORT}. Check the COM port and that the stages are \
powered\n{zaber_err}"))
            self.connexion_established = False

        except exceptions.NoDeviceFoundException as zaber_err :
            logger.error((f"During connection to the stages on port \
{ZaberController._ZABER_COM_PORT}. Check the COM port and that the stages are \
powered\n{zaber_err}"))
            self.connexion_established = False
            
        except ValueError as val_err:
                logger.error((f"During connection to the stages on port \
{ZaberController._ZABER_COM_PORT}. Check the COM port and that the stages are \
powered\n{val_err}"))
                self.connexion_established = False

    def stop_connection(self) -> None:
        if not self.is_connected():
            return
        
        self.stop_update_pos_job()
        self.connection.close()

        self.axis_control = {
            "x" : Axis(None, None),
            "y" : Axis(None, None),
            "z" : Axis(None, None)
        }

        self.connexion_established = False

    def _init_allocation_map(self) -> None :
        """
        Initialize the axis controls according to the allocation 
        (order of appearence of axes X, Y, Z, Joystick) 
        in the daisy chain according to configuration file
        """
        # Get allocation map from configuration file
        allocations_counter = 0
        axis_allocation_map = {}
        for _, value in ZaberController._ZABER_ALLOC_MAP.items():
            if value in ("x", "y", "z", "joystick"):
                axis_allocation_map.update({value : allocations_counter})
                allocations_counter += 1

        if len(self.device_list) != len(axis_allocation_map) :                
            # Case where the allocation of axes provided in .cfg
            # file doesn't match with the number of Zaber devices detected
            exception_msg = f'Detected {len(self.device_list)} devices but {len(axis_allocation_map)} were allocated in Controllers.cfg file. Please check your allocations in {os.path.dirname(__file__)} or check that all devices are connected'
            logger.exception(exception_msg)
            raise ValueError(exception_msg)

        self.axis_control["x"] = self.device_list[
                                    axis_allocation_map.get("x")
                                    ].get_axis(1)
        self.axis_control["y"] = self.device_list[
                                    axis_allocation_map.get("y")
                                    ].get_axis(1)
        self.axis_control["z"] = self.device_list[
                                    axis_allocation_map.get("z")
                                    ].get_axis(1)

        self.set_speed("x", self.speed["x"])
        self.set_speed("y", self.speed["y"])
        self.set_speed("z", self.speed["z"])

    def _check_firmwares(self) -> None:
        """
        Checks the firmware versions of all devices in the device list.
        Iterates through each device, collects their firmware versions, and logs a warning if multiple different versions are detected. 
        If all devices have the same firmware version, logs the version as info.
        """

        firmware_versions_list = []
        for device in self.device_list:
            version = f'{device.firmware_version.major}.{device.firmware_version.minor}'
            if version not in firmware_versions_list :
                firmware_versions_list.append(version)
        if len(firmware_versions_list) > 1 :
            logger.warning(f'Different firmware versions detected on the Zaber devices : {firmware_versions_list}')
        else :
            logger.info(f"Firmware version : {firmware_versions_list[0]}")

    def move_forward(self, direction:Literal["x", "y", "z"], wait_idle=True) -> None:
        """
        Moves the axis "direction" forward (+) by one step. 
        The step value can be set using set_step() method

            -   direction : "x", "y" or "z"
            -   wait_idle : "True" or "False". If True : waits for the 
                movement to end before continuing execution
        """
        if not self.is_connected():
            ControllersException.ZaberNotConnected()
            return
        else:
            logger.debug(f'Moving {direction} at speed : {self.speed[direction]} {UnitTable.get_symbol(self.units["velocity"])}, step={int(self.step[direction])} {UnitTable.get_symbol(self.units["position"])}')

            invert_axis = self._ZABER_ORIENTATION_X if direction == "x" else \
                self._ZABER_ORIENTATION_Y if direction == "y" else \
                self._ZABER_ORIENTATION_Z
            if invert_axis == 0:
                # Positive direction
                self.axis_control[direction].move_relative(
                    float(self.step[direction]),
                    self.units["position"],
                    wait_until_idle=wait_idle,
                    velocity=self.speed[direction],
                    velocity_unit=self.units["velocity"],
                    acceleration=self.acceleration[direction],
                    acceleration_unit=self.units["acceleration"]
                )
            else:
                # Negative direction
                self.axis_control[direction].move_relative(
                    float(-1*self.step[direction]),
                    self.units["position"],
                    wait_until_idle=wait_idle,
                    velocity=self.speed[direction],
                    velocity_unit=self.units["velocity"],
                    acceleration=self.acceleration[direction],
                    acceleration_unit=self.units["acceleration"]
                )

    def move_backward(self, direction:Literal["x", "y", "z"], wait_idle=True) -> None:
        """
        Moves the axis "direction" backward (-) by one step. 
        The step value can be set using set_step() method

            -   direction : "x", "y" or "z"
            -   wait_idle : "True" or "False". If True : waits for the movement
                            to end before continuing execution
        """

        if not self.is_connected():
            ControllersException.ZaberNotConnected()
            return
        else:
            logger.debug(f'Moving {direction} at speed : {self.speed[direction]} {UnitTable.get_symbol(self.units["velocity"])}, step=-{int(self.step[direction])} {UnitTable.get_symbol(self.units["position"])}')
            
            invert_axis = self._ZABER_ORIENTATION_X if direction == "x" else \
                self._ZABER_ORIENTATION_Y if direction == "y" else \
                self._ZABER_ORIENTATION_Z
            if invert_axis == 0:
                # Negative direction
                self.axis_control[direction].move_relative(
                    float(-1*self.step[direction]),
                    self.units["position"],
                    wait_until_idle=wait_idle,
                    velocity=self.speed[direction],
                    velocity_unit=self.units["velocity"],
                    acceleration=self.acceleration[direction],
                    acceleration_unit=self.units["acceleration"]
                )
            else:
                # Prositive direction
                self.axis_control[direction].move_relative(
                    float(self.step[direction]),
                    self.units["position"],
                    wait_until_idle=wait_idle,
                    velocity=self.speed[direction],
                    velocity_unit=self.units["velocity"],
                    acceleration=self.acceleration[direction],
                    acceleration_unit=self.units["acceleration"]
                )

    def move_to_target(self, direction:Literal["x", "y", "z"], wait_idle=True) -> None:
        """
        Moves the axis "direction" to absolute target. 

        The target value can be set using set_target() method

        The movement speed value can be set using set_speed() method

        @Args
        -   direction : "x", "y" or "z"
        -   wait_idle : "True" or "False". If True : waits for the 
                        movement to end before continuing execution
        """

        if not self.is_connected():
            ControllersException.ZaberNotConnected()
            return

        else:
            try:
                logger.debug(f'Move Absolute to target = {self.target[direction]}µm')
                self.axis_control[direction].move_absolute(
                    float(self.target[direction]),
                    self.units["position"],
                    wait_until_idle=wait_idle,
                    velocity=self.speed[direction],
                    velocity_unit=self.units["velocity"])

            except AttributeError as e:
                logger.error(f"Check that Zaber stages are connected\n{e}")
            except exceptions.movement_interrupted_exception.MovementInterruptedException as mvmt_inter :
                logger.warning(f'Absolute movement interrupted')
                raise InterruptedError from mvmt_inter

    def move_velocity(self, direction:Literal['x', 'y', 'z'],
                       invert_axis:bool=False
                       ) -> None:
        """
        Moves the axis "direction" at a certain speed. 
        The speed value can be pre-set using set_speed() method

        Args:
            -   direction (Literal["x", "y", "z"]) :
            which axis to move
            -   invert_axis (bool) :
            If True, movement is set to opposite direction

        Raises:
            -   ConnectionError : if the device is not connected
            -   InterruptedError : if the movement is interrupted
            -   ValueError : if the movement failed
        """

        if not self.is_connected():
            ControllersException.ZaberNotConnected()
            return
        else:
            try :
                logger.debug(f'Moving in {direction} direction at speed {self.speed[direction]} {UnitTable.get_symbol(self.units["velocity"])}')
                if invert_axis :
                    self.axis_control[direction].move_velocity(
                        -1*float(self.speed[direction]),
                        self.units["velocity"])
                else :
                    self.axis_control[direction].move_velocity(
                        float(self.speed[direction]),
                        self.units["velocity"])
            except exceptions.connection_closed_exception.ConnectionClosedException :
                ControllersException.ZaberNotConnected()
            except exceptions.movement_interrupted_exception.MovementInterruptedException as interrupt_err:
                logger.warning(f'Velocity movement interrupted')
                raise InterruptedError from interrupt_err
            except MovementFailedException as movement_err:
                logger.warning(f'Movement failed : {movement_err}')
                raise ValueError from movement_err

    def set_speed(self, direction: Literal["x", "y", "z"], speed: int|float) -> None:
        """
        Set speed value for given axis ("x", "y" or "z")
        """

        if speed <= 0 :
            speed = 1

        # Set the current speed record
        self.speed[direction] = speed

    def set_step(self, direction:Literal["x", "y", "z"], step:int) -> None:
        """
        Sets the movement step size for a specified axis.
        Args:
            direction (Literal["x", "y", "z"]): The axis to set the step size for.
            step (int): The step size in micrometers (µm) to set for the specified axis.
        Raises:
            ValueError: If the provided direction is not "x", "y", or "z".
        Notes:
            The step size determines the increment of movement for the specified axis when using move backward/forward methods.
        """
        if direction in (self.step):
            self.step[direction] = step
        else:
            raise ValueError(f"Wrong direction entered for ZaberController.set_step() argument : {direction} must be \"x\" \"y\" or \"z\"")

    def set_target(self, direction:Literal["x", "y", "z"], target:int) -> None:
        """
        Sets the target position for the specified axis ('x', 'y', or 'z').

        Parameters:
            direction (Literal["x", "y", "z"]): The axis to set the target for.
            target (int): The desired target position.

        Notes:
            - The target value is clamped between 0 and 150000.
            - Raises a ValueError if the direction is not one of 'x', 'y', or 'z'.
        """
        min_range = 0
        max_range = 150000
        # TODO : retrieve range limits from the stage
        if target < min_range :
            target = min_range
        elif target > max_range :
            target = max_range
        if direction in (self.step):
            self.target[direction] = target
        else:
            raise ValueError(f"Wrong target or direction entered for ZaberController.set_step() argument : {direction} must be \"x\" \"y\" or \"z\"")

    def set_origin(self, direction: Literal["x", "y", "z"], origin_value: int) -> None:
        """Set the given origin on given axis.
         
         Only for position value READING with get_dis_pos(direction)
         """
        
        self.curr_pos[direction].set_offset(origin_value)
        position = self.get_dir_pos(direction)
        self.curr_pos[direction].update_lcd(position)

        match direction:
            case 'x':
                self.origin[0] = origin_value
            case 'y':
                self.origin[1] = origin_value
            case 'z':
                self.origin[2] = origin_value

    def move_to_origin(self, directions:tuple[Literal["x", "y", "z"]]) -> None:
        """
        Moves the specified axes ('x', 'y', and/or 'z') to their origin positions.
        Args:
            directions (tuple[Literal["x", "y", "z"]]): 
                A tuple containing one or more axis labels ('x', 'y', 'z') indicating which axes to move to their origin.
        Returns:
            None
        Side Effects:
            Commands the hardware to move the specified axes to their respective origin positions as defined in self.origin.
            Each axis is moved asynchronously (does not wait for movement to complete before returning).
        Example:
            move_to_origin(('x', 'z'))  # Moves the x and z axes to their origin positions.
        """

        target = self.get_xyz_pos()
        if "x" in directions :
            self.set_target('x', self.origin[0])
            self.move_to_target('x', wait_idle=False)

        if "y" in directions :
            self.set_target('y', self.origin[1])
            self.move_to_target('y', wait_idle=False)

        if "z" in directions :
            self.set_target('z', self.origin[2])
            self.move_to_target('z', wait_idle=False)

    def check_can_move_backward(self, distance:float) -> bool:
        """Check if the given distance can be moved backward"""
        if not self.is_connected():
            ControllersException.ZaberNotConnected()
            return False

        backward_limit = self.get_dir_pos('z')
        return distance <= backward_limit

    # @pyqtSlot()
    def home_all_axes(self) -> None:
        """Force home for all axes"""
        
        if not self.is_connected():
            ControllersException.ZaberNotConnected()
            return

        self.force_home_pos('z', speed=2500)
        self.force_home_pos('x', speed=2500)
        self.force_home_pos('y', speed=2500)

    def force_home_pos(self, direction:Literal["x", "y", "z"],
                        speed:float=5000) -> None:

        """
        Force the home position even if the axis 
        was already homed before
        
        - direction : "x", "y" or "z" the axis to be homed
        """

        if not self.is_connected():
            ControllersException.ZaberNotConnected()
            return
        axis = self.axis_control[direction]
        try:
            logger.info(f"Force Home for {direction} direction at speed : {speed} {UnitTable.get_symbol(self.units.get("velocity"))}")
            self.set_speed(direction, speed)
            axis.home(wait_until_idle=False)
        except AttributeError as attrerr:
            raise ConnectionError(f"The XYZ stages is not connected. Impossible to send HOME {direction} command") from attrerr

    def home_pos(self, direction) -> None:

        """
        Home the axis if it was not homed before

        - direction : "x", "y" or "z" the axis to be homed
        """

        axis = self.axis_control[direction]
        try:
            if not axis.is_homed():
                logger.info(f"Going Home for {direction} direction ...")
                axis.home()
                logger.info(f"{direction} homed")
            else :
                logger.info(f"{axis} was already homed")
        except AttributeError as attrerr:
            raise ConnectionError(f"The XYZ stages is not connected. Impossible to send HOME {direction} command") from attrerr

    # @pyqtSlot()
    def home_pos_x(self) -> None:
        """Force home position for X axis"""
        try:
            # self.home_pos("x")
            self.force_home_pos("x")
        except ConnectionError as e:
            logger.error(f'{e}')

    # @pyqtSlot()
    def home_pos_y(self) -> None:
        """Force home position for Y axis"""
        try:
            # self.home_pos("y")
            self.force_home_pos("y")
        except ConnectionError as e:
            logger.error(f'{e}')

    # @pyqtSlot()
    def home_pos_z(self) -> None:
        """Force home position for Z axis"""
        try:
            # self.home_pos("z")
            self.force_home_pos("z")
        except ConnectionError as e:
            logger.error(f'{e}')

    def park_all_axes(self) -> None:
        """Parks the axes in anticipation of turning the power off. 

        It can later be powered on, unparked, and moved without first 
        having to home it."""
        self.axis_control['x'].park()
        self.axis_control['y'].park()
        self.axis_control['z'].park()
    
    def unpark_all_axes(self) -> None:
        """Unparks axes. Axes will now be able to move."""
        self.axis_control['x'].unpark()
        self.axis_control['y'].unpark()
        self.axis_control['z'].unpark()

    def stop_movement(self, direction:Literal["x", "y", "z"]) -> None:
        """
        Stops the movement of the specified axis.
        Parameters:
            direction (Literal["x", "y", "z"]): The axis to stop movement on.
        Raises:
            KeyError: If an invalid direction is provided, triggers an emergency stop and raises this exception.
            ConnectionError: If a connection error occurs during the stop operation.
            MovementFailedException: If stopping the movement fails for any reason.
        Logs:
            - Debug message indicating which axis is being stopped.
            - Error message if a ConnectionError occurs.
            - Exception details if a MovementFailedException occurs.
        """

        try:
            logger.debug(f'Stop {direction} movement')
            match direction:
                case "x":
                    self._stop_x()
                case "y":
                    self._stop_y()
                case "z":
                    self._stop_z()
                case _:
                    self.emergency_stop()
                    raise KeyError(f"Wrong direction provided to stop_movement() method, therefore emergency_stop() wass called.\tdirection entered : {direction}")
        except ConnectionError as connection_error:
                logger.error(f'{connection_error}')
        except MovementFailedException as err:
            logger.exception(err)

    def _stop_x(self) -> None:
        """
        Stops the movement of the X axis without waiting for it to become idle.
        Attempts to send a stop command to the X axis controller.
        Raises:
            ConnectionError: If the X axis controller is not connected.
        """
        try :
            self.axis_control["x"].stop(wait_until_idle=False)
        except AttributeError as attrerr:
            raise ConnectionError("The XYZ stages is not connected. Impossible to send STOP X command") from attrerr

    def _stop_y(self) -> None:
        """
        Stops the movement of the Y axis without waiting for it to become idle.
        Attempts to send a stop command to the Y axis controller.
        Raises:
            ConnectionError: If the Y axis controller is not connected.
        """

        try:
            self.axis_control["y"].stop(wait_until_idle=False)
        except AttributeError as attrerr:
            raise ConnectionError("The XYZ stages is not connected. Impossible to send STOP Y command") from attrerr

    def _stop_z(self) -> None:
        """
        Stops the movement of the Z axis without waiting for it to become idle.
        Attempts to send a stop command to the Z axis controller.
        Raises:
            ConnectionError: If the Z axis controller is not connected.
        """
        try:
            self.axis_control["z"].stop(wait_until_idle=False)
        except AttributeError as attrerr:
            raise ConnectionError("The XYZ stages is not connected. Impossible to send STOP Z command") from attrerr

    def get_limits(self, direction:Literal["x", "y", "z"],
                    unit=Units.LENGTH_MICROMETRES
                    ) -> tuple[int, int]:
        """
        Interogate the device to get the given axis limits converted to 
        the given unit
        """

        max = self.axis_control[direction].settings.get("limit.max",
                                                            unit=unit)
        min = self.axis_control[direction].settings.get("limit.min",
                                                            unit=unit)
        return (min, max)

    def get_xyz_pos(self) -> tuple[int, int, int]:
        """Interogate the device to get xyz axes positions
        """

        x_pos = round(self.axis_control["x"].get_position(
            unit=self.units.get("position")))
        y_pos = round(self.axis_control["y"].get_position(
            unit=self.units.get("position")))
        z_pos = round(self.axis_control["z"].get_position(
            unit=self.units.get("position")))

        return (x_pos, y_pos, z_pos)

    def get_dir_pos(self, direction:Literal["x", "y", "z"]) -> int:
        """Get the last recorded position of the given axis
        (does not interogate the device)
        """
        return self.curr_pos[direction].get_pos()

    def get_velocity(self, direction:Literal["x", "y", "z"]) -> int:
        """Get the last recorded speed of the given direction
          (does not interogate the device)"""
        return int(self.speed.get(direction))

    def get_acceleration(self, direction:Literal['x', 'y', 'z']) -> int:
        """Get the last recorded acceleration  of the given axis
          (does not interogate the device)"""
        
        return int(self.acceleration.get(direction))

    def start_update_pos_job(self, sampling_freq=.1) -> None:
        """Create and start the Job for position update 
        (Cyclicaly interogates the device for all axes positions)"""
        logger.info('Starting position display updating')
        self.update_pos_timer = CyclicJob(target=self._refresh_xyz_pos,
                                        interval=sampling_freq,
                                        name="zaber-position-update-job")
        self.update_pos_timer.start()

    def stop_update_pos_job(self) -> None :
        """Stops the cyclic job that reads position from ZABER stages"""
        if hasattr(self, 'update_pos_timer') :
            logger.info('Stopping position display updating')
            self.update_pos_timer.stop()

    # Threaded method
    def _refresh_xyz_pos(self) -> None:
        """
        Refreshes the current X, Y, and Z position values.
        Retrieves the latest X, Y, and Z positions using `get_xyz_pos()` and updates the corresponding
        entries in `self.curr_pos`.
        """

        x_pos, y_pos, z_pos = self.get_xyz_pos()

        self.curr_pos["x"].setValue(int(x_pos))
        self.curr_pos["y"].setValue(int(y_pos))
        self.curr_pos["z"].setValue(int(z_pos))

    def connect_direction_to_lcd(self, direction:Literal["x", "y", "z"],
                                  lcd_display : QLCDNumber
                                  ) -> None:
        """Connect the given direction position display
        to the given LCD QWidget"""
        self.curr_pos[direction].add_lcd_display(lcd_display)

    # @pyqtSlot()
    def emergency_stop(self):
        """Stops all axes then move Z backward"""
        logger.warning('EMERGENCY STOP PRESSED !')
        try:
            #Stop all axes
            self._stop_x()
            self._stop_y()
            self._stop_z()

            #Move Z backward at high speed
            self.set_speed("z", 10000)
            self.set_step("z", 5000)
            self.move_backward("z", wait_idle=False)
            self.set_speed("z", 1000)

        except ConnectionError as con_err:
            logger.error(f'{con_err}')

    def get_pos_unit(self) -> str:
        """Get the standard symbol of position unit 
        currently used by the device"""
        return UnitTable.get_symbol(self.units.get("position"))

    def get_velocity_unit(self) -> str:
        """Get the standard symbol of velocity unit 
        currently used by the device"""
        return UnitTable.get_symbol(self.units.get("velocity"))

# Distance to object control via LaserController
    def set_pid_kp(self, str_val):
        try :
            self._pid_kp = float(str_val)
        except ValueError :
            self._pid_kp = 0

    def set_pid_ki(self, str_val):
        try :
            self._pid_ki = 0
        except ValueError :
            self._pid_ki = 0

    def set_pid_kd(self, str_val):
        try :
            self._pid_kd = 0
        except ValueError :
            self._pid_kd = 0

    def start_z_distance_control(self, distance_aim_mm: float, laser_controller: LaserController, stop_event:threading.Event, update_frequency_secs:float):
        """
        @Args:
        -   distance_aim_mm : float : distance in millimeter to keep
        between the laser and the study object
        -   laser_controller : LaserController : Laser controller object
        that gives the distance feedback
        -   stop_event : threading.Event : Event that stops distance control 
        if set
        -   update_frequency_secs : float : time to wait between two Z commands (in seconds)
        """
        self._pid_kp = -0.006
        self._pid_ki = -0.035
        self._pid_kd = -0.001

        if not self.is_connected():
            ControllersException.ZaberNotConnected()
            return

        setpoint = distance_aim_mm

        origin_distance = self.get_dir_pos("z")
        output = 0
        pid_controller = PIDController(self._pid_kp, self._pid_ki, self._pid_kd, setpoint)
        while not stop_event.is_set() :
            measurement, _ = laser_controller.get_value_and_unit()
            output = -1* pid_controller.update(measurement)
            self.set_target('z', int(origin_distance + output))
            self.set_speed("z", 5000)
            self.move_to_target('z', wait_idle=False)
            time.sleep(update_frequency_secs)

        self.set_speed("z", 1000)

    def move_to_closest(self, distance_controller: LaserController):
        """Moves Z stage while reading laser distance.
        
        Stops when laser distance sensor controllers safety flag is raised,
        meaning that closest point has been reached"""
        self.set_step('z', 10) # step 10µm
        flag_can_move_closer = distance_controller.query_can_move_closer()
        while flag_can_move_closer :
            self.move_forward('z', wait_idle=True)
            flag_can_move_closer = distance_controller.query_can_move_closer()
            time.sleep(0.010) # sleep 10ms

    def safe_jump(self, xyz_position:tuple[3], backward_z_step = 5000, xy_speed=2000, PointReachedEvent:threading.Event=None, laser_controller:LaserController=None) -> None:
        """
        Safely moves the device to a specified XYZ position by first retracting the Z axis, then moving X and Y, and finally moving Z to the target.
        Args:
            xyz_position (tuple[3]): Target (x, y, z) position to move to.
            backward_z_step (int, optional): Distance (in µm) to retract the Z axis before moving in XY. Defaults to 5000.
            xy_speed (int, optional): Speed to use for X and Y movements. Defaults to 2000.
            PointReachedEvent (threading.Event, optional): Event to set when the target point is reached. Defaults to None.
            laser_controller (LaserController, optional): Laser controller to check if forward Z movement is allowed. Defaults to None.
        Raises:
            ValueError: If the laser controller does not allow the required forward Z movement.
            InterruptedError: If the movement is interrupted.
        Notes:
            - If already at the target position, the function returns immediately.
            - The function ensures Z is retracted before XY movement to avoid collisions.
            - Notifies via PointReachedEvent (if provided) when the move is complete.
        """

        current_xyz = self.get_xyz_pos()
        if current_xyz == xyz_position :
            # Already at the wanted position
            if PointReachedEvent is not None :
                PointReachedEvent.set()
            return

        logger.debug(f'Safe jump to : {xyz_position=} with {backward_z_step=}')
        self.set_speed('x', xy_speed)
        self.set_speed('y', xy_speed)

        try :
            if current_xyz[2] > backward_z_step:
                # Move back Z for *backward_z_step* value
                self.set_step('z', backward_z_step)
                self.move_backward('z', wait_idle=True)

            self.set_target('x', xyz_position[0])
            self.set_target('y', xyz_position[1])
            self.set_target('z', xyz_position[2])

            # Move both x and y to initial position
            self.move_to_target('x', wait_idle=False)
            self.move_to_target('y', wait_idle=True)

            forward_step = xyz_position[2] - self.get_dir_pos('z')
            if forward_step == 0:
                # No need to move Z
                pass
            elif laser_controller is not None and not laser_controller.can_move_forward_by(forward_step):
                # logger.warning(f"Laser controller does not allow move forward by {forward_step}µm")
                # return
                raise ValueError(f"Laser controller does not allow move forward by {forward_step}µm")
            else:
                # Move z to initial position
                self.move_to_target('z', wait_idle=True)

        except InterruptedError as interrupt_err:
            logger.warning(f'Safe jump interrupted')
            raise InterruptedError from interrupt_err
        
        if PointReachedEvent is not None:
            # Notify that the point has been reached
            PointReachedEvent.set()

    def __repr__(self):
            return (f"<ZaberController("
                    f"COM_PORT={self._ZABER_COM_PORT!r}, "
                    f"AllocMap={self._ZABER_ALLOC_MAP!r}, "
                    f"Orientation=({self._ZABER_ORIENTATION_X}, {self._ZABER_ORIENTATION_Y}, {self._ZABER_ORIENTATION_Z}), "
                    f"Connected={getattr(self, 'connexion_established', False)}, "
                    f"Speed={self.speed}, "
                    f"Step={self.step}, "
                    f"Target={self.target}, "
                    f"Origin={self.origin})>")
    
    def __str__(self):
            return (f"<ZaberController(\n"
                    f"COM_PORT={self._ZABER_COM_PORT!r}, \n"
                    f"AllocMap={self._ZABER_ALLOC_MAP!r}, \n"
                    f"Orientation=({self._ZABER_ORIENTATION_X}, {self._ZABER_ORIENTATION_Y}, {self._ZABER_ORIENTATION_Z}), \n"
                    f"Connected={getattr(self, 'connexion_established', False)}, \n"
                    f"Speed={self.speed}, \n"
                    f"Step={self.step}, \n"
                    f"Target={self.target}, \n"
                    f"Origin={self.origin})>\n")

def main():

    # Typical configuration
    config = configparser.ConfigParser()
    config.add_section('Zaber')
    config.set('Zaber', 'com_port', 'COM4')
    config.set('Zaber', 'ZABER_ORIENTATION_X', '0')
    config.set('Zaber', 'ZABER_ORIENTATION_Y', '1')
    config.set('Zaber', 'ZABER_ORIENTATION_Z', '0')
    config.set('Zaber', 'ZABER_DEVICE_1', 'joystick')
    config.set('Zaber', 'ZABER_DEVICE_2', 'x')
    config.set('Zaber', 'ZABER_DEVICE_3', 'y')
    config.set('Zaber', 'ZABER_DEVICE_4', 'z')


    ex = QApplication(sys.argv)

    app = QMainWindow()
    app.show()
    zaber_controller = ZaberController(config)
    print(zaber_controller)

    sys.exit(ex.exec_())

if __name__ == "__main__":
    main()
