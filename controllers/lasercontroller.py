"""This module contains the class(es) for control of the PANASONIC 
Laser : HG-C1100-P with a specific setup on a Arduino"""

# Standard imports
import serial
import serial.tools.list_ports
import time
import configparser
import logging

# PyQt5 imports
from PyQt5.QtCore import pyqtSignal, pyqtSlot
from PyQt5.QtWidgets import QLCDNumber, QWidget

# project sub-modules imports
from threads.control_threads import CyclicJob
from abstractsliderposition import AbsSliderPosition
from exceptions.ControllersException import LaserNotConnected

# logger configuration
logger = logging.getLogger(f'core.{__name__}')

class LaserController(QWidget):
    """
    LaserController is a QWidget-based class for managing a laser distance sensor via serial communication.

    This controller handles connection management, distance measurement, safety checks, and signal emission for UI updates.

    It supports periodic distance polling, range validation, and integration with display widgets.

    This class only works on a specific harware setup (Telemetry Laser + Arduino UNO with specific connections)
    please refer to the documentation for further info

    Signals:
        - SigLaserOutOfRange (bool): Emitted when the laser sensor detects an out-of-range condition.
        - SigLaserDistance (int): Emitted with the measured distance value (in micrometers).
        - sigFocusChanged (float): Emitted when the focus distance is updated.

    Args:
        - controllers_config (configparser.ConfigParser): Configuration object containing sensor and connection parameters.
        - parent (QWidget, optional): Parent widget.

    Attributes:
        - controllers_config: Stores the configuration for controllers.
        - com_port: Serial port for the distance sensor.
        - offset_mm: Center offset for measuring distance (in mm).
        - laser_range_mm: Total measurable range of the sensor (in mm).
        - min_distance_for_safety: Minimum safe distance (in micrometers).
        - focus_distance_micrometers: Current focus distance (in micrometers).
        - measured_distance: AbsSliderPosition object for managing measured distance and display.
        - last_valid_value: Last valid measured distance (in micrometers).
        - unit: String representing the unit of measurement ("µm").
        - port_tuple_repr: Tuple representation of the COM port connection.
        - out_of_range_flag: Boolean indicating if the sensor is out of range.
        - serial_port: Serial port object for communication.
        - connexion_established: Boolean indicating if the connection is established.

    Methods:
        - _build_connections(): Connects internal signals to their respective slots.
        - _read_serial(...): Reads and processes data from the serial port (threaded).
        - _set_out_of_range(flag: bool): Updates the out-of-range status and display color.
        - get_measured_dist() -> float: Returns the measured distance with offset applied.
        - query_can_move_closer() -> bool: Checks if it is safe to move closer based on the current measurement.
        - get_remaining_safe_distance() -> float: Returns the remaining safe distance before reaching the minimum.
        - can_move_forward_by(distance: float) -> bool: Checks if a given forward movement is safe.
        - set_focus_val_to_current_dist(): Sets the focus value to the current measured distance.
        - start_update_distance_job(...): Starts periodic distance polling.
        - stop_update_distance_job(): Stops the distance polling job.
        - is_connected() -> bool: Returns connection status.
        - is_out_of_range() -> bool: Returns out-of-range status.
        - start_connection(): Establishes serial connection to the sensor.
        - stop_connection(): Closes the serial connection and stops polling.
        - laser_on(): Turns the laser on.
        - laser_off(): Turns the laser off.
        - get_value_and_unit(): Returns the last measured value and its unit.
        - connect_to_lcd(lcd_display: QLCDNumber): Connects the measured distance to an LCD display widget.
    
    Exceptions:
        - Raises LaserNotConnected if operations are attempted without an active connection.
        - Raises ConnectionError if unable to connect to the serial port."""

    SigLaserOutOfRange = pyqtSignal(bool)
    SigLaserDistance = pyqtSignal(int)
    sigFocusChanged = pyqtSignal(float)

    def __init__(self, controllers_config:configparser.ConfigParser, parent=None) -> None:
        super().__init__(parent)

        self.controllers_config = controllers_config

        self.com_port = controllers_config.get('Distance_sensor', 'COM_PORT')
        self.offset_mm = int(controllers_config.get('Distance_sensor', 'MEASURING_CENTER_DISTANCE_MM'))
        self.laser_range_mm = int(controllers_config.get('Distance_sensor', 'MEASURING_RANGE_MM'))
        self.min_distance_for_safety = float(controllers_config.get('Distance_sensor', 'MIN_SAFE_DISTANCE_MM')) * 1000
        self.focus_distance_micrometers = float(controllers_config.get('Distance_sensor', 'FOCUS_DISTANCE_MM')) * 1000
        
        range = [(self.offset_mm - self.laser_range_mm//2)*1000,
                                         (self.offset_mm + self.laser_range_mm//2)*1000]
        self.measured_distance = AbsSliderPosition(range)
        self.measured_distance.set_offset(self.min_distance_for_safety)
        self.last_valid_value = self.offset_mm * 1000

        self.unit = "µm"
        self.port_tuple_repr = None
        self.out_of_range_flag = True
        self.serial_port = None
        self.connexion_established = False

        self._build_connections()

    def _build_connections(self):

        self.SigLaserOutOfRange.connect(self._set_out_of_range)
        self.SigLaserDistance.connect(self.measured_distance.setValue)

    # Threaded method
    def _read_serial(self, range_points_num=1024, sampling_rate_secs=.1, feed_to_csv=False, csv_filepath=None):
        """Reads the serial port to update the laser value
        
        Optional : Write the value to a given csv file"""

        if not self.serial_port.is_open :
            return

        # Flush the buffer
        self.serial_port.read_all()
        self.serial_port.readline()
        time.sleep(sampling_rate_secs)

        # Read one line from buffer
        str_val = str(self.serial_port.readline()[:-2])

        if not str_val == "b''":
            str_val = str_val.lstrip("b").strip("'")
            if int(str_val) > 1022 :
                # Out of range of the sensor (100mm +-35mm)
                self.SigLaserOutOfRange.emit(True)

                if feed_to_csv :
                    with open(csv_filepath, 'a') as csvfile:
                        timestamp = time.perf_counter()
                        csvfile.write(f'{timestamp},-1\n')
            else:
                # In range of the sensor (100mm +-35mm)
                self.SigLaserOutOfRange.emit(False)
                value = (self.offset_mm + self.laser_range_mm / 2 - int(str_val) * self.laser_range_mm / range_points_num)*1000

                self.SigLaserDistance.emit(int(round(value)))

                if feed_to_csv :
                    with open(csv_filepath, 'a') as csvfile:
                        timestamp = time.perf_counter()
                        csvfile.write(f'{timestamp},{int(round(value))}\n')

    @pyqtSlot(bool)
    def _set_out_of_range(self, flag:bool):
        self.out_of_range_flag = flag
        if not flag :
            self.measured_distance.set_lcd_color(num_color='black', background_color='green')
            return

        self.measured_distance.set_lcd_color(num_color='black', background_color='red')

    def get_measured_dist(self) -> float:
        """
        Retrieves the measured distance (with the applied offset).

        Returns:
            float: The measured distance with the offset applied.
        """
        return self.measured_distance.get_pos_with_offset()

    def query_can_move_closer(self) -> bool:
        """
        @Returns:
            -   True if the minimal safety distance has not yet been reached,
                False otherwise
        
        @Exceptions:
            -   LaserNotConnected : raised if the telemetry laser sensor is not connected"""
        if not self.is_connected():
            LaserNotConnected
            return False

        if self.is_out_of_range() :
            return True

        return self.measured_distance.value() >= self.min_distance_for_safety

    def get_remaining_safe_distance(self) -> float:
        """Get the remaining distance that the move forward allows
            
            (Depends on MIN_SAFE_DISTANCE_MM from .cfg file)
        """
        remaining_distance = self.measured_distance.value() - self.min_distance_for_safety
        
        return 0 if remaining_distance <= 0 else remaining_distance

    def can_move_forward_by(self, distance:float) -> bool:
        """Returns a bool indicating whethe or not the given distance
        can be moved forward (towards object) without collision"""
        if not self.is_connected() or self.is_out_of_range():
            return False

        return distance < self.get_remaining_safe_distance()

    def set_focus_val_to_current_dist(self):
        self.focus_distance_micrometers = self.get_measured_dist()
        self.controllers_config.set('Distance_sensor', 'FOCUS_DISTANCE_MM', str(self.focus_distance_micrometers / 1000))
        self.sigFocusChanged.emit(self.focus_distance_micrometers)

    def start_update_distance_job(self, sampling_freq=.1, feed_to_csv=False, csv_filepath=None) -> None:
        """Create and start the Job for distance update 
        (Cyclicaly interogates the laser device for distance)"""

        logger.info('Starting laser distance updating')
        self.update_dist_timer = CyclicJob(target=self._read_serial,
                                        interval=sampling_freq,
                                            **{"feed_to_csv":feed_to_csv, "csv_filepath":csv_filepath})
        self.update_dist_timer.setName('LaserDistance-position-update-Job')
        self.update_dist_timer.start()

    def stop_update_distance_job(self):
        """Stops the Job for distance update"""
        if hasattr(self, "update_dist_timer"):
            self.update_dist_timer.stop()

    def is_connected(self) -> bool:
        
        return self.connexion_established

    def is_out_of_range(self) -> bool:
        return self.out_of_range_flag

    def start_connection(self) -> None:

        if self.serial_port is None or not self.serial_port.is_open:
            try :
                logger.debug(f'...Attempting connection to ARDUINO at {self.com_port}')
                self.serial_port = serial.Serial(self.com_port, 9600, timeout=1)

                # Save tuple representation of the COM port connection for later connection checking
                connected_com_ports = [tuple(p) for p in list(serial.tools.list_ports.comports())]
                self.port_tuple_repr = [port for port in connected_com_ports if self.com_port in port ][0]

            except serial.SerialException as ser_err:
                connerr_msg = f'ARDUINO for Distance Laser control not found at COM port : {self.com_port}'
                logger.error(connerr_msg)
                self.connexion_established = False
                raise ConnectionError(connerr_msg)

            self.connexion_established = True
            logger.info(f'ARDUINO for Distance Laser control connected on serial port : {self.com_port}')
            self.start_update_distance_job(sampling_freq=0.01, feed_to_csv=False)

        else :
            pass

    def stop_connection(self) -> None:
        if not self.is_connected():
            return
        self.stop_update_distance_job()
        self._set_out_of_range(True)
        self.serial_port.close()
        self.connexion_established = False

    def laser_on(self) -> None:
        """Turn the laser on"""
        print("ON")
        if not self.is_connected():
            raise LaserNotConnected

        if self.serial_port is not None and self.serial_port.is_open:
            self.serial_port.write(b'0')
            logger.debug('Laser turned ON')

    def laser_off(self) -> None:
        """Turn the laser off"""
        if not self.is_connected():
            raise LaserNotConnected

        if self.serial_port is not None and self.serial_port.is_open:
            self.serial_port.write(b'1')
            logger.debug('Laser turned OFF')

    def get_value_and_unit(self):
        """Returns the last read value of the distance and a string representing the unit"""
        return self.measured_distance.get_pos_with_offset(), self.unit

    def connect_to_lcd(self, lcd_display : QLCDNumber) -> None:
        """Connect the distance reading display to the given LCD QWidget"""
        self.measured_distance.add_lcd_display(lcd_display)

    def __repr__(self):
        return (f"<LaserController("
                f"COM_PORT={self.com_port!r}, "
                f"Offset_mm={self.offset_mm}, "
                f"Range_mm={self.laser_range_mm}, "
                f"MinSafeDist_um={self.min_distance_for_safety}, "
                f"FocusDist_um={self.focus_distance_micrometers}, "
                f"Connected={self.connexion_established}, "
                f"OutOfRange={self.out_of_range_flag}, "
                f"LastValue={self.measured_distance.value()}, "
                f"Unit={self.unit})>")
    
    def __str__(self):
        return (f"<LaserController("
                f"COM_PORT={self.com_port!r}, \n"
                f"Offset_mm={self.offset_mm}, \n"
                f"Range_mm={self.laser_range_mm}, \n"
                f"MinSafeDist_um={self.min_distance_for_safety}, \n"
                f"FocusDist_um={self.focus_distance_micrometers}, \n"
                f"Connected={self.connexion_established}, \n"
                f"OutOfRange={self.out_of_range_flag}, \n"
                f"LastValue={self.measured_distance.value()}, \n"
                f"Unit={self.unit})>\n")

def main():

    import sys
    from PyQt5.QtWidgets import QApplication, QMainWindow, QCheckBox, QVBoxLayout

    # Typical configuration
    config = configparser.ConfigParser()
    config.add_section('Distance_sensor')
    config.set('Distance_sensor', 'com_port', 'COM3')
    config.set('Distance_sensor', 'measuring_center_distance_mm', '100')
    config.set('Distance_sensor', 'measuring_range_mm', '70')
    config.set('Distance_sensor', 'min_safe_distance_mm', '97.250')
    config.set('Distance_sensor', 'focus_distance_mm', '1.383')

    ex = QApplication(sys.argv)

    app = QMainWindow()

    laser_controller = LaserController(config)
    print(laser_controller)
    
    try :
        laser_controller.start_connection()
    except Exception as err:
        print(err)

    layout = QVBoxLayout()
    lcd_display = QLCDNumber()
    on_off_btn = QCheckBox("Laser ON")
    layout.addWidget(lcd_display)
    layout.addWidget(on_off_btn)

    central_widget = QWidget()
    central_widget.setLayout(layout)
    app.setCentralWidget(central_widget)
    app.setGeometry(400, 400, 400, 400)

    laser_controller.connect_to_lcd(lcd_display)
    on_off_btn.stateChanged.connect(lambda flag: laser_controller.laser_on() if flag else laser_controller.laser_off())
    on_off_btn.stateChanged.connect(lambda flag: print(flag))

    app.show()
    sys.exit(ex.exec_())

if __name__ == "__main__":

    main()