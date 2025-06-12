"""This module contains the classes needed for amptek device control

These devices are X-RAY MCA detectors
"""
# Exporting the AmptekDevice class for use in other modules
__all__ = ["AmptekDevice"]

# Standard imports
import struct
import time
import os
from pathlib import Path
import random
import logging
import numpy

# Third-party imports
from overrides import override
import usb.core
import usb.util

# Importing project modules
try :
    from controllers.abstractcontroller import AbsController
    from controllers.threads.control_threads import SpectrumSeriesAcqJob
    from controllers.utils.hdf5_utils import Hdf5Handler
except ModuleNotFoundError :
    # For testing purpose
    from corapp.controllers.abstractcontroller import AbsController
    from corapp.controllers.threads.control_threads import SpectrumSeriesAcqJob
    from corapp.controllers.utils.hdf5_utils import Hdf5Handler

# Setting up logging
logger = logging.getLogger(f'core.{__name__}')

# Dictionary of configuration parameters and their descriptions
_CONF_PARAMS_MCA800D = {"RESC": "Reset Configuration",
                        "PURE": "PUR Interval on/off",
                        "MCAS": "MCA Source",
                        "MCAC": "MCA/MCS Channels",
                        "SOFF": "Set Spectrum Offset",
                        "GAIA": "Analog Gain Index",
                        "PDMD": "Peak Detect Mode (min/max)",
                        "THSL": "Slow Threshold",
                        "TLLD": "LLD Threshold",
                        "GATE": "Gate Control",
                        "AUO1": "AUX OUT Selection",
                        "PRER": "Preset Real Time",
                        "PREL": "Preset Life Time",
                        "PREC": "Preset Counts",
                        "PRCL": "Preset Counts Low Threshold",
                        "PRCH": "Preset Counts High Threshold",
                        "SCOE": "Scope Trigger Edge",
                        "SCOT": "Scope Trigger Position",
                        "SCOG": "Digital Scope Gain",
                        "MCSL": "MCS Low Threshold",
                        "MCSH": "MCS High Threshold",
                        "MCST": "MCS Timebase",
                        "AUO2": "AUX OUT 2 Selection",
                        "GPED": "G.P.Counter Edge",
                        "GPIN": "G.P. Counter Input",
                        "GPME": "G.P. Counter Uses MCA_EN",
                        "GPGA": "G.P. Counter Uses Gate",
                        "GPMC": "G.P. Counter Cleared With MCA",
                        "MCAE": "MCA/MCS Enable",
                        "TPEA": "peaking time",
                        "TFLA": "Flat Top Width",
                        "TPFA": "Fast channel peaking time",
                        "AINP": "Analog input pos/neg"}

# Defining the AmptekDevice class
class AmptekDevice(AbsController):

    """This class is a device controller for Amptek Device"""

    # Mapping of spectrum size based on channel
    _SPECTRUM_SIZE_VS_CHANNEL = {"1": 255,
                                 "2": 255,
                                 "3": 511,
                                 "4": 511,
                                 "5": 1023,
                                 "6": 1023,
                                 "7": 2047,
                                 "8": 2047,
                                 "9": 4095,
                                 "10": 4095,
                                 "11": 8191,
                                 "12": 8191}

    def __init__(self, idvendor, idproduct, researched_sn,
                 endpoint1_in=129, endpoint2_out=2, cmd_timeout=5,
                 answer_timeout=5, maxrf_device=False) -> None:
        # Initialize the device and its parameters
        self.idvendor = int(idvendor, 0)
        self.idproduct = int(idproduct, 0)
        self.connexion_established = False

        self.status = _AmptekStatus()
        self.endpoint2_out = endpoint2_out
        self.endpoint1_in = endpoint1_in
        self.cmd_timeout = cmd_timeout
        self.answer_timeout = answer_timeout
        self.chan_num = 512
        self.last_spectrum = 0
        self.is_maxrf_device = maxrf_device
        self.researched_sn = researched_sn

        self.calibration = {
            'order': 1,
            'A': 0,
            'B': 1,
            'C': 0
        }

        self.map_data = numpy.zeros((1, 1, self.chan_num))

    def init_new_maxrf_mapping(self, num_pixels_per_line, num_pixels_per_col):
        shape = (num_pixels_per_col, num_pixels_per_line, self.get_chan_num())
        self.map_data = numpy.zeros(shape)

    def get_chan_num(self):
        """Return the number of channels"""
        return self.chan_num

    def get_map_data(self):
        return self.map_data

    def get_spectrum(self, get_status: bool = False,
                     clear_spectrum: bool = False
                     ) -> tuple[int, list[int], list[int]]:
        """
        Write "get spectrum" command to Amptek device 
        then read the answer
        
        @Args
        -   get_status : bool : 
        Whether or not retreive AMPTEK device status simultaneously
        -   clear_spectrum : bool : 
        Whether or not reset the spectrum value to 0 
        for all channels after retreiving it

        @Return
        -   max_chan : int : Number of channels for the given spectrum
        -   int_spectrum : list[int] :
        the spectrum data as a list of integer representing 
        the value of each channel
        -   status : bytearray | None : 
        the status as a bytearray (None if status not needed)

        @Exceptions
        -   TimeoutError : in case of timeout in reading the command
        -   ConnectionError : in case of other USBError
        -   BufferError : If the checksum is wrong

        """

        # Setup get_spectrum command options
        pid2 = 0x01
        if get_status is True:
            pid2 += 2
        if clear_spectrum is True:
            pid2 += 1

        # Write get_spectrum command to AmptekDevice with given options
        self._write_cmd(0x02, pid2, '')
        # Read the answer
        max_chan_code, answer = self._read_answer()
        # Retrieve the number of channels for the acquired spectrum
        max_chan = AmptekDevice._SPECTRUM_SIZE_VS_CHANNEL[str(max_chan_code)]
        # Retrieve the spectrum (binary format)
        bin_spectrum = answer[:3*(max_chan+1)]
        # Reconstruct the spectrum (integer format)
        int_spectrum = [
            int.from_bytes(bin_spectrum[index:index+3], 'little')
            for index in range(0, len(bin_spectrum)-2, 3)
            ]
        # Retrieve status if needed
        status = answer[3*(max_chan+1):] if get_status is True else None

        if status is not None :
            self.status = _AmptekStatus(status)

        return max_chan, int_spectrum, status

    def _get_spectrums_series(self,
                            dwell_time: float,
                            acq_number: int,
                            series_index: int
                            ) -> None:
        """
        Gets a given number of spectrums at given sampling frequency. 
        Spectrums are saved to a HDF5 file.
        
        @Args
        - dwell_time: sampling in milliseconds (kHz)
        - acq_number: Number of spectrums to acquire
        - series_index : Index of the series for indexing of the map data

        """
        try :
            overlap_time_secs = 0

            # Clear spectrum (and get_status)
            self._write_cmd(0x02, 0x02, '')
            t_start = time.perf_counter_ns()

            # Read the answer (to clear the buffer)
            _, _ = self._read_answer()

            # Sleep for pixel 0
            remaining = (dwell_time - (time.perf_counter_ns() - t_start)*10**-6)/1000
            time.sleep(remaining)

            for spectrum_sample_index in range(acq_number):

                t_start = time.perf_counter_ns()

                # Query spectrum from device
                try :
                    _, int_spectrum, _ = self.get_spectrum(
                                                        get_status=False,
                                                        clear_spectrum=True)
                    self.last_spectrum = int_spectrum
                except TimeoutError:
                    logger.debug('Skipping pixel %i,%i \
                                    for time out in write or read to Amptek',
                                    spectrum_sample_index, series_index)
                    int_spectrum = self.last_spectrum
                except BufferError:
                    logger.debug('Skipping pixel %i,%i for \
                                    time out in write or read to Amptek',
                                    spectrum_sample_index, series_index)
                    int_spectrum = self.last_spectrum

                if series_index % 2 == 0:
                    # Even line, feed from left to right
                    self.map_data[series_index, spectrum_sample_index] = int_spectrum
                else:
                    # Odd line, feed from right to left
                    self.map_data[series_index, acq_number - (spectrum_sample_index+1)] = int_spectrum

                # Measure the time taken for above execution and remove overlap time from previous sample
                elapsed_time = (time.perf_counter_ns() - t_start) / (10**6) # milliseconds
                remaining_time = (dwell_time - elapsed_time) / (10**3) - overlap_time_secs # seconds

                if remaining_time < 0:
                    # Time taken for above execution excedeed the dwell time.
                    logger.error(
                        f'Could not get spectrum fast enough for pixel ({series_index=}, {spectrum_sample_index=}): sampling freq = {dwell_time}ms, time spent getting/writing spectrum : {elapsed_time}ms, overlap time from previous pixel : {overlap_time_secs*10**3}ms')
                    overlap_time_secs = 0
                else:
                    # Wait the remaining time to reach sampling rate.
                    t_start2 = time.perf_counter()
                    time.sleep(float(remaining_time))
                    measured_wait = time.perf_counter() - t_start2
                    # Measure the actual time waited to report overlap on next sample
                    overlap_time_secs =  measured_wait - remaining_time # seconds
        except Exception as e:
            logger.exception(f'Error in get_spectrums_series : {e}')

    def get_hardware_config(self) -> dict:
        """
        get hardware configuration from device
        """
        data=''

        for conf_param in _CONF_PARAMS_MCA800D:
            data += conf_param + '=?;'

        self._write_cmd(0x20,0x03,data)
        _, answer = self._read_answer()
        length = len(answer)
        hw_format = f'{length}s'
        unpacked_config = struct.unpack(hw_format, answer)
        hw_config = {}

        for param in str(unpacked_config[0])[2:].split(';'):
            pv = param.split('=')
            if len(pv) == 2:
                hw_config.setdefault(pv[0], pv[1])
        return hw_config

    def set_configuration(self, configuration = None):
        r"""Set the active configuration.

        The configuration parameter is the bConfigurationValue field of 
        the configuration you want to set as active. If you call this 
        method without parameter, it will use the first configuration 
        found. As a device hardly ever has more than one configuration, 
        calling the method without arguments is enough to get the device
        ready.
        """

        self.device.set_configuration(configuration=configuration)
        time.sleep(.4) # Tempo for configuration writing in flash memory
                        # (80 to 400ms according to constructor)

    def set_calibration(self, calib_a:float, calib_b:float, calib_c:float, order:int):
        self.calibration['order'] = order
        self.calibration['A'] = calib_a
        self.calibration['B'] = calib_b
        self.calibration['C'] = calib_c

    def get_calibration(self):
        return self.calibration

    def update_status(self):
        """
        Read status from device
        """
        logger.debug('Updating status')
        data = ''
        self._write_cmd(1, 1, data)
        _, new_status = self._read_answer()
        self.status = _AmptekStatus(new_status)

    def get_status(self):
        """Returns the status"""
        return self.status

    def query_sn(self) -> int:
        self.update_status()
        status = self.get_status()
        return status.get("SerialNumber")

    def hardware_config_to_string(self):
        """Reads and returns a string
        representation of the hardware configuration"""
        hw_config = self.get_hardware_config()
        string = \
        f'=============== {self.status.get('device')} CFG ==================\n'
        for key, value in hw_config.items():
            string += f'{key} : {value}\n'
        string += '================================================\n'

        logger.info(string)
        return string

    def enable_mca_mcs(self) -> None:
        """
        Turns ON the Amptek device MCA acquisition.
        """
        logger.debug('Starting MCA/MCS')
        data = ''
        self._write_cmd(0xF0, 0x02, data)
        self._check_ok_ack_packet()

    def disable_mca_mcs(self) -> None:
        """
        Turns OFF the Amptek device MCA acquisition.
        """
        logger.debug('Stopping MCA/MCS')
        data = ''
        self._write_cmd(0xF0, 0x03, data)
        self._check_ok_ack_packet()

    def stop_scan(self) -> None :
        """Stops the acquisition Job (if any) and turn OFF acquisition
        """

        # Disable detector acquisition
        self.disable_mca_mcs()

    def _set_device_config(self, device_param:str, param_value:str|float|int):
        """
        Sends a configuration string to the device
        """
        str_config = f'{device_param}={param_value};'
        self._write_cmd(0x20, 2, str_config)

        self._check_ok_ack_packet()

    def _write_cmd(self, req_pid1:bytes, req_pid2:bytes, data:str) -> None:
        """
        Writes raw command to Amptek device
        @Args:
        - req_pid1 : Packet ID field 1, define the meaning of 
        the packet, see amptek programmers guide for more info
        - req_pid2 : Packet ID field 2, define the meaning of 
        the packet, see amptek programmers guide for more info
        - data :     Data to be sent to Amptek device over usb

        @Exceptions
        - TimeoutError : in case of timeout in writing the command
        - ConnectionError : in case of other USBError
        """
        # Prepare command structure
        header = bytearray(4)
        header[0] = 0xF5        # SYNC1
        header[1] = 0xFA        # SYNC2
        header[2] = req_pid1
        header[3] = req_pid2
        pckged_msg = AmptekDevice._packmsg(header, data)

        # Write to device
        try:
            self.device.write(self.endpoint2_out, pckged_msg, self.cmd_timeout)
        except usb.core.USBTimeoutError as e :
            # logger.exception(str(e))
            raise TimeoutError(e) from e
        except usb.core.USBError as e:
            # logger.exception(str(e))
            raise usb.core.USBError(e) from e
        except KeyError :
            pass

    def _read_answer(self) -> tuple[int, int, bytearray]:
        """
        Reads raw answer from AmptekDevice over usb

        @Return:
        - Channels number code (to retrieve the number of channels)
        - ByteArray of data (contains spectrum or spectrum + status)

        @Exceptions
        - TimeoutError : in case of timeout in reading the command
        - ConnectionError : in case of other USBError
        - BufferError : If the checksum is wrong

        """
        try:
            device_message = self.device.read(
                                            self.endpoint1_in,
                                            65535,
                                            timeout=self.answer_timeout
                                            )
        except usb.core.USBTimeoutError as e :
            logger.exception(str(e))
            raise TimeoutError(e) from e
        except usb.core.USBError as e:
            logger.exception(str(e))
            raise ConnectionError(e) from e

        checksum = AmptekDevice._unpackint(device_message[-2:])
        control = AmptekDevice._checksum(device_message[:-2])
        try:
            assert checksum == control
        except AssertionError as e:
            raise BufferError("Error in buffer") from e

        return (device_message[3], device_message[6:-2])

    def _check_ok_ack_packet(self):
        """
        Reads the buffer from amptek device and checks that it contains
        an "Aknowledge OK" packet

        @Exceptions:
            - BufferError : if the packet in buffer is not recognized as
            a "Aknowledge OK" packet
        """

        device_message = self.device.read(
                                        self.endpoint1_in,
                                        65535,
                                        timeout=self.answer_timeout)

        sync1 = device_message[0]
        sync2 = device_message[1]
        pid1 = device_message[2]
        pid2 = device_message[3]
        len_msb = device_message[4]
        len_lsb = device_message[5]
        checksum_msb = device_message[6]
        checksum_lsb = device_message[7]

        try :
            assert sync1 == 0xF5
            assert sync2 == 0xFA
            assert pid1 == 0xFF
            assert pid2 == 0
            assert len_msb == 0
            assert len_lsb == 0
            assert checksum_msb == 0xFD
            assert checksum_lsb == 0x12
        except AssertionError :
            received = bytearray([sync1, sync2, pid1, pid2, len_msb,
                                  len_lsb, checksum_msb, checksum_lsb])
            expected = bytearray([0xF5, 0xFA,  0xFF,  0,  0,
                                  0,  0xFD,  0x12])
            bufferr = BufferError(f'Acknowledge packet : "OK" was expected but \
                                  not received. \
                                  Expected : {expected}, \
                                  Received : {received}')
            logger.exception(bufferr)
            raise bufferr

        logger.debug('Acknowledge packet : "OK" received')

    @staticmethod
    def _checksum(data: bytearray) -> int:
        """
        Calculates the checksum expected value from the given data

        @Args
            - data : the buffer data to calculate the checksum from
        
        @Returns
            - int : checksum expected value for the given data
        """
        checksum = 0
        for b in bytearray(data):
            checksum += int(b)
        return ((checksum & 0xffff) ^ 0xffff) + 1

    @staticmethod
    def _packint(short_int:int) -> bytes:
        """
        Converts short int to 2 bytes (big endian)
        int: unsigned short to be converted to bytes
        """
        return struct.pack('>H', short_int)

    @staticmethod
    def _packmsg(header:bytearray, data:str) -> bytearray:
        """
        Prepares a msg to be sent to the device
        """
        length = len(data)
        byte_array = header + AmptekDevice._packint(length) \
                            + bytes(data, 'ascii')
        checksum = AmptekDevice._checksum(byte_array)
        return byte_array + AmptekDevice._packint(checksum)

    @staticmethod
    def _unpackint(twobytes:bytearray) -> int:
        """
        Convert 2 bytes to integer (big endian)
        twobytes: bytes to be converted to int
        """
        res = struct.unpack('>H', twobytes)
        return res[0]

    @staticmethod
    def _custom_sleep_ms(milliseconds:float) -> None:
        """Accurate sleep during given milliseconds"""

        t_start = time.perf_counter_ns()
        t = time.perf_counter_ns() - t_start
        while t < (milliseconds * 1e6):
            time.sleep(0.001) # sleep 1ms
            t = time.perf_counter_ns() - t_start

    @override
    def start_connection(self) -> None:
        """
        Attempts initialising the USB connection with the AmptekDevice
        
        @Exceptions

            - ConnectionError : if the device can't be found
        """

        # List all Amptek devices connected to the computer via USB
        logger.debug(f'...Attemping to connect to AMPTEK Device : {self.idvendor=}, {self.idproduct=}, {self.researched_sn=}')
        devices_list: list[usb.core.Device] = list(usb.core.find(idVendor=self.idvendor,
                                                    idProduct=self.idproduct,
                                                    find_all=True))
        logger.debug(f'list of USB devices found : {devices_list}')
        
        # Check if any device is found
        if len(devices_list) == 0:
            self.connexion_established = False
            err_msg = f"Amptek USB device not found with idVendor: {self.idvendor}, idProduct: {self.idproduct}"
            conerr = ConnectionError(err_msg)
            logger.warning(err_msg)
            raise conerr

        # Loop through the devices to find the one with the correct serial number
        found_sn = False
        for dev in devices_list:
            self.device = dev
            try :
                sn_device = self.query_sn()
            except usb.core.USBError :
                # This error occurs when the device interface is already claimed
                sn_device = -1
                pass 
            if int(sn_device) == int(self.researched_sn):
                found_sn = True
                break
            else:
                self.stop_connection()

        # If no device with the correct SN is found, raise an error
        if not found_sn :
            self.connexion_established = False
            err_msg = f"Could not find any Amptek device with Serial Number : \'{self.researched_sn}\'"
            logger.warning(err_msg)
            raise ConnectionError(err_msg)

        # If the device is found, set it as the active device
        logger.info(f'AMPTEK Device : idvendor={self.idvendor}, idproduct={self.idproduct} SN={self.researched_sn} connected')
        self.connexion_established = True
        self.device.set_configuration()
        usb.util.claim_interface(self.device, 0)

        # Query the device to get the number of channels
        max_chan, _, _ = self.get_spectrum()
        self.chan_num = max_chan + 1

    def stop_connection(self):
        """Release and dispose USB interfaces with the device"""
        usb.util.release_interface(self.device, 0)
        usb.util.dispose_resources(self.device)
        self.connexion_established = False

class _AmptekStatus(dict):

    _DEVICES_ID = {
        '0' : 'DP5',
        '1' : 'PX5',
        '2' : 'DP5G',
        '3' : 'MCA8000D',
        '4' : 'TB5',
        '5' : 'DP5-X',
    }

    def __init__(self, bytes_buffer:bytearray = bytearray(64)) -> None:

        try :
            assert len(bytes_buffer) == 64
        except AssertionError as asserr:
            raise ValueError(f"_AmptekStatus constructor expects a \
                             64-bytes array\nbyte_array given : {bytes_buffer} \
                             \nlength : {len(bytes_buffer)}") from asserr

        self.update({'DEVICE' :
                     _AmptekStatus._DEVICES_ID.get(str(bytes_buffer[39]))})
        self.update({'FastCount' :
                     _AmptekStatus._fourbytestoint(bytes_buffer[0:4])})
        self.update({'SlowCount' :
                      _AmptekStatus._fourbytestoint(bytes_buffer[4:8])})
        self.update({'GP_COUNTER' :
                     _AmptekStatus._fourbytestoint(bytes_buffer[8:12])})
        self.update({'AccuTime' :
                     bytes_buffer[12] \
                        + (_AmptekStatus._threebytestoint(
                            bytes_buffer[13:16]
                            ) * 100)}) # in msec
        self.update({'RealTime' :
                     _AmptekStatus._fourbytestoint(
                         bytes_buffer[20:24])}) # in msec
        self.update({'FirmwareVersion' :
                      f'{bytes_buffer[24]>>4}.{bytes_buffer[24] & 0x0F}'})
        self.update({'FPGA' :
                      f'{bytes_buffer[25]>>4}.{bytes_buffer[25] & 0x0F}'})
        self.update({'LiveTime' :
                      _AmptekStatus._fourbytestoint(
                          bytes_buffer[16:20])
                          if self.get("DEVICE") == "MCA8000D" else 'N/A'})
        self.update({'SerialNumber' :
                      (_AmptekStatus._fourbytestoint(
                          bytes_buffer[26:30])
                          if bytes_buffer[29] < 128 else -1)})
        self.update({'DetectorTemp' :
                      (int.from_bytes(bytes_buffer[32:34])*0.1)-273.15}) # in °C
        self.update({'BoardTemp' :
                      range(-128,128)[bytes_buffer[34]]}) # in °C
        self.update({'MCA_EN' :
                      (bytes_buffer[35] & 32) == 32})
        self.update({'PRECNT_REACHED' :
                      (bytes_buffer[35] & 16) == 16})
        self.update({'SCOPE_DR' :
                      (bytes_buffer[35] & 4) == 4})
        self.update({'DP5_CONFIGURED' :
                      (bytes_buffer[35] & 2) == 2})
        self.update({'AOFFSET_LOCKED' :
                      (bytes_buffer[36] & 128) == 128})
        self.update({'MCS_DONE' :
                      (bytes_buffer[36] & 64) == 64})
        self.update({'b80MHzMode' :
                      (bytes_buffer[36] & 2) == 2})
        self.update({'bFPGAAutoClock' :
                      (bytes_buffer[36] & 1) == 1})
        self.update({'PC5_PRESENT' :
                      (bytes_buffer[38] & 128) == 128})
        self.update({'PC5_HV_POL' :
                      ((bytes_buffer[38] & 64) == 64
                       if self.get('PC5_PRESENT') is True else False)})
        self.update({'PC5_8_5V' :
                      ((bytes_buffer[38] & 32) == 32
                       if self.get('PC5_PRESENT') is True else False)})
        self.update({'DPP_ECO' :
                      bytes_buffer[49]})
        # bytes 50 to 63 unused

    def __repr__(self) -> str:

        device = self.get("DEVICE")

        string =  f'================ {device} status ===================\n'
        string += f'Device              : {device} \n'
        string += f'FirmwareVersion     : {self.get("FirmwareVersion")}\n'
        string += f'FPGA                : {self.get("FPGA")}\n'
        string += f'SerialNumber        : {self.get("SerialNumber")}\n'
        string += f'AccumulationTime    : {self.get("AccuTime")} msec\n'
        string += f'RealTime            : {self.get("RealTime")} msec\n'
        string += f'DetectorTemperature : {self.get("BoardTemp")} °C\n'
        string += f'BoardTemperature    : {self.get("BoardTemp")} °C\n'
        string += f'FastCount           : {self.get("FastCount")} \n'
        string += f'SlowCount           : {self.get("SlowCount")} \n'
        string += f'GP Counter          : {self.get("GP_COUNTER")} \n'
        string += f'MCA_EN              : '
        string += 'Yes\n' if self.get("MCA_EN") else 'No\n'
        string += f'MCS_DONE            : '
        string += 'Yes\n' if self.get("MCS_DONE") else 'No\n'
        string += f'PRECNT_REACHED      : '
        string += 'Yes\n' if self.get("PRECNT_REACHED") else 'No\n'
        string += f'PC5_PRESENT         : '
        string += 'Yes\n' if self.get("PC5_PRESENT") else 'No\n'
        string += f'DPP_ECO             : {self.get("DPP_ECO")}\n'
        string += '====================================================\n'

        return string

    @staticmethod
    def _fourbytestoint(fourbytes:bytearray) -> int:
        """
        Convert four bytes (little endian) to integer

        fourbytes: 4 bytes to be converted to integer
        """
        assert len(fourbytes) == 4
        nums = struct.unpack('<I', fourbytes)
        return nums[0]

    @staticmethod
    def _threebytestoint(threebytes: bytes | bytearray):
        """
        Convert three bytes to integer

        threebytes: 3 bytes to be converted to integer
        """
        assert len(threebytes) == 3
        num = int(threebytes[0]) \
            + (int(threebytes[1]) * 256) \
            + (int(threebytes[2]) * 65536)
        return num


if __name__ == "__main__":
    pass
