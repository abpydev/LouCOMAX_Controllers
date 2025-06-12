"""This module defines all the classes needed for CSU2 unit (RX controller)"""
# Standard imports
import logging
import configparser
import socket

# Third-party imports
import csu2controller

# logger configuration
logger = logging.getLogger(f'core.{__name__}')

class RXController(csu2controller.CSU2Controller):

    ERROR_CODES = { 
                    '1111' : "Safety line at housing not connected (RJ45 connector)",
                    '1111' : "Real time clock broken or battery empty",
                    '2112' : "External interlock (RJ45 connector at tube)",
                    '1113' : "Interlock of HV-Generator",
                    '2112' : "Temperature of LED-board critical",
                    '3111' : "Temperature sensor at LED-board broken / not connected",
                    '3112' : "Temperature sensor at shutter-board broken / not connected",
                    '3121' : "Temperature of LED-board above limit",
                    '3122' : "Temperature of shutter-board above limit",
                    '3211' : "HV LED at tube housing broken",
                    '3221' : "Shutter hangs or shutter light bulb broken",
                    '3222' : "Shutter hangs or shutter-LEDs at tube housing broken",
                    '3321' : "Vacuum switch 1 broken",
                    '3322' : "Vacuum switch 2 broken",
                    '3331' : "HV powered on, and filament cable not (properly) connected",
                    '3332' : "'PC'-mode active and HV on / shutter opened, but communication with PC timed out",
                    '3333' : "No connection to HV generator. Power failure? Safety relais?"
                    }

    def __init__(self, controllers_config:configparser.ConfigParser):
        csu_ip_address = controllers_config.get('CSU_RX_Control','CSU_SOCKET_IP')
        csu_port = int(controllers_config.get('CSU_RX_Control','CSU_SOCKET_PORT'))
        super().__init__(csu_ip_address, csu_port)
        
        self.connexion_established = False

    def start_connection(self) -> None:
        """
        Attempts initialising the Socket IP connection with the CSU
        
        @Exceptions
            - ConnectionError : if the device can't be found
        """
        if self.is_socket_closed():
            try :
                logger.debug(f'...Attempting connection with CSU RX Controller at IP: {self.ip_address}:{self.port}')
                self.connect()
                self.connexion_established = True
                logger.info(f'CSU RX Controller connected at IP address: {self.ip_address}:{self.port}')
            except TimeoutError as timeouterr:
                self.connexion_established = False
                conerr = ConnectionError(f"CSU controller device not found with IP address: {self.ip_address}:{self.port}")
                logger.warning(conerr)
                raise conerr
            except ConnectionRefusedError as conn_ref_err:
                self.connexion_established = False
                conerr = ConnectionError(f"CSU controller occupied by some other software")
                logger.warning(conerr)
                raise conerr from conn_ref_err
        else : 
            logger.info(f"CSU RX Controller still connected at IP address: {self.ip_address}:{self.port}")
            self.connexion_established = True

    def stop_connection(self) -> None:
        """Disconnects the CSU RX controller"""
        print(self.query_is_shutter_open())
        if self.query_is_shutter_open():
            self.open_shutter('NO') #Close the shutter before disconnecting
        hv_state = self.query_actual_state_hv().split(" ")
        if len(hv_state) > 1 and hv_state[1] in ['ON', '+']:
            self.power_hv('NO') #Turn off the HV before disconnecting
        if not self.is_socket_closed():
            self.disconnect()
        self.connexion_established = False

    def is_connected(self) -> bool:
        return self.connexion_established
    
    def is_socket_closed(self) -> bool:
        if self.socket is None : 
            return True
        try:
            self.query_ok()
        except Exception as e:
            logger.debug(f"Exception when checking if a socket is closed: {e}")
            return True
        return False

    def query_is_shutter_open(self) -> bool:
        """Query the CSU controller for shutter open/close state.
        
        Returns True if shutter is open, else returns False"""

        str_shutter_state = self.query_shutter_state()
        return True if str_shutter_state in ('+',) else False

def module_test():

    CONTROLLERS_CONFIG = configparser.ConfigParser()
    config_filename = "C:\\Users\\CXRF\\Code\\depthpaint-c-xrf-interface\\corapp\\configurations\\main_config.cfg"
    CONTROLLERS_CONFIG.read(config_filename)

    rx_controller = RXController(CONTROLLERS_CONFIG)
    
    rx_controller.connect()
    resp = rx_controller.query_error_code()

if __name__ == "__main__":
    module_test()
