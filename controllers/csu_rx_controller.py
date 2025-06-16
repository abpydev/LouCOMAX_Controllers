"""This module defines all the classes needed for CSU2 unit (RX controller)"""
# Standard imports
import logging
import configparser

# Third-party imports
import csu2controller

# logger configuration
logger = logging.getLogger(f'core.{__name__}')

class RXController(csu2controller.CSU2Controller):
    """RXController is a subclass of csu2controller.CSU2Controller designed to manage the connection and control of a CSU RX Controller device via a socket interface.
    
    Attributes:
        ERROR_CODES (dict): A mapping of error codes (as strings) to their corresponding human-readable error messages.
    
    Methods:
        __init__(controllers_config: configparser.ConfigParser):
            Initializes the RXController with the provided configuration, setting up the IP address and port for socket communication.
        start_connection() -> None:
            Attempts to establish a socket connection with the CSU RX Controller. Raises ConnectionError if the device cannot be found or is occupied.
        stop_connection() -> None:
            Safely disconnects from the CSU RX Controller, ensuring the shutter is closed and high voltage is powered off before disconnecting.
        is_connected() -> bool:
            Returns True if the connection to the CSU RX Controller is established, otherwise False.
        is_socket_closed() -> bool:
            Checks if the socket connection is closed. Returns True if closed or if an exception occurs during the check.
        query_is_shutter_open() -> bool:
            Queries the CSU RX Controller to determine if the shutter is open. Returns True if open, otherwise False."""

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
    
    def __repr__(self):
        return (f"<RXController("
                f"IP={self.ip_address!r}, "
                f"Port={self.port}, "
                f"Connected={self.connexion_established}, "
                f"SocketClosed={self.is_socket_closed()})>")

    def __str__(self):
        return (f"<RXController(\n"
                f"  IP={self.ip_address!r},\n"
                f"  Port={self.port},\n"
                f"  Connected={self.connexion_established},\n"
                f"  SocketClosed={self.is_socket_closed()}\n"
                f")>")
    
def main():

    config = configparser.ConfigParser()
    config.add_section('CSU_RX_Control')
    config.set('CSU_RX_Control', 'csu_socket_ip', '192.168.1.3')
    config.set('CSU_RX_Control', 'csu_socket_port', '23')

    rx_controller = RXController(config)
    
    print(rx_controller)

if __name__ == "__main__":
    main()
