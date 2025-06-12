"""Module for Abstract device controller class"""

__all__ = ["AbsController"]

from abc import ABC, abstractmethod

class AbsController(ABC):
    """This class is a high level abstracted class for controllers

    It is intended to be overriden by device controllers

    The following methods are mandatorily overriden :
        - start_connection(self)
    """

    def __init__(self):
        self.connexion_established = False

    def is_connected(self) -> bool:
        """Returns True if the device is connected, else returns False"""
        return self.connexion_established

    @abstractmethod
    def start_connection(self) -> None:
        """Initialise and start the connection to the device"""
        # Must be overriden by subclasses or will raise an error
