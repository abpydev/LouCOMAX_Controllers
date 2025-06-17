
__all__ = ["SpectrumSeriesAcqJob", "CyclicJob", "MappingJob"]

import threading
from PyQt5.QtCore import pyqtSignal

import logging
logger = logging.getLogger(f'core.{__name__}')

_thread_list:list[threading.Thread]=[]

class SpectrumSeriesAcqJob(threading.Thread):

    SigSpectrumSeriesAcqEnd = pyqtSignal(name="SigSpectrumSeriesAcqEnd")
    SigSpectrumSeriesAcqStart = pyqtSignal(name="SigSpectrumSeriesAcqStart")

    def __init__(self, target, *args, daemon=True, **kwargs):
        super().__init__()
        self.target = target
        self.args = args
        self.kwargs = kwargs

    def run(self):
        self.SigSpectrumSeriesAcqStart.emit()
        self.target(*self.args, **self.kwargs)
        self.SigSpectrumSeriesAcqEnd.emit()

class CyclicJob(threading.Thread):
    
    def __init__(self, target, interval:float, *args,  name="CyclicJob", daemon=True, stop_event=None, **kwargs):
        threading.Thread.__init__(self, name=name, daemon=daemon)
        self.daemon = daemon
        if stop_event is None:
            self.stopped_event = threading.Event()
        else:
            self.stopped_event = stop_event
        self.interval = interval
        self.target = target
        self.args = args
        self.kwargs = kwargs
        self.setName(f'{self.name}-CyclicJob')
        _thread_list.append(self)

    def stop(self):
        logger.debug(f'Stop {self.name}')
        self.stopped_event.set()
        self.join(timeout=1)

    def run(self):
        logger.debug(f'Start {self.name} for {self.target}, To be called every {self.interval}s')
        while not self.stopped_event.wait(timeout=self.interval):
            try :
                self.target(*self.args, **self.kwargs)
            except RuntimeError as runtime_err :
                logger.exception(f'{runtime_err}')

class MappingJob(threading.Thread):
    def __init__(self, target, stop_event:threading.Event, *args, **kwargs) -> None:
        threading.Thread.__init__(self)
        self.daemon = False
        self.stopped_event = stop_event
        self.target = target
        self.args = args
        self.kwargs = kwargs
        self.name = f'{self.name}-MappingJob'
        _thread_list.append(self)

    def stop(self):
        logger.debug(f'Stop {self.name}')
        self.stopped_event.set()
        self.join(timeout=1)

    def run(self):
        logger.debug(f'Start {self.name} for {self.target}')
        while not self.stopped_event.is_set(): # Check if stopped
            try:
                self.target(*self.args, **self.kwargs)
            except Exception as e:
                logger.exception(f'Exception occurred in {self.name}: {e}')

if __name__ == "__main__":
    
    pass