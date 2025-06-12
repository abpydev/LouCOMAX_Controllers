
# Pymca imports
from PyQt5.QtWidgets import QAbstractSlider, QLCDNumber
from PyQt5.QtGui import QColor

class AbsSliderPosition(QAbstractSlider):

    """
    This class is based on QAbstractSlider from PyQT5
    
    This class aims at :
        -   holding an "slider like" absolute position
        -   allow to connect a lcd discplay to this absolute position
        -   allow to update the display anytime the value is changes
    """

    def __init__(self, range=[0, 150000]) -> None:
        super().__init__()

        self.lcd_displays:list[QLCDNumber] = []
        self.setRange(range[0], range[1])
        self.offset = 0

    def add_lcd_display(self, lcd_display : QLCDNumber) -> None:
        """Connect a new LCD display"""
        self.lcd_displays.append(lcd_display)
        self.valueChanged.connect(self.update_lcd)
        self.update_lcd("------")

    # @pyqtSlot(int)
    def update_lcd(self, value: int|str) -> None:
        """Update all connected LCD displays"""
        for lcd_disp in self.lcd_displays:
            try :
                lcd_disp.display(int(value)-int(self.offset))
            except RuntimeError :
                pass
            except ValueError :
                lcd_disp.display(value)

    def set_lcd_color(self, num_color='red', background_color='blue'):
        """Set color for all LCD displays (that are connected)"""
        for lcd_display in self.lcd_displays:
            lcd_display.setStyleSheet(f"QLCDNumber {{color : {num_color}; background-color:{background_color};}}")

    def get_pos_with_offset(self) -> int:
        """Get absolute position with offset"""
        return self.value() - self.offset

    def get_pos(self) -> int:
        """Get absolute position"""
        return self.value()

    def set_offset(self, offset: int):
        """Set the offset
         
        NB : Only used when displaying the value"""
        self.offset = offset