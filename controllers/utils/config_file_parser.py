
# Third-party imports
import configparser

class ConfigFileParser(configparser.ConfigParser):

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def read(self, filenames, encoding = None):
        self.filenames = filenames
        self.encoding = encoding
        return super().read(filenames, encoding)

    def refresh(self):
        self.read(self.filenames, self.encoding)
