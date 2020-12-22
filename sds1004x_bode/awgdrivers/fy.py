"""API wrapper for cpnverting BaseAWG->fygen."""

from base_awg import BaseAWG
import fygen

AWG_ID = "fy"
AWG_OUTPUT_IMPEDANCE = 50.0

class Error(Exception):
    pass

class InvalidChannelError(Error):
    pass

def _map_channel(channel):
    if channel not in (1,2):
        raise InvalidChannelError('Invalid channel: %d' % channel)
    return channel - 1

class FygenAWG(BaseAWG):
    '''
      Use fygen libraries (For feeltech devices.)
    '''
    
    SHORT_NAME = "fy"

    def __init__(self, port, baud_rate=None, timeout=None):
        self.fy = None
        self.port = port
        self.load_impedance = {
                1: 1000000.0,
                2: 1000000.0,
        }

    def connect(self):
        if not self.fy:
            self.fy = fygen.FYGen(serial_path=self.port)
    
    def disconnect(self):
        if self.fy:
            self.fy.close()
            self.fy = None
    
    def initialize(self):
        self.connect()
        self.enable_output(1, False)
        self.enable_output(2, False)
    
    def get_id(self):
        return AWG_ID
    
    def enable_output(self, channel, on):
        self.fy.set(_map_channel(channel), enable=on)
    
    def set_frequency(self, channel, freq):
        self.fy.set(
                _map_channel(channel),
                freq_uhz=int(freq * 1000000.0))
        
    def set_phase(self, phase):
        self.fy.set(
                1,  # always channel 2
                phase_degrees=phase)

    def set_wave_type(self, channel, wvtp):
        del wvtp  # This parameter is ignored
        self.fy.set(_map_channel(channel), wave=0)
    
    def set_amplitue(self, channel, amp):
        loadz = self.load_impedance[channel]
        self.fy.set(
            _map_channel(channel),
            volts=round(loadz / (AWG_OUTPUT_IMPEDANCE + loadz), 2))
    
    def set_offset(self, channel, offset):
        pass
    
    def set_load_impedance(self, channel, z):
        if z > 10000000.0:
            # We can only set voltage within one hundredth of a volt and
            # it's rounded.
            z = 10000000.0
        _map_channel(channel)  # only used for the check
        self.load_impedance[channel] = z
    
