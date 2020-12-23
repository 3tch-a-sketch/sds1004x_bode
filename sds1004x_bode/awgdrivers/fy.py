"""Minimalist driver for FYXXXX signal generators.

  For a full-featured implementation of all AWG features, see:
    https://github.com/mattwach/fygen
"""

import serial
from base_awg import BaseAWG

AWG_ID = "fy"
AWG_OUTPUT_IMPEDANCE = 50.0
MAX_READ_SIZE = 256
RETRY_COUNT = 3
VERBOSE = True  # Set to True for protocol debugging 

class Error(Exception):
  pass

class CommandTooShortError(Error):
  pass

class CommandNotAcknowledgedError(Error):
  pass

class InvalidChannelError(Error):
  pass

def debug(msg, *args):
  if VERBOSE:
    print(msg % args)

class FygenAWG(BaseAWG):
  """Driver API."""
  
  SHORT_NAME = "fy"

  def __init__(self, port, baud_rate=115200, timeout=5):
    self.fy = None
    self.port = None
    self.serial_path = port
    self.baud_rate = baud_rate
    self.timeout = timeout
    self.load_impedance = {
            1: 1000000.0,
            2: 1000000.0,
    }

  def connect(self):
    if self.port:
      return

    self.port = serial.Serial(
        port=self.serial_path,
        baudrate=115200,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        rtscts=False,
        dsrdtr=False,
        xonxoff=False,
        timeout=self.timeout)

    debug('Connected to %s', self.serial_path)
    self.port.reset_output_buffer()
    self.port.reset_input_buffer()
  
  def disconnect(self):
    if self.port:
      debug('Disconnected from %s', self.serial_path)
      self.port.close()
      self.port = None
  
  def initialize(self):
      self.connect()
      self.enable_output(1, False)
      self.enable_output(2, False)
  
  def get_id(self):
      return AWG_ID
  
  def enable_output(self, channel, on):
    self._retry(
        channel,
        'N',
        '1' if on else '0',
        '255' if on else '0') 
  
  def set_frequency(self, channel, freq):
    uhz = int(freq * 1000000.0)
    self._retry(
        channel,
        'F',
        '%014u' % uhz,
        '%08u.%06u' % (int(freq), int(uhz % 1000000)))
      
  def set_phase(self, phase):
    self._retry(
        2,  # always channel 2 (not sure why)
        'P',
        '%.3f' % phase,
        '%u' % (phase * 1000))

  def set_wave_type(self, channel, wvtp):
    del wvtp  # This parameter is ignored, always set a sin wave
    self._retry(channel, 'W', '0', '0')
  
  def set_amplitue(self, channel, amp):
    loadz = self.load_impedance[channel]
    volts = round(amp * loadz / (AWG_OUTPUT_IMPEDANCE + loadz), 4)
    self._retry(
        channel,
        'A',
        '%.4f' % volts,
        '%u' % (volts * 10000))
  
  def set_offset(self, channel, offset):
      pass
  
  def set_load_impedance(self, channel, z):
      if z > 10000000.0:
          # We can only set voltage within one hundredth of a volt and
          # it's rounded.
          z = 10000000.0
      self.load_impedance[channel] = z

  def _recv(self, command):
    """Waits for device."""
    response = self.port.read_until(size=MAX_READ_SIZE).decode('utf8')
    debug('%s -> %s', command.strip(), response.strip())
    return response

  def _send(self, command, retry_count=5):
    """Sends a low-level command. Returns the response."""
    debug('send (attempt %u/5) -> %s', 6 - retry_count, command)
    if len(command) < 3:
      raise CommandTooShortError('Command too short: %s' % command)

    data = command + '\n'
    data = data.encode()
    self.port.reset_output_buffer()
    self.port.reset_input_buffer()
    self.port.write(data)
    self.port.flush()

    response = self._recv(command)

    if not response and retry_count > 0:
      # sometime the siggen answers queries with nothing.  Wait a bit and try
      # again
      time.sleep(0.1)
      return self.send(command, retry_count - 1)

    return response.strip()

  def _retry(self, channel, command, value, match):
    """Retries the command until match is satisfied."""
    if channel == 1:
      channel = 'M'
    elif channel == 2:
      channel = 'F'
    else:
      raise InvalidChannelError('Channel shoud be 1 or 2')

    for _ in range(RETRY_COUNT):
      self._send('W' + channel + command + value)
      if self._send('R' + channel + command) == match:
        debug('matched %s', match)
        return
      debug('mismatched %s', match)

    raise CommandNotAcknowledgedError(
        '%s did not produce an expectd response after %d retries' % (
            prefix + send_suffix, RETRY_COUNT))
  
