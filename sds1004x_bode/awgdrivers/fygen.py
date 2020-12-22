"""Interface to FYXXXX signal generators.  Tested on an FY2300.

See help.py or use the command fygen.help() for more documentation.
"""
# pylint: disable=too-many-lines
# pylint: disable=too-many-public-methods

import sys
import time
import functools
import six
import serial

# Version numbers.  The minor version number increments when bugs are fixed
# or trivial features are added.  The major version number increments if a
# large feature is added or any APIs change in a non-compatible way.

# Maximum read size
MAX_READ_SIZE = 256

# Initialization state
SET_INIT_STATE = {
    'channel': (0, 1),
    'duty_cycle': 0.5,
    'enable': False,
    'freq_hz': 10000,
    'offset_volts': 0,
    'phase_degrees': 0,
    'volts': 5,
    'wave': 0,  # sin
}

class Error(Exception):
  """Base error class."""

class ChannelActiveError(Error):
  """Tried to define a waveform that is currently being generated"""

class CommandNotAcknowledgedError(Error):
  """The signal generator did not produce the expected response."""

class CommandTooShortError(Error):
  """Command is too short."""

class InvalidChannelError(Error):
  """Tried to pass an invalid channel number."""

class InvalidDutyCycleError(Error):
  """Tried to pass an invalid duty cycle."""

class InvalidVoltageError(Error):
  """Tried to pass an invalid voltage."""

class InvalidVoltageOffsetError(Error):
  """Tried to pass an invalid voltage offset."""

class InvalidFrequencyError(Error):
  """Tried to pass an invalid frequency."""

class UnknownParameterError(Error):
  """Asked for an unknown parameter."""

class UnknownWaveformError(Error):
  """Specified an unknown waveform."""

class FYGen(object):
  """Initialize a connection object with the signal generator.

  One can also simply point this to sys.stdout to see low-level details without
  talking with a real device.
  """
  def __init__(
      self,
      serial_path='/dev/ttyUSB0',
      port=None,
      default_channel=0,
      read_before_write=True,
      init_state=True,
      debug_level=0,
      timeout=5,
      max_volts=20.0,
      min_volts=-20.0,
      _port_is_serial=False,
  ):
    """Initializes connection to device.

    Args:
      serial_path: Path to usb serial device.  The format of this will vary by
        OS and will vary if you have multiple USB serial devices connected.
      port: If not None, specifies an output port.  In this case, path is
        ignored.  One usecase is to set port=sys.stdout to see the commands
        that will be sent.
      default_channel: The channel(s) used when the parameter is omitted.
      read_before_write: If True, then setting a parameter will first get it.
        If the parameter is already set to the desired value, the value is not
        sent.  This is useful because the signal generator responds to get
        operations much more quickly than set ones.
      init_state: If true, then the first set command will set all unspecified
        set parameters to a known state.
      debug_level: If 0, run silently.  If 1, print send and receive commands.
        If 2, print send and receive commands and wait for confirmation on
        send commands.
      timeout: How long to block reads and writes
      max_volts: Maximum volts/offset to allow
      min_volts: Minimum voltage offset to allow
    """
    if port:
      self.port = port
      self.is_serial = _port_is_serial

    else:
      self.port = serial.Serial(
          port=serial_path,
          baudrate=115200,
          bytesize=serial.EIGHTBITS,
          parity=serial.PARITY_NONE,
          stopbits=serial.STOPBITS_ONE,
          rtscts=False,
          dsrdtr=False,
          xonxoff=False,
          timeout=timeout)

      self.is_serial = True
      self.port.reset_output_buffer()
      self.port.reset_input_buffer()

    self.init_state = init_state
    self.read_before_write = read_before_write and self.is_serial
    self.debug_level = debug_level
    self.init_called_for_channel = set()
    self.default_channel = default_channel
    self.max_volts = max_volts
    self.min_volts = min_volts
    # Set to force sweep enable
    self.force_sweep_enable = False


  def close(self):
    """Closes serial port.  Call this at program exit for a clean shutdown."""
    self.port.close()
    self.port = None

  def send(self, command, retry_count=5):
    """Sends command, then waits for a response.  Returns the response."""
    if len(command) < 3:
      raise CommandTooShortError('Command too short: %s' % command)

    if self.debug_level == 2:
      six.moves.input('%s (Press Enter to Send)' % command)

    data = command + '\n'
    if self.is_serial:
      data = data.encode()
      self.port.reset_output_buffer()
      self.port.reset_input_buffer()

    self.port.write(data)
    self.port.flush()

    response = self._recv(command)

    if self.is_serial and not response and retry_count > 0:
      # sometime the siggen answers queries with nothing.  Wait a bit and try
      # again
      time.sleep(0.1)
      return self.send(command, retry_count - 1)

    return response.strip()

  # Note: unused-argument is disabled because we are capturing locals()
  # into a local variable and accessing these variables using that method
  #
  # pylint counts arguments as local variables:
  #pylint: disable=too-many-locals
  def set(
      self,
      channel=None,
      enable=None,         #pylint: disable=unused-argument
      wave=None,           #pylint: disable=unused-argument
      freq_hz=None,        #pylint: disable=unused-argument
      freq_uhz=None,       #pylint: disable=unused-argument
      volts=None,          #pylint: disable=unused-argument
      offset_volts=None,   #pylint: disable=unused-argument
      phase_degrees=None,  #pylint: disable=unused-argument
      duty_cycle=None,     #pylint: disable=unused-argument
      retry_count=3):
    """Change device settings.

    All parameters are optional and should be specified using named
    parameters (e.g. do not depend on parameter ordering.)

    Note there are two parameters for frequency: freq_hz and freq_uhz.  This is
    to avoid floating point rounding issues.  You can pass either freq_hz or
    freq_uhz but not both.

    Essential Args:
      channel: Can be a single number or a list of numbers.
      wave: must be 0
      freq_hz: An integer that specifies the frequency in hertz.
      freq_uhz: An integer that specifies the frequency in micro hertz.
      volts: A float that specifies the amplitude in volts.  This is rounded to
        the nearest hundredth of a volt.
      offset_volts: A float that specifies the voltage offset from zero
      duty_cycle: A float value from 0-1 that specifies the duty cycle.  Note
        that most waveforms ignore this setting.
      phase_degrees: An float value that specifies the phase offset in degrees
      retry_count: If > 0 and read_before_write is True the set results will
        be verified and resent if verification fails.
    """

    # Convert local arguments into a dictionary
    args_dict = dict(locals())

    if freq_hz is not None and freq_uhz is not None:
      raise InvalidFrequencyError(
          'Please, provide freq_hz or freq_uhz, not both.')

    if channel is None:
      channel = self.default_channel

    if not isinstance(channel, (tuple, list)):
      channel = (channel,)

    for c in channel:
      chan_dict = dict(args_dict)
      for _ in range(retry_count):
        if not self._set_for_channel(c, chan_dict):
          break  # nothing was sent
        if not self.read_before_write:
          break  # Since there is no get, we don't know if a retry is needed.
#pylint: enable=too-many-locals


  def _set_for_channel(self, channel, args_dict):
    """Implements set as above, but for a single channel.

    Args:
      channel: Channel to set (0 or 1)
      args_dict: Key-pairs.  e.g. {'volts': 5.5}.  NOTE: this function does
        modify args_dict by *removing* arguments that are already confirmed
        as set from the dictionary.  This is done to avoid redundant reads
        on retries.
    """
    if channel not in (0, 1):
      raise InvalidChannelError('Invalid channel: %s' % channel)

    # Implements init_state functionality.
    if self.init_state and channel not in self.init_called_for_channel:
      # This is the first call to set for this channel.  Fill in non-specified
      # arguments from SET_INIT_STATE
      self.init_called_for_channel.add(channel)
      for k, v in six.iteritems(SET_INIT_STATE):
        if args_dict[k] is None:
          args_dict[k] = v

    # convert args from a dict to a list so we can order enable=True/False
    # properly and remove null values
    args = list((k, v) for k, v in six.iteritems(args_dict) if v is not None)

    enable = args_dict.get('enable', None)

    # enable=False should to be moved to the beginning of the args list to
    # minimize transient states being generated to connected equipment.
    if enable is not None and not enable:
      del args[args.index(('enable', enable))]
      args.insert(0, ('enable', enable))

    # enable=true needs to be moved to the end of the list, again to minimize
    # transient states being generated.
    if enable is not None and enable:
      del args[args.index(('enable', enable))]
      args.append(('enable', enable))

    def should_set(chan, parm_name, expected_value):
      """Returns true if the write to the siggen should proceed."""
      if not self.read_before_write:
        return True

      if self.get(chan, parm_name) == expected_value:
        # No need to set as the value is already where it needs to be.
        # Also, delete the argument from future retries
        del args_dict[parm_name]
        return False

      return True

    # Map various parameter names to function that check arguments
    # and generate the correct low-level string.
    make_command = {
        'duty_cycle': functools.partial(_make_duty_cycle_command, channel),
        'enable': functools.partial(_make_enable_command, channel),
        'freq_hz': functools.partial(_make_freq_hz_command, channel),
        'freq_uhz': functools.partial(_make_freq_uhz_command, channel),
        'offset_volts': functools.partial(
            _make_offset_volts_command,
            channel,
            self.min_volts,
            self.max_volts),
        'phase_degrees': functools.partial(_make_phase_command, channel),
        'volts': functools.partial(
            _make_volts_command, channel, self.max_volts),
        'wave': functools.partial(
            _make_wave_command, channel),
    }

    command_list = []
    for name, value in (a for a in args if a[0] in make_command):
      if not should_set(channel, name, value):
        continue
      command = make_command[name](value)
      if command:
        command_list.append(command)

    for command in command_list:
      self.send(command)

    return len(command_list)


  def get(self, channel=None, params=None):
    """Get one or more parameters from the Signal generator.

    get() supports three styles.

    1) You can provide a string and get will get that one parameter and return
      its value. The parameter names are the same as the set() command.
    2) You can provide any iterable (list, tuple, set, dictionary) and get()
      will read the names within and returns dictionary of name/value pairs.
      This dictionary can later be used with set.  e.g.  s = fy.get()
      fy.set(**s)
    3) You can provide no parameters and get will return a dictionary of every
      parameter it knows about.

    Args:
      channel: A single channel.  If a list is provided, index 0 of the list is
        used.
      params: See above
    """

    if channel is None:
      channel = self.default_channel

    if isinstance(channel, (list, tuple)):
      channel = channel[0]

    if channel not in (0, 1):
      raise InvalidChannelError('Invalid channel: %s' % channel)

    if params is None:
      p = sorted(SET_INIT_STATE)
      del p[p.index('channel')]
    elif isinstance(params, str):
      p = (params,)
    else:
      p = params

    if 'freq_hz' in p and 'freq_uhz' in p:
      raise InvalidFrequencyError(
          'Please, provide freq_hz or freq_uhz, not both.')

    prefix = 'RF' if channel == 1 else 'RM'

    def send(code):
      """self.send shortcut."""
      return self.send(prefix + code)

    def get_waveform_id():
      """Gets the waveform id from the signal generator."""
      return int(send('W'))

    def get_offset_volts():
      """Gets offset volts, correcting for an "unsigned" bug in the fygen."""
      offset_unsigned = int(send('O'))
      if offset_unsigned > 0x80000000:
        offset_unsigned = -(0x100000000 - offset_unsigned)
      return float(offset_unsigned) / 1000

    data = {}

    # mapping of parameters to conversion functions.
    conversions = {
        'duty_cycle': lambda: float(send('D')) / 100000.0,
        'enable': lambda: bool(int(send('N'))),
        'freq_hz': lambda: int(send('F').split('.')[0]),
        'freq_uhz': lambda: int(float(send('F')) * 1000000.0),
        'offset_volts': get_offset_volts,
        'phase_degrees': lambda: float(send('P')) / 1000.0,
        'volts': lambda: float(send('A')) / 10000.0,
        'wave': get_waveform_id,
    }

    for name in p:
      if name not in conversions:
        raise UnknownParameterError('Unknown get parameter: %s' % name)
      data[name] = conversions[name]()

    if isinstance(params, str):
      return data[params]

    return data

  def get_model(self):
    """Returns the device model."""
    return self.send('UMO')

  def _recv(self, command):
    """Waits for device."""
    if not self.is_serial:
      return ''
    response = self.port.read_until(size=MAX_READ_SIZE).decode('utf8')
    if self.debug_level:
      sys.stdout.write('%s -> %s\n' % (command.strip(), response.strip()))
    return response


def _make_command(channel, suffix):
  """Creates a generic command.

  Args:
    channel: 0 or 1
    suffix: The suffix of the command.  e.g. W00 would be for sin waveform.

  Raises:
    InvalidChannelError: if any channel other than 0 or 1 is given.
  """
  if channel == 0:
    return 'WM' + suffix

  if channel == 1:
    return 'WF' + suffix

  raise InvalidChannelError(
      'Invalid channel: %s.  Only 0 or 1 is supported' % channel)

def _make_wave_command(channel, wave):
  """Creates a wave command string.

  Args:
    channel: Channel number
    wave: must == 0

  Raises:
    UnknownWaveformError: If wave != 0
  """
  if wave != 0:
    raise UnknownWaveformError('Only sin waves are supported')
  return _make_command(channel, 'W00')


def _make_freq_uhz_command(channel, freq_uhz):
  """Create a frequency command string.

  freq_hz and freq_uhz are summed for the final result.

  Args:
    channel: Channel number
    freq_uhz: Integer frequency in uhz.  None is acceptable.

  Raises:
    InvalidFrequencyError: If a negative frequency is passed.
  """
  if freq_uhz < 0:
    raise InvalidFrequencyError('Invalid freq_uhz: %d' % freq_uhz)

  return _make_command(channel, 'F%014u' % freq_uhz)

def _make_freq_hz_command(channel, freq_hz):
  return _make_freq_uhz_command(channel, freq_hz * 1000000)


def _make_volts_command(channel, max_volts, volts):
  """Creates a waveform amplitude string."""
  if volts < 0:
    raise InvalidVoltageError('volts is too low: %g < 0' % volts)

  if volts > max_volts:
    raise InvalidVoltageError('volts is too high: %g > %g' % (volts, max_volts))

  return _make_command(channel, 'A%.2f' % volts)


def _make_duty_cycle_command(channel, duty_cycle):
  """Creates a waveform duty cycle string."""
  if duty_cycle <= 0.0:
    raise InvalidDutyCycleError('duty_cycle <= 0: %g' % duty_cycle)

  if duty_cycle >= 1.0:
    raise InvalidDutyCycleError('duty_cycle >= 1: %g' % duty_cycle)

  return _make_command(channel, 'D%.1f' % (duty_cycle * 100.0))


def _make_offset_volts_command(channel, min_volts, max_volts, volts):
  """Create a voltage offset string."""
  if volts < min_volts:
    raise InvalidVoltageOffsetError(
        'offset_volts is too low: %g < %g' % (volts, min_volts))

  if volts > max_volts:
    raise InvalidVoltageOffsetError(
        'offset_volts is too high: %g > %g' % (volts, max_volts))

  return _make_command(channel, 'O%.2f' % volts)


def _make_phase_command(channel, phase_degrees):
  """Creates a phase string."""
  return _make_command(channel, 'P%.3f' % (phase_degrees % 360))


def _make_enable_command(channel, enable):
  """Enable/Disable a channel.

  Args:
    channel: 0 or 1
    enable: True/False
  """
  return _make_command(channel, 'N' + ('1' if enable else '0'))
