import gpiod
from warnings import warn
import os
import sys
import time
from threading import Thread, Event

#
# | |_ ___   __| | ___
# | __/ _ \ / _` |/ _ \
# | || (_) | (_| | (_) |
#  \__\___/ \__,_|\___/

# TODO Docstrings not appearing properly when using help(GPIO)

# TODO Some weirdness with the timing of callbacks (might be due to testing hardware)


# BCM to Board mode conversion table
pin_to_gpio_rev3 = [
                    -1, -1, -1,  2, -1, 3,  -1,  4, 14, -1,     # NOQA
                    15, 17, 18, 27, -1, 22, 23, -1, 24, 10,     # NOQA
                    -1,  9, 25, 11,  8, -1,  7, -1, -1,  5,     # NOQA
                    -1,  6, 12, 13, -1, 19, 16, 26, 20, -1, 21  # NOQA
                   ]

# === User Facing Data ===

# [API] Pin numbering modes
UNKNOWN = 0
BCM     = 1
BOARD   = 2

# Output modes
LOW  = gpiod.Line.ACTIVE_LOW
HIGH = gpiod.Line.ACTIVE_HIGH

_LINE_ACTIVE_STATE_COSNT_TO_FLAG = {
    LOW: gpiod.LINE_REQ_FLAG_ACTIVE_LOW,
    HIGH: 0,  # Active High is set by the default flag
}


# Macro
def active_flag(const):
    return _LINE_ACTIVE_STATE_COSNT_TO_FLAG[const]


# We map RPi.GPIO PUD modes to libgpiod PUD constants
PUD_OFF     = gpiod.Line.BIAS_AS_IS
PUD_UP      = gpiod.Line.BIAS_PULL_UP
PUD_DOWN    = gpiod.Line.BIAS_PULL_DOWN

# We extend RPi.GPIO with the ability to explicitly disable pull up/down
# behavior
PUD_DISABLE = gpiod.Line.BIAS_DISABLE

# libgpiod uses distinct flag values for each line bias constant returned by
# the gpiod.Line.bias() method. To simplify our translation, we map the latter
# to the former with the following dictionary
_LINE_BIAS_CONST_TO_FLAG = {
    PUD_OFF: 0,  # This behavior is indicated with the defualt flag
    PUD_UP: gpiod.LINE_REQ_FLAG_BIAS_PULL_UP,
    PUD_DOWN: gpiod.LINE_REQ_FLAG_BIAS_PULL_DOWN,
    PUD_DISABLE: gpiod.LINE_REQ_FLAG_BIAS_DISABLE,
}


# Macro
def bias_flag(const):
    return _LINE_BIAS_CONST_TO_FLAG[const]


# internal line modes
_line_mode_none     = 0
_line_mode_in       = gpiod.LINE_REQ_DIR_IN
_line_mode_out      = gpiod.LINE_REQ_DIR_OUT
_line_mode_falling  = gpiod.LINE_REQ_EV_FALLING_EDGE
_line_mode_rising   = gpiod.LINE_REQ_EV_RISING_EDGE
_line_mode_both     = gpiod.LINE_REQ_EV_BOTH_EDGES
# As of yet unused and unexposed
# TODO investigate AS_IS kernel behavior
_line_mode_as_is    = gpiod.LINE_REQ_DIR_AS_IS


# [API] Request types
FALLING     = _line_mode_falling
RISING      = _line_mode_rising
BOTH        = _line_mode_both
# As of yet unused and unexposed
#AS_IS       = _line_mode_as_is

# NOTE: libgpiod also exposes enumerated direction constants seperate from the
# request constants, but the distinction is not relevant for our use case

# [API] Data direction types
IN  = _line_mode_in
OUT = _line_mode_out


# We map internal line modes to RPI.GPIO API direction constants for getdirection()
_LINE_MODE_TO_DIR_CONST = {
        _line_mode_none:    -1,
        _line_mode_in:      IN ,
        _line_mode_out:     OUT,
        _line_mode_falling: IN,
        _line_mode_rising:  IN,
        _line_mode_both:    IN,
        # This mode is not used by any functionality and so an appearance of this value
        # signals something gone wrong in the library
        _line_mode_as_is:   -662,
}


# === Internal Data ===

# Internal library state
class _State:
    mode       = UNKNOWN
    warnings   = True
    debuginfo  = False
    chip       = None
    event_ls   = []
    lines      = {}
    line_modes = {}
    threads    = {}
    callbacks  = {}
    killsigs   = {}
    deathsigs  = {}
    timestamps = {}


# === Helper Routines ===

def Dprint(*msgargs):
    """ Print debug information for development purposes"""
    if _State.debuginfo:
        print("[DEBUG]", *msgargs)


# Mess with the internal state for development or recreational purposes
def State_Access():
    return _State


# Reset internal state to default
def Reset():

    # Kill all running threads
    # Close chip object fd and release  any held lines
    cleanup()

    # Reset _State to default values
    _State.mode       = UNKNOWN
    _State.warnings   = True
    _State.debuginfo  = False
    _State.chip       = None
    _State.event_ls   = []
    _State.lines      = {}
    _State.line_modes = {}
    _State.threads    = {}
    _State.callbacks  = {}
    _State.killsigs   = {}
    _State.timestamps = {}


def is_all_ints(data):
    if not is_iterable(data):
        data = [data]
    if len(data) < 1:
        return False
    return all([isinstance(elem, int) for elem in data]) \
        if not isinstance(data, int)\
        else True


def is_all_bools(data):
    if not is_iterable(data):
        data = [data]
    if len(data) < 1:
        return False
    return all([isinstance(elem, bool) for elem in data]) \
        if not isinstance(data, bool)\
        else True


def is_iterable(data):
    try:
        iter(data)
    except TypeError:
        return False
    else:
        return True


def channel_fix_and_validate_bcm(channel):
    if channel < 0 or channel > _State.chip.num_lines() - 1:
        raise ValueError("The channel sent is invalid on a Raspberry Pi")
    else:
        return channel


def channel_fix_and_validate_board(channel):
    if channel < 1 or channel > len(pin_to_gpio_rev3) - 1:
        raise ValueError("The channel sent is invalid on a Raspberry Pi")

    # Use lookup table from RPi.GPIO
    channel = pin_to_gpio_rev3[channel]
    if channel == -1:
        raise ValueError("The channel sent is invalid on a Raspberry Pi")
    else:
        return channel


def channel_fix_and_validate(channel_raw):
    chip_init_if_needed()

    # This function is only defined over three mode settings
    # It should be invariant that mode will contain one of these three values
    # Other values of mode are undefined

    if not isinstance(channel_raw, int):
        raise ValueError("The channel sent is invalid on a Raspberry Pi")

    if _State.mode == UNKNOWN:
        raise RuntimeError("Please set pin numbering mode using GPIO.setmode(GPIO.BOARD) or GPIO.setmode(GPIO.BCM)")
    elif _State.mode == BCM:
        return channel_fix_and_validate_bcm(channel_raw)
    elif _State.mode == BOARD:
        return channel_fix_and_validate_board(channel_raw)


def channel_valid_or_die(channel):
    """
    Validate a channel/pin number
    Returns the pin number on success otherwise throws a ValueError

        channel        - an integer to be validated as a channel
    """
    channel_fix_and_validate(channel)


def validate_gpio_dev_exists():
    gpiochips = []
    for root, dirs, files in os.walk('/dev/'):
        for filename in files:
            if filename.find('gpio') > -1:
                gpiochips.append(filename)
    if not gpiochips:
        raise ValueError("No compatible chips found")


def chip_init():
    # Validate the existence of a gpio character device
    validate_gpio_dev_exists()

    # This is hardcoded for now but that may change soon (or not)
    try:
        _State.chip = gpiod.Chip("gpiochip0")
    except PermissionError:
        print("Script or interpreter must be run as root")
        sys.exit()
    Dprint("state chip has value:", _State.chip)


def chip_close():
    _State.chip.close()
    _State.chip = None


def chip_init_if_needed():
    if _State.chip is None:
        chip_init()
        Dprint("Chip object init() called")
    else:
        Dprint("NO-OP call to Chip object init()")


def chip_close_if_open():
    if _State.chip is not None:
        Dprint("Chip object close() called")
        chip_close()
    else:
        Dprint("NO-OP call to Chip object close()")


def line_set_mode(channel, mode, flags=0):
    if mode == line_get_mode(channel):
        return

    chip_init_if_needed()
    if line_get_mode(channel) != _line_mode_none or mode == _line_mode_none:

        if channel in list(_State.killsigs.keys()):
            cleanup_poll_thread(channel)

        if channel in _State.lines.keys():
            _State.lines[channel].release()
            del _State.lines[channel]

        # We don't want to affect bouncetime handling if channel is used again
        if channel in _State.timestamps.keys():
            del _State.timestamps[channel]

    if mode != _line_mode_none:
        if channel not in _State.lines.keys():
            _State.lines[channel] = _State.chip.get_line(channel)
        _State.lines[channel].request(consumer=str(_State.chip.name()) + str(channel), type=mode, flags=flags)

    _State.line_modes[channel] = mode


def line_get_mode(channel):
    if channel not in _State.line_modes.keys():
        return _line_mode_none
    else:
        return _State.line_modes[channel]


# === Interface Functions ===


def setmode(mode):
    """
    Set up numbering mode to use for channels.
        BOARD - Use Raspberry Pi board numbers
        BCM   - Use Broadcom GPIO 00..nn numbers
    """
    if _State.mode != UNKNOWN:
        raise ValueError("A different mode has already been set!")

    if mode != BCM and mode != BOARD:
        raise ValueError("An invalid mode was passed to setmode()")

    _State.mode = mode

    chip_init_if_needed()

    Dprint("mode set to", _State.mode)


def setwarnings(value):
    """Enable or disable warning messages"""
    _State.warnings = bool(value)
    Dprint("warning output set to", _State.warnings)


def setdebuginfo(value):
    """Enable or disable debug messages"""
    _State.debuginfo = bool(value)
    Dprint("debuginfo output set to", _State.debuginfo)


def setup(channel, direction, pull_up_down=PUD_OFF, initial=None):
    """
    Set up a GPIO channel or list of channels with a direction and (optional) pull/up down control
        channel        - either board pin number or BCM number depending on which mode is set.
        direction      - IN or OUT
        [pull_up_down] - PUD_OFF (default), PUD_UP or PUD_DOWN
        [initial]      - Initial value for an output channel
    """

    # Channel must contain only integral data
    if not is_all_ints(channel):
        raise ValueError("Channel must be an integer or list/tuple of integers")

    # Direction must be valid
    if direction != IN and direction != OUT:
        raise ValueError("An invalid direction was passed to setup()")

    if direction == OUT and pull_up_down != PUD_OFF:
        raise ValueError("pull_up_down parameter is not valid for outputs")

    if direction == IN and initial:
        raise ValueError("initial parameter is not valid for inputs")

    if pull_up_down not in [PUD_OFF, PUD_UP, PUD_DOWN, PUD_DISABLE]:
        raise ValueError("Invalid value for pull_up_down - should be either PUD_OFF, PUD_UP, PUD_DOWN, or PUD_DISABLE")

    # Make the channel data iterable by force
    if not is_iterable(channel):
        channel = [channel]

    # This implements BOARD mode
    for pin in channel:
        pin = channel_fix_and_validate(pin)

    request_flags = 0
    request_flags |= bias_flag(pull_up_down)

    for pin in channel:
        try:
            line_set_mode(pin, direction, request_flags)
            if initial is not None:
                _State.lines[pin].set_value(initial)
        except OSError:
            warn("This channel is already in use, continuing anyway.  Use GPIO.setwarnings(False) to disable warnings.\n \
                    Further attemps to use channel {} will fail unless setup() is run again sucessfully".format(pin))


def output(channel, value):
    """
    Output to a GPIO channel or list of channel
    channel - either board pin number or BCM number depending on which mode is set.
    value   - 0/1 or False/True or LOW/HIGH

    {compat} channel and value parameters may be lists or tuples of equal length
    """
    if not is_all_ints(channel):
        raise ValueError("Channel must be an integer or list/tuple of integers")

    if not is_iterable(channel):
        channel = [channel]

    # This implements BOARD mode
    for chan in channel:
        chan = channel_fix_and_validate(chan)

    if (not is_all_ints(value)) and (not is_all_bools(value)):
        raise ValueError("Value must be an integer/boolean or a list/tuple of integers/booleans")

    if not is_iterable(value):
        value = [value]

    if len(channel) != len(value):
        raise RuntimeError("Number of channel != number of value")

    for chan, val in zip(channel, value):
        if line_get_mode(chan) != _line_mode_out:
            warn("The GPIO channel has not been set up as an OUTPUT\n\tSkipping channel {}".format(chan))
        else:
            try:
                _State.lines[chan].set_value(bool(val))
            except PermissionError:
                warn("Unable to set value of channel {}, did you forget to run setup()? Or did setup() fail?".format(chan))


def input(channel):
    """
    Input from a GPIO channel.  Returns HIGH=1=True or LOW=0=False
    # channel - either board pin number or BCM number depending on which mode is set.
    """

    # This implements BOARD mode
    channel = channel_fix_and_validate(channel)

    if  getdirection(channel) not in [IN, OUT]:
        raise RuntimeError("You must setup() the GPIO channel first")

    return _State.lines[channel].get_value()


def getmode():
    """
    Get numbering mode used for channel numbers.
    Returns BOARD, BCM or None
    """

    return _State.mode if _State.mode else None


def getbias(channel):
    """
    Get bias mode of an active channel
    Returns PUD_OFF, PUD_DOWN, PUD_UP, or PUD disabled if the channel is
        active or just PUD_OFF if the channel is not active.
    """

    channel = channel_fix_and_validate(channel)

    if channel not in _State.lines.keys():
        return PUD_OFF
    else:
        return _State.lines[channel].bias()


def setbias(channel, bias):
    """
    Set bias of an active channel
    """

    channel = channel_fix_and_validate(channel)

    if bias not in [PUD_OFF, PUD_UP, PUD_DOWN, PUD_DISABLE]:
        raise ValueError("An invalid bias was passed to setbias()")

    current = getbias(channel)
    if bias != current:
        flags = getflags(channel)
        flags &= ~bias_flag(getbias(channel))
        flags |= bias_flag(bias)
        _State.lines[channel].set_flags(flags)


def getdirection(channel):
    """
    Get direction of an active channel
    Returns OUT if the channel is in an output mode, IN if the channel is in an input mode,
    and -1 otherwise
    """

    channel = channel_fix_and_validate(channel)
    return _LINE_MODE_TO_DIR_CONST[line_get_mode(channel)]


def setdirection(channel, direction):
    """
    Set direction of an active channel
    """

    channel = channel_fix_and_validate(channel)

    if direction != IN and direction != OUT:
        raise ValueError("An invalid direction was passed to setdirection()")

    current = getdirection(channel)
    if current != -1:
        if current == IN and direction == OUT:
            _State.lines[channel].set_direction_output()
            #line_set_mode(channel, _line_mode_out)
        elif current == OUT and direction == IN:
            _State.lines[channel].set_direction_input()


def getactive_state(channel):
    """
    Get the active_state of an active channel
    Returns HIGH or LOW if the channel is active and -1 otherwise
    """

    channel = channel_fix_and_validate(channel)

    if channel not in _State.lines.keys():
        return -1
    else:
        return _State.lines[channel].active_state()


def setactive_state(channel, active_state):
    """
    Set the active_state of an active channel
    """

    channel = channel_fix_and_validate(channel)

    if active_state not in [HIGH, LOW]:
        raise ValueError("An active state was passed to setactive_state()")

    # NOTE: it would be useful to be able to get the flags integer from libgpiod
    # I may post a patch
    current = getactive_state(channel)
    if active_state != current:
        flags = getflags(channel)
        flags &= ~active_flag(getactive_state(channel))
        flags |= active_flag(active_state)
        _State.lines[channel].set_flags(flags)


# Since libgoiod does not expose a get_flags option, we roll our own here
# by bitwise OR'ing all the flag getters that we use
_LIBGPIOD_FLAG_GETTERS = [
    getbias,
    getactive_state,
]

def getflags(channel):
    flags = 0
    for getter in _LIBGPIOD_FLAG_GETTERS:
        flags |= getter(channel)
    return flags


def wait_for_edge(channel, edge, bouncetime=None, timeout=0):
    """
    Wait for an edge.  Returns the channel number or None on timeout.
    channel      - either board pin number or BCM number depending on which mode is set.
    edge         - RISING, FALLING or BOTH
    [bouncetime] - time allowed between calls to allow for switchbounce
    [timeout]    - timeout in ms

    {compat} bouncetime units are in seconds. this is subject to change
    """

    # This implements BOARD mode
    channel = channel_fix_and_validate(channel)

    # Running this function before setup is allowed but the initial pin value is undefined
    # RPi.GPIO requires one to setup a pin as input before using it for event detection,
    # while libgpiod provides an interface that keeps the two mutually exclusive. We get around
    # this by not requiring it, though to maintain the same semantics as RPi.GPIO, we attempt
    # to release the channel's handle as a an input value, and acquire a new handle for an
    # event value.

    if edge not in [RISING, FALLING, BOTH]:
        raise ValueError("The edge must be set to RISING, FALLING or BOTH")

    if bouncetime is not None and bouncetime <= 0:
        raise ValueError("Bouncetime must be greater than 0")

    if timeout and timeout < 0:
        raise ValueError("Timeout must be greater than or equal to 0")  # error semantics differ from RPi.GPIO

    line_set_mode(channel, edge)

    if channel not in _State.lines.keys() and _State.lines[channel].is_used():
        raise RuntimeError("Channel is currently in use (Device or Resource Busy)")

    # Split up timeout into appropriate parts
    timeout_sec     = int(int(timeout) / 1000)
    timeout_nsec    = (int(timeout) % 1000) * 1000

    if _State.lines[channel].event_wait(sec=timeout_sec, nsec=timeout_nsec):
        # We only care about bouncetime if it is explicitly speficied in the call to this function or if
        # this is not the first call to wait_for_edge on the specified pin
        if bouncetime and channel in _State.timestamps.keys():
            while 1:
                if time.time() - _State.timestamps[channel] > bouncetime:
                    break       # wait for $bouncetime to elapse before continuing
        _State.timestamps[channel] = time.time()
        if channel not in _State.event_ls:
            # Ensure no double appends
            _State.event_ls.append(channel)
        event = _State.lines[channel].event_read()

        # A hack to clear the event buffer by reading a bunch of bytes from the file representing the GPIO line
        eventfd = _State.lines[channel].event_get_fd()
        os.read(eventfd, 10000)
        return event
    else:
        return None


def poll_thread(channel, edge, callback, bouncetime):

    # This implements BOARD mode
    channel = channel_fix_and_validate(channel)

    while not _State.killsigs[channel].is_set():
        #print("wub")
        if wait_for_edge(channel, edge, bouncetime, 10):
            for callback_func in _State.callbacks[channel]:
                callback_func(channel)

    # We are now dead
    _State.deathsigs[channel].set()


def add_event_detect(channel, edge, callback=None, bouncetime=None):
    """
    Enable edge detection events for a particular GPIO channel.
    channel      - either board pin number or BCM number depending on which mode is set.
    edge         - RISING, FALLING or BOTH
    [callback]   - A callback function for the event (optional)
    [bouncetime] - Switch bounce timeout in ms for callback

    {compat} we do not require that the channel be setup as an input as a prerequiste to running this function,
    however the initial value on the channel is undefined
    """

    # This implements BOARD mode
    channel = channel_fix_and_validate(channel)

    valid_edges = [RISING, FALLING, BOTH]
    if edge not in valid_edges:
        raise ValueError("The edge must be set to RISING, FALLING or BOTH")

    if callback and not callable(callback):
        raise TypeError("Parameter must be callable")

    if bouncetime and bouncetime <= 0:
        raise ValueError("Bouncetime must be greater than 0")

    # Start a thread that polls for events on the pin and create a list of event callbacks
    _State.threads[channel] = Thread(target=poll_thread, args=(channel, edge, callback, bouncetime))
    _State.callbacks[channel] = []

    if callback:
        _State.callbacks[channel].append(callback)

    # Create an event to allow us to signal that the thread should terminate
    _State.killsigs[channel] = Event()
    # Also create one to signal when it has terminated
    _State.deathsigs[channel] = Event()

    # Start the edge detection thread
    _State.threads[channel].start()


def add_event_callback(channel, callback):
    """
    Add a callback for an event already defined using add_event_detect()
    channel      - either board pin number or BCM number depending on which mode is set.
    callback     - a callback function"

    {compat} we do not require that the channel be setup as an input
    """

    # This implements BOARD mode
    channel = channel_fix_and_validate(channel)

    if channel not in _State.threads.keys():
        raise RuntimeError("Add event detection using add_event_detect first before adding a callback")

    if not callable(callback):
        raise TypeError("Parameter must be callable")

    _State.callbacks[channel].append(callback)


def remove_event_detect(channel):
    """
    Remove edge detection for a particular GPIO channel
    channel - either board pin number or BCM number depending on which mode is set.
    """

    # This implements BOARD mode
    channel = channel_fix_and_validate(channel)

    if channel in _State.threads.keys():
        cleanup_poll_thread(channel)
    else:
        raise ValueError("event detection not setup on channel {}".format(channel))


def event_detected(channel):
    """
    Returns True if an edge has occurred on a given GPIO.  You need to enable edge detection using add_event_detect() first.
    channel - either board pin number or BCM number depending on which mode is set."
    """

    # This implements BOARD mode
    channel = channel_fix_and_validate(channel)

    if channel in _State.event_ls:
        _State.event_ls.remove(channel)
        return True
    else:
        return False


def cleanup_poll_thread(channel):
    _State.killsigs[channel].set()
    _State.threads[channel].join()
    del _State.threads[channel]
    del _State.killsigs[channel]
    del _State.callbacks[channel]

    while not _State.deathsigs[channel].is_set():
        continue
    del _State.deathsigs[channel]


def cleanup():
    """
    Clean up by resetting all GPIO channels that have been used by this program to INPUT with no pullup/pulldown and no event detection
    [channel] - individual channel or list/tuple of channels to clean up.
    Default - clean every channel that has been used.

    {compat} Cleanup is mostly handled by libgpiod and the kernel, but we use this opportunity to kill any running callback poll threads
        as well as close any open file descriptors
    """

    # We must copy the keylist because the dict will change size during iteration
    masterkeys = list(_State.lines.keys())
    for channel in masterkeys:
        line_set_mode(channel, _line_mode_none)
    chip_close_if_open()


def get_gpio_number(channel):
    return channel_fix_and_validate(channel)


def gpio_function(channel):
    """
    Return the current GPIO function (IN, OUT, PWM, SERIAL, I2C, SPI)
    channel - either board pin number or BCM number depending on which mode is set.

    {compat} This is a stateless function that will return a constant value for every pin
    """

    # This implements BOARD mode
    channel = channel_fix_and_validate(channel)

    # error handling is done in the called function
    return get_gpio_number(channel)
