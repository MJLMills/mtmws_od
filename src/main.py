import machine
import random
import micropython


class RangedVariable(object):
    """A variable constrained to a specific range.

    The variable has a value that is constrained to lie within a specific
    range. It is designed to enable mappings to be constructed between
    pairs of ranged variables. The formula to compute an output value for a
    given input value requires the input value and the minimum and maximum
    values of the input and output values.

    Hardware variables are inherently ranged. For inputs, variables read
    from an ADC or digital pin will have fixed ranges. For outputs, hardware
    imposes a limit on the range of values that can be written.

    Parameters
    ----------
    value : int or float
    minimum : int or float or RangedVariable
    maximum : int or float or RangedVariable
    """

    def __init__(self,
                 value,
                 minimum,
                 maximum):

        self._minimum = minimum
        self._maximum = maximum
        self.value = value

    @property
    def value(self):
        """The current value of this ranged variable.

        Returns
        -------
        int or float
        """
        return self._value

    @value.setter
    def value(self, value) -> None:
        """Set the current value of this ranged variable.

        Parameters
        ----------
        value : int or float
        """
        # if self.minimum_value <= value <= self.maximum_value:
        self._value = value
        # else:
        #    raise ValueError(f"Value outside range: {value}, {self._minimum}, {self._maximum}")

    @property
    def minimum_value(self):
        """The minimum value of this ranged variable.


        Returns
        -------
        int or float
        """
        if isinstance(self._minimum, RangedVariable):
            return self._minimum.value
        else:
            return self._minimum

    @minimum_value.setter
    def minimum_value(self, minimum) -> None:
        """Set the minimum value of this ranged variable.

        Parameters
        ----------
        value : int or float
        """
        if isinstance(self._minimum, RangedVariable):
            self._minimum.value = minimum
        else:
            self._minimum = minimum

    @property
    def maximum_value(self):
        """The maximum value of this ranged variable.

        Returns
        -------
        int or float
        """
        if isinstance(self._maximum, RangedVariable):
            return self._maximum.value
        else:
            return self._maximum

    @maximum_value.setter
    def maximum_value(self, maximum) -> None:
        """Set the maximum value of this ranged variable.

        Parameters
        ----------
        value : int or float
        """
        if isinstance(self._maximum, RangedVariable):
            self._maximum.value = maximum
        else:
            self._maximum = maximum

    @property
    def value_range(self):
        return self.maximum_value - self.minimum_value

    def map_value(self, ranged_variable):
        """Update the value of this ranged variable.

        This method has access to this variable's ranges, and the value and
        ranges of the variable it is to be updated from.
        Using this as a slot will require re-computation of the slope each
        time the signal fires. To avoid this would need a permanent object
        with a slot to connect to the signal which can update this variable.

        Parameters
        ----------
        ranged_variable : RangedVariable
        """
        slope = self.value_range / ranged_variable.value_range
        self.value = self.minimum_value + (slope * (
                ranged_variable.value - ranged_variable.minimum_value))

    def __str__(self):
        str_rep = self.__class__.__name__ + ": "
        str_rep += f"value = {self.value}, "
        str_rep += f"minimum value = {self.minimum_value}, "
        str_rep += f"maximum value = {self.maximum_value}"

        return str_rep


class Signal(object):

    def __init__(self):
        self.slots = []

    def connect(self, *slots):
        for slot in slots:
            if slot not in self.slots:
                self.slots.append(slot)

    def disconnect(self, slot):
        if slot in self.slots:
            self.slots.pop(self.slots.index(slot))

    def emit(self, **kwargs):
        for slot in self.slots:
            slot(**kwargs)

    def __str__(self):
        str_rep = self.__class__.__name__
        return str_rep


class HardwareComponent(object):
    """An abstract class for a hardware object with an associated GPIO pin."""

    @property
    def io_pin_id(self) -> int:
        """The unique identifier of the GPIO pin used by this class."""
        raise NotImplementedError(
            self.__class__.__name__ + " does not implement io_pin_id."
        )


class AnalogOutput(HardwareComponent):
    """A hardware analog output.

    There are four analog outputs on the Computer, each of which is a socket.
    Two are dedicated to CV and are written to using PWM. The other two may be
    used for CV or audio, and are written to through the Computer's DAC.

    See Also
    --------
    CVAudioOutputSocket
        The CV/Audio output sockets of the Computer.
    CVOutputSocket
        The CV output sockets of the Computer.
    """

    # TODO - m is itself another ranged variable, if we want to map it
    # with an input ranged variable
    def __init__(self, m: int = 0):
        # TODO - this is attenuating behaviour, add attenuversion
        self.min_value = RangedVariable(
            minimum=self.hardware_max / 2,
            maximum=m,
            value=0
        )

        self.max_value = RangedVariable(
            minimum=self.hardware_max / 2,
            maximum=self.hardware_max - m,
            value=self.hardware_max - m
        )

        self.ranged_variable = RangedVariable(
            value=self.min_value.value,
            minimum=self.min_value,
            maximum=self.max_value
        )

    def map_min_value(self, ranged_variable):
        self.min_value.map_value(ranged_variable)

    def map_max_value(self, ranged_variable):
        self.max_value.map_value(ranged_variable)

    def map_range(self, ranged_variable):
        self.min_value.map_value(ranged_variable)
        self.max_value.map_value(ranged_variable)

    @property
    def hardware_min(self) -> int:
        raise NotImplementedError(
            self.__class__.__name__ + " does not implement max_value."
        )

    @property
    def hardware_max(self) -> int:
        raise NotImplementedError(
            self.__class__.__name__ + " does not implement max_value."
        )

    def write(self, value):
        raise NotImplementedError(
            self.__class__.__name__ + " does not implement write."
        )

    def map_and_write_value(self, ranged_variable):
        self.ranged_variable.map_value(ranged_variable)
        self.write(self.ranged_variable.value)


class DigitalOutput(HardwareComponent):
    """A hardware digital output.

    See Also
    --------
    PulseOutputSocket
    LED
    """

    def __init__(self):
        super().__init__()
        self._pin = machine.Pin(self.io_pin_id,
                                machine.Pin.OUT)

        self._timer = machine.Timer(-1)

    @property
    def on_value(self) -> int:
        """The value used to represent "on" for this digital output."""
        raise NotImplementedError(
            self.__class__.__name__ + " does not implement on_value."
        )

    @property
    def off_value(self) -> int:
        """The value used to represent "off" for this digital output."""
        raise NotImplementedError(
            self.__class__.__name__ + " does not implement off_value."
        )

    def turn_on(self, timer=None):
        self._pin.value(self.on_value)

    def turn_off(self, timer=None):
        self._pin.value(
            self.off_value)  # may be able to get rid if this can be used as the callback

    def is_on(self):
        return self._pin.value() == self.on_value

    def is_off(self):
        return self._pin.value() == self.off_value

    def toggle(self):
        if self._pin.value == self.ON_VALUE:
            self.turn_off()
        elif self._pin.value == self.OFF_VALUE:
            self.turn_on()

    def pulse(self):
        self.turn_on()

        self._timer.init(mode=machine.Timer.ONE_SHOT,
                         period=100,
                         callback=self.turn_off)


class PulseInputSocket(HardwareComponent):
    """
    Inverted digital input: Low input = High reading.
    For example, use a falling edge to track the start of a pulse.
    NB: Input pin must have the pull-up enabled, this powers the transistor.

    The gates are about 5-6v
    """
    __ON_VALUE = 0
    __OFF_VALUE = 1

    def __init__(self):

        self._pin = machine.Pin(self.io_pin_id,
                                machine.Pin.IN,
                                machine.Pin.PULL_UP)

        self.pulse_started = Signal()

        self.jack_inserted = Signal()
        self.jack_removed = Signal()
        self._has_jack = False

        self.irq = self._pin.irq(handler=self.__emit_pulse_started,
                                 trigger=machine.Pin.IRQ_FALLING)

    @property
    def has_jack(self) -> bool:
        return self._has_jack

    @has_jack.setter
    def has_jack(self, has_jack: bool) -> None:
        """Set whether this socket has a jack inserted."""
        had_jack = self._has_jack
        self._has_jack = has_jack

        if has_jack == had_jack:
            return
        elif has_jack and (not had_jack):
            self.jack_inserted.emit()
        elif (not has_jack) and had_jack:
            self.jack_removed.emit()

    def __emit_pulse_started(self, _):
        self.pulse_started.emit()

    def set_irq(self, handler):
        self._pin.irq(handler=handler, trigger=machine.Pin.IRQ_FALLING)

    def read(self):
        return self._pin.value()

    def read_norm_probe(self):
        return not self.read()

    def is_high(self):
        return self.read() == self.__ON_VALUE

    def is_low(self):
        return self.read() == self.__OFF_VALUE


class PulseInputSocketOne(PulseInputSocket):
    """The first (leftmost) pulse input socket."""
    __IO_PIN_ID = 2

    @property
    def io_pin_id(self):
        """The unique identifier of the GPIO pin used by this class."""
        return self.__IO_PIN_ID


class PulseInputSocketTwo(PulseInputSocket):
    """The second (rightmost) pulse input socket."""
    __IO_PIN_ID = 3

    @property
    def io_pin_id(self):
        """The unique identifier of the GPIO pin used by this class."""
        return self.__IO_PIN_ID


class AnalogInput(HardwareComponent):
    """An abstract class for an analog input hardware object.

    There are eight analog inputs on the Computer. The main, X and Y knobs,
    the Z-switch and two pairs of CV and CV/Audio inputs. Excepting the
    CV/Audio inputs, all reach the RP2040 via a multiplexer, so there are in
    total four unique micropython ADC objects attached to GPIO pins with IDs
    26, 27, 28 and 29. The GPIO pins with IDs 28 and 29 are connected to the
    multiplexer to provide values from the 6 selectable inputs. The GPIO pins
    with IDs 26 and 27 are directly connected to the CV/Audio inputs. This is
    abstracted away in the two subclasses of AnalogInput; MultiplexedInput and
    CVAudioInputSocket.

    See Also
    --------
    MultiplexedInput
        A multiplexed analog input source.
    MainKnob
        The main (big) knob on the Computer module.
    KnobX
        The knob marked X.
    KnobY
        The knob marked Y.
    SwitchZ
        The Z-switch.
    CVInputSocket
        The CV input sockets of the Computer.
    CVAudioInputSocket
        The CV/Audio input sockets of the computer.
    """

    def __init__(self):

        self.ranged_variable = RangedVariable(
            value=self.min_value,
            minimum=self.min_value,
            maximum=self.max_value
        )
        self.value_changed = Signal()

        self._has_jack = False
        self.jack_inserted = Signal()
        self.jack_removed = Signal()

    @property
    def has_jack(self) -> bool:
        return self._has_jack

    @has_jack.setter
    def has_jack(self, has_jack: bool) -> None:
        """Set whether this socket has a jack inserted."""
        had_jack = self._has_jack
        self._has_jack = has_jack

        if has_jack == had_jack:
            return
        elif self._has_jack and (not had_jack):
            self.jack_inserted.emit()
        elif (not self._has_jack) and had_jack:
            self.jack_removed.emit()

    @property
    def adc(self) -> machine.ADC:
        raise NotImplementedError(
            self.__class__.__name__ + " does not implement adc."
        )

    @property
    def min_value(self) -> int:
        raise NotImplementedError(
            self.__class__.__name__ + " does not implement min_value."
        )

    @property
    def max_value(self) -> int:
        raise NotImplementedError(
            self.__class__.__name__ + " does not implement max_value."
        )

    def read(self) -> None:
        """Read a 12-bit uint value from the RP2040's ADC.

        The micropython ADC class provides a single read method, which takes an
        analog reading and returns a 16-bit unsigned integer in the range
        0-65535. The return value represents the raw reading taken by the ADC,
        scaled such that the minimum value is 0 and the maximum value is 65535.

        The ADC on the RP2040 is 12-bit, meaning it can distinguish 4096 values
        at the pin. When reading the ADC with micropython, the 4 least
        significant bits of the u16 will not be meaningful.
        There are two alternatives for converting the 16-bit unsigned integers
        from the micropython ADC read_16 method to 12-bit unsigned integers.
        Either can scale (which uses floating point and rounding)
        u12 = int(u16 / 16)
        or shift bits 4 positions to the right, effectively discarding the four
        least significant bits.
        u12 = u16 >> 4
        Neither of these seems strictly necessary on read since the mappings take
        care of converting to the right ranges, and either way python is storing these
        as integers, 12-bit or 16-bit it doesn't care.
        """
        value = self.ranged_variable.value
        self.ranged_variable.value = self.adc.read_u16()

        if abs(self.ranged_variable.value - value) > 32:
            self.value_changed.emit(ranged_variable=self.ranged_variable)

    def read_norm_probe(self) -> bool:
        self.read()
        if self.ranged_variable.value < 28383:
            return True
        else:
            return False


class Multiplexer(object):
    """The multiplexer attached to the Computer.

    The hardware is a 4052 multiplexer with 2 (ADC pins) by 4 (digitally
    selectable analog input pins) for 8 total channels. The multiplexer has
    two digital output pins (IDs 24 and 25) that are set to specify which
    analog input will be read via the ADC on the GPIO pins (28 and 29). The
    GPIO input pins are connected to analog inputs 2 and 3 on the ADC.

    The truth table is as follows:

    A | B | ADC Channel 2 GPIO 28 | ADC Channel 3 GPIO 29
    --|---|-----------------------|----------------------
    0 | 0 | Main Knob             | CV 1
    1 | 0 | X Knob                | CV 2
    0 | 1 | Y Knob                | CV 1
    1 | 1 | Z Switch              | CV 2

    By default, the multiplexer digital output is set to (0, 0) so reads on pin
    one will return the main knob value and pin two will return the CV 1 input
    value.
    """
    __MUX_LOGIC_A_PIN_ID = 24
    """The ID of the first multiplexer output pin."""
    __MUX_LOGIC_B_PIN_ID = 25
    """The ID of the second multiplexer output pin."""
    MUX_IO_PIN_ONE_ID = 28
    """The ID of the multiplexer's first analog input pin."""
    MUX_IO_PIN_TWO_ID = 29
    """The ID of the multiplexer's second analog input pin."""

    __MUX_LOGIC_A_PIN = machine.Pin(__MUX_LOGIC_A_PIN_ID,
                                    machine.Pin.OUT)
    """The first digital output pin connected to the multiplexer."""

    __MUX_LOGIC_B_PIN = machine.Pin(__MUX_LOGIC_B_PIN_ID,
                                    machine.Pin.OUT)
    """The second digital output pin connected to the multiplexer."""

    __MUX_IO_ADC_ONE = machine.ADC(MUX_IO_PIN_ONE_ID)
    """The ADC connected to the first multiplexer analog output."""

    __MUX_IO_ADC_TWO = machine.ADC(MUX_IO_PIN_TWO_ID)
    """The ADC connected to the second multiplexer analog output."""

    def __init__(self):

        self.mux_logic_pin_a_value = False
        self.mux_logic_pin_b_value = False

    @property
    def mux_logic_pin_a_value(self) -> bool:
        """The value at the first mux logic digital output pin."""
        return self.__MUX_LOGIC_A_PIN.value()

    @mux_logic_pin_a_value.setter
    def mux_logic_pin_a_value(self, value) -> None:
        """Set the value at the first mux logic digital output pin."""
        self.__MUX_LOGIC_A_PIN.value(value)

    @property
    def mux_logic_pin_b_value(self) -> bool:
        """The value at the second mux logic digital output pin."""
        return self.__MUX_LOGIC_B_PIN.value()

    @mux_logic_pin_b_value.setter
    def mux_logic_pin_b_value(self, value) -> None:
        """Set the value at the second mux logic digital output pin."""
        self.__MUX_LOGIC_B_PIN.value(value)

    def set_logic_pin_values(self, value_a: bool, value_b: bool) -> None:
        """Set the values of the multiplexer logic pins.

        Parameters
        ----------
        value_a
            The value to which to set the first multiplexer logic pin.
        value_b
            The value to which to set the second multiplexer logic pin.
        """
        self.mux_logic_pin_a_value = value_a
        self.mux_logic_pin_b_value = value_b

    def get_adc(self, pin_id) -> machine.ADC:
        if pin_id == self.MUX_IO_PIN_ONE_ID:
            return self.__MUX_IO_ADC_ONE
        elif pin_id == self.MUX_IO_PIN_TWO_ID:
            return self.__MUX_IO_ADC_TWO
        else:
            raise ValueError(
                "Supplied pin ID not connected to multiplexer: ", pin_id
            )


class MultiplexedInput(AnalogInput):
    """A multiplexed analog input source.

    The set of analog inputs sharing the multiplexer are the main, x and y
    knobs, the Z switch and CV input sockets 1 and 2. By subclassing this class
    and defining three necessary properties (the ID for the GPIO pin and the
    values for the two multiplexer logic pins), these inputs can all share the
    same implementation of the read method.

    Methods
    -------
    read -> int
        Read the value of this input from the multiplexer.

    Properties
    ----------
    pin_id -> int
        The unique identifier of the GPIO pin used by this class.
    mux_logic_a_pin_value -> bool
        The value of the first multiplexer login pin for this input.
    mux_logic_b_pin_value -> bool
        The value of the second multiplexer login pin for this input.
    adc -> machine.ADC
        The analog-to-digital converter attached to this input.
    """

    def __init__(self):
        super().__init__()
        self.__multiplexer = Multiplexer()
        self._adc = self.__multiplexer.get_adc(self.io_pin_id)

    @property
    def adc(self):
        """The analog-to-digital converter attached to this input."""
        return self._adc

    @property
    def mux_logic_a_pin_value(self) -> bool:
        """The value of the first multiplexer login pin for this input."""
        raise NotImplementedError(
            self.__class__.__name__ + \
            " does not implement mux_logic_a_pin_value."
        )

    @property
    def mux_logic_b_pin_value(self) -> bool:
        """The value of the second multiplexer login pin for this input."""
        raise NotImplementedError(
            self.__class__.__name__ + \
            " does not implement mux_logic_b_pin_value."
        )

    def read(self, set_logic=True) -> None:
        """Set up the multiplexer before reading the value from the ADC."""
        if set_logic:
            self.__multiplexer.set_logic_pin_values(self.mux_logic_a_pin_value,
                                                    self.mux_logic_b_pin_value)

        super().read()


class MainKnob(MultiplexedInput):
    """The main (big) knob on the Computer module.

    The raw value read from the knob is a 16-bit unsigned integer
    with range from 0 to 65535 inclusive. This class maps the raw
    values into a range specified by the user, defaulting to the
    full range of the 16-bit unsigned integer.

    """
    __IO_PIN_ID = 28
    __MIN_VALUE_U16 = 224
    __MAX_VALUE_U16 = 65535
    __MUX_LOGIC_A_PIN_VALUE = False
    __MUX_LOGIC_B_PIN_VALUE = False

    def __init__(self):
        super().__init__()

    @property
    def io_pin_id(self) -> int:
        """The unique identifier of the GPIO pin used by this class."""
        return self.__IO_PIN_ID

    @property
    def min_value(self) -> int:
        return self.__MIN_VALUE_U16

    @property
    def max_value(self) -> int:
        return self.__MAX_VALUE_U16

    @property
    def mux_logic_a_pin_value(self) -> bool:
        """The value of the first multiplexer login pin for this input."""
        return self.__MUX_LOGIC_A_PIN_VALUE

    @property
    def mux_logic_b_pin_value(self) -> bool:
        """The value of the second multiplexer login pin for this input."""
        return self.__MUX_LOGIC_B_PIN_VALUE


class KnobX(MultiplexedInput):
    """The knob marked X."""
    __IO_PIN_ID = 28
    __MIN_VALUE_U16 = 192
    __MAX_VALUE_U16 = 65535
    __MUX_LOGIC_A_PIN_VALUE = True
    __MUX_LOGIC_B_PIN_VALUE = False

    def __init__(self):
        super().__init__()

    @property
    def io_pin_id(self) -> int:
        """The unique identifier of the GPIO pin used by this class."""
        return self.__IO_PIN_ID

    @property
    def min_value(self) -> int:
        return self.__MIN_VALUE_U16

    @property
    def max_value(self) -> int:
        return self.__MAX_VALUE_U16

    @property
    def mux_logic_a_pin_value(self) -> bool:
        """The value of the first multiplexer login pin for this input."""
        return self.__MUX_LOGIC_A_PIN_VALUE

    @property
    def mux_logic_b_pin_value(self) -> bool:
        """The value of the second multiplexer login pin for this input."""
        return self.__MUX_LOGIC_B_PIN_VALUE


class KnobY(MultiplexedInput):
    """The knob marked Y."""
    __IO_PIN_ID = 28
    __MIN_VALUE_U16 = 192
    __MAX_VALUE_U16 = 65535
    __MUX_LOGIC_A_PIN_VALUE = False
    __MUX_LOGIC_B_PIN_VALUE = True

    def __init__(self):
        super().__init__()

    @property
    def io_pin_id(self) -> int:
        """The unique identifier of the GPIO pin used by this class."""
        return self.__IO_PIN_ID

    @property
    def min_value(self) -> int:
        return self.__MIN_VALUE_U16

    @property
    def max_value(self) -> int:
        return self.__MAX_VALUE_U16

    @property
    def mux_logic_a_pin_value(self) -> bool:
        """The value of the first multiplexer login pin for this input."""
        return self.__MUX_LOGIC_A_PIN_VALUE

    @property
    def mux_logic_b_pin_value(self) -> bool:
        """The value of the second multiplexer login pin for this input."""
        return self.__MUX_LOGIC_B_PIN_VALUE


class SwitchZ(MultiplexedInput):
    """The Z-switch.

    The switch has three states:

    Up - latching, high value on read - always 65535
    Middle - latching, medium value on read - ranges 32311 to 32407 over 200 secs (converged after 140 secs)
    Down - momentary, low value on read - ranges 176 to 272 over 200 secs (converged after 4 secs)
    """
    __IO_PIN_ID = 28
    __MIN_VALUE_U16 = 0
    __MAX_VALUE_U16 = 65535
    __MUX_LOGIC_A_PIN_VALUE = True
    __MUX_LOGIC_B_PIN_VALUE = True

    __DOWN_MID_BOUNDARY = 16292
    __MID_UP_BOUNDARY = 48971
    __UP_MAX = 65535

    __DOWN = 0
    __MIDDLE = 1
    __UP = 2

    def __init__(self):

        super().__init__()

        self.switched_up = Signal()
        self.switched_up_to_middle = Signal()
        self.switched_down_to_middle = Signal()
        self.switched_down = Signal()

        self.__set_state()

    @property
    def io_pin_id(self) -> int:
        """The unique identifier of the GPIO pin used by this class."""
        return self.__IO_PIN_ID

    @property
    def min_value(self) -> int:
        return self.__MIN_VALUE_U16

    @property
    def max_value(self) -> int:
        return self.__MAX_VALUE_U16

    @property
    def mux_logic_a_pin_value(self) -> bool:
        """The value of the first multiplexer login pin for this input."""
        return self.__MUX_LOGIC_A_PIN_VALUE

    @property
    def mux_logic_b_pin_value(self) -> bool:
        """The value of the second multiplexer login pin for this input."""
        return self.__MUX_LOGIC_B_PIN_VALUE

    def is_up(self):
        return self.state == self.__UP

    def is_middle(self):
        return self.state == self.__MIDDLE

    def is_down(self):
        return self.state == self.__DOWN

    def __set_state(self):
        super().read()

        value = self.ranged_variable.value
        if 0 <= value < SwitchZ.__DOWN_MID_BOUNDARY:
            self.state = SwitchZ.__DOWN
        elif SwitchZ.__DOWN_MID_BOUNDARY <= value < SwitchZ.__MID_UP_BOUNDARY:
            self.state = SwitchZ.__MIDDLE
        else:  # SwitchZ.__MID_UP_BOUNDARY <= value <= SwitchZ.__UP_MAX:
            self.state = SwitchZ.__UP

    def read(self, set_logic=True) -> None:
        """Read the switch and emit appropriate signals."""
        previous_state = self.state
        self.__set_state()

        state = self.state
        if previous_state == SwitchZ.__UP and state == SwitchZ.__MIDDLE:
            self.switched_up_to_middle.emit()
            return

        if previous_state == SwitchZ.__DOWN and state == SwitchZ.__MIDDLE:
            self.switched_down_to_middle.emit()
            return

        if previous_state == SwitchZ.__MIDDLE and state == SwitchZ.__UP:
            self.switched_up.emit()
            return

        if previous_state == SwitchZ.__MIDDLE and state == SwitchZ.__DOWN:
            self.switched_down.emit()
            return


class CVInputSocket(MultiplexedInput):
    """The CV input sockets of the Computer.

    CV inputs are not inverted.
    -5V reads ~350
    0V reads ~2030
    +5V reads ~3700
    (these are from the docs and are hinting at calibration from the uint12
    values to actual voltages)
    """
    __IO_PIN_ID = 29
    __MIN_VALUE_U16 = 0
    __MAX_VALUE_U16 = 65535

    @property
    def io_pin_id(self) -> int:
        """The unique identifier of the GPIO pin used by this class."""
        return self.__IO_PIN_ID

    @property
    def min_value(self) -> int:
        return self.__MIN_VALUE_U16

    @property
    def max_value(self) -> int:
        return self.__MAX_VALUE_U16


class CVInputSocketOne(CVInputSocket):
    """The first (left-most) CV input socket of the Computer."""
    __MUX_LOGIC_A_PIN_VALUE = False
    __MUX_LOGIC_B_PIN_VALUE = False

    @property
    def mux_logic_a_pin_value(self) -> bool:
        """The value of the first multiplexer login pin for this input."""
        return self.__MUX_LOGIC_A_PIN_VALUE

    @property
    def mux_logic_b_pin_value(self) -> bool:
        """The value of the second multiplexer login pin for this input."""
        return self.__MUX_LOGIC_B_PIN_VALUE


class CVInputSocketTwo(CVInputSocket):
    """The second (right-most) CV input socket of the Computer."""
    __MUX_LOGIC_A_PIN_VALUE = True
    __MUX_LOGIC_B_PIN_VALUE = False

    @property
    def mux_logic_a_pin_value(self) -> bool:
        """The value of the first multiplexer login pin for this input."""
        return self.__MUX_LOGIC_A_PIN_VALUE

    @property
    def mux_logic_b_pin_value(self) -> bool:
        """The value of the second multiplexer login pin for this input."""
        return self.__MUX_LOGIC_B_PIN_VALUE


class CVAudioInputSocket(AnalogInput):
    """The CV/Audio input sockets of the computer.

    The CV/Audio analog inputs are bipolar (inverted) and DC-coupled.
    They require calibration for precise readings.
    The values returned by machine.ADC.read_u16 are in the range:
    +6v = 0
     0v = 32768
    -6v = 65535
    In practice the precision of the internal ADC is closer to 12-bit.

    The left input is normalled to the right input, so if only one
    socket is plugged in, both channels receive the same input.

    The input signals are scaled (presumably in hardware) and then
    go to channels 0 and 1 of an internal analog to digital converter
    (Note that the multiplexer makes use of channels 3 and 4 to read
    the CV inputs, knobs and switch). From there they appear to go to
    two assigned GPIO pins (26 and 27) on the Pi, from which they are
    directly readable as analog inputs.
    """
    __MIN_VALUE_U16 = 0
    __MAX_VALUE_U16 = 65535

    def __init__(self):
        self._adc = machine.ADC(self.io_pin_id)
        super().__init__()

    @property
    def adc(self):
        return self._adc

    @property
    def min_value(self) -> int:
        return self.__MIN_VALUE_U16

    @property
    def max_value(self) -> int:
        return self.__MAX_VALUE_U16


class CVAudioInputSocketOne(CVAudioInputSocket):
    """The left CV/Audio input socket."""
    __IO_PIN_ID = 27

    @property
    def io_pin_id(self) -> int:
        """The unique identifier of the GPIO pin used by this class."""
        return self.__IO_PIN_ID


class CVAudioInputSocketTwo(CVAudioInputSocket):
    """The right CV/Audio input socket."""
    __IO_PIN_ID = 26

    @property
    def io_pin_id(self) -> int:
        """The unique identifier of the GPIO pin used by this class."""
        return self.__IO_PIN_ID


class CVOutputSocket(
    AnalogOutput):  # both AnalogOutput classes have settable ranges to limit output when needed.
    """The CV output sockets of the Computer.

    These sockets use PWM output

    Inverted PWM output.
    Two-pole active filtered. Use 11-bit PWM at 60 kHz.

    The duty cycle is a 16-bit unsigned integer.

    65535 = -6V
    32768 = 0V
    0 = +6V

    Requires firmware calibration for precise values.
    """
    _FREQUENCY_KHZ = 60000
    __HARDWARE_MIN = 0
    __HARDWARE_MAX = 65535

    def __init__(self, duty_cycle: int = 32768):
        super().__init__()

        self.pwm = machine.PWM(self.io_pin_id,
                               freq=self._FREQUENCY_KHZ,
                               duty_u16=duty_cycle,
                               invert=True)

    @property
    def hardware_min(self) -> int:
        return self.__HARDWARE_MIN

    @property
    def hardware_max(self) -> int:
        return self.__HARDWARE_MAX

    def write(self, value: int):
        """Set the PWM duty cycle equal to the provided unsigned 16-bit int value."""
        self.pwm.duty_u16(int(value))


class CVOutputSocketOne(CVOutputSocket):
    """The first (left-most) CV output socket of the Computer."""
    __IO_PIN_ID = 23

    @property
    def io_pin_id(self) -> int:
        """The unique identifier of the GPIO pin used by this class."""
        return self.__IO_PIN_ID


class CVOutputSocketTwo(CVOutputSocket):
    """The second (right-most) CV output socket of the Computer."""
    __IO_PIN_ID = 22

    @property
    def io_pin_id(self) -> int:
        """The unique identifier of the GPIO pin used by this class."""
        return self.__IO_PIN_ID


class CVAudioOutputSocket(AnalogOutput):
    """The CV/Audio output sockets of the Computer.

    https://docs.micropython.org/en/latest/library/machine.SPI.html#machine-spi

    1) How do you write to the outputs?

    The CV/Audio outputs are bipolar (inverted) and DC-coupled.
    Outputs must go through a MCP4822 (2-input-channel) DAC to the sockets.

    "Communication with the device is accomplished via a simple serial interface using SPI protocols."

    Pins 18, 19 and 21 are specified for control of the MCP4822.
    Pin 18 is labeled DAC_SCK / SCK - clock signal from main
    Pin 19 is labeled DAC_SDI / MOSI - serial data from main, most-significant bit first
    Pin 21 is labeled DAC_CS / CS - active-low chip select signal from main to enable communication with a specific sub device.
    MISO is not included as the DAC (sub) does not output to main (the Pi)

    Looks like you setup a connection with the SPI class and write through it.
    There are two analog outputs on the DAC that are going to the sockets
    Each 16-bit word written to the DAC over SPI has a flag on byte 15 for which DAC you want to write to.
    The datasheet has the rest, but there are 12 bits for the value.
    """

    __SCK_PIN_ID = 18
    """Pin ID for clock signal from the RP2040 to the DAC."""

    __SDI_MOSI_PIN_ID = 19
    """Pin ID for serial data from RP2040 to the DAC, most-significant bit first."""

    __CS_PIN_ID = 21
    """Active-low chip select signal from RP2040 to enable communication with the DAC."""

    __BAUD_RATE_HZ = 20_000_000
    """The max SCK clock rate (in Hz) from the MCP4822 datasheet. Equal to 20 MHz"""

    __BITS = 8
    """The width in bits of each transfer."""

    __HARDWARE_MIN = 0
    __HARDWARE_MAX = 4095

    def __init__(self):
        super().__init__()
        # create a chip select on the documented SPI CS pin
        self.__chip_select_pin = machine.Pin(self.__CS_PIN_ID,
                                             mode=machine.Pin.OUT, value=1)

        self.__spi = machine.SPI(
            id=0,
            baudrate=self.__BAUD_RATE_HZ,
            polarity=0,
            phase=0,
            bits=self.__BITS,
            firstbit=machine.SPI.MSB,
            sck=self.__SCK_PIN_ID,
            mosi=self.__SDI_MOSI_PIN_ID,
        )

    @property
    def hardware_min(self) -> int:
        return self.__HARDWARE_MIN

    @property
    def hardware_max(self) -> int:
        return self.__HARDWARE_MAX

    def write(self, value: int):
        """Write the given value to the DAC.

        Parameters
        ----------
        value
            The 12-bit uint (a python int ranging 0 to 4095) to write.

        Writes to the DAC are 16-bit words.
        The value to write to the DAC is a 12-bit unsigned integer.

        The bytes object (immutable) is 16-bits where:

        15 : DAC_SELECTION_BIT in {0, 1}
        14 : IGNORED
        13 : Output gain selection bit, hard-coded to 1
        12 : Output Shutdown Control bit, hard-coded to 1
        11-0 : the data value to write to the DAC
        """

        dac_data = self.__DAC_STRING | (int(value) & 0xFFF)

        try:
            self.__chip_select_pin.value(0)
            self.__spi.write(bytes((dac_data >> 8, dac_data & 0xFF)))
        finally:
            self.__chip_select_pin.value(1)

    def __str__(self):
        return self.__class__.__name__ + ": (min = " + str(
            self.min_value) + ", max = " + str(self.max_value) + ")"


class CVAudioOutputSocketOne(CVAudioOutputSocket):
    __DAC_STRING = 0b0011000000000000


class CVAudioOutputSocketTwo(CVAudioOutputSocket):
    __DAC_STRING = 0b1011000000000000


class PulseOutputSocket(DigitalOutput):
    """An output socket of the computer, sending pulses.

    Inverted digital output: 1/true = low, 0/false=high.
    Scaled via a transistor.
    Pin should be output, no pullup.
    """
    __ON_VALUE = 0
    __OFF_VALUE = 1

    @property
    def on_value(self) -> int:
        """The value used to represent "on" for this digital output."""
        return self.__ON_VALUE

    @property
    def off_value(self) -> int:
        """The value used to represent "off" for this digital output."""
        return self.__OFF_VALUE


class PulseOutputSocketOne(PulseOutputSocket):
    """The first (leftmost) pulse input socket."""

    @property
    def io_pin_id(self) -> int:
        """The unique identifier of the GPIO pin used by this class."""
        return 8


class PulseOutputSocketTwo(PulseOutputSocket):
    """The second (rightmost) pulse input socket."""

    @property
    def io_pin_id(self) -> int:
        """The unique identifier of the GPIO pin used by this class."""
        return 9


class LED(DigitalOutput):
    """A light emitting diode on the module.

    If the pin value is set to 1 (i.e. True), the LED is illuminated.

    Methods
    -------
    turn_on
        Turn this LED on.
    turn_off
        Turn this LED off.

    Properties
    ----------
    value
        The value of this LED.
    """
    __ON_VALUE = 1
    __OFF_VALUE = 0
    __FIRST_LED_PIN_INDEX = 10

    def __init__(self, led_index):
        if led_index not in range(1, 7):
            raise ValueError("Invalid LED index: ", led_index)

        self._pin_id = self.__FIRST_LED_PIN_INDEX + (led_index - 1)
        super().__init__()

    @property
    def io_pin_id(self) -> int:
        """The unique identifier of the GPIO pin used by this class."""
        return self._pin_id

    @property
    def on_value(self) -> int:
        """The value used to represent "on" for this digital output."""
        return self.__ON_VALUE

    @property
    def off_value(self) -> int:
        """The value used to represent "off" for this digital output."""
        return self.__OFF_VALUE


class LEDMatrix(object):
    """The 3x2 LED matrix on the module.

    Abstracts the six LEDs on the panel as a 3x2 (row-major)
    matrix.
    """
    column_indices = {"LEFT": 0, "RIGHT": 1}
    row_indices = {"TOP": 0, "MIDDLE": 1, "BOTTOM": 2}

    LEDS = (
        (LED(led_index=1), LED(led_index=2)),
        (LED(led_index=3), LED(led_index=4)),
        (LED(led_index=5), LED(led_index=6))
    )

    index_to_subscripts = {
        1: (0, 0),
        2: (0, 1),
        3: (1, 0),
        4: (1, 1),
        5: (2, 0),
        6: (2, 1)
    }
    """Hard-coded conversion from running index to matrix subscripts."""

    def __init__(self, start_value=0):

        if start_value not in {0, 1}:
            start_value = 0

        for led_a, led_b in LEDMatrix.LEDS:
            led_a.value = start_value
            led_b.value = start_value

    @staticmethod
    def turn_on(index: int = None):
        if index:

            row_index = LEDMatrix.index_to_subscripts[index][0]
            column_index = LEDMatrix.index_to_subscripts[index][1]
            LEDMatrix.LEDS[row_index][column_index].turn_on()
        else:
            for led_a, led_b in LEDMatrix.LEDS:
                led_a.turn_on()
                led_b.turn_on()

    @staticmethod
    def turn_off(index: int = None):
        if index:
            row_index = LEDMatrix.index_to_subscripts[index][0]
            column_index = LEDMatrix.index_to_subscripts[index][1]
            LEDMatrix.LEDS[row_index][column_index].turn_off()
        else:
            for led_a, led_b in LEDMatrix.LEDS:
                led_a.turn_off()
                led_b.turn_off()

    @staticmethod
    def get(row_index, col_index):
        return LEDMatrix.LEDS[row_index][col_index]

    @staticmethod
    def get_by_index(index):
        row_index = LEDMatrix.index_to_subscripts[index][0]
        col_index = LEDMatrix.index_to_subscripts[index][1]

        return LEDMatrix.LEDS[row_index][col_index]

    @staticmethod
    def turn_row_on(row_index, num_leds=2):
        if row_index not in {0, 1, 2}:
            raise ValueError("Invalid LED row index: ", row_index)

        for led in LEDMatrix.LEDS[row_index][0:num_leds]:
            led.value = 1

    @staticmethod
    def turn_top_row_on(num_leds=2):
        LEDMatrix.turn_row_on(LEDMatrix.row_indices["TOP"], num_leds)

    @staticmethod
    def turn_middle_row_on(num_leds=2):
        LEDMatrix.turn_row_on(LEDMatrix.row_indices["MIDDLE"], num_leds)

    @staticmethod
    def turn_bottom_row_on(num_leds=2):
        LEDMatrix.turn_row_on(LEDMatrix.row_indices["BOTTOM"], num_leds)

    @staticmethod
    def turn_row_off(row_index, num_leds=2):
        if row_index not in {0, 1, 2}:
            raise ValueError("Invalid LED row index: ", row_index)

        for led in LEDMatrix.LEDS[row_index][0:num_leds]:
            led.value = 0

    @staticmethod
    def turn_top_row_off(num_leds=2):
        LEDMatrix.turn_row_off(LEDMatrix.row_indices["TOP"], num_leds)

    @staticmethod
    def turn_middle_row_off(num_leds=2):
        LEDMatrix.turn_row_off(LEDMatrix.row_indices["MIDDLE"], num_leds)

    @staticmethod
    def turn_bottom_row_off(num_leds=2):
        LEDMatrix.turn_row_off(LEDMatrix.row_indices["BOTTOM"], num_leds)

    @staticmethod
    def turn_column_on(col_index, num_leds=3):
        if col_index not in {0, 1}:
            raise ValueError("Invalid LED column index: ", col_index)

        for row in LEDMatrix.LEDS[0:num_leds]:
            row[col_index].value = 1

    @staticmethod
    def turn_left_column_on(num_leds=3):
        LEDMatrix.turn_column_on(LEDMatrix.column_indices["LEFT"], num_leds)

    @staticmethod
    def turn_right_column_on(num_leds=3):
        LEDMatrix.turn_column_on(LEDMatrix.column_indices["RIGHT"], num_leds)

    @staticmethod
    def turn_column_off(col_index, num_leds=3):
        if col_index not in {0, 1}:
            raise ValueError("Invalid LED column index: ", col_index)

        for row in LEDMatrix.LEDS[0:num_leds]:
            row[col_index].value = 0

    @staticmethod
    def turn_left_column_off(num_leds=3):
        LEDMatrix.turn_column_off(LEDMatrix.column_indices["LEFT"], num_leds)

    @staticmethod
    def turn_right_column_off(num_leds=3):
        LEDMatrix.turn_column_off(LEDMatrix.column_indices["RIGHT"], num_leds)


class NormalizationProbe(object):
    """The normalization probe.

    The normalization probe is a digital output connected to GPIO pin 4. When
    nothing is plugged into each input socket (pulse inputs one and two, CV
    inputs one and two, CV/Audio inputs one and two), they are connected to
    the normalization probe and will have the values written there. By writing
    a known pattern of digital values to the sockets and checking their read
    values, it is possible to determine whether each socket is connected to the
    normalization probe, and therefore whether a jack is plugged into its
    socket.
    A pseudorandom sequence of bits is generated on each instantiation. The
    length of this sequence is hard-coded; the longer the sequence the lower
    the possibility of coincidentally receiving it as a genuine input.
    """
    __IO_PIN_ID = 4
    __N_BITS = 16

    def __init__(self):
        self._pin = machine.Pin(self.__IO_PIN_ID,
                                machine.Pin.OUT)

        self.pattern = random.getrandbits(self.__N_BITS)
        self.index = 0
        self.n_bits = self.__N_BITS

    def write(self):
        """Write the next bit of the pattern to pin 4."""
        self._pin.value((self.pattern >> self.index) & 1)

        if self.index == self.__N_BITS - 1:
            self.index = 0
        else:
            self.index += 1

        return self._pin.value()


class Computer(object):
    """Music Thing Modular Workshop System Computer Module.

    This class abstracts the Computer module in order to make interacting with
    its controls as direct as possible. The package uses micropython for
    interaction with the hardware.
    The module provides the following set of controls (top-to-bottom,
    left-to-right):

    A "main knob" - a potentiometer with a large dial.
    X and Y knobs - trimmer potentiometers.
    Z switch - (ON)-OFF-ON, momentary push down, normal pull up.
    Two CV/Audio inputs
    Two CV/Audio outputs
    Two CV inputs
    Two CV outputs
    Two pulse inputs
    Two pulse outputs
    Six LEDs (arranged in a 3x2 matrix)

    Each of these is modeled with a dedicated class, minimizing re-use and
    hiding the complexity of the hardware, while providing access to the
    micropython objects for use where specific functionality is not yet
     implemented.
    """
    KNOWN_BOARD_VERSION_NAMES = {
        (False, False, False): "Proto 1.2",
        (True, False, False): "Proto 2.0, 2.0.1, Rev1"
    }
    """Known versions of the Computer board."""

    PIN_IDS = {
        "UART0_TX": 0,
        "UART0_RX": 1,
        "NORMALIZATION_PROBE": 4,
    }
    """GPIO Pin IDs not assigned to Computer classes.

    NB: GPIO pin 20 is not connected.

    UART0_TX, UART0_RX
        From unpopulated headers next to LEDs.
        There are two UARTS on the RP2040, UART0 and UART1.
        In this case, UART0 has been mapped to GPIO pins 0/1.
    NORMALIZATION_PROBE
        Connected to the switch inputs on all the inputs via a BAT45 protection
        diode. Toggle this pin to identify which input (CV/Audio, CV and pulse)
        sockets have plugs in them.
        The normalization probe high reads ~2600.
    """

    def __init__(self):

        self._board_version = None
        self._board_version_name = None
        self._eeprom = None
        self._uart0 = None

        self._main_knob = None
        self._knob_x = None
        self._knob_y = None
        self._switch_z = None

        self._cv_audio_input_socket_one = None
        self._cv_audio_input_socket_two = None
        self._cv_audio_output_socket_one = None
        self._cv_audio_output_socket_two = None

        self._cv_input_socket_one = None
        self._cv_input_socket_two = None
        self._cv_output_socket_one = None
        self._cv_output_socket_two = None

        self._pulses_input_socket_one = None
        self._pulses_input_socket_two = None
        self._pulses_output_socket_one = None
        self._pulses_output_socket_two = None

        self._led_matrix = None

        self._normalization_probe = NormalizationProbe()

        self.input_sockets = [
            self.cv_audio_input_socket_one,
            self.cv_audio_input_socket_two,
            self.cv_input_socket_one,
            self.cv_input_socket_two,
            self.pulses_input_socket_one,
            self.pulses_input_socket_two
        ]

    def update_input_sockets(self):

        # this could be more efficient going bit by bit instead of socket by socket?
        for socket in self.input_sockets:
            if socket is None:
                continue
            # else:
            #    print(f"{socket} instantiated")

            socket_connected = False
            for i in range(self._normalization_probe.n_bits):
                written_value = self._normalization_probe.write()
                read_value = socket.read_norm_probe()
                # print(written_value, read_value)

                if read_value != written_value:
                    socket_connected = True
                    break

            if socket_connected:
                # print(f"{socket} has jack")
                socket.has_jack = True
            else:
                socket.has_jack = False

    @property
    def eeprom(self):
        if self._eeprom is None:
            self._eeprom = Eeprom()

        return self._eeprom

    @property
    def uart(self):
        if self._uart0 is None:
            self._uart0 = machine.UART(
                0,
                baudrate=9600,  # check value
                tx=machine.Pin(Computer.PIN_IDS["UART0_TX"]),
                rx=machine.Pin(Computer.PIN_IDS["UART0_RX"])
            )

        return self._uart0

    @property
    def main_knob(self):
        if self._main_knob is None:
            self._main_knob = MainKnob()

        return self._main_knob

    @property
    def knob_x(self):
        if self._knob_x is None:
            self._knob_x = KnobX()

        return self._knob_x

    @property
    def knob_y(self):
        if self._knob_y is None:
            self._knob_y = KnobY()

        return self._knob_y

    @property
    def switch_z(self):
        if self._switch_z is None:
            self._switch_z = SwitchZ()

        return self._switch_z

    @property
    def cv_input_socket_one(self):
        if self._cv_input_socket_one is None:
            self._cv_input_socket_one = CVInputSocketOne()

        return self._cv_input_socket_one

    @property
    def cv_input_socket_two(self):
        if self._cv_input_socket_two is None:
            self._cv_input_socket_two = CVInputSocketTwo()

        return self._cv_input_socket_two

    @property
    def cv_output_socket_one(self):
        if self._cv_output_socket_one is None:
            self._cv_output_socket_one = CVOutputSocketOne()

        return self._cv_output_socket_one

    @property
    def cv_output_socket_two(self):
        if self._cv_output_socket_two is None:
            self._cv_output_socket_two = CVOutputSocketTwo()

        return self._cv_output_socket_two

    @property
    def cv_audio_input_socket_one(self):
        """The left CV/Audio input socket on the Computer."""
        if self._cv_audio_input_socket_one is None:
            self._cv_audio_input_socket_one = CVAudioInputSocketOne()

        return self._cv_audio_input_socket_one

    @property
    def cv_audio_input_socket_two(self):
        """The right CV/Audio input socket on the Computer."""
        if self._cv_audio_input_socket_two is None:
            self._cv_audio_input_socket_two = CVAudioInputSocketTwo()

        return self._cv_audio_input_socket_two

    @property
    def cv_audio_output_socket_one(self):
        if self._cv_audio_output_socket_one is None:
            self._cv_audio_output_socket_one = CVAudioOutputSocketOne()

        return self._cv_audio_output_socket_one

    @property
    def cv_audio_output_socket_two(self):
        if self._cv_audio_output_socket_two is None:
            self._cv_audio_output_socket_two = CVAudioOutputSocketTwo()

        return self._cv_audio_output_socket_two

    @property
    def pulses_input_socket_one(self):
        if self._pulses_input_socket_one is None:
            self._pulses_input_socket_one = PulseInputSocketOne()

        return self._pulses_input_socket_one

    @property
    def pulses_input_socket_two(self):
        if self._pulses_input_socket_two is None:
            self._pulses_input_socket_two = PulseInputSocketTwo()

        return self._pulses_input_socket_two

    @property
    def pulses_output_socket_one(self):
        if self._pulses_output_socket_one is None:
            self._pulses_output_socket_one = PulseOutputSocketOne()

        return self._pulses_output_socket_one

    @property
    def pulses_output_socket_two(self):
        if self._pulses_output_socket_two is None:
            self._pulses_output_socket_two = PulseOutputSocketTwo()

        return self._pulses_output_socket_two

    @property
    def led_matrix(self):
        if self._led_matrix is None:
            self._led_matrix = LEDMatrix()

        return self._led_matrix

    def read_analog_inputs(self):
        # may be able to speed this up by setting multiplexer pins here
        # each update of the two multiplexer pins takes ~0.15 ms and we're doing it 8 times each time this is called (should be 4 max)
        """Update the current raw values of all the analog inputs."""
        if self._main_knob is not None:
            self._main_knob.read()

        if self._cv_input_socket_one is not None:
            self._cv_input_socket_one.read()

        if self._knob_x is not None:
            self._knob_x.read()

        if self._knob_y is not None:
            self._knob_y.read()

        if self._switch_z is not None:
            self._switch_z.read()

        if self._cv_input_socket_two is not None:
            self._cv_input_socket_two.read()

        if self._cv_audio_input_socket_one is not None:
            self._cv_audio_input_socket_one.read()

        if self._cv_audio_input_socket_two is not None:
            self._cv_audio_input_socket_two.read()

    @property
    def board_version(self) -> tuple:
        """The version of the Computer board.

        Returns
        -------
        tuple of bool
            The three Boolean values identifying the board version."""
        if self._board_version is None:
            self._board_version = self.__read_board_version()

        return self._board_version

    @property
    def board_version_name(self) -> str:
        """The name of this board version."""

        try:
            return Computer.KNOWN_BOARD_VERSION_NAMES[self.board_version]
        except KeyError:
            raise ValueError(
                "Unknown board version with pin values: ", self.board_version
            )

    @staticmethod
    def __read_board_version() -> tuple:
        """Read the board version ID.

        The board version is stored in three bits that are read from GPIO
        digital input pins with IDs 5, 6 and 7. The following table shows the
        mapping between board version IDs and names provided in the Computer
        documentation:

        (False, False, False) = Proto1.2
        (True, False, False) = Proto 2.0, 2.0.1, Rev1.

        Note that this method (and any code that relies on the board version
        being Proto1.2) is untested due to lack of access to the appropriate
        physical board. Proto 2.0.1 and Rev1 are identical.

        See Also
        --------
        For information on the changes across different board types, see the
        Computer documentation.

        Returns
        -------
        tuple of bool
            The three Boolean values identifying the board version.
        """
        pin_a = machine.Pin(5,
                            machine.Pin.IN,
                            machine.Pin.PULL_UP)

        pin_b = machine.Pin(6,
                            machine.Pin.IN,
                            machine.Pin.PULL_UP)

        pin_c = machine.Pin(7,
                            machine.Pin.IN,
                            machine.Pin.PULL_UP)

        return (bool(pin_a.value()),
                bool(pin_b.value()),
                bool(pin_c.value()))


import array


class Model(object):
    """A model that produces a trajectory."""

    def __init__(self):

        self._coordinates = array.array(
            'f',
            bytearray(4 * self.n_dimensions)
        )

        self._initial_coordinates = array.array(
            'f',
            bytearray(4 * self.n_dimensions)
        )

        self._derivatives = array.array(
            'f',
            bytearray(4 * self.n_dimensions)
        )

        self._parameters = array.array(
            'f',
            bytearray(4 * self.n_parameters)
        )

    @property
    def n_dimensions(self):
        """The number of dimensions of this model."""
        raise NotImplementedError(
            self.__class__.__name__, "does not implement n_dimensions."
        )

    @property
    def n_parameters(self):
        """The number of parameters of this model."""
        raise NotImplementedError(
            self.__class__.__name__, "does not implement n_parameters."
        )

    def take_step(self) -> None:
        """Take a step along the model trajectory."""
        raise NotImplementedError(
            self.__class__.__name__, "does not implement take_step."
        )

    def set_initial_coordinates(self) -> None:
        """Set the initial coordinates to be the current coordinates."""
        for i in range(self.n_dimensions):
            self._initial_coordinates[i] = self._coordinates[i]

    def reset(self) -> None:
        """Return the trajectory to the initial coordinates."""
        for i in range(self.n_dimensions):
            self._coordinates[i] = self._initial_coordinates[i]


class LorenzSystem(Model):
    """The Lorenz system of ordinary differential equations.

    Parameters
    ----------
    x
        The current x-coordinate. Default starting value is 0.0.
    y
        The current y-coordinate. Default starting value is 1.0.
    z
        The current z-coordinate. Default starting value is 1.05.
    sigma
        Default value is 10.
    rho
        Default value is 28.
    beta
        Default value is 8/3.
    timestep
        The size of the step (in time units) to take.
        Default of 0.01 was determined by experimentation only.
    random_factor
        The maximum magnitude of the perturbation applied at reset.

    Properties
    ----------
    n_dimensions -> int
        The number of dimensions of this model.
    n_parameters -> int
        The number of parameters of this model.

    Methods
    -------
    take_step
        Take a step along the trajectory.
    set_initial_coordinates
        Set the initial coordinates to be the current coordinates.
    reset
        Return the trajectory to the initial coordinates.
    """

    def __init__(self,
                 x: float = 0.9,
                 y: float = 0.0,
                 z: float = 0.0,
                 sigma: float = 10.0,
                 rho: float = 28,
                 beta: float = 8 / 3,
                 timestep: RangedVariable = RangedVariable(maximum=0.01,
                                                           minimum=0.001,
                                                           value=0.01),
                 random_factor: RangedVariable = RangedVariable(maximum=1.0,
                                                                minimum=0.0,
                                                                value=0.0)):

        super().__init__()

        self._coordinates = [x, y, z]
        self._initial_coordinates = [x, y, z]
        self._parameters = [sigma, rho, beta]

        # values used outside this class
        self._x = RangedVariable(minimum=-24, maximum=24,
                                 value=self._coordinates[0])
        self._z = RangedVariable(minimum=0, maximum=55,
                                 value=self._coordinates[2])

        # values which can be changed outside this class
        self._timestep = timestep
        self._random_factor = random_factor

        self._previous_x = None

        self.crossed_zero = Signal()
        self.x_changed = Signal()
        self.z_changed = Signal()

    @property
    def timestep(self) -> RangedVariable:
        return self._timestep

    @property
    def random_factor(self) -> RangedVariable:
        return self._random_factor

    @property
    def x(self) -> RangedVariable:
        return self._x

    @property
    def z(self) -> RangedVariable:
        return self._z

    @property
    def n_dimensions(self) -> int:
        return 3

    @property
    def n_parameters(self) -> int:
        return 3

    def reset(self) -> None:
        """Return the trajectory to the initial coordinates."""
        super().reset()
        if self._random_factor:
            for i in range(self.n_dimensions):
                self._coordinates[
                    i] += self._random_factor.value * random.random()

    def take_step(self) -> None:
        """Determine the value of f(t_n+1, x, y, z) at the next timestep.

        This stepper uses the most basic possible scheme to integrate
        the ODEs of the system; Euler's method. This only requires a
        single evaluation of f(t, x, y, z) so is cheap, but is only accurate
        on the order of the square of the timestep.
        """
        previous_x = self._coordinates[0]

        timestep_value = self._timestep.value
        x = self._coordinates[0]
        y = self._coordinates[1]
        z = self._coordinates[2]

        self._coordinates[0] = x + (
                timestep_value * (self._parameters[0] * (y - x)))
        self._coordinates[1] = y + (
                timestep_value * (x * (self._parameters[1] - z) - y))
        self._coordinates[2] = z + (
                timestep_value * ((x * y) - (self._parameters[2] * z)))

        self._x.value = self._coordinates[0]
        self._z.value = self._coordinates[2]

        self.x_changed.emit(ranged_variable=self._x)
        self.z_changed.emit(ranged_variable=self._z)

        if (previous_x * self._coordinates[0]) <= 0:
            self.crossed_zero.emit()


class Looper(object):
    """A looper for models.

    Parameters
    ----------
    models
        A list of the models to be looped.
    looping
        Whether to being looping on instantiation.
        Default value is False.
    """

    def __init__(self,
                 models: list,
                 looping: bool = False):

        self._models = models
        self._looping = looping

        self.looping_started = Signal()
        self.looping_stopped = Signal()
        self.initial_coordinates_set = Signal()
        self.at_loop_start = Signal()

        self._counter = 0
        self._num_steps = None

        self._needs_update = True

    def is_looping(self):
        return self._looping == True

    def is_not_looping(self):
        return not self.is_looping()

    @property
    def needs_update(self) -> bool:
        return self._needs_update

    @needs_update.setter
    def needs_update(self, needs_update):
        self._needs_update = needs_update

    def set_initial_coordinates(self) -> None:
        """Set the initial coordinates of all the looped models."""
        self.reset()
        for model in self._models:
            model.set_initial_coordinates()

        self.initial_coordinates_set.emit()

    def reset(self) -> None:
        """Return the model trajectories to their initial coordinates."""
        self._counter = 0

    def start_looping(self) -> None:
        """Start looping the models.

        On loop start, the current position of the model will be used as the
        final position for the loop. To enable this behaviour, the number of
        steps taken since the start point was set is saved, and the counter is
        reset to zero. Turning on the looping indicator means that the method
        taking model steps will check if the counter hits the desired number of
        steps and restart.
        """
        if self._looping == False:
            self._looping = True
            self._num_steps = self._counter
            self.reset()
            for model in self._models:
                model.reset()

            self.looping_started.emit()

    def stop_looping(self) -> None:
        """Stop looping the models.

        On stopping the loop, the start position is retained but the final
        position is not kept, since it is set when the loop begins. The counter
        continues to count from the start point in case the looping is started
        again, so that the start point is kept.
        """
        if self._looping == True:
            self._looping = False
            self._num_steps = None
            self.looping_stopped.emit()

    # @timed_function
    def take_step(self) -> None:
        """Take a step along each model's trajectory.

        If the system is not being looped, this method just takes a step on the
        model trajectory and increments the counter. If the number of steps in
        the loop has been reached, the looper is at the end of its loop and so
        moves the system back to its initial coordinates and resets the
        counter. At the end of a loop, the "step" is to return to the model's
        initial position, so take_step is not called as this would result in a
        double step in a single cycle.
        """
        # self._needs_update = False

        if self._looping:
            if self._counter == self._num_steps:
                for model in self._models:
                    model.reset()

                self.reset()
                self.at_loop_start.emit()
                return

        for model in self._models:
            model.take_step()

        self._counter += 1

    def toggle_looping(self) -> None:
        """Toggle the looping on or off."""
        if self._looping:
            self.stop_looping()
        else:
            self.start_looping()


class TimerConnector:

    def __init__(self,
                 looper,
                 freq):
        self._timer = machine.Timer(-1,
                                    freq=freq,
                                    callback=self.callback)
        self._looper = looper

    def callback(self, _):
        micropython.schedule(self.update, 0)

    def update(self, _):
        self._looper.take_step()


def printit(**kwargs):
    print(kwargs)


MINIMUM_TIMESTEP: float = 0.0001
"""min timestep has no mathematical limit, must be arbitrarily chosen."""

MAXIMUM_TIMESTEP: float = 0.01
"""max timestep is limited by the Euler method integration accuracy."""

TIMESTEP_FACTOR: float = 5 / 11
"""sets how much slower B is than A."""

DIVERGENCE_MIN: float = 0.0
"""min knob/CV settings will turn off sensitivity to initial conditions."""

DIVERGENCE_MAX: float = 2.0
"""max divergence has no mathematical limit, must be arbitrarily chosen."""

timestep_a = RangedVariable(
    minimum=MINIMUM_TIMESTEP,
    maximum=MAXIMUM_TIMESTEP,
    value=MAXIMUM_TIMESTEP
)

timestep_b = RangedVariable(
    minimum=MINIMUM_TIMESTEP * TIMESTEP_FACTOR,
    maximum=MAXIMUM_TIMESTEP * TIMESTEP_FACTOR,
    value=MAXIMUM_TIMESTEP * TIMESTEP_FACTOR
)

divergence = RangedVariable(minimum=DIVERGENCE_MIN,
                            maximum=DIVERGENCE_MAX,
                            value=DIVERGENCE_MIN)

lorenz_attractor_a = LorenzSystem(
    timestep=timestep_a,
    random_factor=divergence
)
"""The Lorenz attractor running at the faster rate."""

lorenz_attractor_b = LorenzSystem(
    timestep=timestep_b,
    random_factor=divergence
)
"""The Lorenz attractor running at the slower rate."""

looper = Looper(models=[lorenz_attractor_a, lorenz_attractor_b])
"""The looper that controls the time-stepping of multiple models."""

computer = Computer()
computer.led_matrix.turn_off()
main_knob = computer.main_knob
knob_x = computer.knob_x
knob_y = computer.knob_y
switch_z = computer.switch_z

# lorenz A

crossed_zero_led_a = computer.led_matrix.get_by_index(1)
crossed_zero_pulse_output_socket_a = computer.pulses_output_socket_one

lorenz_attractor_a_x_cv_output_socket = computer.cv_output_socket_one
lorenz_attractor_a_z_cv_output_socket = computer.cv_audio_output_socket_one

lorenz_attractor_a.crossed_zero.connect(crossed_zero_led_a.pulse)
lorenz_attractor_a.crossed_zero.connect(
    crossed_zero_pulse_output_socket_a.pulse)
lorenz_attractor_a.x_changed.connect(
    lorenz_attractor_a_x_cv_output_socket.map_and_write_value)
lorenz_attractor_a.z_changed.connect(
    lorenz_attractor_a_z_cv_output_socket.map_and_write_value)

main_knob.value_changed.connect(lorenz_attractor_a.timestep.map_value)
knob_y.value_changed.connect(lorenz_attractor_a.random_factor.map_value)

# lorenz B

crossed_zero_led_b = computer.led_matrix.get_by_index(2)
crossed_zero_pulse_output_socket_b = computer.pulses_output_socket_two

lorenz_attractor_b_x_cv_output_socket = computer.cv_output_socket_two
lorenz_attractor_b_z_cv_output_socket = computer.cv_audio_output_socket_two

lorenz_attractor_b.crossed_zero.connect(crossed_zero_led_b.pulse)
lorenz_attractor_b.crossed_zero.connect(
    crossed_zero_pulse_output_socket_b.pulse)
lorenz_attractor_b.x_changed.connect(
    lorenz_attractor_b_x_cv_output_socket.map_and_write_value)
lorenz_attractor_b.z_changed.connect(
    lorenz_attractor_b_z_cv_output_socket.map_and_write_value)

main_knob.value_changed.connect(lorenz_attractor_b.timestep.map_value)
computer.knob_y.value_changed.connect(
    lorenz_attractor_b.random_factor.map_value)

# hardware
# Knob X acts as a VCA on all four output CVs
# CV/Audio inputs act as VCAs on A and B output CVs

cv_magnitude_a_input_socket = computer.cv_audio_input_socket_one
# cv_magnitude_a_input_socket.jack_inserted.connect(printit)

# assume disconnected
knob_x.value_changed.connect(lorenz_attractor_a_x_cv_output_socket.map_range)
knob_x.value_changed.connect(lorenz_attractor_a_z_cv_output_socket.map_range)

# if a jack is plugged into the CV_a magnitude input socket,
cv_magnitude_a_input_socket.jack_inserted.connect(
    # disconnect knob_x from setting the x,z output socket ranges
    lambda: knob_x.value_changed.disconnect(
        lorenz_attractor_a_x_cv_output_socket.map_range),
    lambda: knob_x.value_changed.disconnect(
        lorenz_attractor_a_z_cv_output_socket.map_range),
    # and connect the socket to set the x, z output ranges
    lambda: cv_magnitude_a_input_socket.value_changed.connect(
        lorenz_attractor_a_x_cv_output_socket.map_range),
    lambda: cv_magnitude_a_input_socket.value_changed.connect(
        lorenz_attractor_a_z_cv_output_socket.map_range),
)

# if a jack is removed from the CV_a magnitude input socket,
cv_magnitude_a_input_socket.jack_removed.connect(
    # connect knob_x to set the x,z output socket ranges
    lambda: knob_x.value_changed.connect(
        lorenz_attractor_a_x_cv_output_socket.map_range),
    lambda: knob_x.value_changed.connect(
        lorenz_attractor_a_z_cv_output_socket.map_range),
    # and disconnect the socket from setting the x, z output ranges
    lambda: cv_magnitude_a_input_socket.value_changed.disconnect(
        lorenz_attractor_a_x_cv_output_socket.map_range),
    lambda: cv_magnitude_a_input_socket.value_changed.disconnect(
        lorenz_attractor_a_z_cv_output_socket.map_range),
    # lambda: lorenz_attractor_a_x_cv_output_socket.map_range(knob_x.ranged_variable)
)

cv_magnitude_b_input_socket = computer.cv_audio_input_socket_two
cv_magnitude_b_input_socket.jack_inserted.connect(printit)

knob_x.value_changed.connect(lorenz_attractor_b_x_cv_output_socket.map_range)
knob_x.value_changed.connect(lorenz_attractor_b_z_cv_output_socket.map_range)

# if a jack is plugged into the CV_b magnitude input socket,
cv_magnitude_b_input_socket.jack_inserted.connect(
    # disconnect knob_x from setting the x,z output socket ranges
    lambda: knob_x.value_changed.disconnect(
        lorenz_attractor_b_x_cv_output_socket.map_range),
    lambda: knob_x.value_changed.disconnect(
        lorenz_attractor_b_z_cv_output_socket.map_range),
    # and connect the socket to set the x, z output ranges
    lambda: cv_magnitude_a_input_socket.value_changed.connect(
        lorenz_attractor_b_x_cv_output_socket.map_range),
    lambda: cv_magnitude_a_input_socket.value_changed.connect(
        lorenz_attractor_b_z_cv_output_socket.map_range),
)

# if a jack is removed from the CV_b magnitude input socket,
cv_magnitude_b_input_socket.jack_removed.connect(
    # connect knob_x to set the x,z output socket ranges
    lambda: knob_x.value_changed.connect(
        lorenz_attractor_b_x_cv_output_socket.map_range),
    lambda: knob_x.value_changed.connect(
        lorenz_attractor_b_z_cv_output_socket.map_range),
    # and disconnect the socket from setting the x, z output ranges
    lambda: cv_magnitude_a_input_socket.value_changed.disconnect(
        lorenz_attractor_b_x_cv_output_socket.map_range),
    lambda: cv_magnitude_a_input_socket.value_changed.disconnect(
        lorenz_attractor_b_z_cv_output_socket.map_range),
    # lambda: lorenz_attractor_a_x_cv_output_socket.map_range(knob_x.ranged_variable)
)

timestep_cv_input_socket = computer.cv_input_socket_one
timestep_cv_input_socket.jack_inserted.connect(printit)

divergence_cv_input_socket = computer.cv_input_socket_two
divergence_cv_input_socket.jack_inserted.connect(printit)

# looper

looper_status_led = computer.led_matrix.get_by_index(3)
set_start_led = computer.led_matrix.get_by_index(4)
loop_begin_led_one = computer.led_matrix.get_by_index(5)
loop_begin_led_two = computer.led_matrix.get_by_index(6)
switch_z = computer.switch_z
loop_toggle_pulse_socket = computer.pulses_input_socket_one
loop_set_initial_coordinates_socket = computer.pulses_input_socket_two

# connect the looper's signals to slots (all LED-controlling functions)
looper.looping_started.connect(looper_status_led.turn_on)
looper.at_loop_start.connect(loop_begin_led_one.pulse)
looper.at_loop_start.connect(loop_begin_led_two.pulse)
looper.looping_stopped.connect(looper_status_led.turn_off)
looper.initial_coordinates_set.connect(set_start_led.pulse)

# connect switch Z signals to slots
if switch_z.is_up():
    looper.start_looping()

# if switch Z is moved to the up position,
switch_z.switched_up.connect(
    # start looping the models
    looper.start_looping,
    # and disconnect the pulse input toggling looping
    lambda: loop_toggle_pulse_socket.pulse_started.disconnect(
        looper.toggle_looping)
)

# if switch Z is moved from the up to the middle position,
switch_z.switched_up_to_middle.connect(
    # stop looping the models
    looper.stop_looping,
    # and if the pulse input toggling looping has a jack in it, connect it to toggle looping
    lambda: loop_toggle_pulse_socket.has_jack and loop_toggle_pulse_socket.pulse_started.connect(
        looper.toggle_looping)
)

# if switch Z is moved to the down position, set the initial coordinates of the looper
switch_z.switched_down.connect(looper.set_initial_coordinates)

# if a jack is inserted into the loop toggle pulse input,
loop_toggle_pulse_socket.jack_inserted.connect(
    # connect the socket to toggle looping if the switch is in the middle
    lambda: switch_z.is_middle and loop_toggle_pulse_socket.pulse_started.connect(
        looper.toggle_looping)
)

# if a jack is removed from the loop toggle pulse input,
loop_toggle_pulse_socket.jack_removed.connect(
    # disconnect the socket to prevent toggling of looping
    lambda: loop_toggle_pulse_socket.pulse_started.disconnect(
        looper.toggle_looping)
)

# if a jack is inserted into the set initial coordinates pulse input,
loop_set_initial_coordinates_socket.jack_inserted.connect(
    # connect the socket to set the initial coordinates
    lambda: loop_set_initial_coordinates_socket.pulse_started.connect(
        looper.set_initial_coordinates)
)

# if a jack is removed from the loop toggle pulse input,
loop_set_initial_coordinates_socket.jack_removed.connect(
    # disconnect the socket to prevent setting the initial coordinates
    lambda: loop_set_initial_coordinates_socket.pulse_started.disconnect(
        looper.set_initial_coordinates)
)

write_output = TimerConnector(
    looper=looper,
    freq=200,  # 225
)

import time

while True:
    computer.update_input_sockets()
    computer.read_analog_inputs()
