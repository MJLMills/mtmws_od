import machine
import time
import random

# BIG TODO - the hardware classes are ranged variables but atm are just implemented casually
# this should be made formal under test

import time


def timed_function(f, *args, **kwargs):
    myname = str(f).split(' ')[1]

    def new_func(*args, **kwargs):
        t = time.ticks_us()
        result = f(*args, **kwargs)
        delta = time.ticks_diff(time.ticks_us(), t)
        print('Function {} Time = {:6.3f}ms'.format(myname, delta / 1000))
        return result

    return new_func


class RangedVariable:  # TODO - change this to RangedVariable
    """
    Ranged variables are controllable parameters.

    They can belong to hardware inputs and hardware outputs.
    They can be software variables (to be set or read).

    The reason they need to be ranged is that other variables are
    mapped to them that have different ranges, so there's (almost)
    always a scaling involved.

    Ranged values are connected together by "mappings". These are
    responsible for taking the input ranged value and computing the
    output ranged value (using the ranges).

    The max and min values can be mutable, that is, you can change
    the range of a ranged value in some cases. Since this introduces
    complexity it should be a subclass.
    """

    def __init__(self, min_value, max_value, value):
        self._value = value
        self._min_value = min_value
        self._max_value = max_value

    @property
    def value(self):
        """The value of this ranged variable."""
        return self._value

    @value.setter
    def value(self, value):
        """Set the value of this ranged variable."""
        self._value = value

    @property
    def min_value(self):
        """The minimum possible value of this ranged variable."""
        return self._min_value

    @property
    def max_value(self):
        """The maximum possible value of this ranged variable."""
        return self._max_value

    def update_latest_value(self):
        """Update the latest value of this analog input."""
        pass

    @property
    def latest_value(self) -> int:
        """The pot value in the range 0, 4095."""
        return self._value

    def write(self, value):
        self._value = value

    def __str__(self):
        return "min value = ", self._min_value, "max value = ", str(
            self._max_value), "value = ", str(self._value)


class HardwareComponent(object):
    """An abstract class for a hardware object with an associated GPIO pin.

    All hardware objects (other than the CV/Audio output sockets) have a single
    associated GPIO pin, regardless of whether they use that pin directly or
    through the multiplexer (in which case the GPIO pin may be shared). This
    pin has a unique integer identifier.

    # i'm fairly convinced there's no way to change the ranges on inputs
    # that's because the read will always return a value in the hardware range
    # that then can be mapped elsewhere. If you e.g. set the min to 1000, the read
    # method will still carry on reading whatever is at the pin. You would have to map
    # the hardware reads to another range, which you're going to be doing anyway so
    # it's a waste of time.
    # the min/max values then are a form of calibration if they might differ for
    # each specific workshop system.

    main knob - its ranged variable is the u16 read at the multiplexer, range is fixed by hardware constraints
    x knob - same
    y knob - same
    z switch - same (but really is mapped to one of three states at any given instant)

    CV input (1,2) - same
    CV/Audio input (1,2) - u16 read straight from pin

    # these you may want to control the range of, because to write a limited range you
    # literally have to change the range of the hardware output because the value is computed
    # by mapping whatever input to the output based on that range.
    CV output (1,2) - PWM output
    CV/Audio output (1,2) - SPI/DAC output

    """

    @property
    def pin_id(self) -> int:
        """The unique identifier of the GPIO pin used by this class."""
        raise NotImplementedError(
            self.__class__.__name__ + " does not implement pin_id."
        )


class AnalogInput(HardwareComponent):
    """An abstract class for an analog input hardware object.

    There are eight analog inputs on the Computer. The main, X and Y knobs,
    the Z-switch and two pairs of CV and CV/Audio inputs. Excepting the
    CV/Audio inputs, all reach the Computer via a multiplexer, so there are in
    total four unique micropython ADC objects attached to GPIO pins with IDs
    26, 27, 28 and 29. The GPIO pins 28 and 29 are connected to the multiplexer
    to provide values from the 6 selectable inputs. The GPIO pins 26 and 27
    are directly connected to the CV/Audio inputs.
    """

    def __init__(
            self):  # input_value property should be a ranged variable instance - this may be true of every IO object?
        self._latest_value = None

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

    def read(self) -> int:
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
        return self.adc.read_u16()

    def update_latest_value(self):
        """Update the latest value of this analog input."""
        self._latest_value = self.read()

    @property
    def latest_value(self) -> int:
        """The pot value in the range 0, 4095."""
        return self._latest_value


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

    # @timed_function
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
        self._adc = self.__multiplexer.get_adc(self.pin_id)

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

    def read(self, set_logic=True) -> int:
        """Set up the multiplexer before reading the value from the ADC."""
        if set_logic:
            self.__multiplexer.set_logic_pin_values(self.mux_logic_a_pin_value,
                                                    self.mux_logic_b_pin_value)

        return super().read()


class MainKnob(MultiplexedInput):
    """The main (big) knob on the Computer module.

    The raw value read from the knob is a 16-bit unsigned integer
    with range from 0 to 65535 inclusive. This class maps the raw
    values into a range specified by the user, defaulting to the
    full range of the 16-bit unsigned integer.
    """

    def __init__(self):
        super().__init__()

    @property
    def min_value(self) -> int:
        return 224

    @property
    def max_value(self) -> int:
        return 65535

    @property
    def pin_id(self) -> int:
        """The unique identifier of the GPIO pin used by this class."""
        return 28

    @property
    def mux_logic_a_pin_value(self) -> bool:
        """The value of the first multiplexer login pin for this input."""
        return False

    @property
    def mux_logic_b_pin_value(self) -> bool:
        """The value of the second multiplexer login pin for this input."""
        return False


class KnobX(MultiplexedInput):
    """The knob marked X."""

    def __init__(self):
        super().__init__()

    @property
    def min_value(self) -> int:
        return 224

    @property
    def max_value(self) -> int:
        return 65535

    @property
    def pin_id(self) -> int:
        """The unique identifier of the GPIO pin used by this class."""
        return 28

    @property
    def mux_logic_a_pin_value(self) -> bool:
        """The value of the first multiplexer login pin for this input."""
        return True

    @property
    def mux_logic_b_pin_value(self) -> bool:
        """The value of the second multiplexer login pin for this input."""
        return False


class KnobY(MultiplexedInput):
    """The knob marked Y."""

    def __init__(self):
        super().__init__()

    @property
    def min_value(self) -> int:
        return 224

    @property
    def max_value(self) -> int:
        return 65535

    @property
    def pin_id(self) -> int:
        """The unique identifier of the GPIO pin used by this class."""
        return 28

    @property
    def mux_logic_a_pin_value(self) -> bool:
        """The value of the first multiplexer login pin for this input."""
        return False

    @property
    def mux_logic_b_pin_value(self) -> bool:
        """The value of the second multiplexer login pin for this input."""
        return True


class AnalogOutput(
    HardwareComponent):  # TODO - right now this _is_ a RangedVariable - it implements the API of that class
    """"""

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
    FREQUENCY_KHZ = 60000

    def __init__(self, duty_cycle: int = 32768):
        super().__init__()

        self.pwm = machine.PWM(self.pin_id,
                               freq=60000,
                               duty_u16=duty_cycle,
                               invert=True)

        self._ranged_min_value = RangedVariable(
            min_value=self.hardware_max / 2,
            max_value=0,
            value=0)

        self._ranged_max_value = RangedVariable(
            min_value=self.hardware_max / 2,
            max_value=self.hardware_max,
            value=self.hardware_max)

    @property
    def hardware_min(self) -> int:
        return 0

    @property
    def hardware_max(self) -> int:
        return 65535

    @property
    def min_value(self) -> int:
        """The minimum value of the analog output."""
        return self._ranged_min_value.value

    @min_value.setter
    def min_value(self, min_value: int) -> None:
        """Set the minimum value of the analog output."""
        self._ranged_min_value.value = int(min_value)

    @property
    def max_value(self) -> int:
        """The maximum value of the analog output."""
        return self._ranged_max_value.value

    @max_value.setter
    def max_value(self, max_value: int) -> None:
        """Set the maximum value of the analog output."""
        self._ranged_max_value.value = int(max_value)

    def write(self, value: int):
        """Set the PWM duty cycle equal to the provided unsigned 16-bit int value."""
        self.pwm.duty_u16(int(value))


class CVOutputSocketOne(CVOutputSocket):
    """The first (left-most) CV output socket of the Computer."""

    @property
    def pin_id(self) -> int:
        """The unique identifier of the GPIO pin used by this class."""
        return 23


class CVOutputSocketTwo(CVOutputSocket):
    """The second (right-most) CV output socket of the Computer."""

    @property
    def pin_id(self) -> int:
        """The unique identifier of the GPIO pin used by this class."""
        return 22


class CVAudioOutputSocket(AnalogOutput):
    """A CV/Audio output socket.

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

        self._ranged_min_value = RangedVariable(
            min_value=self.hardware_max / 2,
            max_value=0,
            value=0)

        self._ranged_max_value = RangedVariable(
            min_value=self.hardware_max / 2,
            max_value=self.hardware_max,
            value=self.hardware_max)

    @property
    def hardware_min(self) -> int:
        return 0

    @property
    def hardware_max(self) -> int:
        return 4095

    @property
    def min_value(self) -> int:
        """The minimum value of the analog output."""
        return self._ranged_min_value.value

    @min_value.setter
    def min_value(self, min_value: int) -> None:
        """Set the minimum value of the analog output."""
        self._ranged_min_value.value = int(min_value)

    @property
    def max_value(self) -> int:
        """The maximum value of the analog output."""
        return self._ranged_max_value.value

    @max_value.setter
    def max_value(self, max_value: int) -> None:
        """Set the maximum value of the analog output."""
        self._ranged_max_value.value = int(max_value)

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

        # value = int((value / 65535) * 4095)

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
                 timestep: RangedVariable = RangedVariable(max_value=0.01,
                                                           min_value=0.001,
                                                           value=0.01),
                 random_factor: RangedVariable = RangedVariable(max_value=1.0,
                                                                min_value=0.0,
                                                                value=0.0)):

        super().__init__()

        self._coordinates = [x, y, z]
        self._initial_coordinates = [x, y, z]
        self._parameters = [sigma, rho, beta]

        # values used outside this class
        self._x = RangedVariable(min_value=-24, max_value=24,
                                 value=self._coordinates[0])
        self._z = RangedVariable(min_value=0, max_value=55,
                                 value=self._coordinates[2])

        # values which can be changed outside this class
        self._timestep = timestep
        self._random_factor = random_factor

        self._crossed_zero = False
        self._previous_x = None

    def crossed_zero(self):
        return self._crossed_zero

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

        self._crossed_zero = False
        if (previous_x * self._coordinates[0]) <= 0:
            self._crossed_zero = True


class Mapping(
    object):  # stop this from assuming float is OK. some mappings can be done by int precision drop (16-12 e.g. is >>4) or fixed point?
    """A connection from an input ranged variable to an output ranged variable."""

    def __init__(self,
                 source: RangedVariable,
                 output: RangedVariable,
                 ranges_frozen: bool = True):

        self._source = source
        self._output = output
        self._ranges_frozen = ranges_frozen

        self._source_min_value = self._source.min_value
        self._source_max_value = self._source.max_value
        self._output_min_value = self._output.min_value
        self._output_max_value = self._output.max_value

        output_range = self._output_max_value - self._output_min_value
        input_range = self._source_max_value - self._source_min_value

        if input_range == output_range:
            self._slope = 1
        else:
            self._slope = (self._output_max_value - self._output_min_value) / (
                        self._source_max_value - self._source_min_value)

        self.update_latest_value(update_source_value=True)

    def compute_source_value(self, update_source_value=False):
        if update_source_value:
            self._source.update_latest_value()

        return self._source.latest_value

    def update_latest_value(self, write_output=False,
                            update_source_value=False) -> None:
        """Update the output property by scaling the input property."""

        source_value = self.compute_source_value(
            update_source_value=update_source_value)

        if self._ranges_frozen:
            slope = self._slope
        else:
            self._source_min_value = self._source.min_value
            self._output_min_value = self._output.min_value
            slope = (self._output.max_value - self._output_min_value) / (
                        self._source.max_value - self._source_min_value)

        self._latest_output_value = self._output_min_value + (
                    slope * (source_value - self._source_min_value))

        if write_output:
            self.write()

    def write(
            self):  # this is only correct if the output is a software variable
        """Copy the latest output value to the output object."""
        self._output.write(self._latest_output_value)

    def latest_value(self):
        return self._latest_output_value

    def update_latest_value_and_write(self) -> None:
        self.update_latest_value()
        self.write()

    def __str__(self):
        print("input = ", self._source,
              "output = ", self._output,
              "slope = ", self._slope)


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


class PulseInputSocket(HardwareComponent):
    """
    Inverted digital input: Low input = High reading.
    For example, use a falling edge to track the start of a pulse.
    NB: Input pin must have the pull-up enabled, this powers the transistor.

    Simple digital on/off signals, buffered and scaled with transistors.
    Use them for clocks, pulses, gates. They could also produce unfiltered PWM signals, so could maybe be used for gnarly audio (loud!) or gritty CV.
    They’ll often be used to trigger the envelopes, which are Serge-style voltage controlled slopes
    The gates are about 5-6v
    """

    def __init__(self):
        self._pin = machine.Pin(self.pin_id,
                                machine.Pin.IN,
                                machine.Pin.PULL_UP)

    @property
    def pin(self):
        return self._pin

    def set_irq(self, handler):
        self._pin.irq(handler=handler, trigger=machine.Pin.IRQ_FALLING)

    def read(self):
        return self._pin.value()

    def is_high(self):
        return self.read() == 0

    def is_low(self):
        return self.read() == 1


class PulseInputSocketOne(PulseInputSocket):
    """The first (leftmost) pulse input socket."""

    @property
    def pin_id(self):
        """The unique identifier of the GPIO pin used by this class."""
        return 2


class PulseInputSocketTwo(PulseInputSocket):
    """The second (rightmost) pulse input socket."""

    @property
    def pin_id(self):
        """The unique identifier of the GPIO pin used by this class."""
        return 3


class LED(HardwareComponent):
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

    FIRST_LED_PIN_INDEX = 10

    def __init__(self, led_index):
        if led_index not in range(1, 7):
            raise ValueError("Invalid LED index: ", led_index)

        self._pin_id = self.FIRST_LED_PIN_INDEX + (led_index - 1)
        self._pin = machine.Pin(self.pin_id,
                                machine.Pin.OUT)

    @property
    def pin(self):
        return self._pin

    @property
    def pin_id(self) -> int:
        """The unique identifier of the GPIO pin used by this class."""
        return self._pin_id

    def turn_on(self):
        self.pin.value(1)

    def turn_off(self):
        self.pin.value(0)

    def toggle(self):
        if self.value == 1:
            self.turn_off()
        elif self.value == 0:
            self.turn_on()

    @property
    def value(self):
        return self.pin.value()

    @value.setter
    def value(self, value):
        self.pin.value(value)

    def set_value(self, value):
        self.pin.value(value)

    def pulse(self, time_s):
        self.toggle()
        time.sleep(time_s)
        self.toggle()


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


class SwitchZ(MultiplexedInput):
    """The Z-switch.

    The switch has three states:

    Up - latching, high value on read - always 65535
    Middle - latching, medium value on read - ranges 32311 to 32407 over 200 secs (converged after 140 secs)
    Down - momentary, low value on read - ranges 176 to 272 over 200 secs (converged after 4 secs)
    """
    # TODO - update these to 12-bit values
    DOWN_MID_BOUNDARY = 16292  # >> 4
    MID_UP_BOUNDARY = 48971  # >> 4
    UP_MAX = 65535  # >> 4

    @property
    def pin_id(self) -> int:
        """The unique identifier of the GPIO pin used by this class."""
        return 28

    @property
    def mux_logic_a_pin_value(self) -> bool:
        """The value of the first multiplexer login pin for this input."""
        return True

    @property
    def mux_logic_b_pin_value(self) -> bool:
        """The value of the second multiplexer login pin for this input."""
        return True

    @property
    def min_value(self) -> int:
        return 0

    @property
    def max_value(self) -> int:
        return 65535

    def is_down(self) -> bool:
        """Determine whether the switch is down."""
        return 0 <= self.latest_value < SwitchZ.DOWN_MID_BOUNDARY

    def is_middle(self) -> bool:
        """Determine whether the switch is in the middle."""
        return SwitchZ.DOWN_MID_BOUNDARY <= self.latest_value < SwitchZ.MID_UP_BOUNDARY

    def is_up(self) -> bool:
        """Determine whether the switch is up."""
        return SwitchZ.MID_UP_BOUNDARY <= self.latest_value <= SwitchZ.UP_MAX


class PulseOutputSocket(HardwareComponent):
    """An output socket of the computer, sending pulses.

    Inverted digital output: 1/true = low, 0/false=high.
    Scaled via a transistor.
    Pin should be output, no pullup.
    """

    def __init__(self):
        self._pin = machine.Pin(self.pin_id,
                                machine.Pin.OUT)

    @property
    def pin(self):
        return self._pin

    def turn_on(self):
        self.pin.value(0)

    def turn_off(self):
        self.pin.value(1)

    def is_on(self):
        return self.pin.value() == 0

    def is_off(self):
        return not self.is_on()

    def pulse(self, duration):
        self.turn_on()
        time.sleep(duration)
        self.turn_off()

    def set_value(self, value):
        self.pin.value(value)


class PulseOutputSocketOne(PulseOutputSocket):
    """The first (leftmost) pulse input socket."""

    @property
    def pin_id(self) -> int:
        """The unique identifier of the GPIO pin used by this class."""
        return 8


class CVInputSocket(MultiplexedInput):
    """The CV input sockets of the Computer.

    CV inputs are not inverted. TODO - check these on a multimeter
    -5V reads ~350
    0V reads ~2030
    +5V reads ~3700
    (these are from the docs and are hinting at calibration from the uint12
    values to actual voltages)
    """

    @property
    def pin_id(self):
        """The unique identifier of the GPIO pin used by this class."""
        return 29  # try not to redefine this here as a literal, get from multiplexer by name?

    @property
    def min_value(self) -> int:
        return 0

    @property
    def max_value(self) -> int:
        return 65535


class CVInputSocketOne(CVInputSocket):
    """The first (left-most) CV input socket of the Computer."""

    @property
    def mux_logic_a_pin_value(self) -> bool:
        """The value of the first multiplexer login pin for this input."""
        return False

    @property
    def mux_logic_b_pin_value(self) -> bool:
        """The value of the second multiplexer login pin for this input."""
        return False


class CVInputSocketTwo(CVInputSocket):
    """The second (right-most) CV input socket of the Computer."""

    @property
    def mux_logic_a_pin_value(self) -> bool:
        """The value of the first multiplexer login pin for this input."""
        return False

    @property
    def mux_logic_b_pin_value(self):
        """The value of the second multiplexer login pin for this input."""
        return True


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

    def __init__(self):
        self._adc = machine.ADC(self.pin_id)
        super().__init__()

    @property
    def adc(self):
        return self._adc

    @property
    def min_value(self) -> int:
        return 0

    @property
    def max_value(self) -> int:
        return 65535


class CVAudioInputSocketOne(CVAudioInputSocket):
    """The left CV/Audio input socket."""

    @property
    def pin_id(self) -> int:
        """The unique identifier of the GPIO pin used by this class."""
        return 27


class CVAudioInputSocketTwo(CVAudioInputSocket):
    """The right CV/Audio input socket."""

    @property
    def pin_id(self) -> int:
        """The unique identifier of the GPIO pin used by this class."""
        return 26


class PulseOutputSocketTwo(PulseOutputSocket):
    """The second (rightmost) pulse input socket."""

    @property
    def pin_id(self) -> int:
        """The unique identifier of the GPIO pin used by this class."""
        return 9


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

        return self._cv_input_socket_one

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

    # @timed_function
    def update_analog_inputs(
            self):  # may be able to speed this up by setting multiplexer pins here
        # each update of the two multiplexer pins takes ~0.15 ms and we're doing it 8 times each time this is called (should be 4 max)
        """Update the current raw values of all of the analog inputs."""
        if self._main_knob is not None:
            self._main_knob.update_latest_value()

        if self._cv_input_socket_one is not None:
            self._cv_input_socket_one.update_latest_value()

        if self._knob_x is not None:
            self._knob_x.update_latest_value()

        if self._knob_y is not None:
            self._knob_y.update_latest_value()

        if self._switch_z is not None:
            self._switch_z.update_latest_value()

        if self._cv_input_socket_two is not None:
            self._cv_input_socket_two.update_latest_value()

        if self._cv_audio_input_socket_one is not None:
            self._cv_audio_input_socket_one.update_latest_value()

        if self._cv_audio_input_socket_two is not None:
            self._cv_audio_input_socket_two.update_latest_value()

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


# setup the output timer
class TimerConnector:

    def __init__(self,
                 mappings,
                 looper,
                 freq,
                 function_mappings=None):

        self._timer = machine.Timer(-1, freq=freq, callback=self.callback)
        self._mappings = mappings
        self._function_mappings = function_mappings
        self._looper = looper

    # @timed_function
    def update_mappings(self):
        for mapping in self._mappings:
            mapping.update_latest_value(write_output=True)

        for function_mapping in self._function_mappings:
            function_mapping.update()

    def callback(self, timer):
        raise NotImplementedError(self.__class__.__name__ + "")


import micropython


class WriteOutput(TimerConnector):
    """
    This class holds a timer that schedules an update of the output values and
    computes the next model values ready for next time the callback is fired.
    # this is the only time it's worth updating the Lorenz->CV out
    # because it's the only time the x, z values have changed (because take_step was called)
    """

    # @timed_function
    def update(self, _):
        self.update_mappings()
        looper.take_step()

    # @timed_function
    def callback(self, _):
        micropython.schedule(self.update, 0)


class IRQConnector(object):
    """Connect an IRQ source to an output.

    For pins on which an IRQ can be set, this class can be used to add a
    callback that has access to the necessary program state without defining
    global variables.

    Parameters
    ----------
    irq_source
        The pin on which the IRQ is to be set.
    outputs
        The class instances providing access to program state.
    """

    def __init__(self,
                 irq_source: machine.Pin,
                 outputs: list,
                 trigger):
        self._outputs = outputs
        irq_source.pin.irq(handler=self.callback,
                           trigger=trigger)

    def callback(self, irq_source_pin):
        raise NotImplementedError(
            "Connector subclasses must define a callback method.")


class ToggleLooping(IRQConnector):
    """Connect a pulse input to toggle looping of the system.

    This class sets an IRQ on a provided digital input pin which toggles the
    provided looper on or off (depending on its current state) when the pin
    goes high.

    Parameters
    ----------
    pulse_input
        The pulse input that will toggle looping.
    looper
        The looper whose state will be toggled by the pulse input.
    """

    def __init__(self,
                 pulse_input: machine.Pin,
                 looper: Looper):
        super().__init__(pulse_input,
                         outputs=[looper],
                         trigger=machine.Pin.IRQ_RISING)

        self._looper = self._outputs[0]

    def callback(self, _):
        """Toggle looping when the pulse input goes high."""
        self._looper.toggle_looping()


# if the start point is pinged, tell the looper (set_initial_coordinates)
class SetStartPoint(IRQConnector):
    """Connect a pulse input one to set the start point of a looper.

    This class sets an IRQ on a provided digital input pin which sets the start
    point of the provided looper when the pin goes high.

    Parameters
    ----------
    pulse_input
        The pulse input that will set the start point.
    looper
        The looper whose start point will be set by the pulse input.
    """

    def __init__(self,
                 pulse_input: machine.Pin,
                 looper: Looper,
                 led: LED):
        super().__init__(pulse_input,
                         outputs=[looper, led],
                         trigger=machine.Pin.IRQ_RISING)

        self._looper = self._outputs[0]
        self._led = self._outputs[1]

    def callback(self, irq_source_pin):
        """Set the start point when the pin goes high."""
        self._looper.set_initial_coordinates()
        self._led.pulse(0.001)


class FunctionMapping(object):  # TODO - one signal can fire multiple slots
    """A mapping between parameter-less functions."""

    def __init__(self, signal, slot):
        self._signal = signal
        self._slot = slot
        self._is_connected = True

    def update(self):
        if self._is_connected & self._signal():
            self._slot()

    def disconnect(self):
        self._is_connected = False

    def connect(self):
        self._is_connected = True


class BoolBoolMapping(object):

    def __init__(self, signal, slot):
        self._signal = signal
        self._slot = slot

    def update(self):
        self._slot(self._signal)


computer = Computer()

led_matrix = computer.led_matrix
led_matrix.turn_on()

timestep_min: float = 0.0001
# min timestep has no mathematical limit, must be arbitrarily chosen.
timestep_max: float = 0.01
# max timestep is limited by the Euler method integration accuracy.
timestep_factor: float = 11 / 5
# sets how much slower B is than A
timestep_a = RangedVariable(min_value=timestep_min,
                            max_value=timestep_max,
                            value=timestep_max)

timestep_b = RangedVariable(max_value=timestep_max / timestep_factor,
                            min_value=timestep_min / timestep_factor,
                            value=timestep_max / timestep_factor)

divergence_min: float = 0.0
# min knob/CV settings will turn off sensitivity to initial conditions.
divergence_max: float = 5.0
# max divergence has no mathematical limit, must be arbitrarily chosen.
divergence = RangedVariable(min_value=divergence_min,
                            max_value=divergence_max,
                            value=divergence_min)

# TODO - here we have one knob mapping to many software variables, could this be done in a single class (one in to many out?)
# try connecting the input socket to these in place of the X knob and see if you can get VCA behaviour from it without timing
cv_output_range_mappings = [
    Mapping(source=computer.knob_x,
            output=computer.cv_audio_output_socket_one._ranged_min_value),
    # int to int
    Mapping(source=computer.knob_x,
            output=computer.cv_audio_output_socket_one._ranged_max_value),
    # int to int

    Mapping(source=computer.knob_x,
            output=computer.cv_audio_output_socket_two._ranged_min_value),
    # int to int
    Mapping(source=computer.knob_x,
            output=computer.cv_audio_output_socket_two._ranged_max_value),
    # int to int

    Mapping(source=computer.knob_x,
            output=computer.cv_output_socket_one._ranged_min_value),
    # int to int
    Mapping(source=computer.knob_x,
            output=computer.cv_output_socket_one._ranged_max_value),
    # int to int

    Mapping(source=computer.knob_x,
            output=computer.cv_output_socket_two._ranged_min_value),
    # int to int
    Mapping(source=computer.knob_x,
            output=computer.cv_output_socket_two._ranged_max_value),
    # int to int
]

lorenz_attractor_a = LorenzSystem(
    timestep=timestep_a,
    random_factor=divergence
)

lorenz_attractor_b = LorenzSystem(
    timestep=timestep_b,
    random_factor=divergence
)

looper = Looper(models=[lorenz_attractor_a, lorenz_attractor_b])
"""The looper that controls the time-stepping of multiple models."""

# instantiate the CV inputs
# timestep_cv_in = computer.cv_audio_input_socket_one
# divergence_cv_in = computer.cv_audio_input_socket_two
# a_magnitude_cv_in = computer.cv_input_socket_one  # try hooking these up
# b_magnitude_cv_in = computer.cv_input_socket_one


hardware_input_mappings = [
    Mapping(source=computer.main_knob, output=lorenz_attractor_a.timestep),
    # int to float
    Mapping(source=computer.main_knob, output=lorenz_attractor_b.timestep),
    # int to float
    #    Mapping(source=computer.cv_audio_input_socket_one, output=lorenz_attractor_a.timestep),
    #    Mapping(source=computer.cv_audio_input_socket_one, output=lorenz_attractor_b.timestep),
    Mapping(source=computer.knob_y, output=lorenz_attractor_a.random_factor),
    # int to float
    Mapping(source=computer.knob_y, output=lorenz_attractor_b.random_factor)
    # int to float
]
"""Mappings from hardware inputs to software outputs."""

switch = computer.switch_z
looper_state_led = computer.led_matrix.get_by_index(3)
looper_set_startpoint_led = computer.led_matrix.get_by_index(4)

# the switch being up should disconnect the loop start/stop IRQ on the pulse inputs - it just gets turned back on every loop anyway
# the switch being middle should reconnect the IRQs on the pulse inputs
switch_down_to_set_startpoint = FunctionMapping(switch.is_down,
                                                looper.set_initial_coordinates)
switch_up_to_start_looping = FunctionMapping(switch.is_up,
                                             looper.start_looping)
switch_middle_to_stop_looping = FunctionMapping(switch.is_middle,
                                                looper.stop_looping)
switch_up_to_middle_connected = FunctionMapping(switch.is_up,
                                                switch_middle_to_stop_looping.connect)
switch_middle_to_middle_disconnected = FunctionMapping(switch.is_middle,
                                                       switch_middle_to_stop_looping.disconnect)

looper_to_led_on = FunctionMapping(looper.is_looping, looper_state_led.turn_on)
looper_to_led_off = FunctionMapping(looper.is_not_looping,
                                    looper_state_led.turn_off)

function_mappings = [switch_down_to_set_startpoint,
                     switch_up_to_start_looping,
                     switch_middle_to_stop_looping,
                     switch_up_to_middle_connected,
                     switch_middle_to_middle_disconnected,
                     looper_to_led_on,
                     looper_to_led_off]

# can we pass mappings to the IRQConnector?
toggle_looping = ToggleLooping(
    pulse_input=computer.pulses_input_socket_one,
    looper=looper,
)

set_start_point = SetStartPoint(
    pulse_input=computer.pulses_input_socket_two,
    looper=looper,
    led=looper_set_startpoint_led
)

pulse_led_a = computer.led_matrix.get_by_index(1)
pulse_led_b = computer.led_matrix.get_by_index(2)

pulse_led_on_crossing_a = BoolBoolMapping(lorenz_attractor_a._crossed_zero,
                                          pulse_led_a.set_value)

pulse_socket_on_crossing_a = BoolBoolMapping(lorenz_attractor_a._crossed_zero,
                                             computer.pulses_output_socket_one.set_value)

pulse_led_on_crossing_b = BoolBoolMapping(lorenz_attractor_b._crossed_zero,
                                          pulse_led_b.set_value)

pulse_socket_on_crossing_b = BoolBoolMapping(lorenz_attractor_b._crossed_zero,
                                             computer.pulses_output_socket_two.set_value)

timer_function_mappings = [pulse_led_on_crossing_a,
                           pulse_socket_on_crossing_a,
                           pulse_led_on_crossing_b,
                           pulse_socket_on_crossing_b]

write_output = WriteOutput(
    mappings=[
        Mapping(source=lorenz_attractor_a.z,  # float (0, 50)
                output=computer.cv_audio_output_socket_one,  # int (0, 4095)
                ranges_frozen=False),
        Mapping(source=lorenz_attractor_a.x,
                output=computer.cv_output_socket_one,
                ranges_frozen=False),
        Mapping(source=lorenz_attractor_b.z,
                output=computer.cv_audio_output_socket_two,
                ranges_frozen=False),
        Mapping(source=lorenz_attractor_b.x,
                output=computer.cv_output_socket_two,
                ranges_frozen=False)
    ],
    looper=looper,
    freq=300,  # max is ~300 atm
    function_mappings=[pulse_led_on_crossing_a,
                       pulse_socket_on_crossing_a,
                       pulse_led_on_crossing_b,
                       pulse_socket_on_crossing_b]
)

time.sleep(0.5)
led_matrix.turn_off()


# these are all candidates for class methods of some sort of main loop class (or class having a main loop)
# @timed_function
def update_function_mappings(function_mappings) -> None:
    """Update mappings from boolean-valued functions (on hardware?) to functions."""
    for function_mapping in function_mappings:
        function_mapping.update()


# @timed_function
def update_cv_output_range_mappings(cv_output_range_mappings) -> None:
    """Update mappings from hardware to the CV output hardware ranges."""
    for mapping in cv_output_range_mappings:
        mapping.update_latest_value(write_output=True,
                                    update_source_value=False)


# @timed_function
def update_hardware_input_mappings(hardware_input_mappings) -> None:
    """Update mappings from hardware objects to software variables."""
    for mapping in hardware_input_mappings:
        mapping.update_latest_value(write_output=True,
                                    update_source_value=False)


while True:
    # poll the analog inputs once (this is currently pointlessly reading the CV ins for timing)
    computer.update_analog_inputs()

    # update mappings from the analog inputs to their destinations
    update_hardware_input_mappings(hardware_input_mappings)
    update_cv_output_range_mappings(cv_output_range_mappings)
    update_function_mappings(function_mappings)

