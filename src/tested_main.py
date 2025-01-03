class WriteOutput(TimerConnector):

    def callback(self, timer):

        for mapping in self._mappings:
            mapping.write()

        if looper._models[0]._crossed_zero:
            self._pulse_led_a.turn_on()
            self._pulse_output_socket_a.pulse(0.001)
            self._pulse_led_a.turn_off()

        if looper._models[1]._crossed_zero:
            self._pulse_led_b.turn_on()
            self._pulse_output_socket_b.pulse(0.001)
            self._pulse_led_b.turn_off()

        self._looper.needs_update = True


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
                 looper: Looper,
                 led: LED):
        super().__init__(pulse_input,
                         outputs=[looper, led],
                         trigger=machine.Pin.IRQ_RISING)

        self._looper = self._outputs[0]
        self._led = self._outputs[1]

    def callback(self, _):
        """Toggle looping when the pulse input goes high."""
        #        self._led.toggle()
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


led_matrix = LEDMatrix()
led_matrix.turn_on()

timestep_min: float = 0.0001
# min timestep has no mathematical limit, must be arbitrarily chosen.
timestep_max: float = 0.01
# max timestep is limited by the Euler method integration accuracy.
timestep_factor: float = 11 / 5
# sets how much slower B is than A
timestep_a = RangedValue(min_value=timestep_min,
                         max_value=timestep_max)

timestep_b = RangedValue(max_value=timestep_max / timestep_factor,
                         min_value=timestep_min / timestep_factor)

divergence_min: float = 0.0
# min knob/CV settings will turn off sensitivity to initial conditions.
divergence_max: float = 5.0
# max divergence has no mathematical limit, must be arbitrarily chosen.
divergence = RangedValue(min_value=divergence_min,
                         max_value=divergence_max)

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

hardware_input_mappings = [
    Mapping(source=MainKnob(), output=lorenz_attractor_a.timestep),
    Mapping(source=MainKnob(), output=lorenz_attractor_b.timestep),
    Mapping(source=KnobY(), output=lorenz_attractor_a.random_factor),
    Mapping(source=KnobY(), output=lorenz_attractor_b.random_factor)
]
"""Mappings from hardware inputs to software outputs."""


# some of these are missing:

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


switch = SwitchZ()

# put these in a list (order matters) then update them iteratively in the main loop
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

looper_to_led_on = FunctionMapping(looper.is_looping,
                                   LEDMatrix.get_by_index(3).turn_on)
looper_to_led_off = FunctionMapping(looper.is_not_looping,
                                    LEDMatrix.get_by_index(3).turn_off)

# the x-knob should be scaling the output CV between 0 and a max value - do this, then solve the multiple-sources problem
# then you're basically done?

# can we pass mappings to the IRQConnector?
toggle_looping = ToggleLooping(
    pulse_input=PulseInputSocketOne(),
    looper=looper,
    led=LEDMatrix.get_by_index(3)
)

set_start_point = SetStartPoint(
    pulse_input=PulseInputSocketTwo(),
    looper=looper,
    led=LEDMatrix.get_by_index(4)
)

write_output = WriteOutput(
    mappings=[
        Mapping(source=lorenz_attractor_a.z,
                output=CVAudioOutputSocketOne()),
        Mapping(source=lorenz_attractor_a.x,
                output=CVOutputSocketOne()),
        Mapping(source=lorenz_attractor_b.z,
                output=CVAudioOutputSocketTwo()),
        Mapping(source=lorenz_attractor_b.x,
                output=CVOutputSocketTwo())],
    looper=looper,
    pulse_led_a=LEDMatrix.get_by_index(1),
    pulse_led_b=LEDMatrix.get_by_index(2),
    pulse_output_socket_a=PulseOutputSocketOne(),
    pulse_output_socket_b=PulseOutputSocketTwo(),
    freq=500
)

time.sleep(0.5)
led_matrix.turn_off()

while True:

    if looper.needs_update:
        looper.take_step()
        write_output.update_mappings()

        # in the full implementation, add a flag `update_raw_inputs` to prevent the mappings
    # from calling read on the analog inputs - instead call computer.update_analog_inputs()
    # so that they only do the reads once per loop
    for mapping in hardware_input_mappings:
        mapping.update_latest_value(write_output=True)

    switch.update_latest_value()  # this will be done in the update_analog_inputs call

    switch_down_to_set_startpoint.update()
    switch_up_to_start_looping.update()
    switch_middle_to_stop_looping.update()

    switch_up_to_middle_connected.update()
    switch_middle_to_middle_disconnected.update()

    looper_to_led_on.update()
    looper_to_led_off.update()
