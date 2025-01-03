import machine

class TimerConnector:

    def __init__(self, freq):  # TODO - has to take all timer params

        self._timer = machine.Timer(-1, freq=freq, callback=self.callback)

    def callback(self, timer):
        raise NotImplementedError(self.__class__.__name__ + "")

class WriteOutput(TimerConnector):
    """Action to take whenever the output timer runs."""
    def __init__(self,
                 looper,
                 lorenz_a_z_mapping,
                 lorenz_b_z_mapping,
                 lorenz_a_x_mapping,
                 lorenz_b_x_mapping,
                 pulse_output_a,
                 pulse_output_b,
                 pulse_led_a,
                 pulse_led_b,
                 freq):

        super().__init__(freq=freq)

        self._looper = looper

        self._lorenz_a_z_mapping = lorenz_a_z_mapping
        self._lorenz_b_z_mapping = lorenz_b_z_mapping
        self._lorenz_a_x_mapping = lorenz_a_x_mapping
        self._lorenz_b_x_mapping = lorenz_b_x_mapping

        self._pulse_output_a = pulse_output_a
        self._pulse_output_b = pulse_output_b
        self._pulse_led_a = pulse_led_a
        self._pulse_led_b = pulse_led_b

    def callback(self, _):
        """Write the properly scaled outputs."""

        self._lorenz_a_z_mapping.write()
        self._lorenz_b_z_mapping.write()
        self._lorenz_a_x_mapping.write()
        self._lorenz_b_x_mapping.write()

        if self._looper.lorenz_system_a.crossed_zero:
            self._pulse_led_a.turn_on()
            self._pulse_output_a.pulse(0.01)
            self._pulse_led_a.turn_off()

        if self._looper.lorenz_system_b.crossed_zero:
            self._pulse_led_b.turn_on()
            self._pulse_output_a.pulse(0.01)
            self._pulse_led_b.turn_off()

        self._looper.needs_update = True
