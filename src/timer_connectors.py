import machine

class TimerConnector:

    def __init__(self, inputs, outputs, freq):  # TODO - has to take all timer params
        self._inputs = inputs
        self._outputs = outputs
        self._timer = machine.Timer(-1, freq=freq, callback=self.callback)

    def callback(self, timer):
        raise NotImplementedError(self.__class__.__name__ + "")

class WriteOutput(TimerConnector):
    """Action to take whenever the output timer runs."""
    def __init__(self,
                 looper,
                 cv_output_x_a,
                 cv_output_z_a,
                 cv_output_x_b,
                 cv_output_z_b,
                 pulse_output_a,
                 pulse_output_b,
                 pulse_led_a,
                 pulse_led_b,
                 freq):

        super().__init__(inputs=[looper],
                         outputs=[looper,
                                  cv_output_x_a,
                                  cv_output_z_a,
                                  cv_output_x_b,
                                  cv_output_z_b,
                                  pulse_output_a,
                                  pulse_output_b,
                                  pulse_led_a,
                                  pulse_led_b],
                         freq=freq)

        self._looper = looper

        self._cv_output_x_a = cv_output_x_a
        self._cv_output_z_a = cv_output_z_a
        self._cv_output_x_b = cv_output_x_b
        self._cv_output_z_b = cv_output_z_b

        self._pulse_output_a = pulse_output_a
        self._pulse_output_b = pulse_output_b
        self._pulse_led_a = pulse_led_a
        self._pulse_led_b = pulse_led_b

    def callback(self, _):
        """Write the properly scaled outputs."""

        self._cv_output_z_a.write(self._looper.lorenz_system_a.mapped_z_value)
        self._cv_output_x_a.write(self._looper.lorenz_system_a.mapped_x_value)
        self._cv_output_z_b.write(self._looper.lorenz_system_b.mapped_z_value)
        self._cv_output_x_b.write(self._looper.lorenz_system_b.mapped_x_value)

        if self._looper.lorenz_system_a.crossed_zero:
            self._pulse_led_a.turn_on()
            self._pulse_output_a.pulse(0.01)
            self._pulse_led_a.turn_off()

        if self._looper.lorenz_system_b.crossed_zero:
            self._pulse_led_b.turn_on()
            self._pulse_output_a.pulse(0.01)
            self._pulse_led_b.turn_off()

        self._looper.needs_update = True
