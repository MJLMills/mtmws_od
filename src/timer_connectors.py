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
                 lorenz_system_a,
                 lorenz_system_b,
                 cv_output_x_a,
                 cv_output_z_a,
                 cv_output_x_b,
                 cv_output_z_b,
                 freq):

        super().__init__(inputs=[lorenz_system_a,
                                 lorenz_system_b],
                         outputs=[cv_output_x_a,
                                  cv_output_z_a,
                                  cv_output_x_b,
                                  cv_output_z_b],
                         freq=freq)

        self._lorenz_system_a = lorenz_system_a
        self._lorenz_system_b = lorenz_system_b

        self._cv_output_x_a = cv_output_x_a
        self._cv_output_z_a = cv_output_z_a
        self._cv_output_x_b = cv_output_x_b
        self._cv_output_z_b = cv_output_z_b

    def callback(self, _):
        """Write the outputs"""
        self._cv_output_z_a.write(self._lorenz_system_a.value_z_u16)
        self._cv_output_x_a.write(self._lorenz_system_a.value_x_u16)
        self._cv_output_z_b.write(self._lorenz_system_b.value_z_u16)
        self._cv_output_x_b.write(self._lorenz_system_b.value_x_u16)

        # TODO - hook this up
        # if self._lorenz_system_a.crossed_zero:
        #     led_matrix.turn_on(led_id_pulse_output_a)
        #     pulse_output_a.pulse(led_pulse_output_length)
        #     led_matrix.turn_off(led_id_pulse_output_a)
        #
        # if self._lorenz_system_b.crossed_zero:
        #     led_matrix.turn_on(led_id_pulse_output_b)
        #     pulse_output_b.pulse(led_pulse_output_length)
        #     led_matrix.turn_off(led_id_pulse_output_b)
