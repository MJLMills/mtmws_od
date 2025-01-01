class Mapping(object):
    def __init__(self,
                 input_,  # class property
                 output_min,
                 output_max):  # class property

        self._input_ = input_
        self._input_min = input_.min_value

        self._output_min = output_min

        self._slope = (output_max - output_min) / (input_.max_value - input_.min_value)

        self._output_value = None

    def update(self) -> None:
        """Set the output property by scaling the input property."""
        # analog inputs should take care of only mapping their values if the value changed.
        # the analog input can have hysteresis, and this method can cache the
        # last accessed value to do a check before doing the float arithmetic
        self._output_value = \
            self._output_min + self._slope * (self._input_.latest_value -
                                              self._input_min)

    @property
    def current_value(self):
        """made accessible for use when needed?"""
        return self._output_value


class Source:
    @property
    def latest_value(self):
        return 0.5

    @property
    def min_value(self):
        return 0

    @property
    def max_value(self):
        return 1

class Sink:
    def __init__(self):
        self._value = None

    @property
    def value(self):
        return self._value

    @value.setter
    def value(self, value):
        self._value = value


source = Source()
sink = Sink()

my_mapping = Mapping(input_=source,
                     output_min=-1,
                     output_max=1)

my_mapping.update()
print(my_mapping.current_value)
