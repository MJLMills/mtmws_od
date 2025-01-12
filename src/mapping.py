from ranged_variable import RangedVariable


class Mapping(object):  # stop this from assuming float is OK. some mappings can be done by int precision drop (16-12 e.g. is >>4) or fixed point?
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
