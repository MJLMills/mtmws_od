from collections.abc import Mapping
from ranged_value import RangedValue

# TODO - squash these into a single class, see if it can be made a ranged variable
class MultiMapping(Mapping):  # this is itself a mapping with input and output ranges

    def __init__(self, sources: list, output: RangedValue):
        """

        Parameters
        ----------
        sources : list of RangedValue
        output : RangedValue
        """
        # there are two sources with input ranges and total output range (0, 1)
        # they both need to be mapped to shared output ranges (0, 1) then multiplied, then that is the
        # value mapped to the output (CV socket)
        # shouldn't the two inputs map to the combined output,
        # then that multi-input ranged value is mapped to the CV output value?

        # there is now an asymmetry here where the user provides a pair of
        # mappings instead of a pair of ranged-values, vs. in the Mapping
        # superclass where they provide a single ranged-value. I can't think of
        # a good reason to force the user to define the output ranges when we
        # can potentially save cycles by picking an efficient one ourselves.
        # also if we make this class take a list of source mappings, then there
        # is no longer a need for this subclass

        # here the two source mappings will be the knob and CV input mapped to 0,1
        # then they have to be multiplied, and that ranged value mapped to the
        # CV output

        self._sources = sources
        self._output = output  # range from 0 to 1

    def compute_source_value(self):
        product = 1
        for source_mapping in self._sources:
            source_mapping.update_latest_value(write_output=False)
            source_mapping.write()
            product *= source_mapping.latest_value

        return product


class Mapping(object):

    def __init__(self,
                 source: RangedValue,
                 output: RangedValue):

        self._source = source
        self._output = output

        self._source_min = self._source.min_value
        self._source_min_is_zero = self._source_min == 0

        self._output_min = self._output.min_value
        self._output_min_is_zero = self._output_min == 0

        self._slope = Mapping.__compute_slope(source, output)
        self._slope_is_one = self._slope == 1

        self._latest_output_value = None

    @staticmethod
    def __compute_slope(source, output):

        if source.min_value == source.max_value:
            raise ValueError()

        return (output.max_value - output.min_value) / \
               (source.max_value - source.min_value)

    def compute_source_value(self):
        return self._source.value

    def update_latest_value(self, write_output=False) -> None:
        """Update the output property by scaling the input property."""
        source_value = self.compute_source_value()

        # TODO - for slope == 1, this can be simplified more
        # for matching ranges, you should have output = source.value
        # instead of this ridiculous if-else tree, determine the correct function once and use it every time
        # three bools map to a single function each (one arg, the source value)
        if self._output_min_is_zero and self._source_min_is_zero:

            if self._slope_is_one:
                self._latest_output_value = source_value
            else:
                self._latest_output_value = self._slope * source_value

        elif self._output_min_is_zero:

            if self._slope_is_one:
                self._latest_output_value = \
                    self._slope * (source_value - self._source_min)
            else:
                self._latest_output_value = \
                    source_value - self._source_min

        elif self._source_min_is_zero:

            self._latest_output_value = \
                self._output_min + self._slope * source_value

        else:
            self._latest_output_value = \
                self._output_min + (self._slope * (source_value -
                                                   self._source_min))

        if write_output:
            self.write()

    def write(self):
        """Copy the latest output value to the output object."""
        self._output.value = self._latest_output_value

    def latest_value(self):
        return self._latest_output_value

    def update_latest_value_and_write(self) -> None:
        self.update_latest_value()
        self.write()

#
# # the test Source and Sink classes are RangedValue subclasses (other than in name)
# class Source:
#     def __init__(self):
#         self._value = None
#
#     @property
#     def value(self):
#         return 0.5
#
#     @value.setter
#     def value(self, value):
#         self._value = value
#
#     @property
#     def min_value(self):
#         return 0
#
#     @property
#     def max_value(self):
#         return 1
#
# class Sink:
#     def __init__(self):
#         self._value = None
#
#     @property
#     def value(self):
#         return self._value
#
#     @value.setter
#     def value(self, value):
#         self._value = value
#
#     @property
#     def min_value(self):
#         return -1
#
#     @property
#     def max_value(self):
#         return 1
#
#
# source = Source()
# sink = Sink()
#
# my_mapping = Mapping(source=source,
#                      output=sink)
#
# my_mapping.update_latest_value()
# my_mapping.write()
#
# print(sink.value)
