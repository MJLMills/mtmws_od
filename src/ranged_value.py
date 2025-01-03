class RangedValue:
    def __init__(self, min_value, max_value):
        self._value = None
        self._min_value = min_value
        self._max_value = max_value

    @property
    def value(self):
        return self._value

    @value.setter
    def value(self, value):
        self._value = value

    @property
    def min_value(self):
        return self._min_value

    @property
    def max_value(self):
        return self._max_value