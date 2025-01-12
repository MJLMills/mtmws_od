import machine


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
    #@timed_function
    def update(self, _):
        self.update_mappings()
        looper.take_step()

    #@timed_function
    def callback(self, _):
        micropython.schedule(self.update, 0)
