import machine
from looper import Looper


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
                 looper: Looper):

        super().__init__(pulse_input,
                         outputs=[looper],
                         trigger=machine.Pin.IRQ_RISING)

        self._looper = self._outputs[0]

    def callback(self, irq_source_pin):
        """Set the start point when the pin goes high."""
        self._looper.set_initial_coordinates()
