from model import Model

class Looper(object):
    """A looper for a model."""
    def __init__(self,
                 model: Model,
                 looping: bool = False):

        self._model = model
        self._looping = looping

        self._counter = 0
        self._num_steps = None

    def set_initial_coordinates(self):
        self.reset()
        self._model.set_initial_coordinates()

    def reset(self):
        self._counter = 0

    def start_looping(self):
        """Start looping the model.

        On loop start, the current position of the model will be used as the
        final position for the loop. To enable this behaviour, the number of
        steps taken since the start point was set is saved, and the counter is
        reset to zero. Turning on the looping indicator means that the method
        taking model steps will check if the counter hits the desired number of
        steps and restart.
        """
        self._looping = True
        self._num_steps = self._counter
        self.reset()
        self._model.reset()


    def stop_looping(self):
        """Stop looping the model.

        On stopping the loop, the start position is retained but the final
        position is not kept, since it is set when the loop begins. The counter
        continues to count from the start point in case the looping is started
        again, so that the start point is kept.
        """
        self._looping = False
        self._num_steps = None

    def take_step(self):
        """Take a step along the model's trajectory.

        If the system is not being looped, this method just takes a step on the
        model trajectory and increments the counter. If the number of steps in
        the loop has been reached, the looper is at the end of its loop and so
        moves the system back to its initial coordinates and resets the
        counter. At the end of a loop, the "step" is to return to the model's
        initial position, so take_step is not called as this would result in a
        double step in a single cycle.
        """
        if self._looping:
            if self._counter == self._num_steps:
                self._model.reset()
                self.reset()
                return

        self._model.take_step()
        self._counter += 1