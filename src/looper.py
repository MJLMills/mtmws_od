from connect.signal import Signal


class Looper(object):
    """A looper for models.

    Parameters
    ----------
    models
        A list of the models to be looped.
    looping
        Whether to being looping on instantiation.
        Default value is False.
    """

    def __init__(self,
                 models: list,
                 looping: bool = False):

        self._models = models
        self._looping = looping

        self.looping_started = Signal()
        self.looping_stopped = Signal()
        self.initial_coordinates_set = Signal()
        self.at_loop_start = Signal()

        self._counter = 0
        self._num_steps = None

        self._needs_update = True

    def is_looping(self):
        return self._looping == True

    def is_not_looping(self):
        return not self.is_looping()

    @property
    def needs_update(self) -> bool:
        return self._needs_update

    @needs_update.setter
    def needs_update(self, needs_update):
        self._needs_update = needs_update

    def set_initial_coordinates(self) -> None:
        """Set the initial coordinates of all the looped models."""
        self.reset()
        for model in self._models:
            model.set_initial_coordinates()

        self.initial_coordinates_set.emit()

    def reset(self) -> None:
        """Return the model trajectories to their initial coordinates."""
        self._counter = 0

    def start_looping(self) -> None:
        """Start looping the models.

        On loop start, the current position of the model will be used as the
        final position for the loop. To enable this behaviour, the number of
        steps taken since the start point was set is saved, and the counter is
        reset to zero. Turning on the looping indicator means that the method
        taking model steps will check if the counter hits the desired number of
        steps and restart.
        """
        if self._looping == False:
            self._looping = True
            self._num_steps = self._counter
            self.reset()
            for model in self._models:
                model.reset()

            self.looping_started.emit()

    def stop_looping(self) -> None:
        """Stop looping the models.

        On stopping the loop, the start position is retained but the final
        position is not kept, since it is set when the loop begins. The counter
        continues to count from the start point in case the looping is started
        again, so that the start point is kept.
        """
        if self._looping == True:
            self._looping = False
            self._num_steps = None
            self.looping_stopped.emit()

    def take_step(self) -> None:
        """Take a step along each model's trajectory.

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
                for model in self._models:
                    model.reset()

                self.reset()
                self.at_loop_start.emit()
                return

        for model in self._models:
            model.take_step()

        self._counter += 1

    def toggle_looping(self) -> None:
        """Toggle the looping on or off."""
        if self._looping:
            self.stop_looping()
        else:
            self.start_looping()
