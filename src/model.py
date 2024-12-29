import array


class Model(object):
    """A model that produces a trajectory."""
    def __init__(self):

        self._coordinates = array.array(
            'f',
            bytearray(4 * self.n_dimensions)
        )

        self._initial_coordinates = array.array(
            'f',
            bytearray(4 * self.n_dimensions)
        )

        self._derivatives = array.array(
            'f',
            bytearray(4 * self.n_dimensions)
        )

        self._parameters = array.array(
            'f',
            bytearray(4 * self.n_parameters)
        )

    @property
    def n_dimensions(self):
        """The number of dimensions of this model."""
        raise NotImplementedError(
            self.__class__.__name__, "does not implement n_dimensions."
        )

    @property
    def n_parameters(self):
        """The number of parameters of this model."""
        raise NotImplementedError(
            self.__class__.__name__, "does not implement n_parameters."
        )

    def take_step(self) -> None:
        """Take a step along the model trajectory."""
        raise NotImplementedError(
            self.__class__.__name__, "does not implement take_step."
        )

    def set_initial_coordinates(self) -> None:
        """Set the initial coordinates to be the current coordinates."""
        for i in range(self.n_dimensions):
            self._initial_coordinates[i] = self._coordinates[i]

    def reset(self) -> None:
        """Return the trajectory to the initial coordinates."""
        for i in range(self.n_dimensions):
            self._coordinates[i] = self._initial_coordinates[i]
