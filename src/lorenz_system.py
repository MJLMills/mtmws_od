from model import Model
import random
from connect.ranged_variable import RangedVariable
from connect.signal import Signal


class LorenzSystem(Model):
    """The Lorenz system of ordinary differential equations.

    Parameters
    ----------
    x
        The current x-coordinate. Default starting value is 0.0.
    y
        The current y-coordinate. Default starting value is 1.0.
    z
        The current z-coordinate. Default starting value is 1.05.
    sigma
        Default value is 10.
    rho
        Default value is 28.
    beta
        Default value is 8/3.
    timestep
        The size of the step (in time units) to take.
        Default of 0.01 was determined by experimentation only.
    random_factor
        The maximum magnitude of the perturbation applied at reset.

    Properties
    ----------
    n_dimensions -> int
        The number of dimensions of this model.
    n_parameters -> int
        The number of parameters of this model.

    Methods
    -------
    take_step
        Take a step along the trajectory.
    set_initial_coordinates
        Set the initial coordinates to be the current coordinates.
    reset
        Return the trajectory to the initial coordinates.
    """

    def __init__(self,
                 x: float = 0.9,
                 y: float = 0.0,
                 z: float = 0.0,
                 sigma: float = 10.0,
                 rho: float = 28,
                 beta: float = 8 / 3,
                 timestep: RangedVariable = RangedVariable(maximum=0.01,
                                                           minimum=0.001,
                                                           value=0.01),
                 random_factor: RangedVariable = RangedVariable(maximum=1.0,
                                                                minimum=0.0,
                                                                value=0.0)):

        super().__init__()

        self._coordinates = [x, y, z]
        self._initial_coordinates = [x, y, z]
        self._parameters = [sigma, rho, beta]

        # values used outside this class
        self._x = RangedVariable(minimum=-24, maximum=24,
                                 value=self._coordinates[0])
        self._z = RangedVariable(minimum=0, maximum=55,
                                 value=self._coordinates[2])

        # values which can be changed outside this class
        self._timestep = timestep
        self._random_factor = random_factor

        self._previous_x = None

        self.crossed_zero = Signal()
        self.x_changed = Signal()
        self.z_changed = Signal()

    @property
    def timestep(self) -> RangedVariable:
        return self._timestep

    @property
    def random_factor(self) -> RangedVariable:
        return self._random_factor

    @property
    def x(self) -> RangedVariable:
        return self._x

    @property
    def z(self) -> RangedVariable:
        return self._z

    @property
    def n_dimensions(self) -> int:
        return 3

    @property
    def n_parameters(self) -> int:
        return 3

    def reset(self) -> None:
        """Return the trajectory to the initial coordinates."""
        super().reset()
        if self._random_factor:
            for i in range(self.n_dimensions):
                self._coordinates[
                    i] += self._random_factor.value * random.random()

    def take_step(self) -> None:
        """Determine the value of f(t_n+1, x, y, z) at the next timestep.

        This stepper uses the most basic possible scheme to integrate
        the ODEs of the system; Euler's method. This only requires a
        single evaluation of f(t, x, y, z) so is cheap, but is only accurate
        on the order of the square of the timestep.
        """
        previous_x = self._coordinates[0]

        timestep_value = self._timestep.value
        x = self._coordinates[0]
        y = self._coordinates[1]
        z = self._coordinates[2]

        self._coordinates[0] = x + (
                timestep_value * (self._parameters[0] * (y - x)))
        self._coordinates[1] = y + (
                timestep_value * (x * (self._parameters[1] - z) - y))
        self._coordinates[2] = z + (
                timestep_value * ((x * y) - (self._parameters[2] * z)))

        self._x.value = self._coordinates[0]
        self._z.value = self._coordinates[2]

        self.x_changed.emit(ranged_variable=self._x)
        self.z_changed.emit(ranged_variable=self._z)

        if (previous_x * self._coordinates[0]) <= 0:
            self.crossed_zero.emit()
