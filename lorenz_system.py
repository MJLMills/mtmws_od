import array
import math

class LorenzSystem:
    """The Lorenz system of ordinary differential equations.

    Parameters
    ----------
    x : float
        The current x-coordinate. Default starting value is 0.0.
    y : float
        The current y-coordinate. Default starting value is 1.0.
    z : float
        The current z-coordinate. Default starting value is 1.05.
    sigma : float
        Default value is 10.
        The value of this parameter must be positive.
        It will be clamped to zero if set negative.
    rho : float
        Default value is 28.
        The value of this parameter must be positive.
        It will be clamped to zero if set negative.
    beta : float
        Default value is 8/3.

    Attributes
    ----------
    x, y, z -> float
        The current coordinates.

    Methods
    -------
    take_step -> None
        Take a single step along the trajectory.
    """

    @staticmethod
    def create_lorenz_attractor():
        return LorenzSystem(sigma=10, rho=28, beta=8 / 3)

    @staticmethod
    def create_torus_knot():
        """With ρ = 99.96 it becomes a T(3,2) torus knot."""
        return LorenzSystem(sigma=10, rho=99.96, beta=8 / 3)

    @staticmethod
    def create_stable_system():
        """Create a system that evolves to one of two fixed point attractors."""
        return LorenzSystem(sigma=10, rho=14, beta=8 / 3)

    def __init__(self,
                 x=0.9,
                 y=0,
                 z=0,
                 sigma=10,
                 rho=28,
                 beta=8 / 3,
                 convergence_threshold=1e-3,
                 random_factor=None):

        self._x_init = x
        self._y_init = y
        self._z_init = z

        self._x = None
        self._y = None
        self._z = None
        self.reset()

        self._sigma = sigma
        self._rho = rho
        self._beta = beta

        self._random_factor = random_factor
        self._convergence_threshold = convergence_threshold

        self._crossed_zero = False

    @property
    def x(self):
        return self._x

    @property
    def y(self):
        return self._y

    @property
    def z(self):
        return self._z

    @property
    def crossed_zero(self):
        return self._crossed_zero

    def coordinates(self):
        return array.array('f', [self._x, self._y, self._z])

    @property
    def convergence_threshold(self):
        """The threshold below which the system is considered converged."""
        return self._convergence_threshold

    @convergence_threshold.setter
    def convergence_threshold(self, convergence_threshold):
        self._convergence_threshold = convergence_threshold

    @property
    def sigma(self):
        return self._sigma

    @sigma.setter
    def sigma(self, sigma):
        if sigma < 0:
            self._sigma = 0
            print("Warning: sigma attempted to go below zero", sigma)
        else:
            self._sigma = sigma

    @property
    def rho(self):
        return self._rho

    @rho.setter
    def rho(self, rho):
        if rho < 0:
            self._rho = 0
            print("Warning: rho attempted to go below zero", rho)
        else:
            self._rho = rho

    @property
    def beta(self):
        return self._beta

    @beta.setter
    def beta(self, beta):
        if beta < 0:
            self._beta = 0
            print("Warning: beta attempted to go below zero", beta)
        else:
            self._beta = beta

    def __compute_derivatives(self, x, y, z):
        dx_dt = self._sigma * (y - x)
        dy_dt = x * (self._rho - z) - y
        dz_dt = (x * y) - (self._beta * z)

        values = array.array('d', [dx_dt, dy_dt, dz_dt])

        return values

    def take_step(self, step_size: float = 0.01) -> None:
        """Determine the value of f(t_n+1, x, y, z) at the next timestep.

        This stepper uses the most basic possible scheme to integrate
        the ODEs of the system; Euler's method. This only requires a
        single evaluation of f(t, x, y, z) so is cheap, but is only accurate
        on the order of the square of the timestep.

        Parameters
        ----------
        step_size
            The size of the step (in time units) to take.
            Default of 0.01 was determined by experimentation only.
        """
        partial_derivatives = self.__compute_derivatives(self._x,
                                                         self._y,
                                                         self._z)

        self._x = self._x + (partial_derivatives[0] * step_size)
        self._y = self._y + (partial_derivatives[1] * step_size)
        self._z = self._z + (partial_derivatives[2] * step_size)

    def reset(self):
        """Return the system to its starting values."""
        self.move_to(self._x_init, self._y_init, self._z_init)

    def move_to(self, x, y, z):
        """Move the system to the provided point.

        This requires restarting the integrator, which for Euler is
        trivial because it has no reliance on past time-steps. If a
        more advanced integration scheme is used this must be taken
        into account.
        """
        self._x = x
        self._y = y
        self._z = z

        if (x * self._x) < 0:
            self._crossed_zero = True
        else:
            self._crossed_zero = False

    def is_stable(self) -> bool:
        """Determine whether this Lorenz system is stable.

        A stable system evolves towards a fixed point attractor.
        """
        return self.sigma > (self.beta + 1)

    def is_converged(self) -> bool:
        """Determine whether this Lorenz system has converged."""
        if not self.is_stable():
            return False

        if self.rho < 1:
            ...  # check if the system has reached the origin yet.
            # get sq distance from x, y, z to (0, 0, 0). If below threshold, return True.
        else:
            # get sq distance from x, y, z to each of the fixed points.
            # if either is below threshold, return True
            ...

    @property
    def fixed_points(self):
        """The pair of fixed points of the system (for rho >1)."""
        rho_minus_one = self.rho - 1
        p = math.sqrt(self.beta * rho_minus_one)

        fixed_point_one = (p, p, rho_minus_one)
        fixed_point_two = (-p, -p, rho_minus_one)

        return fixed_point_one, fixed_point_two

    def __update_fixed_points(self):
        """Compute the pair of fixed points of the system (for rho >1)."""

        rho_minus_one = self.rho - 1
        p = math.sqrt(self.beta * rho_minus_one)

        self._fixed_point_one = (p, p, rho_minus_one)
        self._fixed_point_two = (-p, -p, rho_minus_one)


class LoopableLorenzSystem(LorenzSystem):
    """A Lorenz system that can be looped."""
    def __init__(self,
                 x=0.9,
                 y=0,
                 z=0,
                 sigma=10,
                 rho=28,
                 beta=8 / 3,
                 looping=False):

        super().__init__(x=x,
                         y=y,
                         z=z,
                         sigma=sigma,
                         rho=rho,
                         beta=beta)

        self._x_final = None
        self._y_final = None
        self._z_final = None

        self._looping = looping

    def set_startpoint(self):
        self._x_init = self._x
        self._y_init = self._y
        self._z_init = self._z

    def set_endpoint(self):
        self._x_final = self.x
        self._y_final = self.y
        self._z_final = self.z

    def start_loop(self):
        self._looping = True

    def stop_loop(self):
        self._looping = False

    def take_step(self, step_size: float = 0.01) -> None:
        """Determine the value of f(t_n+1, x, y, z) at the next timestep.

        This stepper uses the most basic possible scheme to integrate
        the ODEs of the system; Euler's method. This only requires a
        single evaluation of f(t, x, y, z) so is cheap, but is only accurate
        on the order of the square of the timestep.

        Parameters
        ----------
        step_size
            The size of the step (in time units) to take.
            Default of 0.01 was determined by experimentation only.
        """
        super().take_step(step_size)
        if self._looping:
            ...  # check whether we reach the end time-point, if so reset, if not do nothing.