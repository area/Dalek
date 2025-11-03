from gpiozero import OutputDevice, PWMOutputDevice


class Motor:
    def __init__(
        self,
        *,
        pwm_pin: PWMOutputDevice,
        direction_pin: OutputDevice,
        velocity_scaling: tuple[float, float] = (1.0, 1.0)
    ):
        self._pwm_pin = pwm_pin
        self._direction_pin = direction_pin
        assert (
            velocity_scaling[0] > 0 and
            velocity_scaling[0] <= 1 and
            velocity_scaling[1] > 0 and
            velocity_scaling[1] <= 1
        ), "Velocity scaling must be in range (0, 1]."
        self._velocity_scaling = velocity_scaling
        self._velocity = 0

    @property
    def velocity(self) -> float:
        return self._velocity

    def set_velocity(self, velocity: float) -> None:
        assert velocity >= -1 and velocity <= 1, "Velocity must be in range [-1, 1]."
        velocity *= self._velocity_scaling[int(velocity < 0)]
        self._velocity = velocity
        if velocity >= 0:
            self._pwm_pin.value = velocity
            self._direction_pin.off()
        else:
            self._pwm_pin.value = -velocity
            self._direction_pin.on()


class TravelLimitedMotor(Motor):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self._limit_reached = (False, False)

    def set_travel_limit(self, fwd_limit_reached: bool, rev_limit_reached: bool) -> None:
        self._limit_reached = (fwd_limit_reached, rev_limit_reached)

    def set_velocity(self, velocity: float) -> None:
        travel_limited = velocity and self._limit_reached[int(velocity < 0)]
        return super().set_velocity(0 if travel_limited else velocity)
