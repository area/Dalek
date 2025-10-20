from gpiozero import OutputDevice, PWMOutputDevice


class Motor:
    def __init__(
        self,
        pwm_pin: PWMOutputDevice,
        direction_pin: OutputDevice,
    ):
        self._pwm_pin = pwm_pin
        self._direction_pin = direction_pin

    def set_velocity(self, velocity: float) -> None:
        assert velocity >= -1 and velocity <= 1, "Velocity must be in range [-1, 1]."
        if velocity >= 0:
            self._pwm_pin.value = velocity
            self._direction_pin.off()
        else:
            self._pwm_pin.value = -velocity
            self._direction_pin.on()
