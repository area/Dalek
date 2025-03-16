class Ears:
    def __init__(self, pin):
        self.pin = pin

    def flash(self):
        self.pin.on()
