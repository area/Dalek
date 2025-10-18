import asyncio

from gpiozero import LED

PERSIST_TIME_S = 0.4


class Ears:
    def __init__(self, pin: LED):
        self._pin = pin
        self._flash_task = None

    def flash(self) -> None:
        if self._flash_task and not self._flash_task.done():
            return
        self._pin.on()

        self._flash_task = asyncio.create_task(self._off())

    async def _off(self):
        await asyncio.sleep(PERSIST_TIME_S)
        self._pin.off()
