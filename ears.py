import asyncio


class Ears:
    def __init__(self, pin):
        self._pin = pin
        self._flash_task = None

    def flash(self):
        self._pin.on()
        if self._flash_task:
            self._flash_task.cancel()
        self._flash_task = asyncio.create_task(self._off())

    async def _off(self):
        await asyncio.sleep(0.5)
        self._pin.off()