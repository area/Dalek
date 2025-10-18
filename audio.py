import asyncio
import os

from pulsectl_asyncio import PulseAsync
from pulsectl_asyncio.pulsectl_async import PulseError

from ears import Ears

EAR_TRIGGER_THRESHOLD = 0.66


class Audio:
    def __init__(self):
        # Explicitly set some required env variables that aren't
        # automatically present when starting via cron.
        os.environ["XDG_RUNTIME_DIR"] = f"/run/user/{os.getuid()}"
        self._pa = PulseAsync("dalek_ears")

    async def __aenter__(self) -> PulseAsync:
        if self._pa.connected:
            return self._pa
        while True:
            try:
                await self._pa.connect()
            except PulseError:
                # PulseAudio server not up and running yet.
                error_count += 1
                if error_count > 5:
                    print("Error connecting to PulseAudio too many times. Raising...")
                    raise
                print("Sleep until PulseAudio server available...")
                await asyncio.sleep(30)
            else:
                break
        return self._pa

    async def __aexit__(self, exc_type, exc_val, exc_tb) -> bool:
        self._pa.close()
        return False


async def init_pygame():
    async with Audio():
        os.environ["SDL_AUDIODRIVER"] = "pulse"
        import pygame
        pygame.mixer.init()
        return pygame


async def monitor_audio_output(ears: Ears):
    async with Audio() as pulse:
        server_info = await pulse.server_info()
        print("Got server info", server_info)
        default_sink_info = await pulse.get_sink_by_name(server_info.default_sink_name)
        print("Got default sink", default_sink_info)
        async for level in pulse.subscribe_peak_sample(default_sink_info.monitor_source_name):
            if level > EAR_TRIGGER_THRESHOLD:
                print(f"Audio level {level} is above threshold, flashing ears...")
                ears.flash()
