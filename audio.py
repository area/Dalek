import asyncio
import os

from pulsectl_asyncio import PulseAsync
from pulsectl_asyncio.pulsectl_async import PulseError

from ears import Ears

EAR_TRIGGER_THRESHOLD = 0.66


async def monitor_audio_output(ears: Ears):
    # Explicitly set some required env variables that aren't
    # automatically present when starting via cron.
    os.environ["XDG_RUNTIME_DIR"] = f"/run/user/{os.getuid()}"
    print("Starting audio output monitoring")
    pa = PulseAsync("dalek_ears")
    error_count = 0
    while True:
        try:
            pulse = await pa.__aenter__()
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
    try:
        server_info = await pulse.server_info()
        print("Got server info", server_info)
        default_sink_info = await pulse.get_sink_by_name(server_info.default_sink_name)
        print("Got default sink", default_sink_info)
        async for level in pulse.subscribe_peak_sample(default_sink_info.monitor_source_name):
            if level > EAR_TRIGGER_THRESHOLD:
                print(f"Audio level {level} is above threshold, flashing ears...")
                ears.flash()
    finally:
        pa.__aexit__(None, None, None)