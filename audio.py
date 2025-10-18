from pulsectl_asyncio import PulseAsync

from ears import Ears

EAR_TRIGGER_THRESHOLD = 0.66


async def monitor_audio_output(ears: Ears):
    print("Starting audio output monitoring")
    async with PulseAsync("dalek_ears") as pulse:
        server_info = await pulse.server_info()
        print("Got server info", server_info)
        default_sink_info = await pulse.get_sink_by_name(server_info.default_sink_name)
        print("Got default sink", default_sink_info)
        async for level in pulse.subscribe_peak_sample(default_sink_info.monitor_source_name):
            if level > EAR_TRIGGER_THRESHOLD:
                print(f"Audio level {level} is above threshold, flashing ears...")
                ears.flash()