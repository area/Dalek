from pulsectl_asyncio import PulseAsync


async def monitor_audio_output(ears):
    async with PulseAsync("dalek_ears") as pulse:
        server_info = await pulse.server_info()
        default_sink_info = await pulse.get_sink_by_name(server_info.default_sink_name)
        async for level in pulse.subscribe_peak_sample(default_sink_info.monitor_source_name):
            if level > 0.5:
                ears.flash()