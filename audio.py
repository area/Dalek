import asyncio

import pyaudio


def list_devices():
    p = pyaudio.PyAudio()
    host_api_info = p.get_host_api_info_by_type(pyaudio.paALSA)
    host_api_index = host_api_info["index"]
    for d in range(host_api_info["deviceCount"]):
        device_info = p.get_device_info_by_host_api_device_index(host_api_index, d)
        print(device_info)
    p.terminate()


async def monitor_audio_output(ears):
    def stream_callback(in_data, frame_count, time_info, status):
        # print(sum(in_data) / frame_count)
        # 4 bytes per frame? 2 channels * 2 byte width?
        print(sum(in_data) / frame_count)
        if sum(in_data) / frame_count > 200:
            ears.flash()
        return None, pyaudio.paContinue

    p = pyaudio.PyAudio()

    host_api_info = p.get_host_api_info_by_type(pyaudio.paALSA)
    host_api_index = host_api_info["index"]
    device_index = None
    sample_rate = None
    for d in range(host_api_info["deviceCount"]):
        device_info = p.get_device_info_by_host_api_device_index(host_api_index, d)
        if "Loopback: PCM (hw:1,1)" in device_info["name"]:
            device_index = device_info["index"]
            sample_rate = device_info["defaultSampleRate"]
            break

    stream = p.open(
        rate=int(44100),
        channels=2,
        format=p.get_format_from_width(2),
        input=True,
        input_device_index=2,
        stream_callback=stream_callback,
    )

    while True:
        try:
            await asyncio.sleep(10)
        except asyncio.CancelledError:
            stream.close()
            p.terminate()
            raise
