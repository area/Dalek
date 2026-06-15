# liberally copy and pasted from https://github.com/nrlakin/robot_voice/blob/master/robot.py
import getopt
import numpy as np
import scipy.io.wavfile as wavfile
import math
import sys
import pyttsx4
from io import BytesIO

"""
Constants
"""

# Diode constants (must be below 1; paper uses 0.2 and 0.4)
VB = 0.2
VL = 0.4

# Controls distortion
H = 40

# Controls N samples in lookup table; probably leave this alone
LOOKUP_SAMPLES = 1024

# Frequency (in Hz) of modulating frequency
MOD_F = 50

class Waveshaper():
    """
    Apply a transform to an audio signal; store transform as curve,
    use curve as lookup table.  Implementation of jQuery's WaveShaperNode
    API:
        http://webaudio.github.io/web-audio-api/#the-waveshapernode-interface
    """
    def __init__(self, curve):
        self.curve = curve
        self.n_bins = self.curve.shape[0]

    def transform(self, samples):
        # normalize to 0 < samples < 2
        max_val = np.max(np.abs(samples))
        if max_val >= 1.0:
            result = samples/np.max(np.abs(samples)) + 1.0
        else:
            result = samples + 1.0
        result = result * (self.n_bins-1)/2
        return self.curve[result.astype(int)]

def diode_lookup(n_samples):
    result = np.zeros((n_samples,))
    for i in range(0, n_samples):
        v = float(i - float(n_samples)/2)/(n_samples/2)
        v = abs(v)
        if v < VB:
            result[i] = 0
        elif VB < v <= VL:
            result[i] = H * ((v - VB)**2)/(2*VL - 2*VB)
        else:
            result[i] = H*v - H*VL + (H*(VL-VB)**2)/(2*VL-2*VB)

    return result

def raw_diode(signal):
    result = np.zeros(signal.shape)
    for i in range(0, signal.shape[0]):
        v = signal[i]
        if v < VB:
            result[i] = 0
        elif VB < v <= VL:
            result[i] = H * ((v - VB)**2)/(2*VL - 2*VB)
    else:
        result[i] = H*v - H*VL + (H*(VL-VB)**2)/(2*VL-2*VB)
    return result


# rate, data = wavfile.read('sample.wav')
def modulate(rate, raw_wav_data):
    if len(raw_wav_data.shape) > 1:
        data = raw_wav_data[:,1]
    else:
        data = raw_wav_data

    # get max value to scale to original volume at the end
    scaler = np.max(np.abs(data))

    # Normalize to floats in range -1.0 < data < 1.0
    data = data.astype(float)/scaler

    # Length of array (number of samples)
    n_samples = data.shape[0]

    # Create the lookup table for simulating the diode.
    d_lookup = diode_lookup(LOOKUP_SAMPLES)
    diode = Waveshaper(d_lookup)

    # Simulate sine wave of frequency MOD_F (in Hz)
    tone = np.arange(n_samples)
    tone = np.sin(2*np.pi*tone*MOD_F/rate)

    # Gain tone by 1/2
    tone = tone * 0.5

    # Junctions here
    tone2 = tone.copy() # to top path
    data2 = data.copy() # to bottom path

    # Invert tone, sum paths
    tone = -tone + data2 # bottom path
    data = data + tone2 #top path

    #top
    data = diode.transform(data) + diode.transform(-data)

    #bottom
    tone = diode.transform(tone) + diode.transform(-tone)

    result = data - tone

    #scale to +-1.0
    result /= np.max(np.abs(result))
    #now scale to max value of input file.
    result *= scaler

    return result
    # wavfile.write wants ints between +-5000; hence the cast
    # wavfile.write('robot.wav', rate, result.astype(np.int16))

if __name__ == '__main__':
    # rate, data = wavfile.read('sample.wav')
    # print(rate, data)
    # modulated_data = modulate(rate, data)
    # # wavfile.write wants ints between +-5000; hence the cast
    # wavfile.write('robot.wav', rate, modulated_data.astype(np.int16))
    newVoiceRate = 120

    engine = pyttsx4.init()
    engine.setProperty('rate',newVoiceRate)
    b = BytesIO()
    engine.save_to_file('EX-TER-MIN-ATE!', b)
    engine.runAndWait()
    #the bs is raw data of the audio.
    bs=b.getvalue()
    audio_array = np.frombuffer(bs, dtype=np.int16)  # Example for 16-bit audio

    print(audio_array)
    # now modulate
    modulated_data = modulate( 22050, audio_array)

    wavfile.write('robot.wav', 22050, modulated_data.astype(np.int16))
    # add an wav file format header
    # b=bytes(b'RIFF')+ (len(modulated_data)+38).to_bytes(4, byteorder='little')+b'WAVEfmt\x20\x12\x00\x00' \
    #                                                             b'\x00\x01\x00\x01\x00' \
    #                                                             b'\x22\x56\x00\x00\x44\xac\x00\x00' +\
    #     b'\x02\x00\x10\x00\x00\x00data' +(len(modulated_data)).to_bytes(4, byteorder='little')+modulated_data
    # # changed to BytesIO
    # b=BytesIO(b)


    # audio = AudioSegment.from_file(b, format="wav")
    # play(audio)
    