import pyaudio
import wave

CHUNK = 2200
FORMAT = pyaudio.paInt24
CHANNELS = 2
RATE = 44100 #sample rate
#RECORD_SECONDS = 5
RECORD_SECONDS = int(raw_input('How many seconds to record? ....... '))
#WAVE_OUTPUT_FILENAME = "output.wav"
FILENAME_1 = 'record_ch1_pyaudio.wav'
FILENAME_2 = 'record_ch2_pyaudio.wav'

p = pyaudio.PyAudio()
print p.get_device_info_by_index(4)
p.is_format_supported(RATE, input_device=4, input_channels=CHANNELS, input_format=FORMAT)

stream = p.open(format=FORMAT,
                channels=CHANNELS,
                rate=RATE,
                input=True,
                input_device_index = 4,               
                frames_per_buffer=CHUNK) #buffer

print("* recording")

data_1 = []
data_2 = []
n = p.get_sample_size(FORMAT)

for i in range(0, int(RATE / CHUNK * RECORD_SECONDS)):
    data = stream.read(CHUNK)
    data_1.extend([data[i:i+n] for i in range(0,len(data),2*n)])
    data_2.extend([data[i+n:i+2*n] for i in range(0,len(data),2*n)])

print("* done recording")

stream.stop_stream()
stream.close()
p.terminate()

value_str_1 = ''.join(data_1)
value_str_2 = ''.join(data_2)
record_1 = wave.open(FILENAME_1, 'w')
record_1.setnchannels(1)
record_1.setsampwidth(n)
record_1.setframerate(RATE)
record_1.writeframesraw(value_str_1)
record_2 = wave.open(FILENAME_2, 'w')
record_2.setnchannels(1)
record_2.setsampwidth(n)
record_2.setframerate(RATE)
record_2.setsampwidth(p.get_sample_size(FORMAT))
record_2.writeframesraw(value_str_2)

print "number of samples(frames): " +  str( len(data_1))
print "SAMPLING RATE: " + str(RATE) + "(Hz)"
print "DURATION: " + str(RECORD_SECONDS) + "(s)"
record_1.close()
record_2.close()
print "WAV FILENAME 1: " + FILENAME_1
print "WAV FILENAME 2: " + FILENAME_2

'''

wf = wave.open(WAVE_OUTPUT_FILENAME, 'wb')
wf.setnchannels(CHANNELS)
wf.setsampwidth(p.get_sample_size(FORMAT))
wf.setframerate(RATE)
wf.writeframes(b''.join(frames))
wf.close()
'''
