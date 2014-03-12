#!/usr/bin/env python
import pyaudio
import wave
import struct
import helper

CHUNK = 2200
FORMAT = pyaudio.paInt24
CHANNELS = 2
RATE = 44100 #sample rate
RECORD_SECONDS = int(raw_input('How many seconds to analyze? ....... '))


device_id = helper.find_audio_interface_device(name='Lexicon Omega: USB Audio (hw:1,0)')

p = pyaudio.PyAudio()
p.is_format_supported(RATE, input_device=device_id, input_channels=CHANNELS, input_format=FORMAT)

stream = p.open(format=FORMAT,
                channels=CHANNELS,
                rate=RATE,
                input=True,
                input_device_index = device_id,               
                frames_per_buffer=CHUNK) #buffer

print("* recording")

data_1 = []
data_2 = []
n = p.get_sample_size(FORMAT)

for i in range(int(RATE / CHUNK * RECORD_SECONDS)):
    data = stream.read(CHUNK)
    data_1.extend([data[i:i+n] for i in range(len(data),2*n)])
    data_2.extend([data[i+n:i+2*n] for i in range(len(data),2*n)])

print("* done recording")

stream.stop_stream()
stream.close()
p.terminate()

value_str_1 = ''.join(data_1)
value_str_2 = ''.join(data_2)

print "number of samples(frames): " +  str(len(data_1))
print "number of bytes: " +  str(len(value_str_1))
print "SAMPLING RATE: " + str(RATE) + "(Hz)"
print "DURATION: " + str(RECORD_SECONDS) + "(s)"

if FORMAT == pyaudio.paInt16:
    signal_1 = struct.unpack("<%uh" % len(data_1), value_str_1)
    signal_2 = struct.unpack("<%uh" % len(data_2), value_str_2)
elif FORMAT == pyaudio.paInt32:
    signal_1 = struct.unpack("<%ul" % len(data_1), value_str_1)
    signal_2 = struct.unpack("<%ul" % len(data_2), value_str_2)
elif FORMAT == pyaudio.paInt8:
    signal_1 = struct.unpack("%ub" % len(data_1), value_str_1)
    signal_2 = struct.unpack("%ub" % len(data_2), value_str_2)
elif FORMAT == pyaudio.paInt24:
    signal_1 = [struct.unpack('<i', value_str_1[i:i+3] + ('\0' if value_str_1[i+2] < '\x80' else '\xff'))[0] for i in range(0, len(value_str_1), 3)]
    signal_2 = [struct.unpack('<i', value_str_2[i:i+3] + ('\0' if value_str_2[i+2] < '\x80' else '\xff'))[0] for i in range(0, len(value_str_2), 3)]

helper.plot_from_rawdata(signal_1, signal_2, RATE)
