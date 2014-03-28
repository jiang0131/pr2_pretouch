#!/usr/bin/env python
'''
Capture sound from 2 channels of the audio device through pyaudio,
and analyze the their spectrum.
@Liang-Ting Jiang 2014-03-12
'''
import pyaudio
import wave
import struct
import helper
import pickle

CHUNK = 2200
FORMAT = pyaudio.paInt16
CHANNELS = 2
RATE = 44100 #sample rate
RECORD_SECONDS = int(raw_input('How many seconds to analyze? ....... '))

LOAD_DATA = True
SAVE_DATA = False

if LOAD_DATA:
    signal = pickle.load(open( "sound_data/signal_5s_4.pickle", "rb" ))
else:
    device_id = helper.find_audio_device(name='Lexicon Omega: USB Audio (hw:1,0)')
    print device_id
    p = pyaudio.PyAudio()
    stream = p.open(format=FORMAT,
                    channels=CHANNELS,
                    rate=RATE,
                    input=True,
                    input_device_index = device_id,               
                    frames_per_buffer=CHUNK) #buffer
    print("* recording")

    ####################################################
    ########### Getting Sound Data #####################
    ####################################################
    n_reads = int(RATE * RECORD_SECONDS / CHUNK)
    data = ''.join([stream.read(CHUNK) for i in range(n_reads)])
    if FORMAT == pyaudio.paInt16:
        signal = struct.unpack("<%uh" % (len(data) / 2), data)
    elif FORMAT == pyaudio.paInt24:
        signal = [struct.unpack('<i', data[i:i+3] + 
                  ('\0' if data[i+2] < '\x80' else '\xff'))[0] 
                  for i in range(0, len(data), 3)]
    elif FORMAT == pyaudio.paInt32:
        signal = struct.unpack("<%ul" % (len(data) / 4), data)
    else:
        raise TypeError("Not supported PyAudio format")
    if SAVE_DATA:
        pickle.dump(signal, open( "signal_5s_5.pickle", "wb" ) )
    print("* done recording")
    stream.stop_stream()
    stream.close()
    p.terminate()

signal_1 = signal[0::2]
signal_2 = signal[1::2]
helper.plot_from_rawdata(signal_1, signal_2, RATE)
