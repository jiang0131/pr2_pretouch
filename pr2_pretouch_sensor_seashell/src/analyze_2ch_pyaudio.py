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
import argparse

CHUNK = 2200
FORMAT = pyaudio.paInt16
CHANNELS = 2
RATE = 44100 #sample rate

parser = argparse.ArgumentParser()
parser.add_argument('-l', '--load_file', type=argparse.FileType('rb'),
                    help="the pickle file to load sound data from")
parser.add_argument('-s', '--save_file', type=argparse.FileType('wb'),
                    help="the pickle file to save sound data to")
parser.add_argument('-n', '--N', type=int,
                    help="the number of data point for computing psd")
parser.add_argument('-ns', '--Ns', type=int,
                    help="the Ns FFT parameter for computing psd")
parser.add_argument('-ov', '--overlap_ratio', type=float,
                    help="the overlap ratio")
args = parser.parse_args()

print args.overlap_ratio

if args.load_file:
    signal = pickle.load(args.load_file)
else:
    try:
        RECORD_SECONDS = float(raw_input('How many seconds to analyze? ....... '))
    except ValueError:
        print 'Wrong input time. Use the default (1 second) for analyis'
        RECORD_SECONDS = 1
    device_id = helper.find_audio_device(name='Lexicon Omega: USB Audio (hw:1,0)')
    p = pyaudio.PyAudio()
    stream = p.open(format=FORMAT,
                    channels=CHANNELS,
                    rate=RATE,
                    input=True,
                    input_device_index = device_id,               
                    frames_per_buffer=CHUNK) #buffer
    print("* recording")

    ####################################################
    ######## Getting Sound Data through PyAudio ########
    ####################################################
    n_reads = int(RATE * RECORD_SECONDS / CHUNK)
    print 'n_reads=', n_reads
    data = ''.join([stream.read(CHUNK) for i in range(n_reads)])
    print 'len(data)=', len(data)
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
    if args.save_file:
        pickle.dump(signal, args.save_file)
    print("* done recording")
    stream.stop_stream()
    stream.close()
    p.terminate()

N = args.N if args.N else 2048
Ns = args.Ns if args.Ns else 1024
overlap_ratio = args.overlap_ratio if args.overlap_ratio else 0.7
signal = signal[:N]
helper.plot_from_rawdata(signal[0::2], signal[1::2], RATE, Ns, overlap_ratio)
