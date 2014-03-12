import numpy as np
import math
import matplotlib.pyplot as plt
from matplotlib import mlab
import scipy.io.wavfile as wavefile
import cookb_signalsmooth

# Spectrum Estimation (WOSA) parameters
Ns = 1024
Overlap_Percent = 0.7
nOverlap = Ns * 0.7

def plot_graph(freq, Pxx_1, Pxx_2, Pxx_diff, Pxx_diff_smoothed):
    plt.figure(1)
    plt.plot(freq, Pxx_1)
    plt.title('Reference Channel (CH1)')
    plt.figure(2)
    plt.plot(freq, Pxx_2)
    plt.title('Sensing Channel (CH2)')
    plt.figure(3)
    plt.plot(freq, Pxx_diff)
    plt.title('Difference')
    plt.figure(4)
    plt.plot(freq, Pxx_diff_smoothed)
    plt.title('Smoothed Difference')
    plt.show()


def plot_from_rawdata(data1, data2, rate):
    (Pxx_in, freq) = mlab.psd(data1, NFFT=Ns, Fs=rate, detrend=mlab.detrend_mean, window=mlab.window_hanning, noverlap=nOverlap, sides='onesided')
    (Pxx_out, freq) = mlab.psd(data2, NFFT=Ns, Fs=rate, detrend=mlab.detrend_mean, window=mlab.window_hanning, noverlap=nOverlap, sides='onesided')
    Pxx_in = np.array([10*math.log(p,10) if p != 0 else p for p in Pxx_in])
    Pxx_out = np.array([10*math.log(p,10) if p != 0 else p for p in Pxx_out])
    Pxx_diff = Pxx_out - Pxx_in
    Pxx_diff_smoothed = cookb_signalsmooth.smooth(Pxx_diff, window_len=51, window='flat'); 

    plot_graph(freq, Pxx_in, Pxx_out, Pxx_diff, Pxx_diff_smoothed)
    

def plot_from_wavfile(file1, file2):
    rate1, data1 = wavefile.read(file1)
    rate2, data2 = wavefile.read(file2)
    plot_from_rawdata(data1, data2, rate1)
   
