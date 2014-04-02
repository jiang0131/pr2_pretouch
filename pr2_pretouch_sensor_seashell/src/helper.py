'''
Contain helper functions for the pr2_pretouch_sensor package
@Liang-Ting Jiang 2014-03-12
'''
import numpy as np
import math
import matplotlib.pyplot as plt
from matplotlib import mlab
import scipy.io.wavfile as wavefile
import cookb_signalsmooth

# Spectrum Estimation (WOSA) parameters
NS = 1024 #1024
OVERLAP_RATIO = 0.7

def log(data):
    '''
    Convert a np.array to log scale

    Args:
        data (np.array): the input array
    Returns:
        log data (np.array): the log of the input array 
    '''
    for i in range(len(data)):
        data[i] = 10*math.log(data[i], 10) if data[i] != 0 else 0
    return data


def SNR(pxx):
    '''
    Compute the sensor performance indicator SNR.

    Args:
        pxx (list/no.array): the power spectral desnsity
    Return:
        snr (float): the SNR value
    '''
    return np.max(pxx) - np.mean(pxx)


def CNR(array, max_dist=10):
    '''
    Compute the sensor performance indicator CNR. Use 10mm as the farest
    distance, and 1mm as the closest distance.
    
    Args:
        arr (np.array): a MxN 2D array. N is the num of distint distance
                        sensor readings are collected. M is the num of 
                        readings collected at each distance.
        max_dist (int): the farest distance in milimeter
    Returns:
        cnr (float): the CNR value
    '''
    if array.shape[1] < 10:
        raise ValueError("Not enough data")
    mean_std = np.mean(np.std(array, axis=1))
    mean_1mm = np.mean(array[:,0])
    mean_10mm = np.mean(array[:,9])
    return (mean_10mm - mean_1mm) / mean_std


def get_spectrums(signal_in, signal_out, rate, 
                  Ns=NS, overlap_ratio=OVERLAP_RATIO):
    '''
    Compute the spectrums for the input time-series data

    Args:
        signal_in: the time-series data from channel 1
        signal_out: the time-series data from channel 2
        rate: the samling rate for the input signals
    Returns:
        freq: the discrete frequencies of the FFT results
        Pxx_in: the spectrum of the time-series data from channel 1
        Pxx_out: the spectrum of the time-series data from channel 2
        Pxx_diff: the difference (Pxx_out-Pxx_in)
        Pxx_diff_smoothed: the smoothed (in frequency domain) Pxx_diff
    '''
    nOverlap = Ns * overlap_ratio
    (Pxx_in, freq) = mlab.psd(signal_in, NFFT=Ns, Fs=rate, detrend=mlab.detrend_mean, window=mlab.window_hanning, noverlap=nOverlap, sides='onesided')
    (Pxx_out, freq) = mlab.psd(signal_out, NFFT=Ns, Fs=rate, detrend=mlab.detrend_mean, window=mlab.window_hanning, noverlap=nOverlap, sides='onesided')
    Pxx_in = log(Pxx_in)
    PxX_out = log(Pxx_out)
    Pxx_diff = Pxx_out - Pxx_in
    Pxx_diff_smoothed = cookb_signalsmooth.smooth(Pxx_diff.ravel(), window_len=51, window='flat')
    return freq, Pxx_in, Pxx_out, Pxx_diff, Pxx_diff_smoothed


def max_amplitude(freq, pxx) :
    bin_width = freq[1] - freq[0] # = Fs/n
    max_k = int(len(pxx) * 0.9)
    k = pxx[0:max_k].argmax()
    f_peak = freq[k]
    return f_peak

'''
def max_amp_approx(freq, pxx, window='hanning'):
    if window == 'hanning':
        P = 1.36
    bin_width = freq[1] - freq[0] # = Fs/N
    max_k = int(len(Pxx) * 0.9)
    k = Pxx[0:max_k].argmax()
    X_next = float(sqrt(10**(Pxx[k+1]/10)) )
    X = float(sqrt(10**(Pxx[k])))
    X_prev = float(sqrt(10**(Pxx[k-1]/10)))
    tau = P * (X_next - X_prev) / (X_prev + X + X_next)
    k_peak = k + tau
    f_peak = k_peak * bin_width
    return f_peak
'''

#find the max amplitude in a certain range
def max_amp_approx(freq,
                   Pxx,
                   min_freq=6000,
                   max_freq=12000,
                   window='hanning'):
    if window == 'hanning':
        P = 1.36
    binwidth = freq[1] - freq[0]
    max_k = int(max_freq / binwidth)
    min_k = int(min_freq / binwidth)
    k = Pxx[min_k:max_k].argmax() + min_k
    X_next = float(math.sqrt(10**(Pxx[k+1]/10)) )
    X = float(math.sqrt(10**(Pxx[k])))
    X_prev = float(math.sqrt(10**(Pxx[k-1]/10)))
    tau = P * (X_next - X_prev) / (X_prev + X + X_next)
    k_peak = k + tau
    f_peak = k_peak * binwidth
    return f_peak


def mean_shift(freq, Pxx, F_MIN, F_MAX, Fs, window_size):
    END_ERROR = 0.5
    stdev = 5
    win = scipy.signal.gaussian(window_size, stdev)
    bin_width = freq[1] - freq[0]
    idx_min = int(round(F_MIN / bin_width) )
    idx_max = int(round(F_MAX / bin_width) )
    half = (window_size -1)/2
    # Initial guess
    idx_peak = Pxx[idx_min:idx_max].argmax() + idx_min
    weighted_Pxx = (win * array(Pxx[idx_peak - half  : idx_peak + half]) )
    num = weighted_Pxx * arange(idx_peak-half, idx_peak+half+1)
    local_mean = num / weighted_Pxx
    while abs(local_mean - idx_peak) > END_ERROR :
        idx_peak = round(local_mean)
        weighted_Pxx = (win * array(Pxx[idx_peak - half  : idx_peak + half]) )
        num = weighted_Pxx * arange(idx_peak-half, idx_peak+half+1)
        local_mean = num / weighted_Pxx
    return Pxx[idx_peak]


def plot_graph(freq, Pxx_1, Pxx_2, Pxx_diff, Pxx_diff_smoothed):
    '''
    Given two frequency spectrum data, plot each of them and the difference
    '''
    def plot_psd(freq, pxx, title, fs=20):
        plt.figure()
        plt.plot(freq, pxx)
        plt.title(title, fontsize=fs)
        plt.xlabel('Frequency (Hz)', fontsize=fs)
        plt.ylabel('Power Spectral Density (dB/Hz)', fontsize=fs)
        plt.grid(True)
        plt.tick_params(axis='both', which='major', labelsize=20)
        plt.tick_params(axis='both', which='minor', labelsize=14)
    plot_psd(freq, Pxx_1, 'Reference Channel')
    plot_psd(freq, Pxx_2, 'Sensing Channel')
    plot_psd(freq, Pxx_diff, 'Difference')
    plot_psd(freq, Pxx_diff_smoothed, 'Smoothed Difference')
    print 'snr for diff: ', SNR(Pxx_diff)
    print 'snr for diff smoothed: ', SNR(Pxx_diff_smoothed)
    plt.show()


def plot_from_rawdata(data1, data2, rate, Ns=NS, overlap_ratio=OVERLAP_RATIO):
    '''
    Given two time-series data and the sampling rates, plot the frequency spectrums
    '''
    nOverlap = Ns * overlap_ratio
    (Pxx_in, freq) = mlab.psd(data1, NFFT=Ns, Fs=rate, 
                              detrend=mlab.detrend_mean, window=mlab.window_hanning,
                              noverlap=nOverlap, sides='onesided')
    (Pxx_out, freq) = mlab.psd(data2, NFFT=Ns, Fs=rate, 
                               detrend=mlab.detrend_mean, window=mlab.window_hanning,
                               noverlap=nOverlap, sides='onesided')
    Pxx_in = log(Pxx_in)
    Pxx_out = log(Pxx_out)
    Pxx_diff = Pxx_out - Pxx_in
    Pxx_diff_smoothed = cookb_signalsmooth.smooth(Pxx_diff.ravel(), window_len=51, window='flat'); 
    plot_graph(freq, Pxx_in, Pxx_out, Pxx_diff, Pxx_diff_smoothed)    


def plot_from_wavfile(file1, file2):
    '''
    Given two wav files, plot their frequency spectrums
    '''
    rate1, data1 = wavefile.read(file1)
    rate2, data2 = wavefile.read(file2)
    plot_from_rawdata(data1, data2, rate1)
  

def find_audio_device(name='Lexicon Omega: USB Audio (hw:1,0)'):
    '''
    Find the device ID used in PyAudio with the specific name
    '''
    import pyaudio
    p = pyaudio.PyAudio()
    for i in range(p.get_device_count()):
        if p.get_device_info_by_index(i)['name'] == name:
            p.terminate()
            return i
    p.terminate()
    return -1
