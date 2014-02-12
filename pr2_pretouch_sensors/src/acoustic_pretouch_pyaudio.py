#!/usr/bin/env python
#import roslib; roslib.load_manifest('pr2_pretouch_acoustic')
import rospy
import pyaudio
import struct
import numpy
import math
import time
from scipy.signal import *
from scipy import poly1d
from matplotlib import pyplot as plt
from matplotlib import mlab
import cookb_signalsmooth
import kalman
from pylab import *
from collections import deque
from pr2_pretouch_msgs.msg import PretouchAcoustic
import sys

####################################################
####  Frequency-Distance Relation Polyfit model ####
#f = poly1d([ 1.38978222e-11, -1.06118063e-07,  3.02635463e-04, -3.80601653e-01,   1.77812735e+02])
f = poly1d([ -8.49877379e-14,   3.10678600e-09,  -4.14966418e-05,
         2.42262153e-01,  -5.23219170e+02])
####################################################

####################################################
####### Parameters #################################
####################################################

# Recording parameters
RECORD_SECONDS = 0.05 #0.05
#RECORD_SECONDS = 0.04
chunk = 2200
#chunk = 1900
FORMAT = pyaudio.paInt16
CHANNELS = 2
RATE = 44100
#RATE = 48000

ENERGY_SWITCH = True
ENERGY_THRESHOLD = 400000

# Spectrum Estimation (WOSA) parameters
Ns = 1024
Overlap_Percent = 0.7
nOverlap = Ns * 0.7

####################################################
####### Peak Detection Algorithm ###################
####################################################
def max_amplitude(freq, Pxx) :
    bin_width = freq[1] - freq[0] # = Fs/N
    max_k = int(len(Pxx) * 0.9)
    k = Pxx[0:max_k].argmax()
    f_peak = freq[k]
    return f_peak

def max_amplitude_approximate(freq, Pxx, window='hanning') :
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

#find the max amplitude in a certain range
def max_amplitude_approximate_ranged(Pxx,
                                     min_freq=6000,
                                     max_freq=12000,
                                     window='hanning') :
    if window == 'hanning':
      P = 1.36
    binwidth = RATE / Ns
    max_k = int(max_freq / binwidth)
    min_k = int(min_freq / binwidth)
    k = Pxx[min_k:max_k].argmax() + min_k
    #for debug only...
    #print "min_k = " + str(min_k)
    #print "max_k = " + str(max_k)
    #print "k = " + str(k)
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

####################################################
########### Initialize ROS Node    r ###############
####################################################
rospy.init_node('pretouch_acoustic')
pub = rospy.Publisher('pretouch', PretouchAcoustic)
msg = PretouchAcoustic()
msg.header.frame_id = "/r_gripper_r_finger_tip_link"
seq = 0

####################################################
########### Initialize Kalman Filter ###############
####################################################
print "***** Initialize Kalman Filter ***********"
ndim = 1
Sigma_x = 1e-5 #Process Vriance
R = 0.01**2 #Measurement Variance = 0.01**2
#R = 0.02**2
k = kalman.Kalman(ndim, Sigma_x, R)
mu_init = array([2000])

####################################################
########### Getting Sound ##########################
### 	    input_device_index = 9       ###########
####################################################
# Initialize Audio Input Streaming
p = pyaudio.PyAudio()
stream = p.open(
		format = FORMAT,
                channels = CHANNELS,
                rate = RATE,
                input = True,
                input_device_index = 4,
                frames_per_buffer = chunk)

"""
####################################################
########### Initialize Graph #######################
####################################################
win_time = 10 #(second)
win_size = int(win_time / RECORD_SECONDS)
ion()
a = numpy.zeros(win_size)*10000
d = deque(a)
d_amp = deque(a)
x = numpy.arange(0, win_size, 1) * RECORD_SECONDS

fig = plt.figure()
ax1 = fig.add_subplot(211)
line1, = ax1.plot(x, d, 'b', linewidth=3)
ax1.axis( [0,10,4000,15000] )
ax1.set_ylabel('Resonant frequency (Hz)')

ax2 = fig.add_subplot(212, sharex=ax1)
line2, = ax2.plot(x, d_amp, 'g', linewidth=3 )
ax2.axis( [0,10, 0, 1000000])
ax2.set_xlabel('time (s)')
ax2.set_ylabel('Total Energy (V^2)')
"""

####################################################
########Main Loop #######################
####################################################
while not rospy.is_shutdown():
    all = []
    for i in range(0, int(RATE / chunk * RECORD_SECONDS)):
        #read() only wait until the entire buffer (chunk) is filled and then return
        data = stream.read(chunk)
        all.append(data)
    data = ''.join(all)

    # Convert string to value
    if FORMAT == pyaudio.paInt16:
        signal = struct.unpack("<%uh" % (len(data) / 2), data)
    elif FORMAT == pyaudio.paInt32:
        signal = struct.unpack("<%ul" % (len(data) / 4), data)

    # Convert 2 channles to numpy arrays
    if CHANNELS== 2:
        signal_in = array (list (signal[0::2]))
        signal_out = array (list (signal[1::2]))
    else:
        signal_out = array (signal)
        signal_in = signal_out


####################################################
########### Getting Spectrum #######################
####################################################
    #print signal_out.shape
    #print signal_out
    (Pxx_in, freq) = mlab.psd(signal_in, NFFT=Ns, Fs=RATE, detrend=mlab.detrend_mean, window=mlab.window_hanning, noverlap=nOverlap, sides='onesided')
    (Pxx_out, freq) = mlab.psd(signal_out, NFFT=Ns, Fs=RATE, detrend=mlab.detrend_mean, window=mlab.window_hanning, noverlap=nOverlap, sides='onesided')

    # Taking 10*log10()  Convert to dB
    # also compute total energy
    amp_sum_in = 0.0
    amp_sum_out = 0.0
    for x in range(len(Pxx_in)):
        amp_sum_in += Pxx_in[x]
        Pxx_in[x] = 10*math.log(Pxx_in[x], 10)
    for x in range(len(Pxx_out)):
        amp_sum_out += Pxx_out[x]
        Pxx_out[x] = 10*math.log(Pxx_out[x], 10)
    #amp_sum_sub = abs(amp_sum_out - amp_sum_in)
    #energy = amp_sum_sub #which energy source to use?
    energy = amp_sum_out
    #print 'Pxx_out shape=' 
    #print  Pxx_out.shape
    # Smooth in Frequency Domain (Moving Average)
    sub_smoothed = cookb_signalsmooth.smooth(Pxx_out-Pxx_in, window_len=51, window='flat');

    if len(sys.argv) > 1:
      if sys.argv[1] == 'plot':
        # Plot Figure for verification
        figure(5)
        plot(freq, Pxx_in)
        title('in')
        figure(6)
        plot(freq, Pxx_out)
        title('out')
        figure(7)
        plot(freq, Pxx_out-Pxx_in)
        title('sub')
        figure(8)
        plot(freq, sub_smoothed)
        title('smoothed')
        show()
    
####################################################
########### Peak Finding  ##########################
####################################################
    #resonant_freq = freq[sub_smoothed.argmax()]
    #resonant_freq = max_amplitude_approximate(freq, sub_smoothed)
    resonant_freq = max_amplitude_approximate_ranged(sub_smoothed)

####################################################
########### Update Kalman Filter ###################
####################################################

    #Some bad conditions
    # JUMP = (resonant_freq - f_est_prev > 4000)

    #if (seq>5) and (abs(resonant_freq - f_est) > 4000):
    '''
    if (resonant_freq < 10):
        print '=== (JUMP TOO MUCH --- SHOULD BE NOISE) ===' 
    elif (ENERGY_SWITCH and (energy > ENERGY_THRESHOLD) ):
        print '=== (TOO NOISY -- STOP TRACKING) ==='
    else:
        k.update(resonant_freq)
        f_est = k.mu_hat_est
    '''
    k.update(resonant_freq)
    f_est = k.mu_hat_est
    distance = f(resonant_freq)
    print ' '
    print 'raw frequency: ', resonant_freq, ' (Hz)'
    print 'Kalman filter estimation: ' , f_est, ' (Hz)'
    print "Distance: ", distance, ' (mm)'
    #print ' Original frequency: ', resonant_freq, ' (Hz)'
    
    """
    #update plots
    d.popleft()
    d.append(f_est)

    d_amp.popleft()
    d_amp.append(energy)
    line1.set_ydata(d)
    line2.set_ydata(d_amp)
    draw()
    """

    #map to distance
    #dist = f(k.mu_hat_est)
    #print 'Distance: ', dist, ' (cm)'

    #f_est_prev = f_est
    resonant_freq_prev = resonant_freq

    msg.header.seq = seq
    msg.header.stamp = rospy.Time.now()
    msg.frequency = f_est
    msg.frequency_raw = resonant_freq
    msg.distance = distance
    pub.publish(msg)
    #rospy.loginfo(msg)

    seq = seq + 1

p.terminate()
