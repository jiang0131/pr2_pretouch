#!/usr/bin/env python
# A ROS node to subscribe to microphone sensor topic and estimate the peak frequency.
# Liang-Ting Jiang 02/06/2013
import rospy
import numpy as np
from matplotlib.pyplot import figure, plot, title, show
import math
from matplotlib import mlab
import cookb_signalsmooth
import kalman
from pr2_pretouch_msgs.msg import PretouchAcoustic, SoundRaw, NoiseVolume, SoundSpectrum

class MicSensor:

  def __init__(self):
    
    #(debug)plot spectrum flag
    self.PLOT = 0

    #Parameters
    self.BUFFER_SIZE = 1500 #This decide how many samples you want to wait before doing frequency estimation (default:1500)
    self.NSAMPLES = 1024 #This decide the FFT Ns when you do Welch's spectrum estimation (mlab.psd)
    self.RATE = 35738 #This is the sampling rate of our sound (each Ch) used to normalize the frequencies.
    self.SOUND_MESSAGE_SIZE = 254 #The number of sampels contained in one message
    self.OVERLAP_PERCENTAGE = 0.7 #The overlap ratio in Welch's Spectrum Estimation.
    self.N_OVERLAP = int (self.NSAMPLES * self.OVERLAP_PERCENTAGE) #Compute the number of samples overlapping.
  
    #energy detection flag
    self.ENERGY_SWITCH = 0
    self.ENERGY_THRESHOLD = 1000
    self.RESONANT_FREQ_LOWER_BOUND = 10

    #initialize class member variables
    self.resonant_freq = 0
    self.count = 0
    self.signal_1 = np.array([])
    self.signal_2 = np.array([])
    self.amp_sum_1 = 0
    self.amp_sum_2 = 0
    self.f_est = 0
    self.msg = PretouchAcoustic()
    self.msg.header.frame_id = "/r_gripper_r_finger_tip_link"
    self.msg_volume = NoiseVolume()
    self.msg_volume.volume = 0.5
    self.seq = 0

    #adaptive noise generator related
    self.VOLUME_GAIN = 0.01 #default: 0.005
    self.SNR_TARGET = 6.0  #default: 10
    self.snr_current = 0.0
    self.SNR_LIST_SIZE = 5  #default: 5
    self.snr_list = []

    #initialize kalman filter
    rospy.loginfo( "***** Initialize Kalman Filter *********")
    ndim = 1
    Sigma_x = 1e-5 #Process Vriance
    R = 0.02**2 #Measurement Variance = 0.01**2  #R = 0.02**2
    self.k = kalman.Kalman(ndim, Sigma_x, R)
    self.mu_init = np.array([2000])

    #ROS node, subscriber, and publishers
    rospy.init_node('pretouch_acoustic')
    side = rospy.get_param('~side', 'right')
    self.pub = rospy.Publisher('pretouch', PretouchAcoustic)
    self.pub_volume = rospy.Publisher('pretouch/noise_volume', NoiseVolume)
    self.sub  = rospy.Subscriber('sound/'+side, SoundRaw, self.sound_callback)
    rospy.spin()


  def sound_callback(self, msg):
    self.count += 1
    self.signal_1 = np.append(self.signal_1, np.array(msg.data[::2]))
    self.signal_2 = np.append(self.signal_2, np.array(msg.data[1::2]))

    #for debug only.....
    #print "shape of msg.data[::2] = " + str(array(msg.data[::2]).shape)
    #print "shape of signal_1 = " + str(self.signal_1.shape)

    if self.count == (self.BUFFER_SIZE / self.SOUND_MESSAGE_SIZE + 1):
      spectrum = self.compute_spectrum()
      #resonant_freq = self.max_amplitude_approximate(spectrum)
      self.resonant_freq = self.max_amplitude_approximate_ranged(spectrum, 6000, 12000)
      print "raw peak freq = " + str(self.resonant_freq)
      self.update_kalman(self.resonant_freq)
      self.publish()
      self.signal_1 = np.array([])
      self.signal_2 = np.array([])
      self.count = 0
    '''
    if len(self.snr_list) > self.SNR_LIST_SIZE:
      self.snr_current = np.mean(self.snr_list)
      print 'snr_current=', self.snr_current
      err = self.snr_current - self.SNR_TARGET
      self.msg_volume.volume -=  self.VOLUME_GAIN * err
      self.pub_volume.publish(self.msg_volume)
      self.snr_list = []
    '''

  def compute_spectrum(self):
    #print "Signal length = " + str((self.signal_1).shape)

    (Pxx_1, freq) = mlab.psd(self.signal_1, NFFT=self.NSAMPLES, Fs=self.RATE, detrend=mlab.detrend_mean, 
                             window=mlab.window_hanning, noverlap=self.N_OVERLAP, sides='onesided')
    (Pxx_2, freq) = mlab.psd(self.signal_2, NFFT=self.NSAMPLES, Fs=self.RATE, detrend=mlab.detrend_mean, 
                             window=mlab.window_hanning, noverlap=self.N_OVERLAP, sides='onesided')
    # Taking 10*log10()  Convert to dB and compute total energy
    #self.amp_sum_in = 0.0
    #self.amp_dum_out = 0.0
    Pxx_1 = np.array([10*math.log(p,10) for p in Pxx_1])
    Pxx_2 = np.array([10*math.log(p,10) for p in Pxx_2])    

    #amp_sum_sub = abs(amp_sum_out - amp_sum_in)
    #energy = amp_sum_sub #which energy source to use?
    #self.energy = self.amp_sum_1
    #print 'Pxx_out shape=' + str(Pxx_1.shape)
    # Smooth in Frequency Domain (Moving Average in time domain)
    temp = np.reshape(Pxx_2-Pxx_1, (self.NSAMPLES/2 + 1,))
    sub_smoothed = cookb_signalsmooth.smooth(temp, window_len=61, window='flat'); #61 or 51

    #compute the SNR
    self.snr_list.append(self.SNR(sub_smoothed))

    if self.PLOT == 1:
      self.plot_graph(freq, Pxx_1, Pxx_2, sub_smoothed)

    return sub_smoothed


  #find the max amplitude, and use the nearby amplitude to estimate the freq
  def max_amplitude_approximate(self, Pxx, window='hanning') :
    if window == 'hanning':
      P = 1.36
    bin_width = self.RATE / self.NSAMPLES
    max_k = int(len(Pxx) * 0.9)
    k = Pxx[0:max_k].argmax()
    X_next = float(math.sqrt(10**(Pxx[k+1]/10)) )
    X = float(math.sqrt(10**(Pxx[k])))
    X_prev = float(math.sqrt(10**(Pxx[k-1]/10)))
    tau = P * (X_next - X_prev) / (X_prev + X + X_next)
    k_peak = k + tau
    f_peak = k_peak * bin_width
    return f_peak


  #find the max amplitude in a certain range
  def max_amplitude_approximate_ranged(self, 
                                       Pxx,
                                       min_freq=6000, 
                                       max_freq=20000, 
                                       window='hanning') :
    if window == 'hanning':
      P = 1.36
    binwidth = self.RATE / self.NSAMPLES
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


  def update_kalman(self, resonant_freq):
    if (resonant_freq < self.RESONANT_FREQ_LOWER_BOUND):
      print '=== (JUMP TOO MUCH --- SHOULD BE NOISE) ==='
    elif (self.ENERGY_SWITCH and (self.energy > self.ENERGY_THRESHOLD) ):
      print '=== (TOO NOISY -- STOP TRACKING) ==='
    else:
      self.k.update(resonant_freq)
      self.f_est = self.k.mu_hat_est
    print 'Kalman filter estimation: ' , self.f_est, ' (Hz)'
    print ' '

  def publish(self):
    self.resonant_freq_prev = self.resonant_freq
    self.msg.header.seq = self.seq
    self.msg.header.stamp = rospy.Time.now()
    self.msg.frequency = self.f_est
    self.msg.frequency_raw = self.resonant_freq
    self.pub.publish(self.msg)
    self.seq += 1

  def plot_graph(self, freq, Pxx_1, Pxx_2, sub_smoothed):
    figure(1)
    plot(freq, Pxx_1)
    title('Ch1')
    figure(2)
    plot(freq, Pxx_2)
    title('Ch2')
    figure(3)
    plot(freq, Pxx_1-Pxx_2)
    title('sub')
    figure(4)
    plot(freq, sub_smoothed)
    title('smoothed')
    show()


  def SNR(self, pxx):
    return np.max(pxx) - np.mean(pxx)

if __name__ == "__main__":
    sensor = MicSensor()
    #sensor.sub()

