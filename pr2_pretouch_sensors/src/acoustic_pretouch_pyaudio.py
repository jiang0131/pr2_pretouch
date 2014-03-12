#!/usr/bin/env python
'''
A ROS node for seashell pretouch sensing using PyAudio
It get audio stream data from a audio interface through PyAudio,
and compute the resonant frequency of the cavity
PretouchAcoustic messages are puiblished to a topic
@Liang-Ting Jiang 2014-03-12
'''
import rospy
import pyaudio
import struct
import numpy as np
import math
from scipy import poly1d
import kalman
from pr2_pretouch_msgs.msg import PretouchAcoustic
import helper

####################################################
####  Frequency-Distance Relation Polyfit model ####
####  A model fitted from real collected data   ####
####################################################
f = poly1d([ -8.49877379e-14,   3.10678600e-09,  -4.14966418e-05,
             2.42262153e-01,  -5.23219170e+02])

####################################################
####### Parameters #################################
####################################################
# PyAudio input stream parameters
RECORD_SECONDS = 0.05
CHUNK = 2200
FORMAT = pyaudio.paInt16
CHANNELS = 2
RATE = 44100

####################################################
########### Initialize ROS Node    r ###############
####################################################
rospy.init_node('pretouch_acoustic_sensing_node')
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
k = kalman.Kalman(ndim, Sigma_x, R)
mu_init = np.array([2000])

####################################################
########### Getting Sound ##########################
####################################################
# Find the actual device ID used by PyAudio
device_id = helper.find_audio_device(name='Lexicon Omega: USB Audio (hw:1,0)')
print device_id
# Initialize Audio Input Streaming
p = pyaudio.PyAudio()
stream = p.open(format = FORMAT,
                channels = CHANNELS,
                rate = RATE,
                input = True,
                input_device_index = device_id,
                frames_per_buffer = CHUNK)

####################################################
################ Main Loop #########################
####################################################
while not rospy.is_shutdown():
    
    ####################################################
    ########### Getting Spectrum #######################
    ####################################################
    n_reads = int(RATE * RECORD_SECONDS / CHUNK)
    data = ''.join([stream.read(CHUNK) for i in range(n_reads)])
    if FORMAT == pyaudio.paInt16:
        signal = struct.unpack("<%uh" % (len(data) / 2), data)
    elif FORMAT == pyaudio.paInt32:
        signal = struct.unpack("<%ul" % (len(data) / 4), data)
    else:
        raise TypeError("Not supported PyAudio format")
    signal_in = signal[0::2]
    signal_out = signal[1::2]

    ####################################################
    ########### Getting Spectrum #######################
    ####################################################
    freq, Pxx_in, Pxx_out, Pxx_diff, Pxx_diff_smoothed = 
                            helper.get_spectrums(signal_in, signal_out, RATE)

    ####################################################
    ########### Peak Finding  ##########################
    ####################################################
    resonant_freq = helper.max_amp_approx_ranged(freq, Pxx_diff_smoothed)

    ####################################################
    ########### Update Kalman Filter ###################
    ####################################################
    k.update(resonant_freq)
    f_est = k.mu_hat_est

    # map the estimated frequency back to distance
    distance = f(resonant_freq)
 
    # ROS message   
    msg.header.seq = seq
    msg.header.stamp = rospy.Time.now()
    msg.frequency = f_est
    msg.frequency_raw = resonant_freq
    msg.distance = distance
    pub.publish(msg)
    seq += 1

    # showing data
    print ' '
    print 'raw frequency: ', resonant_freq, ' (Hz)'
    print 'Kalman filter estimation: ' , f_est, ' (Hz)'
    print "Distance: ", distance, ' (mm)'

p.terminate()
