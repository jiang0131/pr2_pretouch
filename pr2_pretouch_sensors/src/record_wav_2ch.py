#!/usr/bin/env python
# A script to use the two microphones on the right gripper to record sound.
# Useful for verifying the microphones are working properly.
# @Liang-Ting Jiang (jianglt@uw.edu)
import rospy
from pylab import array, append
from pr2_pretouch_msgs.msg import PretouchAcoustic, SoundRaw
import sys
import wave
import struct

####################################################
####### Wave File Parameters #######################
####################################################

FILENAME_1 = 'record_ch1.wav'
FILENAME_2 = 'record_ch2.wav'
DURATION = int(raw_input('How many seconds to record? ....... ')) #in seconds; How long you want to record?
#RATE = 43485 #measure it using rostopic hz /sound, and then times 508 - bytes/s
#RATE = 71450 #for the 71429 Hz case
RATE = 35738

N_SAMPLES = RATE * DURATION
N_CYCLES = N_SAMPLES / 254 #number of rostopic publishes in DURATION
#initialize data array
data = array([])
count = 0

####################################################
########### Initialize ROS Node    r ###############
####################################################
rospy.init_node('sound_recorder')
rospy.loginfo('start recording for %d seconds', DURATION)

####################################################
######## callback function   #######################
####################################################
def callback(msg):

  global record
  global data
  global count

  data = append(data, msg.data)
  #print len(msg.data)
  count += 1

  if count == N_CYCLES:
    #print data
    #print data.shape
    signal_1 = []
    signal_2 = []
    data_1 = data[::2]
    data_2 = data[1::2]
    #print "data_1=", data_1
    #print "data_2=", data_2
    for i in data_1:
      packed_value = struct.pack('B', i)
      signal_1.append(packed_value)
    for i in data_2:
      packed_value = struct.pack('B', i)
      signal_2.append(packed_value)
      #print hex(ord(packed_value))
    value_str_1 = ''.join(signal_1)
    value_str_2 = ''.join(signal_2)

    record_1 = wave.open(FILENAME_1, 'w')
    record_1.setparams((1, 1, RATE, len(signal_1), 'NONE', 'not compressed'))
    record_1.writeframesraw(value_str_1)
    record_2 = wave.open(FILENAME_2, 'w')
    record_2.setparams((1, 1, RATE, len(signal_2), 'NONE', 'not compressed'))
    record_2.writeframesraw(value_str_2)

    print "number of samples(frames): " +  str( len(signal_1))
    print "SAMPLING RATE: " + str(RATE) + "(Hz)"
    print "DURATION: " + str(DURATION) + "(s)"
    record_1.close()
    record_2.close()
    print "WAV FILENAME 1: " + FILENAME_1
    print "WAV FILENAME 2: " + FILENAME_2
    #print "Ctrl-C to close the program"
    rospy.signal_shutdown("Recording finished")  

#subscribe to the sound topic and spin forever
sub = rospy.Subscriber('sound/right', SoundRaw, callback)
rospy.spin()
