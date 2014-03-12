#!/usr/bin/env python
# A script to use the two microphones on the right gripper to record sound.
# Useful for verifying the microphones are working properly.
# @Liang-Ting Jiang (jianglt@uw.edu)
import rospy
from pylab import array, append
from pr2_pretouch_msgs.msg import PretouchAcoustic, SoundRaw
import sys
import struct
import helper

####################################################
####### Wave File Parameters #######################
####################################################

DURATION = int(raw_input('How many seconds to analyze? ....... ')) #in seconds; How long you want to record?
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
    signal_1 = data[::2]
    signal_2 = data[1::2]
    #signal_1 = [struct.unpack('B', struct.pack('b',d))[0] for d in data[::2]]
    #signal_2 = [struct.unpack('B', struct.pack('b',d))[0] for d in data[1::2]]

    print len(signal_1)
    print signal_1
    print "number of samples(frames): " +  str( len(signal_1))
    print "SAMPLING RATE: " + str(RATE) + "(Hz)"
    print "DURATION: " + str(DURATION) + "(s)"

    helper.plot_from_rawdata(signal_1, signal_2, RATE)

    rospy.signal_shutdown("Recording finished")  


#subscribe to the sound topic and spin forever
sub = rospy.Subscriber('sound/right', SoundRaw, callback)
rospy.spin()
