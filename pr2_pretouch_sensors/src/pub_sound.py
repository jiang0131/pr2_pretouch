#!/usr/bin/env python
# A node to convert ByteMultiArray (published by pr2_ethercat realtime control loop) to SoundRaw data.
# @Liang-Ting Jiang (jianglt@uw.edu)
import rospy
from std_msgs.msg import ByteMultiArray
from pr2_pretouch_msgs.msg import SoundRaw

ts = (0, 0, 0, 0)
seq = 0

rospy.init_node("sound_publisher", anonymous=True)

side = rospy.get_param('~side', 'right')
rospy.loginfo("Started the sound_publisher for the %s gripper", side)
if side == 'right':
  sub_topic = 'raw_pressure/r_gripper_motor'
elif side == 'left':
  sub_topic = 'raw_pressure/l_gripper_motor'
else:
  print side + ' is not a valid side parameter (only right/left supported)'
  raise

pub_topic = 'sound/' + side
pub = rospy.Publisher(pub_topic, SoundRaw)

def rawPressureCallback(msg):
  global seq, ts
  #only publish if the timestamp is different from the last message
  if msg.data[:4] != ts:
    seq += 1
    ts = msg.data[:4]
    sr = SoundRaw()
    sr.header.seq = seq
    sr.header.stamp = rospy.Time.now()
    sr.data = msg.data[4:-1]
    pub.publish(sr)
    #print msg.data[4:]

sub = rospy.Subscriber(sub_topic, ByteMultiArray, rawPressureCallback)
rospy.spin()

