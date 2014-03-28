#!/usr/bin/env python
# A node to convert ByteMultiArray (published by the pr2_ethercat realtime control loop) to OpticalBeams sensor data
# @Liang-Ting Jiang (jianglt@uw.edu)

import rospy
from std_msgs.msg import ByteMultiArray
from pr2_pretouch_sensor_optical.msg import OpticalBeams

THRESHOLD = 800
ts = [0, 0, 0, 0]
seq = 0
beams = OpticalBeams()

rospy.init_node("optical_publisher", anonymous=True)
side = rospy.get_param('~side', 'right')
rospy.loginfo("Started the optical_publisher for the %s gripper", side)
if side == 'right':
  sub_topic = 'raw_pressure/r_gripper_motor'
elif side == 'left':
  sub_topic = 'raw_pressure/l_gripper_motor'
else:
  print side + ' is not a valid side parameter (only right/left supported)'
  raise

pub_topic = 'optical/' + side
pub = rospy.Publisher(pub_topic, OpticalBeams)

def rawPressureCallback(msg):
  global seq, ts, beams
  #only publish if the timestamp is different from the last message
  #this is for removing the duplicated messages
  if msg.data[:4] != ts:
    seq += 1
    ts = msg.data[:4]
    beams.header.seq = seq
    beams.header.stamp = rospy.Time.now()
    beams.data = [(256+val)<<2 if val<0 else val<<2 for val in msg.data[4:8]]
    beams.broken = [True if d > THRESHOLD else False for d in beams.data]
    pub.publish(beams)

sub = rospy.Subscriber(sub_topic, ByteMultiArray, rawPressureCallback)
rospy.spin()

