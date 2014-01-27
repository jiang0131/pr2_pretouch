#!/usr/bin/env python
import rospy
from pr2_pretouch_msgs.msg import NoiseVolume
import pygame
from std_srvs.srv import Empty, EmptyRequest

class Noise():

  def __init__(self, filename, init_volume=0.10):
    #rospy.wait_for_service('reset_adaptive_noise')
    #noise_proxy = rospy.ServiceProxy('reset_adaptive_noise', Empty)
    #srv = EmptyRequest()
    #resp = noise_proxy(srv)
    pygame.mixer.init()
    pygame.mixer.music.set_volume(init_volume)
    pygame.mixer.music.load(filename)

  def play(self):
    pygame.mixer.music.play(-1)
    rospy.spin()

  """
  def change(self, volume=1):
    pygame.mixer.music.stop()
    pygame.mixer.music.set_volume(volume)
    pygame.mixer.music.play()

  def onChangeVolume(self, msg):
    if msg.volume > 1.0:
      pygame.mixer.music.set_volume(1)
    elif msg.volume < 0:
      pygame.mixer.music.set_volume(0)
    else:
      pygame.mixer.music.set_volume(msg.volume)
  """

if __name__ == '__main__':
  rospy.init_node('anaptive_noise', anonymous = True)
  volume = rospy.get_param('~volume', 0.20)  
  sound_file = rospy.get_param('~sound_file', '../sound/whitenoise_band_limited.wav')  
  n = Noise(sound_file, volume)
  n.play()
