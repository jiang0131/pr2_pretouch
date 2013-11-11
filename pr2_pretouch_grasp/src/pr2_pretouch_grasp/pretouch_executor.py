#!/usr/bin/python
import roslib
import rospy
import numpy as np
from geometry_msgs.msg import Pose, PoseStamped
from uw_pr2_gripper_grasp_planner_cluster.convert_functions import *
import uw_pr2_gripper_grasp_planner_cluster.draw_functions as draw_functions
import actionlib
#from arm_navigation_msgs.msg import MoveArmAction, MoveArmGoal, SimplePoseConstraint, JointConstraint
#from kinematics_msgs.srv import GetKinematicSolverInfo, GetKinematicSolverInfoRequest
#from pr2_tasks.arm_tasks import ArmTasks
#from pr2_python.arm_mover import ArmMover
#from pr2_python.controller_manager_client import ControllerManagerClient
#from pr2_python.gripper import Gripper
from pr2_pretouch_msgs.msg import PretouchAcoustic
from geometry_msgs.msg import TwistStamped
#from object_manipulation_msgs.msg import PickupAction, PickupGoal
from pr2_pretouch_msgs.srv import AddPoint, AddPointRequest

class PretouchExecutor:

  def __init__(self, tf_listener = None, tf_broadcaster = None):

    #the pretouch sensor frame
    self.pretouch_sensor_frame_id = '/r_gripper_l_finger_tip_pretouch_frame'

    #init a TF transform listener
    if tf_listener == None:
      self.tf_listener = tf.TransformListener()
    else:
      self.tf_listener = tf_listener

    #init a TF transform broadcaster for the object frame
    if tf_broadcaster == None:
      self.tf_broadcaster = tf.TransformBroadcaster()
    else:
      self.tf_broadcaster = tf_broadcaster
    
    #Gripper client and open the gripper
    rospy.loginfo('open the right gripper')
    self.gripper = Gripper('right_arm')
    self.gripper.open()

    #controller_magager_client
    self.cm_client = ControllerManagerClient()
    print 'The current controllers: ', self.cm_client.list_controllers()
    #TwistStamped command publisher for jinv controller
    self.twist_pub = rospy.Publisher('/r_cart/command_twist', TwistStamped)
    self.count = 0

    #init ArmMover and ArmTasks
    self.arm_name = 'right_arm'
    self.arm_tasks = ArmTasks()
    self.arm_tasks.move_arm_to_side(self.arm_name)
    self.arm_mover = ArmMover()

    #pickup action server
    self.pickup_client = actionlib.SimpleActionClient('/object_manipulator/object_manipulator_pickup', PickupAction)
    rospy.loginfo("waiting for PickupAction Server")
    self.pickup_client.wait_for_server()
    rospy.loginfo("PickupAction Server Connected")

    #service client to /add_point service
    self.add_point_client = rospy.ServiceProxy('add_point', AddPoint)

    #draw_functions object for drawing stuff in rviz
    self.draw_functions = draw_functions.DrawFunctions('pretouch_executor_markers')

    #the transform from wrist_frame to the center of the fingers
    self.gripper_to_r_finger = np.eye(4)
    self.gripper_to_r_finger[0][3] = 0.1615 #x-translation from wrist_frame (0.1615)
    self.gripper_to_r_finger[1][3] = -0.0400 #y-translation from wrist_frame (0.0425)    
    self.gripper_to_l_finger = np.eye(4)
    self.gripper_to_l_finger[0][3] = 0.1615 #x-translation from wrist_frame (0.1615)
    self.gripper_to_l_finger[1][3] = 0.0400 #y-translation from wrist_frame (-0.0425)

  def pretouch_callback(self, msg):
    #self.count += 1
    #print 'count = ', self.count
    print 'frequency =', msg.frequency
    print 'raw_freq  =', msg.frequency_raw
    #if self.count == 100:
    #if msg.frequency < 8400 and self.count == 3: #detected
    
    #monitor the probe time to stop probing to the air
    time_limit = rospy.Duration(19.0)
    time_past = rospy.Time.now() - self.start_time
    print time_limit
    print time_past
    print time_past > time_limit
    if time_past > time_limit:
      self.detected = True
      self.count = 0
      rospy.loginfo("Nothing is here!!!")
      rospy.loginfo("stop probe.....")
      self.pretouch_sub.unregister()
      #self.twist_pub.unregister()
      #switch_controllers(start_controllers=[], stop_controllers=[])
      self.cm_client.switch_controllers(['r_arm_controller'], ['r_cart'])
      print 'controller switched back to: ', self.cm_client.list_controllers()
    
    elif msg.frequency < 8650: #detected
      self.detected = True
      self.count = 0
      rospy.loginfo("object detected by pretouch probe!!!")
      rospy.loginfo("stop probe.....")
      self.pretouch_sub.unregister()
      #self.twist_pub.unregister()
      #switch_controllers(start_controllers=[], stop_controllers=[])
      self.cm_client.switch_controllers(['r_arm_controller'], ['r_cart'])
      print 'controller switched back to: ', self.cm_client.list_controllers()
      #draw the marker
      self.draw_pretouch_marker()
      #update free octomap cells
      pose_stamped = PoseStamped()
      pose_stamped.header.frame_id = self.pretouch_sensor_frame_id
      pose_stamped.header.stamp = rospy.Time.now()
      pose_stamped.pose.position.x = 0
      pose_stamped.pose.position.y = 0
      pose_stamped.pose.position.z = 0
      self.tf_listener.waitForTransform(pose_stamped.header.frame_id, 'base_link', pose_stamped.header.stamp, rospy.Duration(1.0)) 
      print 'transform success...'
      pose_stamped = self.tf_listener.transformPose('base_link', pose_stamped)
      print pose_stamped
      req = AddPointRequest()
      req.pose_stamped = pose_stamped
      req.decay_rate = 20.0 #default: 40.0
      req.hit = True
      resp = self.add_point_client(req)

    else: #constant velocity motion
      self.count += 1
      print 'count= ', self.count
      msg = TwistStamped()
      msg.header.frame_id = 'r_wrist_roll_link'
      msg.header.stamp = rospy.Time.now()
      msg.twist.linear.x = 0.003
      #rospy.loginfo(msg)
      self.twist_pub.publish(msg)
    
      if self.count > 10:
        #update free octomap cells
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = self.pretouch_sensor_frame_id
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.pose.position.x = 0
        pose_stamped.pose.position.y = 0
        pose_stamped.pose.position.z = 0
        self.tf_listener.waitForTransform(pose_stamped.header.frame_id, 'base_link', pose_stamped.header.stamp, rospy.Duration(1.0)) 
        print 'transform success...'
        pose_stamped = self.tf_listener.transformPose('base_link', pose_stamped)
        print pose_stamped
        req = AddPointRequest()
        req.pose_stamped = pose_stamped
        req.decay_rate = 40.0
        req.hit = False
        resp = self.add_point_client(req)
        self.count = 0


  def pretouch_probe(self):

    #reset the self.detected flag
    self.detected = False
    self.start_time = rospy.Time.now()
    #switch_controllers(start_controllers=[], stop_controllers=[])
    rospy.loginfo("start probe.....")
    self.cm_client.switch_controllers(['r_cart'], ['r_arm_controller'])
    print 'controller switched to: ', self.cm_client.list_controllers()

    #pretouch subscriber
    self.pretouch_sub = rospy.Subscriber("/pretouch", PretouchAcoustic, self.pretouch_callback)
    #self.twist_pub = rospy.Publisher('/r_cart/command_twist', TwistStamped)
    self.count = 0

    #blocking
    #wait until the object detected (self.flag)
    while not self.detected:
      rospy.sleep(0.1)

    return True

  def draw_pretouch_marker(self):
    print 'drawing actual pretouch point......'
    mat = np.matrix(np.identity(4))
    self.draw_functions.draw_rviz_sphere(mat, 0.0035, frame = self.pretouch_sensor_frame_id, ns = 'actual_pretouch_points', id = 0, duration = 60., color = [1,0.4,0.7], opaque = 0.9, frame_locked = False)

  def move_to_joint_goal(self, positions):
    #joint_position = [-0.817, 0.001, -1.253, -0.892, 60.854, -0.250, 3.338]
    #joint_position=[0.19352680514353882, 1.122675976318801, -0.84954760489618752, -1.8022948875080473, 1.6990523534757012, -1.9945748402742813, 1.0094339881452736]
    # get a joint state message already configured for this arm                                                                                               
    js = self.arm_mover.get_joint_state(self.arm_name)
    rospy.sleep(2.0)
    # set desired joint positions                                                                                                                             
    js.position = positions
    print 'Moving to %s' % (str(positions))

    # send out the command                                                                                                                                    
    handle = self.arm_mover.move_to_goal(self.arm_name, js)
    if not handle.reached_goal():
      print handle.get_errors()
      return False
    else:
      return True

