#!/usr/bin/python
import numpy as np
import roslib
import rospy
import tf
from tf.transformations import translation_matrix
from geometry_msgs.msg import Pose, PoseStamped
from uw_pr2_gripper_grasp_planner_cluster.convert_functions import *
import uw_pr2_gripper_grasp_planner_cluster.draw_functions as draw_functions
import actionlib
#from arm_navigation_msgs.msg import MoveArmAction, MoveArmGoal, SimplePoseConstraint, JointConstraint
#from kinematics_msgs.srv import GetKinematicSolverInfo, GetKinematicSolverInfoRequest
#from pr2_tasks.arm_tasks import ArmTasks
#from pr2_python.arm_mover import ArmMover
from uw_pr2_python.controller_manager_client import ControllerManagerClient
from uw_pr2_python.gripper import Gripper
from pr2_pretouch_msgs.msg import PretouchAcoustic
from geometry_msgs.msg import TwistStamped
#from object_manipulation_msgs.msg import PickupAction, PickupGoal
from pr2_pretouch_msgs.srv import AddPoint, AddPointRequest
from moveit_commander.move_group import MoveGroupCommander
from sensor_msgs.msg import JointState

class PretouchExecutor:

  def __init__(self, side='r', tf_listener = None, tf_broadcaster = None, ctrl_mng=None, moveit_cmdr=None):

    if side == "right" or side == "r":
      self.arm = "right_arm"
      self.side = "r"
    elif side == 'left' or side == 'l':
      self.arm = "left_arm"
      self.side = "l"
    else:
      rospy.logerr("Side " + side + " is not supported")
      raise

    self.tf_listener = tf.TransformListener() if not tf_listener else tf_listener
    #init a TF transform broadcaster for the object frame
    self.tf_broadcaster = tf.TransformBroadcaster() if not tf_broadcaster else tf_broadcaster
    self.ctrl_mng = ControllerManagerClient() if not ctrl_mng else ctrl_mng

    #the pretouch sensor frame
    self.pretouch_sensor_frame_id = '/r_gripper_l_finger_tip_pretouch_frame'

    #Gripper client and open the gripper
    rospy.loginfo('open the ' + side + '_gripper')
    self.gripper = Gripper(self.arm)
    self.gripper.open()    

    #controller_magager_client
    self.ctrl_mng = ControllerManagerClient() if not ctrl_mng else ctrl_mng
    self.ctrl_mng.switch_controllers([self.side+'_arm_controller'], [self.side+'_cart'])
    self.count = 0
    #PoseStamped command publisher for jt controller
    self.pose_pub = rospy.Publisher('/r_cart/command_pose', PoseStamped)
    self.eef_frame = self.side + "_wrist_roll_link"
    self.reference_frame = "base_link"

    # MoveIt! Commander
    self.moveit_cmd = MoveGroupCommander(self.arm) if not moveit_cmdr else moveit_cmdr
    self.moveit_cmd.set_pose_reference_frame(self.reference_frame)
    self.move_arm_to_side() # Move the arm to the sidea
    self.step_size = 0.0002
    self.move_step_mat = np.matrix(translation_matrix(np.array([self.step_size, 0.0, 0.0])))

    #pickup action server
    #self.pickup_client = actionlib.SimpleActionClient('/object_manipulator/object_manipulator_pickup', PickupAction)
    #rospy.loginfo("waiting for PickupAction Server")
    #self.pickup_client.wait_for_server()
    #rospy.loginfo("PickupAction Server Connected")

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
      self.ctrl_mng.switch_controllers([self.side+'_arm_controller'], [self.side+'_cart'])
      print 'controller switched back to: ', self.ctrl_mng.list_controllers()
    
    elif msg.frequency < 8650: #detected
      self.detected = True
      self.count = 0
      rospy.loginfo("object detected by pretouch probe!!!")
      rospy.loginfo("stop probe.....")
      self.pretouch_sub.unregister()
      self.ctrl_mng.switch_controllers([self.side+'_arm_controller'], [self.side+'_cart'])
      print 'controller switched back to: ', self.ctrl_mng.list_controllers()
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

      '''
      msg = TwistStamped()
      msg.header.frame_id = 'r_wrist_roll_link'
      msg.header.stamp = rospy.Time.now()
      msg.twist.linear.x = 0.003
      #rospy.loginfo(msg)
      self.twist_pub.publish(msg)
      '''    

      # Move a step_size
      #self.move_step_mat = np.matrix(translation_matrix(np.array([step_size, 0.0, 0.0])))
      self.current_mat = self.current_mat * self.move_step_mat
      self.pose_pub.publish(stamp_pose(mat_to_pose(self.current_mat), self.reference_frame))
      #rospy.sleep(0.02)    

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
    self.ctrl_mng.switch_controllers([self.side+'_cart'], [self.side+'_arm_controller'])
    print 'controller switched to: ', self.ctrl_mng.list_controllers()

    '''
    # The current pose
    self.tf_listener.waitForTransform(self.eef_frame, self.reference_frame, pose_stamped.header.stamp, rospy.Duration(1.0)) 
    pose = self.tf_listener.lookup
    self.current_mat = pose_to_mat()
    '''

    # Start subscribing to pretouch sensor readings
    self.pretouch_sub = rospy.Subscriber("/pretouch", PretouchAcoustic, self.pretouch_callback)
    self.count = 0

    #blocking
    #wait until the object detected (self.detected)
    while not self.detected:
      rospy.sleep(0.1)

    return True

  def draw_pretouch_marker(self):
    print 'drawing actual pretouch point......'
    mat = np.matrix(np.identity(4))
    self.draw_functions.draw_rviz_sphere(mat, 0.0035, frame = self.pretouch_sensor_frame_id, ns = 'actual_pretouch_points', id = 0, duration = 60., color = [1,0.4,0.7], opaque = 0.9, frame_locked = False)


  def move_to_pose_goal(self, pose):
    return self.moveit_cmd.go(pose, wait=True)
    

  def move_to_joint_goal(self, positions):

    self.moveit_cmd.go(joints = positions, wait=True)

    #joint_position = [-0.817, 0.001, -1.253, -0.892, 60.854, -0.250, 3.338]
    #joint_position=[0.19352680514353882, 1.122675976318801, -0.84954760489618752, -1.8022948875080473, 1.6990523534757012, -1.9945748402742813, 1.0094339881452736]
    # get a joint state message already configured for this arm                                                                                               
    js = self.arm_mover.get_joint_state(self.arm_name)
    rospy.sleep(2.0)
    # set desired joint positions                                                                                                                             
    js.position = positions
    print 'Moving to %s' % (str(positions))

    # send out the command
    self.moveit_cmd.go(joints = positions, wait=True)
    handle = self.arm_mover.move_to_goal(self.arm_name, js)
    if not handle.reached_goal():
      print handle.get_errors()
      return False
    else:
      return True

  def move_arm_to_side(self):
    pose = Pose()
    pose.position.x = 0.258
    pose.position.y = -0.614
    pose.position.z = 1.017
    pose.orientation.w = 0.720
    pose.orientation.x = 0.059
    pose.orientation.y = 0.153
    pose.orientation.z = 0.674
    return self.move_to_pose_goal(pose)
