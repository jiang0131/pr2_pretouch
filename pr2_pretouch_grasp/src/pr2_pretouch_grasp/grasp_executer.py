import rospy
import tf
import tf2_ros
from tf.transformations import translation_matrix, quaternion_matrix
from geometry_msgs.msg import PoseStamped
from uw_pr2_python.controller_manager_client import ControllerManagerClient
from moveit_commander.move_group import MoveGroupCommander
from moveit_msgs.msg import Grasp
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from uw_pr2_gripper_grasp_planner_cluster.convert_functions import *
from uw_pr2_python.gripper import Gripper

class GraspExecuter(object):
  
  def __init__(self, side='r', ctrl_mng=None, moveit_cmdr=None):
    '''
    Constructor
    '''
    if side == "right" or side == "r":
      arm = "right_arm"
      self.side = "r"
    elif side == 'left' or side == 'l':
      arm = "left_arm"
      self.side = "l"
    else:
      rospy.logerr("Side " + side + " is not supported")
      raise
    #self.tf_listener = tf.TransformListener() if not tf_listener else tf_listener
    self.ctrl_mng = ControllerManagerClient() if not ctrl_mng else ctrl_mng
    self.ctrl_mng.switch_controllers([self.side+'_arm_controller'], [self.side+'_cart'])
    self.moveit_cmdr = MoveGroupCommander(arm) if not moveit_cmdr else moveit_cmdr
    self.gripper = Gripper(arm)
    self.pose_pub = rospy.Publisher('/r_cart/command_pose', PoseStamped)
    self.eef_frame = self.side + "_wrist_roll_link"
    self.reference_frame = "base_link"
    self.approach_dist = 0.100
    self.lift_dist = 0.110
    self.step_size = 0.003
    self.moveit_cmdr.set_pose_reference_frame(self.reference_frame)

  def pick(self, grasp):
    '''
    Given a planned grasp, pick up an object.
    '''
    # Open the gripper
    self.gripper.open(max_effort=12)

    # Compute pregrasp pose
    grasp_pose = grasp.grasp_pose.pose
    #pregrasp_mat = np.matrix(translation_matrix(np.array([-0.100, 0.0, 0.0]))) * pose_to_mat(grasp_pose)
    pregrasp_mat = pose_to_mat(grasp_pose) * np.matrix(translation_matrix(np.array([-0.100, 0.0, 0.0])))
    pregrasp_pose = mat_to_pose(pregrasp_mat)
    self.current_mat = pregrasp_mat

    # Move to pregrasp pose
    self.ctrl_mng.switch_controllers([self.side+'_arm_controller'], [self.side+'_cart'])
    pregrasp_pose_stamped = PoseStamped()
    pregrasp_pose_stamped.pose = pregrasp_pose
    pregrasp_pose_stamped.header.frame_id = grasp.grasp_pose.header.frame_id
    pregrasp_pose_stamped.header.stamp = rospy.Time.now()
    print "pregrasp_pose_stamped=", pregrasp_pose_stamped
    result = self.moveit_cmdr.go(pregrasp_pose_stamped)
    if result:
        rospy.loginfo('Reached the Pre-Grasp Posture!')
    else:
        rospy.loginfo('failed to move to pre-grasp posture')
        return False

    # Approach
    self.approach(self.approach_dist, self.step_size)

    # Close gripper
    self.gripper.close(max_effort=12)
    rospy.loginfo('Closed the gripper')
    rospy.sleep(1.0)

    # Attach the CollisionObject

    # Lift
    self.lift(grasp, self.lift_dist, self.step_size)
    return True

  def approach(self, approach_dist, step_size):
    self.ctrl_mng.switch_controllers([self.side+'_cart'], [self.side+'_arm_controller'])
    current_mat = self.current_mat
    move_step_mat = np.matrix(translation_matrix(np.array([step_size, 0.0, 0.0])))
    for i in range(int(approach_dist/step_size)):
      current_mat = current_mat * move_step_mat
      self.pose_pub.publish(stamp_pose(mat_to_pose(current_mat), self.reference_frame))
      rospy.sleep(0.1)    
    self.ctrl_mng.switch_controllers([self.side+'_arm_controller'], [self.side+'_cart'])


  def lift(self, grasp, lift_dist, step_size):
    self.ctrl_mng.switch_controllers([self.side+'_cart'], [self.side+'_arm_controller'])
    current_mat = pose_to_mat(grasp.grasp_pose.pose)
    move_step_mat = np.matrix(translation_matrix(np.array([0.0, 0.0, -step_size])))
    for i in range(int(lift_dist/step_size)):
      #current_mat = current_mat * move_step_mat
      current_mat[2,3] += step_size
      self.pose_pub.publish(stamp_pose(mat_to_pose(current_mat), self.reference_frame))
      rospy.sleep(0.05)    
    self.ctrl_mng.switch_controllers([self.side+'_arm_controller'], [self.side+'_cart'])


if __name__ == "__main__":
  rospy.init_node("grasp_executer_test")
  ge = GraspExecuter()
  grasp = Grasp()
  grasp.grasp_pose.header.frame_id = "base_link"
  grasp.grasp_pose.pose.position.x = 0.391858790977
  grasp.grasp_pose.pose.position.y = -0.144160961162
  grasp.grasp_pose.pose.position.z = 0.759999990463
  grasp.grasp_pose.pose.orientation.w = 0
  grasp.grasp_pose.pose.orientation.x = 0.966644052397
  grasp.grasp_pose.pose.orientation.y = -0.256123556053
  grasp.grasp_pose.pose.orientation.z = 0
  ge.pick(grasp)
  #ge.approach(0.100, 0.005)
