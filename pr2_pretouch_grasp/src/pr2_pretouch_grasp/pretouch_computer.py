#!/usr/bin/python
import rospy
import numpy as np
import pyflann
from math import atan2, degrees, pi
from geometry_msgs.msg import Vector3, Pose
from uw_pr2_gripper_grasp_planner_cluster.convert_functions import *
import uw_pr2_gripper_grasp_planner_cluster.draw_functions as draw_functions
from pr2_pretouch_msgs.msg import GraspableObject, Grasp, Probe
from tf.transformations import quaternion_from_euler, rotation_matrix, translation_matrix
from moveit_msgs.srv import GetPositionIKRequest, GetPositionIK

class PretouchComputer:
  '''
  Class for doing pretouch planning on point clusters
  '''
  def __init__(self, tf_listener = None, tf_broadcaster = None):

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

    #draw_functions object for drawing stuff in rviz
    self.draw_functions = draw_functions.DrawFunctions('pretouch_computer_markers')

    #grasp candidates
    self.grasp_candidates = []
    self.ready_to_grasp = False

    #the transform from wrist_frame to the center of the fingers
    self.gripper_to_r_finger = np.eye(4)
    self.gripper_to_r_finger[0][3] = 0.1615 #x-translation from wrist_frame (0.1615)
    self.gripper_to_r_finger[1][3] = -0.0400 #y-translation from wrist_frame (0.0425)    
    self.gripper_to_l_finger = np.eye(4)
    self.gripper_to_l_finger[0][3] = 0.1615 #x-translation from wrist_frame (0.1615)
    self.gripper_to_l_finger[1][3] = 0.0400 #y-translation from wrist_frame (-0.0425)
  
    #how many neighbor contact points to evaluate the uncertainty?
    self.K = 10

    #what's the threshold of the mean probability from the above K points?    
    self.prob_threshold = 0.9

    #How many candidates do we need before actually attempt to grasp? (defalt=5)
    self.num_grasp_candidates_required = 1

  #initialize some member variables data
  #target: tabletop_octomap/GraspableObject
  #grasp: object_manipulation_msgs/Grasp
  def init_pretouch_computer(self, grasps, target, num_grasp_candidates_required):
    '''
    initialize some member variables data
    Args:
      grasp (object_manipulation_msgs/Grasp)
      target (tabletop_octomap/GraspableObject)
    '''
    self.grasps = grasps
    self.cluster_frame_id = target.cluster.header.frame_id
    self.reference_frame_id = target.reference_frame_id
    self.cluster = target.cluster
    self.probabilities = target.probabilities 
    self.num_grasp_candidates_required = num_grasp_candidates_required
    self.grasp_candidates = []
    self.ready_to_grasp = False

  '''
  def transformToTorso(self, pose)
    #to torso_lift_link
    grasp_pose = PoseStamped()
    grasp_pose.pose = grasp.grasp_pose
    grasp_pose.header.frame_id = self.reference_frame_id
    grasp_pose.header.stamp = target.cluster.header.stamp
    pose = change_pose_stamped_frame(self.tf_listener, grasp_pose, 'torso_lift_link')
    print 'grasp_pose w.r.t. torso_lift_link'
    print pose
  '''
  
  def find_pretouch(self):
    for i in range(len(self.grasps)):
      print ' '
      print 'COMPUTE FOR GRASP #' , i
      (joint_position, have_solution) = self.getConstraintAwareIK(self.grasps[i].grasp_pose.pose)
      if have_solution:
        (r_probe, l_probe, success) = self.find_pretouch_for_one_grasp(self.grasps[i])
        if success:
          if self.ready_to_grasp:
            print "Having enough grasp candidates (Ready to Grasp)"
            return (r_probe, l_probe, self.grasps[0], self.grasp_candidates, self.ready_to_grasp)
          else:
            print "Pretouch is required"
            return (r_probe, l_probe, self.grasps[i], self.grasp_candidates, self.ready_to_grasp)
      else:
        print "    This grasp pose is not feasible"
    print "We don't have enough grasp candidates, and no valid pretouch can be performed..."
    return (r_probe, l_probe, None, self.grasp_candidates, self.ready_to_grasp) 


  def eval_grasp_improvement(self, grasp_pose):
  
    (cluster_mat, cluster_to_reference_transform) = transform_point_cloud(self.tf_listener, self.cluster, self.reference_frame_id)
    pose_mat = pose_to_mat(grasp_pose)
    #transform the grasp_pose to r_finger (4x4 matrix)
    r_finger_mat = pose_mat * self.gripper_to_r_finger
    r_finger_point = r_finger_mat[0:3, 3].T
    #transform the grasp_pose to l_finger (4x4)
    l_finger_mat = pose_mat * self.gripper_to_l_finger
    l_finger_point = l_finger_mat[0:3, 3].T
    #convert pointcloud to numpy 3xN matrix 
    points = cluster_mat[0:3,:].T
    #flann object
    flann = pyflann.FLANN()
    params = flann.build_index(points, target_precision=0.9, log_level = "info"); #points is 3Xn
    #r_finger_contacts
    result, dists = flann.nn_index(r_finger_point, self.K, checks=params["checks"]); #r_finger_point is 3x1
    r_prob = np.array([])
    for i in range(self.K):
      if i == 0:
        mat = cluster_mat[0:3,result[0][i]]
      else:
        mat = np.hstack((mat, cluster_mat[0:3,result[0][i]]))
      r_prob = np.append(r_prob, self.probabilities[result[0][i]])
    #l_finger_contacts
    result, dists = flann.nn_index(l_finger_point, self.K, checks=params["checks"]); #r_finger_point is 3x1
    l_prob = np.array([])
    for i in range(self.K):
      if i == 0:
        mat = cluster_mat[0:3,result[0][i]]
      else:
        mat = np.hstack((mat, cluster_mat[0:3,result[0][i]]))
      l_prob = np.append(l_prob, self.probabilities[result[0][i]])
    probability.r_prob.mean()


  def find_pretouch_for_one_grasp(self, grasp):
    '''
    Find the two contact points in the graspable object
    '''
    grasp_pose = grasp.grasp_pose
    #pregrasp_pose = grasp.pre_grasp_pose

    #self.grasp_pose = grasps[0].grasp_pose

    r_probe = Probe()
    l_probe = Probe()

    #now = rospy.Time.now()
    #self.tf_listener.waitForTransform(self.cluster_frame, self.reference_frame_id, now, rospy.Duration(2.0))
    #(trans, rot) = self.tf_listener.lookupTransform(self.cluster_frame_id, self..reference_frame_id, now)
    #transform the cluster to reference frame (a 4xN matrix)
    (cluster_mat, cluster_to_reference_transform) = transform_point_cloud(self.tf_listener, self.cluster, self.reference_frame_id)
  
    pose_mat = pose_to_mat(grasp_pose.pose)
    #transform the grasp_pose to r_finger (4x4 matrix)
    r_finger_mat = pose_mat * self.gripper_to_r_finger
    r_finger_point = r_finger_mat[0:3, 3].T
    #print r_finger_point
    #transform the grasp_pose to l_finger (4x4)
    l_finger_mat = pose_mat * self.gripper_to_l_finger
    l_finger_point = l_finger_mat[0:3, 3].T
    #print l_finger_point

    #draw the two finger points on rviz
    self.draw_functions.draw_rviz_sphere(r_finger_mat, 0.005, frame = self.reference_frame_id, ns = 'r_finger_points', id = 0, duration = 60., color = [1,0.2,0.2], opaque = 0.9, frame_locked = False)
    self.draw_functions.draw_rviz_sphere(l_finger_mat, 0.005, frame = self.reference_frame_id, ns = 'l_finger_points', id = 0, duration = 60., color = [1,0.2,0.2], opaque = 0.9, frame_locked = False)

    #convert pointcloud to numpy 3xN matrix 
    points = cluster_mat[0:3,:].T
   
    #flann object
    flann = pyflann.FLANN()
    params = flann.build_index(points, target_precision=0.9, log_level = "info"); #points is 3Xn

    #r_finger_contacts
    result, dists = flann.nn_index(r_finger_point, self.K, checks=params["checks"]); #r_finger_point is 3x1
    r_prob = np.array([])
    for i in range(self.K):
      if i == 0:
        mat = cluster_mat[0:3,result[0][i]]
      else:
        mat = np.hstack((mat, cluster_mat[0:3,result[0][i]]))
      r_prob = np.append(r_prob, self.probabilities[result[0][i]])
    self.draw_functions.draw_rviz_points(mat, frame = self.reference_frame_id, size = .005, ns = 'r_touch_points', id = 0, duration = 60., color = [0,1,0], opaque = 1.0, pose_mat = None, frame_locked = False)
    contact_point = np.mean(mat, axis=1)

    #direction
    r_to_l_direction = -(r_finger_point - l_finger_point)[0]
    r_probe.direction = Vector3(x=r_to_l_direction[0,0], y=r_to_l_direction[0,1], z=r_to_l_direction[0,2])

    #touch_pose w.r.t self.reference_frame_id
    r_rot_z = math.atan2(r_probe.direction.y, r_probe.direction.x) # (radians)
    r_touch_mat = np.matrix(rotation_matrix(r_rot_z, np.array([0,0,1]))) * np.matrix(rotation_matrix(pi, np.array([1,0,0])))
    r_touch_mat[0,3] = contact_point[0]
    r_touch_mat[1,3] = contact_point[1]
    r_touch_mat[2,3] = contact_point[2]

    #pretouch_pose w.r.t self.reference_frame_id
    back_up_mat = np.matrix(np.identity(4))
    back_up_mat[0,3] = -0.05 #5cm (pretouch parameter)
    r_pretouch_mat = r_touch_mat * back_up_mat
    '''
    print "back_up_mat="
    print back_up_mat
    print "r_touch_mat="
    print  r_touch_mat
    print "r_pretouch_mat="
    print  r_pretouch_mat
    '''
    self.draw_functions.draw_rviz_sphere(np.matrix(r_touch_mat), 0.01, frame = self.reference_frame_id, ns = 'r_touch', id = 0, duration = 600., color = [1,0,1], opaque = 0.9, frame_locked = False)
    self.draw_functions.draw_rviz_sphere(r_pretouch_mat, 0.01, frame = self.reference_frame_id, ns = 'r_pretouch', id = 0, duration = 600., color = [0,0,1], opaque = 0.9, frame_locked = False)

    #all other members
    #mean probability of the K touch points
    r_probe.probability = r_prob.mean()
    r_probe.touch_pose = mat_to_pose(r_touch_mat)
    r_probe.pretouch_pose = mat_to_pose(r_pretouch_mat)
    r_probe.reference_frame_id = self.reference_frame_id

    #l_finger_contacts
    result, dists = flann.nn_index(l_finger_point, self.K, checks=params["checks"]); #r_finger_point is 3x1
    l_prob = np.array([])
    for i in range(self.K):
      if i == 0:
        mat = cluster_mat[0:3,result[0][i]]
      else:
        mat = np.hstack((mat, cluster_mat[0:3,result[0][i]]))
      l_prob = np.append(l_prob, self.probabilities[result[0][i]])
    self.draw_functions.draw_rviz_points(mat, frame = self.reference_frame_id, size = .005, ns = 'l_nearest_points', id = 0, duration = 60., color = [0,1,0], opaque = 1.0, pose_mat = None, frame_locked = False)
    l_touch_mat = np.matrix(np.identity(4))
    contact_point = np.mean(mat, axis=1)

    #direction
    l_probe.direction = Vector3(x=-r_to_l_direction[0,0], y=-r_to_l_direction[0,1], z=-r_to_l_direction[0,2])

    #touch_pose w.r.t self.reference_frame_id
    l_rot_z = math.atan2(l_probe.direction.y, l_probe.direction.x) # (radians)
    l_touch_mat = np.matrix(rotation_matrix(l_rot_z, np.array([0,0,1]))) * np.matrix(rotation_matrix(pi, np.array([1,0,0])))
    l_touch_mat[0,3] = contact_point[0]
    l_touch_mat[1,3] = contact_point[1]
    l_touch_mat[2,3] = contact_point[2]

    #pretouch_pose w.r.t self.reference_frame_id
    l_pretouch_mat = l_touch_mat * back_up_mat
    '''
    print "back_up_mat="
    print back_up_mat
    print "l_touch_mat="
    print l_touch_mat
    print "l_pretouch_mat="
    print l_pretouch_mat
    '''
    self.draw_functions.draw_rviz_sphere(np.matrix(l_touch_mat), 0.01, frame = self.reference_frame_id, ns = 'l_touch', id = 0, duration = 600., color = [1,0,1], opaque = 0.9, frame_locked = False)
    self.draw_functions.draw_rviz_sphere(l_pretouch_mat, 0.01, frame = self.reference_frame_id, ns = 'l_pretouch', id = 0, duration = 600., color = [0,0,1], opaque = 0.9, frame_locked = False)

    #all other members
    #mean probability of the K touch points
    l_probe.probability = l_prob.mean()
    l_probe.touch_pose = mat_to_pose(l_touch_mat)
    l_probe.pretouch_pose = mat_to_pose(l_pretouch_mat)
    l_probe.reference_frame_id = self.reference_frame_id

    """
    #(debug)
    print 'angle_r = ', degrees(math.atan2(r_probe.direction.y, r_probe.direction.x))
    print 'angle_l = ', degrees(math.atan2(l_probe.direction.y, l_probe.direction.x))
    r_rot_z = math.atan2(r_probe.direction.y, r_probe.direction.x)
    l_rot_z = math.atan2(l_probe.direction.y, l_probe.direction.x)
    r_quat = quaternion_from_euler(0, 0, r_rot_z)
    l_quat = quaternion_from_euler(0, 0, l_rot_z)
    print r_quat
    print l_quat
    """
    
    r_pretouch_found = False
    r_probe.required = False
    # r_finger area is uncertain
    print 'r_probe.probability=', r_probe.probability
    if r_probe.probability < self.prob_threshold:
      r_probe.required = True
      (pretouch_pose, pretouch_joint_states, success) = self.adjustPretouchPose(r_probe.pretouch_pose)
      if (success):
        r_probe.pretouch_pose = pretouch_pose
        r_probe.pretouch_joint_states.position = pretouch_joint_states
        r_pretouch_found = True
      else:
        print 'no IK solution for R'

    l_pretouch_found = False
    l_probe.required = False
    print 'l_probe.probability=', l_probe.probability
    if l_probe.probability < self.prob_threshold:
      l_probe.required = True
      (pretouch_pose, pretouch_joint_states, success) = self.adjustPretouchPose(l_probe.pretouch_pose)
      if (success):
        l_probe.pretouch_pose = pretouch_pose
        l_probe.pretouch_joint_states.position = pretouch_joint_states
        l_pretouch_found = True
      else:
        print 'no IK solution for L'

    #if pretouch is not required for both sides, that's a good grasp already
    #else if either side is required pretouch and found IK solution, consider it an informative probe
    if (not r_probe.required) and (not l_probe.required):
      # Check pregrasp pose?
      self.grasp_candidates.append(grasp)
      print 'Confident Grasp: We now have ', len(self.grasp_candidates), ' Grasp Candidates'
      if len(self.grasp_candidates) >= self.num_grasp_candidates_required:
        self.ready_to_grasp = True
        return (r_probe, l_probe, True)
      else:
        print 'We need more Grasp Candidates to be safe, continue evaluating other grasps'
        return (r_probe, l_probe, False) 
    elif r_probe.required*r_pretouch_found or l_probe.required*l_pretouch_found:
      return (r_probe, l_probe, True)
    else:
      print 'Pretouch is Required, But No Good Pre-Touch'
      return (r_probe, l_probe, False)


  def testGraspFeasibility(self, grasp_pose):
    # a grasp pose w.r.t self.reference_frame_id
    # select one here...
    # (joint_position, have_solution) = self.getIK(pretouch_wrist_roll_pose)
    (joint_position, have_solution) = self.getConstraintAwareIK(grasp_pose.pose)
    return (joint_position, have_solution)


  def adjustPretouchPose(self, pose):    
    #transform the pose from fingertip_frame to X_wrist_roll_link
    r_gripper_l_fingertip_to_r_wrist_roll_link_mat = np.matrix(translation_matrix(np.array([-0.18, -0.049, 0.0])))
  
    current_pose = pose
    pretouch_mat = pose_to_mat(current_pose)
    pretouch_wrist_roll_pose = mat_to_pose(pretouch_mat * r_gripper_l_fingertip_to_r_wrist_roll_link_mat)

    #TODO Query IK solution for that pretouch pose
    # while no IK solution, rotate w.r.t Z-axis toward the robot (positive rotation)
    
    #print 'ik solution for l_probe: ', self.getConstraintAwareIK(l_probe.pretouch_pose)
    #print 'ik solution for r_probe: ', self.getConstraintAwareIK(r_probe.pretouch_pose)

		#select one here...
    #(joint_position, have_solution) = self.getIK(pretouch_wrist_roll_pose)
    (joint_position, have_solution) = self.getConstraintAwareIK(pretouch_wrist_roll_pose)
    
    '''
    if not have_solution:
      #rotate the pose toward the robot
      #rot_mat = rotation_matrix(l_rot_z, np.array([0,0,1])) #0.175 radians = 10 degree
      print 'No IK solution found'
    '''
    return (pretouch_wrist_roll_pose, joint_position, have_solution)




  def getConstraintAwareIK(self, pose):

    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = self.reference_frame_id
    pose_stamped.pose = pose
    #transform pose_stamped (why?)
    #pose_stamped = change_pose_stamped_frame(self.tf_listener, pose_stamped, 'torso_lift_link')

    get_ik_req = GetPositionIKRequest()
    get_ik_req.ik_request.group_name = "right_arm"
    #get_ik_req.ik_request.robot_state = #moveit_msgs/RobotState
    get_ik_req.ik_request.avoid_collisions = False
    get_ik_req.ik_request.ik_link_name = "r_wrist_roll_link"
    get_ik_req.ik_request.pose_stamped = pose_stamped
    get_ik_req.ik_request.timeout = rospy.Duration(5.0)
    get_ik_req.ik_request.attempts = 10
    
    try:
      ik_solver_client = rospy.ServiceProxy('compute_ik', GetPositionIK)
      resp = ik_solver_client(get_ik_req)
    except rospy.ServiceException, e:
      print 'Cannot call ik solver service'%e

    if resp.error_code.val == resp.error_code.SUCCESS:
      return (resp.solution.joint_state.position, True)
    else:
      return ([], False)

    '''
    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = self.reference_frame_id
    pose_stamped.pose = pose
    #transform pose_stamped 
    pose_stamped = change_pose_stamped_frame(self.tf_listener, pose_stamped, 'torso_lift_link')
    try:
      query_client = rospy.ServiceProxy('pr2_right_arm_kinematics/get_ik_solver_info', GetKinematicSolverInfo)
      resp = query_client()
      joint_names = resp.kinematic_solver_info.joint_names
      limits = resp.kinematic_solver_info.limits
    except rospy.ServiceException, e:
      print 'Cannot call ik solver query service'%e
    #get current joint states
    try:
      joint_states_client = rospy.ServiceProxy('return_joint_states', ReturnJointStates)
      resp = joint_states_client(joint_names)
      current_joint_position = resp.position #use current states as the seed
    except rospy.ServiceException, e:
      print "error when calling return_joint_states: %s"%e
    try:
      ik_client = rospy.ServiceProxy('/pr2_right_arm_kinematics/get_constraint_aware_ik', GetConstraintAwarePositionIK)
      req = GetConstraintAwarePositionIKRequest()
      req.ik_request.ik_link_name = "r_wrist_roll_link"
      req.ik_request.pose_stamped = pose_stamped
      req.ik_request.ik_seed_state.joint_state.name = joint_names
      req.ik_request.ik_seed_state.joint_state.position = current_joint_position
      req.timeout = rospy.Duration(5.0)
      #req.constraints = ?? #arm_navigation_msgs/Constraints
      resp = ik_client(req)
      if resp.error_code.val == resp.error_code.SUCCESS:
        print 'Found IK Solution:', resp.solution.joint_state.position
        return (resp.solution.joint_state.position, True)
      else:
        print 'error code: ',  resp.error_code.val
        return ([], False)
    except rospy.ServiceException, e:
      print "Service call failed: %s"%e
      return ([], False)
    '''


  def getIK(self, pose):

    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = self.reference_frame_id
    pose_stamped.pose = pose
    #transform pose_stamped 
    pose_stamped = change_pose_stamped_frame(self.tf_listener, pose_stamped, 'torso_lift_link')
    #get current joint states
    try:
      joint_states_client = rospy.ServiceProxy('return_joint_states', ReturnJointStates)
      resp = joint_states_client()
      current_joint_position = resp.position
    except rospy.ServiceException, e:
      print "error when calling return_joint_states: %s"%e
    try:
      query_client = rospy.ServiceProxy('pr2_right_arm_kinematics/get_ik_solver_info', GetKinematicSolverInfo)
      resp = query_client()
      joint_names = resp.kinematic_solver_info.joint_names
      limits = resp.kinematic_solver_info.limits
    except rospy.ServiceException, e:
      print 'Cannot call ik solver query service'%e
    try:
      ik_client = rospy.ServiceProxy('/pr2_right_arm_kinematics/get_ik', GetPositionIK)
      req = GetPositionIKRequest()
      req.ik_request.ik_link_name = "r_wrist_roll_link"
      req.ik_request.pose_stamped = pose_stamped
      req.ik_request.ik_seed_state.joint_state.name = joint_names
      req.ik_request.ik_seed_state.joint_state.position = current_joint_position
      req.timeout = rospy.Duration(5.0)
      resp = ik_client(req)
      if resp.error_code.val == resp.error_code.SUCCESS:
        print 'Found IK Solution:', resp.solution.joint_state.position
        return (resp.solution.joint_state.position, True)
      else:
        print 'error code: ',  resp.error_code.val
        return ([], False)
    except rospy.ServiceException, e:
      print "Service call failed: %s"%e
      return ([], False)
