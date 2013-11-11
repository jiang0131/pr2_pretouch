#!/usr/bin/python
# Software License Agreement (BSD License)
#
# Copyright (c) 2012, University of Washington, Seattle
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the University of Washington nor the names of
#    its contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# author: Liang-Ting Jiang
# package: pr2_pretouch_grasp
#	Plan grasps using pointcloud and probabilities from the octomap server

import roslib
roslib.load_manifest('pr2_gripper_grasp_planner_cluster')
roslib.load_manifest('tabletop_octomap')
import rospy
from pr2_pretouch_msgs.srv import GraspPlanning, GraspPlanningRequest, GetProbabilisticPointCloud
from pr2_pretouch_msgs.srv import FindPretouch, FindPretouchRequest, ExecutePretouch, ExecutePretouchRequest
from pr2_pretouch_msgs.msg import GraspableObject
from pr2_gripper_grasp_planner_cluster.srv import SetPointClusterGraspParams, SetPointClusterGraspParamsRequest
from object_manipulator.convert_functions import *
import pyflann
import tf
from pr2_python.arm_mover import ArmMover
from geometry_msgs.msg import Vector3Stamped
from object_manipulation_msgs.msg import PickupGoal, PickupAction
from tabletop_collision_map_processing.srv import TabletopCollisionMapProcessing, TabletopCollisionMapProcessingRequest
import actionlib
import object_manipulation_msgs
from std_srvs.srv import Empty
import tabletop_collision_map_processing.collision_map_interface as collision_map_interface
from pr2_python.arm_mover import ArmMover
from pr2_python.controller_manager_client import ControllerManagerClient
from geometry_msgs.msg import TwistStamped
from pr2_python.gripper import Gripper

rospy.init_node('probabilistic_grasping_eval_node', anonymous=True)

"""
#given an object cluster and a grasp pose, compute the two contact points
#return: geometry_msgs::Point32 contact_point1, contact_point2
#				 geometry_msgs::Vector3 normal1, normal2
#				 float uncertainty1, uncertainty2: the mean probability around the contact point
def compute_contact(cluster, probatilities, grasp_pose):
	frame = cluster.header.frame_id
	
	return (contact_point1, contact_point2, normal1, normal2, \\
					uncertainty1, uncertainty2)
"""

#services
rospy.wait_for_service('tabletop_octomap')
rospy.wait_for_service('get_probabilistic_pointcloud')
rospy.wait_for_service('plan_probabilistic_point_cluster_grasp')
rospy.wait_for_service('set_probabilistic_point_cluster_grasp_params')
rospy.wait_for_service('evaluate_probabilistic_point_cluster_grasps')
rospy.wait_for_service('find_pretouch')
rospy.wait_for_service('tabletop_collision_map_processing/tabletop_collision_map_processing')

reference_frame_id = 'base_link'


#reset the octomap
try:
  reset_client = rospy.ServiceProxy('/tabletop_octomap_server/reset', Empty)
  reset_client()
  print "Successfully Reset the Octomap"
except rospy.ServiceException, e:
	print "Octomap Reset Service call failed: %s"%e

#initialize the tabletop octomap prior
try:
  init_prior_client = rospy.ServiceProxy('/tabletop_octomap', Empty)
  init_prior_client()
  print "Successfully Assigned the Tabletop Octomap Prior"
except rospy.ServiceException, e:
	print "Tabletop Octomap Service call failed: %s"%e

##########################################################################################################
#Here Start the Main Loop

ready_to_grasp = False

while not ready_to_grasp:

  #get object cluster and probability from tabletop_octomap server node
  try:
    get_cluster_client = rospy.ServiceProxy('get_probabilistic_pointcloud', GetProbabilisticPointCloud)
    resp = get_cluster_client()
    cloud = resp.cloud
    original_cloud = resp.original_cloud
    probabilities = resp.probabilities
    cloud_mat = point_cloud_to_mat(cloud)
    #frame = cloud.header.frame_id
    print "Successfully got the cloud and prob, the num of points:", cloud_mat.shape[1], ", the size of probabilities:", len(probabilities)
  except rospy.ServiceException, e:
    print "Service call failed: %s"%e

  '''
  #set the grasp planner parameters
  try:
    print "setting grasp planning parameters"
    set_params_client = rospy.ServiceProxy('set_probabilistic_point_cluster_grasp_params', SetPointClusterGraspParams)
    req = SetPointClusterGraspParamsRequest()
    #only want side grasps if the bounding box height is greater than this
    #req.height_good_for_side_grasps
    #bounding box "fits in hand" if the relevant dimension is less than this
    #req.gripper_opening
    #how far to move the gripper to the side with each step when searching for grasps
    #req.side_step = 0.015
    #how far to move the palm inward with each step when searching for grasps
    #req.palm_step = 0.003
    #set this to true to limit the planner to overhead grasps
    #req.overhead_grasps_only = False
    #set this to true to limit the planner to side grasps
    req.side_grasps_only = False
    #set this to false to omit random high-point grasps
    req.include_high_point_grasps = False
    #set this to true to make the pregrasps be just outside the bounding box instead of self.pregrasp_dist away from the grasp
    #req.pregrasp_just_outside_box
    #how many backing-off-from-the-deepest-possible-grasp grasps to keep for each good grasp found
    #req.backoff_depth_steps
    #don't check the neighbors for each grasp (reduces grasps checked, but makes for worse rankings)
    #req.disable_grasp_neighbor_check = False
    #set this to true to randomize the order of the first 30 grasps 
    req.randomize_grasps = True
    resp = set_params_client(req)
  except rospy.ServiceException, e:
    print "Service call failed: %s"%e
  '''

  #request probabilistic grasp planning with weighting
  try:
    print "planning probabilistic grasp.."
    plan_grasp_client = rospy.ServiceProxy('plan_probabilistic_point_cluster_grasp', GraspPlanning) 
    req = GraspPlanningRequest()
    req.arm_name = "right_arm"
    #GraspableObject
    graspable = GraspableObject()
    graspable.use_probability = True
    graspable.cluster = cloud
    graspable.probabilities = probabilities
    graspable.reference_frame_id = reference_frame_id
    req.target = graspable
    resp = plan_grasp_client(req)
    grasps = resp.grasps
    error_code = resp.error_code
    print "Successfully plan the Grasp, number of planned grasp = ", len(grasps)
    #print "the best grasp: "
    #print grasps[0]  
  except rospy.ServiceException, e:
    print "Service call failed: %s"%e

  """
  #request probabilistic grasp planning without weighting (equal prob)
  try:
    print "planning probabilistic grasps without weighting.."
    plan_grasp_client = rospy.ServiceProxy('plan_probabilistic_point_cluster_grasp', GraspPlanning) 
    req = GraspPlanningRequest()
    req.arm_name = "right_arm"
    #GraspableObject
    req.target.use_probability = True
    req.target.cluster = cloud
    req.target.probabilities = [] #empty prob
    req.target.reference_frame_id = reference_frame_id
    resp = plan_grasp_client(req)
    grasps = resp.grasps
    error_code = resp.error_code
    print "Successfully plan the Grasp"
  except rospy.ServiceException, e:
    print "Service call failed: %s"%e
  """
  """
  #request deterministic grasp planning
  try:
    print "planning deterministic grasps.."
    plan_grasp_client = rospy.ServiceProxy('plan_probabilistic_point_cluster_grasp', GraspPlanning) 
    req = GraspPlanningRequest()
    req.arm_name = "right_arm"
    #GraspableObject
    req.target.use_probability = False
    req.target.cluster = original_cloud
    req.target.probabilities = []
    req.target.reference_frame_id = reference_frame_id
    resp = plan_grasp_client(req)
    grasps = resp.grasps
    error_code = resp.error_code
    print "Successfully plan the Grasp"
  except rospy.ServiceException, e:
    print "Service call failed: %s"%e
  """

  """
  def grasp_pose_to_finger_points(grasp_pose, cloud):
    #transform cluster to the reference frame
    tf_ros = tf.TransformerROS()
    k = tf_ros.getFrameStrings()
    print k
    print cloud.header.frame_id
    #tf_ros.waitForTransform(cloud.header.frame_id, reference_frame_id, rospy.Time.now(), rospy.Duration(2.0))
    cloud = tf_ros.transformPointCloud(reference_frame_id, cloud)  
    print cloud.header.frame_id
  """

  #finding good pretouch poses
  try:
    print "Finding Pretouch......"
    find_pretouch_client = rospy.ServiceProxy('find_pretouch', FindPretouch) 
    req = FindPretouchRequest()
    req.target.use_probability = True
    req.target.cluster = cloud
    req.target.probabilities = probabilities
    req.target.reference_frame_id = reference_frame_id
    req.grasps = grasps #the list of planned grasps
    req.num_grasp_candidates_required = 1 #5
    resp = find_pretouch_client(req)
    grasp_candidates = resp.grasp_candidates
    #print resp
  except rospy.ServiceException, e:
    print "find_pretouch Service call failed: %s"%e

  print 'len(grasp_candidates)=', len(grasp_candidates)
  print 'req.num_grasp_candidates_required=', req.num_grasp_candidates_required
  if (resp.ready_to_grasp) or (len(grasp_candidates) >= req.num_grasp_candidates_required):
    ready_to_grasp = True
    print 'Enough Good Grasps...We have ', len(grasp_candidates), ' Good Grasp now. (We need ', req.num_grasp_candidates_required, ')'
  else:
    print 'Not Enough Good Grasps....We only have ', len(grasp_candidates), ' Good Grasp now. (We need ', req.num_grasp_candidates_required, ')'

  #execute the pretouch exploration
  try:
    print "Executing Pretouch......"
    execute_pretouch_client = rospy.ServiceProxy('execute_pretouch', ExecutePretouch) 
    req = ExecutePretouchRequest()
    req.r_probe = resp.r_probe
    req.l_probe = resp.l_probe
    resp = execute_pretouch_client(req)
  except rospy.ServiceException, e:
    print "execute_pretouch Service call failed: %s"%e

  #move arm to side
  #if ready_to_grasp:

  

#re-evaluate the neighbor points around the actual pretouch location
#if good, grasp
#grasp_final = grasp_candidate
rospy.loginfo('!!!!!!THE GRASPS ARE READY TO BE EXECUTED!!!!!!!:')
#print 'We have ', len(grasp_candidates), ' Grasp Candidates'

'''
## reset the collision map for grasping...
#initialize a collision map interface
collision_map_interface = collision_map_interface.CollisionMapInterface()
collision_map_interface.reset_collision_map()
'''

##A simple grasp executor (LT)
##Create ArmMover object
arm_mover = ArmMover()
goal_pose = PoseStamped()
#goal_pose.pose.position.x = 0.437132434132
#goal_pose.pose.position.y = -0.363075993519
#goal_pose.pose.position.z = 0.724999974966
#goal_pose.pose.orientation.x = 0.880697073031
#goal_pose.pose.orientation.y = 0.473679918885
#goal_pose.pose.orientation.z = 0
#goal_pose.pose.orientation.w = 0
goal_pose.pose = grasp_candidates[0].pre_grasp_pose
goal_pose.header.frame_id = '/base_link'
print 'moving to pregrasp pose:'
print goal_pose
handle = arm_mover.move_to_goal('right_arm', goal_pose) 
if handle.reached_goal():
    print 'Reached the Pre-Grasp Pose!'
else:
    print handle.get_errors()

#move gripper forward
cm_client = ControllerManagerClient()
twist_pub = rospy.Publisher('/r_cart/command_twist', TwistStamped)
cm_client.switch_controllers(['r_cart'], ['r_arm_controller'])
start_time = rospy.Time.now()
while (rospy.Time.now() - start_time < rospy.Duration(2.0)):
  msg = TwistStamped()
  msg.header.frame_id = 'r_wrist_roll_link'
  msg.header.stamp = rospy.Time.now()
  msg.twist.linear.x = 0.025
  twist_pub.publish(msg)
  rospy.sleep(0.1)

print 'Reached the Grasp Pose!'
rospy.sleep(2.)

#close the gripper
gripper = Gripper('right_arm')
gripper.close(max_effort=12)
print 'Closed the Gripper!!'
rospy.sleep(1.0)

#lift the object
start_time = rospy.Time.now()
while (rospy.Time.now() - start_time < rospy.Duration(4.0)):
  msg = TwistStamped()
  msg.header.frame_id = 'torso_lift_link'
  msg.header.stamp = rospy.Time.now()
  msg.twist.linear.z = 0.03
  twist_pub.publish(msg)
print 'Lifted the object!'

#switch back the controller
cm_client.switch_controllers(['r_arm_controller'], ['r_cart'])
print 'Grasping succeeded!'


'''
#collision map processing (Seems not required)
collision_processing_client = rospy.ServiceProxy('/tabletop_collision_map_processing/tabletop_collision_map_processing', TabletopCollisionMapProcessing)
processing_req = TabletopCollisionMapProcessingRequest()
processing_req.detection_result = self.detection # TabletopDetection
#ask for the exising map and collision models to be reset??
processing_req.reset_static_map = True;
processing_req.reset_collision_models = True;
processing_req.reset_attached_models = True;
#ask for a new static collision map to be taken with the laser
#after the new models are added to the environment
processing_req.take_static_collision_map = True;
#ask for the results to be returned in base link frame
processing_req.desired_frame = "/base_link";
rospy.loginfo("Creating collision map.........")
try:
  resp = collision_processing_client.call(processing_req)
  rospy.loginfo("The Static Collision Map is created")
except rospy.ServiceException, e:
  print "Collision Map Processing Service call failed: %s"%e

if (len(resp.graspable_objects)==0):
  rospy.logerr("error when calling collision map processing")
  self.throw_exception()
#save the collision map processing results
graspable_objects = resp.graspable_objects
collision_object_names = resp.collision_object_names
collision_support_surface_name = resp.collision_support_surface_name
'''

'''
#convert all Grasps to object_manipulation_msgs/Grasp
cnt = 0
grasps_original = []
for g in grasp_candidates:
  cnt += 1
  print 'The #', cnt, ' Grasp:'
  grasp_original = object_manipulation_msgs.msg.Grasp()
  grasp_original.pre_grasp_posture = g.pre_grasp_posture 
  grasp_original.grasp_posture = g.grasp_posture
  grasp_original.grasp_pose = g.grasp_pose
  grasp_original.success_probability = g.success_probability
  grasp_original.cluster_rep = g.cluster_rep
  grasp_original.desired_approach_distance = g.desired_approach_distance
  grasp_original.min_approach_distance = g.min_approach_distance
  grasp_original.moved_obstacles = g.moved_obstacles
  grasps_original.append(grasp_original)
  print grasp_original

#pickup the object
pickup_client = actionlib.SimpleActionClient('/object_manipulator/object_manipulator_pickup', PickupAction)
pickup_client.wait_for_server()
pickup_goal = PickupGoal()

#tabletop_octomap/Grasp --> object_manipulation_msgs/Grasp (BAD HACK)
#grasp_original = object_manipulation_msgs.msg.Grasp()
#grasp_original.pre_grasp_posture = grasp_final.pre_grasp_posture 
#grasp_original.grasp_posture = grasp_final.grasp_posture
#grasp_original.grasp_pose = grasp_final.grasp_pose
#grasp_original.success_probability = grasp_final.success_probability
#grasp_original.cluster_rep = grasp_final.cluster_rep
#grasp_original.desired_approach_distance = grasp_final.desired_approach_distance
#grasp_original.min_approach_distance = grasp_final.min_approach_distance
#grasp_original.moved_obstacles = grasp_final.moved_obstacles

pickup_goal.desired_grasps = grasps_original #hopefully there are some good grasps

#tabletop_octomap/GraspableObject --> object_manipulation_msgs/GraspableObject (BAD HACK)
graspable_original = object_manipulation_msgs.msg.GraspableObject()
graspable_original.cluster = graspable.cluster
graspable_original.reference_frame_id = graspable.reference_frame_id
print 'graspable_original.reference_frame_id:' , graspable_original.reference_frame_id
pickup_goal.target = graspable_original

#pickup_goal.collision_object_name = collision_object_names[0] #
#pickup_goal.collision_support_surface_name = collision_support_surface_name #
pickup_goal.collision_object_name = ''
pickup_goal.collision_support_surface_name = ''
pickup_goal.allow_gripper_support_collision = False
pickup_goal.ignore_collisions = False #set to True to move to pregrasp (usually hit table)
pickup_goal.arm_name = "right_arm"
direction = Vector3Stamped()
direction.header.stamp = rospy.Time.now()
direction.header.frame_id = "base_link"
direction.vector.x = 0
direction.vector.y = 0
direction.vector.z = 1
pickup_goal.lift.direction = direction
pickup_goal.lift.desired_distance = 0.10
pickup_goal.lift.min_distance = 0.05
pickup_goal.use_reactive_lift = False
pickup_goal.use_reactive_execution = False
try:
  rospy.loginfo("Trying to Pick Up the Object....")
  pickup_client.send_goal(pickup_goal)
except rospy.ServiceException, e:
  rospy.logerr("error when calling action")

finished_within_time = pickup_client.wait_for_result(rospy.Duration(50.0))
if not finished_within_time:
  pickup_client.cancel_goal()
  rospy.logerr("timed out when asking the grasp action client to grasp")

pickup_result = pickup_client.get_result()
print "pickup result code: ", pickup_result.manipulation_result.value
if (pickup_result.manipulation_result.value != pickup_result.manipulation_result.SUCCESS):
  rospy.logerr("The pickup has failed....")
else:
  rospy.loginfo("Pickup has succeeded!!!!!")

'''

'''
#move
arm_mover = ArmMover()
arm_name = 'right_arm'
joint_position = [-0.817, 0.001, -1.253, -0.892, 60.854, -0.250, 3.338]
#joint_position = [-0.2297329370380794, 0.4009141173909514, -1.939933137093478, -1.5425226778992078, 2.7446031147729588, -1.6264057476525355, -1.2200157841711832]
js = arm_mover.get_joint_state(arm_name)
rospy.sleep(1.0)
js.position = joint_position
print 'Moving to %s' % (str(joint_position))
# send out the command                                                                                                                                    
handle = arm_mover.move_to_goal(arm_name, js)
if not handle.reached_goal():
  print handle.get_errors()
'''
