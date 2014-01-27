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

import rospy
from uw_pr2_gripper_grasp_planner_cluster.srv import GraspPlanning, GraspPlanningRequest
from pr2_pretouch_msgs.srv import GetProbabilisticPointCloud
from pr2_pretouch_msgs.srv import FindPretouch, FindPretouchRequest, ExecutePretouch, ExecutePretouchRequest
from uw_pr2_gripper_grasp_planner_cluster.msg import GraspableObject
from uw_pr2_gripper_grasp_planner_cluster.srv import SetPointClusterGraspParams, SetPointClusterGraspParamsRequest
from uw_pr2_gripper_grasp_planner_cluster.convert_functions import *
import pyflann
import tf
from geometry_msgs.msg import Vector3Stamped, TwistStamped
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty
from uw_pr2_python.controller_manager_client import ControllerManagerClient
from uw_pr2_python.gripper import Gripper
from moveit_commander.move_group import MoveGroupCommander
from grasp_executer import GraspExecuter

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
# Reference frame
reference_frame_id = 'base_link'

#services
rospy.wait_for_service('tabletop_octomap')
rospy.wait_for_service('get_probabilistic_pointcloud')
rospy.wait_for_service('plan_point_cluster_grasp')
rospy.wait_for_service('set_point_cluster_grasp_params')
rospy.wait_for_service('evaluate_point_cluster_grasps')
rospy.wait_for_service('find_pretouch')

# Controller switcher
cm_client = ControllerManagerClient()
cm_client.switch_controllers(['r_arm_controller'], ['r_cart'])

# MoveIt! Commander
moveit_cmd = MoveGroupCommander("right_arm")
moveit_cmd.set_pose_reference_frame(reference_frame_id)
pose = Pose()
pose.position.x = 0.258
pose.position.y = -0.614
pose.position.z = 1.017
pose.orientation.w = 0.720
pose.orientation.x = 0.059
pose.orientation.y = 0.153
pose.orientation.z = 0.674
moveit_cmd.go(pose, wait=True)


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
    #eeq.height_good_for_side_grasps
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
    plan_grasp_client = rospy.ServiceProxy('plan_point_cluster_grasp', GraspPlanning) 
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
    req.num_grasp_candidates_required = 5 #5
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
rospy.logerr('Grasps are ready to be executed!')
#print 'We have ', len(grasp_candidates), ' Grasp Candidates'

ge = GraspExecuter(side='r', ctrl_mng=cm_client, moveit_cmdr=moveit_cmd)
for grasp in grasp_candidates:
  print grasp
  if ge.pick(grasp):
    rospy.signal_shutdown("Grasping succeeded")
rospy.logerr("Grasping failed")

'''
# We still have problem using this MoveIt! functionality
rospy.loginfo('grasping using MoveIt! pickup()')
moveit_cmd.pick('part', grasp_candidates)
rospy.loginfo('finished grasping using MoveIt! pickup()')
'''

