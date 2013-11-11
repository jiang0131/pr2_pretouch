import rospy
from uw_pr2_gripper_grasp_planner_cluster.point_cluster_grasp_planner_server import PointClusterGraspPlannerServer
import probabilistic_point_cluster_grasp_planner
import pretouch_computer
import pretouch_executor

class ProbabilisticPointClusterGraspPlannerServer(PointClusterGraspPlannerServer):

  def __init__(self):
    PointClusterGraspPlannerServer.__init__(self)
    # A ProabailisticGraspPlanner instance
    self.pcgp = probabilistic_point_cluster_grasp_planner.ProbabilisticPointClusterGraspPlanner()
    # A PretouchComputer instance
    self.ptc = pretouch_computer.PretouchComputer(self.pcgp.tf_listener, self.pcgp.tf_broadcaster)
    # A PretouchExecutor instance
    self.pte = pretouch_executor.PretouchExecutor(self.pcgp.tf_listener, self.pcgp.tf_broadcaster)
    #advertise service for find a pretouch approach on the object
    rospy.Service('find_pretouch', FindPretouch, self.find_pretouch_callback)
    #advertise service for execute a computed pretouch
    rospy.Service('execute_pretouch', ExecutePretouch, self.execute_pretouch_callback)


  def execute_pretouch_callback(self, req):
    r_probe = req.r_probe
    l_probe = req.l_probe
    result1 = False
    result2 = False

    resp = ExecutePretouchResponse()

    #self.side_arm()
    if r_probe.required:
      print 'len(r_probe.pretouch_joint_states.position) = ', len(r_probe.pretouch_joint_states.position)
      print 'r_probe is requred....'
      result1 = self.pte.move_to_joint_goal(r_probe.pretouch_joint_states.position)
      if result1:
        print 'move to pretouch pose successfully, continue'
        result1 = self.pte.pretouch_probe()
        print 'r_probe pretouch probe result: ', result1
      else:
        print 'failed to move to pretouch pose, cancel the r_probe'
    else:
      print 'r_probe is NOT requred....'

    if l_probe.required:
      print 'len(l_probe.pretouch_joint_states.position) = ', len(l_probe.pretouch_joint_states.position)
      print 'l_probe is requred....'
      result2 = self.pte.move_to_joint_goal(l_probe.pretouch_joint_states.position)
      if result2:
        print 'move to l_pretouch pose successfully, continue'
        result2 = self.pte.pretouch_probe()
        print 'l_probe pretouch probe result: ', result2
      else:
        print 'failed to move to pretouch pose, cancel the l_probe'
    else:
      print 'l_probe is NOT requred....'

    if (not result1) and (not result2):
      print 'Somthing Wrong with the Move Arm, Move Arms Back to the sides'
      self.pte.arm_tasks.move_arm_to_side('right_arm')

    rospy.loginfo('the pretouch exploration has finished')

    #move back to initial pose 
    #self.pte.arm_tasks.move_arm_to_side('right_arm')
    #move arm to some other good poses?

    return resp


  def find_pretouch_callback(self, req):

    #change member variables for pretouch computer
    self.ptc.init_pretouch_computer(req.grasps, req.target, req.num_grasp_candidates_required)

    #find contact points
    (r_probe, l_probe, grasp, grasp_candidates, ready_to_grasp) = self.ptc.find_pretouch()

    #print cloud.header.frame_id
    resp = FindPretouchResponse()
    resp.r_probe = r_probe
    resp.l_probe = l_probe
    resp.grasp_candidates = grasp_candidates
    resp.ready_to_grasp = ready_to_grasp

    #draw the selected grasp (which is able to be pretouched)
    print 'DRAWING THE GRASP CANDIDATE!!!'
    pose_mat = pose_to_mat(grasp.grasp_pose)
    self.pcgp.draw_gripper_model(pose_mat, frame = self.cluster_frame, pause_after_broadcast = 0, duration = 5)
    self.pcgp.draw_gripper_model(pose_mat, frame = self.cluster_frame, pause_after_broadcast = 0, duration = 5)

    return resp


  ##service callback for the evaluate_point_cluster_grasps service
  def evaluate_point_cluster_grasps_callback(self, req):

    #find the cluster bounding box and relevant frames, and transform the cluster
    if len(req.target.cluster.points) > 0:
      if (len(req.target.probabilities) > 0):
        self.pcgp.init_cluster_grasper(req.target.cluster, req.target.probabilities)
      else:
        self.pcgp.init_cluster_grasper(req.target.cluster)
      cluster_frame = req.target.cluster.header.frame_id
    else:
      self.pcgp.init_cluster_grasper(req.target.region.cloud)
      cluster_frame = req.target.region.cloud.header.frame_id

    #evaluate the grasps on the cluster
    probs = self.pcgp.evaluate_point_cluster_grasps(req.grasps_to_evaluate, cluster_frame)

    #return the same grasps with the qualities added
    for (grasp, prob) in zip(req.grasps_to_evaluate, probs):
      grasp.success_probability = prob

    #fill in the response
    resp = GraspPlanningResponse()
    resp.error_code.value = 0
    resp.grasps = req.grasps_to_evaluate

    return resp


  def plan_point_cluster_grasps(self, target, arm_name):
    error_code = GraspPlanningErrorCode()
    grasps = []

    #get the hand joint names from the param server (loaded from yaml config file)
    #hand_description = rospy.get_param('hand_description')
    hand_description = rospy.get_param('~hand_description')
    pregrasp_joint_angles_dict = rospy.get_param('~pregrasp_joint_angles')
    grasp_joint_angles_dict = rospy.get_param('~grasp_joint_angles')
    pregrasp_joint_efforts_dict = rospy.get_param('~pregrasp_joint_efforts')
    grasp_joint_efforts_dict = rospy.get_param('~grasp_joint_efforts')
    if not arm_name:
      arm_name = hand_description.keys()[0]
      rospy.logerr("point cluster grasp planner: missing arm_name in request!  Using "+arm_name)
    try:
      hand_joints = hand_description[arm_name]["hand_joints"]
    except KeyError:
      arm_name = hand_description.keys()[0]
      rospy.logerr("arm_name "+arm_name+" not found!  Using joint names from "+arm_name)
      try:
        hand_joints = hand_description[arm_name]["hand_joints"]
      except KeyError:
        rospy.logerr("no hand joints found for %s!"%arm_name)
        return ([], error_code.OTHER_ERROR)
    try:
      hand_frame = hand_description[arm_name]["hand_frame"]
      hand_approach_direction = hand_description[arm_name]["hand_approach_direction"]
    except KeyError:
      rospy.logerr("couldn't find hand_frame or hand_approach_direction!")
      return ([], error_code.OTHER_ERROR)
    pregrasp_joint_angles = pregrasp_joint_angles_dict[arm_name]
    grasp_joint_angles = grasp_joint_angles_dict[arm_name]
    pregrasp_joint_efforts = pregrasp_joint_efforts_dict[arm_name]
    grasp_joint_efforts = grasp_joint_efforts_dict[arm_name]

    #hand_joints = rospy.get_param('/hand_description/'+arm_name+'/hand_joints')
    rospy.loginfo("hand_joints:"+str(hand_joints))

    #find the cluster bounding box and relevant frames, and transform the cluster
    init_start_time = time.time()

    if len(target.cluster.points) > 0:
      if (target.use_probability): #probabilistic
        if (len(target.probabilities) > 0):
          self.pcgp.init_cluster_grasper(target.cluster, target.probabilities, use_probability=True)
        else: #weighted probabilistic
          self.pcgp.init_cluster_grasper(target.cluster, probabilities=[], use_probability=True)
      else: #deterministic
        self.pcgp.init_cluster_grasper(target.cluster, probabilities=[], use_probability=False)
      cluster_frame = target.cluster.header.frame_id
      self.cluster_frame = cluster_frame
    else:
      self.pcgp.init_cluster_grasper(target.region.cloud)
      cluster_frame = target.region.cloud.header.frame_id
      if len(cluster_frame) == 0:
        rospy.logerr("region.cloud.header.frame_id was empty!")
        error_code.value = error_code.OTHER_ERROR
        return (grasps, error_code)

    init_end_time = time.time()
    #print "init time: %.3f"%(init_end_time - init_start_time)

    #plan grasps for the cluster (returned in the cluster frame)
    grasp_plan_start_time = time.time()
    (pregrasp_poses, grasp_poses, gripper_openings, qualities, pregrasp_dists) = self.pcgp.plan_point_cluster_grasps()
    grasp_plan_end_time = time.time()
    #print "total grasp planning time: %.3f"%(grasp_plan_end_time - grasp_plan_start_time)

    #add error code to service
    error_code.value = error_code.SUCCESS
    grasp_list = []
    if pregrasp_poses == None:
      error_code.value = error_code.OTHER_ERROR
      return (grasps, error_code)
    #fill in the list of grasps
    for (grasp_pose, quality, pregrasp_dist) in zip(grasp_poses, qualities, pregrasp_dists):
      pre_grasp_joint_state = self.create_joint_trajectory(hand_joints, pregrasp_joint_angles, pregrasp_joint_efforts)
      grasp_joint_state = self.create_joint_trajectory(hand_joints, grasp_joint_angles, grasp_joint_efforts)

      #if the cluster isn't in the same frame as the graspable object reference frame,
      #transform the grasps to be in the reference frame
      if cluster_frame == target.reference_frame_id:
        transformed_grasp_pose = stamp_pose(grasp_pose, cluster_frame)
      else:
        transformed_grasp_pose = change_pose_stamped_frame(self.pcgp.tf_listener,
                                   stamp_pose(grasp_pose, cluster_frame),
                                   target.reference_frame_id)
      if self.pcgp.pregrasp_just_outside_box:
        min_approach_distance = pregrasp_dist
      else:
        min_approach_distance = max(pregrasp_dist-.05, .05)
      approach = GripperTranslation(create_vector3_stamped(hand_approach_direction, hand_frame), pregrasp_dist, min_approach_distance)
      retreat = GripperTranslation(create_vector3_stamped([-1.*x for x in hand_approach_direction], hand_frame), pregrasp_dist, min_approach_distance)

      # LT
      grasp_list.append(Grasp(id="id", pre_grasp_posture=pre_grasp_joint_state, grasp_posture=grasp_joint_state,
                              grasp_pose=transformed_grasp_pose, grasp_quality=quality,
                                pre_grasp_approach=approach, post_grasp_retreat=retreat, max_contact_force=-1))

      #if requested, randomize the first few grasps
      if self.randomize_grasps:
        first_grasps = grasp_list[:30]
        random.shuffle(first_grasps)
        shuffled_grasp_list = first_grasps + grasp_list[30:]
        grasps = shuffled_grasp_list
      else:
        grasps = grasp_list

      return (grasps, error_code)

      #fill in the list of grasps
      for (grasp_pose, quality, pregrasp_dist) in zip(grasp_poses, qualities, pregrasp_dists):
        pre_grasp_joint_state = self.create_joint_trajectory(hand_joints, pregrasp_joint_angles, pregrasp_joint_efforts)
        grasp_joint_state = self.create_joint_trajectory(hand_joints, grasp_joint_angles, grasp_joint_efforts)

        #if the cluster isn't in the same frame as the graspable object reference frame,
        #transform the grasps to be in the reference frame
        if cluster_frame == target.reference_frame_id:
          transformed_grasp_pose = stamp_pose(grasp_pose, cluster_frame)
        else:
          transformed_grasp_pose = change_pose_stamped_frame(self.pcgp.tf_listener,
                                     stamp_pose(grasp_pose, cluster_frame),
                                     target.reference_frame_id)
        if self.pcgp.pregrasp_just_outside_box:
          min_approach_distance = pregrasp_dist
        else:
          min_approach_distance = max(pregrasp_dist-.05, .05)
        approach = GripperTranslation(create_vector3_stamped(hand_approach_direction, hand_frame), pregrasp_dist, min_approach_distance)
        retreat = GripperTranslation(create_vector3_stamped([-1.*x for x in hand_approach_direction], hand_frame), pregrasp_dist, min_approach_distance)

        # LT
        grasp_list.append(Grasp(id="id", pre_grasp_posture=pre_grasp_joint_state, grasp_posture=grasp_joint_state,
                                grasp_pose=transformed_grasp_pose, grasp_quality=quality,
                                pre_grasp_approach=approach, post_grasp_retreat=retreat, max_contact_force=-1))

      #if requested, randomize the first few grasps
      if self.randomize_grasps:
        first_grasps = grasp_list[:30]
        random.shuffle(first_grasps)
        shuffled_grasp_list = first_grasps + grasp_list[30:]
        grasps = shuffled_grasp_list
      else:
        grasps = grasp_list

      return (grasps, error_code)

      #fill in the list of grasps
      for (grasp_pose, quality, pregrasp_dist) in zip(grasp_poses, qualities, pregrasp_dists):
        pre_grasp_joint_state = self.create_joint_trajectory(hand_joints, pregrasp_joint_angles, pregrasp_joint_efforts)
        grasp_joint_state = self.create_joint_trajectory(hand_joints, grasp_joint_angles, grasp_joint_efforts)

        #if the cluster isn't in the same frame as the graspable object reference frame,
        #transform the grasps to be in the reference frame
        if cluster_frame == target.reference_frame_id:
          transformed_grasp_pose = stamp_pose(grasp_pose, cluster_frame)
        else:
          transformed_grasp_pose = change_pose_stamped_frame(self.pcgp.tf_listener,
                                   stamp_pose(grasp_pose, cluster_frame),
                                   target.reference_frame_id)
        if self.pcgp.pregrasp_just_outside_box:
          min_approach_distance = pregrasp_dist
        else:
          min_approach_distance = max(pregrasp_dist-.05, .05)
        approach = GripperTranslation(create_vector3_stamped(hand_approach_direction, hand_frame), pregrasp_dist, min_approach_distance)
        retreat = GripperTranslation(create_vector3_stamped([-1.*x for x in hand_approach_direction], hand_frame), pregrasp_dist, min_approach_distance)

        # LT
        grasp_list.append(Grasp(id="id", pre_grasp_posture=pre_grasp_joint_state, grasp_posture=grasp_joint_state,
                              grasp_pose=transformed_grasp_pose, grasp_quality=quality,
                              pre_grasp_approach=approach, post_grasp_retreat=retreat, max_contact_force=-1))

      #if requested, randomize the first few grasps
      if self.randomize_grasps:
        first_grasps = grasp_list[:30]
        random.shuffle(first_grasps)
        shuffled_grasp_list = first_grasps + grasp_list[30:]
        grasps = shuffled_grasp_list
      else:
        grasps = grasp_list

      return (grasps, error_code)


if __name__ == '__main__':
  rospy.init_node('probabilistic_point_cluster_grasp_planner', anonymous=False)
  pcgps = ProbabilisticPointClusterGraspPlannerServer()
  rospy.loginfo("probabilistic point cluster grasp planner is ready for queries")
  rospy.spin()                   
