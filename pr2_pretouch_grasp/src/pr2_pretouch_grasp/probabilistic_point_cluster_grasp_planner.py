import rospy
from geometry_msgs.msg import Pose
from uw_pr2_gripper_grasp_planner_cluster.convert_functions import *
from uw_pr2_gripper_grasp_planner_cluster.point_cluster_grasp_planner import PointClusterGraspPlanner
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject
from shape_msgs.msg import SolidPrimitive

class ProbabilisticPointClusterGraspPlanner(PointClusterGraspPlanner):

  def __init__(self, tf_listener = None, tf_broadcaster = None):
    PointClusterGraspPlanner.__init__(self)
    self.draw_box = True

  ##draw the fitted box model of the object, placed at identy pose (4x4 scipy matrix) in frame (defaults to the object frame)
  def draw_bounding_box(self, ranges, pause_after_broadcast = 0, name = 'deterministic_object_box', color=[0,0.5,1]):
    #broadcast the gripper frame to tf
    (object_frame_pos, object_frame_quat) = mat_to_pos_and_quat(self.object_to_cluster_frame)
    self.tf_broadcaster.sendTransform(object_frame_pos, object_frame_quat, rospy.Time.now(), "object_frame", self.cluster_frame)
    now = rospy.Time.now()
    if pause_after_broadcast:
      time.sleep(.1)

    pose_mat = scipy.matrix(scipy.identity(4))
    # z offset
    pose_mat[2,3] = ranges[2]/2
    self.draw_functions.draw_rviz_box(pose_mat, ranges, frame = 'object_frame', ns = name, \
                                              id = 0, color = color, duration = 100, opaque = .7)

  ##initialization for planning grasps 
  def init_probabilistic_cluster_grasper(self, cluster, probabilities=[], use_probability=True):

    self.cluster_frame = cluster.header.frame_id

    #use PCA to find the object frame and bounding box dims, and to get the cluster points in the object frame (z=0 at bottom of cluster)
    if use_probability:
      if len(probabilities) == 0:
        (self.object_points, self.object_bounding_box_dims, self.object_bounding_box, \
          self.object_to_base_frame, self.object_to_cluster_frame) = \
          self.cbbf.find_object_frame_and_bounding_box(cluster, []) #empty probabilities
        if self.draw_box:
          self.draw_bounding_box(self.object_bounding_box_dims, name='probabilistic_object_box', color=[0.2,0.2,1])
          print 'probabilistic_object_box'
      else:
        (self.object_points, self.object_bounding_box_dims, self.object_bounding_box, \
            self.object_to_base_frame, self.object_to_cluster_frame) = \
            self.cbbf.find_object_frame_and_bounding_box(cluster, probabilities)
        #draw the bounding box
        if self.draw_box:
          self.draw_bounding_box(self.object_bounding_box_dims, name='probabilistic_weighted_object_box', color=[0,0.9,0.9])
          print 'probabilistic_weighted_object_box'

    else:
      (self.object_points, self.object_bounding_box_dims, self.object_bounding_box, \
              self.object_to_base_frame, self.object_to_cluster_frame) = \
              self.cbbf.find_object_frame_and_bounding_box(cluster, probabilities)
      if self.draw_box:
        self.draw_bounding_box(self.object_bounding_box_dims, name='deterministic_object_box', color=[1,0,0])
        print 'deterministic_object_box'

    # MoveIt Stuff: CollisionObject
    print "self.object_bounding_box_dims=", self.object_bounding_box_dims
    print "self.object_bounding_box=", self.object_bounding_box
    print "self.object_to_base_frame.shape=", self.object_to_base_frame.shape
    print "self.object_to_base_frame=", self.object_to_base_frame
    print "self.object_to_cluster_frame=", self.object_to_cluster_frame

    # Add the bounding box as a CollisionObject
    co = CollisionObject()
    co.header.stamp = rospy.get_rostime()
    #co.header.frame_id = "base_footprint"
    co.header.frame_id = "base_link"

    co.primitives.append(SolidPrimitive())
    co.primitives[0].type = SolidPrimitive.BOX

    # Clear the previous CollisionObject
    co.id = "part"
    co.operation = co.REMOVE
    self.pub_co.publish(co)

    # Clear the previously attached object
    aco = AttachedCollisionObject()
    aco.object = co
    self.pub_aco.publish(aco)

    # Add the new CollisionObject
    box_height = self.object_bounding_box_dims[2]

    co.operation = co.ADD
    co.primitives[0].dimensions.append(self.object_bounding_box_dims[0])
    co.primitives[0].dimensions.append(self.object_bounding_box_dims[1])
    co.primitives[0].dimensions.append(self.object_bounding_box_dims[2])
    co.primitive_poses.append(Pose())
    co.primitive_poses[0].position.x = self.object_to_base_frame[0,3]
    co.primitive_poses[0].position.y = self.object_to_base_frame[1,3]
    co.primitive_poses[0].position.z = self.object_to_base_frame[2,3] + box_height/2

    quat = tf.transformations.quaternion_about_axis(math.atan2(self.object_to_base_frame[1,0], self.object_to_base_frame[0,0]), (0,0,1))
    #quat = tf.transformations.quaternion_from_matrix(self.object_to_base_frame)
    co.primitive_poses[0].orientation.x = quat[0]
    co.primitive_poses[0].orientation.y = quat[1]
    co.primitive_poses[0].orientation.z = quat[2]
    co.primitive_poses[0].orientation.w = quat[3]

    self.pub_co.publish(co)
    # END MoveIt! stuff

    #for which directions does the bounding box fit within the hand?
    gripper_space = [self.gripper_opening - self.object_bounding_box_dims[i] for i in range(3)]
    self._box_fits_in_hand = [gripper_space[i] > 0 for i in range(3)]

    #only half benefit for the longer dimension, if one is significantly longer than the other
    if self._box_fits_in_hand[0] and self._box_fits_in_hand[1]:
      if gripper_space[0] > gripper_space[1] and self.object_bounding_box_dims[0]/self.object_bounding_box_dims[1] < .8:
        self._box_fits_in_hand[1] *= .5
      elif gripper_space[1] > gripper_space[0] and self.object_bounding_box_dims[1]/self.object_bounding_box_dims[0] < .8:
        self._box_fits_in_hand[0] *= .5
    #rospy.loginfo("bounding box dims: "+pplist(self.object_bounding_box_dims))
    #rospy.loginfo("self._box_fits_in_hand: "+str(self._box_fits_in_hand))

    #compute the average number of points per sq cm of bounding box surface (half the surface only)
    bounding_box_surf = (self.object_bounding_box_dims[0]*100 * self.object_bounding_box_dims[1]*100) + \
        (self.object_bounding_box_dims[0]*100 * self.object_bounding_box_dims[2]*100) + \
        (self.object_bounding_box_dims[1]*100 * self.object_bounding_box_dims[2]*100)
    self._points_per_sq_cm = self.object_points.shape[1] / bounding_box_surf
    #rospy.loginfo("bounding_box_surf: %.2f, number of points: %d"%(bounding_box_surf, self.object_points.shape[1]))
    #rospy.loginfo("points per sq cm: %.4f"%self._points_per_sq_cm)

