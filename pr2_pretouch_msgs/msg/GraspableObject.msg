# an object that the object_manipulator can work on

# a graspable object can be represented in multiple ways. This message
# can contain all of them. Which one is actually used is up to the receiver
# of this message. When adding new representations, one must be careful that
# they have reasonable lightweight defaults indicating that that particular
# representation is not available.

# the tf frame to be used as a reference frame when combining information from
# the different representations below
string reference_frame_id

# potential recognition results from a database of models
# all poses are relative to the object reference pose
household_objects_database_msgs/DatabaseModelPose[] potential_models

# the point cloud itself
sensor_msgs/PointCloud cluster

# occupancy probability of each point
float32[] probabilities

# weight the object pointcloud with probabilities or not?
bool use_probability

# a region of a PointCloud2 of interest
manipulation_msgs/SceneRegion region

# the name that this object has in the collision environment
string collision_name

