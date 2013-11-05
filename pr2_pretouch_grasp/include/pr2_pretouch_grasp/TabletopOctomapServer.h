/*
 * Copyright (c) 2012-2012, L.T. Jiang
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef OCTOMAP_SERVER_TABLETOPOCTOMAPSERVER_H
#define OCTOMAP_SERVER_TABLETOPOCTOMAPSERVER_H
#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET

#include <math.h>
#include <time.h>

#include <octomap_server/OctomapServer.h>
#include <octomap/OcTreeKey.h>

#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Transform.h>

#include <pr2_pretouch_msgs/GetProbabilisticPointCloud.h>
#include <pr2_pretouch_msgs/AddPoint.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <visualization_msgs/Marker.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_ros/transforms.h>
//#include <pcl/visualization/cloud_viewer.h>

#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <pcl/features/boundary.h>
#include <pcl/common/geometry.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl_conversions/pcl_conversions.h> // for converting headers

//#include <tabletop_object_detector/marker_generator.h>

#include <Eigen/Geometry> 

namespace octomap_server {
class TabletopOctomapServer : public OctomapServer{

	typedef pcl::PointCloud<pcl::PointXYZ> PCLPointCloud;
  typedef pcl::PointXYZRGB    Point;
  typedef pcl::search::KdTree<Point>::Ptr KdTreePtr;

public:
	
  TabletopOctomapServer();
  virtual ~TabletopOctomapServer();

	//override
	void insertCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud);

	//take a snapshot and get a PointCloud2 msg from /cloud_in topic
	sensor_msgs::PointCloud2 getPointCloud2();

	//service callback for /tabletop_octomap service
	bool tabletopProcessingSrv(std_srvs::Empty::Request& req,
                             std_srvs::Empty::Response& resp);
	//service callback for /get_probabilistic_pointcloud service
	bool getProbabilisticPointCloudSrv(pr2_pretouch_msgs::GetProbabilisticPointCloud::Request& req,
                                     pr2_pretouch_msgs::GetProbabilisticPointCloud::Response& resp);
	//service callback for /add_point service
	bool addPointSrv(pr2_pretouch_msgs::AddPoint::Request& req, pr2_pretouch_msgs::AddPoint::Response& resp);

	//Function to detect objects and tables
	void objectDetection(const sensor_msgs::PointCloud2 &cloud,
                             PCLPointCloud &object_pc,
                             PCLPointCloud::Ptr sim_table_pc,
														 PCLPointCloud::Ptr projected_table_pc);

	//find the shadow points on the table
	void getShadow(const PCLPointCloud::Ptr sim_table_pc,
                 const PCLPointCloud::Ptr projected_table_pc,
	                     PCLPointCloud::Ptr shadow_pc);

	//Compute the basic octomap around the table and objects
	void initTabletopPrior(const tf::Point& sensorOriginTf, 
                         const PCLPointCloud &object_pc, 
                         const PCLPointCloud &projected_table_pc,
												 octomap::KeySet &obj_occupied_cells);

	//Compute the octomap for occlusion part
	void initOcclusionPrior(const tf::Point& sensorOriginTf, 
                          const PCLPointCloud &object_pc, 
                          const PCLPointCloud &shadow_pc);
	//
	tf::Transform getPlaneTransform (pcl::ModelCoefficients coeffs, double up_direction, bool flatten_plane);

	//
	template <typename PointT>
	bool getPlanePoints (const pcl::PointCloud<PointT> &table, const tf::Transform& table_plane_trans, sensor_msgs::PointCloud &table_points);
 
	template <typename PointT>
	bool getPlanePoints (const pcl::PointCloud<PointT> &table, const tf::Transform& table_plane_trans, PCLPointCloud::Ptr pcl_points);

  //! Publishes rviz markers for the given tabletop clusters
  template <class PointCloudType>
  void publishClusterMarkers(const std::vector<PointCloudType> &clusters, std_msgs::Header cloud_header);
  //
  void fillHole(PCLPointCloud::Ptr cloud);
  //
  void simTable(PCLPointCloud::Ptr table_projected_ptr, 
                PCLPointCloud::Ptr table_simulated_ptr, 
                pcl::ModelCoefficients::Ptr table_coefficients_ptr);

	//get sensor_msgs::PointCloud and the probabilities of an object from m_octomap
	void getProbabilisticPointCloud (sensor_msgs::PointCloud &cloud,
                                   std::vector<float> &probabilities,
                                   const int object_idx);


  //void publishAll(const ros::Time& rostime = ros::Time::now());
	///tabletop_octomap service server
	ros::ServiceServer m_tabletopProcessingService;
	///get_probabilistic_pointcloud service server
	ros::ServiceServer m_getProbabilisticPointCloudService;
  //add pretouch point service
  ros::ServiceServer m_addPointService;


	/*//////////////////////////////////////
	/Probability Distrubution Functions
	*///////////////////////////////////////

	// exponential distribution function
	float exp_dist(float p0, float decay_rate, float x) const {
  	float p = p0 * exp(-decay_rate * x);
  	return pToLogodd(p);
	}

	// Gaussian distribution function (centered at x=0)
	float gaussian_dist(float p0, float var, float x) const {
		float theta = sqrt(var);
		float p = p0 * 0.39894 / theta * exp(-pow(x,2)/(2*var));
		return pToLogodd(p);
	}

	//convert probability to Logodd
	float pToLogodd(const float p) const { return log(p) - log(1-p); }

protected:

	void initTransparentPrior(const PCLPointCloud::Ptr object_pc,
											      const PCLPointCloud::Ptr shadow_pc,
														const octomap::KeySet &obj_occupied_cells,
														const int object_idx);

	ros::NodeHandle priv_nh_;
	ros::Publisher m_markerPub, m_objectPointsPub, m_tablePointsPub, m_intersectPointsPub,
							   m_simTablePointsPub, m_simTablePointsPub2, m_shadowPointsPub, m_projectedTablePointsPub,
								 m_realShadowPointsPub, m_endPointsPub;
  double m_probPretouchHit, m_probPretouchMiss;
  //the frame if of the camera
  std::string m_camera_frame_id;
  //estimator parameters
  double m_decay_rate_occlusion;
  double m_decay_rate_transparency;
  //! Use probability map to visualize the cells
  bool m_useProbMap;
  double m_sim_table_res;
	std::string m_tabletopProcessingFrame;

  //! Used to remember the number of markers we publish so we can delete them later
  int num_markers_published_;
  //! The current marker being published
  int current_marker_id_;

  //! Min number of inliers for reliable plane detection
  int inlier_threshold_;
  //! Size of downsampling grid before performing plane detection
  double plane_detection_voxel_size_;
  //! Size of downsampling grid before performing clustering
  double clustering_voxel_size_;
  //! Filtering of original point cloud along the z, y, and x axes
  double z_filter_min_, z_filter_max_;
  double y_filter_min_, y_filter_max_;
  double x_filter_min_, x_filter_max_;
  //! Filtering of point cloud in table frame after table detection
  double table_z_filter_min_, table_z_filter_max_;
  //! Min distance between two clusters
  double cluster_distance_;
  //! Min number of points for a cluster
  int min_cluster_size_;
  //! Clouds are transformed into this frame before processing; leave empty if clouds
  //! are to be processed in their original frame
  std::string processing_frame_;
  //! Positive or negative z is closer to the "up" direction in the processing frame?
  double up_direction_;
  bool flatten_table_;
  //! How much the table gets padded in the horizontal direction
  double table_padding_;
	//store the object cluster centroid
	Eigen::Vector4f m_object_centroid;
	//store the last timestamp
	ros::Time m_time_stamp;
  //store the table model
  pcl::ModelCoefficients::Ptr m_table_coefficients_ptr;
  tf::Transform m_table_plane_trans;
	octomap::point3d m_sensorOrigin; 	

	//store the object cells to identify the objects in octoMap
	std::vector<octomap::KeySet> m_objects_keys;

	//which object to work with? (a temperary variable)
	int m_object_idx;

  //save the original object_pc
  PCLPointCloud m_object_pc;

	//table tf broadcaster
	tf::TransformBroadcaster m_br;

};
}

#endif

