/*
 * Copyright (c) 2014-2014, Liang-Ting Jiang
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
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (IN1CLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <pr2_pretouch_grasp/TabletopOctomapServer.h>

using namespace octomap;

namespace octomap_server{

/*////////////////////////////////////////////////////////////////////////////////////////
* Constructor
*/////////////////////////////////////////////////////////////////////////////////////////
TabletopOctomapServer::TabletopOctomapServer(ros::NodeHandle priv_nh_)
: OctomapServer(priv_nh_),
m_tabletopProcessingFrame("camera_rgb_optical_frame"),
m_camera_frame_id("camera_rgb_optical_frame"),
m_probPretouchHit(0.9), m_probPretouchMiss(0.1),
m_decay_rate_occlusion(4.0),
m_decay_rate_transparency(3.0),
m_object_idx(0) // temperary variable
{
  //tabletop octomap processing service
  m_tabletopProcessingService = m_nh.advertiseService("tabletop_octomap", 
                                    &TabletopOctomapServer::tabletopProcessingSrv, this);
  m_getProbabilisticPointCloudService = m_nh.advertiseService("get_probabilistic_pointcloud",
                                    &TabletopOctomapServer::getProbabilisticPointCloudSrv, this);
  m_addPointService = m_nh.advertiseService("add_point",
                                  &TabletopOctomapServer::addPointSrv, this);

  //publisher for visualize table pointclouds
  m_tablePointsPub = priv_nh_.advertise<sensor_msgs::PointCloud2>("table_points", 10);
  //publisher for visualize object pointclouds
  m_objectPointsPub = priv_nh_.advertise<sensor_msgs::PointCloud2>("object_points", 10);
  //publisher for visualize simulated table pointclouds
  m_simTablePointsPub = priv_nh_.advertise<sensor_msgs::PointCloud2>("sim_table_points", 10);
  //publisher for visualize simulated table pointclouds
  m_simTablePointsPub2 = priv_nh_.advertise<sensor_msgs::PointCloud2>("sim_table_points2", 10);
  //publisher for visualize intersecting pointclouds
  m_intersectPointsPub = priv_nh_.advertise<sensor_msgs::PointCloud2>("intersect_points", 10);
  //publisher for visualize shadow pointclouds
  m_shadowPointsPub = priv_nh_.advertise<sensor_msgs::PointCloud2>("shadow_points", 10);
  //publisher for visualize the real shadow pointclouds
  m_realShadowPointsPub = priv_nh_.advertise<sensor_msgs::PointCloud2>("real_shadow_points", 10);
  //publisher for visualize projected pointclouds
  m_projectedTablePointsPub = priv_nh_.advertise<sensor_msgs::PointCloud2>("projected_table_points", 10);
  //publisher for visualize endPoints pointclouds (debug)
  m_endPointsPub = priv_nh_.advertise<sensor_msgs::PointCloud2>("end_points", 10);
  m_markerPub = m_nh.advertise<visualization_msgs::MarkerArray>("occupied_cells_vis_array", 1, m_latchedTopics);

  //initialize parameters
  priv_nh_.param("frame_id", m_tabletopProcessingFrame, m_tabletopProcessingFrame);
  priv_nh_.param("camera_frame_id", m_camera_frame_id, m_camera_frame_id);
  priv_nh_.param("pretouch_sensor_model/hit", m_probPretouchHit, m_probPretouchHit);
  priv_nh_.param("pretouch_sensor_model/miss", m_probPretouchMiss, m_probPretouchMiss);
  priv_nh_.param<double>("decay_rate_occlusion", m_decay_rate_occlusion, m_decay_rate_occlusion);
  priv_nh_.param<double>("decay_rate_transparency", m_decay_rate_transparency, m_decay_rate_transparency);
  priv_nh_.param<int>("inlier_threshold", inlier_threshold_, 300);
  priv_nh_.param<double>("plane_detection_voxel_size", plane_detection_voxel_size_, 0.01);
  priv_nh_.param<double>("clustering_voxel_size", clustering_voxel_size_, 0.003);
  priv_nh_.param<double>("z_filter_min", z_filter_min_, 0.4);
  priv_nh_.param<double>("z_filter_max", z_filter_max_, 1.25);
  priv_nh_.param<double>("y_filter_min", y_filter_min_, -1.0);
  priv_nh_.param<double>("y_filter_max", y_filter_max_, 1.0);
  priv_nh_.param<double>("x_filter_min", x_filter_min_, -1.0);
  priv_nh_.param<double>("x_filter_max", x_filter_max_, 1.0);
  priv_nh_.param<double>("table_z_filter_min", table_z_filter_min_, 0.01);
  priv_nh_.param<double>("table_z_filter_max", table_z_filter_max_, 0.50);
  priv_nh_.param<double>("cluster_distance", cluster_distance_, 0.03);
  priv_nh_.param<int>("min_cluster_size", min_cluster_size_, 130);
  priv_nh_.param<std::string>("processing_frame", processing_frame_, "");
  priv_nh_.param<double>("up_direction", up_direction_, -1.0);
  priv_nh_.param<bool>("flatten_table", flatten_table_, false);
  priv_nh_.param<double>("table_padding", table_padding_, 0.0);
  priv_nh_.param<bool>("use_prob_map", m_useProbMap, false);
  priv_nh_.param<double>("sim_table_resolution", m_sim_table_res, 0.010);

  if(flatten_table_) ROS_DEBUG("flatten_table is true");
  else ROS_DEBUG("flatten_table is false");
}


/*////////////////////////////////////////////////////////////////////////////////////////
* Destructor
*/////////////////////////////////////////////////////////////////////////////////////////
TabletopOctomapServer::~TabletopOctomapServer() {}


/*////////////////////////////////////////////////////////////////////////////////////////
* Service callback for the tabletop_octomap service 
*/////////////////////////////////////////////////////////////////////////////////////////
bool TabletopOctomapServer::tabletopProcessingSrv(std_srvs::Empty::Request& req,
                                    std_srvs::Empty::Response& resp) {

  //record start time
  ros::WallTime startTime = ros::WallTime::now();
  //take a snapshot of the PointCloud2
  sensor_msgs::PointCloud2 snapshot_pc2 = getPointCloud2(); 
  //get PCL PointCloud of the table and objects from the whole PointCloud2
  PCLPointCloud::Ptr sim_table_pc (new PCLPointCloud);
  PCLPointCloud::Ptr projected_table_pc (new PCLPointCloud);
  PCLPointCloud::Ptr object_pc (new PCLPointCloud);
  objectDetection(snapshot_pc2, *object_pc, sim_table_pc, projected_table_pc);

  //save the original object_pc, for future inquiry
  pcl::copyPointCloud(*object_pc, m_object_pc);

  //get the global transform
  tf::StampedTransform sensorToWorldTf;
  std::cout << "m_worldFrameId =  " << m_worldFrameId <<std::endl;
  std::cout << "m_camera_frame_id =  " << m_camera_frame_id <<std::endl;
  std::cout << " object_pc->header.frame_id=  " << object_pc->header.frame_id <<std::endl;
  try {
    //convert the PCL timestamp to ROS timestamp
    ros::Time stamp();
    m_tfListener.lookupTransform(m_worldFrameId, m_camera_frame_id,
                        snapshot_pc2.header.stamp, sensorToWorldTf);
  } catch(tf::TransformException& ex){
    ROS_ERROR_STREAM( "Transform error of sensor data: " << ex.what() << ", quitting service");
    return false;
  }
  //get shadow pointcloud
  PCLPointCloud::Ptr shadow_pc (new PCLPointCloud);
  getShadow(sim_table_pc, projected_table_pc, shadow_pc);

  //assign prior in Octomap for sensed pointcloud
  KeySet obj_occupied_cells;
  initTabletopPrior(sensorToWorldTf.getOrigin(), *object_pc, *projected_table_pc, obj_occupied_cells);

  //assign prior in Octomap for suspicious transparent part
  initTransparentPrior(object_pc, shadow_pc, obj_occupied_cells, m_object_idx);	

  //publish pointcloud for visualization
  sensor_msgs::PointCloud2 pc2;
  pcl::toROSMsg( *sim_table_pc, pc2 );
  m_tablePointsPub.publish(pc2);

  //running time
  double total_elapsed = (ros::WallTime::now() - startTime).toSec();
  ROS_INFO("Tabletop Octomap created in TabletopOctomapServer done (%zu+%zu pts (object/table), %f sec)", 
                  object_pc->size(), sim_table_pc->size(), total_elapsed);

  //publish to all topics
  publishAllProbMap(snapshot_pc2.header.stamp);

  return true; 
}

/*////////////////////////////////////////////////////////////////////////////////////////
* Service callback for the get_probabilistic_pointcloud service 
*/////////////////////////////////////////////////////////////////////////////////////////
bool TabletopOctomapServer::getProbabilisticPointCloudSrv(pr2_pretouch_msgs::GetProbabilisticPointCloud::Request& req,
                                          pr2_pretouch_msgs::GetProbabilisticPointCloud::Response& resp) {
	
  //grab the augmented object pointcloud and its associated probabilities
  sensor_msgs::PointCloud object_cloud;
  std::vector<float> probabilities;
  getProbabilisticPointCloud (object_cloud, probabilities, m_object_idx);
  resp.cloud = object_cloud;
  resp.probabilities = probabilities;
  //Table frame
  geometry_msgs::Transform table_tf;
  tf::transformTFToMsg(m_table_plane_trans, table_tf);
  resp.table_tf = table_tf;

  sensor_msgs::PointCloud original_object_cloud;
  sensor_msgs::PointCloud2 pc2;
  pcl::toROSMsg( m_object_pc, pc2 );
  sensor_msgs::convertPointCloud2ToPointCloud (pc2, original_object_cloud);
  resp.original_cloud = original_object_cloud; 
  std::cout << "size of the deterministic pointcloud: " << original_object_cloud.points.size() << std::endl;
  std::cout << "size of the probabilistic pointcloud: " << object_cloud.points.size() << std::endl;

  return true;
}

bool TabletopOctomapServer::addPointSrv(pr2_pretouch_msgs::AddPoint::Request& req, pr2_pretouch_msgs::AddPoint::Response& resp) {
  
  std::cout << "adding a point in the octomap....: " << std::endl;
  //find the location of the point
  //transform the pose if required
  geometry_msgs::PoseStamped pose_stamped;
  if (req.pose_stamped.header.frame_id  != m_tabletopProcessingFrame) {
    std::cout << "transforming the location from " << req.pose_stamped.header.frame_id << " to the tabletop processing frame" << std::endl;
    int current_try=0, max_tries = 3;
    while (1)
    {
      bool transform_success = true;
      try
      {
        m_tfListener.transformPose(m_tabletopProcessingFrame, req.pose_stamped, pose_stamped);
      }
      catch (tf::TransformException ex)
      {
        transform_success = false;
        if (++current_try >= max_tries)
        {
          ROS_ERROR("Failed to transform Point Pose from frame %s into frame %s in %d attempt(s)", 
                 req.pose_stamped.header.frame_id.c_str(), m_tabletopProcessingFrame.c_str(), current_try);
          return false;
        }
        ROS_DEBUG("Failed to transform Point Pose, attempt %d out of %d, exception: %s", current_try, max_tries, ex.what());
        //sleep a bit to give the listener a chance to get a new transform
        ros::Duration(0.1).sleep();
      }
      if (transform_success) break;
    }
  } else {
    pose_stamped = req.pose_stamped;
  }

  // find the cell in tabletop_octomap
  point3d point(pose_stamped.pose.position.x, pose_stamped.pose.position.y, pose_stamped.pose.position.z);
  // set occupied point
  OcTreeKey key;
  if (m_octree->coordToKeyChecked(point, key)) {
    updateMinKey(key, m_updateBBXMin);
    updateMaxKey(key, m_updateBBXMax);
  }
  /*
  //the current LogOdd value
  float val;
  if (m_octree->search(key)) {
    val = m_octree->search(key)->getLogOdds(); //logodd
  } else {
   val = 0; //logodd
  }
  */

  //Hit or Miss?
  if (req.hit) {
    m_octree->updateNode(key, pToLogodd(m_probPretouchHit));
    //insert this point to the object keySet
		//put into the first object's index for now 
    m_objects_keys[0].insert(key);
		std::cout << "size of the object set increased to: " <<  m_objects_keys[0].size() << std::endl;
  } else {
    m_octree->updateNode(key, pToLogodd(m_probPretouchMiss));
  }
  //std::cout << "(original node) after: " << m_octree->search(key)->getLogOdds() << std::endl;

  //ray-casting to expand the result
  //compute end points
  //float deg_step = 0.523; //30 degree
  //float deg_step = 0.7853981633974483; // 45 degree
  float deg_step = 0.6; // ? degree
  float PI = 3.141592;
  float r = 3 * m_res; //1.5cm (the width of fingertip?)
  
  for (float alpha=0; alpha < 2*PI - deg_step; alpha += deg_step) {
    for (float beta=0; beta < PI - deg_step; beta += deg_step) {
      float x = point.x() + r * cos(alpha) * sin(beta);
      float y = point.y() + r * sin(alpha) * sin(beta); 
      float z = point.z() + r * cos(beta);
      point3d end_point (x,y,z);
      //ray-casting from point to end_point
      if (m_octree->computeRayKeys(point, end_point, m_keyRay)) { 
        float x = 0;
        int count = 0;
        for (KeyRay::const_iterator it = m_keyRay.begin()+1; it != m_keyRay.end(); it++) {
          x += m_res;
          if (req.hit) {
    				m_objects_keys[0].insert(key);
            m_octree->updateNode(*it, exp_dist(m_probPretouchHit, req.decay_rate, x));
						std::cout << "size of the object set increased to: " <<  m_objects_keys[0].size() << std::endl;
          } else {
            m_octree->updateNode(*it, exp_dist(m_probPretouchMiss, req.decay_rate, x));
          }
          count ++;
        }
      }
    }
  }

  resp.result = 1;
  publishAllProbMap(req.pose_stamped.header.stamp);
  return true;
}

/*////////////////////////////////////////////////////////////////////////////////////////
* 
*
*
*
*/////////////////////////////////////////////////////////////////////////////////////////
sensor_msgs::PointCloud2 TabletopOctomapServer::getPointCloud2() {

// -------------------- Step 1: Get a PointCloud2 from "cloud_in" ------------------ //
	ros::Time start_time = ros::Time::now();
	std::string topic = "cloud_in";
  ROS_INFO("objectDetection called; waiting for a point_cloud2 on topic %s", topic.c_str());
  sensor_msgs::PointCloud2::ConstPtr recent_cloud =
    ros::topic::waitForMessage<sensor_msgs::PointCloud2>(topic, m_nh, ros::Duration(3.0));

	if (!recent_cloud)
  {
    ROS_ERROR("objectDetection:: no PointCloud2 has been received");
  }

  ROS_INFO_STREAM("Point cloud received after " << ros::Time::now() - start_time << " seconds; processing");


// ------------- step 2: transform pointcloud2 to m_tabletopprocessingframe ---------//
  sensor_msgs::PointCloud old_cloud;
  sensor_msgs::convertPointCloud2ToPointCloud (*recent_cloud, old_cloud);
  int current_try=0, max_tries = 3;
  while (1)
  {
    bool transform_success = true;
    try
    {
      m_tfListener.transformPointCloud(m_tabletopProcessingFrame, old_cloud, old_cloud);
    }
    catch (tf::TransformException ex)
    {
      transform_success = false;
      if (++current_try >= max_tries)
      {
        ROS_ERROR("Failed to transform cloud from frame %s into frame %s in %d attempt(s)", 
									 old_cloud.header.frame_id.c_str(), m_tabletopProcessingFrame.c_str(), 
																																							current_try);
      }
      ROS_DEBUG("Failed to transform point cloud, attempt %d out of %d, exception: %s", current_try, max_tries, ex.what());
      //sleep a bit to give the listener a chance to get a new transform
      ros::Duration(0.1).sleep();
    }
    if (transform_success) break;
  }
  sensor_msgs::PointCloud2 converted_cloud;
  sensor_msgs::convertPointCloudToPointCloud2 (old_cloud, converted_cloud);
  ROS_INFO_STREAM("Input cloud converted to " << m_tabletopProcessingFrame << " frame after " <<
                  ros::Time::now() - start_time << " seconds");


// ------------------------ step 3: return the PointCloud2 ------------------------//
	return converted_cloud;
}



/*////////////////////////////////////////////////////////////////////////////////////////
* 
*
*
*
*/////////////////////////////////////////////////////////////////////////////////////////

void TabletopOctomapServer::objectDetection(const sensor_msgs::PointCloud2 &cloud,
																													PCLPointCloud &object_pc,
																													PCLPointCloud::Ptr sim_table_pc,
																													PCLPointCloud::Ptr projected_table_pc)
{
  ROS_INFO("Starting process on new cloud in frame %s", cloud.header.frame_id.c_str());

  // PCL objects
  KdTreePtr normals_tree_, clusters_tree_;
  pcl::VoxelGrid<Point> grid_, grid_objects_;
  pcl::PassThrough<Point> pass_;
  pcl::NormalEstimation<Point, pcl::Normal> n3d_;
  pcl::SACSegmentationFromNormals<Point, pcl::Normal> seg_;
  pcl::ProjectInliers<Point> proj_;
  pcl::ConvexHull<Point> hull_;
  pcl::ExtractPolygonalPrismData<Point> prism_;
  pcl::EuclideanClusterExtraction<Point> pcl_cluster_;
  pcl::PointCloud<Point>::Ptr table_hull_ptr (new pcl::PointCloud<Point>);

  // Filtering parameters
  grid_.setLeafSize (plane_detection_voxel_size_, plane_detection_voxel_size_, plane_detection_voxel_size_);
  grid_objects_.setLeafSize (clustering_voxel_size_, clustering_voxel_size_, clustering_voxel_size_);
  grid_.setFilterFieldName ("z");
  grid_.setFilterLimits (z_filter_min_, z_filter_max_);
  grid_.setDownsampleAllData (false);
  grid_objects_.setDownsampleAllData (true);

  normals_tree_ = boost::make_shared<pcl::search::KdTree<Point> > ();
  clusters_tree_ = boost::make_shared<pcl::search::KdTree<Point> > ();

  // Normal estimation parameters
  n3d_.setKSearch (10);
  n3d_.setSearchMethod (normals_tree_);
  // Table model fitting parameters
  seg_.setDistanceThreshold (0.05);
  seg_.setMaxIterations (10000);
  seg_.setNormalDistanceWeight (0.1);
  seg_.setOptimizeCoefficients (true);
  seg_.setModelType (pcl::SACMODEL_NORMAL_PLANE);
  seg_.setMethodType (pcl::SAC_RANSAC);
  seg_.setProbability (0.99);

  proj_.setModelType (pcl::SACMODEL_PLANE);

  // Clustering parameters
  pcl_cluster_.setClusterTolerance (cluster_distance_);
  pcl_cluster_.setMinClusterSize (min_cluster_size_);
  pcl_cluster_.setSearchMethod (clusters_tree_);

  // Step 1 : Filter, remove NaNs and downsample
  pcl::PointCloud<Point>::Ptr cloud_ptr (new pcl::PointCloud<Point>);
  pcl::fromROSMsg (cloud, *cloud_ptr);
  pass_.setInputCloud (cloud_ptr);
  pass_.setFilterFieldName ("z");
  pass_.setFilterLimits (z_filter_min_, z_filter_max_);
  pcl::PointCloud<Point>::Ptr z_cloud_filtered_ptr (new pcl::PointCloud<Point>);
  pass_.filter (*z_cloud_filtered_ptr);

  pass_.setInputCloud (z_cloud_filtered_ptr);
  pass_.setFilterFieldName ("y");
  pass_.setFilterLimits (y_filter_min_, y_filter_max_);
  pcl::PointCloud<Point>::Ptr y_cloud_filtered_ptr (new pcl::PointCloud<Point>);
  pass_.filter (*y_cloud_filtered_ptr);

  pass_.setInputCloud (y_cloud_filtered_ptr);
  pass_.setFilterFieldName ("x");
  pass_.setFilterLimits (x_filter_min_, x_filter_max_);
  pcl::PointCloud<Point>::Ptr cloud_filtered_ptr (new pcl::PointCloud<Point>);
  pass_.filter (*cloud_filtered_ptr);

	ROS_INFO("The size of the snapshot pointcloud: %lu", cloud_filtered_ptr->points.size());
  ROS_INFO("Step 1 done");
  if (cloud_filtered_ptr->points.size() < (unsigned int)min_cluster_size_)
  {
    ROS_INFO("Filtered cloud only has %d points", (int)cloud_filtered_ptr->points.size());
    return;
  }

  pcl::PointCloud<Point>::Ptr cloud_downsampled_ptr (new pcl::PointCloud<Point>);
  grid_.setInputCloud (cloud_filtered_ptr);
  grid_.filter (*cloud_downsampled_ptr);
  if (cloud_downsampled_ptr->points.size() < (unsigned int)min_cluster_size_)
  {
    ROS_INFO("Downsampled cloud only has %d points", (int)cloud_downsampled_ptr->points.size());
    return;
  }
  ROS_INFO("Downsampled cloud has %d points", (int)cloud_downsampled_ptr->points.size());

  // Step 2 : Estimate normals
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_ptr (new pcl::PointCloud<pcl::Normal>);
  n3d_.setInputCloud (cloud_downsampled_ptr);
  n3d_.compute (*cloud_normals_ptr);
  ROS_INFO("Step 2 done");

  // Step 3 : Perform planar segmentation
  tf::Transform table_plane_trans;
  tf::Transform table_plane_trans_flat;

    pcl::PointIndices::Ptr table_inliers_ptr (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr table_coefficients_ptr (new pcl::ModelCoefficients);
    seg_.setInputCloud (cloud_downsampled_ptr);
    seg_.setInputNormals (cloud_normals_ptr);
    seg_.segment (*table_inliers_ptr, *table_coefficients_ptr);
    m_table_coefficients_ptr = table_coefficients_ptr;

    if (table_coefficients_ptr->values.size () <=3)
    {
      ROS_INFO("Failed to detect table in scan");
      return;
    }

    if ( table_inliers_ptr->indices.size() < (unsigned int)inlier_threshold_)
    {
      ROS_INFO("Plane detection has %d inliers, below min threshold of %d", 
								(int)table_inliers_ptr->indices.size(), inlier_threshold_);
      return;
    }

    ROS_INFO ("Table Detection: Model found with %d inliers: [%f %f %f %f].",
              (int)table_inliers_ptr->indices.size (),
              table_coefficients_ptr->values[0], table_coefficients_ptr->values[1],
              table_coefficients_ptr->values[2], table_coefficients_ptr->values[3]);
    ROS_INFO("Step 3 done");

    // Step 4 : Project the table inliers on the table
		pcl::PointCloud<Point>::Ptr tmp_pc (new pcl::PointCloud<Point>);
    proj_.setInputCloud (cloud_downsampled_ptr);
    proj_.setIndices (table_inliers_ptr);
    proj_.setModelCoefficients (table_coefficients_ptr);
    proj_.filter (*tmp_pc); //LT: raeturn table_projected_ptr for table?

		//convert PointXYZRGB to PointXYZ
    PCLPointCloud::Ptr tmp2_pc (new PCLPointCloud);
		tmp2_pc->resize(tmp_pc->size());
		tmp2_pc->header.frame_id = tmp_pc->header.frame_id;
		for (size_t i = 0; i < tmp_pc->size(); i++) {
    	tmp2_pc->points[i].x = tmp_pc->points[i].x;
    	tmp2_pc->points[i].y = tmp_pc->points[i].y;
    	tmp2_pc->points[i].z = tmp_pc->points[i].z;
		}

    //compute the simulated table pointcloud from plane model coefficients
    //Remove the outlier in projected_table_pc
    //create the filtering object
    ROS_INFO("!!!!!!!!!!!!!!!!!!! size of tmp2_pc: %d", 
								(int)tmp2_pc->size());
    
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (tmp2_pc);
    sor.setMeanK (50);
    sor.setStddevMulThresh (2.0);
    sor.filter (*projected_table_pc);
    ROS_INFO("!!!!!!!!!!!!!!!!!!! size of projected_table_pc aftering outlier removal: %d", 
								(int)projected_table_pc->size());
  
    //Table simulation
    simTable(projected_table_pc, sim_table_pc, table_coefficients_ptr);

    ROS_INFO("Step 4 done");

    sensor_msgs::PointCloud table_points;
    sensor_msgs::PointCloud table_hull_points;
    table_plane_trans = getPlaneTransform (*table_coefficients_ptr, up_direction_, false);
    //save to class member
    m_table_plane_trans = table_plane_trans;
		//save the time stamp
		m_time_stamp = cloud.header.stamp;

    //estimate the convex hull (not in table frame)
    hull_.setInputCloud (tmp_pc);
    hull_.reconstruct (*table_hull_ptr);

    if(!flatten_table_)
    {
      //take the points projected on the table and transform them into the PointCloud message
      //while also transforming them into the table's coordinate system
      if (!getPlanePoints<Point> (*tmp_pc, table_plane_trans, table_points))
      {
        return;
      }

      //convert the convex hull points to table frame
      if (!getPlanePoints<Point> (*tmp_pc, table_plane_trans, table_hull_points))
      {
        return;
      }
    }
    if(flatten_table_)
    {
      //if flattening the table, find the center of the convex hull and move the table frame there
      table_plane_trans_flat = getPlaneTransform (*table_coefficients_ptr, up_direction_, flatten_table_);
      tf::Vector3 flat_table_pos;
      double avg_x, avg_y, avg_z;
      avg_x = avg_y = avg_z = 0;
      for (size_t i=0; i<projected_table_pc->points.size(); i++)
      {
        avg_x += projected_table_pc->points[i].x;
        avg_y += projected_table_pc->points[i].y;
        avg_z += projected_table_pc->points[i].z;
      }
      avg_x /= projected_table_pc->points.size();
      avg_y /= projected_table_pc->points.size();
      avg_z /= projected_table_pc->points.size();
      ROS_INFO("average x,y,z = (%.5f, %.5f, %.5f)", avg_x, avg_y, avg_z);

      //place the new table frame in the center of the convex hull
      flat_table_pos[0] = avg_x;
      flat_table_pos[1] = avg_y;
      flat_table_pos[2] = avg_z;
      table_plane_trans_flat.setOrigin(flat_table_pos);

      //shift the non-flat table frame to the center of the convex hull as well
      table_plane_trans.setOrigin(flat_table_pos);

      //take the points projected on the table and transform them into the PointCloud message
      //while also transforming them into the flat table's coordinate system
      sensor_msgs::PointCloud flat_table_points;
      if (!getPlanePoints<pcl::PointXYZ> (*projected_table_pc, table_plane_trans_flat, flat_table_points))
      {
        return;
      }

      //convert the convex hull points to flat table frame
      if (!getPlanePoints<Point> (*table_hull_ptr, table_plane_trans_flat, table_hull_points))
      {
        return;
      }
    }
    ROS_INFO("Table computed");
  
  // Step 5: Get the objects on top of the (non-flat) table
  pcl::PointIndices cloud_object_indices;
  prism_.setInputCloud (cloud_filtered_ptr);
  prism_.setInputPlanarHull (table_hull_ptr);
  ROS_INFO("Using table prism: %f to %f", table_z_filter_min_, table_z_filter_max_);
  prism_.setHeightLimits (table_z_filter_min_, table_z_filter_max_);
  prism_.segment (cloud_object_indices);

  pcl::PointCloud<Point>::Ptr cloud_objects_ptr (new pcl::PointCloud<Point>);
  pcl::ExtractIndices<Point> extract_object_indices;
  extract_object_indices.setInputCloud (cloud_filtered_ptr);
  extract_object_indices.setIndices (boost::make_shared<const pcl::PointIndices> (cloud_object_indices));
  extract_object_indices.filter (*cloud_objects_ptr);

  ROS_INFO (" Number of object point candidates: %d.", (int)cloud_objects_ptr->points.size ());

  if (cloud_objects_ptr->points.empty ())
  {
    ROS_INFO("No objects on table");
    return;
  }

  //downsample the points
  pcl::PointCloud<Point>::Ptr cloud_objects_downsampled_ptr (new pcl::PointCloud<Point>);
  grid_objects_.setInputCloud (cloud_objects_ptr);
  grid_objects_.filter (*cloud_objects_downsampled_ptr);

  // Step 6: Split the objects into Euclidean clusters
  std::vector<pcl::PointIndices> clusters2;
  pcl_cluster_.setInputCloud (cloud_objects_downsampled_ptr);
  pcl_cluster_.extract (clusters2);
  ROS_INFO ("Number of clusters found matching the given constraints: %d.", (int)clusters2.size ());

  //convert clusters into the PointCloud message
  std::vector<sensor_msgs::PointCloud> clusters;
	clusters.resize(clusters2.size());

	//reset the objects_keys
	m_objects_keys.resize(0);
	for (size_t i = 0; i < clusters2.size(); ++i)
  {	
		ROS_INFO("The %lu cluster", i);
    pcl::PointCloud<Point> cloud_cluster;
    pcl::copyPointCloud(*cloud_objects_downsampled_ptr, clusters2[i], cloud_cluster);
    sensor_msgs::PointCloud2 pc2;
    pcl::toROSMsg( cloud_cluster, pc2 );
    sensor_msgs::convertPointCloud2ToPointCloud (pc2, clusters[i]);
		ROS_INFO("The size of this object cloud: %lu", cloud_cluster.size());

		//save the object keys in the octomap
		KeySet* obj_keyset = new KeySet;		

		//only return the 1st object cloud for now
		if (i == m_object_idx) {
			object_pc.resize(cloud_cluster.size());
			for (size_t i = 0; i < cloud_cluster.points.size(); i++) {
    		object_pc.points[i].x = cloud_cluster.points[i].x;
    		object_pc.points[i].y = cloud_cluster.points[i].y;
    		object_pc.points[i].z = cloud_cluster.points[i].z;
				// insert this point to the object keySet
				point3d coord (cloud_cluster.points[i].x, cloud_cluster.points[i].y, cloud_cluster.points[i].z);
				OcTreeKey key;
				m_octree->coordToKeyChecked(coord, key);
				obj_keyset->insert(key);
			}
			//save the keyset representing this particular object
			m_objects_keys.push_back(*obj_keyset);

			object_pc.header.frame_id = cloud_cluster.header.frame_id;			
			object_pc.header.stamp = cloud_cluster.header.stamp;			

			//compute the object centroid
    	if (pcl::compute3DCentroid(object_pc, m_object_centroid) == 0) {
      	ROS_ERROR("cannot find the object centroid!!");
    	} else {
				std::cout << "object centroid: x=" << m_object_centroid[0] << ", y=" << m_object_centroid[1] << ", z=" <<
									   m_object_centroid[2] << " at frame: " << object_pc.header.frame_id <<   std::endl;
			}
		}
  }

  ROS_INFO("Clusters converted");

	//markers for rviz
  //publishClusterMarkers(clusters, cloud.header);

	//publish pointcloud for visualization
  sensor_msgs::PointCloud2 pc2;
  pcl::toROSMsg( object_pc, pc2 );
	m_objectPointsPub.publish(pc2);
	ROS_INFO("object pc published");

}



void TabletopOctomapServer::getShadow(const PCLPointCloud::Ptr sim_table_pc, 
																			const PCLPointCloud::Ptr projected_table_pc,
																		        PCLPointCloud::Ptr shadow_pc)                 
{
	float radius = m_sim_table_res*2;
  float shadow_clustering_tol = m_sim_table_res*2;

	ROS_INFO("finding the shadow on the table....");
	PCLPointCloud::Ptr shadow_pc_all (new PCLPointCloud);
	shadow_pc_all->header.frame_id = sim_table_pc->header.frame_id;
	shadow_pc->header.frame_id = sim_table_pc->header.frame_id;

	//search tree
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud (projected_table_pc);
	pcl::PointXYZ searchPoint;
 	std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;

	for (PCLPointCloud::const_iterator it = sim_table_pc->begin(); it != sim_table_pc->end(); ++it) {
		// search for neighbor in the radius
	  if ( kdtree.radiusSearch (*it, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) == 0 ) {
			shadow_pc_all->push_back(*it);
		}	
	}

	//(debug) How many points? What are the frames?
	std::cout << "shadow frame = " << shadow_pc_all->header.frame_id << std::endl;	
	std::cout << "sim table frame = " << sim_table_pc->header.frame_id  << std::endl;	
	std::cout << "projected table frame = " << projected_table_pc->header.frame_id  << std::endl;	

	//publish for visualization
  sensor_msgs::PointCloud2 pc2;
  pcl::toROSMsg( *shadow_pc_all, pc2 );
	m_shadowPointsPub.publish(pc2);
  pcl::toROSMsg( *projected_table_pc, pc2 );
  m_projectedTablePointsPub.publish(pc2);

	//Clustering to get the real shadow (tricky parts...)
	//Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (shadow_pc_all);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (shadow_clustering_tol); //PARAMETER
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (shadow_pc_all);
  ec.extract (cluster_indices);

	pcl::PCDWriter writer;
  int j = 0;
	std::vector<Eigen::Vector4f> centroids;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
		//compute the centroid of this cluster
		Eigen::Vector4f centroid;
		if (pcl::compute3DCentroid(*shadow_pc_all, *it, centroid) != 0) {
			centroids.push_back(centroid);
			std::cout << "shadow centroid #" << j << ": x=" << centroid[0] << ", y=" << centroid[1] << ", z=" <<
									   centroid[2] << std::endl;
			
		}	else {
			std::cout << "can't find centroid!!!" << std::endl;
		}	
		//(debug) write to PCD files for verification purposes
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
      cloud_cluster->points.push_back (shadow_pc_all->points[*pit]); //*
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    std::stringstream ss;
    ss << "cloud_cluster_" << j << ".pcd";
    writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
    j++;
  }
	//find the closet cluster to the object
	float minDist = 0;
	size_t minClusterIdx = 0;
	for (size_t i = 0; i < centroids.size(); i++) {
		float dist = sqrt( pow(m_object_centroid[0]-centroids[i][0],2) 
										 + pow(m_object_centroid[1]-centroids[i][1],2) 
										 + pow(m_object_centroid[2]-centroids[i][2],2) );
    std::cout << "computed the #" << i << " shadow, distance=" << dist <<  std::endl;
		if (i == 0) {
			minDist = dist;
      minClusterIdx = 0;
		}	
		else if (dist < minDist) {
			minDist = dist;
			minClusterIdx = i;
		}
	}
	
  //(debug)
  std::cout << "picked the #" << minClusterIdx << " shadow, distance=" << minDist <<  std::endl;

	//extract the real shadow
  pcl::ExtractIndices<pcl::PointXYZ> eifilter;
	eifilter.setInputCloud (shadow_pc_all);
 	eifilter.setIndices (boost::make_shared<const pcl::PointIndices> (cluster_indices[(int)minClusterIdx]));
	eifilter.filter (*shadow_pc);
	//visualize the real shadow
  std::cout << pc2.header.frame_id << ",  " << pc2.data.size() << std::endl;
  pcl::toROSMsg( *shadow_pc, pc2 );
  m_realShadowPointsPub.publish(pc2);

	ROS_INFO("finding the shadow on the table.... FINISHED!");
}

// Assign the prior in Octomap based on the estimated shadow pointcloud and object pointcloud
void TabletopOctomapServer::initTransparentPrior(const PCLPointCloud::Ptr object_pc, 
																								 const PCLPointCloud::Ptr shadow_pc,
																								 const KeySet &obj_occupied_cells,
																								 const int object_idx) {

	//Ray-Casting to Shadow PointCloud
  ROS_INFO("ray-casting to the Shadow Points");
	//container for the keys
  KeySet shadow_transparent_cells, obj_intersect_cells, obj_uncertain_cells;
  std::vector<OcTreeKey>::iterator it_ray;

	for (PCLPointCloud::const_iterator it = shadow_pc->begin(); it != shadow_pc->end(); it++) {
  	point3d point(it->x, it->y, it->z);

    float p0 = 0.5;
    //float inc = -0.10;
    float x = 0;
    // maxrange check
    if ((m_maxRange < 0.0) || ((point - m_sensorOrigin).norm() <= m_maxRange) ) {
      if (m_octree->computeRayKeys(m_sensorOrigin, point, m_keyRay)) {
				bool hit = false;
				//iterate the keys along the ray to find object occupied cells
				for (it_ray = m_keyRay.begin(); it_ray != m_keyRay.end(); it_ray++) {
					// Already hit, assigning occlusion uncertainty
					if (hit) {
						x += m_res;
						float val = exp_dist(p0, m_decay_rate_occlusion, x);
            m_octree->updateNode(*it_ray, val);      
		        //insert this point to the object keySet
						m_objects_keys[object_idx].insert(*it_ray);
          	//std::cout << "!!!!!!!!!!!!!!!assigned logodd value: " << val << " for x=" << dist << std::endl;
					} else {
						//check with all obj_occupied_cells
						for (KeySet::const_iterator it_obj = obj_occupied_cells.begin(), 
								            end=obj_occupied_cells.end(); it_obj!=end; it_obj++) {
							// Object intersecting ray
							// Already hit, assigning occlusion uncertainty
							if ((*it_ray) == (*it_obj)) {
              	obj_intersect_cells.insert(*it_ray); //insert this cell to intersect cells
              	x = 0; //reset the distance
              	hit = true;
								//ROS_INFO("this is a INTERSECTING ray");
								//get the probability of the starting cell
								p0 = m_octree->search(*it_ray)->getOccupancy(); 
							}
						}
					}
				}
				
				// Transparent Portion intersectin ray
				if (!hit) {
					//put all Nodes to shadow_transparent_cells
					shadow_transparent_cells.insert(m_keyRay.begin(), m_keyRay.end());
					//ROS_INFO("this is a transparent ray");
				}	
      }
      
    } 
  
  }

	//find the extending direction in table_frame
  geometry_msgs::Vector3Stamped direction;
	geometry_msgs::Vector3Stamped direction_transformed;
  //direction.header.stamp = shadow_pc->header.stamp; // LT new
  direction.header.stamp = m_time_stamp; //LT new
  direction.header.frame_id = "table_frame";
  direction.vector.x = 0.0;
  direction.vector.y = 0.0;
  direction.vector.z = 1.0;

	/*
	//(debug) print the vector values
	std::cout << "(BEFORE TF) direction.vector.x=" << direction.vector.x << ", direction.vector.y=" <<
							 direction.vector.y << ", direction.vector.z=" << direction.vector.z << std::endl;
	std::cout << "FRAME_ID DIRECTION = " << direction.header.frame_id << std::endl;
	*/

  //transform the direction to the original frame
  tf::TransformListener listener;
  tf::StampedTransform table_pose_frame(m_table_plane_trans, m_time_stamp,
                                        shadow_pc->header.frame_id, "table_frame");
  listener.setTransform(table_pose_frame);
  std::string error_msg;
  if (!listener.canTransform("table_frame", shadow_pc->header.frame_id, m_time_stamp, &error_msg))
  {
    ROS_ERROR("Can not transform point cloud from table frame to %s; error %s",
        shadow_pc->header.frame_id.c_str(), error_msg.c_str());
    return;
  }
  int current_try=0, max_tries = 3;
  while (1)
  {
    bool transform_success = true;
    try
    {
      listener.transformVector(shadow_pc->header.frame_id, direction, direction);
    }
    catch (tf::TransformException ex)
    {
      transform_success = false;
      if ( ++current_try >= max_tries )
      {
        ROS_ERROR("Failed to transform point cloud from table_frame to %s; error %s",
                  shadow_pc->header.frame_id.c_str(), ex.what());
        return;
      }
      //sleep a bit to give the listener a chance to get a new transform
      ros::Duration(0.1).sleep();
    }
    if (transform_success) break;
  }
	/*
	//(debug) print the vector values
	std::cout << "(AFTER TF) direction.vector.x=" << direction.vector.x << ", direction.vector.y=" <<
							 direction.vector.y << ", direction.vector.z=" << direction.vector.z << std::endl;
	std::cout << "FRAME_ID DIRECTION = " << direction.header.frame_id << std::endl;
	*/
	//find the boundary points of the object
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
 	pcl::PointCloud<pcl::Normal>::Ptr object_normals (new pcl::PointCloud<pcl::Normal>);
	
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud (object_pc);
  ne.setSearchMethod (tree);
  ne.setRadiusSearch (0.03);  
	ne.compute (*object_normals);

 	pcl::PointCloud<pcl::Boundary> boundaries;
 	pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> est;
 	est.setInputCloud (object_pc);
 	est.setInputNormals (object_normals);
 	est.setRadiusSearch (0.05);   // 2cm radius
 	//est.setSearchMethod (typename pcl::search::KdTree<pcl::PointXYZ>::Ptr (new pcl::search::KdTree<pcl::PointXYZ>));
  est.setSearchMethod (pcl::search::KdTree<pcl::PointXYZ>::Ptr (new pcl::search::KdTree<pcl::PointXYZ>));
 	est.compute (boundaries);

	// store the boundary indices
	pcl::PointIndices::Ptr boundary_indices (new pcl::PointIndices);
	for (size_t i = 0; i < object_pc->size(); i++) {
		if (boundaries.points[i].boundary_point == 1) {
			boundary_indices->indices.push_back((int)i);
		}
	}

	//save the boundary pointcloud
  PCLPointCloud::Ptr boundary_pc (new PCLPointCloud);
  for (std::vector<int>::const_iterator pit = boundary_indices->indices.begin(); 
			 pit != boundary_indices->indices.end (); pit++) {
    boundary_pc->points.push_back (object_pc->points[*pit]);
	}
	boundary_pc->header.frame_id = object_pc->header.frame_id;
	boundary_pc->header.stamp = object_pc->header.stamp;
  boundary_pc->width = boundary_pc->points.size();
  boundary_pc->height = 1;
  boundary_pc->is_dense = true;

	//visualize the boundary pointcloud
  sensor_msgs::PointCloud2 pc2;
  pcl::toROSMsg( *boundary_pc, pc2 );
	m_simTablePointsPub2.publish(pc2);

	//transparent estimator parameters
	double extend_length = 0.20; //10cm
		
	//(debug) save end points
	PCLPointCloud::Ptr end_pc (new PCLPointCloud);
	end_pc->header.frame_id = shadow_pc->header.frame_id;
	end_pc->header.stamp = shadow_pc->header.stamp;

  //Upward
	point3d vector_up (extend_length*direction.vector.x, extend_length*direction.vector.y, extend_length*direction.vector.z);
  for (PCLPointCloud::const_iterator it = boundary_pc->begin(); it != boundary_pc->end(); ++it) {
    point3d point (it->x, it->y, it->z);
    point3d point_up = point + vector_up;

		//(debug)
		pcl::PointXYZ p ((it->x)+extend_length*direction.vector.x,
										 (it->y)+extend_length*direction.vector.y,
                     (it->z)+extend_length*direction.vector.z);

    if (m_octree->computeRayKeys(point, point_up, m_keyRay)) {
			//the first-pass to decide do this ray or not
			//it is slower but right now no other way to do it
			bool found = false;
			int check_steps = 2;
			for (KeyRay::const_iterator cit = m_keyRay.begin(); cit != m_keyRay.end(); cit++) {
				//std::cout << "address of the OcTreeNode: " << (m_octree->search(*cit)) << std::endl;
				if ( (cit - m_keyRay.begin() < check_steps) && ((m_octree->search(*cit)) != 0)) {
					if (m_octree->search(*cit)->getLogOdds() < 0) {
						//some grids on this ray is already assign as free, don't ruin it
						found = true; 
					}
				} else if (m_octree->search(*cit) == 0)  {
					//trick: the node is not there, create it by updating the value = 0 (p=0.5)
					float uniform_logodd = -0.1;
					m_octree->updateNode(*cit , uniform_logodd);
				  //m_octree->updateNode(*it, true);
				}

			}

			//the second pass to update the uncertainty
			if (!found) {
        //std::cout << "assigning value to this ray!!" << std::endl;
        float x = 0 + m_res;
				//get the probability of the starting cell
				float p0 = m_octree->search(*m_keyRay.begin())->getOccupancy(); 
        for (it_ray = m_keyRay.begin()+1; it_ray != m_keyRay.end(); it_ray++) {
          float logodd = exp_dist(p0, m_decay_rate_transparency, x);
          if (logodd < -3.0) { //value too small, neglect it (p ~= 0.05)
        		//std::cout << "value too small, STOP!!" << std::endl;
            break;
          } else {
						if ((m_octree->search(*it_ray)) != 0) { //this node exists
							if (m_octree->search(*it_ray)->getLogOdds() <= 0) { //this is not a sensed object nodes already
								for (KeySet::iterator it = shadow_transparent_cells.begin(), end=shadow_transparent_cells.end(); it!=end; it++) {
									if ((*it_ray) == (*it)) { //intersecting with the transparent ray!
          					//std::cout << "!!!!!!!!!!!!!!!assigned logodd value: " << logodd << " for x=" << x << std::endl;
	              		m_octree->updateNode(*it_ray, logodd); //finally assign uncertainty for the transparent cell
		        				// insert this point to the object keySet
				        		//obj_uncertain_cells->insert(*it_ray);
										m_objects_keys[object_idx].insert(*it_ray);
									} else {
										std::cout << "Not on transparent ray" << std::endl;
									}
								}
							}
						} else {
							ROS_INFO("BADD----- THIS NODE DOESN'T EXIST"); //shouldn't happen
						}
          } 
  	    	x += m_res; //step one octomap grid
        }
      } else {
 	      std::cout << "NOT assigning value to this ray!!" << std::endl;
			}
    }
  } 

  //Downward
  for (PCLPointCloud::const_iterator it = boundary_pc->begin(); it != boundary_pc->end(); ++it) {
    point3d point (it->x, it->y, it->z);
    point3d point_down = point - vector_up;

		//(debug)
		pcl::PointXYZ p ((it->x)-extend_length*direction.vector.x,
										 (it->y)-extend_length*direction.vector.y,
                     (it->z)-extend_length*direction.vector.z);
		end_pc->points.push_back(p);

    if (m_octree->computeRayKeys(point, point_down, m_keyRay)) {
			//the first-pass to decide do this ray or not
			//it is slower but right now no other way to do it
			bool found = false;
			int check_steps = 2;
			for (KeyRay::const_iterator cit = m_keyRay.begin(); cit != m_keyRay.end(); cit++) {
				//std::cout << "address of the OcTreeNode: " << (m_octree->search(*cit)) << std::endl;
				if ( (cit - m_keyRay.begin() < check_steps) && ((m_octree->search(*cit)) != 0)) {
					if (m_octree->search(*cit)->getLogOdds() < 0) {
						//some grids on this ray is already assign as free, don't ruin it
						found = true; 
					}
				} else if (m_octree->search(*cit) == 0)  {
					//trick: the node is not there, create it by updating the value = 0 (p=0.5)
					float uniform_logodd = -0.1;
					m_octree->updateNode(*cit , uniform_logodd);
				}
			}

			//the second pass to update the uncertainty
			if (!found) {
        float x = 0 + m_res;
				//float logodd = 0.5;
				//get the probability of the starting cell
				float p0 = m_octree->search(*m_keyRay.begin())->getOccupancy(); 
        for (it_ray = m_keyRay.begin()+1; it_ray != m_keyRay.end(); it_ray++) {
          float logodd = exp_dist(p0, m_decay_rate_transparency, x);
          if (logodd < -3.0) { //value too small, neglect it (p ~= 0.05)
            break;
          } else {
						if ((m_octree->search(*it_ray)) != 0) { //this node exists
							if (m_octree->search(*it_ray)->getLogOdds() <= 0) { //this is not a sensed object nodes already
								for (KeySet::iterator it = shadow_transparent_cells.begin(), end=shadow_transparent_cells.end(); it!=end; it++) {
									if ((*it_ray) == (*it)) { //intersecting with the transparent ray!
	              		m_octree->updateNode(*it_ray, logodd); //finally assign uncertainty for the transparent cell
		        				//insert this point to the object keySet
				        		//obj_uncertain_cells->insert(*it_ray);
										m_objects_keys[object_idx].insert(*it_ray);
									} else {
										std::cout << "Not on transparent ray" << std::endl;
									}
								}
							}
						} else {
							ROS_INFO("BADD----- THIS NODE DOESN'T EXIST"); //shouldn't happen
						}
          } 
  	    	x += m_res; //step one octomap grid
        }
      } else {
        std::cout << "NOT assigning value to this ray!!" << std::endl;
			}
    }
  } 

	//(debug) publish
  pcl::toROSMsg( *end_pc, pc2 );
	m_endPointsPub.publish(pc2);
}

/*! Assumes plane coefficients are of the form ax+by+cz+d=0, normalized */
tf::Transform TabletopOctomapServer::getPlaneTransform (pcl::ModelCoefficients coeffs, double up_direction, bool flatten_plane)
{
  ROS_ASSERT(coeffs.values.size() > 3);
  double a = coeffs.values[0], b = coeffs.values[1], c = coeffs.values[2], d = coeffs.values[3];
  //asume plane coefficients are normalized
  tf::Vector3 position(-a*d, -b*d, -c*d);
  tf::Vector3 z(a, b, c);

  //if we are flattening the plane, make z just be (0,0,up_direction)
  if(flatten_plane)
  {
    ROS_INFO("flattening plane");
    z[0] = z[1] = 0;
    z[2] = up_direction;
  }
  else
  {
    //make sure z points "up"
    ROS_DEBUG("in getPlaneTransform, z: %0.3f, %0.3f, %0.3f", z[0], z[1], z[2]);
    if ( z.dot( tf::Vector3(0, 0, up_direction) ) < 0)
    {
      z = -1.0 * z;
      ROS_INFO("flipped z");
    }
  }

  //try to align the x axis with the x axis of the original frame
  //or the y axis if z and x are too close too each other
  tf::Vector3 x(1, 0, 0);
  if ( fabs(z.dot(x)) > 1.0 - 1.0e-4) x = tf::Vector3(0, 1, 0);
  tf::Vector3 y = z.cross(x).normalized();
  x = y.cross(z).normalized();

  tf::Matrix3x3 rotation;
  rotation[0] = x;  // x
  rotation[1] = y;  // y
  rotation[2] = z;  // z
  rotation = rotation.transpose();
  tf::Quaternion orientation;
  rotation.getRotation(orientation);
  ROS_DEBUG("in getPlaneTransform, x: %0.3f, %0.3f, %0.3f", x[0], x[1], x[2]);
  ROS_DEBUG("in getPlaneTransform, y: %0.3f, %0.3f, %0.3f", y[0], y[1], y[2]);
  ROS_DEBUG("in getPlaneTransform, z: %0.3f, %0.3f, %0.3f", z[0], z[1], z[2]);
  return tf::Transform(orientation, position);
}


template <typename PointT>
bool TabletopOctomapServer::getPlanePoints (const pcl::PointCloud<PointT> &table,
         const tf::Transform& table_plane_trans,
         sensor_msgs::PointCloud &table_points)
{
  // Prepare the output 
  table_points.header = pcl_conversions::fromPCL(table.header); //LT new
  table_points.points.resize(table.points.size ());
  for (size_t i = 0; i < table.points.size (); ++i)
  {
    table_points.points[i].x = table.points[i].x;
    table_points.points[i].y = table.points[i].y;
    table_points.points[i].z = table.points[i].z;
  }

  // Transform the data
  tf::TransformListener listener;
  tf::StampedTransform table_pose_frame(table_plane_trans, table_points.header.stamp,
                                        table.header.frame_id, "table_frame");
  listener.setTransform(table_pose_frame);
  std::string error_msg;
  if (!listener.canTransform("table_frame", table_points.header.frame_id, table_points.header.stamp, &error_msg))
  {
    ROS_ERROR("Can not transform point cloud from frame %s to table frame; error %s",
        table_points.header.frame_id.c_str(), error_msg.c_str());
    return false;
  }
  int current_try=0, max_tries = 3;
  while (1)
  {
    bool transform_success = true;
    try
    {
      listener.transformPointCloud("table_frame", table_points, table_points);
    }
    catch (tf::TransformException ex)
    {
      transform_success = false;
      if ( ++current_try >= max_tries )
      {
        ROS_ERROR("Failed to transform point cloud from frame %s into table_frame; error %s",
                  table_points.header.frame_id.c_str(), ex.what());
        return false;
      }
      //sleep a bit to give the listener a chance to get a new transform
      ros::Duration(0.1).sleep();
    }
    if (transform_success) break;
  }
  //table_points.header.stamp = table.header.stamp;
  table_points.header = pcl_conversions::fromPCL(table.header); //LT new
  table_points.header.frame_id = "table_frame";
  return true;
}

// Overloaded. Return transformed PCLPointCloud instead od sensor_msgs::PointCloud
template <typename PointT>
bool TabletopOctomapServer::getPlanePoints (const pcl::PointCloud<PointT> &table,
         const tf::Transform& table_plane_trans,
         PCLPointCloud::Ptr pcl_points)
{
  sensor_msgs::PointCloud table_points;
  // Prepare the temp output (sensor msgs)
  //table_points.header = table.header;
  table_points.header = pcl_conversions::fromPCL(table.header); //LT new
  table_points.points.resize(table.points.size ());
  for (size_t i = 0; i < table.points.size (); ++i)
  {
    table_points.points[i].x = table.points[i].x;
    table_points.points[i].y = table.points[i].y;
    table_points.points[i].z = table.points[i].z;
  }

  // Transform the data
  tf::TransformListener listener;
  tf::StampedTransform table_pose_frame(table_plane_trans, table_points.header.stamp,
                                        table.header.frame_id, "table_frame");
  listener.setTransform(table_pose_frame);
  std::string error_msg;
  if (!listener.canTransform("table_frame", table_points.header.frame_id, table_points.header.stamp, &error_msg))
  {
    ROS_ERROR("Can not transform point cloud from frame %s to table frame; error %s",
        table_points.header.frame_id.c_str(), error_msg.c_str());
    return false;
  }
  int current_try=0, max_tries = 3;
  while (1)
  {
    bool transform_success = true;
    try
    {
      listener.transformPointCloud("table_frame", table_points, table_points);
    }
    catch (tf::TransformException ex)
    {
      transform_success = false;
      if ( ++current_try >= max_tries )
      {
        ROS_ERROR("Failed to transform point cloud from frame %s into table_frame; error %s",
                  table_points.header.frame_id.c_str(), ex.what());
        return false;
      }
      //sleep a bit to give the listener a chance to get a new transform
      ros::Duration(0.1).sleep();
    }
    if (transform_success) break;
  }
  //table_points.header.stamp = table.header.stamp;
  table_points.header = pcl_conversions::fromPCL(table.header); //LT new
  table_points.header.frame_id = "table_frame";

  //convert sensor_msgs::PointCloud to PCLPointCloud
  sensor_msgs::PointCloud2 pc2;
  sensor_msgs::convertPointCloudToPointCloud2 (table_points, pc2);
  pcl::fromROSMsg (pc2, *pcl_points);
  return true;
}


void TabletopOctomapServer::initTabletopPrior(const tf::Point& sensorOriginTf, 
                    					                const PCLPointCloud &object_pc, 
                                              const PCLPointCloud &projected_table_pc,
																							KeySet& obj_occupied_cells) {

  std::cout << "object_pc.header.frame_id =  " <<  object_pc.header.frame_id <<std::endl;
  std::cout << "projected_table_pc.header.frame_id =  " <<  object_pc.header.frame_id <<std::endl;

  ROS_INFO("assining initial prior around the table and objects...");
  point3d sensorOrigin = pointTfToOctomap(sensorOriginTf);
	m_sensorOrigin = sensorOrigin;

  if (!m_octree->coordToKeyChecked(sensorOrigin, m_updateBBXMin)
      || !m_octree->coordToKeyChecked(sensorOrigin, m_updateBBXMax))
  {
    ROS_ERROR_STREAM("Could not generate Key for origin "<<sensorOrigin);
  }

  // fisrt assign occupancy for the object using Kinect sensor model
  KeySet obj_free_cells;
  // object points: free on ray, occupied on endpoint:
  for (PCLPointCloud::const_iterator it = object_pc.begin(); it != object_pc.end(); ++it) {
    point3d point(it->x, it->y, it->z);
    // maxrange check
    if ((m_maxRange < 0.0) || ((point - sensorOrigin).norm() <= m_maxRange) ) {

      // free cells
      if (m_octree->computeRayKeys(sensorOrigin, point, m_keyRay)){
        obj_free_cells.insert(m_keyRay.begin(), m_keyRay.end());
      }
      // occupied endpoint
      OcTreeKey key;
      if (m_octree->coordToKeyChecked(point, key)){
        obj_occupied_cells.insert(key);

        updateMinKey(key, m_updateBBXMin);
        updateMaxKey(key, m_updateBBXMax);
      }
    } else {// ray longer than maxrange:;
      point3d new_end = sensorOrigin + (point - sensorOrigin).normalized() * m_maxRange;
      if (m_octree->computeRayKeys(sensorOrigin, new_end, m_keyRay)){
        obj_free_cells.insert(m_keyRay.begin(), m_keyRay.end());

        octomap::OcTreeKey endKey;
        if (m_octree->coordToKeyChecked(new_end, endKey)){
          updateMinKey(endKey, m_updateBBXMin);
          updateMaxKey(endKey, m_updateBBXMax);
        } else{
          ROS_ERROR_STREAM("Could not generate Key for endpoint "<<new_end);
        }
      }
    }
  }

  ROS_INFO("updating the object occupancy cells");
  // update the probability (Log-Odd)
  // assign free cells only if not seen occupied in this cloud
  for(KeySet::iterator it = obj_free_cells.begin(), end=obj_free_cells.end(); it!= end; ++it){
    if (obj_occupied_cells.find(*it) == obj_occupied_cells.end()){
      m_octree->updateNode(*it, false); //this is using the sensor model/probHitLog to update (-0.4) (p=0.4)
      //m_octree->updateNode(*it, -1.5); //this is using arbitrary log-odd value to update the node
      //ROS_INFO("object free cells updated: p=%f", m_octree->search(*it)->getLogOdds()); //-0.4
    }
  }

  // assign occupied cells:
  for (KeySet::iterator it = obj_occupied_cells.begin(), end=obj_occupied_cells.end(); it!= end; it++) {
		//float val = 3.0;
    m_octree->updateNode(*it, true); //this is using the sensor model/probMissLog to update (0.85) (p=0.7)
    //m_octree->updateNode(*it, val); //this is using the sensor model/probMissLog to update
    //ROS_INFO("object occupied cells updated: p=%f", m_octree->search(*it)->getLogOdds()); //0.85
  }

  ROS_INFO("finished updating the object occupancy cells");


  //Raycasting the projected table
  ROS_INFO("ray-casting for the table");
  KeySet table_free_cells, table_occupied_cells;

	for (PCLPointCloud::const_iterator it = projected_table_pc.begin(); it != projected_table_pc.end(); ++it) {
  	point3d point(it->x, it->y, it->z);

    // maxrange check
    if ((m_maxRange < 0.0) || ((point - sensorOrigin).norm() <= m_maxRange) ) {
      if (m_octree->computeRayKeys(sensorOrigin, point, m_keyRay)) {

        //free along the ray
        table_free_cells.insert(m_keyRay.begin(), m_keyRay.end());

        // occupied endpoint
        OcTreeKey key;
        if (m_octree->coordToKeyChecked(point, key)){
          table_occupied_cells.insert(key);
          updateMinKey(key, m_updateBBXMin);
          updateMaxKey(key, m_updateBBXMax);
        }
      }
      
    } else {// ray longer than maxrange:;
      point3d new_end = sensorOrigin + (point - sensorOrigin).normalized() * m_maxRange;
      if (m_octree->computeRayKeys(sensorOrigin, new_end, m_keyRay)){
        table_free_cells.insert(m_keyRay.begin(), m_keyRay.end());

        octomap::OcTreeKey endKey;
        if (m_octree->coordToKeyChecked(new_end, endKey)){
          updateMinKey(endKey, m_updateBBXMin);
          updateMaxKey(endKey, m_updateBBXMax);
        } else{
          ROS_ERROR_STREAM("Could not generate Key for endpoint "<<new_end);
        }
      }
    }
  
  }

  ROS_INFO("finished ray-casting for the table");

  //mark free table cells
  for(KeySet::iterator it = table_occupied_cells.begin(), end=table_occupied_cells.end(); it!= end; ++it){
    if (table_occupied_cells.find(*it) == table_occupied_cells.end()){
      m_octree->updateNode(*it, false); //this is using the sensor model/probHitLog to update
    }
  }

  // update occupied table cells:
  for (KeySet::iterator it = table_occupied_cells.begin(), end=table_occupied_cells.end(); it!= end; it++) {
    float val = 3.0;
    m_octree->updateNode(*it, val); //this is using an arbitrary Log-Odd to update
    //m_octree->updateNode(*it, true); //this is using the sensor model/probHitLog to update
  }
  // TODO: eval lazy+updateInner vs. proper insertion
  // non-lazy by default (updateInnerOccupancy() too slow for large maps)
  //m_octree->updateInnerOccupancy();
  octomap::point3d minPt, maxPt;
  ROS_DEBUG_STREAM("Bounding box keys (before): " << m_updateBBXMin[0] << " " <<m_updateBBXMin[1] << " " << m_updateBBXMin[2] << " / " <<m_updateBBXMax[0] << " "<<m_updateBBXMax[1] << " "<< m_updateBBXMax[2]);

  // TODO: we could also limit the bbx to be within the map bounds here (see publishing check)
  minPt = m_octree->keyToCoord(m_updateBBXMin);
  maxPt = m_octree->keyToCoord(m_updateBBXMax);
  ROS_DEBUG_STREAM("Updated area bounding box: "<< minPt << " - "<<maxPt);
  ROS_DEBUG_STREAM("Bounding box keys (after): " << m_updateBBXMin[0] << " " <<m_updateBBXMin[1] << " " << m_updateBBXMin[2] << " / " <<m_updateBBXMax[0] << " "<<m_updateBBXMax[1] << " "<< m_updateBBXMax[2]);

  if (m_compressMap) // LT:compress or not??
    m_octree->prune();

}


void TabletopOctomapServer::initOcclusionPrior(const tf::Point& sensorOriginTf, 
                    					                 const PCLPointCloud &object_pc, 
                                               const PCLPointCloud &shadow_pc) {

  ROS_INFO("assining initial prior around the table and objects...");
  point3d sensorOrigin = pointTfToOctomap(sensorOriginTf);

  if (!m_octree->coordToKeyChecked(sensorOrigin, m_updateBBXMin)
      || !m_octree->coordToKeyChecked(sensorOrigin, m_updateBBXMax))
  {
    ROS_ERROR_STREAM("Could not generate Key for origin "<<sensorOrigin);
  }

  // fisrt assign occupancy for the object using Kinect sensor model
  KeySet obj_free_cells, obj_occupied_cells;
  // object points: free on ray, occupied on endpoint:
  for (PCLPointCloud::const_iterator it = object_pc.begin(); it != object_pc.end(); ++it) {
    point3d point(it->x, it->y, it->z);
    // maxrange check
    if ((m_maxRange < 0.0) || ((point - sensorOrigin).norm() <= m_maxRange) ) {

      // free cells
      if (m_octree->computeRayKeys(sensorOrigin, point, m_keyRay)){
        obj_free_cells.insert(m_keyRay.begin(), m_keyRay.end());
      }
      // occupied endpoint
      OcTreeKey key;
      if (m_octree->coordToKeyChecked(point, key)){
        obj_occupied_cells.insert(key);

        updateMinKey(key, m_updateBBXMin);
        updateMaxKey(key, m_updateBBXMax);
      }
    } else {// ray longer than maxrange:;
      point3d new_end = sensorOrigin + (point - sensorOrigin).normalized() * m_maxRange;
      if (m_octree->computeRayKeys(sensorOrigin, new_end, m_keyRay)){
        obj_free_cells.insert(m_keyRay.begin(), m_keyRay.end());

        octomap::OcTreeKey endKey;
        if (m_octree->coordToKeyChecked(new_end, endKey)){
          updateMinKey(endKey, m_updateBBXMin);
          updateMaxKey(endKey, m_updateBBXMax);
        } else{
          ROS_ERROR_STREAM("Could not generate Key for endpoint "<<new_end);
        }

      }
    }
  }
  ROS_INFO("updating the object occupancy cells");
  // actually update the probability
  // mark free cells only if not seen occupied in this cloud
  for(KeySet::iterator it = obj_free_cells.begin(), end=obj_free_cells.end(); it!= end; ++it){
    if (obj_occupied_cells.find(*it) == obj_occupied_cells.end()){
      m_octree->updateNode(*it, false); //this is using the sensor model/probHitLog to update
      //m_octree->updateNode(*it, -1.5); //this is using arbitrary log-odd value to update the node
      //ROS_INFO("object free cells updated: p=%f", m_octree->search(*it)->getLogOdds()); //-0.4
    }
  }

  // now mark all occupied cells:
  for (KeySet::iterator it = obj_occupied_cells.begin(), end=obj_occupied_cells.end(); it!= end; it++) {
    m_octree->updateNode(*it, true); //this is using the sensor model/probMissLog to update
    //m_octree->updateNode(*it, 1.5); //this is using the sensor model/probMissLog to update
    //ROS_INFO("object occupied cells updated: p=%f", m_octree->search(*it)->getLogOdds()); //0.85
  }

  ROS_INFO("finished updating the object occupancy cells");

	// Raycasting: Table
  ROS_INFO("ray-casting for the table");

  KeySet table_free_cells, table_occupied_cells, intersect_cells;
	//in which frame??
	std::cout << "frame id: " << shadow_pc.header.frame_id << std::endl;
	
  int count1 = 0; //DEGUG

	for (PCLPointCloud::const_iterator it = shadow_pc.begin(); it != shadow_pc.end(); ++it) {
  	point3d point(it->x, it->y, it->z);

    float val_intersect = 3.0;
    float inc = -0.10;
    int dist = 0;
    bool found = false;
    // maxrange check
    if ((m_maxRange < 0.0) || ((point - sensorOrigin).norm() <= m_maxRange) ) {
      if (m_octree->computeRayKeys(sensorOrigin, point, m_keyRay)){
        // iterate along the ray
        std::vector<OcTreeKey>::iterator it_ray;
        for (it_ray=m_keyRay.begin(); it_ray!=m_keyRay.end(); it_ray++) {
          // check this key with all keys of the object occupied cells
          for (KeySet::iterator it_obj = obj_occupied_cells.begin(), end=obj_free_cells.end(); it_obj!=end; it_obj++) {
            if ((*it_ray) == (*it_obj)) { //this ray intersects with the object
              intersect_cells.insert(*it_ray); //insert this cell to intersect cells
              count1++;
              m_octree->updateNode(*it_ray, val_intersect);      
              dist = 0; //reset the distance
              found = true;
            }
          }
          if (dist>0) m_octree->updateNode(*it_ray, val_intersect + dist * inc);
          if (found)  dist++;
        }
				
				// this ray doesn't interesct with objects, maybe transparent portion
				// iterate along the ray again to assign prior for transparent grids
				if (!found) {
        	for (it_ray=m_keyRay.begin(); it_ray!=m_keyRay.end(); it_ray++) {
						
					}
				} 
	      //table_free_cells.insert(m_keyRay.begin(), m_keyRay.end());
      }
      
      // occupied endpoint
      OcTreeKey key;
      if (m_octree->coordToKeyChecked(point, key)){
        table_occupied_cells.insert(key);
        updateMinKey(key, m_updateBBXMin);
        updateMaxKey(key, m_updateBBXMax);
      }
    } else {// ray longer than maxrange:;
      point3d new_end = sensorOrigin + (point - sensorOrigin).normalized() * m_maxRange;
      if (m_octree->computeRayKeys(sensorOrigin, new_end, m_keyRay)){
        //table_free_cells.insert(m_keyRay.begin(), m_keyRay.end());

        octomap::OcTreeKey endKey;
        if (m_octree->coordToKeyChecked(new_end, endKey)){
          updateMinKey(endKey, m_updateBBXMin);
          updateMaxKey(endKey, m_updateBBXMax);
        } else{
          ROS_ERROR_STREAM("Could not generate Key for endpoint "<<new_end);
        }
      }
    }
  
  }

  ROS_INFO("finished ray-casting for the table");
  ROS_INFO("Number of intersected cells: %d", count1);

  // update occupied table cells:
  for (KeySet::iterator it = table_occupied_cells.begin(), end=table_occupied_cells.end(); it!= end; it++) {
    //m_octree->updateNode(*it, true); //this is using the sensor model/probMissLog to update
    float val = 2.0;
    m_octree->updateNode(*it, val); //this is using an arbitrary Log-Odd to update
    //ROS_INFO("table cells updated: p=%f", m_octree->search(*it)->getLogOdds()); //table: 2.0
  }

  // correlate the table grids and the object grids
  // TODO: eval lazy+updateInner vs. proper insertion
  // non-lazy by default (updateInnerOccupancy() too slow for large maps)
  //m_octree->updateInnerOccupancy();
  octomap::point3d minPt, maxPt;
  ROS_DEBUG_STREAM("Bounding box keys (before): " << m_updateBBXMin[0] << " " <<m_updateBBXMin[1] << " " << m_updateBBXMin[2] << " / " <<m_updateBBXMax[0] << " "<<m_updateBBXMax[1] << " "<< m_updateBBXMax[2]);

  // TODO: snap max / min keys to larger voxels by m_maxTreeDepth
//   if (m_maxTreeDepth < 16)
//   {
//      OcTreeKey tmpMin = getIndexKey(m_updateBBXMin, m_maxTreeDepth); // this should give us the first key at depth m_maxTreeDepth that is smaller or equal to m_updateBBXMin (i.e. lower left in 2D grid coordinates)
//      OcTreeKey tmpMax = getIndexKey(m_updateBBXMax, m_maxTreeDepth); // see above, now add something to find upper right
//      tmpMax[0]+= m_octree->getNodeSize( m_maxTreeDepth ) - 1;
//      tmpMax[1]+= m_octree->getNodeSize( m_maxTreeDepth ) - 1;
//      tmpMax[2]+= m_octree->getNodeSize( m_maxTreeDepth ) - 1;
//      m_updateBBXMin = tmpMin;
//      m_updateBBXMax = tmpMax;
//   }
  // TODO: we could also limit the bbx to be within the map bounds here (see publishing check)
  minPt = m_octree->keyToCoord(m_updateBBXMin);
  maxPt = m_octree->keyToCoord(m_updateBBXMax);
  ROS_DEBUG_STREAM("Updated area bounding box: "<< minPt << " - "<<maxPt);
  ROS_DEBUG_STREAM("Bounding box keys (after): " << m_updateBBXMin[0] << " " <<m_updateBBXMin[1] << " " << m_updateBBXMin[2] << " / " <<m_updateBBXMax[0] << " "<<m_updateBBXMax[1] << " "<< m_updateBBXMax[2]);

  if (m_compressMap) // LT:compress or not??
    m_octree->prune();
}


void TabletopOctomapServer::simTable(PCLPointCloud::Ptr table_projected_ptr, 
                                     PCLPointCloud::Ptr table_simulated_ptr, 
                                     pcl::ModelCoefficients::Ptr table_coefficients_ptr) {

    PCLPointCloud::Ptr projected_transformed (new PCLPointCloud);
    // transform the projected_table_points to the table_frame
    tf::Transform table_plane_trans;
    table_plane_trans = getPlaneTransform (*table_coefficients_ptr, up_direction_, false);
    if (!getPlanePoints<pcl::PointXYZ> (*table_projected_ptr, table_plane_trans, projected_transformed)) {
      return;
    }

    //Find the min,max in table_projected_ptr
    double x_min = 0;
    double x_max = 0;
    double y_min = 0;
    double y_max = 0;
    double z_min = 0;
    double z_max = 0;
    if (!projected_transformed->points.empty())
    {
      x_min = projected_transformed->points[0].x;
      x_max = projected_transformed->points[0].x;
      y_min = projected_transformed->points[0].y;
      y_max = projected_transformed->points[0].y;
      z_min = projected_transformed->points[0].z;
      z_max = projected_transformed->points[0].z;
    } 
    for (size_t i=1; i<projected_transformed->points.size(); ++i)
    {
      if (projected_transformed->points[i].x<x_min && projected_transformed->points[i].x>-3.0) x_min = projected_transformed->points[i].x;
      if (projected_transformed->points[i].x>x_max && projected_transformed->points[i].x< 3.0) x_max = projected_transformed->points[i].x;
      if (projected_transformed->points[i].y<y_min && projected_transformed->points[i].y>-3.0) y_min = projected_transformed->points[i].y;
      if (projected_transformed->points[i].y>y_max && projected_transformed->points[i].y< 3.0) y_max = projected_transformed->points[i].y;
      if (projected_transformed->points[i].z<z_min && projected_transformed->points[i].z>-3.0) z_min = projected_transformed->points[i].z;
      if (projected_transformed->points[i].z>z_max && projected_transformed->points[i].z< 3.0) z_max = projected_transformed->points[i].z;
    }
    std::cout << "FRAME=" << projected_transformed->header.frame_id << std::endl;
    std::cout << "x_min=" << x_min << std::endl;
    std::cout << "x_max=" << x_max << std::endl;
    std::cout << "y_min=" << y_min << std::endl;
    std::cout << "y_max=" << y_max << std::endl;
    std::cout << "z_min=" << z_min << std::endl;
    std::cout << "z_max=" << z_max << std::endl;

    //Start simulation
    sensor_msgs::PointCloud table_simulated;
    double res = m_sim_table_res;   // 10.0mm
    size_t nx = int((x_max - x_min) / res);
    size_t ny = int((y_max - y_min) / res);
    double z = (z_max + z_min) / 2;
    //assign values (Method2)
    for (size_t i=0; i<nx; ++i) {
      for (size_t j=0; j<ny; ++j) {
        //compute distance (from point to the table plane)
        double x = x_min+i*res;
        double y = y_min+j*res;
        geometry_msgs::Point32 point;
        point.x = x;
        point.y = y;
        point.z = z;
        table_simulated.points.push_back(point);
      }
    }

    //still in table_frame
    table_simulated.header.frame_id = "table_frame";
    //table_simulated.header.stamp = table_projected_ptr->header.stamp;
    table_simulated.header.stamp = m_time_stamp; //LT new

    //transform back to original frame
    tf::TransformListener listener;
    tf::StampedTransform table_pose_frame(table_plane_trans, m_time_stamp,
                                        table_projected_ptr->header.frame_id, "table_frame");

    listener.setTransform(table_pose_frame);
    std::string error_msg;
    if (!listener.canTransform(table_projected_ptr->header.frame_id, "table_frame", 
                               table_simulated.header.stamp, &error_msg))
    {
      ROS_ERROR("Can not transform point cloud from table frame to %s; error %s",
          table_projected_ptr->header.frame_id.c_str(), error_msg.c_str());
    }
    int current_try=0, max_tries = 3;
    while (1)
    {
      bool transform_success = true;
      try
      {
        listener.transformPointCloud(table_projected_ptr->header.frame_id, table_simulated, table_simulated);
      }
      catch (tf::TransformException ex)
      {
        transform_success = false;
        if ( ++current_try >= max_tries )
        {
          ROS_ERROR("Failed to transform point cloud from table_frame to %s; error %s",
                  table_projected_ptr->header.frame_id.c_str(), ex.what());
        }
        //sleep a bit to give the listener a chance to get a new transform
        ros::Duration(0.1).sleep();
      }
      if (transform_success) break;
    }

    //ROS message to PCLPointCloud
    sensor_msgs::PointCloud2 pc2;
    sensor_msgs::convertPointCloudToPointCloud2 (table_simulated, pc2);
    pcl::fromROSMsg(pc2, *table_simulated_ptr);

	  //publish pointcloud for visualization
    m_simTablePointsPub.publish(pc2);
    ROS_INFO("Published the simulated table pointcloud");
    ROS_INFO("size of *table_simulated_ptr pointcloud: %lu", table_simulated_ptr->size());
    std::cout << "FRAMES...:" << std::endl;
    std::cout << "projected table frame " << table_projected_ptr->header.frame_id << std::endl;
    std::cout << "sim table frame " << table_simulated_ptr->header.frame_id << std::endl;
}


void TabletopOctomapServer::fillHole(PCLPointCloud::Ptr cloud) {

  ROS_INFO("trying to fill the holes....");

  // Normal estimation*
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud);
  n.setInputCloud (cloud);
  n.setSearchMethod (tree);
  n.setKSearch (20);
  n.compute (*normals);
  //* normals should not contain the point normals + surface curvatures

  // Concatenate the XYZ and normal fields*
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
  pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
  //* cloud_with_normals = cloud + normals

  // Create search tree*
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
  tree2->setInputCloud (cloud_with_normals);

  // Initialize objects
  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
  pcl::PolygonMesh triangles;

  // Set the maximum distance between connected points (maximum edge length)
  gp3.setSearchRadius (5); //0.025

  // Set typical values for the parameters
  gp3.setMu (50); //2~2.5
  gp3.setMaximumNearestNeighbors (10000); //100
  gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
  gp3.setMinimumAngle(0); // 10 degrees = M_PI/18
  gp3.setMaximumAngle(2*M_PI/3); // 120 degrees = 2*M_PI/3
  gp3.setNormalConsistency(false);

  // Get result
  gp3.setInputCloud (cloud_with_normals);
  gp3.setSearchMethod (tree2);
  gp3.reconstruct (triangles);

  // Additional vertex information
  std::vector<int> parts = gp3.getPartIDs();
  std::vector<int> states = gp3.getPointStates();

  // Finish
  pcl::io::saveVTKFile ("mesh.vtk", triangles);
}


void TabletopOctomapServer::insertCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud) {
	// Do Nothing
}

void TabletopOctomapServer::getProbabilisticPointCloud (sensor_msgs::PointCloud &cloud,
																												std::vector<float> &probabilities,
																												const int object_idx) {
	cloud.points.resize(0);
	cloud.header.frame_id = m_tabletopProcessingFrame;
	cloud.header.stamp = m_time_stamp;
	probabilities.resize(0);

	for (KeySet::iterator it = m_objects_keys[object_idx].begin(); it != m_objects_keys[object_idx].end(); it++) {
		//check if occupancy value is larger than the threshold
	  float p = m_octree->search(*it)->getOccupancy();
		if (p >= 0.5) {
			point3d point = m_octree->keyToCoord(*it);
			geometry_msgs::Point32 point_ros;
			point_ros.x = point.x();
			point_ros.y = point.y();
			point_ros.z = point.z();
			cloud.points.push_back(point_ros);
			//save the occupancy probabilities
			probabilities.push_back(p);
		}
	}
	std::cout << "size of the probability: " << probabilities.size() << std::endl;
}


void TabletopOctomapServer::publishAllProbMap(const ros::Time& rostime) {
  ros::WallTime startTime = ros::WallTime::now();
  size_t octomapSize = m_octree->size();
  // TODO: estimate num occ. voxels for size of arrays (reserve)
  if (octomapSize <= 1){
    ROS_WARN("Nothing to publish, octree is empty");
    return;
  }

  bool publishFreeMarkerArray = m_publishFreeSpace && (m_latchedTopics || m_fmarkerPub.getNumSubscribers() > 0);
  bool publishMarkerArray = (m_latchedTopics || m_markerPub.getNumSubscribers() > 0);
  bool publishPointCloud = (m_latchedTopics || m_pointCloudPub.getNumSubscribers() > 0);
  bool publishBinaryMap = (m_latchedTopics || m_binaryMapPub.getNumSubscribers() > 0);
  bool publishFullMap = (m_latchedTopics || m_fullMapPub.getNumSubscribers() > 0);
  m_publish2DMap = (m_latchedTopics || m_mapPub.getNumSubscribers() > 0);

  // init markers for free space:
  visualization_msgs::MarkerArray freeNodesVis;
  // each array stores all cubes of a different size, one for each depth level:
  freeNodesVis.markers.resize(m_treeDepth+1);

  geometry_msgs::Pose pose;
  pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

  // init markers:
  visualization_msgs::MarkerArray occupiedNodesVis;
  // each array stores all cubes of a different size, one for each depth level:
  occupiedNodesVis.markers.resize(m_treeDepth+1);

  // init pointcloud:
  pcl::PointCloud<pcl::PointXYZ> pclCloud;

  // call pre-traversal hook:
  handlePreNodeTraversal(rostime);

  // now, traverse all leafs in the tree:
  for (OcTree::iterator it = m_octree->begin(m_maxTreeDepth),
      end = m_octree->end(); it != end; ++it)
  {
    bool inUpdateBBX = isInUpdateBBX(it);

    // call general hook:
    handleNode(it);
    if (inUpdateBBX)
      handleNodeInBBX(it);

    if (m_octree->isNodeOccupied(*it)){
      double z = it.getZ();
      if (z > m_occupancyMinZ && z < m_occupancyMaxZ)
      {
        double size = it.getSize();
        double x = it.getX();
        double y = it.getY();

        // Ignore speckles in the map:
        if (m_filterSpeckles && (it.getDepth() == m_treeDepth +1) && isSpeckleNode(it.getKey())){
          ROS_DEBUG("Ignoring single speckle at (%f,%f,%f)", x, y, z);
          continue;
        } // else: current octree node is no speckle, send it out

        handleOccupiedNode(it);
        if (inUpdateBBX)
          handleOccupiedNodeInBBX(it);


        //create marker:
        if (publishMarkerArray){
          unsigned idx = it.getDepth();
          assert(idx < occupiedNodesVis.markers.size());

          geometry_msgs::Point cubeCenter;
          cubeCenter.x = x;
          cubeCenter.y = y;
          cubeCenter.z = z;

          occupiedNodesVis.markers[idx].points.push_back(cubeCenter);
          if (m_useHeightMap){
            double minX, minY, minZ, maxX, maxY, maxZ;
            m_octree->getMetricMin(minX, minY, minZ);
            m_octree->getMetricMax(maxX, maxY, maxZ);

            //double h = (1.0 - std::min(std::max((cubeCenter.z-minZ)/ (maxZ - minZ), 0.0), 1.0)) *m_colorFactor;
            // ProbabilityMap
            double minP = 0;
            double maxP = 4;
            double prob = it->getLogOdds();
            double h = (1.0 - std::min(std::max((prob-minP)/ (maxP - minP), 0.0), 1.0)) *m_colorFactor;
            occupiedNodesVis.markers[idx].colors.push_back(heightMapColor(h));
          }
        }

        // insert into pointcloud:
        if (publishPointCloud)
          pclCloud.push_back(pcl::PointXYZ(x, y, z));

      }
    } else{ // node not occupied => mark as free in 2D map if unknown so far
      double z = it.getZ();
      if (z > m_occupancyMinZ && z < m_occupancyMaxZ)
      {
        handleFreeNode(it);
        if (inUpdateBBX)
          handleFreeNodeInBBX(it);

        if (m_publishFreeSpace){
          double x = it.getX();
          double y = it.getY();

          //create marker for free space:
          if (publishFreeMarkerArray){
            unsigned idx = it.getDepth();
            assert(idx < freeNodesVis.markers.size());

            geometry_msgs::Point cubeCenter;
            cubeCenter.x = x;
            cubeCenter.y = y;
            cubeCenter.z = z;

            freeNodesVis.markers[idx].points.push_back(cubeCenter);
          }
        }
      }
    }
  }

  // call post-traversal hook:
  handlePostNodeTraversal(rostime);

  // finish MarkerArray:
  if (publishMarkerArray){
    for (unsigned i= 0; i < occupiedNodesVis.markers.size(); ++i){
      double size = m_octree->getNodeSize(i);

      occupiedNodesVis.markers[i].header.frame_id = m_worldFrameId;
      occupiedNodesVis.markers[i].header.stamp = rostime;
      occupiedNodesVis.markers[i].ns = "map";
      occupiedNodesVis.markers[i].id = i;
      occupiedNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
      occupiedNodesVis.markers[i].scale.x = size;
      occupiedNodesVis.markers[i].scale.y = size;
      occupiedNodesVis.markers[i].scale.z = size;
      occupiedNodesVis.markers[i].color = m_color;


      if (occupiedNodesVis.markers[i].points.size() > 0)
        occupiedNodesVis.markers[i].action = visualization_msgs::Marker::ADD;
      else
        occupiedNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
    }
    m_markerPub.publish(occupiedNodesVis);
  }


  // finish FreeMarkerArray:
  if (publishFreeMarkerArray){
    for (unsigned i= 0; i < freeNodesVis.markers.size(); ++i){
      double size = m_octree->getNodeSize(i);

      freeNodesVis.markers[i].header.frame_id = m_worldFrameId;
      freeNodesVis.markers[i].header.stamp = rostime;
      freeNodesVis.markers[i].ns = "map";
      freeNodesVis.markers[i].id = i;
      freeNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
      freeNodesVis.markers[i].scale.x = size;
      freeNodesVis.markers[i].scale.y = size;
      freeNodesVis.markers[i].scale.z = size;
      freeNodesVis.markers[i].color = m_colorFree;


      if (freeNodesVis.markers[i].points.size() > 0)
        freeNodesVis.markers[i].action = visualization_msgs::Marker::ADD;
      else
        freeNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
    }

    m_fmarkerPub.publish(freeNodesVis);
  }


  // finish pointcloud:
  if (publishPointCloud){
    sensor_msgs::PointCloud2 cloud;
    pcl::toROSMsg (pclCloud, cloud);
    cloud.header.frame_id = m_worldFrameId;
    cloud.header.stamp = rostime;
    m_pointCloudPub.publish(cloud);
  }

  if (publishBinaryMap)
    publishBinaryOctoMap(rostime);

  if (publishFullMap)
    publishFullOctoMap(rostime);

  double total_elapsed = (ros::WallTime::now() - startTime).toSec();
  ROS_DEBUG("Map publishing in OctomapServer took %f sec", total_elapsed);
}

} //end of namespace
