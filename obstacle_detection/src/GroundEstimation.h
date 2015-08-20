/*
 * GroundEstimation.h
 *
 *  Created on: Feb 10, 2015
 *      Author: mikkel
 */

#ifndef GROUNDESTIMATION_H_
#define GROUNDESTIMATION_H_

#include <iostream>
#include <sstream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d.h>
#include <string>

#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_3d.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <vector>
#include <pcl/segmentation/region_growing.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/grid_minimum.h>
#include <pcl/segmentation/segment_differences.h>

#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/filters/project_inliers.h>

class GroundEstimation {
public:
	typedef typename pcl::PointXYZRGB PointT;
	typedef typename pcl::PointCloud<PointT> CloudT;
	typedef typename CloudT::Ptr PointCloudConstPtr;

	//GroundEstimation();
	void extractPlane(const PointCloudConstPtr &cloudIn, const PointCloudConstPtr &groundCloud, const PointCloudConstPtr &objectCloud);
	void extractPlane(const PointCloudConstPtr &cloudIn, const PointCloudConstPtr &groundCloud, const PointCloudConstPtr &objectCloud, const PointCloudConstPtr &gridCloud);

	void extractParallelPlanes(const PointCloudConstPtr &cloudIn, std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>* groundClouds, const PointCloudConstPtr &objectCloud, const PointCloudConstPtr &gridCloud);
	//virtual ~GroundEstimation();

};

#endif /* GROUNDESTIMATION_H_ */
