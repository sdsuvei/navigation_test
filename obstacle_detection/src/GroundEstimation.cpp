/*
 * groundEstimation.cpp
 *
 *  Created on: Feb 10, 2015
 *      Author: mikkel
 */

#include "GroundEstimation.h"

#define MIN_PLANE_SIZE 100
#define GRID_RESOLUTION 1.0

//GroundEstimation::GroundEstimation() {
//	// TODO Auto-generated constructor stub
//
//}
//
//GroundEstimation::~GroundEstimation() {
//	// TODO Auto-generated destructor stub
//}

void GroundEstimation::extractPlane(const PointCloudConstPtr &cloudIn, const PointCloudConstPtr &groundCloud, const PointCloudConstPtr &objectCloud) {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr gridCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	extractPlane(cloudIn, groundCloud, objectCloud, gridCloud);
}

void GroundEstimation::extractPlane(const PointCloudConstPtr &cloudIn, const PointCloudConstPtr &groundCloud, const PointCloudConstPtr &objectCloud, const PointCloudConstPtr &gridCloud) {
	// Grid Minimum
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr gridCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::GridMinimum<pcl::PointXYZRGB> gridm(GRID_RESOLUTION); // Set grid resolution
	gridm.setInputCloud(cloudIn);
	gridm.filter(*gridCloud);

	// Step 3: Find the ground plane using RANSAC
	pcl::ModelCoefficients ground_coefficients;
	pcl::PointIndices ground_indices;
	pcl::SACSegmentation<pcl::PointXYZRGB> ground_finder;
	ground_finder.setOptimizeCoefficients(true);
	ground_finder.setModelType(pcl::SACMODEL_PLANE);
	ground_finder.setMethodType(pcl::SAC_RANSAC);
	ground_finder.setDistanceThreshold(0.15);
	ground_finder.setInputCloud(gridCloud);
	ground_finder.segment(ground_indices, ground_coefficients);

	// Step 3a. Extract the ground plane inliers
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ground_points(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::ExtractIndices<pcl::PointXYZRGB> extractor;
	extractor.setInputCloud(gridCloud);
	extractor.setIndices(boost::make_shared<pcl::PointIndices>(ground_indices));
	extractor.filter(*ground_points);

	// Work around: add points far away to represent a large enough plane (otherwise convex hull will create problems)
	pcl::PointXYZRGB p1;
	p1.x = 100.0;
	p1.y = 100.0;
	ground_points->push_back(p1);
	p1.x = 100.0;
	p1.y = -100.0;
	ground_points->push_back(p1);
	p1.x = -100.0;
	p1.y = 100.0;
	ground_points->push_back(p1);
	p1.x = -100.0;
	p1.y = -100.0;
	ground_points->push_back(p1);

	// Step 3b. Extract the ground plane outliers
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_points(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::ExtractIndices<pcl::PointXYZRGB> outlier_extractor;
	outlier_extractor.setInputCloud(gridCloud);
	outlier_extractor.setIndices(boost::make_shared<pcl::PointIndices>(ground_indices));
	outlier_extractor.setNegative(true);
	outlier_extractor.filter(*object_points);

	// Step 3c. Project the ground inliers
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::ProjectInliers<pcl::PointXYZRGB> proj;
	proj.setModelType(pcl::SACMODEL_PLANE);
	proj.setInputCloud(ground_points);
	proj.setModelCoefficients(boost::make_shared<pcl::ModelCoefficients>(ground_coefficients));
	proj.filter(*cloud_projected);

	// Step 3d. Create a Convex Hull representation of the projected inliers
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ground_hull(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::ConvexHull<pcl::PointXYZRGB> chull;
	chull.setInputCloud(cloud_projected);
	chull.reconstruct(*ground_hull);

	// Step 3e. Extract only those outliers that lie above the ground plane's convex hull
	pcl::PointIndices object_indices;
	pcl::ExtractPolygonalPrismData<pcl::PointXYZRGB> hull_limiter;
	hull_limiter.setInputCloud(cloudIn);
	hull_limiter.setInputPlanarHull(ground_hull);
	hull_limiter.setHeightLimits(0.3, 100.0);
	hull_limiter.segment(object_indices);

	// Ectract ground cloud
	pcl::ExtractIndices<pcl::PointXYZRGB> outlier_extractor2;
	outlier_extractor2.setInputCloud(cloudIn);
	outlier_extractor2.setIndices(boost::make_shared<pcl::PointIndices>(object_indices));
	outlier_extractor2.setNegative(true);
	outlier_extractor2.filter(*groundCloud);

	// Extract object cloud
	pcl::ExtractIndices<pcl::PointXYZRGB> object_extractor;
	object_extractor.setInputCloud(cloudIn);
	object_extractor.setIndices(boost::make_shared<pcl::PointIndices>(object_indices));
	object_extractor.filter(*objectCloud);
}

void GroundEstimation::extractParallelPlanes(const PointCloudConstPtr &cloudIn, std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>* groundClouds, const PointCloudConstPtr &objectCloud, const PointCloudConstPtr &gridCloud) {
	//groundClouds->push_back(6);

	// Grid Minimum
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr gridCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::GridMinimum<pcl::PointXYZRGB> gridm(GRID_RESOLUTION); // Set grid resolution
	gridm.setInputCloud(cloudIn);
	gridm.filter(*gridCloud);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr gridCloudRemaining(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::copyPointCloud(*gridCloud,*gridCloudRemaining);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudRemaining(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::copyPointCloud(*cloudIn,*cloudRemaining);
	std::vector<pcl::ModelCoefficients> ground_coefficients_v;
	pcl::ModelCoefficients ground_coefficients;
	int nGroundCloud = 0;
	int planeFittingDone = 0;
	while (planeFittingDone == 0)
	{
		// Step 3: Find the ground plane using RANSAC
		pcl::PointIndices ground_indices;
		int i=0;
		do
		{

			pcl::SACSegmentation<pcl::PointXYZRGB> ground_finder;
			ground_finder.setOptimizeCoefficients(true);
			if (nGroundCloud == 0)
				ground_finder.setModelType(pcl::SACMODEL_PLANE);
			else
			{
				ground_finder.setModelType(pcl::SACMODEL_PARALLEL_PLANE);

				// Find vector parallel to one of the previously found planes
				Eigen::Vector3f axis = Eigen::Vector3f(ground_coefficients_v[i].values[0],ground_coefficients_v[i].values[1],ground_coefficients_v[i].values[2]);
				Eigen::Vector3f axis2 = Eigen::Vector3f(1.0,1.0,1.0);
				axis = axis.cross(axis2);
				ground_finder.setAxis(axis);
				ground_finder.setEpsAngle( 5.0f * (M_PI/180.0f) );
			}
			ground_finder.setMethodType(pcl::SAC_RANSAC);
			ground_finder.setDistanceThreshold(0.05);
			ground_finder.setInputCloud(gridCloudRemaining);
			ground_finder.segment(ground_indices, ground_coefficients);

			if (ground_indices.indices.size() >= MIN_PLANE_SIZE)
				break;

			i++;
		}
		while (i<ground_coefficients_v.size()); // Run through all previously found planes. The new plane must be almost parallel to one of the others.

		ground_coefficients_v.push_back(ground_coefficients);

		if (ground_indices.indices.size() < MIN_PLANE_SIZE)
			break;

		//ROS_INFO("ground_indices length = %d",ground_indices.indices.size());

		// Step 3a. Extract the ground plane inliers
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr ground_points(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::ExtractIndices<pcl::PointXYZRGB> extractor;
		extractor.setInputCloud(gridCloudRemaining);
		extractor.setIndices(boost::make_shared<pcl::PointIndices>(ground_indices));
		extractor.filter(*ground_points);

		// Work around: add points far away to represent a large enough plane (otherwise convex hull will create problems)
		pcl::PointXYZRGB p1;
		p1.x = 100.0;
		p1.y = 100.0;
		ground_points->push_back(p1);
		p1.x = 100.0;
		p1.y = -100.0;
		ground_points->push_back(p1);
		p1.x = -100.0;
		p1.y = 100.0;
		ground_points->push_back(p1);
		p1.x = -100.0;
		p1.y = -100.0;
		ground_points->push_back(p1);

		// Remove plane inliers from gridCloudRemaining
		pcl::ExtractIndices<pcl::PointXYZRGB> extractorGridCloud;
		extractorGridCloud.setInputCloud(gridCloudRemaining);
		extractorGridCloud.setIndices(boost::make_shared<pcl::PointIndices>(ground_indices));
		extractorGridCloud.setNegative(true);
		extractorGridCloud.filter(*gridCloudRemaining);

		// Step 3b. Extract the ground plane outliers
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_points(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::ExtractIndices<pcl::PointXYZRGB> outlier_extractor;
		outlier_extractor.setInputCloud(gridCloudRemaining);
		outlier_extractor.setIndices(boost::make_shared<pcl::PointIndices>(ground_indices));
		outlier_extractor.setNegative(true);
		outlier_extractor.filter(*object_points);

		// Step 3c. Project the ground inliers
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::ProjectInliers<pcl::PointXYZRGB> proj;
		proj.setModelType(pcl::SACMODEL_PLANE);
		proj.setInputCloud(ground_points);
		proj.setModelCoefficients(boost::make_shared<pcl::ModelCoefficients>(ground_coefficients));
		proj.filter(*cloud_projected);

		// Step 3d. Create a Convex Hull representation of the projected inliers
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr ground_hull(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::ConvexHull<pcl::PointXYZRGB> chull;
		chull.setInputCloud(cloud_projected);
		chull.reconstruct(*ground_hull);

		// Step 3e. Extract only those outliers that lie above the ground plane's convex hull
		pcl::PointIndices object_indices;
		pcl::ExtractPolygonalPrismData<pcl::PointXYZRGB> hull_limiter;
		hull_limiter.setInputCloud(cloudIn);
		hull_limiter.setInputPlanarHull(ground_hull);
		hull_limiter.setHeightLimits(-0.15, 0.15);
		hull_limiter.segment(object_indices);

		// Extract ground cloud
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr groundCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		groundClouds->push_back(groundCloud);
		pcl::ExtractIndices<pcl::PointXYZRGB> outlier_extractor2;
		outlier_extractor2.setInputCloud(cloudIn);
		outlier_extractor2.setIndices(boost::make_shared<pcl::PointIndices>(object_indices));
		//outlier_extractor2.setNegative(true);
		outlier_extractor2.filter(*(*groundClouds)[nGroundCloud]);


		// Step 3e. Extract only those outliers that lie above the ground plane's convex hull
		pcl::PointIndices object_indices2;
		hull_limiter.setInputCloud(cloudRemaining);
		hull_limiter.setInputPlanarHull(ground_hull);
		hull_limiter.setHeightLimits(-0.15, 0.15);
		hull_limiter.segment(object_indices2);

		// Extract object cloud
		pcl::ExtractIndices<pcl::PointXYZRGB> outlier_extractor3;
		outlier_extractor3.setInputCloud(cloudRemaining);
		outlier_extractor3.setIndices(boost::make_shared<pcl::PointIndices>(object_indices2));
		outlier_extractor3.setNegative(true);
		outlier_extractor3.filter(*cloudRemaining);

		nGroundCloud++;
	}

	pcl::copyPointCloud(*cloudRemaining,*objectCloud);
	// Extract object cloud
//	pcl::ExtractIndices<pcl::PointXYZRGB> object_extractor;
//	object_extractor.setInputCloud(cloudIn);
//	object_extractor.setIndices(boost::make_shared<pcl::PointIndices>(object_indices));
//	object_extractor.filter(*objectCloud);
}
