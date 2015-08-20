//#define PCL_NO_PRECOMPILE

#define OLD_METHOD false

#include <iostream>
#include <sstream>
#include <dirent.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d.h>
#include <string>
#include <math.h>

#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_3d.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <vector>
#include <algorithm>
#include <iterator>
//#include <pcl/segmentation/region_growing.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pcl/filters/statistical_outlier_removal.h>
//#include <pcl/filters/grid_minimum.h>
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

#include <pcl/features/moment_invariants.h>
#include "GlobalPlaneDistance.h"
//#include "GroundEstimation.h"

#include <pcl/common/transforms.h>
#include <pcl/filters/random_sample.h>

#include <pcl/octree/octree_impl.h>
#include <pcl/octree/octree_iterator.h>
#include <pcl/octree/octree_pointcloud_density.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/ml/ml.hpp>

#include "PrincipalComponentsAnalysis.h"
//#include "nn_classification.h"
#include "obstacle_detection.h"
#include "HelpingFunctions.h"
#include "NeighborhoodFeatures.h"

//#include <pcl/segmentation/region_growing_rgb.h>
//#include "region_growing2.h"
#include "region_growing.h"
//#include "region_growing_rgb2.h"
#include <pcl/common/centroid.h>
#include <obstacle_detection/boundingbox.h>
#include <obstacle_detection/boundingboxes.h>
#include <std_msgs/Float64MultiArray.h>


using namespace std;
using namespace cv;



void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* cookie)
{
	if (event.getKeySym () == "space" && event.keyDown ()) // Save annotation file
	{
		viewer->spinOnce();
	}
}

ros::Publisher* pubProcessedPointCloudPointer;
ros::Subscriber* subRawPointCloudPointer;
ros::Subscriber* sub2LinesPointer;
ros::Publisher* pubOccupancyGridPointer;
ros::Publisher* pubBBoxesPointer;
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudIn(new pcl::PointCloud<pcl::PointXYZI>());
float theta, r1, r2;
bool receivedHough = false;
int n;

void linesHandler(const std_msgs::Float64MultiArray& houghLines)
{
	theta = houghLines.data[0];
	r1 = houghLines.data[1];
	r2 = houghLines.data[2];

	if (r2 < r1)
	{
		float tmp = r1;
		r1 = r2;
		r2 = tmp;
	}
	receivedHough = true;
}

void rawCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloud)
{
	//double timeScanCur = laserCloud->header.stamp.toSec();
	ros::Time timestamp = laserCloud->header.stamp;
	//bool returnBool;

	pcl::fromROSMsg(*laserCloud, *laserCloudIn);

	viewer->removeAllPointClouds(0);
	viewer->removeAllShapes(0);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudRGB(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudLabeled(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloudRaw(new pcl::PointCloud<pcl::PointXYZI>);
	//	std::vector<std::string> labelsC1;
	//	std::vector<std::string> labelsC2;
	//	pcl::PointCloud<SAFEFeatures>::Ptr featureCloudC1Acc(new pcl::PointCloud<SAFEFeatures>);
	//	pcl::PointCloud<SAFEFeatures>::Ptr featureCloudC2Acc(new pcl::PointCloud<SAFEFeatures>);
	std::vector<std::string> labelsC1;
	std::vector<std::string> labelsC2;
	std::vector<std::string> labelsC3;
	std::vector<std::string> labels;
	pcl::PointCloud<AllFeatures>::Ptr featureCloudAcc(new pcl::PointCloud<AllFeatures>);
	pcl::PointCloud<AllFeatures>::Ptr featureCloudC1Acc(new pcl::PointCloud<AllFeatures>);
	pcl::PointCloud<AllFeatures>::Ptr featureCloudC2Acc(new pcl::PointCloud<AllFeatures>);
	pcl::PointCloud<AllFeatures>::Ptr featureCloudC3Acc(new pcl::PointCloud<AllFeatures>);
	std::vector<Eigen::Matrix3f> confMats;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr classificationCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	std::vector<DatasetContainer> dataset;

	std::string folder = "/home/mikkel/catkin_ws/src/trainingSet/";

	pcl::copyPointCloud(*laserCloudIn, *cloudRGB);
	pcl::copyPointCloud(*laserCloudIn, *cloudRaw);

	// Single colored
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> color1(cloudRGB, 0, 0, 255);

	viewer->addPointCloud<pcl::PointXYZRGB>(cloudRGB,color1,"cloudRGB",viewP(RawCloud));
	//viewer->addText ("RawCloud", 10, 10, "vPP text", viewP(RawCloud));

	// Ground modeling
	//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudStat(new pcl::PointCloud<pcl::PointXYZRGB>);
	//	pcl::PointCloud<pcl::PointXYZ>::Ptr planeDistCloud(new pcl::PointCloud<pcl::PointXYZ>);
	//	pcl::copyPointCloud(*cloudRGB,*cloudStat);
	//	pcl::copyPointCloud(*cloudRGB,*planeDistCloud);


	// Remove everything outside the two lines
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
	//pcl::copyPointCloud(*cloudRaw, *cloud_filtered);

	// Rotation
	float thetaZ = theta*PI/180; // The angle of rotation in radians
	Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
	transform_2.rotate (Eigen::AngleAxisf (thetaZ, Eigen::Vector3f::UnitZ()));
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
	pcl::transformPointCloud (*cloudRaw, *cloud_filtered, transform_2);

	// Passthrough
	pcl::PassThrough<pcl::PointXYZI> passX;
	passX.setInputCloud (cloud_filtered);
	passX.setFilterFieldName ("x");
	passX.setFilterLimits (-0.5, 100.0);
	//pass.setFilterLimitsNegative (true);
	passX.filter (*cloud_filtered);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filteredRGB(new pcl::PointCloud<pcl::PointXYZRGB>);
	if (receivedHough)
	{
		pcl::PassThrough<pcl::PointXYZI> pass;
		pass.setInputCloud (cloud_filtered);
		pass.setFilterFieldName ("y");
		pass.setFilterLimits (r1+WALL_CLEARANCE, r2-WALL_CLEARANCE);
		//pass.setFilterLimitsNegative (true);
		pass.filter (*cloud_filtered);
	}

	pcl::copyPointCloud(*cloud_filtered, *cloud_filteredRGB);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> color9(cloudRGB, 0, 0, 255);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud_filteredRGB,color9,"LinesFiltered",viewP(LinesFiltered));
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "LinesFiltered");
	viewer->addText ("LinesFiltered", 10, 10, fontsize, 1, 1, 1, "LinesFiltered text", viewP(LinesFiltered));



	// Grid Minimum
//	pcl::PointCloud<pcl::PointXYZI>::Ptr gridCloud(new pcl::PointCloud<pcl::PointXYZI>);
//	pcl::GridMinimum<pcl::PointXYZI> gridm(1.0); // Set grid resolution
//	gridm.setInputCloud(cloud_filtered);
//	gridm.filter(*gridCloud);

	//*** Transform point cloud to adjust for a Ground Plane ***//
	pcl::ModelCoefficients ground_coefficients;
	pcl::PointIndices ground_indices;
	pcl::SACSegmentation<pcl::PointXYZI> ground_finder;
	ground_finder.setOptimizeCoefficients(true);
	ground_finder.setModelType(pcl::SACMODEL_PLANE);
	ground_finder.setMethodType(pcl::SAC_RANSAC);
	ground_finder.setDistanceThreshold(0.15);
	ground_finder.setInputCloud(cloud_filtered);
	ground_finder.segment(ground_indices, ground_coefficients);

	// Convert plane normal vector from type ModelCoefficients to Vector4f
	Eigen::Vector3f np = Eigen::Vector3f(ground_coefficients.values[0],ground_coefficients.values[1],ground_coefficients.values[2]);
	// Find rotation vector, u, and angle, theta
	Eigen::Vector3f u = np.cross(Eigen::Vector3f(0,0,1));
	u.normalize();
	float theta = acos(np.dot(Eigen::Vector3f(0,0,1)));
	// Construct transformation matrix (rotation matrix from axis and angle)
	Eigen::Affine3f tf = Eigen::Affine3f::Identity();
	float d = ground_coefficients.values[3];
	tf.translation() << d*np[0], d*np[1], d*np[2];
	tf.rotate (Eigen::AngleAxisf (theta, u));
	// Execute the transformation
	pcl::PointCloud<pcl::PointXYZI>::Ptr transformedCloud (new pcl::PointCloud<pcl::PointXYZI> ());
	pcl::transformPointCloud(*cloud_filtered, *transformedCloud, tf);


	// Compute statistical moments for least significant direction in neighborhood
#if (OLD_METHOD)
	pcl::NeighborhoodFeatures<pcl::PointXYZI, AllFeatures> features;
	pcl::PointCloud<AllFeatures>::Ptr featureCloud(new pcl::PointCloud<AllFeatures>);
	features.setInputCloud(transformedCloud);
	pcl::search::KdTree<pcl::PointXYZI>::Ptr search_tree5( new pcl::search::KdTree<pcl::PointXYZI>());
	features.setSearchMethod(search_tree5);
	//principalComponentsAnalysis.setRadiusSearch(0.6);
	//features.setKSearch(30);
	features.setRadiusSearch(0.01); // This value does not do anything! Look inside "NeighborgoodFeatures.h" to see adaptive radius calculation.
	features.compute(*featureCloud);

	//	ros::Time tFeatureCalculation2 = ros::Time::now();
	//	ros::Duration tFeatureCalculation = tFeatureCalculation2-tPreprocessing2;
	//	if (executionTimes==true)
	//		ROS_INFO("Feature calculation time = %i",tFeatureCalculation.nsec);



	visualizeFeature(*cloudRGB, *featureCloud, 0, viewP(GPDistMean), "GPDistMean");
	visualizeFeature(*cloudRGB, *featureCloud, 1, viewP(GPDistMin), "GPDistMin");
	visualizeFeature(*cloudRGB, *featureCloud, 2, viewP(GPDistPoint), "GPDistPoint");
	visualizeFeature(*cloudRGB, *featureCloud, 3, viewP(GPDistVar), "GPDistVar");
	visualizeFeature(*cloudRGB, *featureCloud, 4, viewP(PCA1), "PCA1"); // 4 = PCA1
	visualizeFeature(*cloudRGB, *featureCloud, 5, viewP(PCA2), "PCA2"); // 3 = GPVar
	visualizeFeature(*cloudRGB, *featureCloud, 6, viewP(PCA3), "PCA3"); // 0 = GPMean
	visualizeFeature(*cloudRGB, *featureCloud, 7, viewP(PCANX), "PCANX");
	visualizeFeature(*cloudRGB, *featureCloud, 8, viewP(PCANY), "PCANY");
	visualizeFeature(*cloudRGB, *featureCloud, 9, viewP(PCANZ), "PCANZ"); // 9 = PCANZ
	visualizeFeature(*cloudRGB, *featureCloud, 10, viewP(PointDist), "PointDist");
	visualizeFeature(*cloudRGB, *featureCloud, 11, viewP(RSS), "RSS");
	visualizeFeature(*cloudRGB, *featureCloud, 12, viewP(Reflectance), "Reflectance");

	ROS_INFO("Computed all neighborhood features");

	std::vector<float>* centroid = new std::vector<float>();
	std::vector<float>* stds = new std::vector<float>();

	//*** Testing ***//
	if (training == false)
	{
		Eigen::Matrix3f confMat = Eigen::Matrix3f::Zero();
		if (testdata==true)
		{
			*featureCloud = *featureCloudAcc;
		}

		pcl::copyPointCloud(*cloudRGB,*classificationCloud);

		// Load feature normalization parameters
		std::ifstream input_file((folder + "centroid").c_str());
		std::istream_iterator<float> start(input_file), end;
		std::vector<float> numbers(start, end);
		std::copy(numbers.begin(), numbers.end(), std::back_inserter(*centroid));
		std::ifstream input_file2((folder + "stds").c_str());
		std::istream_iterator<float> start2(input_file2), end2;
		std::vector<float> numbers2(start2, end2);
		std::copy(numbers2.begin(), numbers2.end(), std::back_inserter(*stds));


		// Normalize features
		//		if (pcl::io::savePCDFile ("testFeaturesNonNormalized.pcd", *featureCloud) != 0)
		//			return (false);

		ros::Time tNormalization1 = ros::Time::now();
		normalizeFeatures(*featureCloud,*featureCloud, &centroid, &stds);
		ros::Time tNormalization2 = ros::Time::now();
		ros::Duration tNormalization = tNormalization2-tNormalization1;
		if (executionTimes==true)
			ROS_INFO("Normalization time = %f",(float)(tNormalization.nsec)/1000000);

		pcl::PointIndices::Ptr unclassifiedIndices (new pcl::PointIndices ());
		//		if (pcl::io::savePCDFile ("testFeatures.pcd", *featureCloud) != 0)
		//			return (false);

		CvSVM SVM;
		//int dim = sizeof(featureCloud->points[0])/sizeof(float);
		int dim = useFeatures.size();//sizeof(useFeatures)/sizeof(int);

		SVM.load((folder + "svm").c_str());
		//SVM.load((folder + "svm2014-11-06-11-22-59").c_str());
		//SVM.load((folder + "svm2014-11-07-14-00-31").c_str());
		//SVM.load((folder + "svm2014-11-07-13-16-28").c_str());

		ros::Time tClassification1 = ros::Time::now();
		//int nMissingLabels = 0;
		for (size_t i=0;i<classificationCloud->points.size();i++)
		{
			float dataSVM[dim];
			for (int d=0;d<dim;d++)
			{
				//dataSVM[d] = featureCloud->points[i].data[d];
				dataSVM[d] = featureCloud->points[i].data[useFeatures[d]];
			}
			Mat labelsMat(1, dim, CV_32FC1, dataSVM);

			float response = SVM.predict(labelsMat);

			if (response == 1.0)
				classificationCloud->points[i].b = 255;
			else if (response == 2.0)
				classificationCloud->points[i].g = 255;
			else if (response == 3.0)
				classificationCloud->points[i].r = 255;
			else
				ROS_INFO("Another class label = %f",response);

			if (testdata==true)
			{
				int groundTruth;
				if (labels[i].compare("ground") == 0)
					groundTruth = 1;
				else if (labels[i].compare("vegetation") == 0)
					groundTruth = 2;
				else if (labels[i].compare("object") == 0)
					groundTruth = 3;
				confMat(groundTruth-1,(int)response-1)++;
			}
		}
		ros::Time tClassification2 = ros::Time::now();
		ros::Duration tClassification = tClassification2-tClassification1;
		if (executionTimes==true)
			ROS_INFO("Classification time = %f",(float)(tClassification.nsec)/100000);

		//ROS_INFO("Number of missing class labels = %i", nMissingLabels);

		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> colorC(classificationCloud);
		viewer->addPointCloud<pcl::PointXYZRGB>(classificationCloud,colorC,"Classification",viewP(Classification));
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "Classification");
		viewer->addText ("Classification", 10, 10, fontsize, 1, 1, 1, "Classification text", viewP(Classification));

		// Test set - calculate performance statistics
		if (testdata == true)
		{
			std::cout << confMat << std::endl;
			confMats.push_back(confMat);

			if (true)//k==files.size()-1)
			{
				ofstream myfile;
				string resultsFolder = "/home/mikkel/catkin_ws/src/Results/";
				myfile.open ((resultsFolder + "run_number_here.txt").c_str());

				// Distance threshold
				myfile << "DistanceThreshold:" << distThr << ";\n";

				// Neighborhood parameters
				myfile << "Neighborhood:" << rmin << ";" << rmindist << ";" << rmax << ";" << rmaxdist << ";\n";

				// Features
				myfile << "Features:";
				for (int d=0;d<dim;d++)
					myfile << useFeatures[d] << ";";
				myfile << "\n";

				myfile << "ConfusionMatrix:";
				for (size_t i=0;i<confMats.size();i++)
				{
					for (int r=0;r<confMats[i].rows();r++)
						for (int c=0;c<confMats[i].cols();c++)
							myfile << confMats[i](r,c) << ";";
					myfile << "\n";
				}

				myfile << "Accuracy:" << confMat.diagonal().sum()/confMat.sum() << ";\n";


				myfile.close();
			}

		}
		featureCloudAcc->clear();
		labels.clear();

#else

		for (size_t i=0;i<transformedCloud->size();i++)
		{
			pcl::PointXYZI pTmp2 = transformedCloud->points[i];
			pcl::PointXYZRGB pTmp;
			pTmp.x = pTmp2.x;
			pTmp.y = pTmp2.y;
			pTmp.z = pTmp2.z;
			pTmp.r = 255;
			pTmp.g = 255;
			if (pTmp.z > 0.1) // Object
			{
				classificationCloud->push_back(pTmp);
			}
		}


#endif





		// REGION GROWING
		//ROS_INFO("region growing 5");
		pcl::PointCloud <pcl::PointXYZRGB>::Ptr objectCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		//pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud <pcl::PointXYZRGB>);

		for (size_t i=0;i<classificationCloud->size();i++)
		{
			pcl::PointXYZRGB pTmp = classificationCloud->points[i];
			if ((pTmp.r == 255) || (pTmp.g == 255)) // Object
			{
				objectCloud->push_back(pTmp);
			}
		}

		pcl::RegionGrowing<pcl::PointXYZRGB, pcl::Normal> reg;

		reg.setInputCloud (objectCloud);
		//reg.setIndices (indices);
		pcl::search::Search<pcl::PointXYZRGB>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB> > (new pcl::search::KdTree<pcl::PointXYZRGB>);
		reg.setSearchMethod (tree);
		//reg.setDistanceThreshold (0.0001f);
		//reg.setResidualThreshold(0.01f);
		//ROS_INFO("res = %f",reg.getSmoothnessThreshold());
		//reg.setPointColorThreshold (6);
		//reg.setRegionColorThreshold (5);
		reg.setMinClusterSize (1);
		reg.setResidualThreshold(0.2f);
		reg.setCurvatureTestFlag(false);
		//		reg.setResidualTestFlag(true);
		//		reg.setNormalTestFlag(false);
		//		reg.setSmoothModeFlag(false);
		std::vector <pcl::PointIndices> clusters;
		reg.extract (clusters);

		//ROS_INFO("clusters = %d", clusters.size());

		//std::cout << "testefter" << std::endl;

		pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();

		pcl::PointCloud <pcl::PointXYZRGB>::Ptr clustersFilteredCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		std::vector <pcl::PointIndices> clustersFiltered;
		//pcl::PointXYZRGB min_pt, max_pt;
		Eigen::Vector4f min_pt, max_pt;
		obstacle_detection::boundingbox bbox;
		obstacle_detection::boundingboxes bboxes;
		bboxes.header.stamp = timestamp;
		bboxes.header.frame_id = "/velodyne";

		viewer->removeAllShapes(viewP(SegmentsFiltered));

		for (size_t i=0;i<clusters.size();i++)
		{
			//ROS_INFO("cluster size = %d",clusters[i].indices.size());
			if (clusters[i].indices.size() > 100)
			{
				clustersFiltered.push_back(clusters[i]);

				for (size_t pp=0;pp<clusters[i].indices.size();pp++)
				{
					clustersFilteredCloud->push_back(colored_cloud->points[clusters[i].indices[pp]]);
				}

				// Calculate bounding box
				pcl::getMinMax3D(*colored_cloud, clusters[i], min_pt, max_pt);
				bbox.start.x = min_pt[0];
				bbox.start.y = min_pt[1];
				bbox.start.z = min_pt[2];
				bbox.width.x = max_pt[0]-min_pt[0];
				bbox.width.y = max_pt[1]-min_pt[1];
				bbox.width.z = max_pt[2]-min_pt[2];
				bbox.header.stamp = timestamp;
				bbox.header.frame_id = "/velodyne";

				bboxes.boundingboxes.push_back(bbox);

				viewer->addCube (min_pt[0], max_pt[0], min_pt[1], max_pt[1], min_pt[2], max_pt[2], 1.0f, 1.0f, 1.0f, (boost::format("%04d") % i).str().c_str(), viewP(SegmentsFiltered));
			}


		}

		pubBBoxesPointer->publish(bboxes);
		//
		//		//pcl::visualization::CloudViewer viewer ("Cluster viewer");
		//		//viewer.showCloud (colored_cloud);
		//
		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> colorC2(colored_cloud);
		viewer->addPointCloud<pcl::PointXYZRGB>(colored_cloud,colorC2,"Segments",viewP(Segments));
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "Segments");
		viewer->addText ("Segments", 10, 10, fontsize, 1, 1, 1, "Segments text", viewP(Segments));


		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> colorC3(clustersFilteredCloud);
		viewer->addPointCloud<pcl::PointXYZRGB>(clustersFilteredCloud,colorC3,"SegmentsFiltered",viewP(SegmentsFiltered));
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "SegmentsFiltered");
		viewer->addText ("Segments", 10, 10, fontsize, 1, 1, 1, "SegmentsFiltered text", viewP(SegmentsFiltered));
		//
		//
		//		// BOUNDING BOXES



		//		viewer->addCube (min_pt.x, max_pt.x, min_pt.y, max_pt.y, min_pt.z, max_pt.z);





		//		std_msgs::Float64 testF;
		//		//testF.data = 10;
		//		testF.data = 10;
		//



		// Compute principal directions
		//		Eigen::Vector4f pcaCentroid;
		//		pcl::compute3DCentroid(*clustersFilteredCloud, pcaCentroid);
		//		Eigen::Matrix3f covariance;
		//		computeCovarianceMatrixNormalized(*clustersFilteredCloud, pcaCentroid, covariance);
		//		Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
		//		Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
		//		eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));
		//
		//		// Transform the original cloud to the origin where the principal components correspond to the axes.
		//		Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
		//		projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
		//		projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());
		//		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPointsProjected (new pcl::PointCloud<pcl::PointXYZRGB>);
		//		pcl::transformPointCloud(*clustersFilteredCloud, *cloudPointsProjected, projectionTransform);
		//		// Get the minimum and maximum points of the transformed cloud.
		//		pcl::PointXYZRGB minPoint, maxPoint;
		//		pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
		//		const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());
		//
		//		// Final transform
		//		const Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA); //Quaternions are a way to do rotations https://www.youtube.com/watch?v=mHVwd8gYLnI
		//		const Eigen::Vector3f bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();
		//		viewer->addCube(bboxTransform, bboxQuaternion, maxPoint.x - minPoint.x, maxPoint.y - minPoint.y, maxPoint.z - minPoint.z, "bbox");


























		//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> colorTTF1(featureCloudTest, 0, 255, 0);
		//	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> colorTTF1(featureCloudTest);
		//	PCL_INFO("featureCloudTest size = %i",featureCloudTest->points.size());
		//	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> colorTTF2(featureCloudTrain, 255, 0, 0);
		//	viewer->addPointCloud<pcl::PointXYZRGB>(featureCloudTest,colorTTF1,"TestTrainFeatures1",viewP(TestTrainFeatures));
		//	viewer->addPointCloud<pcl::PointXYZRGB>(featureCloudTrain,colorTTF2,"TestTrainFeatures2",viewP(TestTrainFeatures));
		//	viewer->addText ("TestTrainFeatures", 10, 10, "TestTrainFeatures text", viewP(TestTrainFeatures));

		//		sensor_msgs::PointCloud2 processedCloud;
		//		pcl::toROSMsg(*classificationCloud, processedCloud);
		//		processedCloud.header.stamp = ros::Time::now();//processedCloud->header.stamp;
		//		processedCloud.header.frame_id = "/velodyne";
		//		pubProcessedPointCloudPointer->publish(processedCloud);
		//
		//		pcl::PointCloud<pcl::PointXYZRGB>::Ptr groundCloud2D(new pcl::PointCloud<pcl::PointXYZRGB>);
		//		pcl::PointCloud<pcl::PointXYZRGB>::Ptr objectCloud2D(new pcl::PointCloud<pcl::PointXYZRGB>);
		//
		//		for (size_t i=0;i<classificationCloud->size();i++)
		//		{
		//			pcl::PointXYZRGB pTmp = classificationCloud->points[i];
		//			pTmp.z = 0; // 3D -> 2D
		//			if ((pTmp.r == 255) || (pTmp.g == 255)) // Object
		//			{
		//				objectCloud2D->push_back(pTmp);
		//			}
		//			else if (pTmp.b == 255) // Object
		//			{
		//				groundCloud2D->push_back(pTmp);
		//			}
		//		}
		//
		//		//std::vector<int> indexVector;
		//		unsigned int leafNodeCounter = 0;
		//		unsigned int voxelDensity = 0;
		//		unsigned int totalPoints = 0;
		//		//int occupancyW = OCCUPANCY_WIDTH/OCCUPANCY_GRID_RESOLUTION;
		//		//int occupancyH = OCCUPANCY_DEPTH/OCCUPANCY_GRID_RESOLUTION;
		//		std::vector<unsigned int> ocGroundGrid(OCCUPANCY_WIDTH*OCCUPANCY_DEPTH, 0);
		//		std::vector<unsigned int> ocObjectGrid(OCCUPANCY_WIDTH*OCCUPANCY_DEPTH, 0);
		//		std::vector<signed char> ocGridFinal(OCCUPANCY_WIDTH*OCCUPANCY_DEPTH,-1);
		//
		//		// Ground grid
		//		pcl::octree::OctreePointCloudDensity< pcl::PointXYZRGB > octreeGround (OCCUPANCY_GRID_RESOLUTION);
		//		octreeGround.setInputCloud (groundCloud2D);
		//		pcl::PointXYZ bot(-OCCUPANCY_DEPTH_M/2,-OCCUPANCY_WIDTH_M/2,-0.5);
		//		pcl::PointXYZ top(OCCUPANCY_DEPTH_M/2,OCCUPANCY_WIDTH_M/2,0.5);
		//		octreeGround.defineBoundingBox(bot.x, bot.y, bot.z, top.x, top.y, top.z);    // I have already calculated the BBox of my point cloud
		//		octreeGround.addPointsFromInputCloud ();
		//		pcl::octree::OctreePointCloudDensity<pcl::PointXYZRGB>::LeafNodeIterator it1;
		//		pcl::octree::OctreePointCloudDensity<pcl::PointXYZRGB>::LeafNodeIterator it1_end = octreeGround.leaf_end();
		//		int minx = 1000;
		//		int miny = 1000;
		//		int maxx = -1000;
		//		int maxy = -1000;
		//		for (it1 = octreeGround.leaf_begin(); it1 != it1_end; ++it1)
		//		{
		//			pcl::octree::OctreePointCloudDensityContainer& container = it1.getLeafContainer();
		//			voxelDensity = container.getPointCounter();
		//			Eigen::Vector3f min_pt, max_pt;
		//			octreeGround.getVoxelBounds (it1, min_pt, max_pt);
		//			//ROS_INFO("x = %f, y = %f, z = %f, x2 = %f, y2 = %f, z2 = %f",min_pt[0],min_pt[1],min_pt[2],max_pt[0],max_pt[1],max_pt[2]);
		//			totalPoints += voxelDensity;
		//			if (min_pt[0] < minx)
		//				minx = min_pt[0];
		//			if (min_pt[0] > maxx)
		//				maxx = min_pt[0];
		//			if (min_pt[1] < miny)
		//				miny = min_pt[1];
		//			if (min_pt[1] > maxy)
		//				maxy = min_pt[1];
		//
		//			int r = OCCUPANCY_DEPTH-(min_pt[0]+OCCUPANCY_DEPTH_M/2)/OCCUPANCY_GRID_RESOLUTION;
		//			int c = OCCUPANCY_WIDTH-(min_pt[1]+OCCUPANCY_WIDTH_M/2)/OCCUPANCY_GRID_RESOLUTION;
		//
		//			//			if ((r == 51) || (c==51))
		//			//				ROS_INFO("r = %i,c = %i",r,c);
		//
		//			//if (((int)min_pt[0] == -1) || ((int)min_pt[1]==-1))
		//			//ROS_INFO("min_pt[0] = %f, min_pt[1]=%f",min_pt[0],min_pt[1]);
		//			if (!((r < 0) || (c < 0) || (r > OCCUPANCY_DEPTH-1) || (c > OCCUPANCY_WIDTH-1)))
		//				ocGroundGrid[r+OCCUPANCY_DEPTH*c] = voxelDensity;
		//			//ROS_INFO("density = %i, r = %i, c=%i",voxelDensity, r, c);
		//			//ROS_INFO("x (%f) -> r (%i), y (%f) -> c (%i)",min_pt[0],r,min_pt[1],c);
		//			leafNodeCounter++;
		//		}
		//
		//
		//		// Object grid
		//		pcl::octree::OctreePointCloudDensity< pcl::PointXYZRGB > octreeObject (OCCUPANCY_GRID_RESOLUTION);
		//		octreeObject.setInputCloud (objectCloud2D);
		//		octreeObject.defineBoundingBox(bot.x, bot.y, bot.z, top.x, top.y, top.z);    // I have already calculated the BBox of my point cloud
		//		octreeObject.addPointsFromInputCloud ();
		//		pcl::octree::OctreePointCloudDensity<pcl::PointXYZRGB>::LeafNodeIterator it2;
		//		pcl::octree::OctreePointCloudDensity<pcl::PointXYZRGB>::LeafNodeIterator it2_end = octreeObject.leaf_end();
		//		minx = 1000;
		//		miny = 1000;
		//		maxx = -1000;
		//		maxy = -1000;
		//		leafNodeCounter = 0;
		//		voxelDensity = 0;
		//		totalPoints = 0;
		//		int t1 = 0;
		//		int t2 = 0;
		//		int t3 = 0;
		//		for (it2 = octreeObject.leaf_begin(); it2 != it2_end; ++it2)
		//		{
		//			pcl::octree::OctreePointCloudDensityContainer& container = it2.getLeafContainer();
		//			voxelDensity = container.getPointCounter();
		//			Eigen::Vector3f min_pt, max_pt;
		//			octreeObject.getVoxelBounds (it2, min_pt, max_pt);
		//			//ROS_INFO("x = %f, y = %f, z = %f, x2 = %f, y2 = %f, z2 = %f",min_pt[0],min_pt[1],min_pt[2],max_pt[0],max_pt[1],max_pt[2]);
		//			totalPoints += voxelDensity;
		//			if (min_pt[0] < minx)
		//				minx = min_pt[0];
		//			if (min_pt[0] > maxx)
		//				maxx = min_pt[0];
		//			if (min_pt[1] < miny)
		//				miny = min_pt[1];
		//			if (min_pt[1] > maxy)
		//				maxy = min_pt[1];
		//
		//			int r = OCCUPANCY_DEPTH-(min_pt[0]+OCCUPANCY_DEPTH_M/2)/OCCUPANCY_GRID_RESOLUTION;
		//			int c = OCCUPANCY_WIDTH-(min_pt[1]+OCCUPANCY_WIDTH_M/2)/OCCUPANCY_GRID_RESOLUTION;
		//			if (!((r < 0) || (c < 0) || (r > OCCUPANCY_DEPTH-1) || (c > OCCUPANCY_WIDTH-1)))
		//				ocObjectGrid[r+OCCUPANCY_DEPTH*c] = voxelDensity;
		//			//ROS_INFO("object points = %i -> %i: x (%f) -> r (%i), y (%f) -> c (%i)",voxelDensity,ocGridFinal[r+OCCUPANCY_DEPTH*c],min_pt[0],r,min_pt[1],c);
		//			//ocGrid[r+OCCUPANCY_DEPTH*c] = voxelDensity;
		//			//ROS_INFO("density = %i, r = %i, c=%i",voxelDensity, r, c);
		//
		//			leafNodeCounter++;
		//		}
		//
		//		for (int r=0;r<OCCUPANCY_DEPTH;r++)
		//		{
		//			for (int c=0;c<OCCUPANCY_WIDTH;c++)
		//			{
		//				if ((ocGroundGrid[r+OCCUPANCY_DEPTH*c] == 0) && (ocObjectGrid[r+OCCUPANCY_DEPTH*c] <= 1))
		//				{
		//					ocGridFinal[r+OCCUPANCY_DEPTH*c] = -1.0; // 0.5 default likelihood
		//					t1++;
		//				}
		//				else if ((ocObjectGrid[r+OCCUPANCY_DEPTH*c] == 0))// && (ocGroundGrid[r+OCCUPANCY_DEPTH*c] > 0))
		//				{
		//					ocGridFinal[r+OCCUPANCY_DEPTH*c] = OCCUPANCY_MIN_PROB;
		//					t2++;
		//				}
		//				else
		//				{
		//					ocGridFinal[r+OCCUPANCY_DEPTH*c] = 50+(OCCUPANCY_MAX_PROB-50)*ocObjectGrid[r+OCCUPANCY_DEPTH*c]/(ocObjectGrid[r+OCCUPANCY_DEPTH*c]+ocGroundGrid[r+OCCUPANCY_DEPTH*c]);
		//					t3++;
		//				}
		//			}
		//		}
		//
		//		nav_msgs::OccupancyGrid occupancyGrid;
		//		occupancyGrid.info.resolution = OCCUPANCY_GRID_RESOLUTION;
		//		occupancyGrid.header.stamp = timestamp;//ros::Time::now();
		//		occupancyGrid.header.frame_id = "/velodyne";
		//		occupancyGrid.info.width = OCCUPANCY_WIDTH;
		//		occupancyGrid.info.height = OCCUPANCY_DEPTH;
		//		//geometry_msgs::Pose lidarPose;
		//		geometry_msgs::Point lidarPoint;
		//		lidarPoint.x = 0;
		//		lidarPoint.y = 0;
		//		lidarPoint.z = 0;
		//		geometry_msgs::Quaternion lidarQuaternion;
		//		lidarQuaternion.w = 1;
		//		lidarQuaternion.x = 0;
		//		lidarQuaternion.y = 0;
		//		lidarQuaternion.z = 0;
		//		occupancyGrid.info.origin.position = lidarPoint;
		//		occupancyGrid.info.origin.orientation = lidarQuaternion;
		//		occupancyGrid.data = ocGridFinal;
		//
		//		//ROS_INFO("total points = %i, total leaves = %i",totalPoints,leafNodeCounter);
		//		//ROS_INFO("minx = %i, maxx = %i, miny = %i, maxy = %i",minx,maxx,miny,maxy);
		//		//ROS_INFO("t1 = %i, t2 = %i, t3 = %i",t1,t2,t3);
		//
		//
		//		pubOccupancyGridPointer->publish(occupancyGrid);


		//		int l = 0;
		//		while (*++itLeafs)
		//		{
		//		    //Iteratively explore only the leaf nodes..
		//		    std::vector<PointXYZ> points;
		//		    itLeafs->getData(points);
		//		    l++;
		//		}
		//
		//		ROS_INFO("l = %i",l);

		//pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (OCCUPANCY_GRID_RESOLUTION);

		//		sensor_msgs::PointCloud2::Ptr cloud_filtered (new sensor_msgs::PointCloud2 ());
		//		pcl::VoxelGrid<sensor_msgs::PointCloud2> sor;
		//		sor.setInputCloud (processedCloud);
		//		sor.setLeafSize (0.01f, 0.01f, 0.01f);
		//		sor.filter (*cloud_filtered);
		if (n==0)
		{
			//viewer->setCameraPosition(-20,0,10,1,0,2,1,0,2,v1);
			//viewer->setCameraPosition(-20,0,10,1,0,2,1,0,2,v2);
			viewer->loadCameraParameters("pcl_video.cam");
		}
#if (OLD_METHOD)
	}
#endif

	viewer->spinOnce();

	// Save screenshot
	std::stringstream tmp;
	tmp << n;
	viewer->saveScreenshot("/home/mikkel/images/" + (boost::format("%04d") % n).str() + ".png");
	n++;
}

int
main (int argc, char** argv)
{
	ros::init(argc, argv, "obstacle_detection");
	ros::NodeHandle nh;


	ros::Publisher pubProcessedPointCloud = nh.advertise<sensor_msgs::PointCloud2>
	("/obstacle_detection/point_cloud_processed", 1);

	ros::Subscriber subRawPointCloud = nh.subscribe<sensor_msgs::PointCloud2>
	("/velodyne_points", 2, rawCloudHandler);

	ros::Subscriber sub2Lines = nh.subscribe("/measHough", 2, linesHandler);


	ros::Publisher pubOccupancyGrid = nh.advertise<nav_msgs::OccupancyGrid>
	("/obstacle_detection/velodyne_evidence_grid", 1);

	ros::Publisher pubBBoxes = nh.advertise<obstacle_detection::boundingboxes>
	("/obstacle_detection/boundingboxes", 1);

	pubProcessedPointCloudPointer = &pubProcessedPointCloud;
	subRawPointCloudPointer = &subRawPointCloud;
	pubOccupancyGridPointer = &pubOccupancyGrid;
	pubBBoxesPointer = &pubBBoxes;
	sub2LinesPointer = &sub2Lines;

	// Set up PCL Viewer
	viewer.reset (new pcl::visualization::PCLVisualizer (argc, argv, "PointCloud"));
	viewer->registerKeyboardCallback (keyboardEventOccurred);
	//
	//	// Create view ports
	//	// 1 view port
	//	viewer->createViewPort(0.0,0.0,1.0,1.0,v2);

	// 2 view ports
	//viewer->createViewPort(0.0,0.0,0.5,1.0,v1);
	//viewer->createViewPort(0.5,0.0,1.0,1.0,v2);

	// 4 view ports
	viewer->createViewPort (0.0, 0.5, 0.5, 1.0, v1);
	viewer->createViewPort (0.5, 0.5, 1.0, 1.0, v2);
	viewer->createViewPort (0.0, 0.0, 0.5, 0.5, v3);
	viewer->createViewPort (0.5, 0.0, 1.0, 0.5, v4);


	viewer->addCoordinateSystem(1.0);

	n = 0;

	ros::spin();

	return (0);
}
