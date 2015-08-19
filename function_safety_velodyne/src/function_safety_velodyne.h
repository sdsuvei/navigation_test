/*
 * HelpingFunctions.h
 *
 *  Created on: Feb 18, 2015
 *      Author: mikkel
 */

#ifndef PCL_ROS_SRC_PCL_ROS_H_
#define PCL_ROS_SRC_PCL_ROS_H_

const int RawCloud = 1;
const int StickCloud = 2;
const int PCAvalues = 3;
const int SinglePlane = 4;
const int ParallelPlanes = 5;
const int GPDistMean = 6;
const int Classification = 7;
const int PlaneTransform = 8;
const int TestTrainFeatures = 9;
const int PCANZ = 10;
const int PCA1 = 11;
const int PCA2 = 12;
const int PCA3 = 13;
const int PCANX = 14;
const int PCANY = 15;
const int GPDistMin = 17;
const int GPDistPoint = 18;
const int GPDistVar = 19;
const int PointDist = 20;
const int RSS = 21;
const int Reflectance = 22;
const int GroundTruth = 23;
const int Segments = 24;
const int SegmentsFiltered = 25;
const int BLACK = 99;

#define SHOW1 RawCloud
#define SHOW2 StickCloud
#define SHOW3 Segments
#define SHOW4 SegmentsFiltered

#define training false

#define testdata false
#define featureSelection false
const std::string featureSelectionMethod = "forward"; // "backward"
#define crossvalidation true

#define executionTimes true

#define knn 1
#define svm 2
#define distThr 200.0

#define classifier svm // "knn"

#define OCCUPANCY_GRID_RESOLUTION 0.2f // 10 cm
#define OCCUPANCY_WIDTH_M 100.0f
#define OCCUPANCY_DEPTH_M 100.0f
#define OCCUPANCY_MAX_PROB 80
#define OCCUPANCY_MIN_PROB 20
static const int OCCUPANCY_WIDTH = OCCUPANCY_WIDTH_M/OCCUPANCY_GRID_RESOLUTION;
static const int OCCUPANCY_DEPTH = OCCUPANCY_DEPTH_M/OCCUPANCY_GRID_RESOLUTION;

// Neighborhood parameters
#define rmin 0.3
#define rmindist 2.0
#define rmax 3.0
#define rmaxdist 100.0

#define fontsize 0
#define nClasses 3

// Viewer
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

// View Ports
int v1(SHOW1);
int v2(SHOW2);
int v3(SHOW3);
int v4(SHOW4);
int viewP(const int vP)
{
	switch(vP)
	{
	case SHOW1:
		return v1;
	case SHOW2:
		return v2;
	case SHOW3:
		return v3;
	case SHOW4:
		return v4;
	default:
		return 99;
	}
}

// Point Cloud Coloring
typedef struct {
	double r,g,b;
} COLOUR;
pcl::PointXYZ colors[] = {pcl::PointXYZ(255,0,0),pcl::PointXYZ(0,255,0),pcl::PointXYZ(255,255,0),pcl::PointXYZ(255,255,255),pcl::PointXYZ(0,255,255),pcl::PointXYZ(255,0,255),pcl::PointXYZ(128,0,0),pcl::PointXYZ(0,128,0),pcl::PointXYZ(128,128,0),pcl::PointXYZ(128,0,128),pcl::PointXYZ(0,0,255)};

struct AllFeatures
{
	//PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
	union
	{
		float data[13];
		struct
		{
			float GPDistMean; 	// 0
			float GPDistMin; 	// 1
			float GPDistPoint; 	// 2
			float GPDistVar; 	// 3
			float PCA1; 		// 4
			float PCA2; 		// 5
			float PCA3; 		// 6
			float PCANX; 		// 7
			float PCANY; 		// 8
			float PCANZ; 		// 9
			float PointDist; 	// 10
			float RSS;			// 11
			float Reflectance;	// 12
		};
	};
	//EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
};// EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

const int allFeatures[] = {0,1,2,3,4,5,6,7,8,9,10,11,12};
//const int useFeatures[] = {0,1,2,3,4,5,6,7,8,9,10,11};
std::vector<int> useFeatures(allFeatures, allFeatures + sizeof(allFeatures) / sizeof(allFeatures[0]) );

POINT_CLOUD_REGISTER_POINT_STRUCT (AllFeatures,           // here we assume a XYZ + "test" (as fields)
		(float, GPDistMean, GPDistMean)
		(float, GPDistMin, GPDistMin)
		(float, GPDistPoint, GPDistPoint)
		(float, GPDistVar, GPDistVar)
		(float, PCA1, PCA1)
		(float, PCA2, PCA2)
		(float, PCA3, PCA3)
		(float, PCANX, PCANX)
		(float, PCANY, PCANY)
		(float, PCANZ, PCANZ)
		(float, PointDist, PointDist)
		(float, RSS, RSS)
		(float, Reflectance, Reflectance)
)

struct DatasetContainer{
	std::string bagName;
	std::vector<pcl::PointCloud<AllFeatures>::Ptr > features;
	std::vector<std::vector<std::string> > labels;
};

struct PointClass
{
	std::string classLabel;
	std::vector<pcl::PointXYZ> colors;
};

struct PCAFeatures
{
	//PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
	union
	{
		float data[3];
		struct
		{
			float PCA1;
			float PCA2;
			float PCA3;
		};
	};
	union
	{
	  float data_n[4];
	  float normal[3];
	  struct
	  {
	    float normal_x;
	    float normal_y;
	    float normal_z;
	  };
	};
	//EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
};

struct GPDist
{
	//PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
	union
	{
		float data[2];
		struct
		{
			float GPDistMean;
			float GPDistVar;
		};
	};
	//EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
};

// kNN Classification
typedef std::pair< std::vector< std::string >, std::vector< float > > Result;
typedef boost::shared_ptr< Result > ResultPtr;



#endif /* PCL_ROS_SRC_PCL_ROS_H_ */
