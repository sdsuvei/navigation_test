/*
 * HelpingFunctions.h
 *
 *  Created on: Feb 18, 2015
 *      Author: mikkel
 */

#ifndef PCL_ROS_SRC_HELPINGFUNCTIONS_H_
#define PCL_ROS_SRC_HELPINGFUNCTIONS_H_

#include "obstacle_detection.h"
#include <pcl/point_representation.h>
#include <sstream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/ml/ml.hpp>

using namespace cv;
using namespace std;

COLOUR GetColour(double v,double vmin,double vmax)
{
	COLOUR c = {1.0,1.0,1.0}; // white
	double dv;

	if (v < vmin)
		v = vmin;
	if (v > vmax)
		v = vmax;
	dv = vmax - vmin;

	if (v < (vmin + 0.25 * dv)) {
		c.r = 0;
		c.g = 4 * (v - vmin) / dv;
	} else if (v < (vmin + 0.5 * dv)) {
		c.r = 0;
		c.b = 1 + 4 * (vmin + 0.25 * dv - v) / dv;
	} else if (v < (vmin + 0.75 * dv)) {
		c.r = 4 * (v - vmin - 0.5 * dv) / dv;
		c.b = 0;
	} else {
		c.g = 1 + 4 * (vmin + 0.75 * dv - v) / dv;
		c.b = 0;
	}

	return(c);
}

bool cmpCP(pcl::PointXYZRGB p, pcl::PointXYZ pc)
{
	return ((p.r == pc.x) && (p.g == pc.y) && (p.b == pc.z));
}

template <typename PointT>
void normalizeFeatures(const pcl::PointCloud<PointT> &cloudIn, pcl::PointCloud<PointT> &cloudOut, std::vector<float>** centroid, std::vector<float>** stds)
{
	//				Eigen::Matrix< float, Eigen::Dynamic, 1 > centroid;
	//				pcl::computeNDCentroid (*featureCloud, centroid);
	//pcl::demeanPointCloud(*featureCloud, centroid, *featureCloud);
	//				EIGEN_ALIGN16 Eigen::Matrix3f covariance_matrix;
	//				pcl::computeCovarianceMatrix(*featureCloud, covariance_matrix);
	//				covariance_matrix(0,0) = sqrt((fl	oat)covariance_matrix(0,0));
	//				covariance_matrix(1,1) = sqrt((float)covariance_matrix(1,1));
	//				covariance_matrix(2,2) = sqrt((float)covariance_matrix(2,2));
	//				for (size_t i=0;i<featureCloud->points.size();i++)
	//				{
	//					featureCloud->points[i].GPDistMean = featureCloud->points[i].GPDistMean/covariance_matrix(0,0);
	//					featureCloud->points[i].GPDistVar = featureCloud->points[i].GPDistVar/covariance_matrix(1,1);
	//					featureCloud->points[i].PCA1 = featureCloud->points[i].PCA1/covariance_matrix(2,2);
	//					//featureCloud->points[i].PCA2 = PCACloud->points[i].j2;
	//					//featureCloud->points[i].PCA3 = PCACloud->points[i].j3;
	//				}

	// PointRepresentationConstPtr
	// pcl::PointRepresentation< PointT >
	//pcl::PointRepresentation<PointT> test;
	int dim = sizeof(cloudIn.points[0])/sizeof(float);//test.getNumberOfDimensions();
	//ROS_INFO("centroid1 size = %d",centroid1->size());
	//ROS_INFO("dim = %i",dim);
	typename pcl::PointCloud<PointT>::Ptr cloudN(new pcl::PointCloud<PointT>());
	pcl::copyPointCloud(cloudIn,*cloudN);
	//PointT tmp;
	//cloudOut = new pcl::PointCloud<PointT>(cloudIn.width,cloudIn.height,tmp);
	//test.copyToFloatArray();
	//pcl::PointXYZ testp;
	//testp.x = 1;
	//cloudN->push_back(testp);
	pcl::DefaultPointRepresentation<PointT> fRep;


	//float floatArray[dim];
	//float *floatArray = new float[dim];
	//pcl::DefaultPointRepresentation<pcl::PointXYZ>::copyToFloatArray(cloudN,floatArray);
	//	fRep.copyToFloatArray(*cloudN,floatArray);

	//	std::vector<float> centroid(dim,0.0);
	//
	//	std::vector<float> stds(dim,0.0);
	//*centroid = std::vector<float> (dim,0.0);
	//*stds = std::vector<float> (dim,0.0);

	//*** Calculate sample mean and variance (only when training!) ***//
	if ((*centroid)->empty())
	{
		*centroid = new std::vector<float> (dim,0.0);
		*stds = new std::vector<float> (dim,0.0);
		//centroid.reserve(dim);
		//stds.reserve(dim);

		ROS_INFO("Calculating new centroid and stds");
		//ROS_INFO("centroid = %f,%f,%f,%f",centroid[0],centroid[1],centroid[2],centroid[3]);

		// Calculate sample mean
		for (size_t i=0;i<cloudIn.points.size();i++)
		{
			//fRep.copyToFloatArray(cloudIn.points[i],floatArray);
			for (size_t d=0;d<dim;d++)
			{
				(**centroid)[d] += cloudIn.points[i].data[d];//floatArray[d];
			}
		}

		for (size_t d=0;d<dim;d++)
			(**centroid)[d] = (**centroid)[d]/cloudIn.points.size();

		// Calculate sample variance (actually, standard variation)
		for (size_t i=0;i<cloudIn.points.size();i++)
		{
			//fRep.copyToFloatArray(cloudIn.points[i],floatArray);
			for (size_t d=0;d<dim;d++)
			{
				(**stds)[d] += (cloudIn.points[i].data[d]-(**centroid)[d])*(cloudIn.points[i].data[d]-(**centroid)[d]);
			}
		}
		for (size_t d=0;d<dim;d++)
			(**stds)[d] = sqrt((**stds)[d]/cloudIn.points.size());
	}

	//ROS_INFO("centroid = %f",(**centroid)[12]);
	//ROS_INFO("stds = %f,%f,%f,%f",(**stds)[0],(**stds)[1],(**stds)[2],(**stds)[3]);
	//*** Perform normalization ***//
	for (size_t i=0;i<cloudIn.points.size();i++)
	{
		//float* ptr = reinterpret_cast<float*> (&cloudN->points[i]);
		//fRep.copyToFloatArray(cloudIn.points[i],floatArray);
		for (size_t d=0;d<dim;d++)
		{
			//floatArray[d] = (floatArray[d]-(**centroid)[d])/(**stds)[d];

			//ptr[d] = (floatArray[d]-centroid[d])/stds[d];
			//ptr[d] = floatArray[d];
			cloudN->points[i].data[d] = (cloudIn.points[i].data[d]-(**centroid)[d])/(**stds)[d];
		}
		//cloudOut.points[i] = floatArray[0];

		//(reinterpret_cast<const float>cloudOut.points[i]) = floatArray;
		//memcpy(&cloudOut.points[i],floatArray,sizeof(*floatArray));
	}
	pcl::copyPointCloud(*cloudN,cloudOut);
}

//template <typename PointInT, typename PointOutT>
//void colourCloud(const pcl::PointCloud<PointInT> &cloudIn, pcl::PointCloud<PointOutT> &cloudOut, int dim)
//{
//	float S;
//	COLOUR rgb;
//
//	Eigen::Array4f min_p, max_p;
//			min_p.setConstant (FLT_MAX);
//			     max_p.setConstant (-FLT_MAX);
//	for (size_t i = 0; i < cloudIn.points.size (); ++i)
//		   {
//			   pcl::Array4fMapConst pt = cloudIn.points[i].getArray4fMap();
//			   min_p = min_p.min(pt);
//			   max_p = max_p.max(pt);
//		   }
//
//	ROS_INFO("min = %f, max = %f",min_p[dim],max_p[dim]);
//
//	for (int i=0;i<cloudIn.points.size();i++)
//	{
//		rgb = GetColour(cloudIn.points[i].data[dim],(double)min_p[dim],(double)max_p[dim]);
//		cloudOut.points[i].r = rgb.r*255;
//		cloudOut.points[i].g = rgb.g*255;
//		cloudOut.points[i].b = rgb.b*255;
//	}
//}

template <typename PointT>
void visualizeFeature(const pcl::PointCloud<pcl::PointXYZRGB> &cloudRGB, const pcl::PointCloud<PointT> &cloudIn, int dim, int viewPort, std::string text)
{
	float minV = FLT_MAX;
	float maxV = -FLT_MAX;
	visualizeFeature(cloudRGB, cloudIn, dim, viewPort, text, minV, maxV);
}

template <typename PointT>
void visualizeFeature(const pcl::PointCloud<pcl::PointXYZRGB> &cloudRGB, const pcl::PointCloud<PointT> &cloudIn, int dim, int viewPort, std::string text, float minV, float maxV)
{
	//float S;
	COLOUR rgb;

	int dims = sizeof(cloudIn.points[0])/sizeof(float);
	Eigen::ArrayXf min_p(dims), max_p(dims);
	min_p.setConstant (minV);
	max_p.setConstant (maxV);
	if ((minV == FLT_MAX) && (maxV == -FLT_MAX))
	{
		for (size_t i = 0; i < cloudIn.points.size (); ++i)
		{
			Eigen::ArrayXf p_(dims);
			for (int d=0;d<dims;d++)
			{
				p_[d] = cloudIn.points[i].data[d];
			}
			min_p = min_p.min(p_);
			max_p = max_p.max(p_);
		}
	}
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudColor(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::copyPointCloud(cloudRGB,*cloudColor);

	//ROS_INFO("min = %f, max = %f",min_p[dim],max_p[dim]);

	for (int i=0;i<cloudIn.points.size();i++)
	{
		rgb = GetColour(cloudIn.points[i].data[dim],(double)min_p[dim],(double)max_p[dim]);
		cloudColor->points[i].r = rgb.r*255;
		cloudColor->points[i].g = rgb.g*255;
		cloudColor->points[i].b = rgb.b*255;
	}

	//	typename pcl::PointCloud<PointT>::Ptr cloudN(new pcl::PointCloud<PointT>);
	//		std::vector<float>* centroid = new std::vector<float>();
	//		std::vector<float>* stds = new std::vector<float>();
	//
	//		normalizeFeatures(cloudIn, *cloudN, &centroid, &stds);
	//		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudColor(new pcl::PointCloud<pcl::PointXYZRGB>);
	//		pcl::copyPointCloud(cloudRGB,*cloudColor);
	//		for (int i=0;i<cloudIn.points.size();i++)
	//		{
	//			rgb = GetColour(cloudN->points[i].data[dim],(double)-1.0,(double)1.0);
	//			cloudColor->points[i].r = rgb.r*255;
	//			cloudColor->points[i].g = rgb.g*255;
	//			cloudColor->points[i].b = rgb.b*255;
	//		}

	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> color(cloudColor);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloudColor,color,text,viewPort);
	viewer->addText (text, 10, 10, fontsize, 1, 1, 1, text + " text", viewPort);
}

void trainSVM(std::vector<DatasetContainer> dataset, std::vector<std::string> labels, std::string trainingSetfolder, std::string resultsFolder, int ff, int f, std::vector<float>** accuracy)
{
	int dim = useFeatures.size();//sizeof(useFeatures)/sizeof(int);

	ofstream myfile;
	stringstream ss;
	ss << ff;
	stringstream ss2;
	ss2 << f;
	myfile.open ((resultsFolder + ss.str() + "_" + ss2.str() + ".txt").c_str());

	// Distance threshold
	myfile << "DistanceThreshold:" << distThr << ";\n";

	// Neighborhood parameters
	myfile << "Neighborhood:" << rmin << ";" << rmindist << ";" << rmax << ";" << rmaxdist << ";\n";

	// Features
	myfile << "Features:";
	for (int d=0;d<dim;d++)
		myfile << useFeatures[d] << ";";
	myfile << "\n";




	pcl::PointCloud<AllFeatures>::Ptr featuresTrain(new pcl::PointCloud<AllFeatures>);
	pcl::PointCloud<AllFeatures>::Ptr featuresTest(new pcl::PointCloud<AllFeatures>);
	std::vector<std::string> labelsTrain;
	std::vector<std::string> labelsTest;

	std::vector<float> accuracies;


	std::vector<DatasetContainer> trainingSet;
	std::vector<DatasetContainer> testSet;
	std::vector<string> bagNames;
	if (crossvalidation==true)
	{

		for (size_t s=0;s<dataset.size();s++)
		{
			bagNames.push_back(dataset[s].bagName);
		}
		std::sort (bagNames.begin(), bagNames.end());
		std::vector<string>::iterator it;
		it = std::unique (bagNames.begin(), bagNames.end());
		bagNames.resize( std::distance(bagNames.begin(),it) );



	}
	else
	{
		for (size_t s=0;s<dataset.size();s++)
			for (int c=0;c<nClasses;c++)
			{
				*featuresTrain += *dataset[s].features[c];
			}
		bagNames.push_back("noCV");
		*featuresTest = *featuresTrain;
		labelsTrain = labels;
		labelsTest = labels;
	}

	Eigen::Matrix3f confMatSum = Eigen::Matrix3f::Zero();
	for (size_t r=0;r<bagNames.size();r++)
	{
		if (crossvalidation==true)
		{
			featuresTest->clear();
			featuresTrain->clear();
			labelsTest.clear();
			labelsTrain.clear();
			for (size_t s=0;s<dataset.size();s++)
			{
				for (int c=0;c<nClasses;c++)
				{
					if (dataset[s].bagName.compare(bagNames[r]) == 0)
					{
						//testD.features.push_back(dataset[s].features[c]);
						*featuresTest += *dataset[s].features[c];
						labelsTest.insert(labelsTest.end(),dataset[s].labels[c].begin(),dataset[s].labels[c].end());
						//testD.labels.push_back(dataset[s].labels[c]);
					}
					else
					{
						//trainingSet.push_back(dataset[s]);
						//						trainD.features.push_back(dataset[s].features[c]);
						//						trainD.labels.push_back(dataset[s].labels[c]);
						*featuresTrain += *dataset[s].features[c];
						labelsTrain.insert(labelsTrain.end(),dataset[s].labels[c].begin(),dataset[s].labels[c].end());
					}
				}
			}
		}

		//ROS_INFO("trainingSet = %i, testSet = %i",trainingSet.size(),testSet.size());
		ROS_INFO("bagname = %s",bagNames[r].c_str());

		size_t Np = featuresTrain->points.size();

		// Normalize features
		std::vector<float>* centroid = new std::vector<float>();
		std::vector<float>* stds = new std::vector<float>();
		normalizeFeatures(*featuresTrain,*featuresTrain, &centroid, &stds);

//		// Save normalization parameters
//		std::ofstream output_file((trainingSetfolder + "centroid").c_str());
//		std::ostream_iterator<float> output_iterator(output_file,"\n");
//		std::copy((*centroid).begin(), (*centroid).end(), output_iterator);
//		output_file.close();
//		std::ofstream output_file2((trainingSetfolder + "stds").c_str());
//		std::ostream_iterator<float> output_iterator2(output_file2,"\n");
//		std::copy((*stds).begin(), (*stds).end(), output_iterator2);
//		output_file2.close();



		float labelsTrainingSVM[Np];
		float trainingData[Np][dim];

		ROS_INFO("Np = %i",Np);

		// Convert training data to SVM labels and data
		for (size_t i=0;i<Np;i++)
		{
			//ROS_INFO("i = %i",i);
			for (int d=0;d<dim;d++)
			{
				//trainingData[i][d] = featureCloudAcc->points[i].data[d];
				trainingData[i][d] = featuresTrain->points[i].data[useFeatures[d]];
			}
			if (labelsTrain[i].compare("ground") == 0)
				labelsTrainingSVM[i] = 1.0;
			else if (labelsTrain[i].compare("vegetation") == 0)
				labelsTrainingSVM[i] = 2.0;
			else if (labelsTrain[i].compare("object") == 0)
				labelsTrainingSVM[i] = 3.0;
			else
				ROS_INFO("Not labeled");
		}
		Mat trainingLabelsMat(Np, 1, CV_32FC1, labelsTrainingSVM);
		Mat trainingDataMat(Np, dim, CV_32FC1, trainingData);

		// Convert test data to SVM labels and data
		normalizeFeatures(*featuresTest,*featuresTest, &centroid, &stds);
		Np = featuresTest->points.size();
		float labelsTestSVM[Np];
		float testData[Np][dim];
		for (size_t i=0;i<Np;i++)
		{
			//ROS_INFO("i = %i",i);
			for (int d=0;d<dim;d++)
			{
				//trainingData[i][d] = featureCloudAcc->points[i].data[d];
				testData[i][d] = featuresTest->points[i].data[useFeatures[d]];
			}
			if (labelsTest[i].compare("ground") == 0)
				labelsTestSVM[i] = 1.0;
			else if (labelsTest[i].compare("vegetation") == 0)
				labelsTestSVM[i] = 2.0;
			else if (labelsTest[i].compare("object") == 0)
				labelsTestSVM[i] = 3.0;
			else
				ROS_INFO("Not labeled");
		}
		Mat testLabelsMat(Np, 1, CV_32FC1, labelsTestSVM);
		Mat testDataMat(Np, dim, CV_32FC1, testData);

		// Set up SVM's parameters



		ROS_INFO("Training SVM");
		CvSVM SVM;
		CvSVMParams params;
		//						params.svm_type    = CvSVM::C_SVC;
		//						params.kernel_type = CvSVM::LINEAR;
		//						params.term_crit   = cvTermCriteria(CV_TERMCRIT_ITER, 10000, 1e-6);

		params.svm_type    = CvSVM::C_SVC;
		//params.C = 0.1;
		params.kernel_type = CvSVM::RBF;
		//params.gamma = 1.0;
		params.term_crit   = cvTermCriteria(CV_TERMCRIT_ITER, 10000, 1e-7);

		// Train the SVM
		SVM.train(trainingDataMat, trainingLabelsMat, Mat(), Mat(), params);
		//SVM.train_auto(trainingDataMat, labelsMat, Mat(), Mat(), CvSVMParams());
		//SVM.train_auto(trainingDataMat, labelsMat, Mat(), Mat(), params);


		// Save training data
		SVM.save((trainingSetfolder + "svm").c_str());
		SVM.save((trainingSetfolder + "svm" + bagNames[r]).c_str());

		ROS_INFO("SVM training done");

		Mat testDataResultsMat;
		SVM.predict(testDataMat, testDataResultsMat);

		//ROS_INFO("trainingDataResultsMat = %i, %i",trainingDataResultsMat.rows,trainingDataResultsMat.cols);
		//ROS_INFO("val = %f",trainingDataResultsMat.at<float>(1));

		Eigen::Matrix3f confMat = Eigen::Matrix3f::Zero();
		for (int i=0;i<testDataResultsMat.rows;i++)
		{
			confMat((int)testLabelsMat.at<float>(i)-1,(int)testDataResultsMat.at<float>(i)-1)++;
		}
		std::cout << confMat << std::endl;
		confMatSum += confMat;

		float accuracyTmp = confMat.diagonal().sum()/confMat.sum();

		accuracies.push_back(accuracyTmp);

		ROS_INFO("accuracy = %f",accuracyTmp);


		myfile << "ConfusionMatrix:" << bagNames[r] << ";";
		for (int r=0;r<confMat.rows();r++)
			for (int c=0;c<confMat.cols();c++)
				myfile << confMat(r,c) << ";";
		myfile << "\n";

		myfile << "Accuracy:" << bagNames[r] << ";" << accuracyTmp << ";\n";

	}
	myfile.close();

	//	float accuracyTmp = 0;
	//	for (int i=0;i<accuracies.size();i++)
	//		accuracyTmp += accuracies[i];
	//	accuracyTmp /= accuracies.size();
	float accuracyTmp = confMatSum.diagonal().sum()/confMatSum.sum();
	(*accuracy)->push_back(accuracyTmp);
}

//template <typename PointT>
//void trainSVM2(const pcl::PointCloud<PointT> &featureCloudAcc, std::vector<std::string> labels, std::string trainingSetfolder, std::string resultsFolder, int ff, int f, std::vector<float>** accuracy)
//{
//	int dim = useFeatures.size();//sizeof(useFeatures)/sizeof(int);
//	int Np = featureCloudAcc.points.size();
//	float labelsSVM[Np];
//	float trainingData[Np][dim];
//
//	// Convert data to SVM labels and data
//	for (size_t i=0;i<Np;i++)
//	{
//		//ROS_INFO("i = %i",i);
//		for (int d=0;d<dim;d++)
//		{
//			//trainingData[i][d] = featureCloudAcc->points[i].data[d];
//			trainingData[i][d] = featureCloudAcc.points[i].data[useFeatures[d]];
//		}
//		if (labels[i].compare("ground") == 0)
//			labelsSVM[i] = 1.0;
//		else if (labels[i].compare("vegetation") == 0)
//			labelsSVM[i] = 2.0;
//		else if (labels[i].compare("object") == 0)
//			labelsSVM[i] = 3.0;
//		else
//			ROS_INFO("Not labeled");
//	}
//	Mat labelsMat(Np, 1, CV_32FC1, labelsSVM);
//	Mat trainingDataMat(Np, dim, CV_32FC1, trainingData);
//
//	// Set up SVM's parameters
//
//
//
//	ROS_INFO("Training SVM");
//	CvSVM SVM;
//	CvSVMParams params;
//	//						params.svm_type    = CvSVM::C_SVC;
//	//						params.kernel_type = CvSVM::LINEAR;
//	//						params.term_crit   = cvTermCriteria(CV_TERMCRIT_ITER, 10000, 1e-6);
//
//	params.svm_type    = CvSVM::C_SVC;
//	//params.C = 0.1;
//	params.kernel_type = CvSVM::RBF;
//	//params.gamma = 1.0;
//	params.term_crit   = cvTermCriteria(CV_TERMCRIT_ITER, 10000, 1e-7);
//
//	// Train the SVM
//	SVM.train(trainingDataMat, labelsMat, Mat(), Mat(), params);
//	//SVM.train_auto(trainingDataMat, labelsMat, Mat(), Mat(), CvSVMParams());
//	//SVM.train_auto(trainingDataMat, labelsMat, Mat(), Mat(), params);
//
//
//	// Save traning data
//	SVM.save((trainingSetfolder + "svm").c_str());
//
//	ROS_INFO("SVM training done");
//
//	Mat trainingDataResultsMat;
//	SVM.predict(trainingDataMat, trainingDataResultsMat);
//
//	//ROS_INFO("trainingDataResultsMat = %i, %i",trainingDataResultsMat.rows,trainingDataResultsMat.cols);
//	//ROS_INFO("val = %f",trainingDataResultsMat.at<float>(1));
//
//	Eigen::Matrix3f confMat = Eigen::Matrix3f::Zero();
//	for (int i=0;i<trainingDataResultsMat.rows;i++)
//	{
//		confMat((int)labelsMat.at<float>(i)-1,(int)trainingDataResultsMat.at<float>(i)-1)++;
//	}
//	std::cout << confMat << std::endl;
//
//	(*accuracy)->push_back(confMat.diagonal().sum()/confMat.sum());
//
//	ROS_INFO("accuracy = %f",(**accuracy)[f]);
//
//
//	ofstream myfile;
//	stringstream ss;
//	ss << ff;
//	stringstream ss2;
//	ss2 << f;
//	myfile.open ((resultsFolder + ss.str() + "_" + ss2.str() + ".txt").c_str());
//
//	// Distance threshold
//	myfile << "DistanceThreshold:" << distThr << ";\n";
//
//	// Neighborhood parameters
//	myfile << "Neighborhood:" << rmin << ";" << rmindist << ";" << rmax << ";" << rmaxdist << ";\n";
//
//	// Features
//	myfile << "Features:";
//	for (int d=0;d<dim;d++)
//		myfile << useFeatures[d] << ";";
//	myfile << "\n";
//
//	myfile << "ConfusionMatrix:";
//	for (int r=0;r<confMat.rows();r++)
//		for (int c=0;c<confMat.cols();c++)
//			myfile << confMat(r,c) << ";";
//	myfile << "\n";
//
//	myfile << "Accuracy:" << (**accuracy)[f] << ";\n";
//	myfile.close();
//}

#endif /* PCL_ROS_SRC_HELPINGFUNCTIONS_H_ */
