/*
 * PlaneOrthogonalStatistics.h
 *
 *  Created on: Feb 13, 2015
 *      Author: mikkel
 */

#ifndef PCL_ROS_SRC_PLANEORTHOGONALSTATISTICS_H_
#define PCL_ROS_SRC_PLANEORTHOGONALSTATISTICS_H_

#include <pcl/features/feature.h>
#include <pcl/common/centroid.h>
//#include <pcl/point_types.h>
//#include <pcl/impl/instantiate.hpp>

using namespace pcl;

namespace pcl {

template <typename PointInT, typename PointOutT>
class PlaneOrthogonalStatistics: public Feature<PointInT, PointOutT>
{
public:
	//	typedef boost::shared_ptr<MyMomentInvariantsEstimation<PointInT, PointOutT> > Ptr;
	//	typedef boost::shared_ptr<const MyMomentInvariantsEstimation<PointInT, PointOutT> > ConstPtr;
	typedef typename Feature<PointInT, PointOutT>::PointCloudOut PointCloudOut;
	//	using Feature<PointInT, PointOutT>::feature_name_;

	typedef boost::shared_ptr<PlaneOrthogonalStatistics<PointInT, PointOutT> > Ptr;
	typedef boost::shared_ptr<const PlaneOrthogonalStatistics<PointInT, PointOutT> > ConstPtr;
	using Feature<PointInT, PointOutT>::feature_name_;
	using Feature<PointInT, PointOutT>::getClassName;
	using Feature<PointInT, PointOutT>::indices_;
	using Feature<PointInT, PointOutT>::k_;
	using Feature<PointInT, PointOutT>::search_parameter_;
	using Feature<PointInT, PointOutT>::surface_;
	using Feature<PointInT, PointOutT>::input_;

	/** \brief Empty constructor. */
	PlaneOrthogonalStatistics () : xyz_centroid_ (), temp_pt_ ()
	{
		feature_name_ = "PlaneOrthogonalStatistics";
	};

	/** \brief Estimate moment invariants for all points given in <setInputCloud (), setIndices ()> using the surface
	 * in setSearchSurface () and the spatial locator in setSearchMethod ()
	 * \param[out] output the resultant point cloud model dataset that contains the moment invariants
	 */
	void
	computePointMomentInvariants (
			const pcl::PointCloud<PointInT> &cloud, const std::vector<int> &indices,
			float &GPDistMean, float &GPDistVar)
	{

		float orthVar = 0;
		//		// Estimate the XYZ centroid
		compute3DCentroid (cloud, indices, xyz_centroid_);

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmpCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		for (size_t nn_idx = 0; nn_idx < indices.size (); ++nn_idx)
		{
			tmpCloud->push_back(cloud.points[indices[nn_idx]]);
		}

		if (tmpCloud->size() > 3)
		{
			pcl::ModelCoefficients ground_coefficients;
			pcl::PointIndices ground_indices;
			pcl::SACSegmentation<PointInT> plane_finder;
			plane_finder.setOptimizeCoefficients(true);
			plane_finder.setModelType(pcl::SACMODEL_PLANE);
			plane_finder.setMethodType(pcl::SAC_RANSAC);
			plane_finder.setDistanceThreshold(0.15);
			plane_finder.setInputCloud(tmpCloud);

			//		plane_finder.setInputCloud(cloud.makeShared());
			//		boost::shared_ptr<std::vector<int> > indicesptr (new std::vector<int> (indices));
			//		plane_finder.setIndices(indicesptr);

			plane_finder.segment(ground_indices, ground_coefficients);

			//ROS_INFO("indices.size () = %d",indices.size ());
			//
			//		// Initalize the centralized moments

			//		float dist;
			//		// Iterate over the nearest neighbors set
			//		//float vect[indices.size ()] = {};
			std::vector<float> vect;
			//		for (size_t nn_idx = 0; nn_idx < indices.size (); ++nn_idx)
			//		{
			//			vect.push_back((ground_coefficients.values[0]*cloud.points[indices[nn_idx]].x+ground_coefficients.values[1]*cloud.points[indices[nn_idx]].y+ground_coefficients.values[2]*cloud.points[indices[nn_idx]].z+ground_coefficients.values[3])/sqrt(ground_coefficients.values[0]*ground_coefficients.values[0]+ground_coefficients.values[1]*ground_coefficients.values[1]+ground_coefficients.values[2]*ground_coefficients.values[2]));
			//		}

			// Ide: normaliser med udstrækningen af neighborhood

			// Calculate distances from points to plane
			float sMean = 0;
			float tmp = 0;
			for (size_t nn_idx = 0; nn_idx < tmpCloud->size(); ++nn_idx)
			{
				tmp = (ground_coefficients.values[0]*tmpCloud->points[nn_idx].x+ground_coefficients.values[1]*tmpCloud->points[nn_idx].y+ground_coefficients.values[2]*tmpCloud->points[nn_idx].z+ground_coefficients.values[3])/sqrt(ground_coefficients.values[0]*ground_coefficients.values[0]+ground_coefficients.values[1]*ground_coefficients.values[1]+ground_coefficients.values[2]*ground_coefficients.values[2]);
				vect.push_back(tmp);
				sMean += tmp;
			}

			sMean = sMean/tmpCloud->size();

			// Calculate variance
			for (size_t nn_idx = 0; nn_idx < tmpCloud->size(); ++nn_idx)
			{
				// Demean the points
				orthVar += (vect[nn_idx]-sMean)*(vect[nn_idx]-sMean);
			}
		}

		//ROS_INFO("indices.size () = %d",tmpCloud->size());
		// Save the moment invariants
		GPDistMean = xyz_centroid_[2];
		GPDistVar = orthVar/indices.size(); //tmpCloud->size();//
		//j3 = 0;
	}

protected:
	void
	computeFeature (PointCloudOut &output)
	{
		// Allocate enough space to hold the results
		// \note This resize is irrelevant for a radiusSearch ().
		std::vector<int> nn_indices (k_);
		std::vector<float> nn_dists (k_);

		output.is_dense = true;
		// Save a few cycles by not checking every point for NaN/Inf values if the cloud is set to dense
		if (input_->is_dense)
		{
			// Iterating over the entire index vector
			for (size_t idx = 0; idx < indices_->size (); ++idx)
			{
				if (this->searchForNeighbors ((*indices_)[idx], search_parameter_, nn_indices, nn_dists) == 0)
				{
					output.points[idx].GPDistMean = output.points[idx].GPDistVar = std::numeric_limits<float>::quiet_NaN ();
					output.is_dense = false;
					continue;
				}

				computePointMomentInvariants (*surface_, nn_indices,
						output.points[idx].GPDistMean, output.points[idx].GPDistVar);
			}
		}
		else
		{
			// Iterating over the entire index vector
			for (size_t idx = 0; idx < indices_->size (); ++idx)
			{
				if (!isFinite ((*input_)[(*indices_)[idx]]) ||
						this->searchForNeighbors ((*indices_)[idx], search_parameter_, nn_indices, nn_dists) == 0)
				{
					output.points[idx].GPDistMean = output.points[idx].GPDistVar = std::numeric_limits<float>::quiet_NaN ();
					output.is_dense = false;
					continue;
				}

				computePointMomentInvariants (*surface_, nn_indices,
						output.points[idx].GPDistMean, output.points[idx].GPDistVar);
			}
		}
	}

private:
	/** \brief 16-bytes aligned placeholder for the XYZ centroid of a surface patch. */
	Eigen::Vector4f xyz_centroid_;

	/** \brief Internal data vector. */
	Eigen::Vector4f temp_pt_;
	//}
};

} /* namespace pcl */



#endif /* PCL_ROS_SRC_PLANEORTHOGONALSTATISTICS_H_ */
