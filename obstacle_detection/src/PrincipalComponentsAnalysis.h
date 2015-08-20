/*
 * PrincipalComponentsAnalysis.h
 *
 *  Created on: Feb 15, 2015
 *      Author: mikkel
 */

#ifndef PCL_ROS_SRC_PRINCIPALCOMPONENTSANALYSIS_H_
#define PCL_ROS_SRC_PRINCIPALCOMPONENTSANALYSIS_H_

#include <pcl/features/feature.h>
#include <pcl/common/centroid.h>
#include "obstacle_detection.h"
//#include <pcl/point_types.h>
//#include <pcl/impl/instantiate.hpp>

using namespace pcl;

namespace pcl {

template <typename PointInT, typename PointOutT>
class PrincipalComponentsAnalysis: public Feature<PointInT, PointOutT>
{
public:
	typedef typename Feature<PointInT, PointOutT>::PointCloudOut PointCloudOut;

	typedef boost::shared_ptr<PrincipalComponentsAnalysis<PointInT, PointOutT> > Ptr;
	typedef boost::shared_ptr<const PrincipalComponentsAnalysis<PointInT, PointOutT> > ConstPtr;
	using Feature<PointInT, PointOutT>::feature_name_;
	using Feature<PointInT, PointOutT>::getClassName;
	using Feature<PointInT, PointOutT>::indices_;
	using Feature<PointInT, PointOutT>::k_;
	using Feature<PointInT, PointOutT>::search_parameter_;
	using Feature<PointInT, PointOutT>::surface_;
	using Feature<PointInT, PointOutT>::input_;

	/** \brief Empty constructor. */
	PrincipalComponentsAnalysis () : xyz_centroid_ (), temp_pt_ ()
	{
		feature_name_ = "PrincipalComponentsAnalysis";
	};

	void
	computePointMomentInvariants (
			const pcl::PointCloud<PointInT> &cloud, const std::vector<int> &indices,
			float &PCA1, float &PCA2, float &PCA3, float &normal_x, float &normal_y, float &normal_z)
	{
		// Estimate the XYZ centroid
		compute3DCentroid (cloud, indices, xyz_centroid_);

		// Placeholder for the 3x3 covariance matrix at each surface patch
		EIGEN_ALIGN16 Eigen::Matrix3f covariance_matrix;

		// Compute the 3x3 covariance matrix
		pcl::computeCovarianceMatrixNormalized (cloud, indices, xyz_centroid_, covariance_matrix);

		// Extract the eigenvalues and eigenvectors
		EIGEN_ALIGN16 Eigen::Vector3f eigen_values;
		EIGEN_ALIGN16 Eigen::Matrix3f eigen_vectors;
		pcl::eigen33 (covariance_matrix, eigen_vectors, eigen_values); // eigen values in ascending order

		normal_x = eigen_vectors(0,0);
		normal_y = eigen_vectors(1,0);
		normal_z = fabs(eigen_vectors(2,0));

		//ROS_INFO("test1 = %f, test2 = %f",eigen_vectors(2,0), fabs(eigen_vectors(2,0)));

		// Save eigenvalues in ascending order
		PCA1 = eigen_values[0];
		PCA2 = eigen_values[1];
		PCA3 = eigen_values[2];
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
					output.points[idx].PCA1 = output.points[idx].PCA2 = output.points[idx].PCA3 = output.points[idx].normal_x = output.points[idx].normal_y = output.points[idx].normal_z = std::numeric_limits<float>::quiet_NaN ();
					output.is_dense = false;
					continue;
				}

				computePointMomentInvariants (*surface_, nn_indices,
						output.points[idx].PCA1, output.points[idx].PCA2, output.points[idx].PCA3, output.points[idx].normal_x, output.points[idx].normal_y, output.points[idx].normal_z);
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
					output.points[idx].PCA1 = output.points[idx].PCA2 = output.points[idx].PCA3 = output.points[idx].normal_x = output.points[idx].normal_y = output.points[idx].normal_z = std::numeric_limits<float>::quiet_NaN ();
					output.is_dense = false;
					continue;
				}

				computePointMomentInvariants (*surface_, nn_indices,
						output.points[idx].PCA1, output.points[idx].PCA2, output.points[idx].PCA3, output.points[idx].normal_x, output.points[idx].normal_y, output.points[idx].normal_z);
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



#endif /* PCL_ROS_SRC_PRINCIPALCOMPONENTSANALYSIS_H_ */
