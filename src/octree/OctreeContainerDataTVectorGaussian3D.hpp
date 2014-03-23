#ifndef _OCTREE_CONTAINER_DATAT_VECTOR_GAUSSIAN_3D_HPP_
#define _OCTREE_CONTAINER_DATAT_VECTOR_GAUSSIAN_3D_HPP_

// PCL
#include <pcl/octree/octree.h>
#include <pcl/common/centroid.h> // pcl::computeMeanAndCovarianceMatrix
#include <pcl/common/eigen.h>		// pcl::invert3x3SymMatrix

namespace GPMap
{	
	/** \brief @b Octree leaf class that does store a vector of int elements and a 3D Gaussian Distribution.
	* \note Enables the octree to store multiple int elements within its leaf nodes.
	*/
	class OctreeContainerDataTVectorGaussian3D : public pcl::octree::OctreeContainerDataTVector<int>
	{
	public:
		/** \brief Empty constructor. */
		OctreeContainerDataTVectorGaussian3D()
		{
			sumInvSigmaMu_.setZero();
			sumInvSigma_.setZero();
		}

		/** \brief Copy constructor. */
		OctreeContainerDataTVectorGaussian3D(const OctreeContainerDataTVectorGaussian3D &source) :
			pcl::octree::OctreeContainerDataTVector<int>(source),
			sumInvSigmaMu_(source.sumInvSigmaMu_),
			sumInvSigma_(source.sumInvSigma_)
		{
		}

		/** \brief Empty deconstructor. */
 		virtual ~OctreeContainerDataTVectorGaussian3D()
		{
		}

		/** \brief Octree deep copy method */
		virtual OctreeContainerDataTVectorGaussian3D* deepCopy() const
		{
			return (new OctreeContainerDataTVectorGaussian3D(*this));
		}

		/** \brief Reset leaf node. Clear int vector.*/
		void reset()
		{
			pcl::octree::OctreeContainerDataTVector<int>::reset();
			sumInvSigmaMu_.setZero();
			sumInvSigma_.setZero();
		}

		/** \brief Compute the mean and covariance.*/
		template<typename PointT>
		bool update(const pcl::PointCloud<PointT> &cloud)
		{
			// minimum number of points
			if(getSize() >= 3)
			{
				leafDataTVector_.clear();
				return false;
			}

			// new mean vector and covariance matrix
			Eigen::Vector4f mu;
			Eigen::Matrix3f Sigma;
			if(pcl::computeMeanAndCovarianceMatrix(cloud, leafDataTVector_, Sigma, mu) < 3) return false;

			// update
			Eigen::Matrix3f invSigma;
			if(pcl::invert3x3SymMatrix(Sigma, invSigma) == 0.f) // Sigma_k^{-1}
			{
				leafDataTVector_.clear();
				return false;
			}

			// Sigma = (\sum_k Sigma_k^{-1})^{-1}
			sumInvSigma_ += invSigma;

			// mu = Sigma \sum_k Sigma_k^{-1} mu_k
			sumInvSigmaMu_ += invSigma * Eigen::Vector3f(mu.coeff(0), mu.coeff(1), mu.coeff(2));

			// consume the points
			leafDataTVector_.clear();

			return true;
		}

	protected:
		/** \brief 3x1 vector: \sum_k Sigma_k^{-1} mu_k */
		Eigen::Vector3f sumInvSigmaMu_;

		/** \brief 3x3 matrix: \sum_k Sigma_k^{-1} */
		Eigen::Matrix3f sumInvSigma_;
	};
}

#endif // _OCTREE_CONTAINER_DATAT_VECTOR_GAUSSIAN_3D_HPP_