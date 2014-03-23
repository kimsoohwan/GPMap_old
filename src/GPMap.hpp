#ifndef _GAUSSIAN_PROCESS_MAP_HPP_
#define _GAUSSIAN_PROCESS_MAP_HPP_

// STL
#include <string>

// pcl
#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>

// GPMap
#include "OctreeContainerDataTGaussianVector.hpp"

namespace GPMap{

	template<typename Scalar, class PointT, 
				class LocalMeanFunc,  class LocalCovFunc,  class LocalLikFunc,  template<class, class, class> class LocalInfMethod,
				class GlobalMeanFunc, class GlobalCovFunc, class GlobalLikFunc, template<class, class, class> class GlobalInfMethod>
	class GPMap : public pcl::octree::OctreePointCloud<PointT, OctreeContainerPointTGaussianVector<Scalar> >
	{
	public:
		/** \brief Constructor. */
		GPMap(const Scalar blockSize, const Scalar abstractBlockSize) : 
			OctreePointCloud(blockSize), 
			abstractBlockSize_(abstractBlockSize)
		{
			// set the center as the origin
			this->defineBoundingBox(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
		}

		/** \brief Destructor */
		virtual ~GPMap()
		{
		}

		/** \brief Set the hyperparameters of Gaussian processes
		  * \param[in] meanLogHyp log mean hyperparameter
		  * \param[in] covLogHyp log covariance hyperparameter
		  * \param[in] likLogHyp log likelihood hyperparameter
		  */
		void setHyperparameters(const LocalMeanFunc::MeanHyp		&meanLogHyp,
										const LocalCovFunc::CovHyp			&covLogHyp,
										const LocalLikFunc::LikHyp			&likLogHyp,
										const GlobalMeanFunc::MeanHyp		&meanLogHyp,
										const GlobalCovFunc::CovHyp		&covLogHyp,
										const GlobalLikFunc::LikHyp		&likLogHyp)
		{
			globaGP_.setHyperparameters(globalmeanLogHyp, globalMeanLogHyp, globalCovLogHyp, globalInfLogHyp);
			localGP_.setHyperparameters(globaGP_, localMeanLogHyp, localCovLogHyp, localInfLogHyp);
		}

		/** \brief Update the map with new observations
		  * \param[in] robotPosition robot position which is needed for estimating normal vectors
		  * \param[in] cloud point cloud
		  */
		void update(const pcl::PointXYZ robotPosition, const pcl::PointCloud<PointT>::Ptr cloud)
		{
			// Step 1: Estimate normal vectors
			pcl::PointNormals pointNormals;

			// Step 2: Add points to the octree
			this->setInputCloud(cloud);
			this->addPointsFromInputCloud();

			// Step 3: Estimate abstract/global observations
			OctreeAbstractPointCloud octreeAbstractPointCloud(abstractBlockSize_);
			octreeAbstractPointCloud.setInputCloud(pointNormals);
			octreeAbstractPointCloud.addPointsFromInputCloud();
			pcl::PointNormals abstractPointNormal = octreeAbstractPointCloud.getAbstractPointCloud();

			// Step 4: Global Gaussian process for the mean function
			globaGP_.setTrainingData(abstractPointNormal);

			// Step 5: For each block, update the means and variances
			//         If only positives/negatives, prune the node.
			update(localGP_);
			// 4.1 calculate means
			// 4.2 calculate variances
			// 4.3 update means and variances
			// 4.4 if only one kind of signs (positives/negatives) are found in means, prune the node
		}

		/** \brief Save the map
		  * \param[in] filename file name
		  */
		void save(const std::string &filename) const
		{
		}

	protected:
		Scalar	abstractBlockSize_; // block size for abstracting point cloud
		GaussianProcess<LocalMeanFunc,  LocalCovFunc,  LocalLikFunc,  LocalInfMethod>			localGP_;	// Gaussian process
		GaussianProcess<GlobalMeanFunc, GlobalCovFunc, GlobalLikFunc, GlobalInfMethod>		globalGP_;	// Gaussian process for means
	};
}

#endif // _GAUSSIAN_PROCESS_MAP_HPP_