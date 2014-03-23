#ifndef _OCTREE_POINT_CLOUD_VOXEL_GAUSSIAN_HPP_
#define _OCTREE_POINT_CLOUD_VOXEL_GAUSSIAN_HPP_

// OpenMP
#include <omp.h>

// PCL
#include <pcl/octree/octree_impl.h> // pcl::octree::OctreePointCloud

#include "octree/OctreeContainerDataTVectorGaussian3D.hpp"

namespace GPMap
{	
	template<typename PointT>
	class OctreePointCloudVoxelGaussian3D : public pcl::octree::OctreePointCloud<PointT, OctreeContainerDataTVectorGaussian3D>
	{
	public:
		/** \brief OctreePointCloudVoxelGaussian3D class constructor.
		*  \param resolution_arg: octree resolution at lowest octree level
		* */
		OctreePointCloudVoxelGaussian3D(const double resolution_arg) :
			pcl::octree::OctreePointCloud<PointT, OctreeContainerDataTVectorGaussian3D>(resolution_arg)
		{
			defineBoundingBox(resolution_arg);
		}

		/** \brief Empty class deconstructor. */
		virtual ~OctreePointCloudVoxelGaussian3D()
		{
		}

      /** \brief Add points from input point cloud to octree.
         * \param[in] cloud_arg the const boost shared pointer to a PointCloud message
         * \param[in] indices_arg the point indices subset that is to be used from \a cloud - if 0 the whole point cloud is used
			*/
      void addPointsFromCloud(const PointCloudConstPtr &cloud_arg,
										const IndicesConstPtr &indices_arg = IndicesConstPtr())
		{
			// set member variables
			input_ = cloud_arg;
			indices_ = indices_arg;

			// add points
			if(indices_)
			{
				for(std::vector<int>::const_iterator current = indices_->begin(); current != indices_->end(); ++current)
				{
					if(isFinite(input_->points[*current]))
					{
						assert( (*current>=0) && (*current < static_cast<int> (input_->points.size())));

						// add points to octree
						this->addPointIdx(*current);
					}
				}
			}
			else
			{
				for(size_t i = 0; i < input_->points.size(); i++)
				{
					if(isFinite(input_->points[i]))
					{
						// add points to octree
						this->addPointIdx(static_cast<unsigned int> (i));
					}
				}
			}

			// consume points
			#if (defined(_WIN32) || ((__GNUC__ > 4) && (__GNUC_MINOR__ > 2))) &&	(defined(_OMPIMP) && (_OPENMP < 200805)) // OpenMP 2.0: no Task
				// store all leaf node iterator in a vector
				std::vector<LeafNodeIterator> leafNodeVector;
				LeafNodeIterator leafIter(*this);
				for(++leafIter; *leafIter != NULL; ++leafIter)
					leafNodeVector.push_back(leafIter);

				// omp parallel for
				//double runtime = omp_get_wtime();
				#pragma omp parallel for schedule (dynamic, 1)
				for(int i = 0; i < leafNodeVector.size(); i++)
				{
					pcl::octree::OctreeKey key = leafNodeVector[i].getCurrentOctreeKey();
					//#pragma omp critical
					//std::cout << "key = " << key.x << ", " << key.y << ", " << key.z << " (" << leafNodeVector[i].getNodeID() << ")" << std::endl;
					dynamic_cast<OctreeContainerDataTVectorGaussian3D*>(*(leafNodeVector[i]))->update(*input_);
				}
				//std::cout << "runtime = " << omp_get_wtime() - runtime << " sec" << std::endl;
			#else
				#pragma omp parallel
				{
					#pragma omp single
					{
						LeafNodeIterator leafIter(*this);
						while(*++leafIter)
						{
							#pragma omp task firstprivate(leafIter)
							dynamic_cast<OctreeContainerDataTVectorGaussian3D*>(*leafIter)->update(*input_);
						}
					}
				}
			#endif
		}
	};

}

#endif // _OCTREE_POINT_CLOUD_VOXEL_GAUSSIAN_HPP_