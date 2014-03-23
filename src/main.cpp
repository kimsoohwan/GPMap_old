// STL
#include <ctime>

// PCL
#include <pcl/point_cloud.h>

// GPMap
#include "octree/OctreePointCloudVoxelGaussian.hpp"

int main(int argc, char** argv)
{
	// create a random point cloud
	srand((unsigned int) time(NULL));

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	// Generate pointcloud data
	cloud->width = 100;
	cloud->height = 1;
	cloud->points.resize(cloud->width * cloud->height);

	for(size_t i = 0; i < cloud->points.size(); ++i)
	{
		cloud->points[i].x = 1024.0f * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].y = 1024.0f * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].z = 1024.0f * rand() / (RAND_MAX + 1.0f);
	}

	// octree
	const float resolution = 128.0f;
	//pcl::octree::OctreePointCloud<pcl::PointXYZ> octree(resolution);
	//octree.setInputCloud(cloud);
	//octree.addPointsFromInputCloud();
	GPMap::OctreePointCloudVoxelGaussian3D<pcl::PointXYZ> octree(resolution);
	octree.addPointsFromCloud(cloud);

	//pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZ> centroid(resolution);
	//centroid.setInputCloud(cloud);
	//centroid.addPointsFromInputCloud();
	//pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZ>::LeafNodeIterator leafIter(centroid);
	//while(*++leafIter)
	//{
	//	dynamic_cast<OctreeContainerDataTVectorGaussian3D*>(*leafIter)->update(*input_);
	//}

  return 0;
}
