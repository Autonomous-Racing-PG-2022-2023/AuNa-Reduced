// Von https://github.com/james-yoo/DBSCAN geklaut

#ifndef DBSCAN_H
#define DBSCAN_H

#include <vector>
#include <limits>

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree_search.h>

typedef pcl::PointXYZRGBL Point_;

class DBSCAN
{
public:
	//NOTE: Cannot use const here, cause make_shared then will try to capture these values by reference what fails: https://stackoverflow.com/questions/46722050/undefined-reference-to-const-int-within-shared-ptr
	static constexpr int MAX_VALUE = std::numeric_limits<int>::max();
	static constexpr int UNCLASSIFIED = (MAX_VALUE - 1);
	static constexpr int CORE_POINT = 1;
	static constexpr int BORDER_POINT = 2;
	static constexpr int NOISE = (MAX_VALUE - 2);
	static constexpr int SUCCESS = 0;
	static constexpr int FAILURE = -3;
	
    DBSCAN(pcl::PointCloud<pcl::PointXYZRGBL>::Ptr points, unsigned int minPoints, float eps)
    : m_points(points), m_minPoints(minPoints), m_epsilon(eps)
	{}
	
    ~DBSCAN()
    {}

    uint32_t run(const pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGBL>& octree, const pcl::IndicesConstPtr indices, uint32_t firstClusterID);
	uint32_t run(const pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGBL>& octree, uint32_t firstClusterID);
    int expandCluster(const pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGBL>& octree, Point_& point, uint32_t clusterID);

    private:
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr m_points;
	unsigned int m_minPoints;
	float m_epsilon;
};

#endif // DBSCAN_H
