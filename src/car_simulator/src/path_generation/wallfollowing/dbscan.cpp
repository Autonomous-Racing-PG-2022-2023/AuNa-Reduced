#include "dbscan.hpp"

#include <algorithm>
#include <execution>

uint32_t DBSCAN::run(const pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGBL>& octree, const pcl::IndicesConstPtr indices, uint32_t firstClusterID)
{
    uint32_t clusterID = firstClusterID;
    for (const size_t index : *indices)
    {
        if (m_points->at(index).label == UNCLASSIFIED)
        {
            if (expandCluster(octree, m_points->at(index), clusterID) != FAILURE)
            {
                clusterID += 1;
            }
        }
    }

    return clusterID;
}

uint32_t DBSCAN::run(const pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGBL>& octree, uint32_t firstClusterID)
{
    uint32_t clusterID = firstClusterID;
    for (size_t index = 0; index < m_points->size(); index++)
    {
        if (m_points->at(index).label == UNCLASSIFIED)
        {
            if (expandCluster(octree, m_points->at(index), clusterID) != FAILURE)
            {
                clusterID += 1;
            }
        }
    }

    return clusterID;
}

int DBSCAN::expandCluster(const pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGBL>& octree, Point_& point, uint32_t clusterID)
{
    pcl::Indices clusterSeeds;
	std::vector<float> squared_distances;
	octree.radiusSearch(point, m_epsilon, clusterSeeds, squared_distances);

    if (clusterSeeds.size() < m_minPoints)
    {
        point.label = NOISE;
        return FAILURE;
    }
    else
    {
		std::for_each(std::execution::par_unseq, clusterSeeds.begin(), clusterSeeds.end(), [this, &clusterID](const uint32_t index){
			m_points->at(index).label = clusterID;
		});

		//Add neighbours of neighbours
        for (std::vector<uint32_t>::size_type i = 0, n = clusterSeeds.size(); i < n; ++i)
        {
            pcl::Indices clusterNeighors;
			octree.radiusSearch(m_points->at(clusterSeeds[i]), m_epsilon, clusterNeighors, squared_distances);

            if (clusterNeighors.size() >= m_minPoints)
            {
                pcl::Indices::iterator iterNeighors;
                for (iterNeighors = clusterNeighors.begin(); iterNeighors != clusterNeighors.end(); ++iterNeighors)
                {
                    if (m_points->at(*iterNeighors).label == UNCLASSIFIED || m_points->at(*iterNeighors).label == NOISE)
                    {
                        if (m_points->at(*iterNeighors).label == UNCLASSIFIED)
                        {
                            clusterSeeds.push_back(*iterNeighors);
                            n = clusterSeeds.size();
                        }
                        m_points->at(*iterNeighors).label = clusterID;
                    }
                }
            }
        }

        return SUCCESS;
    }
}
