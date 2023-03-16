#include "dbscan.hpp"

#include <algorithm>
#include <execution>

int DBSCAN::run()
{
    uint32_t clusterID = 1;
    for (size_t i = 0; i < m_points->size(); i++)
    {
        if (m_points->at(i).label == UNCLASSIFIED)
        {
            if (expandCluster(m_points->at(i), clusterID) != FAILURE)
            {
                clusterID += 1;
            }
        }
    }

    return 0;
}

int DBSCAN::expandCluster(Point_& point, uint32_t clusterID)
{
    pcl::Indices clusterSeeds;
	std::vector<float> squared_distances;
	m_octree.radiusSearch(point, m_epsilon, clusterSeeds, squared_distances);

    if (clusterSeeds.size() < m_minPoints)
    {
        point.label = NOISE;
        return FAILURE;
    }
    else
    {
		std::for_each(std::execution::par, clusterSeeds.begin(), clusterSeeds.end(), [this, &clusterID](const uint32_t index){
			m_points->at(index).label = clusterID;
		});

		//Add neighbours of neighbours
        for (std::vector<uint32_t>::size_type i = 0, n = clusterSeeds.size(); i < n; ++i)
        {
            pcl::Indices clusterNeighors;
			m_octree.radiusSearch(m_points->at(clusterSeeds[i]), m_epsilon, clusterNeighors, squared_distances);

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
