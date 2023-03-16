#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/sample_consensus/sac_model_circle.h>
#include <pcl/sample_consensus/mlesac.h>

#include "circle.hpp"
namespace FitWalls{

template<typename PointT>
inline Circle<PointT> fitWall(const typename pcl::PointCloud<PointT>::ConstPtr cloud, const pcl::IndicesConstPtr wall, double distance_to_model_threshold, int max_iterations)
{
	typename pcl::SampleConsensusModelCircle2D<PointT>::Ptr sample_consensus_model = std::make_shared<typename pcl::SampleConsensusModelCircle2D<PointT>>(cloud);
	sample_consensus_model->setIndices(*wall);

	pcl::MaximumLikelihoodSampleConsensus<PointT> sample_consensus(sample_consensus_model, distance_to_model_threshold);
	
	sample_consensus.setMaxIterations(max_iterations);
	
	if(!sample_consensus.computeModel()){
		throw std::runtime_error("Could not fit circle to wall");
	}
	Eigen::VectorXf model_coefficients(3);
	sample_consensus.getModelCoefficients(model_coefficients);
	
	return Circle(PointT(model_coefficients[0], model_coefficients[1], 0.0), model_coefficients[2]);
}

}