#pragma once

#include <algorithm>
#include <execution>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/sample_consensus/sac_model_circle.h>
#include <pcl/sample_consensus/mlesac.h>
#include <pcl/sample_consensus/ransac.h>

#include <pcl/common/pca.h>

#include "circle.hpp"
#include "geometric_math.hpp"
namespace FitWalls{

//TODO: In future versions maybe use pcl::surface::on_nurbs for fitting or another external library?
//TODO: Maybe use values of previous iterations for smoothing (Keep last values(by time?), remove outliers (also consider how far or fast we moved), then use newest inlier). Or smooth by assumption that input is somehow smooth in time (Only if necessary, stabilization of clusterization should already have fixed this)
template<typename PointT>
inline Circle<PointT> fitWall(const typename pcl::PointCloud<PointT>::ConstPtr cloud, const pcl::IndicesConstPtr wall, double distance_to_model_threshold, int max_iterations)
{
	/*
	typename pcl::SampleConsensusModelCircle2D<PointT>::Ptr sample_consensus_model = std::make_shared<typename pcl::SampleConsensusModelCircle2D<PointT>>(cloud, *wall);
	
	//sample_consensus_model->setRadiusLimits(0.1, 0.4);

	//pcl::MaximumLikelihoodSampleConsensus<PointT> sample_consensus(sample_consensus_model, distance_to_model_threshold);
	pcl::RandomSampleConsensus<PointT> sample_consensus(sample_consensus_model, distance_to_model_threshold);
	sample_consensus.setMaxIterations(max_iterations);
	
	//sample_consensus.setProbability(0.01);
	
	
	if(!sample_consensus.computeModel(1)){
		throw std::runtime_error("Could not fit circle to wall");
	}
	//sample_consensus.refineModel(0.01, 1000);
	
	Eigen::VectorXf model_coefficients(3);
	sample_consensus.getModelCoefficients(model_coefficients);

	
	return Circle(PointT(model_coefficients[0], model_coefficients[1], 0.0), model_coefficients[2]);
	*/
	
	(void) distance_to_model_threshold;
	(void) max_iterations;
	
	//Only use x-y values
	//TODO: Maybe do that without another cloud? Somehow project eigenvalues or cloud?
	typename pcl::PointCloud<PointT>::Ptr cloud_flat = std::make_shared<pcl::PointCloud<PointT>>(*cloud, *wall);
	
	std::for_each(std::execution::par_unseq, cloud_flat->begin(), cloud_flat->end(), [](PointT& point){
		point.z = 0.0;
	});
	
	//Compute eigen values of point cloud
	pcl::PCA<PointT> pca;
	pca.setInputCloud(cloud_flat);
	
	//Compute greatest distance and mean distance from mean in direction of greatest eigenvalue
	const PointT mean(pca.getMean()[0], pca.getMean()[1], 0.0);
	const Eigen::Vector3f main_direction = pca.getEigenVectors().col(0);
	
	//Calculate distances from mean
	std::vector<double> distances(cloud_flat->size());
	double mean_distance = 0.0;
	
	//TODO: Make parallel?
	size_t index = 0;
	std::for_each(std::execution::seq, cloud_flat->begin(), cloud_flat->end(), [&mean, &main_direction, &distances, &mean_distance, &index](const PointT& point){
		const double current_distance = std::abs(main_direction.dot(Eigen::Vector3f(point.x - mean.x, point.y - mean.y, 0.0f)));
		distances[index++] = current_distance;
		mean_distance += current_distance / static_cast<double>(distances.size());
	});
	
	//const double max_distance = *(std::max_element(std::execution::par_unseq, distances.begin(), distances.end()));
	
	//Create circle that fits many points of the cloud
	Circle main_circle(PointT(pca.getMean()[0], pca.getMean()[1], 0.0), mean_distance);
	
	//TODO: Move the circle around till most points fit its border (move in direction of least variance)? Or just move as long till the circle only touches the cloud?
	//Does not work cause we don't know in which direction to move
	//const Eigen::Vector3f new_center = Eigen::Vector3f(main_circle.getCenter().x, main_circle.getCenter().y, 0.0f) - (pca.getEigenVectors().col(1) * static_cast<float>(mean_distance));
	//main_circle.getCenter().x = new_center[0];
	//main_circle.getCenter().y = new_center[1];
	
	return main_circle;
	
	/*
	(void) distance_to_model_threshold;
	const size_t cloud_size = wall->size();

	double sum_x = 0;
    double sum_y = 0;
    for (const size_t index : *wall)
    {
		const PointT& point = cloud->at(index);
        sum_x += point.x;
        sum_y += point.y;
    }
    PointT mean(sum_x / cloud_size, sum_y / cloud_size, 0.0f);

	unsigned int mod = 0;
	// compute moments
	double Mxx, Mxy, Mxz, Myy, Myz, Mzz;
	for (const size_t index : *wall)
	{
		const PointT& point = cloud->at(index);
		const double Xi = point.x - mean.x;
		const double Yi = (point.y + mod % 9 * 0.001) - mean.y;
		const double Zi = Xi * Xi + Yi * Yi;

		Mxy += Xi * Yi;
		Mxx += Xi * Xi;
		Myy += Yi * Yi;
		Mxz += Xi * Zi;
		Myz += Yi * Zi;
		Mzz += Zi * Zi;

		mod++;
	}
	Mxx /= cloud_size;
	Myy /= cloud_size;
	Mxy /= cloud_size;
	Mxz /= cloud_size;
	Myz /= cloud_size;
	Mzz /= cloud_size;

	// computing the coefficients of characteristic polynomial
	const double Mz = Mxx + Myy;
	const double Cov_xy = Mxx * Myy - Mxy * Mxy;
	const double Var_z = Mzz - Mz * Mz;

	const double A2 = 4.0 * Cov_xy - 3.0 * Mz * Mz - Mzz;
	const double A1 = Var_z * Mz + 4.0 * Cov_xy * Mz - Mxz * Mxz - Myz * Myz;
	const double A0 = Mxz * (Mxz * Myy - Myz * Mxy) + Myz * (Myz * Mxx - Mxz * Mxy) - Var_z * Cov_xy;
	const double A22 = A2 + A2;

	// finding the root of the characteristic polynomial
	double y = A0;
	double x = 0.0;
	for (int i = 0; i < max_iterations; i++)
	{
		const double Dy = A1 + x * (A22 + 16.0 * x * x);
		const double x_new = x - y / Dy;
		if (x_new == x || !std::isfinite(x_new)){
			break;
		}
		const double y_new = A0 + x_new * (A1 + x_new * (A2 + 4.0 * x_new * x_new));
		if (std::abs(y_new) >= std::abs(y)){
			break;
		}
		x = x_new;
		y = y_new;
	}
	const double det = x * x - x * Mz + Cov_xy;

	PointT center;
	center.x = (Mxz * (Myy - x) - Myz * Mxy) / det / 2.0;
	center.y = (Myz * (Mxx - x) - Mxz * Mxy) / det / 2.0;

	double radius = std::sqrt(std::abs(center.x * center.x + center.y * center.y + Mz));
	center.x = center.x + mean.x;
	center.y = center.y + mean.y;

	return Circle(center, radius);
	*/
}

}