
//
// src: normal_estimation.cpp
//
// last update: '18.xx.xx
// author: matchey
//
// memo:
//   Implementation for converting a Velodyne 3D LIDAR PointXYZIR cloud
//   to PointXYZINormal
//

#include <ros/ros.h>
// #include <pcl/point_types.h>
// #include <pcl/point_cloud.h>
#include <pcl/common/pca.h>
#include <Eigen/Core>
// #include "perfect_velodyne/rawdata.h"
#include "perfect_velodyne/normal_estimation.h"

using namespace std;

namespace perfect_velodyne
{

	NormalEstimator::NormalEstimator()
		: num_lasers(32)
	{
		// ros::param::param<double>("min_range", min_range, 130.0);
		// ros::param::param<double>("max_range", max_range, 0.9);
		ros::param::param<int>("VN", num_vertical, 1);
		ros::param::param<int>("HN", num_horizontal, 2);
	}

	void NormalEstimator::normalSetter(perfect_velodyne::VPointCloudNormal::Ptr& pc)
	{
		// pcl::PCA<Point> pca;
		// pcl::PCA<VPointNormal> pca;
		// Eigen::Matrix3f vectors;
		// Eigen::Vector3f values; // in descending order
		// double curvature, lambda_sum;
        //
		// for(size_t i = 0; i != pc->points.size(); ++i){
		// 	int ringId = i % num_lasers;
		// 	if((ringId >= num_vertical) && (ringId < num_lasers - num_vertical)){
		// 		pca.setInputCloud( getNeighbor(pc, i) );
		// 		vectors = pca.getEigenVectors();
		// 		values = pca.getEigenValues();
		// 		lambda_sum = values(0) + values(1) + values(2);
		// 		if(lambda_sum){
		// 			curvature = 3.0 * values(0) / lambda_sum;
		// 			pc->points[i].normal_x = vectors(0, 0);
		// 			pc->points[i].normal_y = vectors(0, 1);
		// 			pc->points[i].normal_z = vectors(0, 2);
		// 			pc->points[i].curvature = curvature;
		// 		}
		// 	}
		// }
	}

	// private
	bool NormalEstimator::pointInRange(const VPointNormal& point)
	{
		double distance = pow(point.x, 2) + pow(point.y, 2) + pow(point.z, 2);
		// 0.4^2 <= distance^2 <= 130.0^2
		return (distance >= 0.16 && distance <= 16900.0);
	}

	size_t NormalEstimator::orderIndex(const size_t& idx)
	{
		size_t ringId = idx % num_lasers;

		return  ringId < 16 ? idx + ringId : idx + ringId - 31;
	}

	VpcNormalPtr NormalEstimator::getNeighbor(const perfect_velodyne::VPointCloudNormal::Ptr& pc,
			                                 const size_t& idx)
	{
		VpcNormalPtr neighbors (new VPointCloudNormal);
		const int width = num_horizontal*num_lasers;
		// const int num_scan = pc->points.size();
		// const int num_scan = pc->points.size() / num_lasers;

		for(size_t vert = idx - num_vertical; vert <= idx + num_vertical; ++vert){
			for(size_t horiz = vert - width;
						horiz <= vert + width; vert+=num_lasers){
				int i = (horiz + pc->points.size()) % pc->points.size();
				neighbors->points.push_back(pc->points[i]);
			}
		}

		return neighbors;
	}

} // namespace perfect_velodyne

