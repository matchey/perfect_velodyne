
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
// #include "perfect_velodyne/rawdata.h"
#include "perfect_velodyne/normal_estimation.h"

using namespace std;

namespace perfect_velodyne
{

	NormalEstimator::NormalEstimator()
	{
		// ros::param::param<double>("min_range", min_range, 130.0);
		// ros::param::param<double>("max_range", max_range, 0.9);
		ros::param::param<int>("VN", num_vertical, 1);
		ros::param::param<int>("HN", num_horizontal, 2);
	}

	void NormalEstimator::normalSetter(perfect_velodyne::VPointCloudNormal::Ptr& pc)
	{
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
		size_t ringId = idx % 32;

		return  ringId < 16 ? idx + ringId : idx + ringId - 31;
	}

	PointCloudNormal getNeighbor(perfect_velodyne::VPointCloudNormal::Ptr& pc, const size_t& idx)
	{
		PointCloudNormalPtr neighbors (new PointCloudNormal);

		return *neighbors;
	}

} // namespace perfect_velodyne

