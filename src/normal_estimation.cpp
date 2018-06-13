
//
// src: normal_estimation.cpp
//
// last update: '18.xx.xx
// author: matchey
//
// memo:
//

#include <ros/ros.h>
#include "perfect_velodyne/normal_estimation.h"

using namespace std;

NormalEstimator::NormalEstimator()
{
	// ros::param::param<double>("min_range", min_range, 130.0);
	// ros::param::param<double>("max_range", max_range, 0.9);
	ros::param::param<int>("VN", num_vertical, 1);
	ros::param::param<int>("HN", num_horizontal, 2);
}

// private

// bool NormalEstimator::pointInRange(float range)
// {
// 	return (range >= min_range
// 	        && range <= max_range);
// }

