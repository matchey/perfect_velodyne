
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
#include <pcl/common/pca.h>
#include <omp.h>
#include "perfect_velodyne/normal_estimation.h"

using std::cerr;
using std::endl;

namespace perfect_velodyne
{

	NormalEstimator::NormalEstimator()
		: num_lasers(32), vpc(new VPointCloud)
	{
		ros::param::param<int>("/perfect_velodyne/VN", num_vertical, 1);
		ros::param::param<int>("/perfect_velodyne/HN", num_horizontal, 2);
		ros::param::param<bool>("/perfect_velodyne/OpenMP", flag_omp, false);
	}

	// void NormalEstimator::normalSetter(perfect_velodyne::VPointCloud::Ptr& vpointcloud)
	void NormalEstimator::normalSetter(VPointCloudPtr& vpointcloud)
	{
		vpc = vpointcloud;

		npoints = vpc->points.size();
		
#pragma omp parallel for if(flag_omp) num_threads(2)
		for(size_t i = 0; i < npoints; ++i){ // omp : != to <
			int idx = getIndex(i);
			bool is_invalid = true;
			if(vpc->points[idx].range){
				PointCloudPtr neighbors(new PointCloud);
				if(getNeighbors(i, neighbors)){
					Eigen::Matrix3f vectors;
					Eigen::Vector3f values;
					pcl::PCA<PointT> pca;
					pca.setInputCloud(neighbors);
					values = pca.getEigenValues(); // are sorted in descending order
					vectors = pca.getEigenVectors();

					if(0 < Eigen::Vector3f(vpc->points[idx].x,
										   vpc->points[idx].y,
										   vpc->points[idx].z).dot(vectors.col(2))){
						vectors.col(2) *= -1;
					}
					float lambda_sum = values(0) + values(1) + values(2);
					// lambda_sum = sqrt(values(0)) + sqrt(values(1)) + sqrt(values(2));
					if(lambda_sum){
						float curvature = 3.0 * values(2) / lambda_sum;
						// curvature = 3.0 * sqrt(values(2)) / lambda_sum;
						vpc->points[idx].normal_x = vectors.coeffRef(0, 2);
						vpc->points[idx].normal_y = vectors.coeffRef(1, 2);
						vpc->points[idx].normal_z = vectors.coeffRef(2, 2);
						vpc->points[idx].curvature = curvature;
						is_invalid = false;
					}
				}
			}
			if(is_invalid){
				vpc->points[idx].x = 0.0;
				vpc->points[idx].y = 0.0;
				vpc->points[idx].z = 0.0;
				vpc->points[idx].normal_x = 0.0;
				vpc->points[idx].normal_y = 0.0;
				vpc->points[idx].normal_z = -1.0;
				vpc->points[idx].curvature = 0.0;
			}
		}
		// showNeighbor(31713);
		// showNeighbor(31744);
	}

	// private
	bool NormalEstimator::getNeighbors(const int& ordered, PointCloudPtr& neighbors)
	{
		const float threshold = 0.9f;
		// const float v_height = 1.3f; // Velodyne height
		// const int ring_level = 23; // horizontal laser of hdl32e (0~32)

		// const float theta_min = -0.535292f; // bottom laser's horizontal angle
		// const float theta = -0.0232735861f; // theta_min / ring_level;

		const int width = num_horizontal * num_lasers;
		const int ringId = ordered % num_lasers;
		const int vbegin =  ringId < num_vertical ? ordered - ringId : ordered - num_vertical;
		const int vend = ringId < num_lasers - num_vertical ? ordered + num_vertical + 1:
															  ordered + num_lasers - ringId + 1;

		const int idx_center = getIndex(ordered);

		const float dist_pred = vpc->points[idx_center].range;

		// const float angle_center = theta * (ring_level - ringId);

		// bool is_onWall = true;
		// if(ringId < ring_level){
		// 	is_onWall = vpc->points[idx_center].range * sin(angle_center) / v_height < threshold;
		// }

		size_t num_neighbors = 0;

		for(int vert = vbegin; vert != vend; ++vert){
			// float angle = theta * (ring_level - (vert % num_lasers));
			// float dist_pred;
			// if(is_onWall){
			// 	dist_pred = vpc->points[idx_center].range * cos(angle_center) / cos(angle);
			// }else{
			// 	dist_pred = vpc->points[idx_center].range * sin(angle_center) / sin(angle);
			// }
			for(int horiz = vert - width; horiz <= vert + width; horiz += num_lasers){
				int idx = getIndex((horiz + npoints) % npoints);
				if(threshold < vpc->points[idx].range / dist_pred){
					PointT p(vpc->points[idx].x, vpc->points[idx].y, vpc->points[idx].z);
					neighbors->points.push_back(p);
					++num_neighbors;
				}
			}
		}

		return 4 < num_neighbors;
	}

	void NormalEstimator::showNeighbor(const int& ordered)
	{
		const int width = num_horizontal * num_lasers;
		const int ringId = ordered % num_lasers;
		const int vbegin =  ringId < num_vertical ? ordered - ringId : ordered - num_vertical;
		const int vend = ringId < num_lasers - num_vertical ? ordered + num_vertical +1 :
															  ordered + num_lasers - ringId + 1;

		for(int vert = vbegin; vert != vend; ++vert){
			for(int horiz = vert - width; horiz <= vert + width; horiz += num_lasers){
				int idx = getIndex((horiz + npoints) % npoints);
				vpc->points[idx].normal_z = 10.0;
			}
		}
	}

	size_t NormalEstimator::getIndex(const size_t& ordered)
	{
		size_t ringId = ordered % num_lasers; // 2^n の余りだからビットシフトのが早い??

		return  ringId < 16 ? ordered + ringId : ordered + ringId - num_lasers + 1;
	}

} // namespace perfect_velodyne

