
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
#include <pcl/filters/extract_indices.h>
#include <omp.h>
// #include "perfect_velodyne/rawdata.h"
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
		
		Eigen::Matrix3d vectors;
		Eigen::Vector3d values; // in descending order
		double curvature, lambda_sum;

#pragma omp parallel for if(flag_omp)\
	private(vectors, values, curvature, lambda_sum) num_threads(2)
		for(size_t i = 0; i < npoints; ++i){ // omp : != to <
			int idx = getIndex(i);
			if(vpc->points[idx].range){
				Eigen::Vector3dArray neighbors;
				getNeighbor(i, neighbors);
				if(neighbors.cols() < 3){
					// cerr << "[PCA] number of points < 3" << endl;
					// vpc->points[idx].normal_z = 10;
					continue;
				}
				Eigen::JacobiSVD<Eigen::Vector3dArray> svd(neighbors,
														Eigen::ComputeThinU | Eigen::ComputeThinV);
				vectors = svd.matrixU();
				int sign = Eigen::Vector3d(vpc->points[idx].x,
										   vpc->points[idx].y,
										   vpc->points[idx].z).dot(vectors.col(2)) < 0 ? 1 : -1;
				vectors.col(2) *= sign;
				values = svd.singularValues();
				lambda_sum = values(0) + values(1) + values(2);
				// lambda_sum = sqrt(values(0)) + sqrt(values(1)) + sqrt(values(2));
				if(lambda_sum){
					curvature = 3.0 * values(2) / lambda_sum;
					vpc->points[idx].normal_x = vectors(0, 2);
					vpc->points[idx].normal_y = vectors(1, 2);
					vpc->points[idx].normal_z = vectors(2, 2);
					vpc->points[idx].curvature = curvature;
					// curvature = 3.0 * sqrt(values(2)) / lambda_sum;
				}
			}
		}
		showNeighbor(31713); // ng
		// showNeighbor(31744); // ok
	}

	// private
	size_t NormalEstimator::getIndex(const size_t& ordered)
	{
		size_t ringId = ordered % num_lasers; // 2^n の余りだからビットシフトのが早い??

		return  ringId < 16 ? ordered + ringId : ordered + ringId - 31; // 三項演算子は遅い??
	}

	void NormalEstimator::getNeighbor(const int& ordered, Eigen::Vector3dArray& neighbors)
	{
		const int width = num_horizontal * num_lasers;

		const int ringId = ordered % num_lasers;
		const int vbegin =  ringId < num_vertical ? ordered - ringId : ordered - num_vertical;
		const int vend = ringId < num_lasers - num_vertical ? ordered + num_vertical :
															  ordered + num_lasers - ringId;
		neighbors.resize(Eigen::NoChange, (2*num_horizontal+1) * (vend-vbegin+1));

		int idx = getIndex(ordered);
		Eigen::Vector3d center(vpc->points[idx].x, vpc->points[idx].y, vpc->points[idx].z);
		Eigen::Vector6d vcov; // xx, xy, xz, yy, yz, zz

		size_t num_neighbors = 0;

		for(int vert = vbegin; vert != vend; ++vert){
			for(int horiz = vert - width; horiz <= vert + width; horiz += num_lasers){
				idx = getIndex((horiz + npoints) % npoints);
				if(vpc->points[idx].range){
					Eigen::Vector3d p(vpc->points[idx].x, vpc->points[idx].y, vpc->points[idx].z);
					neighbors.col(num_neighbors) = p - center;
					int cnt = 0;
					for(int i = 0; i != 3; ++i){
						for(int j = i; j != 3; ++j){
							++cnt;
						}
					}
					++num_neighbors;
				}
			}
		}

		neighbors.conservativeResize(Eigen::NoChange, num_neighbors);

		if(num_neighbors){
			Eigen::Matrix3d inv;
			if(inverse(vcov, inv)){
				// removeOutliers(neighbors, inv, center);
			}
		}
	}

	void NormalEstimator::removeOutliers(Eigen::Vector3dArray& neighbors,
											const Eigen::Matrix3d& inv, const Eigen::Vector3d& ave)
	{
		// const double th = 10.0;

		size_t num_neighbors = neighbors.cols();

		for(size_t i = 0; i != num_neighbors; ++i){
			// Eigen::Vector3d p(pc->points[i].x, pc->points[i].y, pc->points[i].z);
			// double mdist = (p - ave).transpose() * vcov.householderQr().solve(p - ave);
			// double mdist = (p - ave).norm();
			// double mdist = (p - ave).transpose() * inv * (p - ave);
			// if(th < mdist){
			// 	// cerr << "dist " << mdist << endl;
			// }
		}
	}

	bool NormalEstimator::inverse(const Eigen::Vector6d& v, Eigen::Matrix3d& m)
	{
		double det = v(0)*v(3)*v(5) + 2*v(1)*v(2)*v(4)
					- v(2)*v(2)*v(3) - v(1)*v(1)*v(5) - v(0)*v(4)*v(4);

		if(det == 0){
			// cerr << "det = 0" << endl;
			return false;
		}

		double b = v(2)*v(4) - v(1)*v(5); // [a b c] 
		double c = v(1)*v(4) - v(2)*v(3); // [b d e]
		double e = v(1)*v(2) - v(0)*v(4); // [c e f]

		m << v(3)*v(5) - v(4)*v(4), b, c,
		  	 b, v(0)*v(5) - v(2)*v(2), e,
			 c, e, v(0)*v(3) - v(1)*v(1);

		m /= det;

		return true;
	}

	void NormalEstimator::showNeighbor(const int& ordered)
	{
		const int width = num_horizontal * num_lasers;
		const int ringId = ordered % num_lasers;
		const int vbegin =  ringId < num_vertical ? ordered - ringId : ordered - num_vertical;
		const int vend = ringId < num_lasers - num_vertical ? ordered + num_vertical :
															  ordered + num_lasers - ringId;

		for(int vert = vbegin; vert != vend; ++vert){
			for(int horiz = vert - width; horiz <= vert + width; horiz += num_lasers){
				int idx = getIndex((horiz + npoints) % npoints);
				vpc->points[idx].normal_z = 10.0;
			}
		}
	}
} // namespace perfect_velodyne

