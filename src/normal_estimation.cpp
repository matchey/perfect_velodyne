
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
		Eigen::Vector3d values; // not in descending order
		double curvature, lambda_sum;

#pragma omp parallel for if(flag_omp)\
	private(vectors, values, curvature, lambda_sum) num_threads(2)
		for(size_t i = 0; i < npoints; ++i){ // omp : != to <
			int idx = getIndex(i);
			if(vpc->points[idx].range){
				Eigen::Matrix3d vcov;
				if(getMatCov(i, vcov)){
					Eigen::EigenSolver<Eigen::Matrix3d> es(vcov);
					values = es.eigenvalues().real(); // are not sorted in any particular order.
					vectors = es.eigenvectors().real();

					int min = 0;// int mid = 0; int max = 0;
					for(int i = 1; i < 3; ++i){
						min = values(i) < values(min) ? i : min;
					}
					
					int sign = Eigen::Vector3d(vpc->points[idx].x,
							vpc->points[idx].y,
							vpc->points[idx].z).dot(vectors.col(min)) < 0 ? 1 : -1;
					vectors.col(min) *= sign;
					// values = svd.singularValues();
					lambda_sum = values(0) + values(1) + values(2);
					// lambda_sum = sqrt(values(0)) + sqrt(values(1)) + sqrt(values(2));
					if(lambda_sum){
						curvature = 3.0 * values(min) / lambda_sum;
						vpc->points[idx].normal_x = vectors(0, min);
						vpc->points[idx].normal_y = vectors(1, min);
						vpc->points[idx].normal_z = vectors(2, min);
						vpc->points[idx].curvature = curvature;
						// curvature = 3.0 * sqrt(values(2)) / lambda_sum;
					}
				}
			}
		}
		// showNeighbor(31713);
		// showNeighbor(31744);
	}

	// private
	bool NormalEstimator::getMatCov(const int& ordered, Eigen::Matrix3d& mat)
	{
		const int width = num_horizontal * num_lasers;

		const int ringId = ordered % num_lasers;
		const int vbegin =  ringId < num_vertical ? ordered - ringId : ordered - num_vertical;
		const int vend = ringId < num_lasers - num_vertical ? ordered + num_vertical :
															  ordered + num_lasers - ringId;

		int idx = getIndex(ordered);
		Eigen::Vector3d center(vpc->points[idx].x, vpc->points[idx].y, vpc->points[idx].z);
		Eigen::Vector6d vcov; // xx, xy, xz, yy, yz, zz
		vcov << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

		const double threshold = 1.5*0.0216*pow(center.norm(), 1.8967);

		Eigen::Vector3dArray neighbors;
		neighbors.resize(Eigen::NoChange, (2*num_horizontal+1) * (vend-vbegin+1));
		size_t num_neighbors = 0;

		for(int vert = vbegin; vert != vend; ++vert){
			for(int horiz = vert - width; horiz <= vert + width; horiz += num_lasers){
				idx = getIndex((horiz + npoints) % npoints);
				if(vpc->points[idx].range){
					Eigen::Vector3d p(vpc->points[idx].x, vpc->points[idx].y, vpc->points[idx].z);
					p -= center;
					if(p.norm() < threshold){
						int cnt = 0;
						for(int i = 0; i != 3; ++i){
							for(int j = i; j != 3; ++j){
								vcov(cnt) += p(i) * p(j);
								++cnt;
							}
						}
						neighbors.col(num_neighbors) = p;
						++num_neighbors;
					}
				}
			}
		}

		if(num_neighbors < 5) return false;

		vcov /= num_neighbors;
		// Eigen::Matrix3d inv;
        //
		// if(inverse(vcov, inv)){
		// 	// num_neighbors = removeOutliers(neighbors, inv);
		// }
        //
		// if(num_neighbors < 5) return false;

		mat << vcov(0), vcov(1), vcov(2),
			   vcov(1), vcov(3), vcov(4),
			   vcov(2), vcov(4), vcov(5);
		// mat = inv;
		// neighbors.conservativeResize(Eigen::NoChange, num_neighbors);

		return true;
	}

	int NormalEstimator::removeOutliers(Eigen::Vector3dArray& neighbors, Eigen::Matrix3d& mat)
	{
		const double threshold = 1.5;

		size_t num_neighbors = neighbors.cols();

		Eigen::Matrix3d inv = mat;

		int count_extract = 0;
		Eigen::Vector3d ave(0.0, 0.0, 0.0);
		Eigen::Vector6d prodsum; // xx, xy, xz, yy, yz, zz
		prodsum << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
		Eigen::Vector6d vcov; // xx, xy, xz, yy, yz, zz

		for(size_t i = 0; i != num_neighbors; ++i){
			Eigen::Vector3d p(neighbors.col(i));
			double mdist = p.transpose() * inv * p;
			if(mdist < threshold){
				ave += p;
				int cnt = 0;
				for(int i = 0; i != 3; ++i){
					for(int j = i; j != 3; ++j){
						prodsum(cnt) += p(i) * p(j);
						++cnt;
					}
				}
				++count_extract;
			}
		}

		if(count_extract){
			ave /= count_extract;
			int cnt = 0;
			for(int i = 0; i != 3; ++i){
				for(int j = i; j != 3; ++j){
					vcov(cnt) = prodsum(cnt) / count_extract - ave(i)*ave(j);
					++cnt;
				}
			}

			mat << vcov(0), vcov(1), vcov(2),
				   vcov(1), vcov(3), vcov(4),
				   vcov(2), vcov(4), vcov(5);
		}

		return count_extract;
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

	size_t NormalEstimator::getIndex(const size_t& ordered)
	{
		size_t ringId = ordered % num_lasers; // 2^n の余りだからビットシフトのが早い??

		return  ringId < 16 ? ordered + ringId : ordered + ringId - 31; // 三項演算子は遅い??
	}

} // namespace perfect_velodyne

