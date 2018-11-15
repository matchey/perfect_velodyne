
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
		
		Eigen::Matrix3f vectors;
		Eigen::Vector3f values;
		float curvature, lambda_sum;

#pragma omp parallel for if(flag_omp)\
	private(vectors, values, curvature, lambda_sum) num_threads(2)
		for(size_t i = 0; i < npoints; ++i){ // omp : != to <
			int idx = getIndex(i);
			bool is_invalid = true;
			if(vpc->points[idx].range){
				PointCloudPtr neighbors(new PointCloud);
				if(getNeighbors(i, neighbors)){
					pcl::PCA<PointT> pca;
					pca.setInputCloud(neighbors);
					values = pca.getEigenValues(); // are sorted in descending order
					vectors = pca.getEigenVectors();

					int sign = Eigen::Vector3f(vpc->points[idx].x,
											   vpc->points[idx].y,
											   vpc->points[idx].z).dot(vectors.col(2)) < 0 ? 1 : -1;
					vectors.col(2) *= sign;
					lambda_sum = values(0) + values(1) + values(2);
					// lambda_sum = sqrt(values(0)) + sqrt(values(1)) + sqrt(values(2));
					if(lambda_sum){
						curvature = 3.0 * values(2) / lambda_sum;
						vpc->points[idx].normal_x = vectors(0, 2);
						vpc->points[idx].normal_y = vectors(1, 2);
						vpc->points[idx].normal_z = vectors(2, 2);
						vpc->points[idx].curvature = curvature;
						// curvature = 3.0 * sqrt(values(2)) / lambda_sum;
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
		const float threshold = 0.3;

		const int width = num_horizontal * num_lasers;
		const int ringId = ordered % num_lasers;
		const int vbegin =  ringId < num_vertical ? ordered - ringId : ordered - num_vertical;
		const int vend = ringId < num_lasers - num_vertical ? ordered + num_vertical + 1:
															  ordered + num_lasers - ringId + 1;

		int idx_center = getIndex(ordered);
		size_t num_neighbors = 0;

		for(int vert = vbegin; vert != vend; ++vert){
			if(vert == ordered){
				for(int horiz = vert - width; horiz <= vert + width; horiz += num_lasers){
					int idx = getIndex((horiz + npoints) % npoints);
					if(vpc->points[idx].range
					&& fabs(vpc->points[idx].range - vpc->points[idx_center].range) < threshold){
						PointT p(vpc->points[idx].x, vpc->points[idx].y, vpc->points[idx].z);
						neighbors->points.push_back(p);
						++num_neighbors;
					}
				}
			}else{
				const int ndistances = 2 * num_horizontal + 1;
				Eigen::MatrixXf distances;
				distances.resize(ndistances, ndistances);
				int m_row = 0;
				for(int horiz = vert - width; horiz <= vert + width; horiz += num_lasers){
					distances.coeffRef(m_row, m_row) = 0.0;
					int idx_row = getIndex((horiz + npoints) % npoints);
					if(vpc->points[idx_row].range){
						int m_col = m_row+1;
						for(int col = horiz+num_lasers; col <= vert + width; col += num_lasers){
							int idx_col = getIndex((col + npoints) % npoints);
							if(vpc->points[idx_col].range){
								float dist = fabs(vpc->points[idx_row].range
												- vpc->points[idx_col].range);
								distances.coeffRef(m_row, m_col) = dist;
								distances.coeffRef(m_col, m_row) = dist;
							}else{
								distances.coeffRef(m_row, m_col) = 120;
								distances.coeffRef(m_col, m_row) = 120;
							}
							++m_col;
						}
					}else{
						int m_col = m_row+1;
						for(int col = horiz+num_lasers; col <= vert + width; col += num_lasers){
							distances.coeffRef(m_row, m_col) = 120;
							distances.coeffRef(m_col, m_row) = 120;
							++m_col;
						}
					}
					++m_row;
				}
				std::vector<float> dist_sum(ndistances, 0);
				for(int i = 0; i < ndistances; ++i){
					dist_sum[i] = distances.row(i).sum();
				}
				float distsum_min = dist_sum[0];
				for(int i = 1; i < ndistances; ++i){
					if(dist_sum[i] < distsum_min){
						distsum_min = dist_sum[i];
					}
				}
				for(int i = 0; i < ndistances; ++i){
					if(dist_sum[i] - distsum_min < threshold){
						int horiz = (vert - width) + num_lasers * i;
						int idx = getIndex((horiz + npoints) % npoints);
						PointT p(vpc->points[idx].x, vpc->points[idx].y, vpc->points[idx].z);
						neighbors->points.push_back(p);
						++num_neighbors;
					}
				}
			}
		}

		// double density = 1.0 * num_neighbors / ((vend - vbegin) * (2*num_horizontal+1));
		// return density < 0.9 ? false : true;
		return num_neighbors < 5 ? false : true;
	}

	int NormalEstimator::removeOutliers(Eigen::Vector3fArray& neighbors, Eigen::Matrix3f& mat)
	{
		return 0;
	}

	bool NormalEstimator::inverse(const Eigen::Vector6f& v, Eigen::Matrix3f& m)
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

		return  ringId < 16 ? ordered + ringId : ordered + ringId - 31; // 三項演算子は遅い??
	}

} // namespace perfect_velodyne

