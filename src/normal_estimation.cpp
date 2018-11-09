
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
		: num_lasers(32), vpc(new VPointCloudNormal)
	{
		// ros::param::param<double>("min_range", min_range, 130.0);
		// ros::param::param<double>("max_range", max_range, 0.9);
		ros::param::param<int>("/perfect_velodyne/VN", num_vertical, 1);
		ros::param::param<int>("/perfect_velodyne/HN", num_horizontal, 2);
		ros::param::param<bool>("/perfect_velodyne/OpenMP", flag_omp, false);
	}

	void NormalEstimator::normalSetter(perfect_velodyne::VPointCloudNormal::Ptr& vpointcloud)
	{
		// PointCloudPtr pc (new PointCloud);
		PointCloudPtr neighbors (new PointCloud);

		// vpcloud2pcl(vpc, pc);
		vpc = vpointcloud;

		npoints = vpc->points.size();
		
		pcl::PCA<Point> pca;
		Eigen::Matrix3f vectors;
		Eigen::Vector3f values; // in descending order
		double curvature, lambda_sum;

#pragma omp parallel for if(flag_omp)\
	private(pca, vectors, values, curvature, lambda_sum, neighbors) num_threads(2)
		for(size_t i = 0; i < npoints; ++i){ // omp : != to <
			int idx = getIndex(i);
			if(vpc->points[idx].range){
				neighbors = getNeighbor(vpc, i);
				if(neighbors->points.size() < 3){
					// cerr << "[PCA] number of points < 3" << endl;
					// vpc->points[idx].normal_z = 10;
					continue;
				}
				pca.setInputCloud(neighbors); // SVDのほうが安定するらしい
				vectors = pca.getEigenVectors();
				int sign = Eigen::Vector3f(vpc->points[idx].x,
						vpc->points[idx].y,
						vpc->points[idx].z).dot(vectors.col(2)) < 0 ? 1 : -1;
				vectors.col(2) *= sign;
				values = pca.getEigenValues();
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
		showNeighbor(vpc, 31713); // ng
		// showNeighbor(vpc, 31744); // ok
	}

	// private
	bool NormalEstimator::pointInRange(const VPointNormal& point)
	{
		double distance = pow(point.x, 2) + pow(point.y, 2) + pow(point.z, 2);
		// 0.4^2 <= distance^2 <= 130.0^2
		return (distance >= 0.16 && distance <= 16900.0);
	}

	size_t NormalEstimator::getIndex(const size_t& ordered)
	{
		size_t ringId = ordered % num_lasers; // 2^n の余りだからビットシフトのが早い??

		return  ringId < 16 ? ordered + ringId : ordered + ringId - 31; // 三項演算子は遅い??
	}

	// void NormalEstimator::vpoint2pcl(const VPointNormal& vp, Point& p)
	// {
	// 	p.x = vp.x;
	// 	p.y = vp.y;
	// 	p.z = vp.z;
	// }
    //
	// void NormalEstimator::vpcloud2pcl(const VpcNormalPtr& vpc, PointCloudPtr& pc)
	// {
	// 	Point p;
    //
	// 	if(!pc->points.empty()){
	// 		pc->points.clear();
	// 	}
	// 	for(auto it = vpc->points.begin(); it != vpc->points.end(); ++it){
	// 		vpoint2pcl(*it, p);
	// 		pc->points.push_back(p);
	// 	}
	// }

	// void NormalEstimator::pcl2vpoint(const Point& p, VPointNormal& vp)
	// {
	// 	vp.x = p.x;
	// 	vp.y = p.y;
	// 	vp.z = p.z;
	// }
    //
	// void NormalEstimator::pcl2vpcloud(const PointCloudPtr& pc, VpcNormalPtr& vpc)
	// {
	// 	VPointNormal vp;
    //
	// 	if(!vpc->points.empty()){
	// 		vpc->points.clear();
	// 	}
	// 	for(auto it = pc->points.begin(); it != pc->points.end(); ++it){
	// 		pcl2vpoint(*it, vp);
	// 		vpc->points.push_back(vp);
	// 	}
	// }

	PointCloudPtr NormalEstimator::getNeighbor(const VpcNormalPtr& pc, const int& ordered)
	// VpcNormalPtr NormalEstimator::getNeighbor(const VpcNormalPtr& pc, const int& ordered)
	{
		PointCloudPtr neighbors(new PointCloud); // openMP thread内でそれぞれ必要

		const int width = num_horizontal * num_lasers;

		const int ringId = ordered % num_lasers;
		const int vbegin =  ringId < num_vertical ? ordered - ringId : ordered - num_vertical;
		const int vend = ringId < num_lasers - num_vertical ? ordered + num_vertical :
															  ordered + num_lasers - ringId;

		int idx = getIndex(ordered);
		Eigen::Vector3d center(pc->points[idx].x, pc->points[idx].y, pc->points[idx].z);
		// Eigen::Vector3d sum(0.0, 0.0, 0.0); // sum_x, sum_y, sum_z
		Eigen::Vector6d prodsum; // xx, xy, xz, yy, yz, zz
		prodsum << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
		Eigen::Vector6d vcov; // xx, xy, xz, yy, yz, zz

		for(int vert = vbegin; vert != vend; ++vert){
			for(int horiz = vert - width; horiz <= vert + width; horiz += num_lasers){
				idx = getIndex((horiz + npoints) % npoints);
				if(pc->points[idx].range){
					Eigen::Vector3d p(pc->points[idx].x, pc->points[idx].y, pc->points[idx].z);
					// sum += p;
					int cnt = 0;
					for(int i = 0; i != 3; ++i){
						for(int j = i; j != 3; ++j){
							prodsum(cnt) += p(i) * p(j);
							++cnt;
						}
					}
					Point point;
					point.x = pc->points[idx].x;
					point.y = pc->points[idx].y;
					point.z = pc->points[idx].z;
					neighbors->points.push_back(point);
				}
			}
		}

		size_t num_neighbors = neighbors->points.size();

		if(num_neighbors){
			// ave = sum / num_neighbors;

			int cnt = 0;
			for(int i = 0; i != 3; ++i){
				for(int j = i; j != 3; ++j){
					vcov(cnt) = prodsum(cnt) / num_neighbors - center(i) * center(j);
					++cnt;
				}
			}

			Eigen::Matrix3d inv;
			// inv << vcov(0), vcov(1), vcov(2),
			//      vcov(1), vcov(3), vcov(4),
			//      vcov(2), vcov(4), vcov(5);
			if(inverse(vcov, inv)){
				removeOutliers(neighbors, inv, center);
			}
		}

		return neighbors;
	}

	// void NormalEstimator::removeOutliers(PointCloudPtr& pc, const Eigen::Matrix3d& inv,
	void NormalEstimator::removeOutliers(PointCloudPtr& pc, const Eigen::Matrix3d& inv,
															const Eigen::Vector3d& ave)
	{
		const double th = 10.0;

		pcl::PointIndices::Ptr indices(new pcl::PointIndices());

		size_t num_neighbors = pc->points.size();

		for(size_t i = 0; i != num_neighbors; ++i){
			Eigen::Vector3d p(pc->points[i].x, pc->points[i].y, pc->points[i].z);
			// double mdist = (p - ave).transpose() * vcov.householderQr().solve(p - ave);
			double mdist = (p - ave).norm();
			// double mdist = (p - ave).transpose() * inv * (p - ave);
			if(th < mdist){
				// pc->points.erase(it++);
				// cerr << "dist " << mdist << endl;
				indices->indices.push_back(i);
			}
		}

		// pcl::ExtractIndices<Point> extract;
		// extract.setInputCloud(pc);
		// extract.setIndices(indices);
		// extract.setNegative(true); // true: indicesを除去
		// extract.filter(*pc);
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

	void NormalEstimator::showNeighbor(VpcNormalPtr& pc, const int& ordered)
	{
		const int width = num_horizontal * num_lasers;
		const int ringId = ordered % num_lasers;
		const int vbegin =  ringId < num_vertical ? ordered - ringId : ordered - num_vertical;
		const int vend = ringId < num_lasers - num_vertical ? ordered + num_vertical :
															  ordered + num_lasers - ringId;

		for(int vert = vbegin; vert != vend; ++vert){
			for(int horiz = vert - width; horiz <= vert + width; horiz += num_lasers){
				int idx = getIndex((horiz + npoints) % npoints);
				pc->points[idx].normal_z = 10.0;
			}
		}
	}

	void NormalEstimator::showPoints(const pcl::PointIndices::Ptr& indices)
	{
		for(auto ordered = indices->indices.begin(); ordered != indices->indices.end(); ++ordered){
			// vpc->points[ordered].normal_z = 10.0;
		}
	}
} // namespace perfect_velodyne

