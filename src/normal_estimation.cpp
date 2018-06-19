
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

	void NormalEstimator::normalSetter(perfect_velodyne::VPointCloudNormal::Ptr& vpc)
	{
		PointCloudPtr pc (new PointCloud);
		PointCloudPtr neighbors (new PointCloud);

		vpcloud2pcl(vpc, pc);
		
		pcl::PCA<Point> pca;
		Eigen::Matrix3f vectors;
		Eigen::Vector3f values; // in descending order
		double curvature, lambda_sum;

		for(size_t i = 0; i != pc->points.size(); ++i){
			int ordered = orderIndex(i);
			int ringId = ordered % num_lasers;
			if((ringId >= num_vertical) && (ringId < num_lasers - num_vertical)){
				neighbors = getNeighbor(pc, i);
				if(neighbors->points.empty()){
					cerr << "i : " << i << ", ring : " << ringId << endl;
					break;
				}
				pca.setInputCloud(neighbors);
				vectors = pca.getEigenVectors();
				values = pca.getEigenValues();
				lambda_sum = values(0) + values(1) + values(2);
				if(lambda_sum){
					curvature = 3.0 * values(2) / lambda_sum;
					vpc->points[i].normal_x = vectors(0, 0);
					vpc->points[i].normal_y = vectors(0, 1);
					vpc->points[i].normal_z = vectors(0, 2);
					vpc->points[i].curvature = curvature;
				}
			}
			if(i == 16015){
				pcl2vpcloud(neighbors, vpc);
				break;
			}
		}
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

	void NormalEstimator::vpoint2pcl(const VPointNormal& vp, Point& p)
	{
		p.x = vp.x;
		p.y = vp.y;
		p.z = vp.z;
	}

	void NormalEstimator::vpcloud2pcl(const VpcNormalPtr& vpc, PointCloudPtr& pc)
	{
		Point p;

		if(pc->points.empty()){
			for(auto it = vpc->points.begin(); it != vpc->points.end(); ++it){
				vpoint2pcl(*it, p);
				pc->points.push_back(p);
			}
		}else{
			PointCloudPtr pcXYZ (new PointCloud);

			for(auto it = vpc->points.begin(); it != vpc->points.end(); ++it){
				vpoint2pcl(*it, p);
				pcXYZ->points.push_back(p);
			}

			pc = pcXYZ;
		}
	}

	void NormalEstimator::pcl2vpoint(const Point& p, VPointNormal& vp)
	{
		vp.x = p.x;
		vp.y = p.y;
		vp.z = p.z;
	}

	void NormalEstimator::pcl2vpcloud(const PointCloudPtr& pc, VpcNormalPtr& vpc)
	{
		VPointNormal vp;

		if(vpc->points.empty()){
			for(auto it = pc->points.begin(); it != pc->points.end(); ++it){
				pcl2vpoint(*it, vp);
				vpc->points.push_back(vp);
			}
		}else{
			VpcNormalPtr v_pc (new VPointCloudNormal);

			for(auto it = pc->points.begin(); it != pc->points.end(); ++it){
				pcl2vpoint(*it, vp);
				v_pc->points.push_back(vp);
			}

			vpc = v_pc;
		}
	}

	PointCloudPtr NormalEstimator::getNeighbor(const PointCloudPtr& pc, const int& idx)
	{
		PointCloudPtr neighbors (new PointCloud);
		const int width = num_horizontal * num_lasers;

		for(int vert = idx - num_vertical; vert <= idx + num_vertical; ++vert){
			// cerr << "idx : " << idx << endl;
			// cerr << "vert : " << vert << endl;
			// cerr << "num_vertical : " << num_vertical << endl;
			// cerr << "num_horizontal : " << num_horizontal << endl;
			// cerr << "num_lasers : " << num_lasers << endl;
			// cerr << "width : " << width << endl;
			for(int horiz = vert - width; horiz <= vert + width; horiz += num_lasers){
				int i = (orderIndex(horiz) + pc->points.size()) % pc->points.size();
				neighbors->points.push_back(pc->points[i]);
				// cerr << "i : " << i << endl;
				// break;
			}
			// break;
		}
		// cerr << "HOGEHOGE" << endl;
		// if(neighbors->points.empty()){
		// 	Point p;
		// 	p.x = 0.0;
		// 	p.y = 0.0;
		// 	p.z = 0.0;
		// 	neighbors->points.push_back(p);
		// 	p.x = 1.0;
		// 	p.y = 1.0;
		// 	p.z = 0.0;
		// 	neighbors->points.push_back(p);
		// 	p.x = 1.0;
		// 	p.y = 1.0;
		// 	p.z = 0.0;
		// 	neighbors->points.push_back(p);
		// 	p.x = 0.0;
		// 	p.y = 1.0;
		// 	p.z = 1.0;
		// 	neighbors->points.push_back(p);
		// }

		return neighbors;
	}

} // namespace perfect_velodyne

