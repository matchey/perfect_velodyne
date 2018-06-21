
//
// include: normal_estimation.h
//
// last update: '18.xx.xx
// author: matchey
//
// memo:
//

#ifndef NORMAL_ESTIMATION_H
#define NORMAL_ESTIMATION_H

#include "perfect_velodyne/point_types.h"

namespace perfect_velodyne
{
	typedef PointXYZIRNormal VPointNormal;
	typedef pcl::PointCloud<VPointNormal> VPointCloudNormal;
	typedef VPointCloudNormal::Ptr VpcNormalPtr;
	// typedef pcl::PointXYZINormal PointNormal;
	// typedef pcl::PointCloud<PointNormal> PointCloudNormal;
	// typedef pcl::PointCloud<PointNormal>::Ptr PointCloudNormalPtr;
	typedef pcl::PointXYZ Point;
	typedef pcl::PointCloud<Point> PointCloud;
	typedef PointCloud::Ptr PointCloudPtr;

	class NormalEstimator
	{
		int num_vertical; // vertical num for PCA
		int num_horizontal; // horizontal num for PCA
		const int num_lasers; // HDL32e -> 32
		bool flag_omp;
		bool flag_pub_org;

		bool pointInRange(const VPointNormal&);
		size_t orderIndex(const size_t&);
		void vpoint2pcl(const VPointNormal&, Point&);
		void vpcloud2pcl(const VpcNormalPtr&, PointCloudPtr&);
		// void pcl2vpoint(const Point&, VPointNormal&);
		// void pcl2vpcloud(const PointCloudPtr&, VpcNormalPtr&);
		PointCloudPtr getNeighbor(const PointCloudPtr&, const int&);
		void showNeighbor(VpcNormalPtr&, const int&);

		public:
		NormalEstimator();

		// calc normal with principal component analysis
		void normalSetter(perfect_velodyne::VPointCloudNormal::Ptr&);

	};

} // namespace perfect_velodyne

#endif

