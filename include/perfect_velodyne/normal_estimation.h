
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

#include <Eigen/Core>
#include "perfect_velodyne/point_types.h"

namespace Eigen{
	using Vector6f = Matrix<float, 6, 1>;
	using Vector3fArray = Matrix<float, 3, Dynamic>;
}

namespace perfect_velodyne
{
	using VPoint = PointXYZIRNormal;
	using VPointCloud = pcl::PointCloud<VPoint>;
	using VPointCloudPtr = VPointCloud::Ptr;

	using PointT = pcl::PointXYZ;
	using PointCloud = pcl::PointCloud<PointT>;
	using PointCloudPtr = PointCloud::Ptr;

	class NormalEstimator
	{
		public:
		NormalEstimator();
		void normalSetter(VPointCloudPtr&); // calc normal with singular value decomposition

		private:
		bool getNeighbors(const int&, PointCloudPtr&);
		int removeOutliers(Eigen::Vector3fArray&, Eigen::Matrix3f&);
		bool inverse(const Eigen::Vector6f&, Eigen::Matrix3f&);
		void showNeighbor(const int&);
		size_t getIndex(const size_t&);

		int num_vertical; // vertical num for PCA
		int num_horizontal; // horizontal num for PCA
		bool flag_omp; // whether to use OpenMP

		const int num_lasers; // HDL32e -> 32
		VPointCloudPtr vpc; // input pointcloud
		size_t npoints; //number of input points
	};

} // namespace perfect_velodyne

#endif

