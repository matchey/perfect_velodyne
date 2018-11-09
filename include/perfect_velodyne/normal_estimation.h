
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
	using Vector6d = Matrix<double, 6, 1>;
	using Vector3dArray = Matrix<double, 3, Eigen::Dynamic>;
}

namespace perfect_velodyne
{
	using VPoint = PointXYZIRNormal;
	using VPointCloud = pcl::PointCloud<VPoint>;
	using VPointCloudPtr = VPointCloud::Ptr;

	class NormalEstimator
	{
		public:
		NormalEstimator();
		void normalSetter(VPointCloudPtr&); // calc normal with singular value decomposition

		private:
		size_t getIndex(const size_t&);
		void getNeighbor(const int&, Eigen::Vector3dArray&);
		void removeOutliers(Eigen::Vector3dArray&, const Eigen::Matrix3d&, const Eigen::Vector3d&);
		bool inverse(const Eigen::Vector6d&, Eigen::Matrix3d&);
		void showNeighbor(const int&);

		int num_vertical; // vertical num for PCA
		int num_horizontal; // horizontal num for PCA
		bool flag_omp; // whether to use OpenMP

		const int num_lasers; // HDL32e -> 32
		VPointCloudPtr vpc; // input pointcloud
		size_t npoints; //number of input points
	};

} // namespace perfect_velodyne

#endif

