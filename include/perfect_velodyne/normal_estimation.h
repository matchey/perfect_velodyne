
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
	typedef pcl::PointXYZINormal PointNormal;
	typedef pcl::PointCloud<PointNormal> PointCloudNormal;
	typedef pcl::PointCloud<PointNormal>::Ptr PointCloudNormalPtr;

	class NormalEstimator
	{
		int num_vertical; // vertical num for PCA
		int num_horizontal; // horizontal num for PCA

		bool pointInRange(const VPointNormal&);
		size_t orderIndex(const size_t&);
		PointCloudNormal getNeighbor(perfect_velodyne::VPointCloudNormal::Ptr&, const size_t&);

		public:
		NormalEstimator();

		// calc normal with principal component analysis
		void normalSetter(perfect_velodyne::VPointCloudNormal::Ptr&);

	};

} // namespace perfect_velodyne

#endif

