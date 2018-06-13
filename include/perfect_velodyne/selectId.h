
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

#include <velodyne_pointcloud/rawdata.h>
#include "perfect_velodyne/point_types.h"

namespace perfect_velodyne
{
	// Shorthand typedefs for point cloud representations
	// typedef pcl::PointXYZINormal pXYZINormal;
	// typedef pcl::PointCloud<pXYZINormal> pcXYZINormal;
	typedef PointXYZIRNormal VPointNormal;
	typedef pcl::PointCloud<VPointNormal> VPointCloudNormal;

	class IndexSelector : public velodyne_rawdata::RawData
	{
		int num_vertical; // vertical num for PCA
		int num_horizontal; // horizontal num for PCA

		/** configuration parameters */
		typedef struct {
			std::string calibrationFile;     ///< calibration file name
			double max_range;                ///< maximum range to publish
			double min_range;                ///< minimum range to publish
			int min_angle;                   ///< minimum angle to publish
			int max_angle;                   ///< maximum angle to publish

			double tmp_min_angle;
			double tmp_max_angle;
		} Config;
		Config config_;

		/** 
		 * Calibration file
		 */
		velodyne_pointcloud::Calibration calibration_;
		float sin_rot_table_[velodyne_rawdata::ROTATION_MAX_UNITS];
		float cos_rot_table_[velodyne_rawdata::ROTATION_MAX_UNITS];

		/** add private function to handle the VLP16 **/ 
		void unpack_vlp16(const velodyne_msgs::VelodynePacket &pkt, VPointCloudNormal &pc);

		/** in-line test whether a point is in range */
		bool pointInRange(float range)
		{
			// return (range >= config_.min_range
			//         && range <= config_.max_range);
			return true;
		}

		// principal component analysis
		void pca();

		public:
		NormalEstimator();
		void unpack(const velodyne_msgs::VelodynePacket &pkt, VPointCloudNormal &pc);

	};

} // namespace perfect_velodyne

#endif

