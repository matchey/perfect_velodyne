
//
// include: rawdata.h
//
// last update: '18.xx.xx
// author: matchey
//
// memo:
//

#ifndef PERFECT_VELODYNE_RAWDATA_H
#define PERFECT_VELODYNE_RAWDATA_H

#include <velodyne_pointcloud/rawdata.h>
#include "perfect_velodyne/point_types.h"

namespace perfect_velodyne
{
	// Shorthand typedefs for point cloud representations
	// typedef pcl::PointXYZINormal pXYZINormal;
	// typedef pcl::PointCloud<pXYZINormal> pcXYZINormal;
	typedef PointXYZIRNormal VPointNormal;
	typedef pcl::PointCloud<VPointNormal> VPointCloudNormal;

	class RawDataWithNormal : public velodyne_rawdata::RawData
	{
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
			return (range >= config_.min_range
			        && range <= config_.max_range);
			// return true;
		}

		public:
		RawDataWithNormal();
		void unpack(const velodyne_msgs::VelodynePacket &pkt, VPointCloudNormal &pc);

	};

} // namespace perfect_velodyne

#endif

