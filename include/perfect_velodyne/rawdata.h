
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

#include <errno.h>
#include <stdint.h>
#include <string>
#include <boost/format.hpp>
#include <math.h>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <velodyne_msgs/VelodyneScan.h>
#include <velodyne_pointcloud/calibration.h>
#include "perfect_velodyne/point_types.h"

namespace perfect_velodyne
{
	// Shorthand typedefs for point cloud representations
	// typedef pcl::PointXYZINormal pXYZINormal;
	// typedef pcl::PointCloud<pXYZINormal> pcXYZINormal;
	typedef PointXYZIRNormal VPointNormal;
	typedef pcl::PointCloud<VPointNormal> VPointCloudNormal;

	static const int SIZE_BLOCK = 100;
	static const int RAW_SCAN_SIZE = 3;
	static const int SCANS_PER_BLOCK = 32;
	static const int BLOCK_DATA_SIZE = (SCANS_PER_BLOCK * RAW_SCAN_SIZE);

	static const float ROTATION_RESOLUTION      =     0.01f;  // [deg]
	static const uint16_t ROTATION_MAX_UNITS    = 36000u;     // [deg/100]
	static const float DISTANCE_RESOLUTION      =     0.002f; // [m]

	/** @todo make this work for both big and little-endian machines */
	static const uint16_t UPPER_BANK = 0xeeff;
	static const uint16_t LOWER_BANK = 0xddff;


	typedef struct raw_block
	{
		uint16_t header;        ///< UPPER_BANK or LOWER_BANK
		uint16_t rotation;      ///< 0-35999, divide by 100 to get degrees
		uint8_t  data[BLOCK_DATA_SIZE];
	} raw_block_t;

	/** used for unpacking the first two data bytes in a block
	 *
	 *  They are packed into the actual data stream misaligned.  I doubt
	 *  this works on big endian machines.
	 */
	union two_bytes
	{
		uint16_t uint;
		uint8_t  bytes[2];
	};

	static const int PACKET_SIZE = 1206;
	static const int BLOCKS_PER_PACKET = 12;
	static const int PACKET_STATUS_SIZE = 4;
	static const int SCANS_PER_PACKET = (SCANS_PER_BLOCK * BLOCKS_PER_PACKET);

	/** \brief Raw Velodyne packet.
	 *
	 *  revolution is described in the device manual as incrementing
	 *    (mod 65536) for each physical turn of the device.  Our device
	 *    seems to alternate between two different values every third
	 *    packet.  One value increases, the other decreases.
	 *
	 *  \todo figure out if revolution is only present for one of the
	 *  two types of status fields
	 *
	 *  status has either a temperature encoding or the microcode level
	 */
	typedef struct raw_packet
	{
		raw_block_t blocks[BLOCKS_PER_PACKET];
		uint16_t revolution;
		uint8_t status[PACKET_STATUS_SIZE]; 
	} raw_packet_t;

	class RawDataWithNormal
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
		float sin_rot_table_[ROTATION_MAX_UNITS];
		float cos_rot_table_[ROTATION_MAX_UNITS];

		/** in-line test whether a point is in range */
		bool pointInRange(float range)
		{
			return (range >= config_.min_range
					&& range <= config_.max_range);
			// return true;
		}

		public:
		RawDataWithNormal();
		~RawDataWithNormal() {}
		void unpack(const velodyne_msgs::VelodynePacket &pkt, VPointCloudNormal &pc);

		int setup(ros::NodeHandle private_nh);

		int setupOffline(std::string calibration_file, double max_range_, double min_range_);

		void setParameters(double min_range, double max_range, double view_direction,
						   double view_width);

	};

} // namespace perfect_velodyne

#endif

