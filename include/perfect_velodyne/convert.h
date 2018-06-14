
//
// include: convert.h
//
// last update: '18.xx.xx
// author: matchey
//
// memo:
//

#ifndef PERFECT_VELODYNE_CONVERT_H
#define PERFECT_VELODYNE_CONVERT_H

#include <velodyne_pointcloud/convert.h>
#include "perfect_velodyne/point_types.h"
#include "perfect_velodyne/rawdata.h"

namespace perfect_velodyne
{
	class ConvertWithNormal : public velodyne_pointcloud::Convert
	{
		void callback(velodyne_pointcloud::CloudNodeConfig &config,
				uint32_t level);
		void processScan(const velodyne_msgs::VelodyneScan::ConstPtr &scanMsg);

		///Pointer to dynamic reconfigure service srv_
		boost::shared_ptr<dynamic_reconfigure::Server<velodyne_pointcloud::
			CloudNodeConfig> > srv_;

		boost::shared_ptr<perfect_velodyne::RawDataWithNormal> data_;
		ros::Subscriber velodyne_scan_;
		ros::Publisher output_;

		/// configuration parameters
		typedef struct {
			int npackets;                    ///< number of packets to combine
		} Config;
		Config config_;

		public:
		ConvertWithNormal(ros::NodeHandle node, ros::NodeHandle private_nh);

	};

} // namespace perfect_velodyne

#endif

