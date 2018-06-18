
//
// src: convert.cpp
//
// last update: '18.xx.xx
// author: matchey
//
// memo:
//

#include "perfect_velodyne/convert.h"
#include "perfect_velodyne/normal_estimation.h"

#include <pcl_conversions/pcl_conversions.h>

namespace perfect_velodyne
{

	ConvertWithNormal::ConvertWithNormal(ros::NodeHandle node, ros::NodeHandle private_nh)
		: data_(new perfect_velodyne::RawDataWithNormal())
	{
		data_->setup(private_nh);

		// advertise output point cloud (before subscribing to input data)
		output_ =
			node.advertise<sensor_msgs::PointCloud2>("perfect_velodyne/normal", 10);

		srv_ = boost::make_shared <dynamic_reconfigure::Server<perfect_velodyne::
			CloudNodeConfig> > (private_nh);
		dynamic_reconfigure::Server<perfect_velodyne::CloudNodeConfig>::
			CallbackType f;
		f = boost::bind (&ConvertWithNormal::callback, this, _1, _2);
		srv_->setCallback (f);

		// subscribe to VelodyneScan packets
		velodyne_scan_ =
			node.subscribe("velodyne_packets", 10,
					&ConvertWithNormal::processScan, (ConvertWithNormal *) this,
					ros::TransportHints().tcpNoDelay(true));
	}

	// private
	void ConvertWithNormal::callback(perfect_velodyne::CloudNodeConfig &config,
			uint32_t level)
	{
		ROS_INFO("Reconfigure Request");
		data_->setParameters(config.min_range, config.max_range, config.view_direction,
				config.view_width);
	}

	/** @brief Callback for raw scan messages. */
	void ConvertWithNormal::processScan(const velodyne_msgs::VelodyneScan::ConstPtr &scanMsg)
	{
		if (output_.getNumSubscribers() == 0)         // no one listening?
			return;                                     // avoid much work

		// allocate a point cloud with same time and frame ID as raw data
		perfect_velodyne::VPointCloudNormal::Ptr
			outMsg(new perfect_velodyne::VPointCloudNormal());
		// outMsg's header is a pcl::PCLHeader, convert it before stamp assignment
		outMsg->header.stamp = pcl_conversions::toPCL(scanMsg->header).stamp;
		outMsg->header.frame_id = scanMsg->header.frame_id;
		outMsg->height = 1;

		// process each packet provided by the driver
		for (size_t i = 0; i < scanMsg->packets.size(); ++i)
		{
			data_->unpack(scanMsg->packets[i], *outMsg);
		}

		//add normal to points
		perfect_velodyne::NormalEstimator ne;
		ne.normalSetter(outMsg);

		// publish the accumulated cloud message
		ROS_DEBUG_STREAM("Publishing " << outMsg->height * outMsg->width
				<< " Velodyne points, time: " << outMsg->header.stamp);
		output_.publish(outMsg);
	}

} // namespace perfect_velodyne

