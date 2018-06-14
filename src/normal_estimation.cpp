
//
// src: normal_estimation.cpp
//
// last update: '18.xx.xx
// author: matchey
//
// memo:
//

#include <pcl/common/pca.h>
#include "perfect_velodyne/normal_estimation.h"

using namespace std;

namespace perfect_velodyne
{

	NormalEstimator::NormalEstimator(ros::NodeHandle node, ros::NodeHandle private_nh)
		: Convert(node, private_nh),
		  data_(new perfect_velodyne::RawDataWithNormal())
	{
		// ros::param::param<double>("min_range", min_range, 130.0);
		// ros::param::param<double>("max_range", max_range, 0.9);
		ros::param::param<int>("VN", num_vertical, 1);
		ros::param::param<int>("HN", num_horizontal, 2);

		data_->setup(private_nh);

		// advertise output point cloud (before subscribing to input data)
		output_ =
			node.advertise<sensor_msgs::PointCloud2>("perfect_velodyne/normal", 10);

		srv_ = boost::make_shared <dynamic_reconfigure::Server<velodyne_pointcloud::
			CloudNodeConfig> > (private_nh);
		dynamic_reconfigure::Server<velodyne_pointcloud::CloudNodeConfig>::
			CallbackType f;
		f = boost::bind (&NormalEstimator::callback, this, _1, _2);
		srv_->setCallback (f);

		// subscribe to VelodyneScan packets
		velodyne_scan_ =
			node.subscribe("velodyne_packets", 10,
					&NormalEstimator::processScan, (NormalEstimator *) this,
					ros::TransportHints().tcpNoDelay(true));
	}

	// private
	void NormalEstimator::callback(velodyne_pointcloud::CloudNodeConfig &config,
			uint32_t level)
	{
		ROS_INFO("Reconfigure Request");
		data_->setParameters(config.min_range, config.max_range, config.view_direction,
				config.view_width);
	}

	/** @brief Callback for raw scan messages. */
	void NormalEstimator::processScan(const velodyne_msgs::VelodyneScan::ConstPtr &scanMsg)
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
		normalSetter(outMsg);

		// publish the accumulated cloud message
		ROS_DEBUG_STREAM("Publishing " << outMsg->height * outMsg->width
				<< " Velodyne points, time: " << outMsg->header.stamp);
		output_.publish(outMsg);
	}

	void NormalEstimator::normalSetter(const perfect_velodyne::VPointCloudNormal::Ptr& pc)
	{
	}

} // namespace perfect_velodyne

