
#include "perfect_velodyne/obstacles_publisher.h"

namespace perfect_velodyne
{
	ObstaclesPublisher::ObstaclesPublisher(ros::NodeHandle node, ros::NodeHandle priv_nh)
	{
		// max 20 [deg] --> 0.9397
		priv_nh.param("z_threshold", normal_z_threshold, 0.9);

		obstacle_publisher = node.advertise<PointCloud>("velodyne_obstacles", 1);
		clear_publisher = node.advertise<PointCloud>("velodyne_clear", 1);

		velodyne_scan = node.subscribe("perfect_velodyne/normal", 10,
									   &ObstaclesPublisher::processData, this,
									   ros::TransportHints().tcpNoDelay(true));
	}

	ObstaclesPublisher::~ObstaclesPublisher() {}

	//private
	void ObstaclesPublisher::processData(const PointCloud::ConstPtr &scan)
	{
		if ((obstacle_publisher.getNumSubscribers() == 0)
			&& (clear_publisher.getNumSubscribers() == 0))
			return;

		// pass along original time stamp and frame ID
		obstacle_cloud.header.stamp = scan->header.stamp;
		obstacle_cloud.header.frame_id = scan->header.frame_id;

		// pass along original time stamp and frame ID
		clear_cloud.header.stamp = scan->header.stamp;
		clear_cloud.header.frame_id = scan->header.frame_id;

		// set the exact point cloud size -- the vectors should already have
		// enough space
		size_t npoints = scan->points.size();
		obstacle_cloud.points.resize(npoints);
		clear_cloud.points.resize(npoints);

		size_t obs_count = 0;
		size_t empty_count = 0;

		constructObstacleClouds(scan, npoints, obs_count, empty_count);

		obstacle_cloud.points.resize(obs_count);

		clear_cloud.points.resize(empty_count);

		if (obstacle_publisher.getNumSubscribers() > 0)
			obstacle_publisher.publish(obstacle_cloud);

		if (clear_publisher.getNumSubscribers() > 0)
			clear_publisher.publish(clear_cloud);
	}

	void ObstaclesPublisher::constructObstacleClouds(const PointCloud::ConstPtr& scan,
													 unsigned npoints, size_t& obs_count,
													 size_t& empty_count)
	{
		for(unsigned i = 0; i != npoints; ++i){
			if(-normal_z_threshold < scan->points[i].normal_z
								  && scan->points[i].normal_z < normal_z_threshold){
				obstacle_cloud.points[obs_count] = scan->points[i];
				++obs_count;
			}else{
				clear_cloud.points[empty_count] = scan->points[i];
				++empty_count;
			}
		}
	}
}

