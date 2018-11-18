
#ifndef _PERFECT_VELODYNE_OBSTACLES_PUBLISHER_H
#define _PERFECT_VELODYNE_OBSTACLES_PUBLISHER_H

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h> // usable pcl for rosmsgs

namespace perfect_velodyne
{
	using PointT = pcl::PointXYZINormal;
	using PointCloud = pcl::PointCloud<PointT>;

	class ObstaclesPublisher
	{
		public:
		ObstaclesPublisher(ros::NodeHandle, ros::NodeHandle);
		~ObstaclesPublisher();

		private:
		void processData(const PointCloud::ConstPtr&);

		void constructObstacleClouds(const PointCloud::ConstPtr&,
									 unsigned, size_t&, size_t&);


		// rosparam
		double normal_z_threshold;

		// Point clouds generated in processData
		PointCloud obstacle_cloud;
		PointCloud clear_cloud;

		// ROS topics
		ros::Subscriber velodyne_scan;
		ros::Publisher obstacle_publisher;
		ros::Publisher clear_publisher;
	};
}

#endif

