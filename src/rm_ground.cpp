
#include <ros/ros.h>
#include "perfect_velodyne/obstacles_publisher.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "rm_ground_node");

	std::cout << "remove ground points with normal_z" << std::endl;

	ros::NodeHandle node;
	ros::NodeHandle priv_nh("~");

	perfect_velodyne::ObstaclesPublisher op(node, priv_nh);

	ros::spin();

	return 0;
}

