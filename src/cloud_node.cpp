/*
 * author: matchey
*/

/** \file

    This ROS node converts raw Velodyne LIDAR packets to PointCloud2 with normal.

*/

#include <ros/ros.h>
#include "perfect_velodyne/convert.h"

/** Main node entry point. */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "normal_estimation_node");
  ros::NodeHandle node;
  ros::NodeHandle priv_nh("~");

  // create conversion class, which subscribes to raw data
  perfect_velodyne::ConvertWithNormal conv(node, priv_nh);

  // handle callbacks until shut down
  ros::spin();

  return 0;
}
