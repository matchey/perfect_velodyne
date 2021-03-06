
#ifndef __PERFECT_VELODYNE_POINT_TYPES_H
#define __PERFECT_VELODYNE_POINT_TYPES_H

#include <pcl/point_types.h>

namespace perfect_velodyne
{
  // struct PointXYZIR
  // {
  //   PCL_ADD_POINT4D;                    // quad-word XYZ
  //   float    intensity;                 ///< laser intensity reading
  //   uint16_t ring;                      ///< laser ring number
  //   EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // ensure proper alignment
  // } EIGEN_ALIGN16;

  struct PointXYZIRNormal
  {
    PCL_ADD_POINT4D;                    // quad-word XYZ
	PCL_ADD_NORMAL4D;
    float    intensity;                 ///< laser intensity reading
	float   curvature;
    uint16_t ring;                      ///< laser ring number
    float range;                         ///< whether a point is in range
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // ensure proper alignment
  } EIGEN_ALIGN16;

}; // namespace perfect_velodyne

// POINT_CLOUD_REGISTER_POINT_STRUCT(perfect_velodyne::PointXYZIR,
//                                   (float, x, x)
//                                   (float, y, y)
//                                   (float, z, z)
//                                   (float, intensity, intensity)
//                                   (uint16_t, ring, ring))

POINT_CLOUD_REGISTER_POINT_STRUCT(perfect_velodyne::PointXYZIRNormal,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (float, normal_x, normal_x)
                                  (float, normal_y, normal_y)
                                  (float, normal_z, normal_z)
                                  (float, curvature, curvature)
                                  (uint16_t, ring, ring)
                                  (float, range, range))

#endif // __PERFECT_VELODYNE_POINT_TYPES_H

