#ifndef _POINT_CLOUD_H_
#define _POINT_CLOUD_H
#define PCL_NO_PRECOMPILE
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

struct  MyPointType
{
  PCL_ADD_POINT4D;
  float intensity;
  float intensity_cov;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
}EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (MyPointType,
                                  (float,x,x)
                                  (float,y,y)
                                  (float,z,z)
                                  (float,intensity,intensity)
                                  (float,intensity_cov,intensity_cov)
)
#endif
