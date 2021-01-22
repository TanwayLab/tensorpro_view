#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

struct Tanway_TP_Point
{
  PCL_ADD_POINT4D;
  int ring;
  float intensity;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;


POINT_CLOUD_REGISTER_POINT_STRUCT(Tanway_TP_Point,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (int, ring, ring)
                                  (float, intensity, intensity)                                 )
