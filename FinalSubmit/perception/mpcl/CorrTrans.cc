#include "CorrTrans.h"

void FromLocalToWorld(Eigen::Vector3d& point, PointCloud& pc)
{
    point=pc.rotation*point+pc.translation;
}