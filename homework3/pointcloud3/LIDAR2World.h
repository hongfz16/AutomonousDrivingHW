#ifndef _LIDAR2WORLD_H_
#define _LIDAR2WORLD_H_

#include "pointcloud.h"

void LIDAR2World(Eigen::Vector3d& orig, Eigen::Vector3d& target, Eigen::Matrix3d& rotation, Eigen::Vector3d& transition);


#endif //_LIDAR2WORLD_H_
