#include "LIDAR2World.h"

void LIDAR2World(Eigen::Vector3d& orig, Eigen::Vector3d& target, Eigen::Matrix3d& rotation, Eigen::Vector3d& transition)
{
	target=rotation*orig+transition;
}