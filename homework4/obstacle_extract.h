#ifndef _OBSTACEL_EXTRACT_H_
#define _OBSTACEL_EXTRACT_H_

#include "common/proto/object_labeling_3d.pb.h"
#include "common/proto/perception.pb.h"
#include "homework2/PointCloud/pointcloud.h"

#include <iostream>
#include <vector>
#include <limits>
#include <cmath>

void SingleFrameExtract(PointCloud& pointcloud, 
						interface::object_labeling::ObjectLabels& objectlabels, 
						interface::perception::PerceptionObstacles& obstacles);

#endif //_OBSTACEL_EXTRACT_H_