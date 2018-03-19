#ifndef _PROCPOINTCLOUD_H_
#define _PROCPOINTCLOUD_H_

#include <iostream>
#include <vector>
#include <algorithm>
#include "pointcloud.h"

using namespace std;
using namespace Eigen;

#define CAR_HEIGHT 2

void preprocess(vector<Vector3d>& points,vector<vector<Vector3d> >& sortedpoints);
void SeperateGroundPoints(vector<vector<Vector3d> >& sortedpoints,vector<Vector3d>& ground,vector<Vector3d>& nonground,double alphamax,double missmax,double hmin);

#endif //_PROCPOINTCLOUD_H_