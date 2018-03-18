#ifndef _DRAWHISTOGRAM_H_
#define _DRAWHISTOGRAM_H_

#include <iostream>
#include <cmath>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#include "pointcloud.h"

using namespace std;
using namespace cv;

void DrawHistHight(const vector<Eigen::Vector3d>& points);
void DrawHistDist(const vector<Eigen::Vector3d>& points);

#endif //_DRAWHISTOGRAM_H_