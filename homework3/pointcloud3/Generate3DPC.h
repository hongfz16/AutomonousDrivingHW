#ifndef _GENERATE3DPC_H_
#define _GENERATE3DPC_H_

#include <opencv2/viz.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/opencv.hpp>

#include <Eigen/Core>

using namespace std;
using namespace cv;

void Generate3DPC(Mat& pc);
void Generate3DPCSepGround(Mat& ground,Mat& nonground);
Mat ReadPointCloud(const vector<Eigen::Vector3d>& points);

#endif //_GENERATE3DPC_H_