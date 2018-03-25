#ifndef _LANEDETECT_H_
#define _LANEDETECT_H_

#include <iostream>
#include <cmath>
#include <algorithm>

#include <opencv2/highgui.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>

using namespace std;
using namespace cv;

void LaneDetect(const Mat& srcimg,Mat& _result);

#endif //_LANEDETECT_H_