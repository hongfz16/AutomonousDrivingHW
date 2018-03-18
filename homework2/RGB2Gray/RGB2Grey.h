#ifndef _RGB2GREY_H_
#define _RGB2GREY_H_

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

using namespace std;
using namespace cv;

void RGB2Grey(const Mat& src,Mat& re);

#endif //_RGB2GREY_H_