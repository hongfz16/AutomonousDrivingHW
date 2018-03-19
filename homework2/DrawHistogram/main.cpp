#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>

#include "DrawHistogram.h"
#include "pointcloud.h"

using namespace std;  
using namespace cv;

int main()
{
  PointCloud pointcloud = ReadPointCloudFromTextFile(
      "/home/hongfz/Documents/Learn/AutomonousDrivingHW/pony_data/VelodyneDevice32c/152.txt");

  // Generate Two Histograms
  DrawHistHight(pointcloud.points);
  DrawHistDist(pointcloud.points);
  waitKey(0);

  return 0;
}
