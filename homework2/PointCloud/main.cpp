#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>

#include "Generate3DPC.h"
#include "pointcloud.h"
#include "sepGround.h"

using namespace std;  
using namespace cv;

Mat ReadPointCloud(const vector<Eigen::Vector3d>& points)
{
	int pointnum=points.size();
	Mat cloud(1,pointnum,CV_32FC3);
	Point3f* data=cloud.ptr<cv::Point3f>();
	for(size_t i=0;i<pointnum;++i)
	{
		data[i].x=points[i](0);
		data[i].y=points[i](1);
		data[i].z=points[i](2);
	}
	return cloud;
}

int main()
{
  PointCloud pointcloud = ReadPointCloudFromTextFile(
      "/home/hongfz/Documents/Learn/AutomonousDrivingHW/pony_data/VelodyneDevice32c/152.txt");

  // Generate Point Cloud Picture

  double PI=3.141592653;
  vector<vector<Vector3d> > sortedpoints;
  preprocess(pointcloud.points,sortedpoints);
  vector<Vector3d> ground,nonground;
  SeperateGroundPoints(sortedpoints,ground,nonground,PI/4,0.1,0.1);

  Mat groundmat=ReadPointCloud(ground);
  Mat nongroundmat=ReadPointCloud(nonground);
  Generate3DPC(groundmat,nongroundmat);

  return 0;
}
