// Copyright @2018 Pony AI Inc. All rights reserved.
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <iostream>
#include <string>
using namespace std;

using namespace cv;

int main() {
  cv::Mat image;
  // ATTENTION!!! : please use absolute path for reading the data file.
  image = imread("/home/hongfz/Documents/Learn/AutomonousDrivingHW/homework3/chessboard/chessboard_undistorted.png", CV_LOAD_IMAGE_COLOR);
  
  //cout<<image.cols<<" "<<image.rows<<endl;
  int focal=350;
  Mat cameraMatrix=(Mat_<double>(3,3)<<focal,0,400,0,focal,400,0,0,1);
  Mat distortion=(Mat_<double>(4,1)<<0.1,0.1,0,0);

  Mat distorted;

  undistort(image,distorted,cameraMatrix,distortion);
  
  imwrite(string("/home/hongfz/Documents/Learn/AutomonousDrivingHW/homework3/chessboard/")+to_string(focal)+string(".png"),distorted);

  imshow("distorted",distorted);


  //namedWindow("chessboard");
  imshow("chessboard", image);
  waitKey(0);
  return 0;
}
