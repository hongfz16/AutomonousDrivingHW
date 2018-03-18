#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>

#include "RGB2Grey.h"

using namespace std;  
using namespace cv;

int main()
{
  Mat srcImage = imread("/home/hongfz/Documents/Learn/AutomonousDrivingHW/pony_data/GigECameraDeviceWideAngle/0.jpg");  

  //Convert RGB To Grey
  //imshow("srcIMage",srcImage);  
  Mat grey;
  RGB2Grey(srcImage,grey);
  imshow("grey",grey);
  imwrite("/home/hongfz/Documents/Learn/AutomonousDrivingHW/homework2/RGB2Gray/Converted/0.jpg",grey);

  waitKey(0);
  return 0;
}
