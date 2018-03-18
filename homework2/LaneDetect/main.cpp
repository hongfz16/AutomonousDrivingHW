#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>

#include "LaneDetect.h"
//#include "LD_github.hpp"

using namespace std;  
using namespace cv;

int main()
{
  string telephoto("GigECameraDeviceTelephoto");
  string wideangle("GigECameraDeviceWideAngle");
  vector<string> paths;
  paths.push_back(telephoto);
  paths.push_back(wideangle);
  namedWindow("Result");
  for(int j=0;j<paths.size();++j)
  {
    //Mat init=imread(string("/home/hongfz/Documents/Learn/AutomonousDrivingHW/pony_data/")+paths[j]+string("/")+to_string(0)+string(".jpg"));
    //LaneDetect detect(init);
    for(int i=0;i<200;++i)
    {
      Mat srcImage = imread(string("/home/hongfz/Documents/Learn/AutomonousDrivingHW/pony_data/")+paths[j]+string("/")+to_string(i)+string(".jpg"));  
      //cvtColor(srcImage,srcImage, CV_BGR2GRAY);
      //detect.nextFrame(srcImage);
      Mat result;
      LaneDetect(srcImage,result);
      //imwrite(string("/home/hongfz/Documents/Learn/AutomonousDrivingHW/pony_data/")+paths[j]+string("_HW2Proc/")+to_string(i)+string(".jpg"),result);
      if(waitKey(10)==27)
        break;
    }
  }
  //waitKey(0);
  return 0;
}
