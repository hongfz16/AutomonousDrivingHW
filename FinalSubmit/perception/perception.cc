// Copyright @2018 Pony AI Inc. All rights reserved.

#include "perception/perception.h"

#include <iostream>
#include <string>
#include <cmath>
#include <vector>
#include <algorithm>

#include "perception/mpcl/sepGround.h"
#include "perception/mpcl/CorrTrans.h"
#include "perception/mpcl/GridUtils.h"
#include "perception/FeatureExtract/FeatureExtract.h"
#include "perception/SVM/svm_classifier.h"

using namespace std;

const int n=100;
const double height_th=0.15;
const double range=30;
MODEL* model = NULL;

string path_prefix("/home/hongfz/Documents/Learn/AutomonousDrivingHW");

int gcount=0;

interface::perception::PerceptionObstacles Perception::RunPerception(
    const PointCloud& pointcloud, const utils::Optional<cv::Mat>& image) {
  if(model==NULL)
  {
    string str_file;
    str_file=path_prefix+"/perception/SVM/model";
    char* modelfile=new char[str_file.length()+1];
    strcpy(modelfile,str_file.c_str());
    model=read_model(modelfile);
  }

  interface::perception::PerceptionObstacles perception_result;

  // if(gcount++!=5)
  //   return perception_result;

  PointCloud mpc(pointcloud);
  remove_ground_points(mpc);
  vector<vector<double> > max_height;
  vector<vector<double> > min_height;
  vector<vector<int> > mark;
  vector<vector<bool> > visited;
  vector<vector<vector<Eigen::Vector3d> > > grid_points;
  vector<vector<pair<int,int> > > blocks;
  vector<ObstaclePos> positions;

  max_height.resize(n);
  min_height.resize(n);
  mark.resize(n);
  grid_points.resize(n);
  for(int i=0;i<n;++i)
  {
    max_height[i].resize(n,-1e8);
    min_height[i].resize(n,1e8);
    mark[i].resize(n,0);
    grid_points[i].resize(n);
  }

  vector<Eigen::Vector3d>& points=mpc.points;
  Eigen::Vector3d lidar_point(0,0,0);
  FromLocalToWorld(lidar_point,mpc);
  for(int i=0;i<points.size();++i)
  {
    FromLocalToWorld(points[i],mpc);
    points[i]=points[i]-lidar_point;
  }

  FillGrid(grid_points,points,range);
  RemoveNoisePoints(grid_points);
  MarkGrid(grid_points,max_height,min_height,mark,height_th);
  FindConnectedBlocks(mark,blocks);
  FindObstacles(blocks,grid_points,max_height,min_height,positions);

  for(int i=0;i<positions.size();++i)
  {
    auto* obstacle=perception_result.add_obstacle();
    double z_min=positions[i].z_min;
    for(int j=0;j<positions[i].MBR.size();++j)
    {
      auto* polygon_point=obstacle->add_polygon_point();
      polygon_point->set_x(positions[i].MBR[j].first+lidar_point(0));
      polygon_point->set_y(positions[i].MBR[j].second+lidar_point(1));
      polygon_point->set_z(z_min+lidar_point(2));
    }
    obstacle->set_height(positions[i].height);
    string feature_str=ExtractFeature(positions[i].points);
    char* feature_cstr=new char[feature_str.length()+1];
    strcpy(feature_cstr, feature_str.c_str());
    if(SVM_Classifier(feature_cstr,model)==1)
    {
      obstacle->set_id(string("car_")+to_string(i));
      obstacle->set_type(interface::perception::ObjectType::CAR);
    }
    else
    {
      if(positions[i].height<2)
      {
        double linesize=0;
        for(int j=0;j<positions[i].MBR.size();++j)
        {
          pair<double,double> cu,ne;
          int next=(j+1)%positions[i].MBR.size();
          cu=positions[i].MBR[j];
          ne=positions[i].MBR[next];
          linesize+=sqrt(pow(cu.first-ne.first,2)+pow(cu.second-ne.second,2));
        }
        linesize/=positions[i].MBR.size();
        if(linesize<0.5)
        {
          obstacle->set_id(string("pedestrain_")+to_string(i));
          obstacle->set_type(interface::perception::ObjectType::PEDESTRIAN);
        }
        else
        {
          obstacle->set_id(string("unknown_")+to_string(i));
          obstacle->set_type(interface::perception::ObjectType::UNKNOWN_TYPE);
        }
      }
    }
    delete [] feature_cstr;
  }

  cout<<gcount++<<endl;

  return perception_result;
}