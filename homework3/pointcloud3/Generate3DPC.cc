#include "Generate3DPC.h"

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

void Generate3DPC(Mat& pc)
{
  viz::Viz3d window("mywindow");
  window.showWidget("Coordinate Widget", viz::WCoordinateSystem());
  
  
  Vec3f cam_position(0.0f, 0.0f, 0.0f), cam_focal_point(3.f, 3.f, 3.0f), cam_y_direc(0.0f,0.0f,0.0f);
  Affine3f cam_pose = viz::makeCameraPose(cam_position, cam_focal_point, cam_y_direc);
  Affine3f transform = viz::makeTransformToGlobal(Vec3f(0.0f,1.0f, 0.0f), Vec3f(1.0f, 0.0f, 0.0f), Vec3f(0.0f, 0.0f, 1.0f), cam_position);
  
  viz::WCloud point_cloud(pc,viz::Color::green());
  
  double z = CV_PI*0.01f;
  Affine3f cloud_pose_global;
  while(!window.wasStopped())
  {
    cloud_pose_global = transform.inv()*Affine3f(Vec3f(0.0, 0.0, z), Vec3f(0.0, 0.0, 2.0))*Affine3f::Identity();
    window.showWidget("Point Cloud", point_cloud, cloud_pose_global);
    
    viz::WCameraPosition camera(0.5);
    viz::WCameraPosition camera_frustum(Matx33f(3.1,0,0.1,0,3.2,0.2,0,0,1));
    window.spinOnce(1, true);
  }
}

void Generate3DPCSepGround(Mat& ground,Mat& nonground)
{
  viz::Viz3d window("mywindow");
  window.showWidget("Coordinate Widget", viz::WCoordinateSystem());
  
  
  Vec3f cam_position(0.0f, 0.0f, 0.0f), cam_focal_point(3.f, 3.f, 3.0f), cam_y_direc(0.0f,0.0f,0.0f);
  Affine3f cam_pose = viz::makeCameraPose(cam_position, cam_focal_point, cam_y_direc);
  Affine3f transform = viz::makeTransformToGlobal(Vec3f(0.0f,1.0f, 0.0f), Vec3f(1.0f, 0.0f, 0.0f), Vec3f(0.0f, 0.0f, 1.0f), cam_position);
  
  viz::WCloud point_cloud(ground,viz::Color::red());
  viz::WCloud point_cloud2(nonground,viz::Color::green());
  
  double z = CV_PI*0.01f;
  Affine3f cloud_pose_global;
  while(!window.wasStopped())
  {
    cloud_pose_global = transform.inv()*Affine3f(Vec3f(0.0, 0.0, z), Vec3f(0.0, 0.0, 2.0))*Affine3f::Identity();
    window.showWidget("Point Cloud", point_cloud, cloud_pose_global);
    window.showWidget("nonground Cloud", point_cloud2, cloud_pose_global);
    
    viz::WCameraPosition camera(0.5);
    viz::WCameraPosition camera_frustum(Matx33f(3.1,0,0.1,0,3.2,0.2,0,0,1));
    window.spinOnce(1, true);
  }
}
