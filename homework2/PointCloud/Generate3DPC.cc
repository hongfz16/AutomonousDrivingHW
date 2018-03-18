#include "Generate3DPC.h"

void Generate3DPC(Mat& pc,Mat& pc2)
{
  viz::Viz3d window("mywindow");
  window.showWidget("Coordinate Widget", viz::WCoordinateSystem());
  
  
  Vec3f cam_position(0.0f, 0.0f, 0.0f), cam_focal_point(3.f, 3.f, 3.0f), cam_y_direc(0.0f,0.0f,0.0f);
  Affine3f cam_pose = viz::makeCameraPose(cam_position, cam_focal_point, cam_y_direc);
  Affine3f transform = viz::makeTransformToGlobal(Vec3f(0.0f,1.0f, 0.0f), Vec3f(1.0f, 0.0f, 0.0f), Vec3f(0.0f, 0.0f, 1.0f), cam_position);
  
  viz::WCloud point_cloud(pc,viz::Color::red());
  viz::WCloud point_cloud2(pc2,viz::Color::green());
  
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
