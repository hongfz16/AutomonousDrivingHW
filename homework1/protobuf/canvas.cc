// Copyright @2018 Pony AI Inc. All rights reserved.

#include "homework1/protobuf/canvas.h"

#include <iostream>
#include <cmath>
#include <glog/logging.h>

namespace homework1 {

using homework1::geometry::Point3D;

void Canvas::Draw() const {
  for (const auto& p : polygon_.point()) {
    std::cout << "Point:" << p.x() << ", " << p.y() << ", " << p.z() << std::endl;
  }
}

void Canvas::AddPoint(double x, double y, double z) {
  Point3D point;
  point.set_x(x);
  point.set_y(y);
  point.set_z(z);
  AddPoint(point);
}

void Canvas::AddPoint(const Point3D& p) {
  auto* point = polygon_.add_point();
  point->CopyFrom(p);
}

const Point3D& Canvas::GetPoint(int index) const {
  return polygon_.point(index);
}

void Canvas::ParseFromString(const std::string& serialzation) {
  polygon_.ParseFromString(serialzation);
}

const std::string Canvas::SerializeToString() const {
  std::string serialzation;
  CHECK(polygon_.SerializeToString(&serialzation)) << "Canvas serialization failed.";
  return serialzation;
}

void Canvas::AddPointToPolyline(double x,double y,double z)
{
  Point3D point;
  point.set_x(x);
  point.set_y(y);
  point.set_z(z);
  auto* po=polyline_.add_point();
  po->CopyFrom(point);
}

double Canvas::LengthOfPolyline()
{
  homework1::geometry::Polyline& pl=polyline_;
  int psize=pl.point_size();
  if(psize==0 || psize==1)
    return 0.0;
  double totalLength=0;
  for(int i=0;i<psize-1;++i)
  {
    Point3D lp=pl.point(i+1);
    Point3D rp=pl.point(i);
    totalLength+=sqrt(pow(lp.x()-rp.x(),2)+pow(lp.y()-rp.y(),2)+pow(lp.z()-rp.z(),2));
  }
  return totalLength;
}

}  // namespace homework1

