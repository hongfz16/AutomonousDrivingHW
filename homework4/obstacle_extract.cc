#include "obstacle_extract.h"

using namespace std;

void SingleFrameExtract(PointCloud& pointcloud, 
						interface::object_labeling::ObjectLabels& objectlabels, 
						interface::perception::PerceptionObstacles& obstacles)
{
	vector<Eigen::Vector3d> points_in_world;
	for(const auto& point : pointcloud.points)
	{
		Eigen::Vector3d point_in_world=pointcloud.rotation*point+pointcloud.translation;
		points_in_world.push_back(point_in_world);
	}	
	for(const auto& object : objectlabels.object())
	{
		double minx=std::numeric_limits<double>::infinity();
		double maxx=-std::numeric_limits<double>::infinity();
		double miny=std::numeric_limits<double>::infinity();
		double maxy=-std::numeric_limits<double>::infinity();
		double minz=std::numeric_limits<double>::infinity();
		for(const auto& point : object.polygon().point())
		{
			minx=min(minx,point.x());
			maxx=max(maxx,point.x());
			miny=min(miny,point.y());
			maxy=max(maxy,point.y());
			minz=min(minz,point.z());
		}

		auto* pobs=obstacles.add_obstacle();
		interface::perception::PerceptionObstacle obs;
		obs.set_id(object.id());
		obs.set_heading(object.heading());
		obs.set_height(object.height());
		obs.set_type(object.type());
		for(const auto& point : object.polygon().point())
		{
			auto* ppolygon_point=obs.add_polygon_point();
			ppolygon_point->CopyFrom(point);
		}

		for(const auto& testpoint : points_in_world)
		{
			if(testpoint[0]<minx || testpoint[0]>maxx || testpoint[1]<miny || testpoint[1]>maxy)
				continue;
			if(testpoint[2]<minz || testpoint[2]>minz+object.height())
				continue;
			int nCross=0;
			int psize=object.polygon().point_size();
			for(int i=0;i<psize;++i)
			{
				interface::geometry::Point3D p1,p2;
				p1=object.polygon().point(i);
				p2=object.polygon().point((i+1)%psize);
				if(p1.y()==p2.y())
					continue;
				if(testpoint[1]<min(p1.y(),p2.y()))
					continue;
				if(testpoint[1]>max(p1.y(),p2.y()))
					continue;
				double xx=(testpoint[1]-p1.y())*(p2.x()-p1.x())/(p2.y()-p1.y())+p1.x();
				if(xx>testpoint[0])
					nCross++;
			}
			if(nCross%2==1)
			{
				auto* npoint=obs.add_object_points();
				interface::geometry::Point3D ntestpoint;
				ntestpoint.set_x(testpoint[0]);
				ntestpoint.set_y(testpoint[1]);
				ntestpoint.set_z(testpoint[2]);
				npoint->CopyFrom(ntestpoint);
			}
		}
		pobs->CopyFrom(obs);
		//cout<<obs.object_points_size()<<endl;
	}
}