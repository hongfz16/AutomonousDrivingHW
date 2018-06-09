#ifndef _FINDROUTE_H_
#define _FINDROUTE_H_

#include<iostream>
#include<string>
#include<vector>
#include<stack>
#include<queue>
#include<cmath>
#include"common/proto/route.pb.h"
#include"common/proto/map.pb.h"
#include"common/utils/file/file.h"
#include"common/proto/geometry.pb.h"
#include"common/proto/transform.pb.h"

#include "glog/logging.h"

using namespace std;

namespace hongfz16
{

int GetId(const string& strid)
{
	string sub=strid.substr(1,strid.size());
	return stoi(sub);
}

//Decide which lane the starting and ending point in
int FindLaneId(const interface::geometry::Point2D& testpoint, const interface::map::Map& mapdata)
{
	///cout<<testpoint.x()<<" "<<testpoint.y()<<endl;
	for(const auto& lane : mapdata.lane())
	{
		interface::geometry::Point3D l0,l1,r0,r1;
		int lsize=lane.left_bound().boundary().point_size();
		int rsize=lane.right_bound().boundary().point_size();
		l0=lane.left_bound().boundary().point(0);
		l1=lane.left_bound().boundary().point(lsize-1);
		r0=lane.right_bound().boundary().point(0);
		r1=lane.right_bound().boundary().point(rsize-1);

		//use simple square box to decide
		double xmin=min(min(l0.x(),l1.x()),min(r0.x(),r1.x()));
		double ymin=min(min(l0.y(),l1.y()),min(r0.y(),r1.y()));
		double xmax=max(max(l0.x(),l1.x()),max(r0.x(),r1.x()));
		double ymax=max(max(l0.y(),l1.y()),max(r0.y(),r1.y()));
		if(testpoint.x()>=xmin && testpoint.x()<=xmax && testpoint.y()>=ymin && testpoint.y()<=ymax)
			return GetId(lane.id().id());
	}
	return -1;
}

//use id(int) to find the corresponding lane
interface::map::Lane FindCorrectLane(int id,const interface::map::Map& map)
{
	for(const auto& lane : map.lane())
	{
		if(GetId(lane.id().id())==id)
			return lane;
	}
}

double CalcDistPoint2d(interface::geometry::Point2D& lp,interface::geometry::Point2D& rp)
{
	return std::sqrt((lp.x()-rp.x())*(lp.x()-rp.x())+(lp.y()-rp.y())*(lp.y()-rp.y()));
}

//inputfilename is the proto file containing starting and ending point
void FindRoute(const string& mapfilename, const interface::geometry::Vector3d& start3d, const interface::geometry::Point3D& ene3d, interface::route::Route& route)
{
	//Read all the information
	interface::map::Map mapdata;

	CHECK(file::ReadFileToProto(mapfilename, &mapdata));
	interface::geometry::Point2D start_point,end_point;
	start_point.set_x(start3d.x());
	start_point.set_y(start3d.y());
	end_point.set_x(ene3d.x());
	end_point.set_y(ene3d.y());
	
	int start_id=FindLaneId(start_point,mapdata);
	int end_id=FindLaneId(end_point,mapdata);
	if(start_id==-1 || end_id==-1)
	{
		cout<<"Point could not been find on the map"<<endl;
		return;
	}
	//cout<<start_id<<" "<<end_id<<endl;
	interface::map::Lane start_lane=FindCorrectLane(start_id,mapdata);
	interface::map::Lane end_lane=FindCorrectLane(end_id,mapdata);
	int lanesize=mapdata.lane_size();
	vector<int> record;
	record.resize(lanesize+1,-1);
	queue<interface::map::Lane> ss;
	ss.push(start_lane);
	record[start_id]=0;
	bool found=false;

	//BFS
	int cnt=0;
	while(!ss.empty())
	{
		cout<<++cnt<<endl;
		//cout<<"inside while"<<endl;
		interface::map::Lane frontlane=ss.front();
		ss.pop();
		vector<int> next;
		for(const auto& id : frontlane.predecessor())
		{
		//	cout<<id.id()<<endl;
			next.push_back(GetId(id.id()));
		}
		for(const int& id : next)
		{
		//	cout<<id<<endl;
			if(record[id]!=-1)
				continue;
			const interface::map::Lane templane=FindCorrectLane(id,mapdata);
			ss.push(templane);
			record[id]=GetId(frontlane.id().id());
			if(id==end_id)
			{
				found=true;
				break;
			}
		}
		if(found)
			break;
	}

	//Reconstruct the whole route
	vector<int> routeid;
	routeid.push_back(end_id);
	while(1)
	{
		int last=routeid[routeid.size()-1];
		if(record[last]==0)
			break;
		routeid.push_back(record[last]);
	}

	//Add route points(lanes' central line points) to route

	interface::geometry::Point2D last_add_route_point;
	bool updated=false;
	for(int i=routeid.size()-1;i>=0;--i)
	{
		interface::map::Lane templane=FindCorrectLane(routeid[i],mapdata);
		int compflag=0;
		if(i==routeid.size()-1)
		{
			interface::geometry::Point3D flagpoint=start_lane.central_line().point(0);
			if(abs(flagpoint.x()-start_point.x())<abs(flagpoint.y()-start_point.y()))
			{
				if(flagpoint.y()<start_point.y())
					compflag=2;
				else
					compflag=-2;
			}
			else
			{
				if(flagpoint.x()<start_point.x())
					compflag=1;
				else
					compflag=-1;
			}
		}
		if(i==0)
		{
			int tempsize=end_lane.central_line().point_size();
			interface::geometry::Point3D flagpoint=end_lane.central_line().point(tempsize-1);
			if(abs(flagpoint.x()-end_point.x())<abs(flagpoint.y()-end_point.y()))
			{
				if(flagpoint.y()<end_point.y())
					compflag=2;
				else
					compflag=-2;
			}
			else
			{
				if(flagpoint.x()<end_point.x())
					compflag=1;
				else
					compflag=-1;
			}
		}
		// interface::geometry::Point2D nsp;
		// nsp.set_x(start_point.x());
		// nsp.set_y(start_point.y());
		// auto* newstartpoint=route.add_route_point();
		// newstartpoint->CopyFrom(nsp);
		for(const auto& point : templane.central_line().point())
		{
			if(i==routeid.size()-1)
			{
				if(compflag==1)
					if(point.x()<start_point.x())
						continue;
				if(compflag==-1)
					if(point.x()>start_point.x())
						continue;
				if(compflag==2)
					if(point.y()<start_point.y())
						continue;
				if(compflag==-2)
					if(point.y()>start_point.y())
						continue;
			}
			if(i==0)
			{
				if(compflag==1)
					if(point.x()<end_point.x())
						continue;
				if(compflag==-1)
					if(point.x()>end_point.x())
						continue;
				if(compflag==2)
					if(point.y()<end_point.y())
						continue;
				if(compflag==-2)
					if(point.y()>end_point.y())
						continue;
			}
			interface::geometry::Point2D p2d;
			p2d.set_x(point.x());
			p2d.set_y(point.y());
			if(updated)
			{
				if(CalcDistPoint2d(last_add_route_point,p2d)<2)
					continue;
			}
			last_add_route_point.CopyFrom(p2d);
			updated=true;
			auto* newroutepoint=route.add_route_point();
			newroutepoint->CopyFrom(p2d);
		}
		// nsp.set_x(end_point.x());
		// nsp.set_y(end_point.y());
		// auto* newendpoint=route.add_route_point();
		// newendpoint->CopyFrom(nsp);
		//cout<<routeid[i]<<endl;
	}
}

}

#endif //_FINDROUTE_H_